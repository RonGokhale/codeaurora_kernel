/*
 * Copyright 2013 NetApp, Inc. All Rights Reserved, contribution by
 * Morgan Mears.
 *
 * Copyright 2013 Red Hat, Inc.
 *
 * This file is released under the GPL.
 */

#include "dm-cache-policy.h"
#include "dm-cache-policy-internal.h"
#include "dm-cache-shim-utils.h"
#include "dm.h"

#include <linux/hash.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <linux/delay.h>

/*----------------------------------------------------------------*/

#define DM_MSG_PREFIX "cache-policy-era"

typedef uint32_t era_t;

#define ERA_OVERFLOW (~((uint32_t) 0))

struct era_policy {
	struct dm_cache_policy policy;

	struct mutex lock;

	dm_cblock_t cache_size;

	era_t *eras;
	atomic64_t current_era;
};

/*----------------------------------------------------------------*/

static struct era_policy *to_era_policy(struct dm_cache_policy *p)
{
	return container_of(p, struct era_policy, policy);
}

typedef int (*era_match_fn_t)(era_t, era_t);

/*
 * If the era counter value provided by the user matches the current
 * counter value while under lock, increment the counter (intention is to
 * prevent races).  Rollover problems are avoided by locking the counter at
 * a maximum value.  The application must take appropriate action on this
 * error to preserve correction, but a properly behaved set of applications
 * will never trigger it; the era counter is meant to increment less than
 * once a second and is 32 bits.
 */
static int increment_era(struct era_policy *era, const char *old_era_str,
			 era_match_fn_t dummy)
{
	era_t old_era;

	if (kstrtou32(old_era_str, 10, &old_era)) {
		DMERR("couldn't parse old era");
		return -EINVAL;
	}

	if (old_era != atomic64_read(&era->current_era)) {
		DMERR("old era doesn't match, got %u, expected %u",
		      (unsigned) old_era,
		      (unsigned) atomic64_read(&era->current_era));
		return -ECANCELED;
	}

	if (atomic64_read(&era->current_era) >= ERA_OVERFLOW) {
		DMERR("era has overflowed");
		return -EOVERFLOW;
	}

	atomic64_inc(&era->current_era);
	return 0;
}

static void *era_cblock_to_hint(struct shim_walk_map_ctx *ctx,
				dm_cblock_t cblock, dm_oblock_t oblock)
{
	struct era_policy *era = to_era_policy(ctx->my_policy);
	era_t era_val;
	era_val = era->eras[from_cblock(cblock)];
	ctx->le32_buf = cpu_to_le32(era_val);
	return &ctx->le32_buf;
}

static int era_is_gt_value(era_t era, era_t value)
{
	return era > value;
}

static int era_is_gte_value(era_t era, era_t value)
{
	return era >= value;
}

static int era_is_lte_value(era_t era, era_t value)
{
	return era <= value;
}

static int era_is_lt_value(era_t era, era_t value)
{
	return era < value;
}

struct inval_oblocks_ctx {
	struct era_policy *era;
	era_match_fn_t era_match_fn;
	era_t test_era;
};

static int era_inval_oblocks(void *context, dm_cblock_t cblock,
			     dm_oblock_t oblock, void *unused)
{
	struct inval_oblocks_ctx *ctx = (struct inval_oblocks_ctx *)context;
	struct dm_cache_policy *child;
	era_t act_era;

	act_era = ctx->era->eras[from_cblock(cblock)];
	if (ctx->era_match_fn(act_era, ctx->test_era)) {
		child = ctx->era->policy.child;

		/*
		 * This deadlocks (lock against self) because child is calling
		 * us via the walk_mappings context callback, child's
		 * walk_mappings holds child's lock, and child's remove_mappings
		 * tries to get it again.  Not fixing because I believe the
		 * invalidate API is going to change.
		 */
		/* child->remove_mapping(child, oblock); */
	}

	return 0;
}

static int cond_unmap_by_era(struct era_policy *era, const char *test_era_str,
			     era_match_fn_t era_match_fn)
{
	struct shim_walk_map_ctx ctx;
	struct inval_oblocks_ctx io_ctx;
	era_t test_era;
	int r;

	/*
	 * Unmap blocks with eras matching the given era, according to the
	 * given matching function.
	 */

	if (kstrtou32(test_era_str, 10, &test_era))
		return -EINVAL;

	io_ctx.era = era;
	io_ctx.era_match_fn = era_match_fn;
	io_ctx.test_era = test_era;

	ctx.parent_ctx = &io_ctx;
	ctx.parent_fn = era_inval_oblocks;
	ctx.my_policy = &era->policy;
	ctx.child_hint_buf = NULL;
	ctx.cblock_to_hint_fn = NULL;

	mutex_lock(&era->lock);
	r = dm_cache_shim_utils_walk_map_with_ctx(&ctx);
	mutex_unlock(&era->lock);

	return r;
}

/*
 * Public interface, via the policy struct.  See dm-cache-policy.h for a
 * description of these.
 */

static void era_destroy(struct dm_cache_policy *p)
{
	struct era_policy *era = to_era_policy(p);

	vfree(era->eras);
	kfree(era);
}

static void __update_era(struct era_policy *era, dm_cblock_t b)
{
	era->eras[from_cblock(b)] = atomic64_read(&era->current_era);
}

static int era_map(struct dm_cache_policy *p, dm_oblock_t oblock,
		   bool can_block, bool can_migrate, bool discarded_oblock,
		   struct bio *bio, struct policy_result *result)
{
	struct era_policy *era = to_era_policy(p);
	int r;

	result->op = POLICY_MISS;

	if (can_block)
		mutex_lock(&era->lock);

	else if (!mutex_trylock(&era->lock))
		return -EWOULDBLOCK;

	/* Check for a mapping */
	r = policy_map(p->child, oblock, can_block, can_migrate,
		       discarded_oblock, bio, result);
	if (r)
		goto out;

	switch (result->op) {
	case POLICY_HIT:
		if (bio_data_dir(bio) == WRITE)
			__update_era(era, result->cblock);
		break;

	case POLICY_MISS:
		/* do nothing */
		break;

	case POLICY_NEW:
	case POLICY_REPLACE:
		__update_era(era, result->cblock);
		break;
	}

out:
	mutex_unlock(&era->lock);
	return r;
}

static void load_era(struct era_policy *era, dm_cblock_t cblock, __le32 *hint)
{
	era_t recovered_era = le32_to_cpu(*hint);
	era->eras[from_cblock(cblock)] = recovered_era;

	/*
	 * Make sure the current era is the highest seen.
	 */

	/*
	 * FIXME: problem if era incremented then suspend/resume without
	 * any cblock eras being updated.
	 */
	if (recovered_era >= atomic64_read(&era->current_era))
		atomic64_set(&era->current_era, recovered_era);
}

static int era_load_mapping(struct dm_cache_policy *p,
			    dm_oblock_t oblock, dm_cblock_t cblock,
			    void *hint, bool hint_valid)
{
	struct era_policy *era = to_era_policy(p);
	struct dm_cache_policy *child = era->policy.child;

	if (!hint_valid)
		return  policy_load_mapping(child, oblock, cblock, NULL, hint_valid);

	load_era(era, cblock, (__le32 *) hint);
	return policy_load_mapping(child, oblock, cblock, hint + sizeof(__le32), hint_valid);
}

static int era_walk_mappings(struct dm_cache_policy *p, policy_walk_fn fn,
			     void *context)
{
	return dm_cache_shim_utils_walk_map(p, fn, context, era_cblock_to_hint);
}

static void era_force_mapping(struct dm_cache_policy *p, dm_oblock_t old_oblock,
			      dm_oblock_t new_oblock)
{
	struct era_policy *era = to_era_policy(p);
	dm_cblock_t cblock;

	mutex_lock(&era->lock);

	if (!policy_lookup(p->child, old_oblock, &cblock)) {
		era->eras[from_cblock(cblock)] = atomic64_read(&era->current_era);
	}

	policy_force_mapping(p->child, old_oblock, new_oblock);

	mutex_unlock(&era->lock);
}

struct config_value_handler {
	const char *cmd;
	int (*handler_fn)(struct era_policy *, const char *, era_match_fn_t);
	era_match_fn_t match_fn;
};

/* FIXME: is a delete unmap request needed or is reloading the mapping sufficient to achieve it? */
static int era_set_config_value(struct dm_cache_policy *p, const char *key,
				const char *value)
{
	struct era_policy *era = to_era_policy(p);
	struct config_value_handler *vh, value_handlers[] = {
		{ "increment_era",                          increment_era,  NULL },
		{ "unmap_blocks_from_later_eras",           cond_unmap_by_era, era_is_gt_value },
		{ "unmap_blocks_from_this_era_and_later",   cond_unmap_by_era, era_is_gte_value },
		{ "unmap_blocks_from_this_era_and_earlier", cond_unmap_by_era, era_is_lte_value },
		{ "unmap_blocks_from_earlier_eras",         cond_unmap_by_era, era_is_lt_value },
		{ NULL }
	};

	for (vh = value_handlers; vh->cmd; vh++) {
		if (!strcasecmp(key, vh->cmd))
			return vh->handler_fn(era, value, vh->match_fn);
	}

	return policy_set_config_value(p->child, key, value);
}

static unsigned era_count_config_pairs(struct dm_cache_policy *p)
{
	return policy_count_config_pairs(p->child) + 1;
}

static int era_emit_config_values(struct dm_cache_policy *p, char *result,
				  unsigned maxlen)
{
	struct era_policy *era = to_era_policy(p);
	ssize_t sz = 0;

	DMEMIT("current_era %u ", (unsigned) atomic64_read(&era->current_era));
	return policy_emit_config_values(p->child, result + sz, maxlen - sz);
}

/* Init the policy plugin interface function pointers. */
static void init_policy_functions(struct era_policy *era)
{
	dm_cache_shim_utils_init_shim_policy(&era->policy);
	era->policy.destroy = era_destroy;
	era->policy.map = era_map;
	era->policy.load_mapping = era_load_mapping;
	era->policy.walk_mappings = era_walk_mappings;
	era->policy.force_mapping = era_force_mapping;
	era->policy.count_config_pairs = era_count_config_pairs;
	era->policy.emit_config_values = era_emit_config_values;
	era->policy.set_config_value = era_set_config_value;
}

static struct dm_cache_policy *era_create(dm_cblock_t cache_size,
					  sector_t origin_size,
					  sector_t cache_block_size)
{
	struct era_policy *era = kzalloc(sizeof(*era), GFP_KERNEL);

	if (!era)
		return NULL;

	init_policy_functions(era);
	era->cache_size = cache_size;
	mutex_init(&era->lock);

	era->eras = vzalloc(from_cblock(era->cache_size) *
			    sizeof(*era->eras));
	if (era->eras) {
		atomic64_set(&era->current_era, 0);
		return &era->policy;
	}

	kfree(era);
	return NULL;
}

/*----------------------------------------------------------------*/

static struct dm_cache_policy_type era_policy_type = {
	.name = "era",
	.version = {1, 0, 0},
	.hint_size = 4,
	.owner = THIS_MODULE,
	.create = era_create,
	.features = DM_CACHE_POLICY_SHIM
};

static int __init era_init(void)
{
	int r;

	r = dm_cache_policy_register(&era_policy_type);
	if (!r) {
		DMINFO("version %u.%u.%u loaded",
		       era_policy_type.version[0],
		       era_policy_type.version[1],
		       era_policy_type.version[2]);
		return 0;
	}

	DMERR("register failed %d", r);

	dm_cache_policy_unregister(&era_policy_type);
	return -ENOMEM;
}

static void __exit era_exit(void)
{
	dm_cache_policy_unregister(&era_policy_type);
}

module_init(era_init);
module_exit(era_exit);

MODULE_AUTHOR("Morgan Mears <dm-devel@redhat.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("era cache policy shim");
