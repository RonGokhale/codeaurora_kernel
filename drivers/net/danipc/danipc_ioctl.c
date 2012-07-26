/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>

#include "danipc_k.h"
#include "danipc_ioctl.h"

#include "I_sys_types.h"
#include "IPC_api.h"
#include "danipc_lowlevel.h"


typedef struct {
	struct task_struct	*task;
	pid_t			pid;
} agent_data_t;
agent_data_t			agent_data[MAX_LOCAL_AGENT];

static int
is_process_alive(const agent_data_t *agent_data_p)
{
	int			rc = 0;
	struct task_struct	*task = find_task_by_vpid(agent_data_p->pid);

	if (task) {
		/* Paranoid check: task_struct exists, now verify it belongs
		 * to the correct process.
		 */
		if (agent_data_p->task == task && agent_data_p->pid == task->pid)
			rc = 1;
	}

	return rc;
}

static void
ipc_gc(void)
{
	unsigned		lid;

	for (lid = 0; lid < MAX_LOCAL_AGENT; lid++) {
		agent_data_t	*agent_data_p = &agent_data[lid];

		if (agent_data_p->task && !is_process_alive(agent_data_p)) {
			agent_data_p->task	= NULL;
			agent_data_p->pid	= 0;
		}
	}
}


/* Second registration is allowed only for the same process. */
static int
second_registration(danipc_reg_t *danipc_reg_p, const int aid, const unsigned lid)
{
	return (strcmp(agentTable[aid].agentName, danipc_reg_p->name) == 0 &&
		agent_data[lid].task == current) ? 1 : 0;
}

static int
register_agent(struct net_device *dev, danipc_reg_t *danipc_reg_p)
{
	danipc_reg_t		danipc_reg;
	unsigned		r_lid;
	unsigned		agent_id = INVALID_ID;
	int			rc = 0;

	ipc_gc();

	if (copy_from_user(&danipc_reg, danipc_reg_p, sizeof(danipc_reg)))
		return -EFAULT;

	r_lid = danipc_reg.requested_lid;

	if (r_lid != INVALID_ID) {
		const unsigned r_aid = __IPC_AGENT_ID(PLATFORM_my_ipc_id, r_lid);

		/* Requested ID is not used, so assign it */
		if ( !*agentTable[r_aid].agentName ||
			second_registration(&danipc_reg, r_aid, r_lid) ||
			(!strcmp(danipc_reg.name, agentTable[r_aid].agentName) &&
			 agent_data[r_lid].task == NULL)) {
			if (put_user(r_lid, &danipc_reg_p->assigned_lid))
				return -EFAULT;
			agent_id = r_aid;
		}
	}

	if (agent_id == INVALID_ID) {
		unsigned	lid;
		/* Scan for the ID already assigned */
		for(lid = 0; lid < MAX_LOCAL_AGENT; lid++) {
			const unsigned aid = __IPC_AGENT_ID(PLATFORM_my_ipc_id, lid);
			if (!strcmp(danipc_reg.name, agentTable[aid].agentName) &&
					agent_data[lid].task == NULL) {
				if (put_user(lid, &danipc_reg_p->assigned_lid))
					return -EFAULT;
				agent_id = aid;
				break;
			}
		}
	}

	if (agent_id == INVALID_ID) {
		unsigned	lid;
		/* Scan for the 1st free ID */
		for(lid = 0; lid < MAX_LOCAL_AGENT; lid++) {
			const unsigned aid = __IPC_AGENT_ID(PLATFORM_my_ipc_id, lid);
			if ( !*agentTable[aid].agentName ||
					second_registration(&danipc_reg, aid, lid)) {
				if (put_user(lid, &danipc_reg_p->assigned_lid))
					return -EFAULT;
				agent_id = aid;
				break;
			}
		}
	}

	if (agent_id != INVALID_ID) {
		const uint16_t cookie = cpu_to_be16(AGENTID_TO_COOKIE(agent_id, danipc_reg.prio));
		strncpy(agentTable[agent_id].agentName, danipc_reg.name,
			sizeof(agentTable[agent_id].agentName));
		agentTable[agent_id].agentName[sizeof(agentTable[agent_id].agentName)-1] = 0;
		if (put_user(cookie, &danipc_reg_p->cookie))
			rc = -EFAULT;
		agent_data[IPC_LocalId(agent_id)].task	= current;
		agent_data[IPC_LocalId(agent_id)].pid	= current->pid;
		netdev_dbg(dev, "%s(): agent_id=0x%x assigned_lid=0x%x agentTable[]=\"%s\"\n",
			__func__, agent_id, danipc_reg.assigned_lid,
			agentTable[agent_id].agentName);
	} else
		rc = -ENOBUFS;

	return rc;
}


static int
get_name_by_addr(struct net_device *dev, danipc_name_t *danipc_name_p)
{
	int			rc = -ENODATA;
	danipc_addr_t		addr;

	if (get_user(addr, &danipc_name_p->addr))
		return -EFAULT;

	if (*agentTable[addr].agentName) {
		if (copy_to_user(danipc_name_p->name,
				agentTable[danipc_name_p->addr].agentName,
				sizeof(danipc_name_p->name)))
			rc = -EFAULT;
		else
			rc = 0;
		netdev_dbg(dev, "%s(): addr=0x%x -> name=%s\n", __func__,
			addr, agentTable[danipc_name_p->addr].agentName);
	}

	return rc;
}


static int
get_addr_by_name(struct net_device *dev, danipc_name_t *danipc_name_p)
{
	char			name[MAX_AGENT_NAME];
	int			rc = -ENODATA;
	unsigned		aid;

	if (copy_from_user(name, danipc_name_p->name, sizeof(name)))
		return -EFAULT;

	for(aid = 0; aid < MAX_AGENTS; aid++)
	{
		if ( !strncmp(name, agentTable[aid].agentName,
				sizeof(name)) ) {
			const unsigned cpuid = aid / MAX_LOCAL_AGENT;
			const unsigned lid = aid % MAX_LOCAL_AGENT;
			if (put_user(__IPC_AGENT_ID(cpuid, lid), &danipc_name_p->addr))
				rc = -EFAULT;
			else
				rc = 0;
			netdev_dbg(dev, "%s(): name=%s -> addr=0x%x\n", __func__,
				agentTable[aid].agentName, aid);
		}
	}

	return rc;
}



int danipc_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int			rc = -EINVAL;

	if (dev && ifr->ifr_data) {
		danipc_priv_t	*priv = netdev_priv(dev);

		mutex_lock(&priv->lock);
		switch(cmd)
		{
		case DANIPC_IOCS_REGISTER:
			rc = register_agent(dev, (danipc_reg_t *)ifr->ifr_data);
			break;
		case DANIPC_IOCG_ADDR2NAME:
			rc = get_name_by_addr(dev, (danipc_name_t *)ifr->ifr_data);
			break;
		case DANIPC_IOCG_NAME2ADDR:
			rc = get_addr_by_name(dev, (danipc_name_t *)ifr->ifr_data);
			break;
		}
		mutex_unlock(&priv->lock);
	}
	return rc;
}

