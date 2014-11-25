#undef TRACE_SYSTEM
#define TRACE_SYSTEM fs

#if !defined(_TRACE_FS_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_FS_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(fs__inode,
	TP_PROTO(struct inode *inode),

	TP_ARGS(inode),

	TP_STRUCT__entry(
		__field(	dev_t,	dev			)
		__field(	ino_t,	ino			)
		__field(	uid_t,	uid			)
		__field(	gid_t,	gid			)
		__field(	__u16, mode			)
	),

	TP_fast_assign(
		__entry->dev	= inode->i_sb->s_dev;
		__entry->ino	= inode->i_ino;
		__entry->uid	= i_uid_read(inode);
		__entry->gid	= i_gid_read(inode);
		__entry->mode	= inode->i_mode;
	),

	TP_printk("dev %d,%d ino %lu mode 0%o uid %u gid %u",
		  MAJOR(__entry->dev), MINOR(__entry->dev),
		  (unsigned long) __entry->ino, __entry->mode,
		  __entry->uid, __entry->gid)
);

DEFINE_EVENT(fs__inode, fs_lazytime_defer,
	TP_PROTO(struct inode *inode),

	TP_ARGS(inode)
);

DEFINE_EVENT(fs__inode, fs_lazytime_evict,
	TP_PROTO(struct inode *inode),

	TP_ARGS(inode)
);

DEFINE_EVENT(fs__inode, fs_lazytime_flush,
	TP_PROTO(struct inode *inode),

	TP_ARGS(inode)
);
#endif /* _TRACE_FS_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
