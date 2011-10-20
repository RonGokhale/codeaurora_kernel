#ifndef _ASM_X86_UNISTD_64_COMPAT_H
#define _ASM_X86_UNISTD_64_COMPAT_H

#define __NR_rt_sigaction		__NR_64_rt_sigaction
#define __NR_rt_sigprocmask		__NR_64_rt_sigprocmask
#define __NR_rt_sigreturn		__NR_64_rt_sigreturn
#define __NR_ioctl			__NR_64_ioctl
#define __NR_readv			__NR_64_readv
#define __NR_writev			__NR_64_writev
#define __NR_recvfrom			__NR_64_recvfrom
#define __NR_sendmsg			__NR_64_sendmsg
#define __NR_recvmsg			__NR_64_recvmsg
#define __NR_execve			__NR_64_execve
#define __NR_times			__NR_64_times
#define __NR_rt_sigpending		__NR_64_rt_sigpending
#define __NR_rt_sigtimedwait		__NR_64_rt_sigtimedwait
#define __NR_rt_sigqueueinfo		__NR_64_rt_sigqueueinfo
#define __NR_sigaltstack		__NR_64_sigaltstack
#define __NR__sysctl			__NR_64__sysctl
#define __NR_nfsservctl			__NR_64_nfsservctl
#define __NR_timer_create		__NR_64_timer_create
#define __NR_mq_notify			__NR_64_mq_notify
#define __NR_kexec_load			__NR_64_kexec_load
#define __NR_waitid			__NR_64_waitid
#define __NR_set_robust_list		__NR_64_set_robust_list
#define __NR_get_robust_list		__NR_64_get_robust_list
#define __NR_vmsplice			__NR_64_vmsplice
#define __NR_move_pages			__NR_64_move_pages
#define __NR_preadv			__NR_64_preadv
#define __NR_pwritev			__NR_64_pwritev
#define __NR_rt_tgsigqueueinfo		__NR_64_rt_tgsigqueueinfo
#define __NR_recvmmsg			__NR_64_recvmmsg
#define __NR_sendmmsg			__NR_64_sendmmsg

#endif /* _ASM_X86_UNISTD_64_COMPAT_H */
