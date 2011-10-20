#ifndef _ASM_X86_UNISTD_64_H
#define _ASM_X86_UNISTD_64_H

#ifndef __SYSCALL
#define __SYSCALL(a, b)
#endif

#ifndef __X32_SYSCALL
#define __X32_SYSCALL(a, b)
#endif

/* X32 system call mask.  */
#define __X32_SYSCALL_BIT	0x40000000

/* The first x32 system call.  */
#define __X32_SYSCALL_BASE	512

#ifdef __KERNEL__
#define __NR_COMMON_SYSCALL(n)	(n)
#define __NR_X32_SYSCALL(n)	(__X32_SYSCALL_BASE + n)
#else
#if __LP64__
#define __NR_COMMON_SYSCALL(n)	(n)
#else
#define __NR_COMMON_SYSCALL(n)	(__X32_SYSCALL_BIT | (n))
#endif
#define __NR_X32_SYSCALL(n)	(__X32_SYSCALL_BIT | (__X32_SYSCALL_BASE + n))
#endif

/*
 * This file contains the system call numbers.
 *
 * Note: holes are not allowed.
 */

/* at least 8 syscall per cacheline */
#define __NR_read				__NR_COMMON_SYSCALL(0)
__SYSCALL(__NR_read, sys_read)
#define __NR_write				__NR_COMMON_SYSCALL(1)
__SYSCALL(__NR_write, sys_write)
#define __NR_open				__NR_COMMON_SYSCALL(2)
__SYSCALL(__NR_open, sys_open)
#define __NR_close				__NR_COMMON_SYSCALL(3)
__SYSCALL(__NR_close, sys_close)
#define __NR_stat				__NR_COMMON_SYSCALL(4)
__SYSCALL(__NR_stat, sys_newstat)
#define __NR_fstat				__NR_COMMON_SYSCALL(5)
__SYSCALL(__NR_fstat, sys_newfstat)
#define __NR_lstat				__NR_COMMON_SYSCALL(6)
__SYSCALL(__NR_lstat, sys_newlstat)
#define __NR_poll				__NR_COMMON_SYSCALL(7)
__SYSCALL(__NR_poll, sys_poll)

#define __NR_lseek				__NR_COMMON_SYSCALL(8)
__SYSCALL(__NR_lseek, sys_lseek)
#define __NR_mmap				__NR_COMMON_SYSCALL(9)
__SYSCALL(__NR_mmap, sys_mmap)
#define __NR_mprotect				__NR_COMMON_SYSCALL(10)
__SYSCALL(__NR_mprotect, sys_mprotect)
#define __NR_munmap				__NR_COMMON_SYSCALL(11)
__SYSCALL(__NR_munmap, sys_munmap)
#define __NR_brk				__NR_COMMON_SYSCALL(12)
__SYSCALL(__NR_brk, sys_brk)
#define __NR_64_rt_sigaction			13
__SYSCALL(__NR_64_rt_sigaction, sys_rt_sigaction)
#define __NR_64_rt_sigprocmask			14
__SYSCALL(__NR_64_rt_sigprocmask, sys_rt_sigprocmask)
#define __NR_64_rt_sigreturn			15
__SYSCALL(__NR_64_rt_sigreturn, stub_rt_sigreturn)

#define __NR_64_ioctl				16
__SYSCALL(__NR_64_ioctl, sys_ioctl)
#define __NR_pread64				__NR_COMMON_SYSCALL(17)
__SYSCALL(__NR_pread64, sys_pread64)
#define __NR_pwrite64				__NR_COMMON_SYSCALL(18)
__SYSCALL(__NR_pwrite64, sys_pwrite64)
#define __NR_64_readv				19
__SYSCALL(__NR_64_readv, sys_readv)
#define __NR_64_writev				20
__SYSCALL(__NR_64_writev, sys_writev)
#define __NR_access				__NR_COMMON_SYSCALL(21)
__SYSCALL(__NR_access, sys_access)
#define __NR_pipe				__NR_COMMON_SYSCALL(22)
__SYSCALL(__NR_pipe, sys_pipe)
#define __NR_select				__NR_COMMON_SYSCALL(23)
__SYSCALL(__NR_select, sys_select)

#define __NR_sched_yield			__NR_COMMON_SYSCALL(24)
__SYSCALL(__NR_sched_yield, sys_sched_yield)
#define __NR_mremap				__NR_COMMON_SYSCALL(25)
__SYSCALL(__NR_mremap, sys_mremap)
#define __NR_msync				__NR_COMMON_SYSCALL(26)
__SYSCALL(__NR_msync, sys_msync)
#define __NR_mincore				__NR_COMMON_SYSCALL(27)
__SYSCALL(__NR_mincore, sys_mincore)
#define __NR_madvise				__NR_COMMON_SYSCALL(28)
__SYSCALL(__NR_madvise, sys_madvise)
#define __NR_shmget				__NR_COMMON_SYSCALL(29)
__SYSCALL(__NR_shmget, sys_shmget)
#define __NR_shmat				__NR_COMMON_SYSCALL(30)
__SYSCALL(__NR_shmat, sys_shmat)
#define __NR_shmctl				__NR_COMMON_SYSCALL(31)
__SYSCALL(__NR_shmctl, sys_shmctl)

#define __NR_dup				__NR_COMMON_SYSCALL(32)
__SYSCALL(__NR_dup, sys_dup)
#define __NR_dup2				__NR_COMMON_SYSCALL(33)
__SYSCALL(__NR_dup2, sys_dup2)
#define __NR_pause				__NR_COMMON_SYSCALL(34)
__SYSCALL(__NR_pause, sys_pause)
#define __NR_nanosleep				__NR_COMMON_SYSCALL(35)
__SYSCALL(__NR_nanosleep, sys_nanosleep)
#define __NR_getitimer				__NR_COMMON_SYSCALL(36)
__SYSCALL(__NR_getitimer, sys_getitimer)
#define __NR_alarm				__NR_COMMON_SYSCALL(37)
__SYSCALL(__NR_alarm, sys_alarm)
#define __NR_setitimer				__NR_COMMON_SYSCALL(38)
__SYSCALL(__NR_setitimer, sys_setitimer)
#define __NR_getpid				__NR_COMMON_SYSCALL(39)
__SYSCALL(__NR_getpid, sys_getpid)

#define __NR_sendfile				__NR_COMMON_SYSCALL(40)
__SYSCALL(__NR_sendfile, sys_sendfile64)
#define __NR_socket				__NR_COMMON_SYSCALL(41)
__SYSCALL(__NR_socket, sys_socket)
#define __NR_connect				__NR_COMMON_SYSCALL(42)
__SYSCALL(__NR_connect, sys_connect)
#define __NR_accept				__NR_COMMON_SYSCALL(43)
__SYSCALL(__NR_accept, sys_accept)
#define __NR_sendto				__NR_COMMON_SYSCALL(44)
__SYSCALL(__NR_sendto, sys_sendto)
#define __NR_64_recvfrom			45
__SYSCALL(__NR_64_recvfrom, sys_recvfrom)
#define __NR_64_sendmsg				46
__SYSCALL(__NR_64_sendmsg, sys_sendmsg)
#define __NR_64_recvmsg				47
__SYSCALL(__NR_64_recvmsg, sys_recvmsg)

#define __NR_shutdown				__NR_COMMON_SYSCALL(48)
__SYSCALL(__NR_shutdown, sys_shutdown)
#define __NR_bind				__NR_COMMON_SYSCALL(49)
__SYSCALL(__NR_bind, sys_bind)
#define __NR_listen				__NR_COMMON_SYSCALL(50)
__SYSCALL(__NR_listen, sys_listen)
#define __NR_getsockname			__NR_COMMON_SYSCALL(51)
__SYSCALL(__NR_getsockname, sys_getsockname)
#define __NR_getpeername			__NR_COMMON_SYSCALL(52)
__SYSCALL(__NR_getpeername, sys_getpeername)
#define __NR_socketpair				__NR_COMMON_SYSCALL(53)
__SYSCALL(__NR_socketpair, sys_socketpair)
#define __NR_setsockopt				__NR_COMMON_SYSCALL(54)
__SYSCALL(__NR_setsockopt, sys_setsockopt)
#define __NR_getsockopt				__NR_COMMON_SYSCALL(55)
__SYSCALL(__NR_getsockopt, sys_getsockopt)

#define __NR_clone				__NR_COMMON_SYSCALL(56)
__SYSCALL(__NR_clone, stub_clone)
#define __NR_fork				__NR_COMMON_SYSCALL(57)
__SYSCALL(__NR_fork, stub_fork)
#define __NR_vfork				__NR_COMMON_SYSCALL(58)
__SYSCALL(__NR_vfork, stub_vfork)
#define __NR_64_execve				59
__SYSCALL(__NR_64_execve, stub_execve)
#define __NR_exit				__NR_COMMON_SYSCALL(60)
__SYSCALL(__NR_exit, sys_exit)
#define __NR_wait4				__NR_COMMON_SYSCALL(61)
__SYSCALL(__NR_wait4, sys_wait4)
#define __NR_kill				__NR_COMMON_SYSCALL(62)
__SYSCALL(__NR_kill, sys_kill)
#define __NR_uname				__NR_COMMON_SYSCALL(63)
__SYSCALL(__NR_uname, sys_newuname)

#define __NR_semget				__NR_COMMON_SYSCALL(64)
__SYSCALL(__NR_semget, sys_semget)
#define __NR_semop				__NR_COMMON_SYSCALL(65)
__SYSCALL(__NR_semop, sys_semop)
#define __NR_semctl				__NR_COMMON_SYSCALL(66)
__SYSCALL(__NR_semctl, sys_semctl)
#define __NR_shmdt				__NR_COMMON_SYSCALL(67)
__SYSCALL(__NR_shmdt, sys_shmdt)
#define __NR_msgget				__NR_COMMON_SYSCALL(68)
__SYSCALL(__NR_msgget, sys_msgget)
#define __NR_msgsnd				__NR_COMMON_SYSCALL(69)
__SYSCALL(__NR_msgsnd, sys_msgsnd)
#define __NR_msgrcv				__NR_COMMON_SYSCALL(70)
__SYSCALL(__NR_msgrcv, sys_msgrcv)
#define __NR_msgctl				__NR_COMMON_SYSCALL(71)
__SYSCALL(__NR_msgctl, sys_msgctl)

#define __NR_fcntl				__NR_COMMON_SYSCALL(72)
__SYSCALL(__NR_fcntl, sys_fcntl)
#define __NR_flock				__NR_COMMON_SYSCALL(73)
__SYSCALL(__NR_flock, sys_flock)
#define __NR_fsync				__NR_COMMON_SYSCALL(74)
__SYSCALL(__NR_fsync, sys_fsync)
#define __NR_fdatasync				__NR_COMMON_SYSCALL(75)
__SYSCALL(__NR_fdatasync, sys_fdatasync)
#define __NR_truncate				__NR_COMMON_SYSCALL(76)
__SYSCALL(__NR_truncate, sys_truncate)
#define __NR_ftruncate				__NR_COMMON_SYSCALL(77)
__SYSCALL(__NR_ftruncate, sys_ftruncate)
#define __NR_64_getdents			78
__SYSCALL(__NR_64_getdents, sys_getdents)
#define __NR_getcwd				__NR_COMMON_SYSCALL(79)
__SYSCALL(__NR_getcwd, sys_getcwd)

#define __NR_chdir				__NR_COMMON_SYSCALL(80)
__SYSCALL(__NR_chdir, sys_chdir)
#define __NR_fchdir				__NR_COMMON_SYSCALL(81)
__SYSCALL(__NR_fchdir, sys_fchdir)
#define __NR_rename				__NR_COMMON_SYSCALL(82)
__SYSCALL(__NR_rename, sys_rename)
#define __NR_mkdir				__NR_COMMON_SYSCALL(83)
__SYSCALL(__NR_mkdir, sys_mkdir)
#define __NR_rmdir				__NR_COMMON_SYSCALL(84)
__SYSCALL(__NR_rmdir, sys_rmdir)
#define __NR_creat				__NR_COMMON_SYSCALL(85)
__SYSCALL(__NR_creat, sys_creat)
#define __NR_link				__NR_COMMON_SYSCALL(86)
__SYSCALL(__NR_link, sys_link)
#define __NR_unlink				__NR_COMMON_SYSCALL(87)
__SYSCALL(__NR_unlink, sys_unlink)

#define __NR_symlink				__NR_COMMON_SYSCALL(88)
__SYSCALL(__NR_symlink, sys_symlink)
#define __NR_readlink				__NR_COMMON_SYSCALL(89)
__SYSCALL(__NR_readlink, sys_readlink)
#define __NR_chmod				__NR_COMMON_SYSCALL(90)
__SYSCALL(__NR_chmod, sys_chmod)
#define __NR_fchmod				__NR_COMMON_SYSCALL(91)
__SYSCALL(__NR_fchmod, sys_fchmod)
#define __NR_chown				__NR_COMMON_SYSCALL(92)
__SYSCALL(__NR_chown, sys_chown)
#define __NR_fchown				__NR_COMMON_SYSCALL(93)
__SYSCALL(__NR_fchown, sys_fchown)
#define __NR_lchown				__NR_COMMON_SYSCALL(94)
__SYSCALL(__NR_lchown, sys_lchown)
#define __NR_umask				__NR_COMMON_SYSCALL(95)
__SYSCALL(__NR_umask, sys_umask)

#define __NR_gettimeofday			__NR_COMMON_SYSCALL(96)
__SYSCALL(__NR_gettimeofday, sys_gettimeofday)
#define __NR_getrlimit				__NR_COMMON_SYSCALL(97)
__SYSCALL(__NR_getrlimit, sys_getrlimit)
#define __NR_getrusage				__NR_COMMON_SYSCALL(98)
__SYSCALL(__NR_getrusage, sys_getrusage)
#define __NR_sysinfo				__NR_COMMON_SYSCALL(99)
__SYSCALL(__NR_sysinfo, sys_sysinfo)
#define __NR_64_times				100
__SYSCALL(__NR_64_times, sys_times)
#define __NR_ptrace				__NR_COMMON_SYSCALL(101)
__SYSCALL(__NR_ptrace, sys_ptrace)
#define __NR_getuid				__NR_COMMON_SYSCALL(102)
__SYSCALL(__NR_getuid, sys_getuid)
#define __NR_syslog				__NR_COMMON_SYSCALL(103)
__SYSCALL(__NR_syslog, sys_syslog)

/* at the very end the stuff that never runs during the benchmarks */
#define __NR_getgid				__NR_COMMON_SYSCALL(104)
__SYSCALL(__NR_getgid, sys_getgid)
#define __NR_setuid				__NR_COMMON_SYSCALL(105)
__SYSCALL(__NR_setuid, sys_setuid)
#define __NR_setgid				__NR_COMMON_SYSCALL(106)
__SYSCALL(__NR_setgid, sys_setgid)
#define __NR_geteuid				__NR_COMMON_SYSCALL(107)
__SYSCALL(__NR_geteuid, sys_geteuid)
#define __NR_getegid				__NR_COMMON_SYSCALL(108)
__SYSCALL(__NR_getegid, sys_getegid)
#define __NR_setpgid				__NR_COMMON_SYSCALL(109)
__SYSCALL(__NR_setpgid, sys_setpgid)
#define __NR_getppid				__NR_COMMON_SYSCALL(110)
__SYSCALL(__NR_getppid, sys_getppid)
#define __NR_getpgrp				__NR_COMMON_SYSCALL(111)
__SYSCALL(__NR_getpgrp, sys_getpgrp)

#define __NR_setsid				__NR_COMMON_SYSCALL(112)
__SYSCALL(__NR_setsid, sys_setsid)
#define __NR_setreuid				__NR_COMMON_SYSCALL(113)
__SYSCALL(__NR_setreuid, sys_setreuid)
#define __NR_setregid				__NR_COMMON_SYSCALL(114)
__SYSCALL(__NR_setregid, sys_setregid)
#define __NR_getgroups				__NR_COMMON_SYSCALL(115)
__SYSCALL(__NR_getgroups, sys_getgroups)
#define __NR_setgroups				__NR_COMMON_SYSCALL(116)
__SYSCALL(__NR_setgroups, sys_setgroups)
#define __NR_setresuid				__NR_COMMON_SYSCALL(117)
__SYSCALL(__NR_setresuid, sys_setresuid)
#define __NR_getresuid				__NR_COMMON_SYSCALL(118)
__SYSCALL(__NR_getresuid, sys_getresuid)
#define __NR_setresgid				__NR_COMMON_SYSCALL(119)
__SYSCALL(__NR_setresgid, sys_setresgid)

#define __NR_getresgid				__NR_COMMON_SYSCALL(120)
__SYSCALL(__NR_getresgid, sys_getresgid)
#define __NR_getpgid				__NR_COMMON_SYSCALL(121)
__SYSCALL(__NR_getpgid, sys_getpgid)
#define __NR_setfsuid				__NR_COMMON_SYSCALL(122)
__SYSCALL(__NR_setfsuid, sys_setfsuid)
#define __NR_setfsgid				__NR_COMMON_SYSCALL(123)
__SYSCALL(__NR_setfsgid, sys_setfsgid)
#define __NR_getsid				__NR_COMMON_SYSCALL(124)
__SYSCALL(__NR_getsid, sys_getsid)
#define __NR_capget				__NR_COMMON_SYSCALL(125)
__SYSCALL(__NR_capget, sys_capget)
#define __NR_capset				__NR_COMMON_SYSCALL(126)
__SYSCALL(__NR_capset, sys_capset)

#define __NR_64_rt_sigpending			127
__SYSCALL(__NR_64_rt_sigpending, sys_rt_sigpending)
#define __NR_64_rt_sigtimedwait			128
__SYSCALL(__NR_64_rt_sigtimedwait, sys_rt_sigtimedwait)
#define __NR_64_rt_sigqueueinfo			129
__SYSCALL(__NR_64_rt_sigqueueinfo, sys_rt_sigqueueinfo)
#define __NR_rt_sigsuspend			__NR_COMMON_SYSCALL(130)
__SYSCALL(__NR_rt_sigsuspend, sys_rt_sigsuspend)
#define __NR_64_sigaltstack			131
__SYSCALL(__NR_64_sigaltstack, stub_sigaltstack)
#define __NR_utime				__NR_COMMON_SYSCALL(132)
__SYSCALL(__NR_utime, sys_utime)
#define __NR_mknod				__NR_COMMON_SYSCALL(133)
__SYSCALL(__NR_mknod, sys_mknod)

/* Only needed for a.out */
#define __NR_64_uselib				134
__SYSCALL(__NR_64_uselib, sys_ni_syscall)
#define __NR_personality			__NR_COMMON_SYSCALL(135)
__SYSCALL(__NR_personality, sys_personality)

#define __NR_ustat				__NR_COMMON_SYSCALL(136)
__SYSCALL(__NR_ustat, sys_ustat)
#define __NR_statfs				__NR_COMMON_SYSCALL(137)
__SYSCALL(__NR_statfs, sys_statfs)
#define __NR_fstatfs				__NR_COMMON_SYSCALL(138)
__SYSCALL(__NR_fstatfs, sys_fstatfs)
#define __NR_sysfs				__NR_COMMON_SYSCALL(139)
__SYSCALL(__NR_sysfs, sys_sysfs)

#define __NR_getpriority			__NR_COMMON_SYSCALL(140)
__SYSCALL(__NR_getpriority, sys_getpriority)
#define __NR_setpriority			__NR_COMMON_SYSCALL(141)
__SYSCALL(__NR_setpriority, sys_setpriority)
#define __NR_sched_setparam			__NR_COMMON_SYSCALL(142)
__SYSCALL(__NR_sched_setparam, sys_sched_setparam)
#define __NR_sched_getparam			__NR_COMMON_SYSCALL(143)
__SYSCALL(__NR_sched_getparam, sys_sched_getparam)
#define __NR_sched_setscheduler			__NR_COMMON_SYSCALL(144)
__SYSCALL(__NR_sched_setscheduler, sys_sched_setscheduler)
#define __NR_sched_getscheduler			__NR_COMMON_SYSCALL(145)
__SYSCALL(__NR_sched_getscheduler, sys_sched_getscheduler)
#define __NR_sched_get_priority_max		__NR_COMMON_SYSCALL(146)
__SYSCALL(__NR_sched_get_priority_max, sys_sched_get_priority_max)
#define __NR_sched_get_priority_min		__NR_COMMON_SYSCALL(147)
__SYSCALL(__NR_sched_get_priority_min, sys_sched_get_priority_min)
#define __NR_sched_rr_get_interval		__NR_COMMON_SYSCALL(148)
__SYSCALL(__NR_sched_rr_get_interval, sys_sched_rr_get_interval)

#define __NR_mlock				__NR_COMMON_SYSCALL(149)
__SYSCALL(__NR_mlock, sys_mlock)
#define __NR_munlock				__NR_COMMON_SYSCALL(150)
__SYSCALL(__NR_munlock, sys_munlock)
#define __NR_mlockall				__NR_COMMON_SYSCALL(151)
__SYSCALL(__NR_mlockall, sys_mlockall)
#define __NR_munlockall				__NR_COMMON_SYSCALL(152)
__SYSCALL(__NR_munlockall, sys_munlockall)

#define __NR_vhangup				__NR_COMMON_SYSCALL(153)
__SYSCALL(__NR_vhangup, sys_vhangup)

#define __NR_modify_ldt				__NR_COMMON_SYSCALL(154)
__SYSCALL(__NR_modify_ldt, sys_modify_ldt)

#define __NR_pivot_root				__NR_COMMON_SYSCALL(155)
__SYSCALL(__NR_pivot_root, sys_pivot_root)

#define __NR_64__sysctl				156
__SYSCALL(__NR_64__sysctl, sys_sysctl)

#define __NR_prctl				__NR_COMMON_SYSCALL(157)
__SYSCALL(__NR_prctl, sys_prctl)
#define __NR_arch_prctl				__NR_COMMON_SYSCALL(158)
__SYSCALL(__NR_arch_prctl, sys_arch_prctl)

#define __NR_adjtimex				__NR_COMMON_SYSCALL(159)
__SYSCALL(__NR_adjtimex, sys_adjtimex)

#define __NR_setrlimit				__NR_COMMON_SYSCALL(160)
__SYSCALL(__NR_setrlimit, sys_setrlimit)

#define __NR_chroot				__NR_COMMON_SYSCALL(161)
__SYSCALL(__NR_chroot, sys_chroot)

#define __NR_sync				__NR_COMMON_SYSCALL(162)
__SYSCALL(__NR_sync, sys_sync)

#define __NR_acct				__NR_COMMON_SYSCALL(163)
__SYSCALL(__NR_acct, sys_acct)

#define __NR_settimeofday			__NR_COMMON_SYSCALL(164)
__SYSCALL(__NR_settimeofday, sys_settimeofday)

#define __NR_mount				__NR_COMMON_SYSCALL(165)
__SYSCALL(__NR_mount, sys_mount)
#define __NR_umount2				__NR_COMMON_SYSCALL(166)
__SYSCALL(__NR_umount2, sys_umount)

#define __NR_swapon				__NR_COMMON_SYSCALL(167)
__SYSCALL(__NR_swapon, sys_swapon)
#define __NR_swapoff				__NR_COMMON_SYSCALL(168)
__SYSCALL(__NR_swapoff, sys_swapoff)

#define __NR_reboot				__NR_COMMON_SYSCALL(169)
__SYSCALL(__NR_reboot, sys_reboot)

#define __NR_sethostname			__NR_COMMON_SYSCALL(170)
__SYSCALL(__NR_sethostname, sys_sethostname)
#define __NR_setdomainname			__NR_COMMON_SYSCALL(171)
__SYSCALL(__NR_setdomainname, sys_setdomainname)

#define __NR_iopl				__NR_COMMON_SYSCALL(172)
__SYSCALL(__NR_iopl, stub_iopl)
#define __NR_ioperm				__NR_COMMON_SYSCALL(173)
__SYSCALL(__NR_ioperm, sys_ioperm)

#define __NR_64_create_module			174
__SYSCALL(__NR_64_create_module, sys_ni_syscall)
#define __NR_init_module			__NR_COMMON_SYSCALL(175)
__SYSCALL(__NR_init_module, sys_init_module)
#define __NR_delete_module			__NR_COMMON_SYSCALL(176)
__SYSCALL(__NR_delete_module, sys_delete_module)
#define __NR_64_get_kernel_syms			177
__SYSCALL(__NR_64_get_kernel_syms, sys_ni_syscall)
#define __NR_64_query_module			178
__SYSCALL(__NR_64_query_module, sys_ni_syscall)

#define __NR_quotactl				__NR_COMMON_SYSCALL(179)
__SYSCALL(__NR_quotactl, sys_quotactl)

#define __NR_64_nfsservctl			180
__SYSCALL(__NR_64_nfsservctl, sys_ni_syscall)

/* reserved for LiS/STREAMS */
#define __NR_getpmsg				__NR_COMMON_SYSCALL(181)
__SYSCALL(__NR_getpmsg, sys_ni_syscall)
#define __NR_putpmsg				__NR_COMMON_SYSCALL(182)
__SYSCALL(__NR_putpmsg, sys_ni_syscall)

/* reserved for AFS */
#define __NR_afs_syscall			__NR_COMMON_SYSCALL(183)
__SYSCALL(__NR_afs_syscall, sys_ni_syscall)

/* reserved for tux */
#define __NR_tuxcall				__NR_COMMON_SYSCALL(184)
__SYSCALL(__NR_tuxcall, sys_ni_syscall)

#define __NR_security				__NR_COMMON_SYSCALL(185)
__SYSCALL(__NR_security, sys_ni_syscall)

#define __NR_gettid				__NR_COMMON_SYSCALL(186)
__SYSCALL(__NR_gettid, sys_gettid)

#define __NR_readahead				__NR_COMMON_SYSCALL(187)
__SYSCALL(__NR_readahead, sys_readahead)
#define __NR_setxattr				__NR_COMMON_SYSCALL(188)
__SYSCALL(__NR_setxattr, sys_setxattr)
#define __NR_lsetxattr				__NR_COMMON_SYSCALL(189)
__SYSCALL(__NR_lsetxattr, sys_lsetxattr)
#define __NR_fsetxattr				__NR_COMMON_SYSCALL(190)
__SYSCALL(__NR_fsetxattr, sys_fsetxattr)
#define __NR_getxattr				__NR_COMMON_SYSCALL(191)
__SYSCALL(__NR_getxattr, sys_getxattr)
#define __NR_lgetxattr				__NR_COMMON_SYSCALL(192)
__SYSCALL(__NR_lgetxattr, sys_lgetxattr)
#define __NR_fgetxattr				__NR_COMMON_SYSCALL(193)
__SYSCALL(__NR_fgetxattr, sys_fgetxattr)
#define __NR_listxattr				__NR_COMMON_SYSCALL(194)
__SYSCALL(__NR_listxattr, sys_listxattr)
#define __NR_llistxattr				__NR_COMMON_SYSCALL(195)
__SYSCALL(__NR_llistxattr, sys_llistxattr)
#define __NR_flistxattr				__NR_COMMON_SYSCALL(196)
__SYSCALL(__NR_flistxattr, sys_flistxattr)
#define __NR_removexattr			__NR_COMMON_SYSCALL(197)
__SYSCALL(__NR_removexattr, sys_removexattr)
#define __NR_lremovexattr			__NR_COMMON_SYSCALL(198)
__SYSCALL(__NR_lremovexattr, sys_lremovexattr)
#define __NR_fremovexattr			__NR_COMMON_SYSCALL(199)
__SYSCALL(__NR_fremovexattr, sys_fremovexattr)
#define __NR_tkill				__NR_COMMON_SYSCALL(200)
__SYSCALL(__NR_tkill, sys_tkill)
#define __NR_time				__NR_COMMON_SYSCALL(201)
__SYSCALL(__NR_time, sys_time)
#define __NR_futex				__NR_COMMON_SYSCALL(202)
__SYSCALL(__NR_futex, sys_futex)
#define __NR_sched_setaffinity			__NR_COMMON_SYSCALL(203)
__SYSCALL(__NR_sched_setaffinity, sys_sched_setaffinity)
#define __NR_sched_getaffinity			__NR_COMMON_SYSCALL(204)
__SYSCALL(__NR_sched_getaffinity, sys_sched_getaffinity)
#define __NR_64_set_thread_area			205
__SYSCALL(__NR_64_set_thread_area, sys_ni_syscall)	/* use arch_prctl */
#define __NR_io_setup				__NR_COMMON_SYSCALL(206)
__SYSCALL(__NR_io_setup, sys_io_setup)
#define __NR_io_destroy				__NR_COMMON_SYSCALL(207)
__SYSCALL(__NR_io_destroy, sys_io_destroy)
#define __NR_io_getevents			__NR_COMMON_SYSCALL(208)
__SYSCALL(__NR_io_getevents, sys_io_getevents)
#define __NR_io_submit				__NR_COMMON_SYSCALL(209)
__SYSCALL(__NR_io_submit, sys_io_submit)
#define __NR_io_cancel				__NR_COMMON_SYSCALL(210)
__SYSCALL(__NR_io_cancel, sys_io_cancel)
#define __NR_64_get_thread_area			211
__SYSCALL(__NR_64_get_thread_area, sys_ni_syscall)	/* use arch_prctl */
#define __NR_lookup_dcookie			__NR_COMMON_SYSCALL(212)
__SYSCALL(__NR_lookup_dcookie, sys_lookup_dcookie)
#define __NR_epoll_create			__NR_COMMON_SYSCALL(213)
__SYSCALL(__NR_epoll_create, sys_epoll_create)
#define __NR_64_epoll_ctl_old			214
__SYSCALL(__NR_64_epoll_ctl_old, sys_ni_syscall)
#define __NR_64_epoll_wait_old			215
__SYSCALL(__NR_64_epoll_wait_old, sys_ni_syscall)
#define __NR_remap_file_pages			__NR_COMMON_SYSCALL(216)
__SYSCALL(__NR_remap_file_pages, sys_remap_file_pages)
#define __NR_getdents64				__NR_COMMON_SYSCALL(217)
__SYSCALL(__NR_getdents64, sys_getdents64)
#define __NR_set_tid_address			__NR_COMMON_SYSCALL(218)
__SYSCALL(__NR_set_tid_address, sys_set_tid_address)
#define __NR_restart_syscall			__NR_COMMON_SYSCALL(219)
__SYSCALL(__NR_restart_syscall, sys_restart_syscall)
#define __NR_semtimedop				__NR_COMMON_SYSCALL(220)
__SYSCALL(__NR_semtimedop, sys_semtimedop)
#define __NR_fadvise64				__NR_COMMON_SYSCALL(221)
__SYSCALL(__NR_fadvise64, sys_fadvise64)
#define __NR_64_timer_create			222
__SYSCALL(__NR_64_timer_create, sys_timer_create)
#define __NR_timer_settime			__NR_COMMON_SYSCALL(223)
__SYSCALL(__NR_timer_settime, sys_timer_settime)
#define __NR_timer_gettime			__NR_COMMON_SYSCALL(224)
__SYSCALL(__NR_timer_gettime, sys_timer_gettime)
#define __NR_timer_getoverrun			__NR_COMMON_SYSCALL(225)
__SYSCALL(__NR_timer_getoverrun, sys_timer_getoverrun)
#define __NR_timer_delete			__NR_COMMON_SYSCALL(226)
__SYSCALL(__NR_timer_delete, sys_timer_delete)
#define __NR_clock_settime			__NR_COMMON_SYSCALL(227)
__SYSCALL(__NR_clock_settime, sys_clock_settime)
#define __NR_clock_gettime			__NR_COMMON_SYSCALL(228)
__SYSCALL(__NR_clock_gettime, sys_clock_gettime)
#define __NR_clock_getres			__NR_COMMON_SYSCALL(229)
__SYSCALL(__NR_clock_getres, sys_clock_getres)
#define __NR_clock_nanosleep			__NR_COMMON_SYSCALL(230)
__SYSCALL(__NR_clock_nanosleep, sys_clock_nanosleep)
#define __NR_exit_group				__NR_COMMON_SYSCALL(231)
__SYSCALL(__NR_exit_group, sys_exit_group)
#define __NR_epoll_wait				__NR_COMMON_SYSCALL(232)
__SYSCALL(__NR_epoll_wait, sys_epoll_wait)
#define __NR_epoll_ctl				__NR_COMMON_SYSCALL(233)
__SYSCALL(__NR_epoll_ctl, sys_epoll_ctl)
#define __NR_tgkill				__NR_COMMON_SYSCALL(234)
__SYSCALL(__NR_tgkill, sys_tgkill)
#define __NR_utimes				__NR_COMMON_SYSCALL(235)
__SYSCALL(__NR_utimes, sys_utimes)
#define __NR_64_vserver				236
__SYSCALL(__NR_64_vserver, sys_ni_syscall)
#define __NR_mbind				__NR_COMMON_SYSCALL(237)
__SYSCALL(__NR_mbind, sys_mbind)
#define __NR_set_mempolicy			__NR_COMMON_SYSCALL(238)
__SYSCALL(__NR_set_mempolicy, sys_set_mempolicy)
#define __NR_get_mempolicy			__NR_COMMON_SYSCALL(239)
__SYSCALL(__NR_get_mempolicy, sys_get_mempolicy)
#define __NR_mq_open				__NR_COMMON_SYSCALL(240)
__SYSCALL(__NR_mq_open, sys_mq_open)
#define __NR_mq_unlink				__NR_COMMON_SYSCALL(241)
__SYSCALL(__NR_mq_unlink, sys_mq_unlink)
#define __NR_mq_timedsend			__NR_COMMON_SYSCALL(242)
__SYSCALL(__NR_mq_timedsend, sys_mq_timedsend)
#define __NR_mq_timedreceive			__NR_COMMON_SYSCALL(243)
__SYSCALL(__NR_mq_timedreceive, sys_mq_timedreceive)
#define __NR_64_mq_notify			244
__SYSCALL(__NR_64_mq_notify, sys_mq_notify)
#define __NR_mq_getsetattr			__NR_COMMON_SYSCALL(245)
__SYSCALL(__NR_mq_getsetattr, sys_mq_getsetattr)
#define __NR_64_kexec_load			246
__SYSCALL(__NR_64_kexec_load, sys_kexec_load)
#define __NR_64_waitid				247
__SYSCALL(__NR_64_waitid, sys_waitid)
#define __NR_add_key				__NR_COMMON_SYSCALL(248)
__SYSCALL(__NR_add_key, sys_add_key)
#define __NR_request_key			__NR_COMMON_SYSCALL(249)
__SYSCALL(__NR_request_key, sys_request_key)
#define __NR_keyctl				__NR_COMMON_SYSCALL(250)
__SYSCALL(__NR_keyctl, sys_keyctl)
#define __NR_ioprio_set				__NR_COMMON_SYSCALL(251)
__SYSCALL(__NR_ioprio_set, sys_ioprio_set)
#define __NR_ioprio_get				__NR_COMMON_SYSCALL(252)
__SYSCALL(__NR_ioprio_get, sys_ioprio_get)
#define __NR_inotify_init			__NR_COMMON_SYSCALL(253)
__SYSCALL(__NR_inotify_init, sys_inotify_init)
#define __NR_inotify_add_watch			__NR_COMMON_SYSCALL(254)
__SYSCALL(__NR_inotify_add_watch, sys_inotify_add_watch)
#define __NR_inotify_rm_watch			__NR_COMMON_SYSCALL(255)
__SYSCALL(__NR_inotify_rm_watch, sys_inotify_rm_watch)
#define __NR_migrate_pages			__NR_COMMON_SYSCALL(256)
__SYSCALL(__NR_migrate_pages, sys_migrate_pages)
#define __NR_openat				__NR_COMMON_SYSCALL(257)
__SYSCALL(__NR_openat, sys_openat)
#define __NR_mkdirat				__NR_COMMON_SYSCALL(258)
__SYSCALL(__NR_mkdirat, sys_mkdirat)
#define __NR_mknodat				__NR_COMMON_SYSCALL(259)
__SYSCALL(__NR_mknodat, sys_mknodat)
#define __NR_fchownat				__NR_COMMON_SYSCALL(260)
__SYSCALL(__NR_fchownat, sys_fchownat)
#define __NR_futimesat				__NR_COMMON_SYSCALL(261)
__SYSCALL(__NR_futimesat, sys_futimesat)
#define __NR_newfstatat				__NR_COMMON_SYSCALL(262)
__SYSCALL(__NR_newfstatat, sys_newfstatat)
#define __NR_unlinkat				__NR_COMMON_SYSCALL(263)
__SYSCALL(__NR_unlinkat, sys_unlinkat)
#define __NR_renameat				__NR_COMMON_SYSCALL(264)
__SYSCALL(__NR_renameat, sys_renameat)
#define __NR_linkat				__NR_COMMON_SYSCALL(265)
__SYSCALL(__NR_linkat, sys_linkat)
#define __NR_symlinkat				__NR_COMMON_SYSCALL(266)
__SYSCALL(__NR_symlinkat, sys_symlinkat)
#define __NR_readlinkat				__NR_COMMON_SYSCALL(267)
__SYSCALL(__NR_readlinkat, sys_readlinkat)
#define __NR_fchmodat				__NR_COMMON_SYSCALL(268)
__SYSCALL(__NR_fchmodat, sys_fchmodat)
#define __NR_faccessat				__NR_COMMON_SYSCALL(269)
__SYSCALL(__NR_faccessat, sys_faccessat)
#define __NR_pselect6				__NR_COMMON_SYSCALL(270)
__SYSCALL(__NR_pselect6, sys_pselect6)
#define __NR_ppoll				__NR_COMMON_SYSCALL(271)
__SYSCALL(__NR_ppoll, sys_ppoll)
#define __NR_unshare				__NR_COMMON_SYSCALL(272)
__SYSCALL(__NR_unshare,	sys_unshare)
#define __NR_64_set_robust_list			273
__SYSCALL(__NR_64_set_robust_list, sys_set_robust_list)
#define __NR_64_get_robust_list			274
__SYSCALL(__NR_64_get_robust_list, sys_get_robust_list)
#define __NR_splice				__NR_COMMON_SYSCALL(275)
__SYSCALL(__NR_splice, sys_splice)
#define __NR_tee				__NR_COMMON_SYSCALL(276)
__SYSCALL(__NR_tee, sys_tee)
#define __NR_sync_file_range			__NR_COMMON_SYSCALL(277)
__SYSCALL(__NR_sync_file_range, sys_sync_file_range)
#define __NR_64_vmsplice			278
__SYSCALL(__NR_64_vmsplice, sys_vmsplice)
#define __NR_64_move_pages			279
__SYSCALL(__NR_64_move_pages, sys_move_pages)
#define __NR_utimensat				__NR_COMMON_SYSCALL(280)
__SYSCALL(__NR_utimensat, sys_utimensat)
#define __IGNORE_getcpu		/* implemented as a vsyscall */
#define __NR_epoll_pwait			__NR_COMMON_SYSCALL(281)
__SYSCALL(__NR_epoll_pwait, sys_epoll_pwait)
#define __NR_signalfd				__NR_COMMON_SYSCALL(282)
__SYSCALL(__NR_signalfd, sys_signalfd)
#define __NR_timerfd_create			__NR_COMMON_SYSCALL(283)
__SYSCALL(__NR_timerfd_create, sys_timerfd_create)
#define __NR_eventfd				__NR_COMMON_SYSCALL(284)
__SYSCALL(__NR_eventfd, sys_eventfd)
#define __NR_fallocate				__NR_COMMON_SYSCALL(285)
__SYSCALL(__NR_fallocate, sys_fallocate)
#define __NR_timerfd_settime			__NR_COMMON_SYSCALL(286)
__SYSCALL(__NR_timerfd_settime, sys_timerfd_settime)
#define __NR_timerfd_gettime			__NR_COMMON_SYSCALL(287)
__SYSCALL(__NR_timerfd_gettime, sys_timerfd_gettime)
#define __NR_accept4				__NR_COMMON_SYSCALL(288)
__SYSCALL(__NR_accept4, sys_accept4)
#define __NR_signalfd4				__NR_COMMON_SYSCALL(289)
__SYSCALL(__NR_signalfd4, sys_signalfd4)
#define __NR_eventfd2				__NR_COMMON_SYSCALL(290)
__SYSCALL(__NR_eventfd2, sys_eventfd2)
#define __NR_epoll_create1			__NR_COMMON_SYSCALL(291)
__SYSCALL(__NR_epoll_create1, sys_epoll_create1)
#define __NR_dup3				__NR_COMMON_SYSCALL(292)
__SYSCALL(__NR_dup3, sys_dup3)
#define __NR_pipe2				__NR_COMMON_SYSCALL(293)
__SYSCALL(__NR_pipe2, sys_pipe2)
#define __NR_inotify_init1			__NR_COMMON_SYSCALL(294)
__SYSCALL(__NR_inotify_init1, sys_inotify_init1)
#define __NR_64_preadv				295
__SYSCALL(__NR_64_preadv, sys_preadv)
#define __NR_64_pwritev				296
__SYSCALL(__NR_64_pwritev, sys_pwritev)
#define __NR_64_rt_tgsigqueueinfo		297
__SYSCALL(__NR_64_rt_tgsigqueueinfo, sys_rt_tgsigqueueinfo)
#define __NR_perf_event_open			__NR_COMMON_SYSCALL(298)
__SYSCALL(__NR_perf_event_open, sys_perf_event_open)
#define __NR_64_recvmmsg			299
__SYSCALL(__NR_64_recvmmsg, sys_recvmmsg)
#define __NR_fanotify_init			__NR_COMMON_SYSCALL(300)
__SYSCALL(__NR_fanotify_init, sys_fanotify_init)
#define __NR_fanotify_mark			__NR_COMMON_SYSCALL(301)
__SYSCALL(__NR_fanotify_mark, sys_fanotify_mark)
#define __NR_prlimit64				__NR_COMMON_SYSCALL(302)
__SYSCALL(__NR_prlimit64, sys_prlimit64)
#define __NR_name_to_handle_at			__NR_COMMON_SYSCALL(303)
__SYSCALL(__NR_name_to_handle_at, sys_name_to_handle_at)
#define __NR_open_by_handle_at			__NR_COMMON_SYSCALL(304)
__SYSCALL(__NR_open_by_handle_at, sys_open_by_handle_at)
#define __NR_clock_adjtime			__NR_COMMON_SYSCALL(305)
__SYSCALL(__NR_clock_adjtime, sys_clock_adjtime)
#define __NR_syncfs                             __NR_COMMON_SYSCALL(306)
__SYSCALL(__NR_syncfs, sys_syncfs)
#define __NR_64_sendmmsg			307
__SYSCALL(__NR_64_sendmmsg, sys_sendmmsg)
#define __NR_setns				__NR_COMMON_SYSCALL(308)
__SYSCALL(__NR_setns, sys_setns)

/* X32 support.  */
#define __NR_x32_rt_sigaction			__NR_X32_SYSCALL(0)
__X32_SYSCALL(__NR_x32_rt_sigaction, sys32_rt_sigaction)
#define __NR_x32_rt_sigprocmask			__NR_X32_SYSCALL(1)
__X32_SYSCALL(__NR_x32_rt_sigprocmask, sys32_rt_sigprocmask)
#define __NR_x32_rt_sigreturn			__NR_X32_SYSCALL(2)
__X32_SYSCALL(__NR_x32_rt_sigreturn, stub_x32_rt_sigreturn)
#define __NR_x32_ioctl				__NR_X32_SYSCALL(3)
__X32_SYSCALL(__NR_x32_ioctl, compat_sys_ioctl)
#define __NR_x32_readv				__NR_X32_SYSCALL(4)
__X32_SYSCALL(__NR_x32_readv, compat_sys_readv)
#define __NR_x32_writev				__NR_X32_SYSCALL(5)
__X32_SYSCALL(__NR_x32_writev, compat_sys_writev)
#define __NR_x32_recvfrom			__NR_X32_SYSCALL(6)
__X32_SYSCALL(__NR_x32_recvfrom, compat_sys_recvfrom)
#define __NR_x32_sendmsg			__NR_X32_SYSCALL(7)
__X32_SYSCALL(__NR_x32_sendmsg, compat_sys_sendmsg)

#define __NR_x32_recvmsg			__NR_X32_SYSCALL(8)
__X32_SYSCALL(__NR_x32_recvmsg, compat_sys_recvmsg)
#define __NR_x32_execve				__NR_X32_SYSCALL(9)
__X32_SYSCALL(__NR_x32_execve, stub_x32_execve)
#define __NR_x32_times				__NR_X32_SYSCALL(10)
__X32_SYSCALL(__NR_x32_times, compat_sys_times)
#define __NR_x32_rt_sigpending			__NR_X32_SYSCALL(11)
__X32_SYSCALL(__NR_x32_rt_sigpending, sys32_rt_sigpending)
#define __NR_x32_rt_sigtimedwait		__NR_X32_SYSCALL(12)
__X32_SYSCALL(__NR_x32_rt_sigtimedwait, compat_sys_rt_sigtimedwait)
#define __NR_x32_rt_sigqueueinfo		__NR_X32_SYSCALL(13)
__X32_SYSCALL(__NR_x32_rt_sigqueueinfo, sys32_rt_sigqueueinfo)
#define __NR_x32_sigaltstack			__NR_X32_SYSCALL(14)
__X32_SYSCALL(__NR_x32_sigaltstack, stub_x32_sigaltstack)
#define __NR_x32_timer_create			__NR_X32_SYSCALL(15)
__X32_SYSCALL(__NR_x32_timer_create, compat_sys_timer_create)

#define __NR_x32_mq_notify			__NR_X32_SYSCALL(16)
__X32_SYSCALL(__NR_x32_mq_notify, compat_sys_mq_notify)
#define __NR_x32_kexec_load			__NR_X32_SYSCALL(17)
__X32_SYSCALL(__NR_x32_kexec_load, compat_sys_kexec_load)
#define __NR_x32_waitid				__NR_X32_SYSCALL(18)
__X32_SYSCALL(__NR_x32_waitid, compat_sys_waitid)
#define __NR_x32_set_robust_list		__NR_X32_SYSCALL(19)
__X32_SYSCALL(__NR_x32_set_robust_list, compat_sys_set_robust_list)
#define __NR_x32_get_robust_list		__NR_X32_SYSCALL(20)
__X32_SYSCALL(__NR_x32_get_robust_list, compat_sys_get_robust_list)
#define __NR_x32_vmsplice			__NR_X32_SYSCALL(21)
__X32_SYSCALL(__NR_x32_vmsplice, compat_sys_vmsplice)
#define __NR_x32_move_pages			__NR_X32_SYSCALL(22)
__X32_SYSCALL(__NR_x32_move_pages, compat_sys_move_pages)
#define __NR_x32_preadv				__NR_X32_SYSCALL(23)
__SYSCALL(__NR_x32_preadv, compat_sys_preadv64)

#define __NR_x32_pwritev			__NR_X32_SYSCALL(24)
__SYSCALL(__NR_x32_pwritev, compat_sys_pwritev64)
#define __NR_x32_rt_tgsigqueueinfo		__NR_X32_SYSCALL(25)
__X32_SYSCALL(__NR_x32_rt_tgsigqueueinfo, compat_sys_rt_tgsigqueueinfo)
#define __NR_x32_recvmmsg			__NR_X32_SYSCALL(26)
__SYSCALL(__NR_x32_recvmmsg, compat_sys_recvmmsg)
#define __NR_x32_sendmmsg			__NR_X32_SYSCALL(27)
__SYSCALL(__NR_x32_sendmmsg, compat_sys_sendmmsg)

#ifndef __NO_STUBS
#define __ARCH_WANT_OLD_READDIR
#define __ARCH_WANT_OLD_STAT
#define __ARCH_WANT_SYS_ALARM
#define __ARCH_WANT_SYS_GETHOSTNAME
#define __ARCH_WANT_SYS_PAUSE
#define __ARCH_WANT_SYS_SGETMASK
#define __ARCH_WANT_SYS_SIGNAL
#define __ARCH_WANT_SYS_UTIME
#define __ARCH_WANT_SYS_WAITPID
#define __ARCH_WANT_SYS_SOCKETCALL
#define __ARCH_WANT_SYS_FADVISE64
#define __ARCH_WANT_SYS_GETPGRP
#define __ARCH_WANT_SYS_LLSEEK
#define __ARCH_WANT_SYS_NICE
#define __ARCH_WANT_SYS_OLD_GETRLIMIT
#define __ARCH_WANT_SYS_OLD_UNAME
#define __ARCH_WANT_SYS_OLDUMOUNT
#define __ARCH_WANT_SYS_SIGPENDING
#define __ARCH_WANT_SYS_SIGPROCMASK
#define __ARCH_WANT_SYS_RT_SIGACTION
#define __ARCH_WANT_SYS_RT_SIGSUSPEND
#define __ARCH_WANT_SYS_TIME
#define __ARCH_WANT_COMPAT_SYS_TIME
#endif	/* __NO_STUBS */

#ifdef __KERNEL__

#ifndef COMPILE_OFFSETS
#include <asm/asm-offsets.h>
#define NR_syscalls (__NR_syscall_max + 1)
#endif

/*
 * "Conditional" syscalls
 *
 * What we want is __attribute__((weak,alias("sys_ni_syscall"))),
 * but it doesn't work on all toolchains, so we just do it by hand
 */
#define cond_syscall(x) asm(".weak\t" #x "\n\t.set\t" #x ",sys_ni_syscall")
#endif	/* __KERNEL__ */

#endif /* _ASM_X86_UNISTD_64_H */
