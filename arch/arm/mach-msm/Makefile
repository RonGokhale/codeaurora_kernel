obj-y += io.o irq.o timer.o dma.o memory.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += clock.o clock-pcom.o
obj-y += gpio.o generic_gpio.o
obj-y += remote_spinlock.o
obj-y += pmic.o

obj-$(CONFIG_USB_FS_HOST) += rpc_fsusb.o
obj-$(CONFIG_USB_EHCI_MSM) += rpc_hsusb.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o
obj-$(CONFIG_MACH_QSD8X50_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_FFA) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_COMET) += board-qsd8x50.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_CPU_V6) += idle-v6.o
obj-$(CONFIG_CPU_V7) += idle-v7.o
obj-$(CONFIG_MSM_JTAG_V7) += jtag-v7.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o

ifdef CONFIG_MSM_N_WAY_SMSM
	obj-$(CONFIG_PM) += pm2.o
else
	obj-$(CONFIG_PM) += pm.o
endif

obj-$(CONFIG_MSM_SMD) += smd.o
obj-$(CONFIG_MSM_SMEM_LOG) += smem_log.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_clients.o

obj-$(CONFIG_DEBUG_FS) += pmic_debugfs.o

obj-$(CONFIG_MSM_RPCSERVER_TIME_REMOTE) += rpc_server_time_remote.o

obj-y += pcom_stub.o
