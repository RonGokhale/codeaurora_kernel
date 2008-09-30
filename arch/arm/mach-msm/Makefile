obj-y += io.o idle.o irq.o timer.o dma.o clock.o clock-7x00a.o memory.o
obj-y += vreg.o
obj-y += gpio.o generic_gpio.o
obj-y += proc_comm.o

# Common code for board init
obj-y += common.o nand_partitions.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-keypad.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o
obj-$(CONFIG_MACH_TROUT) += board-trout-gpio.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_MSM_PERF) += perf.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
