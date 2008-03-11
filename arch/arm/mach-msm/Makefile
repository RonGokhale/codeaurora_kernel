obj-y += io.o idle.o irq.o timer.o dma.o clock.o clock-7x00a.o
obj-y += vreg.o
obj-y += gpio.o generic_gpio.o

# Common code for board init
obj-y += common.o nand_partitions.o

obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o board-halibut-keypad.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_servers.o
obj-$(CONFIG_MSM_ADSP) += adsp.o
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_MSM_PERF) += perf.o
obj-$(CONFIG_PM) += pm.o
