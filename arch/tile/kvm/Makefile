#
# Makefile for Kernel-based Virtual Machine module
#

ccflags-y := -Ivirt/kvm -Iarch/tile/kvm

kvm-y += $(addprefix ../../../virt/kvm/, kvm_main.o)

kvm-y += kvm-tile.o
kvm-y += entry.o

obj-$(CONFIG_KVM) += kvm.o
