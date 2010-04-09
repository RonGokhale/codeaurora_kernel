#
# Makefile for Chromeos Supplied Third-Party device drivers
#

obj-$(CONFIG_ATH6K_LEGACY)	+= ath6kl/
obj-$(CONFIG_MWIFIEX)	+= mwifiex/
obj-$(CONFIG_TCG_TPM)	+= tpmd_dev/

# This is a stupid trick to get kbuild to create chromeos/built-in.o
obj- += foo.o
