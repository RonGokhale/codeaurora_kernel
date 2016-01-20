# Choose to use clang as your compiler
HOSTCC		:= clang
CC		:= clang

CPP		:= $(CC) -E

# Cross compiling variables
#ARCH		:= arm
#CROSS_COMPILE	:= arm-linux-gnueabihf-

ifdef ARCH
export ARCH
endif
ifdef CROSS_COMPILE
export CROSS_COMPILE
endif
ifdef CC
export CC
endif
ifdef CPP
export CPP
endif
ifdef HOSTCC
export HOSTCC
endif
ifdef O
export O
endif

# This allows you to compile out of tree depending on ARCH and CC

# Remove spaces from CC
#empty		:=
#space		:= $(empty) $(empty)
#CC_NOSPACES	:= $(subst $(space),_,$(CC))

# Where to put build output
#ARCH		?= x86
#O		:= build/$(ARCH)-$(CC_NOSPACES)

LINUXFLAGS	= CPP="$(CPP)" CC="$(CC)" HOSTCC="$(HOSTCC)"
linuxmake	= $(MAKE) -f Makefile $(LINUXFLAGS) O=$(1) $(2)

%:
	@$(call linuxmake,$(O),$@)

%/: force
	@$(call linuxmake,$(O),$@)

%.o: %.c
	@$(call linuxmake,$(O),$@)

%.i: %.c
	@$(call linuxmake,$(O),$@)

%.s: %.c
	@$(call linuxmake,$(O),$@)

%.s: %.S
	@$(call linuxmake,$(O),$@)

all:
	@$(call linuxmake,$(O),$@)

.PHONY: force
force: ;
	:
