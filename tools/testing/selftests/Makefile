TARGETS = breakpoints
TARGETS += cpu-hotplug
TARGETS += efivarfs
TARGETS += exec
TARGETS += firmware
TARGETS += ftrace
TARGETS += kcmp
TARGETS += memfd
TARGETS += memory-hotplug
TARGETS += mount
TARGETS += mqueue
TARGETS += net
TARGETS += powerpc
TARGETS += ptrace
TARGETS += sysctl
TARGETS += timers
TARGETS += user
TARGETS += vm
#Please keep the TARGETS list alphabetically sorted

INSTALL_TARGETS = breakpoints cpu-hotplug efivarfs firmware ipc
INSTALL_TARGETS += kcmp memfd memory-hotplug mqueue mount net
INSTALL_TARGETS += ptrace sysctl timers user vm

TARGETS_HOTPLUG = cpu-hotplug
TARGETS_HOTPLUG += memory-hotplug

all:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET; \
	done;

install:
ifdef INSTALL_KSFT_PATH
	make all
	@echo #!/bin/sh\n# Kselftest Run Tests .... >> $(KSELFTEST)
	@echo # This file is generated during kselftest_install >> $(KSELFTEST)
	@echo # Please don't change it !!\n  >> $(KSELFTEST)
	@echo echo ============================== >> $(KSELFTEST)
	for TARGET in $(INSTALL_TARGETS); do \
		echo Installing $$TARGET; \
		make -C $$TARGET install; \
	done;
	chmod +x $(KSELFTEST)
else
	@echo Run make kselftest_install in top level source directory
endif

run_tests:
ifdef INSTALL_KSFT_PATH
	@cd $(INSTALL_KSFT_PATH); ./kselftest.sh; cd -
# TODO: include ftrace and powerpc in install targets
	for TARGET in ftrace powerpc; do \
		make -C $$TARGET run_tests; \
	done;
else
	make all
	for TARGET in $(TARGETS); do \
		make -C $$TARGET run_tests; \
	done;
endif

hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET; \
	done;

run_hotplug: hotplug
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET run_full_test; \
	done;

clean_hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET clean; \
	done;

clean:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET clean; \
	done;
