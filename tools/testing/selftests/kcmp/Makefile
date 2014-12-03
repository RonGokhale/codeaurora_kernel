CC := $(CROSS_COMPILE)$(CC)
CFLAGS += -I../../../../usr/include/

TEST_STR = ./kcmp_test || echo kcmp_test: [FAIL]

all: kcmp_test

install:
ifdef INSTALL_KSFT_PATH
ifeq ($(ARCH),x86)
	install ./kcmp_test $(INSTALL_KSFT_PATH)
	@echo echo Start kcmp test .... >> $(KSELFTEST)
	@echo "$(TEST_STR)" >> $(KSELFTEST)
	@echo rm -f kcmp-test-file >> $(KSELFTEST)
	@echo echo End kcmp test .... >> $(KSELFTEST)
	@echo echo ============================== >> $(KSELFTEST)
else
	@echo Not an x86 target, unable install kcmp selftests
endif
else
	@echo Run make kselftest_install in top level source directory
endif

run_tests: all
ifeq ($(ARCH),x86)
	@$(TEST_STR)
endif

clean:
	$(RM) kcmp_test kcmp-test-file
