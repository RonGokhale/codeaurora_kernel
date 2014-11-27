TEST_STR = ./posix_timers

all:
	gcc posix_timers.c -o posix_timers -lrt

install:
ifdef INSTALL_KSFT_PATH
	install ./posix_timers $(INSTALL_KSFT_PATH)
	@echo echo Start timers test .... >> $(KSELFTEST)
	@echo "$(TEST_STR)" >> $(KSELFTEST)
	@echo echo End timers test .... >> $(KSELFTEST)
	@echo echo ============================== >> $(KSELFTEST)
else
	@echo Run make kselftest_install in top level source directory
endif

run_tests: all
	@$(TEST_STR)

clean:
	rm -f ./posix_timers
