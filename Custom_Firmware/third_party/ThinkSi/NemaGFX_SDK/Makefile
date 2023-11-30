.PHONY: all

CPPCHECK?=cppcheck

%:
	@$(MAKE) -f Makefile.nemagfx $@
	@$(MAKE) -f Makefile.nemavg $@
	@$(MAKE) -f Makefile.nemadc $@

all:
	@$(MAKE) -f Makefile.nemagfx
	@$(MAKE) -f Makefile.nemavg
	@$(MAKE) -f Makefile.nemadc

cppcheck:
	$(CPPCHECK) -iPrivate --quiet --enable=all --error-exitcode=-1 --inconclusive --std=c99 --language=c --suppressions-list=suppressions.txt .
