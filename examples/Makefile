# Try to find gprof - use system gprof or create dummy output on macOS
GPROF := $(shell which gprof 2>/dev/null)
ifeq ($(GPROF),)
    GPROF_CMD = echo "gprof not available on this system, skipping profiling analysis" >
else
    GPROF_CMD = $(GPROF)
endif

slow_charge_tl:
	./slow_charge_tl.out 69 0.3 ./sctl 0 450
	$(GPROF_CMD) ./slow_charge_tl.out gmon.out > analysis_tl.txt
slow_charge_rb:
	./slow_charge_rb.out 69 0.3 ./scrb/ 900
	$(GPROF_CMD) ./slow_charge_rb.out gmon.out > analysis_rb.txt
stalingrado:
	./stalingrado.out
	$(GPROF_CMD) ./stalingrado.out gmon.out > analysis_stalingrado.txt