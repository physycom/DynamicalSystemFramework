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
	for i in 1.0 1.1 1.2 1.3 1.4 1.5 1.6 1.7; do \
		for j in 0.2 0.4 0.6 0.8 1.0 1.2 ; do \
			./stalingrado_tl.out $$i $$j; \
			./stalingrado_notl.out $$i $$j; \
		done \
	done