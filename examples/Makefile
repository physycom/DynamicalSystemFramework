slow_charge_tl:
	./slow_charge_tl.out 69 0.3 ./sctl 0 450
slow_charge_rb:
	./slow_charge_rb.out 69 0.3 ./scrb/ 900
	gprof ./slow_charge_rb.out gmon.out > analysis_rb.txt
stalingrado:
	./stalingrado.out