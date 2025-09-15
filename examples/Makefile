slow_charge_tl:
	./slow_charge_tl.out 69 0.3 ./sctl 0 450
	gprof ./slow_charge_tl.out gmon.out > analysis_tl.txt
slow_charge_rb:
	./slow_charge_rb.out 69 0.3 ./scrb/ 900
	gprof ./slow_charge_rb.out gmon.out > analysis_rb.txt
stalingrado:
	./stalingrado.out
	gprof ./stalingrado.out gmon.out > analysis_stalingrado.txt