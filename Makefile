stm32f2xx.idc: addr
	./addr > stm32f2xx.idc

addr: addr.c stm32f2xx.h core_cm3.h
	gcc -o addr addr.c

