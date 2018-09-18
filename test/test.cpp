#include <stdio.h>
#include <conio.h>

#include <irsdefs.h>
#include <timer.h>

int main()
{
	init_to_cnt();
	
	counter_t to;
	const calccnt_t time_ms = 5000;
	//calccnt_t time_cnt = time_ms*COUNTER_PER_INTERVAL/(calccnt_t(1000)*SECONDS_PER_INTERVAL);
	calccnt_t time_cnt = TIME_TO_CNT(time_ms, 1000);
	set_to_cnt(to, time_cnt);
  for (;;) {
  	if (test_to_cnt(to)) {
  		set_to_cnt(to, time_cnt);
    	printf("%lld\n", counter_get());
    }
    if (kbhit()) {
       getch();
       break;
    }
  }
  
  deinit_to_cnt();
  
  return 0;
}
