// Модуль счетчика
// Watcom DOS
// Используем встроенный счетчик процессора
// Версия 0.2
#include <counter.h>
//#include <time.h>

// Структура в которую считывается счетчик процессора
//typedef long long counter_t;

// Время калибровки в тиках
//const clock_t time_int_calibr = 18;

// Число секунд в интервале
//counter_t SECONDS_PER_INTERVAL = 1;
// Количество отсчетов в секунде
//counter_t COUNTER_PER_SECOND = 300000000;

// Чтение встроенного счетчика процессора
extern counter_t get_rdtsc(void);
#pragma aux get_rdtsc = \
        ".586"          \
        "db 0Fh, 31h"   \
        "cld"           \
        "nop"           \
        "nop"           \
        "nop"           \
        "nop"           \
        modify [eax edx];
// Инициализация счетчика
void counter_init()
{
	#ifdef NOP
	clock_t start_clock = clock();
	while (clock() - cur_clock < 1);
	counter_t start_cnt = get_rdtsc();
	while (clock() - cur_clock < time_int_calibr);
	#endif
}
// Чтение счетчика
counter_t counter_get()
{
	counter_t cnt = get_rdtsc();
	return cnt;
}
// Деинициализация счетчика
void counter_deinit()
{
}
