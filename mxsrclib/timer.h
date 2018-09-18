// Работа с таймером
// Версия 0.6
#ifndef TIMERH
#define TIMERH

#include <irsdefs.h>
#include <counter.h>

// Преобразование времени в секундах в виде отношения числителя
// _TIME_NUM_ к знаменателю _TIME_DENOM_ в масштаб счетчика
#define TIME_TO_CNT(_TIME_NUM_, _TIME_DENOM_)\
	( \
	  ((calccnt_t)(_TIME_NUM_)) * COUNTER_PER_INTERVAL /\
  	( ((calccnt_t)(_TIME_DENOM_)) * SECONDS_PER_INTERVAL )\
  )
// Преобразование из миллисекунд в количество тактов счетчика
#define MS_TO_CNT(ms) ((calccnt_t)ms * COUNTER_PER_INTERVAL / ((calccnt_t)1000 * SECONDS_PER_INTERVAL))
// Преобразование из микросекунд в количество тактов счетчика
#define MCS_TO_CNT(mcs) ((calccnt_t)mcs * COUNTER_PER_INTERVAL / ((calccnt_t)1000000 * SECONDS_PER_INTERVAL))
// Преобразование счетчика в вещественное время, с
#define CNT_TO_DBLTIME(_CNT_)\
  ( ((double)(SECONDS_PER_INTERVAL)) * ((double)(_CNT_)) / \
    ((double)(COUNTER_PER_INTERVAL)) )

// Установка таймаута t по переменной counter
void set_to_cnt(counter_t &to, counter_t t);
// Установка следующего таймаута t по переменной counter
void next_to_cnt(counter_t &to, counter_t t);
// Проверка таймаута по переменной counter
irs_bool test_to_cnt(counter_t to);
// Инициализация таймера
void init_to_cnt();
// Деинициализация таймера
void deinit_to_cnt();

#endif //TIMERH
