// Работа с таймером
// Версия 0.6
#include <timer.h>
#include <irsdefs.h>

// Установка таймаута t по переменной counter
void set_to_cnt(counter_t &to, counter_t t)
{
  to = counter_get() + t;
}
// Установка следующего таймаута t по переменной counter
void next_to_cnt(counter_t &to, counter_t t)
{
  to += t;
}
// Проверка таймаута по переменной counter
irs_bool test_to_cnt(counter_t to)
{
  irs_bool res = irs_false;
  if (counter_get() - to > 0) res = irs_true;
  return res;
}
// Инициализация таймера
void init_to_cnt()
{
  counter_init();
}
// Деинициализация таймера
void deinit_to_cnt()
{
  counter_deinit();
}
