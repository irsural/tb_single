// Объявления типов для измерительных приборов
// Версия 0.1

#ifndef MESDEFH
#define MESDEFH
// Тип для статуса
typedef enum _mes_status_t {
  mes_status_success = 0,
  mes_status_busy = 1,
  mes_status_error = 2
} mes_status_t;
#endif //MESDEFH

