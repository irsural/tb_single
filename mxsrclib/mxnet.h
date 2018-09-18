// Протокол MxNet (Max Network)
// Версия 0.5
#ifndef MXNETH
#define MXNETH

#include <irsdefs.h>
#include <mxifa.h>
#include <mxnetda.h>
#include <timer.h>

// Версия (<старший байт>.<младший байт>)
#define MXN_VERSION 0x0003

// Предварительное объявление данных для протокола mxnet
struct _mxn_data_t;
typedef _mxn_data_t mxn_data_t;
// Тип для элементарной функции
typedef void (*mxn_tick_f)(mxn_data_t &data);
// Данные для протокола mxnet
struct _mxn_data_t {
  // Количество переменных
  mxn_cnt_t count;
  // Внешний массив переменных
  irs_i32 *vars_ext;
  // Внутренний массив переменных
  mxn_packet_t *packet;
  // Массив помечающий переменные только для чтения
  irs_bool *read_only;
  // Работа в режиме широковещания
  irs_bool is_broadcast;
  // Переменная таймаута широковещательных рассылок
  counter_t to_broadcast;
  // Изменение режима широковещания на новый
  irs_bool is_broadcast_new;
  // Канал по которому работает протокол
  mxifa_ch_t channel;
  // Данные канала
  void *data_ch;
  // Данные об удаленном клиенте
  mxifa_dest_t dest_info;
  // Количество переменных в принятом пакете
  mxn_cnt_t var_cnt_packet;
  // Следующая элементарная функция
  mxn_tick_f mxn_tick_next;
  // Элементарная функция
  mxn_tick_f mxn_tick;
  // Количество отсылаемых байт
  mxn_sz_t count_send;
  // Широковещательная передача
  irs_bool broadcast_send;
  // Ошибка в аргументах записи
  irs_bool write_error;
};

// Инициализация протокола
void mxn_init(mxn_data_t &data, mxifa_ch_t channel, irs_i32 *vars,
  mxn_cnt_t count);
// Деинициализация протокола
void mxn_deinit(mxn_data_t &data);
// Установка только для чтение переменная или нет
void mxn_set_read_only(mxn_data_t &data, mxn_cnt_t ind_var, irs_bool read_only);
// Установка нового режима широковещания
void mxn_set_broadcast(mxn_data_t &data, irs_bool is_broadcast);
// Элементарная функция
void mxn_tick(mxn_data_t &data);
// Чтение указателя на дескриптор канала
//void *mxn_get_data_channel(mxn_data_t &data);
// Установка локального IP
void mxn_set_ip(mxn_data_t &data, mxip_t ip);
// Чтение локального IP
mxip_t mxn_get_ip(mxn_data_t &data);
// Установка локальной маски
void mxn_set_mask(mxn_data_t &data, mxip_t mask);
// Чтение локальной маски
mxip_t mxn_get_mask(mxn_data_t &data);

#endif //MXNETH
