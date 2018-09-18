// Абстакция интерфейса для каналов обмена (интерфейсов)
// Max Interface Abstraction
// Версия 0.4
#ifndef MXIFAH
#define MXIFAH

#include <irsdefs.h>
#include <mxifal.h>
#include <mxifar.h>

// Версия (<старший байт>.<младший байт>)
#define MXIFA_VERSION 0x0003

// Номера каналов закрепленных за программами или модулями
// mxnet
#define MXIFA_MXNET 1
// mxnetc
#define MXIFA_MXNETC 2   
                         
// Тип канала
typedef enum _mxifa_ei_t {
  // Неизвестен
  mxifa_ei_unknown,
  // UDP/IP Димы Уржумцева
  mxifa_ei_avr128_ether,
  // Win32 TCP/IP
  mxifa_ei_win32_tcp_ip
} mxifa_ei_t;

// Структура для описания удаленной системы
typedef struct _mxifa_avr128_ether_dest_t {
  // Удаленный IP
  mxip_t ip;
  // Удаленный порт
  irs_u16 port;
} mxifa_avr128_ether_dest_t;

// Структура для описания удаленной системы
typedef struct _mxifa_win32_tcp_ip_dest_t {
  // Удаленный IP
  mxip_t ip;
  // Удаленный порт
  irs_u16 port;
} mxifa_win32_tcp_ip_dest_t;

// Обобщенная структура для описания удаленной системы
typedef union _mxifa_dest_t {
  // Структура для описания удаленной системы канала типа mxifa_ei_avr128_ether
  mxifa_avr128_ether_dest_t avr128_ether;
  // Структура для описания удаленной системы канала типа mxifa_ei_win32_tcp_ip
  mxifa_win32_tcp_ip_dest_t win32_tcp_ip;
} mxifa_dest_t;

// Структура для установки конфигурации avr128_ether в функции mxifa_set_config
typedef struct _mxifa_avr128_cfg {
  // Локальный IP
  mxip_t *ip;
  // Маска подсети
  mxip_t *mask;
} mxifa_avr128_cfg;

// Структура для установки конфигурации win32_tcp_ip в функции mxifa_set_config
typedef struct _mxifa_win32_tcp_ip_cfg {
  // Удаленный IP
  mxip_t dest_ip;
  // Удаленный порт
  irs_u16 dest_port;
  // Локальный порт
  irs_u16 local_port;
} mxifa_win32_tcp_ip_cfg;

// Инициализация mxifa
void mxifa_init();
// Деинициализация mxifa
void mxifa_deinit();
// Элементарное действие
void mxifa_tick();
// Блокирующее открытие channel
// channel - номер канала, занимается в специальной таблице для каждой программы
// is_broadcast - открытие канала в широковещательном режиме
// Возврат - указатель на данные канала
void *mxifa_open(mxifa_ch_t channel, irs_bool is_broadcast);
// Неблокирующее открытие channel
// channel - номер канала, занимается в специальной таблице для каждой программы
// is_broadcast - открытие канала в широковещательном режиме
// Возврат - указатель на данные канала
void *mxifa_open_begin(mxifa_ch_t channel, irs_bool is_broadcast);
// Проверка окончания открытия channel
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// abort - немедленное завершение с отменой операции
// Возврат - операция окончена
irs_bool mxifa_open_end(void *pchdata, bool abort);
// Блокирующее закрытие channel
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// Возврат - операция окончена
irs_bool mxifa_close(void *pchdata);
// Неблокирующее закрытие channel
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// Возврат - операция окончена
irs_bool mxifa_close_begin(void *pchdata);
// Проверка окончания закрытия channel
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// abort - немедленное завершение с отменой операции
// Возврат - операция окончена
irs_bool mxifa_close_end(void *pchdata, bool abort);
// Запись буфера в интерфейс для передачи
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// dest - приемник данных, если IRS_NULL, то используется приемник канала
//   по умолчанию
// buf - укзатель на данные для передачи
// size - размер данных в байтах
// Возврат - операция окончена
irs_bool mxifa_write_begin(void *pchdata, mxifa_dest_t *dest, 
  irs_u8 *buf, mxifa_sz_t size);
// Проверка окончания записи
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// abort - немедленное завершение с отменой операции
// Возврат - операция окончена
irs_bool mxifa_write_end(void *pchdata, irs_bool abort);
// Чтение принятых данных из интерфейса в буфер
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// dest - источник данных, если IRS_NULL, то используется источник канала
//   по умолчанию
// buf - укзатель на буфер для приема
// size - размер буфера в байтах
// Возврат - операция окончена
irs_bool mxifa_read_begin(void *pchdata, mxifa_dest_t *dest,
  irs_u8 *buf, mxifa_sz_t size);
// Проверка окончания чтения
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// abort - немедленное завершение с отменой операции
// Возврат - операция окончена
irs_bool mxifa_read_end(void *pchdata, irs_bool abort);
// Чтение типа канала
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// Возврат - тип канала
mxifa_ei_t mxifa_get_channel_type(void *pchdata);
// Установка конфигурации канала
// pchdata - указатель на данные канала, возвращаемые mxifa_open или 
//   mxifa_open_begin
// config - указатель на структуру ..._cfg соответствующую данному типу канала
void mxifa_set_config(void *pchdata, void *config);
// Чтение конфигурации канала
// config - указатель на структуру ..._cfg соответствующую данному типу канала
void mxifa_get_config(void *pchdata, void *config);

#endif //MXIFAH
