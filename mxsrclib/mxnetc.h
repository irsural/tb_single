// Протокол MxNetC (Max Network Client)
// Версия 0.3
//---------------------------------------------------------------------------
#ifndef mxnetcH
#define mxnetcH

#include <irsdefs.h>
#include <mxnetda.h>
#include <mxifa.h>
#include <timer.h>
//---------------------------------------------------------------------------
// Размер идентификатора начала пакета
#define bpa_size 8
//---------------------------------------------------------------------------
typedef enum _mxnc_status_t {
  mxnc_status_success = 0,
  mxnc_status_busy = 1,
  mxnc_status_error = 2
} mxnc_status_t;
typedef enum _mxnc_mode_t {
  mxnc_mode_free,
  mxnc_mode_packet,
  mxnc_mode_write,
  mxnc_mode_write_wait,
  mxnc_mode_read,
  mxnc_mode_read_wait,
  mxnc_mode_begin_packet,
  mxnc_mode_chunk_read_wait,
  mxnc_mode_checksum,
  mxnc_mode_proc,
  mxnc_mode_reset,

  mxnc_mode_broadcast,
  mxnc_mode_broadcast_wait,
  mxnc_mode_broadcast_vars,
  mxnc_mode_broadcast_vars_wait
} mxnc_mode_t;
typedef enum _mxnc_error_t {
  mxnc_err_none = 0,
  mxnc_err_invalid_param = 1,
  mxnc_err_nomem = 2,
  mxnc_err_create = 3,
  mxnc_err_abort = 4,
  mxnc_err_busy = 5,
  mxnc_err_bad_server = 6,
  mxnc_err_remote_invalid_param = 7,
  mxnc_err_timeout = 8
} mxnc_error_t;
//---------------------------------------------------------------------------
// Класс для
class mxnetc
{
  // Буфер для приема/отправки пакета
  mxn_packet_t *f_packet;
  // Размер буфера, байт
  mxn_sz_t f_packet_size;
  // Размер принимаемого пакета, байт
  mxn_sz_t f_receive_size;
  // Размер отправляемого пакета, байт
  mxn_sz_t f_send_size;
  // Текущий режим
  mxnc_mode_t f_mode;
  // Режим после возврата из подрежима
  //mxnc_mode_t f_mode_return;
  // Текущая команда
  mxn_cnt_t f_command;
  // Текущий статус операции
  mxnc_status_t f_status;
  // Статус операции передаваемый в mxnc_mode_reset
  mxnc_status_t f_local_status;
  // Указатель на переменную для версии
  irs_u16 *f_version;
  // Дескриптор канала
  void *f_handle_channel;
  // Ошибка создания класса
  irs_bool f_create_error;
  // Запрос прерывания операции
  irs_bool f_abort_request;
  // Позиция в пакете для поиска начала пакета
  //mxn_sz_t f_packet_pos;
  // Идентификатор начала пакета
  irs_u8 beg_pack_array[bpa_size];
  // Ожидание чтения для поиска начала пакета
  //irs_bool f_fbpc_wait_read;//!!!!!!!!
  // Признак выхода из fbpc по таймауту
  //irs_bool f_fbpc_end_to;//!!!!!!!!
  // Последняя ошибка
  mxnc_error_t f_last_error;
  // Указатель на ползователскую переменную количества переменных
  mxn_cnt_t *f_user_size;
  // Первая переменная для чтения/записи
  mxn_cnt_t f_user_index;
  // Количество переменных для чтения/записи
  mxn_cnt_t f_user_count;
  // Широковещательный режим заданный пользователем
  irs_bool f_user_broadcast; 
  // Указатель на пользовательские переменные
  irs_i32 *f_user_vars;
  // Переменная таймаута сетевых операций
  counter_t f_net_to;
  // Буфер для приема широковещательного пакета
  mxn_packet_t *f_broadcast_packet;
  // Размер буфера для приема широковещательного пакета
  mxn_sz_t f_broadcast_packet_size;
  // Число переменных полученных при широковещательной рассылке
  mxn_cnt_t f_broadcast_count;
  // Переменные полученые при широковещательной рассылке
  irs_i32 *f_broadcast_vars;
  // Широковещательный режим модуля
  irs_bool f_broadcast_mode;
  // Переменная таймаута сетевых операций
  counter_t f_oper_to;
  // Переменная времени сетевых операций
  counter_t f_oper_t;

  // Запрещаем конструктор по умолчанию
  mxnetc::mxnetc();
  // Поиск начала в пакете
  irs_bool mxnetc::find_begin_in_data(irs_u8 *buf, mxn_sz_t size,
    mxn_sz_t &pos);
  // Проверка контрольной суммы
  irs_bool mxnetc::checksum_valid(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size);
  // Расчет контрольной суммы
  irs_i32 mxnetc::checksum_calc(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size);
  // Запрещение обработки
  irs_bool mxnetc::deny_proc();
  // Заполнение пакета
  irs_bool mxnetc::packet_fill(mxn_cnt_t code_comm, mxn_cnt_t packet_var_first,
    mxn_cnt_t packet_var_count, irs_i32 *vars, mxn_cnt_t var_count);
  // Перераспределение памяти с сохранением данных
  void *mxnetc::renew(void *pointer, mxn_sz_t old_size, mxn_sz_t new_size);
public:
  // Конструктор
  mxnetc::mxnetc(mxifa_ch_t channel);
  // Деструктор
  mxnetc::~mxnetc();
  // Чтение версии протокола mxnet
  void mxnetc::get_version(irs_u16 *version);
  // Чтение размера массива (количество переменных)
  void mxnetc::get_size(mxn_cnt_t *size);
  // Чтение переменных
  void mxnetc::read(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars);
  // Запись переменных
  void mxnetc::write(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars);
  // Включение/выключение широковещательного режима
  void mxnetc::set_broadcast(irs_bool broadcast_mode);
  // Статус текущей операции
  mxnc_status_t mxnetc::status();
  // Элементарное действие
  void mxnetc::tick();
  // Прерывание операции
  void mxnetc::abort();
  // Чтение последней ошибки
  mxnc_error_t mxnetc::get_last_error();
  // Установка таймаута операций, с
  void mxnetc::set_timeout(calccnt_t t_num, calccnt_t t_denom);
  // Установка ip-адреса удаленного устройства
  void mxnetc::set_dest_ip(mxip_t ip);
  // Установка порта удаленного устройства
  void mxnetc::set_dest_port(irs_u16 port);
  // Установка локального порта
  void mxnetc::set_local_port(irs_u16 port);
};
//---------------------------------------------------------------------------
#endif
