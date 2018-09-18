// Протокол MxNetC (Max Network Client)
// Версия 0.3
//---------------------------------------------------------------------------
//#include <vcl.h>
//#pragma hdrstop

#include <mxnetc.h>
#include <mxnetd.h>
#include <string.h>

//---------------------------------------------------------------------------
// Нет команды
#define MXN_COM_NONE          ((mxn_cnt_t)-1)
// Таймаут сетевых операций
#define net_t TIME_TO_CNT(1, 2)
//---------------------------------------------------------------------------
// Конструктор
mxnetc::mxnetc(mxifa_ch_t channel):
  f_packet(IRS_NULL),
  f_packet_size(0),
  f_receive_size(0),
  f_send_size(0),
  f_mode(mxnc_mode_free),
  //f_mode_return(mxnc_mode_free),
  f_command(MXN_COM_NONE),
  f_status(mxnc_status_success),
  f_local_status(mxnc_status_success),
  f_version(IRS_NULL),
  f_handle_channel(IRS_NULL),
  f_create_error(irs_false),
  f_abort_request(irs_false),
  f_last_error(mxnc_err_none),
  f_user_size(IRS_NULL),
  f_user_index(0),
  f_user_count(0),
  f_user_broadcast(irs_false),
  f_user_vars(IRS_NULL),
  f_net_to(0),
  f_broadcast_packet(IRS_NULL),
  f_broadcast_packet_size(0),
  f_broadcast_count(0),
  f_broadcast_vars(IRS_NULL),
  f_broadcast_mode(irs_false),
  f_oper_to(0),
  f_oper_t(0)
{
  init_to_cnt();
  mxifa_init();
  f_handle_channel = mxifa_open(MXIFA_MXNETC, irs_false);
  if (!f_handle_channel) {
    f_create_error = irs_true;
    return;
  }
  LODWORD(beg_pack_array[0]) = MXN_CONST_IDENT_BEG_PACK_FIRST;
  HIDWORD(beg_pack_array[0]) = MXN_CONST_IDENT_BEG_PACK_SECOND;
}
//---------------------------------------------------------------------------
// Деструктор
mxnetc::~mxnetc()
{
  if (f_handle_channel) {
    //mxifa_close(f_handle_channel);
    mxifa_close_begin(f_handle_channel);
    mxifa_close_end(f_handle_channel, irs_true);
  }
  mxifa_deinit();
  deinit_to_cnt();
  if (f_packet) {
    delete f_packet;
    f_packet = IRS_NULL;
  }
  if (f_broadcast_packet) {
    delete f_broadcast_packet;
    f_broadcast_packet = IRS_NULL;
  }
  if (f_broadcast_vars) {
    delete f_broadcast_vars;
    f_broadcast_vars = IRS_NULL;
  }
}
//---------------------------------------------------------------------------
// Чтение версии протокола mxnet
void mxnetc::get_version(irs_u16 *version)
{
  if (deny_proc()) return;
  if (!version) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return;
  }

  f_version = version;
  f_command = MXN_GET_VERSION;
  f_status = mxnc_status_busy;
}
//---------------------------------------------------------------------------
// Чтение размера массива (количество переменных)
void mxnetc::get_size(mxn_cnt_t *size)
{
  if (deny_proc()) return;
  if (!size) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return;
  }

  f_user_size = size;
  f_command = MXN_READ_COUNT;
  f_status = mxnc_status_busy;
}
//---------------------------------------------------------------------------
// Чтение переменных
void mxnetc::read(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars)
{
  if (deny_proc()) return;
  irs_bool index_over_max = (count > MXN_CNT_MAX - index);
  irs_bool vars_invalid = !vars;
  irs_bool count_zero = !count;
  if (index_over_max || vars_invalid || count_zero) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return;
  }

  f_user_index = index;
  f_user_count = count;
  f_user_vars = vars;
  f_command = MXN_READ;
  f_status = mxnc_status_busy;
}
//---------------------------------------------------------------------------
// Запись переменных
void mxnetc::write(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars)
{
  if (deny_proc()) return;
  irs_bool index_over_max = (count > MXN_CNT_MAX - index);
  irs_bool vars_invalid = !vars;
  irs_bool count_zero = !count;
  if (index_over_max || vars_invalid || count_zero) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return;
  }

  f_user_index = index;
  f_user_count = count;
  f_user_vars = vars;
  f_command = MXN_WRITE;
  f_status = mxnc_status_busy;
}
//---------------------------------------------------------------------------
// Включение/выключение широковещательного режима
void mxnetc::set_broadcast(irs_bool broadcast_mode)
{
  if (deny_proc()) return;
  if ((broadcast_mode != irs_true) && (broadcast_mode != irs_false)) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return;
  }

  f_user_broadcast = broadcast_mode;
  f_command = MXN_SET_BROADCAST;
  f_status = mxnc_status_busy;
}
//---------------------------------------------------------------------------
// Статус текущей операции
mxnc_status_t mxnetc::status()
{
  if (f_create_error) {
    f_last_error = mxnc_err_create;
    return mxnc_status_error;
  } else {
    return f_status;
  }
}
//---------------------------------------------------------------------------
// Элементарное действие
void mxnetc::tick()
{
  mxifa_tick();
  switch (f_mode) {
    case mxnc_mode_free: {
      f_mode = mxnc_mode_broadcast;
    } break;
    case mxnc_mode_packet: {
      mxn_cnt_t index = 0;
      mxn_cnt_t count = 0;
      irs_bool command_performed = irs_false;
      switch (f_command) {
        case MXN_GET_VERSION: {
          index = 0;
          count = 0;
        } break;
        case MXN_READ_COUNT: {
          if (f_broadcast_mode) {
            *f_user_size = f_broadcast_count;
            command_performed = irs_true;
            f_mode = mxnc_mode_reset;
            f_local_status = mxnc_status_success;
          } else {
            index = 0;
            count = 0;
          }
        } break;
        case MXN_READ: {
          if (f_broadcast_mode) {
            if (f_user_index + f_user_count <= f_broadcast_count) {
              memcpy(f_user_vars, (void *)(f_broadcast_vars + f_user_index),
                f_user_count*sizeof(irs_i32));
              f_local_status = mxnc_status_success;
            } else {
              f_last_error = mxnc_err_invalid_param;
              f_local_status = mxnc_status_error;
            }
            f_mode = mxnc_mode_reset;
            command_performed = irs_true;
          } else {
            index = f_user_index;
            count = f_user_count;
          }
        } break;
        case MXN_WRITE: {
          index = f_user_index;
          count = f_user_count;
        } break;
        case MXN_SET_BROADCAST: {
          index = f_user_broadcast;
          count = 0;
        } break;
      }
      if (!command_performed) {
        irs_bool fill_success = packet_fill(f_command, index, count,
          f_user_vars, count);
        if (fill_success) {
          mxifa_write_begin(f_handle_channel, IRS_NULL, (irs_u8 *)f_packet,
            f_send_size);
          f_mode = mxnc_mode_write;
        } else {
          f_mode = mxnc_mode_reset;
          f_last_error = mxnc_err_nomem;
          f_local_status = mxnc_status_error;
        }
      }
    } break;
    case mxnc_mode_write: {
      mxifa_write_begin(f_handle_channel, IRS_NULL, (irs_u8 *)f_packet,
        f_send_size);
      if (f_oper_t) {
        set_to_cnt(f_oper_to, f_oper_t);
      }
      f_mode = mxnc_mode_write_wait;
    } break;
    case mxnc_mode_write_wait: {
      if (mxifa_write_end(f_handle_channel, irs_false)) {
        f_mode = mxnc_mode_read;
      } else if (f_abort_request) {
        mxifa_write_end(f_handle_channel, irs_true);
        f_mode = mxnc_mode_reset;
        f_last_error = mxnc_err_abort;
        f_local_status = mxnc_status_error;
      } else if (f_oper_t) {
        if (test_to_cnt(f_oper_to)) {
          mxifa_write_end(f_handle_channel, irs_true);
          f_mode = mxnc_mode_reset;
          f_last_error = mxnc_err_timeout;
          f_local_status = mxnc_status_error;
        }
      }
    } break;
    case mxnc_mode_read: {
      mxifa_read_begin(f_handle_channel, IRS_NULL, (irs_u8 *)f_packet,
        f_receive_size);
      set_to_cnt(f_net_to, net_t);
      f_mode = mxnc_mode_read_wait;
    } break;
    case mxnc_mode_read_wait: {
      if (mxifa_read_end(f_handle_channel, irs_false)) {
        f_mode = mxnc_mode_begin_packet;
      } else if (f_abort_request) {
        mxifa_read_end(f_handle_channel, irs_true);
        f_mode = mxnc_mode_reset;
        f_last_error = mxnc_err_abort;
        f_local_status = mxnc_status_error;
      } else if (test_to_cnt(f_net_to)) {
        mxifa_read_end(f_handle_channel, irs_true);
        f_mode = mxnc_mode_packet;
      } else if (f_oper_t) {
        if (test_to_cnt(f_oper_to)) {
          mxifa_read_end(f_handle_channel, irs_true);
          f_mode = mxnc_mode_reset;
          f_last_error = mxnc_err_timeout;
          f_local_status = mxnc_status_error;
        }
      }
    } break;
    case mxnc_mode_begin_packet: {
      mxn_sz_t pos = 0;
      void *src = IRS_NULL;
      size_t move_size = 0;
      irs_bool move_and_read = irs_false;
      if (find_begin_in_data((irs_u8 *)f_packet, f_receive_size, pos)) {
        if (pos) {
          src = (void *)(((irs_u8 *)f_packet) + pos);
          move_size = f_receive_size - pos;

          move_and_read = irs_true;
          f_mode = mxnc_mode_chunk_read_wait;
        } else {
          move_and_read = irs_false;
          f_mode = mxnc_mode_checksum;
        }
      } else {
        src = (void *)(((irs_u8 *)f_packet) +
          (f_receive_size - SIZE_OF_BEG_PACK));
        move_size = SIZE_OF_BEG_PACK;

        move_and_read = irs_true;
        f_mode = mxnc_mode_read_wait;
      }
      if (move_and_read) {
        void *dest = (void *)f_packet;
        memmove(dest, src, move_size);

        irs_u8 *buf = ((irs_u8 *)f_packet) + move_size;
        mxn_sz_t size = f_receive_size - move_size;
        mxifa_read_begin(f_handle_channel, IRS_NULL, buf, size);
      }
    } break;
    case mxnc_mode_chunk_read_wait: {
      if (mxifa_read_end(f_handle_channel, irs_false)) {
          f_mode = mxnc_mode_checksum;
      } else if (f_abort_request) {
        mxifa_read_end(f_handle_channel, irs_true);
        f_mode = mxnc_mode_reset;
        f_last_error = mxnc_err_abort;
        f_local_status = mxnc_status_error;
      } else if (f_oper_t) {
        if (test_to_cnt(f_oper_to)) {
          mxifa_read_end(f_handle_channel, irs_true);
          f_mode = mxnc_mode_reset;
          f_last_error = mxnc_err_timeout;
          f_local_status = mxnc_status_error;
        }
      }
    } break;
    case mxnc_mode_checksum: {
      mxn_cnt_t var_count = 0;
      if (f_packet->code_comm == MXN_READ) {
        var_count = f_packet->var_count;
      } else {
        var_count = 0;
      }
      mxn_sz_t cur_packet_size =
        (SIZE_OF_HEADER + var_count + 1)*sizeof(irs_i32);
      irs_bool packet_size_ok = (cur_packet_size == f_receive_size);
      irs_bool checksum_ok = checksum_valid(f_packet, var_count,
        f_receive_size);
      if (packet_size_ok && checksum_ok) {
        f_mode = mxnc_mode_proc;
      } else {
        f_mode = mxnc_mode_read;
      }
    } break;
    case mxnc_mode_proc: {
      if (f_packet->code_comm == f_command) {
        switch (f_command) {
          case MXN_GET_VERSION: {
            if (f_version) {
              *f_version = LOWORD(f_packet->var_ind_first);
              f_mode = mxnc_mode_reset;
              f_local_status = mxnc_status_success;
            } else {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_invalid_param;
              f_local_status = mxnc_status_error;
            }
          } break;
          case MXN_READ_COUNT: {
            if (f_user_size) {
              *f_user_size = f_packet->var_count;
              f_mode = mxnc_mode_reset;
              f_local_status = mxnc_status_success;
            } else {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_invalid_param;
              f_local_status = mxnc_status_error;
            }
          } break;
          case MXN_READ: {
            irs_bool user_vars_valid = (f_user_vars != IRS_NULL);
            irs_bool index_valid = (f_user_index == f_packet->var_ind_first);
            irs_bool count_valid = (f_user_count == f_packet->var_count);
            if (user_vars_valid && index_valid && count_valid) {
              f_mode = mxnc_mode_reset;
              f_local_status = mxnc_status_success;
              memcpy((void *)f_user_vars, (void *)f_packet->var,
                f_user_count*sizeof(irs_i32));
            } else if (!user_vars_valid) {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_invalid_param;
              f_local_status = mxnc_status_error;
            } else {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_bad_server;
              f_local_status = mxnc_status_error;
            }
          } break;
          case MXN_WRITE: {
            irs_bool index_valid = (0 == f_packet->var_ind_first);
            irs_bool par_invalid = (1 == f_packet->var_ind_first);
            irs_bool count_valid = (0 == f_packet->var_count);
            if (index_valid && count_valid) {
              f_mode = mxnc_mode_reset;
              f_local_status = mxnc_status_success;
            } else if (par_invalid) {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_remote_invalid_param;
              f_local_status = mxnc_status_error;
            } else {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_bad_server;
              f_local_status = mxnc_status_error;
            }
          } break;
          case MXN_SET_BROADCAST: {
            irs_bool index_valid =
              (f_user_broadcast == (irs_bool)f_packet->var_ind_first);
            irs_bool count_valid = (0 == f_packet->var_count);
            if (index_valid && count_valid) {
              f_mode = mxnc_mode_reset;
              f_local_status = mxnc_status_success;
            } else {
              f_mode = mxnc_mode_reset;
              f_last_error = mxnc_err_bad_server;
              f_local_status = mxnc_status_error;
            }
          } break;
        }
      } else {
        f_mode = mxnc_mode_packet;
      }
    } break;
    case mxnc_mode_reset: {
      f_abort_request = irs_false;
      f_version = IRS_NULL;
      f_user_size = IRS_NULL;
      f_user_index = 0;                 
      f_user_count = 0;
      f_user_vars = IRS_NULL;
      f_user_broadcast = irs_false;
      f_command = MXN_COM_NONE;
      f_mode = mxnc_mode_free;
      f_status = f_local_status;
    } break;






    
    case mxnc_mode_broadcast: {
      if (!f_broadcast_packet) {
        f_broadcast_packet_size = SIZE_OF_HEADER*sizeof(irs_i32);
        f_broadcast_packet =
          (mxn_packet_t *)new irs_u8[f_broadcast_packet_size];
      }
      mxifa_read_begin(f_handle_channel, IRS_NULL, (irs_u8 *)f_broadcast_packet,
        f_broadcast_packet_size);
      set_to_cnt(f_net_to, net_t);
      f_mode = mxnc_mode_broadcast_wait;
    } break;
    case mxnc_mode_broadcast_wait: {
      if (f_command != MXN_COM_NONE) {
        f_mode = mxnc_mode_packet;
        f_status = mxnc_status_busy;
        mxifa_read_end(f_handle_channel, irs_true);
      } else {
        if (mxifa_read_end(f_handle_channel, irs_false)) {
          f_mode = mxnc_mode_broadcast_vars;
        } else if (test_to_cnt(f_net_to)) {
          f_broadcast_mode = irs_false;
          mxifa_read_end(f_handle_channel, irs_true);
          f_mode = mxnc_mode_broadcast;
        }
      }
    } break;
    case mxnc_mode_broadcast_vars: {
      mxn_sz_t pos = 1;
      irs_bool begin_finded = find_begin_in_data((irs_u8 *)f_broadcast_packet,
        f_broadcast_packet_size, pos);
      irs_bool begin_in_zero_pos = (0 == pos);
      irs_bool bad_packet = irs_false;
      for (;;) {
        if (!begin_finded || !begin_in_zero_pos) {
          bad_packet = irs_true;
          break;
        }
        if (f_broadcast_packet->code_comm != MXN_WRITE_BROADCAST) {
          bad_packet = irs_true;
          break;
        }
        if (f_broadcast_packet->var_ind_first != 0) {
          bad_packet = irs_true;
          break;
        }
        f_broadcast_count = f_broadcast_packet->var_count;

        mxn_sz_t old_size = f_broadcast_packet_size;
        f_broadcast_packet_size =
          (SIZE_OF_HEADER + f_broadcast_count + 1)*sizeof(irs_i32);
        f_broadcast_packet = (mxn_packet_t *)renew (f_broadcast_packet,
          old_size, f_broadcast_packet_size);
        mxifa_read_begin(f_handle_channel, IRS_NULL,
          ((irs_u8 *)f_broadcast_packet) + old_size,
          f_broadcast_packet_size - old_size);
        set_to_cnt(f_net_to, net_t);
        break;
      }
      if (bad_packet) {
        f_mode = mxnc_mode_broadcast;
      } else {
        f_mode = mxnc_mode_broadcast_vars_wait;
      }
    } break;
    case mxnc_mode_broadcast_vars_wait: {
      if (mxifa_read_end(f_handle_channel, irs_false)) {
        if (checksum_valid(f_broadcast_packet, f_broadcast_count,
          f_broadcast_packet_size)) {
          if (f_broadcast_vars) {
            delete f_broadcast_vars;
            f_broadcast_vars = IRS_NULL;
          }
          f_broadcast_vars = new irs_i32[f_broadcast_count];
          if (f_broadcast_vars) {
            memcpy((void *)f_broadcast_vars, (void *)f_broadcast_packet->var,
              f_broadcast_count*sizeof(irs_i32));
            f_broadcast_mode = irs_true;
          } else {
            f_broadcast_mode = irs_false;
          }
        }
        f_mode = mxnc_mode_broadcast;
      } else if (test_to_cnt(f_net_to)) {
        mxifa_read_end(f_handle_channel, irs_true);
        f_mode = mxnc_mode_broadcast;
      }
    } break;
    #ifdef NOP
    case mxnc_mode_free: {
    } break;
    #endif //NOP
  }
}
//---------------------------------------------------------------------------
// Прерывание операции
void mxnetc::abort()
{
  if (f_command != MXN_COM_NONE) {
    f_abort_request = irs_true;
  }
}
//---------------------------------------------------------------------------
// Поиск начала в пакете
irs_bool mxnetc::find_begin_in_data(irs_u8 *buf, mxn_sz_t size, mxn_sz_t &pos)
{
  //beg_pack_array[
  mxn_sz_t bpa_ind = 0;
  pos = -1;
  for (mxn_sz_t buf_ind = 0; buf_ind < size; buf_ind++) {
    if (buf[buf_ind] == beg_pack_array[bpa_ind]) {
      if (bpa_ind >= bpa_size - 1) {
        pos = buf_ind - bpa_size + 1;
        return irs_true;
      }
      bpa_ind++;
    } else {
      bpa_ind = 0;
    }
  }
  return irs_false;
}
//---------------------------------------------------------------------------
// Проверка контрольной суммы
irs_bool mxnetc::checksum_valid(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size)
{
  mxn_sz_t request_packet_size =
    (SIZE_OF_HEADER + var_count + 1)*sizeof(irs_i32);
  if (request_packet_size <= packet_size) {
    return packet->var[var_count] == checksum_calc(packet, var_count,
      packet_size);
  } else {
    return irs_false;
  }
}
//---------------------------------------------------------------------------
// Расчет контрольной суммы
irs_i32 mxnetc::checksum_calc(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size)
{
  mxn_sz_t request_packet_size =
    (SIZE_OF_HEADER + var_count + 1)*sizeof(irs_i32);
  if (request_packet_size > packet_size) {
    return 0;
  }

  irs_i32 chksum = 0;
  chksum += packet->code_comm;
  chksum += packet->var_ind_first;
  chksum += packet->var_count;
  for (mxn_cnt_t ind_var = 0; ind_var < var_count; ind_var++) {
    chksum += packet->var[ind_var];
  }
  return chksum;
}
//---------------------------------------------------------------------------
// Чтение последней ошибки
mxnc_error_t mxnetc::get_last_error()
{
  return f_last_error;
}
//---------------------------------------------------------------------------
// Разрешение обработки
irs_bool mxnetc::deny_proc()
{
  if (f_create_error) {
    f_last_error = mxnc_err_invalid_param;
    f_status = mxnc_status_error;
    return irs_true;
  }
  if (f_command != MXN_COM_NONE) {
    f_last_error = mxnc_err_busy;
    f_status = mxnc_status_error;
    return irs_true;
  }
  return irs_false;
}
//---------------------------------------------------------------------------
// Заполнение пакета
irs_bool mxnetc::packet_fill(mxn_cnt_t code_comm, mxn_cnt_t packet_var_first,
  mxn_cnt_t packet_var_count, irs_i32 *vars, mxn_cnt_t var_count)
{
  f_packet_size = (SIZE_OF_HEADER + var_count + 1)*sizeof(irs_i32);
  if (code_comm == MXN_WRITE) {
    f_receive_size = (SIZE_OF_HEADER + 1)*sizeof(irs_i32);
  } else {
    f_receive_size = f_packet_size;
  }
  if (code_comm == MXN_READ) {
    f_send_size = (SIZE_OF_HEADER + 1)*sizeof(irs_i32);
  } else {
    f_send_size = f_packet_size;
  }
  if (f_packet) {
    delete f_packet;
    f_packet = IRS_NULL;
  }
  f_packet = (mxn_packet_t *)new irs_u8[f_packet_size];
  if (f_packet) {
    f_packet->ident_beg_pack_first = MXN_CONST_IDENT_BEG_PACK_FIRST;
    f_packet->ident_beg_pack_second = MXN_CONST_IDENT_BEG_PACK_SECOND;
    f_packet->code_comm = code_comm;
    f_packet->var_ind_first = packet_var_first;
    f_packet->var_count = packet_var_count;
    if (code_comm == MXN_WRITE) {
      memcpy((void *)f_packet->var, (void *)vars, var_count*sizeof(irs_i32));
      f_packet->var[var_count] = checksum_calc(f_packet, var_count,
        f_send_size);
    } else {
      f_packet->var[0] = checksum_calc(f_packet, 0, f_send_size);
    }
    return irs_true;
  } else {
    return irs_false;
  }
}
//---------------------------------------------------------------------------
// Перераспределение памяти с сохранением данных
void *mxnetc::renew(void *pointer, mxn_sz_t old_size, mxn_sz_t new_size)
{
  void *new_pointer = IRS_NULL;
  if (new_size) {
    new_pointer = new void *[new_size];
    if (!new_pointer) return new_pointer;
  }
  if (pointer) {
    if (old_size && new_size) {
      memcpy(new_pointer, pointer, irs_min(old_size, new_size));
    }
    delete pointer;
    pointer = IRS_NULL;
  }
  return new_pointer;
}
//---------------------------------------------------------------------------
// Установка таймаута операций, с
void mxnetc::set_timeout(calccnt_t t_num, calccnt_t t_denom)
{
  if (t_denom != 0) {
    f_oper_t = TIME_TO_CNT(t_num, t_denom);
  } else {
    f_oper_t = 0;
  }
}
//---------------------------------------------------------------------------
// Установка ip-адреса удаленного устройства
void mxnetc::set_dest_ip(mxip_t ip)
{
  if (mxifa_get_channel_type(f_handle_channel) == mxifa_ei_win32_tcp_ip) {
    mxifa_win32_tcp_ip_cfg cfg;
    mxifa_get_config(f_handle_channel, (void *)&cfg);
    cfg.dest_ip = ip;
    mxifa_set_config(f_handle_channel, (void *)&cfg);
  }
}
//---------------------------------------------------------------------------
// Установка порта удаленного устройства
void mxnetc::set_dest_port(irs_u16 port)
{
  if (mxifa_get_channel_type(f_handle_channel) == mxifa_ei_win32_tcp_ip) {
    mxifa_win32_tcp_ip_cfg cfg;
    mxifa_get_config(f_handle_channel, (void *)&cfg);
    cfg.dest_port = port;
    mxifa_set_config(f_handle_channel, (void *)&cfg);
  }
}
//---------------------------------------------------------------------------
// Установка локального порта
void mxnetc::set_local_port(irs_u16 port)
{
  if (mxifa_get_channel_type(f_handle_channel) == mxifa_ei_win32_tcp_ip) {
    mxifa_win32_tcp_ip_cfg cfg;
    mxifa_get_config(f_handle_channel, (void *)&cfg);
    cfg.local_port = port;
    mxifa_set_config(f_handle_channel, (void *)&cfg);
  }
}
//---------------------------------------------------------------------------
//#pragma package(smart_init)

