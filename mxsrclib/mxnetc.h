// �������� MxNetC (Max Network Client)
// ������ 0.3
//---------------------------------------------------------------------------
#ifndef mxnetcH
#define mxnetcH

#include <irsdefs.h>
#include <mxnetda.h>
#include <mxifa.h>
#include <timer.h>
//---------------------------------------------------------------------------
// ������ �������������� ������ ������
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
// ����� ���
class mxnetc
{
  // ����� ��� ������/�������� ������
  mxn_packet_t *f_packet;
  // ������ ������, ����
  mxn_sz_t f_packet_size;
  // ������ ������������ ������, ����
  mxn_sz_t f_receive_size;
  // ������ ������������� ������, ����
  mxn_sz_t f_send_size;
  // ������� �����
  mxnc_mode_t f_mode;
  // ����� ����� �������� �� ���������
  //mxnc_mode_t f_mode_return;
  // ������� �������
  mxn_cnt_t f_command;
  // ������� ������ ��������
  mxnc_status_t f_status;
  // ������ �������� ������������ � mxnc_mode_reset
  mxnc_status_t f_local_status;
  // ��������� �� ���������� ��� ������
  irs_u16 *f_version;
  // ���������� ������
  void *f_handle_channel;
  // ������ �������� ������
  irs_bool f_create_error;
  // ������ ���������� ��������
  irs_bool f_abort_request;
  // ������� � ������ ��� ������ ������ ������
  //mxn_sz_t f_packet_pos;
  // ������������� ������ ������
  irs_u8 beg_pack_array[bpa_size];
  // �������� ������ ��� ������ ������ ������
  //irs_bool f_fbpc_wait_read;//!!!!!!!!
  // ������� ������ �� fbpc �� ��������
  //irs_bool f_fbpc_end_to;//!!!!!!!!
  // ��������� ������
  mxnc_error_t f_last_error;
  // ��������� �� �������������� ���������� ���������� ����������
  mxn_cnt_t *f_user_size;
  // ������ ���������� ��� ������/������
  mxn_cnt_t f_user_index;
  // ���������� ���������� ��� ������/������
  mxn_cnt_t f_user_count;
  // ����������������� ����� �������� �������������
  irs_bool f_user_broadcast; 
  // ��������� �� ���������������� ����������
  irs_i32 *f_user_vars;
  // ���������� �������� ������� ��������
  counter_t f_net_to;
  // ����� ��� ������ ������������������ ������
  mxn_packet_t *f_broadcast_packet;
  // ������ ������ ��� ������ ������������������ ������
  mxn_sz_t f_broadcast_packet_size;
  // ����� ���������� ���������� ��� ����������������� ��������
  mxn_cnt_t f_broadcast_count;
  // ���������� ��������� ��� ����������������� ��������
  irs_i32 *f_broadcast_vars;
  // ����������������� ����� ������
  irs_bool f_broadcast_mode;
  // ���������� �������� ������� ��������
  counter_t f_oper_to;
  // ���������� ������� ������� ��������
  counter_t f_oper_t;

  // ��������� ����������� �� ���������
  mxnetc::mxnetc();
  // ����� ������ � ������
  irs_bool mxnetc::find_begin_in_data(irs_u8 *buf, mxn_sz_t size,
    mxn_sz_t &pos);
  // �������� ����������� �����
  irs_bool mxnetc::checksum_valid(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size);
  // ������ ����������� �����
  irs_i32 mxnetc::checksum_calc(mxn_packet_t *packet, mxn_cnt_t var_count,
    mxn_sz_t packet_size);
  // ���������� ���������
  irs_bool mxnetc::deny_proc();
  // ���������� ������
  irs_bool mxnetc::packet_fill(mxn_cnt_t code_comm, mxn_cnt_t packet_var_first,
    mxn_cnt_t packet_var_count, irs_i32 *vars, mxn_cnt_t var_count);
  // ����������������� ������ � ����������� ������
  void *mxnetc::renew(void *pointer, mxn_sz_t old_size, mxn_sz_t new_size);
public:
  // �����������
  mxnetc::mxnetc(mxifa_ch_t channel);
  // ����������
  mxnetc::~mxnetc();
  // ������ ������ ��������� mxnet
  void mxnetc::get_version(irs_u16 *version);
  // ������ ������� ������� (���������� ����������)
  void mxnetc::get_size(mxn_cnt_t *size);
  // ������ ����������
  void mxnetc::read(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars);
  // ������ ����������
  void mxnetc::write(mxn_cnt_t index, mxn_cnt_t count, irs_i32 *vars);
  // ���������/���������� ������������������ ������
  void mxnetc::set_broadcast(irs_bool broadcast_mode);
  // ������ ������� ��������
  mxnc_status_t mxnetc::status();
  // ������������ ��������
  void mxnetc::tick();
  // ���������� ��������
  void mxnetc::abort();
  // ������ ��������� ������
  mxnc_error_t mxnetc::get_last_error();
  // ��������� �������� ��������, �
  void mxnetc::set_timeout(calccnt_t t_num, calccnt_t t_denom);
  // ��������� ip-������ ���������� ����������
  void mxnetc::set_dest_ip(mxip_t ip);
  // ��������� ����� ���������� ����������
  void mxnetc::set_dest_port(irs_u16 port);
  // ��������� ���������� �����
  void mxnetc::set_local_port(irs_u16 port);
};
//---------------------------------------------------------------------------
#endif
