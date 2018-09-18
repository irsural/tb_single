// �������� MxNet (Max Network)
// ������ 0.5
#ifndef MXNETH
#define MXNETH

#include <irsdefs.h>
#include <mxifa.h>
#include <mxnetda.h>
#include <timer.h>

// ������ (<������� ����>.<������� ����>)
#define MXN_VERSION 0x0003

// ��������������� ���������� ������ ��� ��������� mxnet
struct _mxn_data_t;
typedef _mxn_data_t mxn_data_t;
// ��� ��� ������������ �������
typedef void (*mxn_tick_f)(mxn_data_t &data);
// ������ ��� ��������� mxnet
struct _mxn_data_t {
  // ���������� ����������
  mxn_cnt_t count;
  // ������� ������ ����������
  irs_i32 *vars_ext;
  // ���������� ������ ����������
  mxn_packet_t *packet;
  // ������ ���������� ���������� ������ ��� ������
  irs_bool *read_only;
  // ������ � ������ �������������
  irs_bool is_broadcast;
  // ���������� �������� ����������������� ��������
  counter_t to_broadcast;
  // ��������� ������ ������������� �� �����
  irs_bool is_broadcast_new;
  // ����� �� �������� �������� ��������
  mxifa_ch_t channel;
  // ������ ������
  void *data_ch;
  // ������ �� ��������� �������
  mxifa_dest_t dest_info;
  // ���������� ���������� � �������� ������
  mxn_cnt_t var_cnt_packet;
  // ��������� ������������ �������
  mxn_tick_f mxn_tick_next;
  // ������������ �������
  mxn_tick_f mxn_tick;
  // ���������� ���������� ����
  mxn_sz_t count_send;
  // ����������������� ��������
  irs_bool broadcast_send;
  // ������ � ���������� ������
  irs_bool write_error;
};

// ������������� ���������
void mxn_init(mxn_data_t &data, mxifa_ch_t channel, irs_i32 *vars,
  mxn_cnt_t count);
// ��������������� ���������
void mxn_deinit(mxn_data_t &data);
// ��������� ������ ��� ������ ���������� ��� ���
void mxn_set_read_only(mxn_data_t &data, mxn_cnt_t ind_var, irs_bool read_only);
// ��������� ������ ������ �������������
void mxn_set_broadcast(mxn_data_t &data, irs_bool is_broadcast);
// ������������ �������
void mxn_tick(mxn_data_t &data);
// ������ ��������� �� ���������� ������
//void *mxn_get_data_channel(mxn_data_t &data);
// ��������� ���������� IP
void mxn_set_ip(mxn_data_t &data, mxip_t ip);
// ������ ���������� IP
mxip_t mxn_get_ip(mxn_data_t &data);
// ��������� ��������� �����
void mxn_set_mask(mxn_data_t &data, mxip_t mask);
// ������ ��������� �����
mxip_t mxn_get_mask(mxn_data_t &data);

#endif //MXNETH
