// ��������� ���������� ��� ������� ������ (�����������)
// Max Interface Abstraction
// ������ 0.4
#ifndef MXIFAH
#define MXIFAH

#include <irsdefs.h>
#include <mxifal.h>
#include <mxifar.h>

// ������ (<������� ����>.<������� ����>)
#define MXIFA_VERSION 0x0003

// ������ ������� ������������ �� ����������� ��� ��������
// mxnet
#define MXIFA_MXNET 1
// mxnetc
#define MXIFA_MXNETC 2   
                         
// ��� ������
typedef enum _mxifa_ei_t {
  // ����������
  mxifa_ei_unknown,
  // UDP/IP ���� ���������
  mxifa_ei_avr128_ether,
  // Win32 TCP/IP
  mxifa_ei_win32_tcp_ip
} mxifa_ei_t;

// ��������� ��� �������� ��������� �������
typedef struct _mxifa_avr128_ether_dest_t {
  // ��������� IP
  mxip_t ip;
  // ��������� ����
  irs_u16 port;
} mxifa_avr128_ether_dest_t;

// ��������� ��� �������� ��������� �������
typedef struct _mxifa_win32_tcp_ip_dest_t {
  // ��������� IP
  mxip_t ip;
  // ��������� ����
  irs_u16 port;
} mxifa_win32_tcp_ip_dest_t;

// ���������� ��������� ��� �������� ��������� �������
typedef union _mxifa_dest_t {
  // ��������� ��� �������� ��������� ������� ������ ���� mxifa_ei_avr128_ether
  mxifa_avr128_ether_dest_t avr128_ether;
  // ��������� ��� �������� ��������� ������� ������ ���� mxifa_ei_win32_tcp_ip
  mxifa_win32_tcp_ip_dest_t win32_tcp_ip;
} mxifa_dest_t;

// ��������� ��� ��������� ������������ avr128_ether � ������� mxifa_set_config
typedef struct _mxifa_avr128_cfg {
  // ��������� IP
  mxip_t *ip;
  // ����� �������
  mxip_t *mask;
} mxifa_avr128_cfg;

// ��������� ��� ��������� ������������ win32_tcp_ip � ������� mxifa_set_config
typedef struct _mxifa_win32_tcp_ip_cfg {
  // ��������� IP
  mxip_t dest_ip;
  // ��������� ����
  irs_u16 dest_port;
  // ��������� ����
  irs_u16 local_port;
} mxifa_win32_tcp_ip_cfg;

// ������������� mxifa
void mxifa_init();
// ��������������� mxifa
void mxifa_deinit();
// ������������ ��������
void mxifa_tick();
// ����������� �������� channel
// channel - ����� ������, ���������� � ����������� ������� ��� ������ ���������
// is_broadcast - �������� ������ � ����������������� ������
// ������� - ��������� �� ������ ������
void *mxifa_open(mxifa_ch_t channel, irs_bool is_broadcast);
// ������������� �������� channel
// channel - ����� ������, ���������� � ����������� ������� ��� ������ ���������
// is_broadcast - �������� ������ � ����������������� ������
// ������� - ��������� �� ������ ������
void *mxifa_open_begin(mxifa_ch_t channel, irs_bool is_broadcast);
// �������� ��������� �������� channel
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// abort - ����������� ���������� � ������� ��������
// ������� - �������� ��������
irs_bool mxifa_open_end(void *pchdata, bool abort);
// ����������� �������� channel
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// ������� - �������� ��������
irs_bool mxifa_close(void *pchdata);
// ������������� �������� channel
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// ������� - �������� ��������
irs_bool mxifa_close_begin(void *pchdata);
// �������� ��������� �������� channel
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// abort - ����������� ���������� � ������� ��������
// ������� - �������� ��������
irs_bool mxifa_close_end(void *pchdata, bool abort);
// ������ ������ � ��������� ��� ��������
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// dest - �������� ������, ���� IRS_NULL, �� ������������ �������� ������
//   �� ���������
// buf - �������� �� ������ ��� ��������
// size - ������ ������ � ������
// ������� - �������� ��������
irs_bool mxifa_write_begin(void *pchdata, mxifa_dest_t *dest, 
  irs_u8 *buf, mxifa_sz_t size);
// �������� ��������� ������
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// abort - ����������� ���������� � ������� ��������
// ������� - �������� ��������
irs_bool mxifa_write_end(void *pchdata, irs_bool abort);
// ������ �������� ������ �� ���������� � �����
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// dest - �������� ������, ���� IRS_NULL, �� ������������ �������� ������
//   �� ���������
// buf - �������� �� ����� ��� ������
// size - ������ ������ � ������
// ������� - �������� ��������
irs_bool mxifa_read_begin(void *pchdata, mxifa_dest_t *dest,
  irs_u8 *buf, mxifa_sz_t size);
// �������� ��������� ������
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// abort - ����������� ���������� � ������� ��������
// ������� - �������� ��������
irs_bool mxifa_read_end(void *pchdata, irs_bool abort);
// ������ ���� ������
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// ������� - ��� ������
mxifa_ei_t mxifa_get_channel_type(void *pchdata);
// ��������� ������������ ������
// pchdata - ��������� �� ������ ������, ������������ mxifa_open ��� 
//   mxifa_open_begin
// config - ��������� �� ��������� ..._cfg ��������������� ������� ���� ������
void mxifa_set_config(void *pchdata, void *config);
// ������ ������������ ������
// config - ��������� �� ��������� ..._cfg ��������������� ������� ���� ������
void mxifa_get_config(void *pchdata, void *config);

#endif //MXIFAH
