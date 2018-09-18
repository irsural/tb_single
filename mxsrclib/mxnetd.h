// �������� MxNet defines
// ������ 0.2
#ifndef MXNETDH
#define MXNETDH

// ������ ���������, 4 ����� (32-������ �����)
#define SIZE_OF_HEADER 5
// ������ �������������� ������ ������, ����
#define SIZE_OF_BEG_PACK 8

// ���� ������� ������
#define MXN_READ_COUNT        0
#define MXN_READ              1
#define MXN_WRITE             2
//#define MXN_WRITE_ACK         3
#define MXN_GET_VERSION       4
#define MXN_SET_BROADCAST     5
#define MXN_WRITE_BROADCAST   6

// �������� �������� ����������������� �������
#define MXN_BROADCAST_INTERVAL (TIME_TO_CNT(1, 1))

// ��� ������ ������ - ������ �����
#define MXN_CONST_IDENT_BEG_PACK_FIRST  ((irs_i32)0xFF43439A)
// ��� ������ ������ - ����� �����
#define MXN_CONST_IDENT_BEG_PACK_SECOND ((irs_i32)0xE81F3AAD)

#endif //MXNETDH
