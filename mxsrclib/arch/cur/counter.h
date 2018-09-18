// ����� ���稪�
// Watcom DOS
// �ᯮ��㥬 ���஥��� ���稪 ������
// ����� 0.2
#ifndef COUNTERH
#define COUNTERH

#include <irsdefs.h>

// ��� ���稪�
//typedef irs_i32 counter_t;
typedef irs_i64 counter_t;
// ��� � ���஬ �����⢫����� 楫��᫥��� �����
typedef irs_i64 calccnt_t;

// ���ᨬ��쭮� �६� ���஥ ����� �������
#define COUNTER_MAX IRS_I64_MAX
// ��᫮ ᥪ㭤 � ���ࢠ��
//extern inttime_t SECONDS_PER_INTERVAL;
#define SECONDS_PER_INTERVAL 1
// ������⢮ ����⮢ � ���ࢠ��
//extern counter_t COUNTER_PER_INTERVAL;
#define COUNTER_PER_INTERVAL 300000000

// ���樠������ ���稪�
void counter_init();
// �⥭�� ���稪�
counter_t counter_get();
// �����樠������ ���稪�
void counter_deinit();

#endif //COUNTERH
