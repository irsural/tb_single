// ����� ���稪�
// Watcom DOS
// �ᯮ��㥬 ���஥��� ���稪 ������
// ����� 0.2
#include <counter.h>
//#include <time.h>

// ������� � ������ ���뢠���� ���稪 ������
//typedef long long counter_t;

// �६� �����஢�� � ⨪��
//const clock_t time_int_calibr = 18;

// ��᫮ ᥪ㭤 � ���ࢠ��
//counter_t SECONDS_PER_INTERVAL = 1;
// ������⢮ ����⮢ � ᥪ㭤�
//counter_t COUNTER_PER_SECOND = 300000000;

// �⥭�� ���஥����� ���稪� ������
extern counter_t get_rdtsc(void);
#pragma aux get_rdtsc = \
        ".586"          \
        "db 0Fh, 31h"   \
        "cld"           \
        "nop"           \
        "nop"           \
        "nop"           \
        "nop"           \
        modify [eax edx];
// ���樠������ ���稪�
void counter_init()
{
	#ifdef NOP
	clock_t start_clock = clock();
	while (clock() - cur_clock < 1);
	counter_t start_cnt = get_rdtsc();
	while (clock() - cur_clock < time_int_calibr);
	#endif
}
// �⥭�� ���稪�
counter_t counter_get()
{
	counter_t cnt = get_rdtsc();
	return cnt;
}
// �����樠������ ���稪�
void counter_deinit()
{
}
