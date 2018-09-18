// ������ � ��������
// ������ 0.6
#ifndef TIMERH
#define TIMERH

#include <irsdefs.h>
#include <counter.h>

// �������������� ������� � �������� � ���� ��������� ���������
// _TIME_NUM_ � ����������� _TIME_DENOM_ � ������� ��������
#define TIME_TO_CNT(_TIME_NUM_, _TIME_DENOM_)\
	( \
	  ((calccnt_t)(_TIME_NUM_)) * COUNTER_PER_INTERVAL /\
  	( ((calccnt_t)(_TIME_DENOM_)) * SECONDS_PER_INTERVAL )\
  )
// �������������� �� ����������� � ���������� ������ ��������
#define MS_TO_CNT(ms) ((calccnt_t)ms * COUNTER_PER_INTERVAL / ((calccnt_t)1000 * SECONDS_PER_INTERVAL))
// �������������� �� ����������� � ���������� ������ ��������
#define MCS_TO_CNT(mcs) ((calccnt_t)mcs * COUNTER_PER_INTERVAL / ((calccnt_t)1000000 * SECONDS_PER_INTERVAL))
// �������������� �������� � ������������ �����, �
#define CNT_TO_DBLTIME(_CNT_)\
  ( ((double)(SECONDS_PER_INTERVAL)) * ((double)(_CNT_)) / \
    ((double)(COUNTER_PER_INTERVAL)) )

// ��������� �������� t �� ���������� counter
void set_to_cnt(counter_t &to, counter_t t);
// ��������� ���������� �������� t �� ���������� counter
void next_to_cnt(counter_t &to, counter_t t);
// �������� �������� �� ���������� counter
irs_bool test_to_cnt(counter_t to);
// ������������� �������
void init_to_cnt();
// ��������������� �������
void deinit_to_cnt();

#endif //TIMERH
