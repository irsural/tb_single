// ������ � ��������
// ������ 0.6
#include <timer.h>
#include <irsdefs.h>

// ��������� �������� t �� ���������� counter
void set_to_cnt(counter_t &to, counter_t t)
{
  to = counter_get() + t;
}
// ��������� ���������� �������� t �� ���������� counter
void next_to_cnt(counter_t &to, counter_t t)
{
  to += t;
}
// �������� �������� �� ���������� counter
irs_bool test_to_cnt(counter_t to)
{
  irs_bool res = irs_false;
  if (counter_get() - to > 0) res = irs_true;
  return res;
}
// ������������� �������
void init_to_cnt()
{
  counter_init();
}
// ��������������� �������
void deinit_to_cnt()
{
  counter_deinit();
}
