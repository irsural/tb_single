//---------------------------------------------------------------------------
//#include <vcl.h>
//#pragma hdrstop

#include "alg.h"
#include <string.h>
//---------------------------------------------------------------------------
// ������ ������������ �� ���������
alg_ring::alg_ring()
{
}
//---------------------------------------------------------------------------
// �����������
alg_ring::alg_ring(int size_max)
{
  // ������� ������ ������
  f_begin = 0;
  // ������� ����� ������
  f_size = 0;
  // ������������ ����� ������
  f_size_max = size_max;
  // ��������� �����
  f_buf_ring = IRS_NULL;
  // ������ � ������������
  f_create_error = irs_false;
  // ��������� �����
  f_buf_ring = new void*[f_size_max];
  memset(f_buf_ring, 0, f_size_max*sizeof(void*));
  if (!f_buf_ring) f_create_error = irs_true;
}
//---------------------------------------------------------------------------
//����������
alg_ring::~alg_ring()
{
  if (f_buf_ring) delete f_buf_ring;
}
//---------------------------------------------------------------------------
// ������ � �������� ������� �������� ������
void *alg_ring::read_and_remove_first()
{
  void *readed_elem = IRS_NULL;
  if (f_size > 0) {
    readed_elem = f_buf_ring[f_begin];
    begin_inc();
    f_size--;
  } 
  return readed_elem;
}
//---------------------------------------------------------------------------
// ���������� ������ �������� � ����� ������
void *alg_ring::add_last_and_get_removed(void *buf_elem)
{
  void *removed_elem = IRS_NULL;
  if (f_size >= f_size_max) {
    removed_elem = f_buf_ring[f_begin];
    f_buf_ring[f_begin] = buf_elem;
    begin_inc();
  } else {
    irs_u32 end = get_end();
    f_buf_ring[end] = buf_elem;
    f_size++;
  }
  return removed_elem;
}
//---------------------------------------------------------------------------
// ������ � ������������
irs_bool alg_ring::create_error()
{
  return f_create_error;
}
//---------------------------------------------------------------------------
// ���������� ������ ������ �� 1 � ������ �������
void alg_ring::begin_inc()
{
  f_begin++;
  if (f_begin >= f_size_max) f_begin = 0;
}
//---------------------------------------------------------------------------
// ���������� ����� ������
irs_u32 alg_ring::get_end()
{
  irs_u32 end = f_begin + f_size;
  if (end >= f_size_max) end -= f_size_max;
  return end;
}
//---------------------------------------------------------------------------
#pragma package(smart_init)
