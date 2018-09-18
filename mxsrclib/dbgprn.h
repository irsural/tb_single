//---------------------------------------------------------------------------
#ifndef dbgprnH
#define dbgprnH

#include <irsdefs.h>
//---------------------------------------------------------------------------
// typedef-�����������
typedef irs_u8 dbg_char;
typedef irs_u32 dbg_int;
typedef void (*out_str_fn)(const dbg_char *s);
#ifdef NOP
//---------------------------------------------------------------------------
// ����� ���������� ���������� �� ���� puts
void dputs(out_str_fn out_str, const dbg_char *dbg_msg);
// ����� ���������� ���������� �� ���� printf
void dprintf(out_str_fn out_str, const dbg_char *format, ...);
#endif
//---------------------------------------------------------------------------
// ����� ��� ������ ���������� ����������
class dbg_out
{
private:
  // ����
  out_str_fn f_out_str;

public:
  // ������������ � �����������
  __fastcall dbg_out();
  __fastcall ~dbg_out();
  __fastcall dbg_out(out_str_fn a_out_str);

  // ������
  // ����� ���������� ���������� �� ���� puts
  void puts(const char *dbg_msg);
  // ����� ���������� ���������� �� ���� printf
  void printf(const char *format, ...);
}; //dbg_out
//---------------------------------------------------------------------------
#endif
