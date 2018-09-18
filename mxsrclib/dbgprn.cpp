//---------------------------------------------------------------------------
//#include <vcl.h>
//#pragma hdrstop

#include <stdarg.h>
#include <stdio.h>
#include <alloc.h>

#include <dbgprn.h>

//***************************************************************************
// Класс dbg_out
// Класс для вывода отладочной информации

//---------------------------------------------------------------------------
// Конструктор
__fastcall dbg_out::dbg_out(out_str_fn a_out_str)
{
  memset(this, 0, sizeof(*this));
  f_out_str = a_out_str;
}
//---------------------------------------------------------------------------
// Конструктор по умолчанию
__fastcall dbg_out::dbg_out()
{
  memset(this, 0, sizeof(*this));
}
//---------------------------------------------------------------------------
// Деструктор по умолчанию
__fastcall dbg_out::~dbg_out()
{
  memset(this, 0, sizeof(*this));
}
//---------------------------------------------------------------------------
// Вывод отладочной информации по типу puts
void dbg_out::puts(const char *dbg_msg)
{
  //const char *dbg_msg = dbg_msg_;
  if (!f_out_str) return;
  if (!dbg_msg) return;
  dbg_int len = strlen((char *)dbg_msg);
  dbg_char *dbg_msg_crlf = (dbg_char *)malloc(len + 2);
  if (!dbg_msg_crlf) return;
  strcpy((char *)dbg_msg_crlf, (char *)dbg_msg);
  dbg_msg_crlf[len] = '\n';
  dbg_msg_crlf[len + 1] = 0;
  f_out_str(dbg_msg_crlf);
}
//---------------------------------------------------------------------------
// Вывод отладочной информации по типу printf
void dbg_out::printf(const char *format, ...)
{
  //const char *format = format_;
  if (!f_out_str) return;
  if (!format) return;
  dbg_char *msgbuf;
  dbg_int len;
  va_list arglist;
  va_start(arglist, format);
  len = vsnprintf(NULL, 0, (char *)format, arglist);
  len++;
  va_end(arglist);
  msgbuf = (dbg_char *)malloc(len);
  if (!msgbuf) return;
  va_start(arglist, format);
  vsnprintf((char *)msgbuf, len, (char *)format, arglist);
  msgbuf[len - 1] = 0;
  va_end(arglist);
  f_out_str(msgbuf);
  free(msgbuf);
}
//---------------------------------------------------------------------------
#pragma package(smart_init)
