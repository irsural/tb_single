// Функции для mxifa (mxifa routines)
// Версия 0.1

//---------------------------------------------------------------------------
#ifndef mxifarH
#define mxifarH

#include <irsdefs.h>

// Длина строки для ip-адреса
#define IPSTR_LEN 16
// Максимальное число сегмента ip-адреса
#define IPNUM_MAX 255
//---------------------------------------------------------------------------
// Тип IP-адреса
typedef struct _mxip_t {
  irs_u8 val[4];
} mxip_t;
//---------------------------------------------------------------------------
// Нулевой IP
extern const mxip_t zero_ip;
//---------------------------------------------------------------------------
// Оператор == для mxip_t
irs_bool operator ==(mxip_t ip1, mxip_t ip2);
// Оператор != для mxip_t
irs_bool operator !=(mxip_t ip1, mxip_t ip2);
// Преобразование IP в строку
void mxip_to_cstr(irs_u8 *ip_in_str, mxip_t ip);
// Преобразование, если возможно строки в IP
irs_bool cstr_to_mxip(mxip_t &ip, irs_u8 *str_ip);
//---------------------------------------------------------------------------
#endif
