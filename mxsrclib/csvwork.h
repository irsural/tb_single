//---------------------------------------------------------------------------
#ifndef csvworkH
#define csvworkH

#include <string.h>
#include <stdio.h>

#include <irsdefs.h>

namespace csvwork {
//---------------------------------------------------------------------------
// typedef-определения
//typedef irs_i32 var_val_t;
//typedef irs_u8 csv_char;
typedef irs_u32 csv_int;
typedef enum {ec_i32_type, ec_float_type, ec_str_type} e_val_type;
typedef (*dbg_text_out_fn)(const char *dbg_text);
//---------------------------------------------------------------------------
// Константы
const csv_int max_line_count = 30000;
const csv_int max_par_var_count = 1000;
const csv_int max_len_fn = 1000;
const csv_int max_len_par_var_name = 50;
const csv_int max_len_str_var = 50;
//---------------------------------------------------------------------------
// typedef-определения составных типов
typedef union {
  irs_i32 uf_i32_type;
  float uf_float_type;
  char uf_str_type[max_len_str_var + 1];
} var_val_t;
//---------------------------------------------------------------------------
// Класс для работы с csv-файлами
class csv_file
{
private:
  // Поля
  csv_int     f_lines_writed_count;
  csv_int     f_par_count;
  csv_int     f_var_count;
  //char        f_file_name[max_len_fn];
  char        f_par_var_names [max_par_var_count][max_len_par_var_name + 1];
  var_val_t   f_par_var_values[max_par_var_count];
  e_val_type  f_par_var_types [max_par_var_count];
  dbg_text_out_fn f_dbg_text_out_fn;
  FILE*       f_ini_file;

  // Методы
  // Вывод отладочной информации
  void dbg_msg(const char *msg);
  // Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
  irs_bool add_par(const char *par_name, e_val_type val_type,
    var_val_t par_value);
  // Запись переменной, при ошибке возвращает irs_false
  irs_bool set_var(const char *var_name, var_val_t var_value);

public:
  // Конструкторы и деструкторы
  csv_file();
  ~csv_file();

  // Методы
  // Установка функции для вывода отладочной информации
  void set_dbg_text_out_fn(dbg_text_out_fn a_dbg_text_out_fn);
  // Запись переменной, при ошибке возвращает irs_false
  irs_bool set_var(const char *var_name, irs_i32 var_value);
  irs_bool set_var(const char *var_name, float var_value);
  irs_bool set_var(const char *var_name, const char *var_value);
  // Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
  irs_bool add_par(const char *par_name, irs_i32 par_value);
  irs_bool add_par(const char *par_name, float par_value);
  irs_bool add_par(const char *par_name, const char *par_value);
  // Добавить название столбца csv-файла, при ошибке возвращает irs_false
  irs_bool add_col(const char *col_name, e_val_type val_type);
  // Открыть файл name, при ошибке возвращает irs_false
  irs_bool open(const char *file_name);
  // Закрыть файл
  irs_bool close();
  // Запись строки в csv-файл, при превышении количества записей возвращает irs_false
  irs_bool write_line();
  // Очистить столбцы
  void clear_cols();
  // Очистить переменные
  void clear_pars();
}; //csv_file
//---------------------------------------------------------------------------
} // namespace csvwork

using namespace csvwork;
#endif
