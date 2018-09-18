//---------------------------------------------------------------------------
#include <assert.h>

#include <csvwork.h>

//---------------------------------------------------------------------------

//***************************************************************************
// Класс csv_file - Запсь csv-файлов

//---------------------------------------------------------------------------
// Конструктор
csv_file::csv_file()
{
  memset(this, 0, sizeof(*this));
}
//---------------------------------------------------------------------------
// Деструктор
csv_file::~csv_file()
{
  memset(this, 0, sizeof(*this));
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::add_par(const char *par_name, e_val_type val_type,
  var_val_t par_value)
{
  if (f_par_count + f_var_count < max_par_var_count) {
    f_par_count++;
    csv_int index_cur_par = max_par_var_count - f_par_count;
    strncpy(f_par_var_names[index_cur_par], par_name, max_len_par_var_name);
    f_par_var_types[index_cur_par] = val_type;
    switch (val_type) {
      case ec_i32_type:
        f_par_var_values[index_cur_par].uf_i32_type = par_value.uf_i32_type;
        break;
      case ec_float_type:
        f_par_var_values[index_cur_par].uf_float_type = par_value.uf_float_type;
        break;
      case ec_str_type:
        strncpy(f_par_var_values[index_cur_par].uf_str_type,
          par_value.uf_str_type, max_len_str_var);
        f_par_var_values[index_cur_par].uf_str_type[max_len_str_var] = 0;
        break;
      default:
        int invalid_type = 0;
        assert(invalid_type);
        break;
    }
    return irs_true;
  } else {
    return irs_false;
  }
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::add_par(const char *par_name, irs_i32 par_value)
{
  var_val_t par_value_vvt;
  par_value_vvt.uf_i32_type = par_value;
  return add_par(par_name, ec_i32_type, par_value_vvt);
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::add_par(const char *par_name, float par_value)
{
  var_val_t par_value_vvt;
  par_value_vvt.uf_float_type = par_value;
  return add_par(par_name, ec_float_type, par_value_vvt);
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::add_par(const char *par_name, const char *par_value)
{
  var_val_t par_value_vvt;
  strncpy(par_value_vvt.uf_str_type, par_value, max_len_str_var);
  par_value_vvt.uf_str_type[max_len_str_var] = 0;
  return add_par(par_name, ec_float_type, par_value_vvt);
}
//---------------------------------------------------------------------------
// Запись переменной, при ошибке возвращает irs_false
irs_bool csv_file::set_var(const char *var_name, var_val_t var_value)
{
  //f_var_count
  irs_bool is_var_finded = irs_false;
  csv_int index_finded_var = 0;
  for (csv_int index_cur_var = 0; index_cur_var < f_var_count; index_cur_var++) {
    if (strncmp(var_name, f_par_var_names[index_cur_var],
      max_len_par_var_name) == 0) {
      index_finded_var = index_cur_var;
      is_var_finded = irs_true;
    }
  }
  if (is_var_finded) {
    switch (f_par_var_types[index_finded_var]) {
      case ec_i32_type:
        f_par_var_values[index_finded_var].uf_i32_type = var_value.uf_i32_type;
        break;
      case ec_float_type:
        f_par_var_values[index_finded_var].uf_float_type = var_value.uf_float_type;
        break;
      case ec_str_type:
        strncpy(f_par_var_values[index_finded_var].uf_str_type,
          var_value.uf_str_type, max_len_str_var);
        f_par_var_values[index_finded_var].uf_str_type[max_len_str_var] = 0;
        break;
      default:
        int invalid_type = 0;
        assert(invalid_type);
        break;
    }
    return irs_true;
  } else {
    return irs_false;
  }
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::set_var(const char *var_name, irs_i32 var_value)
{
  var_val_t var_value_vvt;
  var_value_vvt.uf_i32_type = var_value;
  return set_var(var_name, var_value_vvt);
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::set_var(const char *var_name, float var_value)
{
  var_val_t var_value_vvt;
  var_value_vvt.uf_float_type = var_value;
  return set_var(var_name, var_value_vvt);
}
//---------------------------------------------------------------------------
// Добавить параметр в начале csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::set_var(const char *var_name, const char *var_value)
{
  var_val_t var_value_vvt;
  strncpy(var_value_vvt.uf_str_type, var_value, max_len_str_var);
  var_value_vvt.uf_str_type[max_len_str_var] = 0;
  return set_var(var_name, var_value_vvt);
}
//---------------------------------------------------------------------------
// Добавить название столбца csv-файла, при ошибке возвращает irs_false
irs_bool csv_file::add_col(const char *col_name, e_val_type val_type)
{
  if (f_par_count + f_var_count < max_par_var_count) {
    csv_int index_cur_var = f_var_count;
    f_var_count++;
    strncpy(f_par_var_names[index_cur_var], col_name, max_len_par_var_name);
    f_par_var_types[index_cur_var] = val_type;
    return irs_true;
  } else {
    return irs_false;
  }
}
//---------------------------------------------------------------------------
// Открыть файл name, при ошибке возвращает irs_false
irs_bool csv_file::open(const char *file_name)
{
  if (f_ini_file) return irs_true;

  f_lines_writed_count = 0;

  f_ini_file = fopen(file_name, "wt");
  if (!f_ini_file) goto _error;

  return irs_true;

  _error:
  return irs_false;
}
//---------------------------------------------------------------------------
// Закрыть файл
irs_bool csv_file::close()
{
  if (f_ini_file)
  if (!fclose(f_ini_file)) {
    f_ini_file = NULL;
  } else {
    return irs_false;
  }
  return irs_true;
}
//---------------------------------------------------------------------------
// Запись строки в csv-файл, при превышении количества записей возвращает irs_false
irs_bool csv_file::write_line()
{
  // Обработка лимита по количеству строк
  if (f_lines_writed_count >= max_line_count) return irs_false;
  f_lines_writed_count++;

  if (1 == f_lines_writed_count) {
    // Список параметров в начале файла
    csv_int index_last_par = max_par_var_count - f_par_count;
    for (csv_int index_cur_par = max_par_var_count - 1;
      index_cur_par >= index_last_par; index_cur_par--) {
      fprintf(f_ini_file, "%s", f_par_var_names[index_cur_par]);
      switch (f_par_var_types[index_cur_par]) {
        case ec_i32_type:
          fprintf(f_ini_file, ";%ld\n", f_par_var_values[index_cur_par].uf_i32_type);
          break;
        case ec_float_type:
          fprintf(f_ini_file, ";%g\n", f_par_var_values[index_cur_par].uf_float_type);
          break;
        case ec_str_type:
          fprintf(f_ini_file, ";%s\n", f_par_var_values[index_cur_par].uf_str_type);
          break;
        default:
          int invalid_type = 0;
          assert(invalid_type);
          break;
      }
      fprintf(f_ini_file, "*\n");
    }

    // Строка с названиями переменных
    for (csv_int index_cur_var = 0; index_cur_var < f_var_count;
      index_cur_var++) {
      fprintf(f_ini_file, "%s;", f_par_var_names[index_cur_var]);
    }
    fprintf(f_ini_file, "\n");
  }

  // Строка со значениями переменных
  char converted_var[max_len_str_var + 1];
  csv_int var_len_in_chars = 0;
  float var_float = 0.f;
  for (csv_int index_cur_var = 0; index_cur_var < f_var_count; index_cur_var++) {
    switch (f_par_var_types[index_cur_var]) {
      case ec_i32_type:
        fprintf(f_ini_file, "%ld;",
          f_par_var_values[index_cur_var].uf_i32_type);
        break;
      case ec_float_type:
        var_float = f_par_var_values[index_cur_var].uf_float_type;
        sprintf(converted_var, "%g;", var_float);
        var_len_in_chars = strlen(converted_var);
        for (csv_int index_var_in_char = 0;
          index_var_in_char < var_len_in_chars;
          index_var_in_char++) {
          if ('.' == converted_var[index_var_in_char])
            converted_var[index_var_in_char] = ',';
        }
        fprintf(f_ini_file, converted_var);
        //fprintf(f_ini_file, "%g;", f_par_var_values[index_cur_var].uf_float_type);
        break;
      case ec_str_type:
        fprintf(f_ini_file, "%s;", f_par_var_values[index_cur_var].uf_str_type);
        break;
      default:
        int invalid_type = 0;
        assert(invalid_type);
        break;
    }
  }
  fprintf(f_ini_file, "\n");

  return true;
}
//---------------------------------------------------------------------------
// Очистить столбцы
void csv_file::clear_cols()
{
  f_var_count = 0;
}
//---------------------------------------------------------------------------
// Очистить столбцы
void csv_file::clear_pars()
{
  f_par_count = 0;
}
//---------------------------------------------------------------------------
