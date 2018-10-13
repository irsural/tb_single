// Основная программа контроллера турбины
#define NO_INT_IN_INT // Запрет прерывания в прерывании
#define PROC_IN_LOOP // Обработка основного алгоритма в цикле
//#define NOEXIT // Запрет выхода из программы
#define WDT_ON // Включение сторожевого таймера
//#define WDT_INT // Перенос сброса сторожевого таймера в ti0
//#define NO_AI_FILTR // Блокировка фильтра AI
//#define DEBUG // Режим отладки на ПК
//#define BLOCK_ADC_ON_DBG // Блокировка обсчета АЦП в режиме отладки
#define BLOCK_SHOW_DPORT // Блокировка отображения портов в отладке
//#define DBGONCPU // Режим отладки на CPU686 (Убрать BLOCK_ADC_ON_DBG,
  //можно комбинировать с DEBUG)
#define IGNORE_INI // Игнорирование ini-файла
#define NO_AI3 // Отключение 3-ей платы AI
#define NEW_BUT // Изменение обработки кнопок ВД, НД
//#define N_BAD_BLOCK // Блокировка переменной n_bad
//#define INV_SRV // Инверсия направления вращения серводрайвера
//#define LAN_SINGLE // Запуск только сети
//#define OFF_BAD_AI // Выключение детектирования аварии аналоговых датчиков
//#define OFF_BAD_RELE_ISVT // Отключение отображения неисправных реле
//#define AI_INT // Включение прерывания AI
#define PAR_IN_CPU_FLASH // Запись параметров во внутреннюю флэш
#define CORR_HZU_ON // Включение коррекции hzu
//#define TEST // Тестовый режим
#define VALVE_GENER // Включение тестового генератора в расхаживании клапанов
#define UNIO_NEW // Новая обработка UNIO
#define ISODR // Включение изодромного звена
//#define NELIN // Включение нелинейности
//#define SHOW_TLOOP // Отображение времени цикла
//#define SHOW_INT_T0 // Отображение времени обработки прерывания при выходе

// Новое в программе начиная с 17.04.06
#define DETECT_KG // Детектирование сброса нагрузки
//#define NEW_SENS_SEL // Новый алгоритм выбора датчиков

// Новое в программе начиная с 11.05.06
#define RCHVT_DEFENSE // Включение защитного порога для интегратора РЧВТ
#define REST_T0 // Восстановление канала 0 таймера

// Новое с 9.06.06
#define FADC_IN_LOOP //Перенос фильтра АЦП в цикл
#define RES_PKT_DRV // Перезапуск пакетного драйвера
//#define RES_PKT_DRV_DBG //Интервалы времени для отладки перезапуска
//#define RES_PKT_DRV_DELAY_OFF //Отключение задержки после перезагрузки
  //пакетного драйвера
#define RES_PKT_SWITCH //Перезапуск свитча
#define RES_PKT_CPU //Сброс контроллера вместо пакетного драйвера
//#define ND_NEW //Новый алгоритм НД
//#define ND_WITHOUT_SENS //Алгоритм НД без исползования датчика
#define ND_EN_LIM_SW //Реакция на концевые выключатели НД
#define NEW_FADC //Новый алгоритм фильтров АЦП
#define UNIO_SINGLE //Работа с 1 платой UNIO
#define WDT_NEW //Включение WDT в all_proc
//#define DISABLE_TICK //Отключения прерывания для тиков
#define DETECT_N_ANOMAL //Детектирование аномальных частот
#define RES_BL_BAD_SENS //Сброс блокировок больше/меньше при
  //неисправных датчиках
#define LAN_RECEIVE_NOT_BLOCKING // Не блокировать прием данных
  //при старте программы
#define CORR_HZU_ERR_CORR // Исправление ошибок в коррекции hzu
//#define DPRINTF_OFF //Выключение вывода отладочной информаации
#define DPRINTF_UDP //Вывод отладочной информации в порт UDP
#define DPRINTF_CON //Вывод отладочной информации на консоль

// Новое с 5.12.06
#define USE_CPU_COUNTER // Использовать встроенный 64-битный счетчик ЦП
#if defined(USE_CPU_COUNTER) && defined(FADC_IN_LOOP) && defined(PROC_IN_LOOP)
#define ADC_IN_LOOP // Перенос чтения АЦП в цикл
#endif //defined(USE_CPU_COUNTER) && defined(FADC_IN_LOOP) && 
  //defined(PROC_IN_LOOP)
#ifndef USE_CPU_COUNTER
#define counter_t long
#endif //USE_CPU_COUNTER

// Новое с 29.08.08
// Блокировка переключения датчиков при расхаживании
#define SENS_SEL_BLOCK_ON_VALVE_TEST
// Блокировка фильтра частоты детектирования сброса нагрузки по n_all_bad_isvt
#define BLOCK_FILTR_DETECT_KG_ON_N_ALL_BAD_ISVT
// Блокировка выбора частоты с текущей расхаживаемой платы
#define BLOCK_SELECT_N_VALVE_TEST_ISVT
// Увеличение таймаута после взведения ИСВТ
#define VALVE_TEST_TIMEOUT_BIGGER
// Уменьшения диапазона совпадения датчиков с 10 до 3 об./мин
#define SENS_SEL_REDUCE_RANGE_EQUAL

#ifdef DBGONCPU
#define DBGISVT // Выключение чтения частоты с ИСВТ
//#define PERM_TEST_GENER // Включить постоянный тестовый генератор
  //(DBGISVT отключить)
#endif //DBGONCPU

extern "C" {
#include <tcp.h>
#include <arpa\inet.h>
}

#include "tbdefs.h"
#include "isvt.h"

#ifdef USE_CPU_COUNTER
#include <timer.h>
#include <irsdefs.h>
#endif //USE_CPU_COUNTER
#include <stdio.h>
#include <conio.h>
#include <process.h>
#include <time.h>
#include <malloc.h>
#include <float.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#include <sys\stat.h>
#include <dos.h>
#include <i86.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <limits.h>
#include <stdarg.h>

// ******************************************************************
// define-констаны
// ******************************************************************
// Порт для входящих подключений
#define MXINDTCP_PORT   5000
// Порт для входящих подключений 2
#define MXINDTCP_PORT2  5001
// Порт для вывода отладочной информации
#define MXLOGUDP_PORT   5004
// Максимальное количество переменных
#define CNT_VAR 400
// Максимальное количество терминалов
#define CNT_TRM 10
// Вектор канала 0 таймера
#define TIMER0_VECTOR 8
// Число тактов через которое происходит прерывания по каналу 0
// таймера
#ifdef NO_INT_IN_INT
  #define TIMER0_CNT 512
  //#define TIMER0_CNT 65535L
#else //NO_INT_IN_INT
  #define TIMER0_CNT 512
#endif //NO_INT_IN_INT
// Число тактов для тиков
#define TICK_CNTT 65536L
// Число прерываний для тиков
#define TICK_CNT uint(TICK_CNTT/TIMER0_CNT)
// Частота в мегагерцах которой тактируется таймер
#define TACT_T0 1.19318
// Константы для АЦП
#define INC       0x0001  // Автосканирование
#define INT       0x0002  // Бит включения прерываний
#define TMR       0x0010  // Бит включения Таймера
#define FIFO      0x0800  // Бит включения записи в FIFO
#define RS_FIFO   0x10    // Бит Reset FIFO
#define END       0x0     // Код верхней границы FIFO
#define SHARE     0x2000  // Общее прерывание для всех AI
#define ST_RDY    0x0080  // Пуск/Готовность
#define AIIRQ     5
#define IRQMask   0x20
#define AIVEC     (AIIRQ + 8)
// Число задействованных аналоговых входов
#define NUM_AI 15
// Значение АЦП соответствующее 20 мА
#define ADC_20mA 0xFFF
// Положительный максимум АЦП
//#define ADC_MAX 0x1FFF
// Минимальное значение АЦП
#define ADC_min 600
// Значение АЦП соответствующее 4 мА
#define ADC_4mA 819
// Значение АЦП выше которого считается что датчик неисправен
#define ADC_max 0x2000
// Регистры ИСВТ
// ***
// Биты регистров статуса ИСВТ
#define Bit_Status1_Rele1_ON  (byte)0x01 //0-выключено, 1-включено
#define Bit_Status1_Rele2_ON  (byte)0x02 //0-выключено, 1-включено
#define Bit_Status1_Error_Rele1  (byte)0x04 //0-исправно, 1-неисправно
#define Bit_Status1_Error_Rele2  (byte)0x08 //0-исправно, 1-неисправно
#define Bit_Status1_Sensor1  (byte)0x10 //0-выключено, 1-включено для реакции на рэле
#define Bit_Status1_Sensor2  (byte)0x20 //0-выключено, 1-включено для реакции на рэле
#define Bit_Status1_Error_Sensor1  (byte)0x40 //0-исправен, 1-неисправен
#define Bit_Status1_Error_Sensor2  (byte)0x80 //0-исправен, 1-неисправен
#define Bit_Status2_Rele1_Enabled  (byte)0x80 //0-выключено, 1-включено для реакции
#define Bit_Status2_Rele2_Enabled  (byte)0x40 //0-выключено, 1-включено для реакции
//0-выключено, 1-включено для расчёта значения частоты и производной по частоте
#define Bit_Status2_Sensor1_Enabled  (byte)0x20
//0-выключено, 1-включено для расчёта значения частоты и производной по частоте
#define Bit_Status2_Sensor2_Enabled  (byte)0x10
#define Bit_Status2_Test  (byte)0x08 // 0-рабочий режим, 1 - режим теститрования
#define Bit_Status2_Defense_Freq (byte)0x04 // 1 - сработала защита по частоте
//1 - сработала защита по производной частоты
#define Bit_Status2_Defense_Acceleration (byte)0x02
// датчик, по которому сработала защита 0 - по первому, 1- по второму
#define Bit_Status2_Defense_Sensor (byte)0x01
// ***

// ******************************************************************
// Макроопределения
// ******************************************************************
// Запрет прерываний на время обращения к сетевым переменным
#define sync(_oper_)\
  _disable();\
  _oper_;\
  _enable();
// Запрет прерывания на время обращения к функциям ИСВТ
#ifdef NO_INT_IN_INT
  #define sync_isvt(_oper_) _oper_;
#else //NO_INT_IN_INT
  #define sync_isvt(_oper_) sync(_oper_)
#endif //NO_INT_IN_INT
#ifndef IGNORE_INI
// Выделение памяти в функциях обработки ini-файла с обработкой
// ошибок
#define ini_alloc(_var_, _size_)\
  if (_var_) free(_var_);\
  _var_ = (char *)calloc(_size_, 1);\
  if (!_var_) {\
    dprintf("Error in line %d\n", __LINE__);\
    goto err_alloc;\
  }
// Обработка базовых адресов плат в ini-файле
#define proc_isaba(_var_)\
  if (cv == nv) {\
    def = 0;\
    if (!sval) {\
      def = 1;\
    } else {\
      uv = (uint)strtol(sval, NULL, 0);\
      if ((uv&0XFC0F) != 0) def = 1;\
    }\
    if (def) {\
      ini_alloc(sval, 15);\
      strcpy(sval, "0x");\
      ltoa(_var_, sval + 2, 16);\
      return 1;\
    } else {\
      _var_ = uv;\
      return 0;\
    }\
  } cv++; if (cv >= iv_count) goto err_exit;
// Обработка конфигурации схемы unio для tbi в ini-файле
#define proc_tbi(_var_)\
  if (cv == nv) {\
    def = 0;\
    if (!sval) {\
      def = 1;\
    } else {\
      uv = (uint)strtol(sval, NULL, 0);\
    }\
    if (def) {\
      ini_alloc(sval, 15);\
      strcpy(sval, "0x");\
      ltoa(_var_, sval + 2, 16);\
      return 1;\
    } else {\
      _var_ = uv;\
      return 0;\
    }\
  } cv++; if (cv >= iv_count) goto err_exit;
// Обработка конфигурации дискретных входов/выходов в ini-файле
#define proc_discr(_var_)\
  if (cv == nv) {\
    def = 0;\
    if (!sval) {\
      def = 1;\
    } else {\
      uv = (uint)strtol(sval, NULL, 0);\
      if ((uv&0x1F) > 23) def = 1;\
    }\
    if (def) {\
      ini_alloc(sval, 15);\
      sprintf(sval, "0x%04X", _var_);\
      return 1;\
    } else {\
      _var_ = uv;\
      return 0;\
    }\
  } cv++; if (cv >= iv_count) goto err_exit;
// Обработка конфигурации аналоговых входов в ini-файле
#define proc_adc(_var_)\
  if (cv == nv) {\
    def = 0;\
    if (!sval) {\
      def = 1;\
    } else {\
      iv = (int)strtol(sval, NULL, 0);\
      if ((iv < 0) || (iv > 23)) def = 1;\
    }\
    if (def) {\
      ini_alloc(sval, 15);\
      sprintf(sval, "%d", _var_);\
      return 1;\
    } else {\
      _var_ = iv;\
      return 0;\
    }\
  } cv++; if (cv >= iv_count) goto err_exit;
// Выделение частей из переменных
#endif //IGNORE_INI
#ifndef USE_CPU_COUNTER
#define HIBYTE(N) *((byte *)&(N) + 1)
#define LOBYTE(N) *((byte *)&(N))
#define HIWORD(N) *((unsigned short *)&(N) + 1)
#define LOWORD(N) *((unsigned short *)&(N))
#endif //USE_CPU_COUNTER

// ******************************************************************
// typedef-определения
// ******************************************************************
// Определение структуры с данными для ф-ции обработки сетевых
// переменных
typedef struct _landata {
  void *fn;
  word port;
  counter_t to;
  double tact;
  void *udp_buf;
  // UDP-сокет
  udp_Socket us;
  // Таймаут для UDP
  //clock_t uto;
} landata;
// Определение ф-ции обработки сетевых перменных
typedef void (* lanfn)(landata *ld);
// Структура для обработки ini-файла
typedef struct _inidata {
  // Имя переменной
  char *varname;
  // Строковое значение переменной
  char *strval;
  // Числовое значение переменной
  //long val;
  // Признак того, что переменная обработана
  long flag;
} inidata;
// Структура для ПИД-регулятора
typedef struct _pid_data {
  // Общий коэффициент передачи регулятора
  double k;
  // Коэффициент передачи интегратора
  double ki;
  // Коэффициент передачи дифференциатора
  double kd;
  // Минимальное значение на выходе
  double min;
  // Максимальное значение на выходе
  double max;
  // Предыдущее значение ошибки
  double prev_e;
  // Предпредыдущее значение ошибки
  double pp_e;
  // Предыдущее значение на выходе регулятора
  double prev_out;
  // Блокировка интегратора
  int block_int;
  // Внешняя блокировка интегратора
  int block_int_ext;
  // Сумма на интеграторе
  double int_val;
} pid_data;
// Структура для ограничителя темпа
typedef struct _ogr_temp_data {
  // Текущее значение параметра
  float cur;
  // Целевое значение параметра
  //float ref;
  // Шаг вызова ф-ции по времени, с
  float dt;
  // Крутизна, <ед. изм>/мин.
  float slope;
  // Максимальное изменение величины за шаг вызова
  float dl;
  // Предыдущее значение шага вызова ф-ции по времени, с
  float dt_prev;
  // Предыдущее значение крутизны, <ед. изм>/мин.
  float slope_prev;
  // Принудительный пересчет dl
  int recalc;
} ogr_temp_data;
#ifdef TEST
// Стуктура данных массива тестовых значений
typedef struct _tst_data {
  float time;
  float hzu;
  float hzu_f;
  long servo_cnt;
  long servo_cnt_ref;
  long servo_cnt_cur;
  float hsvd;
  float n;
  float n_ref;
} tst_data;
#endif //TEST
// Структура с данными для инерционного звена
typedef struct _fade_data {
  // x[n-1]
  double x1;
  // y[n-1]
  double y1;
  // Постоянная времени
  double t;
} fade_data;
// Структура с данными для изодромного звена
typedef struct _isodr_data {
  // k - коэффициент передачи пропорциональной части
  double k;
  // Структура данных инерционного звена
  fade_data fd;
} isodr_data;
#ifdef DETECT_N_ANOMAL
// Структура с данными для определения аномальных частот
typedef struct _n_anomal_data {
  // При первом вызове функции должна быть единица
  _in_out byte _init;
  // Перед запуском функции должна обнулятся, при детектировании
  // устанавливается в 1
  _in_out byte detect;
  // Предыдущая частота, об/мин.
  _internal float n_prev;
  // Текущая аномальная частота, об./мин
  _out float n_anomal_cur;
  // Предыдущая аномальная частота об./мин
  _out float n_anomal_prev;
  // Время появления аномальной частоты, 0xddhhmmss
  _out long time_n_anomal;
} n_anomal_data;
// Структура с данными для определения аномальных частот по датчикам
typedef struct _nx_anomal_data {
  // Данные для шести датчиков
  _internal n_anomal_data n_anomal_v[6];
} nx_anomal_data;
#endif //DETECT_N_ANOMAL


// ******************************************************************
// Предварительное определение ф-ции
// ******************************************************************
// Ф-ции обработки сетевых переменных
void lan01(landata *ld);
//   Передача на терминал и обработка прочих сетевых операции
void lan02(landata *ld);
//   Подготовка буфера для UDP
void prepare_ubuf();
#ifndef ADC_IN_LOOP
#ifndef DEBUG
// Установка своего прерывания на канал 0 таймера
void init_timer0();
// Восстановление старого прерывания на канале 0 таймера
void deinit_timer0();
#endif
#endif //ADC_IN_LOOP
// Чтение значения текущего датчика температуры РТО1
float get_t_rto1();
// Чтение значения текущего датчика температуры РТО2
float get_t_rto2();
#ifdef DBGONCPU
// Модель турбины в режиме отладки
void dbg_proc_int();
void dbg_proc();
#endif //DBGONCPU
#ifdef RES_PKT_DRV
// Процедура сброса пакетного драйвера
void res_pd_proc();
// Функция реинициализации пакетного драйвера
void reinit_packet_drv();
#endif //RES_PKT_DRV
#ifdef DEBUG
// Функция для задания тестовых значений в режиме отладки на ПК
void dbg_proc();
#endif //DEBUG
// Инерционное звено
double fade(fade_data *fd, double x);
// Отправка импульсов управления в серводрайвер
void send_srv_pulses(int pulses);
// Процедура сброса серводрайвера
void reset_servo_proc();
#ifdef CORR_HZU_ON
// Коррекция hzu при инициализации
void corr_hzu_init();
// Коррекция hzu по hsvd
void corr_hzu();
#endif //CORR_HZU_ON
#ifdef ISODR
// Изодромное звено
// id - структура данных изодромного звена
// x - входное значение
double isodr(isodr_data *id, double x);
#endif //ISODR
#ifdef NELIN
// Нелинейность по частоте перед РЧВТ
// k - коэффициент передачи в зоне нелинейности
// nb - начало зоны нелинейности, об./мин.
// ne - конец зоны нелинейности, об./мин.
// x - текущая частота, об./мин.
double nelin(double k, double nb, double ne, double x);
#endif //NELIN
#ifdef DEBUG
void ti0();
#else //DEBUG
// Мое прерывания для канала 0 таймера
void interrupt far ti0();
#endif //DEBUG
#ifdef TEST
// Инициализация теста
void test_init();
// Деинициализация теста
void test_deinit();
// Запись тестового файла
void test_save();
// Обработка теста
void test_proc();
#endif //TEST
// Выполнение начальной инициализации
void init();
// Выполнение деинициализации
void deinit();
// Первод счетчика в мс
double cnttoms(long cnt);
// Перевод мс в счетчик
long mstocnt(double ms);
// Запись сетевых переменных на флэш
void writebuf(long *abuf);
// Чтение сетевых переменных из флэш
int readbuf(long *abuf);
#ifndef IGNORE_INI
// Чтение ini-файла
void readini();
// Инициализация массива ini_data
int creat_inidata();
// Освобождение массива ini_data
void free_inidata();
// Обработка строки ini-файла
int proc_iniline(char *line);
// Очистка строки от начальных и конечных пробельных символов
char *trimstr(char *s);
// Обработка переменной ini-файла
int proc_inivar(int nv);
#endif //IGNORE_INI
// Отладочная версия inp
byte dinp(uint port);
// Отладочная версия outp
void doutp(uint port, byte val);
// Фунция для проверки макросов для UNIO - 2
void _unios(uint N);
// Функция проверки бита дискретного входа
int uniobit(uint N);
// Установка дискретных выходов UNIO
void unioout();
// Функция установки состояния дискретного выхода
void unioset(uint N, int bit);
// Функция чтения событий дискретных входов
int unioev(uint N);
// Функция для сброса событий дискретных входов
void uniorev(uint N);
// Проверяет массив записи на правильность и корректирует, в случае
// необходимости. Копирует правильные значения в буфер чтения
void filtrbuf();
// Проверяет входит ли переменная x в диапазон min - max и если не входит,
// заменяет ее значение на ближайшую границу диапазона
//template <class T>
//T bound(T x, T min, T max);
int bound(int x, int min, int max);
long bound(long x, long min, long max);
float bound(float x, float min, float max);
double bound(double x, double min, double max);
// Дополнительная фильтрация переменных при инициализации
void filtrbufex();
// Чтение счетчика канала 0 таймера 8254
uint get_timer0();
// Деинициализация UNIO
void unio_deinit();
// Чтение значений счетчиков из схемы t00
long get_t00(int i);
// Инициализация UNIO
void unio_init();
// Инициализация плат АЦП AI
void ai_init();
// Инициализация плат ИСВТ
void isvt_init();

#ifdef SENS_SEL_BLOCK_ON_VALVE_TEST
// Перевод номера из sensor_select вида в вид ИСВТ
int isvt_sensor_from_sensor_select(int a_sensor_select_num);
#endif //SENS_SEL_BLOCK_ON_VALVE_TEST

// Чтение всех каналов АЦП
void get_ai_all();
#ifndef NO_AI_FILTR
#ifdef FADC_IN_LOOP
// Фильтрация АЦП
void filter_ADC();
#endif //FADC_IN_LOOP
#endif //NO_AI_FILTR
#ifndef AI_INT
// Чтение АЦП без прерывания
void ai_int();
#endif //AI_INT
#ifdef AI_INT
// Прерывание аналоговых плат
void interrupt aii(...);
#endif //AI_INT
// Синхронизация ПИД-регулятора на текущее значение регулируемого параметра
// при выключенном регуляторе
// pd - Параметры ПИД-регулятора,
// e - ошибка на входе регулятора
// rp - значение выдаваемое на исполнительный механизм на который
// осуществляется синхронизация
void pid_reg_sync(pid_data *pd, double e, double rp);
// Синхронизация ПИД-регулятора на текущее значение регулируемого параметра
// при включенном регуляторе
// pd - Параметры ПИД-регулятора,
void pid_reg_sync(pid_data *pd);
// Зона нечувствительности для КЧ
float kch_insens(float v);
// Зона нечувствительности для РТО1
float rto1_insens(float t_r);
// Зона нечувствительности для РТО2
float rto2_insens(float t_r);
// Зона нечувствительности для РТПВ
float rtpv_insens(float t_r);
// Прерывание UNIO (t00)
void interrupt ui(...);
// Деинициализация плат АЦП AI
void ai_deinit();
// Проверка событий unio
// var - переменная для сохранения предыдущего состояния
// signal - проверяемый сигнал
// front - 0 - событие по переднему фронту, 1 - по заднему, 2 - оба фронта
int chk_event(int *var, int signal, int front);
// Обработка данных от плат AI
void ai_proc();
// Обработка данных от UNIO
void unio_proc();
// Обработка управления по ВД
void vd_proc();
// Включение соответствующих реле управляющих НД
// dir - направление НД: -1 - открытие; 1 - закрытие; 0 - на месте
void nd_exec(int dir);
// Обработка управления по НД
void nd_proc();
// ПИД-регулятор
double pid_reg(pid_data *pd, double e);
// Перевод показаний таймера 0 из отсчетов в мс
double tm0toms(uint cnt);
// Шаг дискретизации регуляторов, с
double tm0tos(uint cnt);
// Зона нечувствительности для РЧВТ
float rchvt_insens(float v);
// Экстренный останов турбины
void stop_turbine();
// Обнуление сброса серводрайвера
//void stop_res_servo();
// Проверка на аварийное состояние
void check_fail();
// Взведение АСК
void vzv_ask();
// Выбор текущего датчика Pнто
void pnto_select();
// Обработка кнопок ВД
void k_vd_proc();
// Установка таймаута t в мс по переменной counter
void set_to_cnt_t(counter_t &to, double t);
// Установка следующего таймаута t в мс по переменной counter
void next_to_cnt_t(counter_t &to, double t);
// Проверка таймаута в мс по переменной counter
int test_to_cnt_t(counter_t to);
// Зона нечувствительности для РМ
float rm_insens(float v);
// Ограничитель темпа
float ogr_temp(ogr_temp_data *otd, float v);
// Ограничитель по больше/меньше РМ
float rm_bol_men(float *p, float v);
// Ограничитель по больше/меньше РТО1
float rto1_bol_men(float *p, float v);
// Ограничитель по больше/меньше РТО2
float rto2_bol_men(float *p, float v);
// Ограничитель по больше/меньше РТПВ
float rtpv_bol_men(float *p, float v);
// Ограничитель по больше/меньше РТПВ
void k_nd_proc();
// Выдача команды ИСВТ на взведение АСК
void reset_fail();
// Обработка данных плат ИСВТ
int isvt_proc();
// Взведение i-го золотника (0-2)
void vzv_zz(int i);
// Взведение всех золотников
void vzv_zz_all();
// Сброс i-го золотника (0-2)
void sbr_zz(int i);
// Тестирование i-го золотника (0-2)
void tst_zz(int i);
// Расхаживание клапанов
void valve_test();
// Выбор текущих датчиков частоты ИСВТ
void select_isvt_sensor();
// Установка счетчика энкодера
void set_servo();
// Чтение счетчика энкодера
void get_servo();
// Преобразование количества импульсов серводрайвера в градусы
float scnt_to_grad(long scnt);
// Преобразование градусов в количества импульсов серводрайвера
long grad_to_scnt(float grad);
// Блокировка всех включенных контуров регулирования с самоблокировкой
void samo_bl_all();
// Отслеживание изменения переменных
int change_var(byte *_init, int *xp, int x);
int change_var(byte *_init, long *xp, long x);
int change_var(byte *_init, float *xp, float x);
int change_var(byte *_init, double *xp, double x);
// Отображение времени цикла
void show_tloop();
// Вывод отладочной информации
void dbg_info();
// Выдача текущего времени в формате: 0xddhhmmss
long get_time_to_long();
#ifdef DETECT_N_ANOMAL
// Детектирование общей аномальной частоты
void n_anomal_proc(
  // Исследуемая частота, об./мин.
  _in float an,
  // Входные/выходные данные
  _in_out n_anomal_data *a_n_anomal
);
// Детектирование аномальной частоты по конкретным датчикам
void nx_anomal_proc(
  // Массив частот от 6 датчиков, *10 об./мин.
  _in const uint *anm[],
  // При первом вызове функции должна быть единица
  _in_out byte &_init,
  // Текущая аномальная частота датчика, об./мин
  _out float &a_nx_anomal_cur,
  // Предыдущая аномальная частота датчика, об./мин
  _out float &a_nx_anomal_prev,
  // Время появления аномальной частоты датчика, 0xddhhmmss
  _out long &a_time_nx_anomal,
  // Время появления аномальной частоты датчика, 0xddhhmmss
  _out long &a_ind_nx_anomal,
  // Внутренние данные
  _internal nx_anomal_data *nx_anomal_a
);
#endif //DETECT_N_ANOMAL
// Vyvod otladochnoi informacii po tipy puts
void dputs(const char *s);
// Vyvod otladochnoi informacii po tipy printf
void dprintf(const char *format, ...);
// Vyvod otladochnoi informacii
void doutstr(const char *s);

// ******************************************************************
// Переменные недоступные по сети
// ******************************************************************
// IP-адрес терминала 1
//longword t1ip = inet_addr("192.168.0.3");
// Установить в единицу для выхода из программы
int stop_loop = 0;
// WDT vkljuchen pri vyhode
int exit_wdt_on = 0;
// Cтруктура с данными для ф-ции обработки сетевых переменных
landata ldata;
// Ф-ции обработки сетевых перменных
lanfn lanex;
// Cтруктура с данными для ф-ции обработки сетевых переменных
// по дополнительному порту
landata ldata2;
// UDP soket dlya zapisi logov
udp_Socket udp_log_sock;
// Старый обработчик прерывания канала 0 таймера
void (interrupt far *oti0)();
// Старый обработчик прерывания AI
void (interrupt far *oaii)();
// Старый обработчик прерывания UNIO
void (interrupt far *oui)();
// Счетчик инкрементирующийся каждое прерывание канала 0 таймера
volatile long counter = 0;
// Признак что в данный момент происходит фильтарция сетевых
// переменных
//int flag_filtr = 0;
#ifdef DEBUG
// Файл для хранения сетевых переменных
char *lan_var_fn = "C:\\lan.dat";
// Временный файл для хранения сетевых переменных
char *tmp_lan_var_fn = "C:\\lan.tmp";
// Имя ini-файла
char *ini_fn = "C:\\tb.ini";
// Имя временного ini-файла
char *tmp_ini_fn = "C:\\tb.tmp";
// Путь для файлов теста
char *test_path = "C:\\";
#else //DEBUG
// Путь для файлов теста
char *test_path = "D:\\";
#ifdef PAR_IN_CPU_FLASH
// Файл для хранения сетевых переменных
char *lan_var_fn = "C:\\lan.dat";
// Временный файл для хранения сетевых переменных
char *tmp_lan_var_fn = "C:\\lan.tmp";
// Имя ini-файла
char *ini_fn = "C:\\tb.ini";
// Имя временного ini-файла
char *tmp_ini_fn = "C:\\tb.tmp";
#else //PAR_IN_CPU_FLASH
// Файл для хранения сетевых переменных
char *lan_var_fn = "D:\\lan.dat";
// Временный файл для хранения сетевых переменных
char *tmp_lan_var_fn = "D:\\lan.tmp";
// Имя ini-файла
char *ini_fn = "D:\\tb.ini";
// Имя временного ini-файла
char *tmp_ini_fn = "D:\\tb.tmp";
#endif //PAR_IN_CPU_FLASH
#endif //DEBUG
// Размер буфера для ini-файла
//ulong size_ini = 30000;
// Максимальная длина строки ini-файла
int sz_ini_line = 1000;
// Данные необходимые для обработки ini-файла
inidata far *ini_data;
// Максимальное число переменных в ini-файле
int iv_count = 100;
// Текущее число переменных в ini-файле
int iv_cnt;
// Массив unio
uint unio[2] = {0x110, 0x120};
// Смещение адресов для матриц UNIO
const uint uniom[4] = {0xA000, 0xA400, 0xA800, 0xAC00};
// Базовый адрес платы UNIO A1.6
uint &unio_a1_6 = unio[0];
// Базовый адрес платы UNIO A1.7
uint &unio_a1_7 = unio[1];
// Базовый адрес платы AI A1.3
uint ai_a1_3 = 0x130;
// Базовый адрес платы AI A1.4
uint ai_a1_4 = 0x140;
// Базовый адрес платы AI A1.5
uint ai_a1_5 = 0x150;
#ifdef DBGONCPU
// Базовый адрес платы ИСВТ A1.8
uint isvt_a1_8 = 0xD220;
// Базовый адрес платы ИСВТ A1.9
uint isvt_a1_9 = 0xB220;
// Базовый адрес платы ИСВТ A1.10
uint isvt_a1_10 = 0xC220;
// Базовый адрес платы ИСВТ A1.11
uint isvt_a1_11 = 0xA220;
#else //DBGONCPU
// Базовый адрес платы ИСВТ A1.8
uint isvt_a1_8 = 0xA220;
// Базовый адрес платы ИСВТ A1.9
uint isvt_a1_9 = 0xB220;
// Базовый адрес платы ИСВТ A1.10
uint isvt_a1_10 = 0xC220;
// Базовый адрес платы ИСВТ A1.11
uint isvt_a1_11 = 0xD220;
#endif //DBGONCPU
// Массив БА ИСВТ
uint *isvt_ba[4] = {&isvt_a1_8, &isvt_a1_9, &isvt_a1_10, &isvt_a1_11};
uint tbi[8] = {
0xFF00, //A2
0xFF01, //A3
0x0202, //A4
0xFF03, //A5
0xFF04, //A6
0xFF05, //A7
0xFF06, //A8
0xFF07, //A9
};
// Конфигурация для платы TBI A2
uint &tbi_a2 = tbi[0];
// Конфигурация для платы TBI A3
uint &tbi_a3 = tbi[1];
// Конфигурация для платы TBI A4
uint &tbi_a4 = tbi[2];
// Конфигурация для платы TBI A5
uint &tbi_a5 = tbi[3];
// Конфигурация для платы TBI A6
uint &tbi_a6 = tbi[4];
// Конфигурация для платы TBI A7
uint &tbi_a7 = tbi[5];
// Конфигурация для платы TBI A8
uint &tbi_a8 = tbi[6];
// Конфигурация для платы TBI A9
uint &tbi_a9 = tbi[7];
// Дискретные входы/выходы
// ****
// Дискретный выход P55 НД больше (A5-0)
uint ndb = 0x0300;
// Дискретный выход P55 НД меньше (A5-1)
uint ndm = 0x0301;
// Дискретный выход P55 SRV_S-ON (A5-4)
uint srv_s_on = 0x0304;
// Дискретный выход P55 SRV_ALM-RST (A5-5)
uint srv_alm_rst = 0x0305;
// Дискретный выход P55 Авария (A5-6)
uint avariya = 0x0306;
#ifdef RES_PKT_SWITCH
// Дискретный выход P55 Выключение свитча (A5-2)
uint switch_off = 0x0302;
#endif //RES_PKT_SWITCH
// Дискретный вход P55 SRV_ALM (A2-0)
uint srv_alm = 0x1000;
// Дискретный вход P55 SRV_ALO1 (A2-1)
uint srv_alo1 = 0x0001;
// Дискретный вход P55 SRV_ALO2 (A2-2)
uint srv_alo2 = 0x0002;
// Дискретный вход P55 SRV_ALO3 (A2-3)
uint srv_alo3 = 0x0003;
// Дискретный вход P55 ДЗЗ1 (A2-4)
uint dzz1 = 0x0004;
// Дискретный вход P55 ДЗЗ2 (A2-5)
uint dzz2 = 0x0005;
// Дискретный вход P55 ДЗЗ3 (A2-6)
uint dzz3 = 0x0006;
// Дискретный вход P55 КГ (A2-7)
uint kg = 0x1007;
// Дискретный вход P55 СЗ (A2-8)
uint sz = 0x0008;
// Дискретный вход P55 СНД откр. (A2-9)
uint snd_zakr = 0x0009;
// Дискретный вход P55 СНД закр. (A2-10)
uint snd_otkr = 0x000A;
// Дискретный вход P55 АСК откр. (A2-11)
uint ask_otkr = 0x000B;
// Дискретный вход P55 АСК закр. (A2-12)
uint ask_zakr = 0x000C;
// Дискретный вход P55 кнопка ВД бол. (A2-13)
uint k_vd_bol = 0x000D;
// Дискретный вход P55 кнопка ВД мен. (A2-14)
uint k_vd_men = 0x000E;
// Дискретный вход P55 кнопка Стоп (A2-15)
uint k_stop = 0x000F;
// Дискретный вход P55 Режим О (A2-16)
uint rezim_o = 0x0010;
// Дискретный вход P55 Режим П (A2-17)
uint rezim_p = 0x0011;
// Дискретный вход P55 Режим Т (A2-18)
uint rezim_t = 0x0012;
// Дискретный вход P55 220_ON ШУ2 (A2-19)
uint shu2_220_on = 0x0013;
// Дискретный вход P55 кнопка НД бол. (A2-20)
uint k_nd_bol = 0x0014;
// Дискретный вход P55 кнопка НД мен. (A2-21)
uint k_nd_men = 0x0015;
// Дискретный вход P55 кн. взведение АСК (A2-22)
uint k_vzv_ask = 0x0016;
// Дискретный вход P55 220 подано в ШУ1 (A2-23)
uint shu1_220_on = 0x0017;
// Дискретный вход P55 НД авт. (A3-0)
uint nd_avt = 0x0100;
// Дискретный вход P55 НД руч. (A3-1)
uint nd_ruch = 0x0101;
// Дискретный вход P55 ВД авт. (A3-2)
uint vd_avt = 0x0102;
// Дискретный вход P55 ВД руч. (A3-3)
uint vd_ruch = 0x0103;
// Дискретный вход P55 ВД авар. (A3-4)
uint vd_avar = 0x0104;
// Дискретный вход P55 исправность БП 24 В ШУ2 (A3-5)
uint bp24v_shu2 = 0x0105;
// Дискретный вход P55 исправность БП 24 В 1-го дискретного ШУ2 (A3-6)
uint bp24v1 = 0x0106;
// Дискретный вход P55 исправность БП 24 В 2-го дискретного ШУ2 (A3-7)
uint bp24v2 = 0x0107;
// Дискретный вход P55 исправность БП 24 В 3-го аналогового ШУ2 (A3-8)
uint bp24v3 = 0x0108;
#ifndef UNIO_SINGLE
// Дискретный вход P55 исправность БП 5 В для ИСВТ1 (A6-0)
uint bp5v1 = 0x0400;
// Дискретный вход P55 исправность БП 5 В для ИСВТ2 (A6-1)
uint bp5v2 = 0x0401;
// Дискретный вход P55 исправность БП 5 В для ИСВТ3 (A6-2)
uint bp5v3 = 0x0402;
#else //UNIO_SINGLE
// Дискретный вход P55 исправность БП 5 В для ИСВТ1 (A6-0)
uint bp5v1 = 0x0110;
// Дискретный вход P55 исправность БП 5 В для ИСВТ2 (A6-1)
uint bp5v2 = 0x0111;
// Дискретный вход P55 исправность БП 5 В для ИСВТ3 (A6-2)
uint bp5v3 = 0x0112;
#endif //UNIO_SINGLE
// Дискретный вход P55 сброс серводрайвера (A3-9)
uint sbros_srv = 0x0109;
// Дискретный вход P55 переключатель загрузки (A3-10)
uint boot_sw = 0x010A;
// ****
// Аналоговые входы
// ****
// Массив значений АЦП
int adc[24];
#ifdef NEW_FADC
// Данные для инерционных звеньев фильтров АЦП
fade_data adcf[24];
#else //NEW_FADC
// Фильтрованный массив АЦП
double adcf[24];
#endif //NEW_FADC
#ifdef FADC_IN_LOOP
byte adc_flags[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0};
#endif //FADC_IN_LOOP
// Вакуум в конденсаторе
int vk_adc = 0;
// Давление в регулирующей ступени
int prs_adc = 1;
// Давление масла на регулирование
int prm_adc = 2;
// Давление импульсного масла
int pim_adc = 3;
// Давление в нижнем отборе 1
int pnto1_adc = 4;
// Давление в нижнем отборе 2
int pnto2_adc = 5;
// Давление сетевой воды
int psv_adc = 6;
// Давление острого пара
int pt_adc = 7;
// Положение СВД
int hsvd_adc = 8;
// Положение СНД
int hesnd_adc = 9;
// Температура подпиточной воды
int tpv_adc = 10;
// Температура сетевой воды 1
int t1_adc = 11;
// Температура сетевой воды 2
int t2_adc = 12;
// Температура обратной сетевой воды
int tobr_adc = 13;
// Электрическая мощность
int ne_adc = 14;
// Массив номеров входов АЦП
int *num_adc[15] = {&pim_adc, &prm_adc, &ne_adc, &prs_adc, &pt_adc,
  &pnto1_adc, &pnto2_adc, &tpv_adc, &t1_adc, &t2_adc, &tobr_adc,
  &vk_adc, &psv_adc, &hesnd_adc, &hsvd_adc};
// ****
// Переменная для отладки ф-ций доступа к дискретным входам/выходам
byte vport = 0x21;
#ifdef NOP
// Пределы измерений аналоговых датчиков
float pim_smin = 0.f;
float pim_smax = 20.f;
float prm_smin = 0.f;
float prm_smax = 20.f;
float ne_smin = 0.f;
float ne_smax = 138.f;
float prs_smin = 0.f;
float prs_smax = 92.f;
float pt_smin = 0.f;
float pt_smax = 140.f;
float pnto1_smin = -1.f;
float pnto1_smax = 1.5f;
float pnto2_smin = -1.f;
float pnto2_smax = 1.5f;
float tpv_smin = 0.f;
float tpv_smax = 100.f;
float t1_smin = 0.f;
float t1_smax = 200.f;
float t2_smin = 0.f;
float t2_smax = 200.f;
float tobr_smin = 0.f;
float tobr_smax = 200.f;
float vk_smin = -1.f;
float vk_smax = 0.f;
float psv_smin = 0.f;
float psv_smax = 10.f;
float hesnd_smin = 0.f;
float hesnd_smax = 250.f;
float hsvd_smin = 0.f;
float hsvd_smax = 250.f;
#endif
float hzu_smin = 0.f;
float hzu_smax = 90.f;
float n_min = 0.f;
float n_max = 3500.f;
//float dn_min = -3000.f;
//float dn_max = 3000.f;
// Счетчик положения серводрайвера
long servo_cnt = 0;
// Задатчик положения серводрайвера
//long servo_cnt_ref = 0;
// Количество импульсов серводрайвера которому соответствует 90 гр.
long d90_cnt = 512;
// Максимальная длительность прерывания по каналу 0 таймера
uint t_int_t0 = TIMER0_CNT;
// Режим работы с UNIO
// 0 - норма
// 1 - тест
// 2 - авария
uint unio_mode = 0;
// Режим работы с AI
// 0 - норма
// 1 - тест
// 2 - авария
uint ai_mode = 1;
// Флаг необходимый при инициализации плат AI
int ai_end = 0;
// Отличие hesnd от hesnd_ref меньше которого, регулирование не происходит, мм
//float hesnd_ref_delta = 4.f;
// Задание по hesnd
//float hesnd_ref;
// Задание по hsvd
//float hsvd_ref;
// Включение ВД
int start_vd = 0;
// Включение НД
int start_nd = 0;
// Пуск регулятора ВД
int start_reg_vd = 0;
// Пуск регулятора НД
int start_reg_nd = 0;
// Данные для регулятора РЧВТ
pid_data rchvt_reg;
// Данные для регулятора ЭСЗО
pid_data eszo_reg;
// Данные для регулятора ВД
pid_data hsvd_reg;
// Изодромное звено по частоте на входе РЧВТ
isodr_data rchvt_isodr;
// Шаг дискретизации регуляторов, мс
double dtms = 20.;
// Шаг теста, мс (Задается через dt_test)
double dtms_test;
// Шаг дискретизации регуляторов, с
double dt = dtms/1e3;
#ifdef DEBUG
// Шаг отладки, с
double dt_dbg = tm0tos(TIMER0_CNT);
#endif //DEBUG
// Таймаут регуляторов
counter_t to_dt;
// Минимально допустимое задание по частоте
float n_ref_min = 0.f;
// Максимально допустимое задание по частоте
float n_ref_max = 3200.f;
// Авария турбины
int fail = 0;
// Текущее pnto
float pnto;
// Часть hsvd, образуемая ЭСЗО, РМ или РТО1
float hsvd2 = 0.f;
// Параметры для кнопок ВД
float k_vd_k = pow(2, dt);
float k_vd_b = 0.01f;
float k_vd_c = 0.f;
float k_vd_d = 0.f;
float *k_vd_par = NULL;
float k_vd_par1;
float *k_vd_min = NULL;
float *k_vd_max = NULL;
counter_t vd_dig_to;
counter_t vd_dec_to;
int vd_dig_f = 0;
int vd_dec_f = 0;
// Параметры для кнопок НД
float k_nd_k = pow(2, dt);
float k_nd_b = 0.01f;
float k_nd_c = 0.f;
float k_nd_d = 0.f;
float *k_nd_par = NULL;
float k_nd_par1;
float *k_nd_min = NULL;
float *k_nd_max = NULL;
counter_t nd_dig_to;
counter_t nd_dec_to;
int nd_dig_f = 0;
int nd_dec_f = 0;
// Переменная по которой закрывается СВД при сбросах нагрузки
int kg_off_svd_zakr = 0;
// Событие по которому закрывается СВД при сбросах нагрузки
int kg_off_svd_zakr_e = 0;
// Переменная по которой закрывается ЭСНД при сбросах нагрузки
int kg_off_esnd_zakr = 0;
// Интервал смены цифры кнопками ВД, НД, мс
float t_dig = 1000.f;
// Интервал смены декады кнопками ВД, НД, мс
float t_dec = 4000.f;
// Данные ограничителя темпа задания РМ
ogr_temp_data rm_ot;
// Данные ограничителя темпа задания РТО1
ogr_temp_data rto1_ot;
// Данные ограничителя темпа задания РТО2
ogr_temp_data rto2_ot;
// Данные ограничителя темпа задания РТПВ
ogr_temp_data rtpv_ot;
// Данные для регулятора РМ
pid_data rm_reg;
// Данные для регулятора РТО1
pid_data rto1_reg;
// Данные для регулятора РТО2
pid_data rto2_reg;
// Данные для регулятора РТПВ
pid_data rtpv_reg;
// Предыдущее значение для ограничителя по больше/меньше РМ
float rm_bol_men_prev = 1e6f;
// Предыдущее значение для ограничителя по больше/меньше РТО1
float rto1_bol_men_prev = 1e6f;
// Предыдущее значение для ограничителя по больше/меньше РТО1
float rto2_bol_men_prev = 1e6f;
// Предыдущее значение для ограничителя по больше/меньше РТПВ
float rtpv_bol_men_prev = 1e6f;
// РЧВТ включены
int rchvt_on = 0;
// ЭСЗО включены
int eszo_on = 0;
// РМ включен
int rm_on = 0;
// РТО1 включен
int rto1_on = 0;
// РТО2 включен
int rto2_on = 0;
// РТПВ включен
int rtpv_on = 0;
// КЧ (компенсатор частоты) включен
int kch_on = 0;
// Запуск процедуры взведения АСК
//int vzv_ask_ex = 0;
// Шаг таймера AI, мкс
uint dt_tmr_ai = 50;
// Постоянная времени фильтра АЦП в с
//float t_ai_filtr = .08f;
// Постоянная времени фильтра АЦП в отсчетах АЦП
double n_ai_filtr;
// = t_ai_filtr/(16*tm0tos(TIMER0_CNT));
// Значение hsvd, которое в случае необходимости эмулируется
//float hsvd_2 = 0.f;
// Положения серводрайвера, которое уже отправлено на сервордрайвер
long servo_cnt_cur = 0L;
// Задание по hzu от РМ, РТО1 и ручное
long servo_cnt_ref1 = 0L;
// Коэффициент преобразования hzu из градусов в целое
float k_cnt_hzu = d90_cnt/90.;
// Разрешение управления серводрайвером
//int pulse_en = 0;
// Число попыток сбросить серводрайвер
//int res_srv_cnt = 3;
// Значения датчиков частоты в 1/10 об./мин.
uint n1 = 0;
uint n2 = 0;
uint n3 = 0;
uint n4 = 0;
uint n5 = 0;
uint n6 = 0;
// Массивы ссылающийся на датчики частоты
uint *n1m[3] = {&n1, &n3, &n5};
uint *n2m[3] = {&n2, &n4, &n6};
uint *nm[6] = {&n1, &n2, &n3, &n4, &n5, &n6};
// Массивы производных частоты ИСВТ
int dn1m[3];
int dn2m[3];
// Массив выбора датчиков ИСВТ
// 0 - датчик 1; 1 - датчик 2
int sensor_select[3];
// Ожидание переключения на другой датчик ИСВТ
int wait_switch_isvt_sensor = 0;
// Ожидание завершения интервала проверки датчиков ИСВТ
int wait_interval_isvt = 0;

// Количество датчиков частоты на всех ИСВТ
const int sensor_count = 6;
// Номер первого датчика для sensor_select
const int sensor_first = 0;
// Номер второго датчика для sensor_select
const int sensor_second = 1;
// Номер датчика для sensor_select, если датчик не выбран
const int sensor_none = -1;
// Количество каналов ИСВТ для автомата безопасности (контроль частоты)
const int isvt_freq_count = 3;
// Номер первого датчика для ИСВТ
const int isvt_sensor_first = 1;
// Номер второго датчика для ИСВТ
const int isvt_sensor_second = 2;

// Режим работы с ИСВТ
// 0 - норма
// 1 - тест
// 2 - авария
int isvt_mode = 0;
// Таймауты необходимые при смене датчика ИСВТ
long to_isvt1 = 0;
long to_isvt2 = 0;
// Максимальнное ожидание переключения датчиков ИСВТ
long max_delay_isvt = 100;
// Таймаут для автосброса ИСВТ
long isvt_test_to = 0;
// Время таймаута для автосброса ИСВТ, мс
float isvt_test_time = 10000.f;
// Признак запуска таймаута ИСВТ
int isvt_test_flag = 0;
// Расхождение датчиков в отн. ед., больше которых происходит
// перевыбор датчиков
//float isvt_sens_delta = 0.15f;
float isvt_sens_delta = 1.6f;
// Отклонение датчиков hesnd и hsvd от минимальных значений при
// которых они считаются закрытыми в отн. ед.
float h_delta = 0.05f;
#ifdef DBGONCPU
// Кнопки ВД и НД для отладки
int dbg_nd_bol = 0;
int dbg_nd_men = 0;
int dbg_vd_bol = 0;
int dbg_vd_men = 0;
#endif //DBGONCPU
// Число циклов через которое выполняется запись во флэш
long cnt_flash = 100;
// Последнее направление вращения серводрайвера
//int last_srv_sign = 0;
// Сдвиг относительно границ перемещения золотника управления
float delta_srv = 5.f;
// Пуск турбины по АСК
int start_ask = 0;
// Закрытие ВД
int vd_zakr = 0;
// Размер буфера для UDP
uint udp_buf_size = 16384;
// Буфер для UDP
void *udp_buf = NULL;
// Переменнная для отсчета интервала между широковещательными пакетами UDP
long to_tact_udp;
// Признак взведения золотника защиты 1
int dzz1_v = 0;
// Признак взведения золотника защиты 2
int dzz2_v = 0;
// Признак взведения золотника защиты 3
int dzz3_v = 0;
// Признак нажатия на кнопку ВД больше
int k_vd_bol_v = 0;
// Признак нажатия на кнопку ВД меньше
int k_vd_men_v = 0;
// Признак нажатия на кнопку НД больше
int k_nd_bol_v = 0;
// Признак нажатия на кнопку НД меньше
int k_nd_men_v = 0;
// Признак нажатия на кнопку Стоп
int k_stop_v = 0;
// Признак нажатия на кнопку Сброс серводрайвера
int sbros_srv_v = 0;
// Признак нажатия на кнопку Взведение АСК
int k_vzv_ask_v = 0;
// Состояние контакта генератора
int kg_v = 0;
// Состояние сигнала защит
int sz_v = 0;
// Событие по кнопке Стоп
int k_stop_e = 0;
// Событие по кнопке Сброс серводрайвера
int sbros_srv_e = 0;
// Событие по кнопке Взведение АСК
int k_vzv_ask_e = 0;
// Событие по контактe генератора
int kg_e = 0;
// Событие по сигналe защит
int sz_e = 0;
// Срабатывание коцевика АСК открыт
int ask_otkr_v = 0;
// Срабатывание коцевика АСК закрыт
int ask_zakr_v = 0;
// Срабатывание коцевика СНД открыт
int snd_otkr_v = 0;
// Срабатывание коцевика СНД закрыт
int snd_zakr_v = 0;
// Событие по коцевику АСК открыт
int ask_otkr_e = 0;
// Событие по коцевику АСК закрыт
int ask_zakr_e = 0;
// Событие по коцевику СНД открыт
int snd_otkr_e = 0;
// Событие по коцевику СНД закрыт
int snd_zakr_e = 0;
// Признак тревоги серводравера
int srv_alm_v = 0;
// 1-й бит кодированной ошибки серводрайвера
int srv_alo1_v = 0;
// 2-й бит кодированной ошибки серводрайвера
int srv_alo2_v = 0;
// 3-й бит кодированной ошибки серводрайвера
int srv_alo3_v = 0;
// Признак неисправности всех датчиков частоты
int n_all_bad_isvt = 0;
// Необходимость записи переменных во флэш
int need_writebuf = 0;
// Необходимость сброса серводрайвера
int reset_servo = 0;
#ifdef TEST
// Размер массива в байтах
long size_arr_tst = 200L*1024L;
// Массив для теста
tst_data huge *arr_tst;
// Счетчик в начале теста
long st_counter = 0L;
// Признак записи файла
int test_save_f = 0;
#endif //TEST
const char *rchvt_t = "Hzu";
// Признак ожидания коррекции при инициализации
int init_corr = 1;
// Маска выходных сигналов UNIO
byte unio_mask = 0;
// Переменная для блокировки сброса золотников во время расхаживания
int valve_test_proc = 0;
// Минимальное значение hzu в отсчетах серводрайвера
long servo_cnt_min = 0;
// Максимальное значение hzu в отсчетах серводрайвера
long servo_cnt_max = 0;
// Включение коррекции hzu
int corr_hzu_en = 0;
// Компенсируемое рассогласование по заданию на серводрайвер
long delta_cnt_corr = 0;
// Таймаут сброса переменной srv_bad_pos
long to_res_sbp = 0L;
// Время через которое сбрасывается переменная srv_bad_pos
double t_res_sbp = 1000.;
// Признак необходимости сброса серводрайвера во время инициализации
int init_res_servo_f = 0;
// Фильтр частоты вращения турбины для детектора сброса нагрузки
fade_data filtr_n;
// Постоянная времени фильтра частоты вращения турбины, c
double t_filtr_n = .1;
// Текущая тестируемая плата ИСВТ
int cur_test_isvt = -1;
// Разрешение сетевого обмена в corr_hzu_init
int lan_en = 0;
// Коэффициент при умножении на который получается дельта для интегратора
double k_d_pid = 0.1;
#ifdef DBGONCPU
int dbg_k_vzv_ask = 0;
int dbg_stop = 0;
#endif //DBGONCPU
// Переменная для точки останова в отладке
int brk_pnt = 0;
// Блокирование обработки
int bl_proc = 0;
#ifdef ND_WITHOUT_SENS
// Направление вращения двигателя НД
char nd_dir = 0;
// Режим работы двигателя НД: 0 - постоянный, 1 - прерывистый
char nd_mode = 0;
#endif //ND_WITHOUT_SENS
// Priznak otkrytiya UDP soketa dlya loga
char udp_log_opened = 0;

// ******************************************************************
// Массивы сетевых переменных
// ******************************************************************
// Основной массив
long buf[CNT_VAR];
// Массив записи
long wbuf[CNT_VAR];
// Массив сравнения
char vbuf[CNT_VAR];
// Массив для UDP
long ubuf[CNT_VAR + 2];

// Минимум диапазона измерения датчика давления импульсного масла (pim)
float &pim_smin = *(float *)&buf[1];
// Максимум диапазона измерения датчика давления импульсного масла (pim)
float &pim_smax = *(float *)&buf[2];
// Минимум диапазона измерения датчика давления масла на регулирование (prm)
float &prm_smin = *(float *)&buf[3];
// Максимум диапазона измерения датчика давления масла на регулирование (prm)
float &prm_smax = *(float *)&buf[4];
// Минимум диапазона измерения датчика активной мощности (ne)
float &ne_smin = *(float *)&buf[5];
// Максимум диапазона измерения датчика активной мощности (ne)
float &ne_smax = *(float *)&buf[6];
// Минимум диапазона измерения датчика давления в регулирующей ступени (prs)
float &prs_smin = *(float *)&buf[7];
// Максимум диапазона измерения датчика давления в регулирующей ступени (prs)
float &prs_smax = *(float *)&buf[8];
// Минимум диапазона измерения датчика давления острого пара (pt)
float &pt_smin = *(float *)&buf[9];
// Максимум диапазона измерения датчика давления острого пара (pt)
float &pt_smax = *(float *)&buf[10];
// Минимум диапазона измерения датчика давления в нижнем отборе первого (pnto1)
float &pnto1_smin = *(float *)&buf[11];
// Максимум диапазона измерения датчика давления в нижнем отборе первого (pnto1)
float &pnto1_smax = *(float *)&buf[12];
// Минимум диапазона измерения датчика давления в нижнем отборе второго (pnto2)
float &pnto2_smin = *(float *)&buf[13];
// Максимум диапазона измерения датчика давления в нижнем отборе второго (pnto2)
float &pnto2_smax = *(float *)&buf[14];
// Минимум диапазона измерения датчика температуры подпиточной воды (tpv)
float &tpv_smin = *(float *)&buf[15];
// Максимум диапазона измерения датчика температуры подпиточной воды (tpv)
float &tpv_smax = *(float *)&buf[16];
// Минимум диапазона измерения датчика температуры сетевой воды 1 (t1)
float &t1_smin = *(float *)&buf[17];
// Максимум диапазона измерения датчика температуры сетевой воды 1 (t1)
float &t1_smax = *(float *)&buf[18];
// Минимум диапазона измерения датчика температуры сетевой воды 2 (t2)
float &t2_smin = *(float *)&buf[19];
// Максимум диапазона измерения датчика температуры сетевой воды 2 (t2)
float &t2_smax = *(float *)&buf[20];
// Признак срабатывания защиты по частоте
long &n_error = buf[21];
// Признак  нажатия на кнопку СТОП
long &stop_error = buf[22];
// Положение тумблера режимов ОПТ
long &opt_sw = buf[23];
// Положение тумблера ВД
long &vd_sw = buf[24];
// Положение тумблера НД
long &nd_sw = buf[25];
// Включение режима настройки граничных значений hzu
long &mode_hzu = buf[26];
// Пуск/останов турбины
long &start_turbine = buf[27];
// Режим тестирования датчиков частоты
long &mode_testn = buf[28];
// Статус плат ИСВТ
long &isvt_st = buf[29];
// Неисправные каналы защиты по частоте вращения
long &n_bad = buf[30];
// Статус золотников защиты
long &zz_st = buf[31];
// Выбор текущего давления Pнто
long &pnto_sel = buf[32];
// Давление импульсного масла
float &pim = *(float *)&buf[33];
// Давление масла на регулирование
float &prm = *(float *)&buf[34];
// Давление в регулирующей ступени
float &prs = *(float *)&buf[35];
// Давление острого пара
float &pt = *(float *)&buf[36];
// Давление в нижнем отборе 1
float &pnto1 = *(float *)&buf[37];
// Давление в нижнем отборе 2
float &pnto2 = *(float *)&buf[38];
// Температура подпиточной воды
float &tpv = *(float *)&buf[39];
// Температура сетевой воды 1
float &t1 = *(float *)&buf[40];
// Температура сетевой воды 2
float &t2 = *(float *)&buf[41];
// Температура обратной сетевой воды
float &tobr = *(float *)&buf[42];
// Давление сетевой воды
float &psv = *(float *)&buf[43];
// Положение СНД
float &hesnd = *(float *)&buf[44];
// Положение сервомотора высокого давления - hСВД - текущее
float &hsvd = *(float *)&buf[45];
// Положение сервомотора высокого давления - hСВД - минимальное
float &hsvd_min = *(float *)&buf[46];
// Положение сервомотора высокого давления - hСВД - максимальное
float &hsvd_max = *(float *)&buf[47];
// Текущий заданный угол сервомотора
float &hzu = *(float *)&buf[48];
// Минимальный угол сервомотора
float &hzu_min = *(float *)&buf[49];
// Максимальный угол сервомотора
float &hzu_max = *(float *)&buf[50];
// Электрическая мощность
float &ne = *(float *)&buf[51];
// Результат проверки датчиков частоты (результат расхаживания клапанов)
long &testn_st = buf[52];
// Наиболее достоверная текущая частота
float &n = *(float *)&buf[53];
// Текущая частота 1
float &n12 = *(float *)&buf[54];
// Текущая частота 2
float &n34 = *(float *)&buf[55];
// Текущая частота 3
float &n56 = *(float *)&buf[56];
// Порог по частоте 1
float &n12t = *(float *)&buf[57];
// Порог по частоте 2
float &n34t = *(float *)&buf[58];
// Порог по частоте 3
float &n56t = *(float *)&buf[59];
// Зона нечувствительности РЧВТ, об./мин.
float &rchvt_ins = *(float *)&buf[60];
// Зона нечувствительности РМ, МВт.
float &rm_ins = *(float *)&buf[61];
// Зона нечувствительности РТО1,  C.
float &rto1_ins = *(float *)&buf[62];
// Зона нечувствительности РТО2,  C.
float &rto2_ins = *(float *)&buf[63];
// Зона нечувствительности РТПВ,  C.
float &rtpv_ins = *(float *)&buf[64];
// Защитное значение превышения частоты n над заданием
// n_ref, при котором происходит сброс значения на интеграторе [об/мин.]
float &n_defense = *(float *)&buf[65];
// Задание по частоте вращения турбины
float &n_ref = *(float *)&buf[66];
// Пропорциональный коэффициент РЧВТ
float &reg_rchvt_k = *(float *)&buf[67];
// Интегральный коэффициент РЧВТ
float &reg_rchvt_ki = *(float *)&buf[68];
// Дифференциальный коэффициент РЧВТ
float &reg_rchvt_kd = *(float *)&buf[69];
// Начало зоны нечувствительности по частоте РЧВТ
float &n_ns_rchvt_b = *(float *)&buf[70];
// Конец зоны нечувствительности по частоте РЧВТ
float &n_ns_rchvt_e = *(float *)&buf[71];
// Причина невзведения АСК
long &ask_vzv_err = buf[72];
// Коэффициент преобразования частоты вращения в hСВД для компенсатора
// частоты (КЧ) [мм/(об./мин.)]
float &a_hsvd = *(float *)&buf[73];
// Порог включения ЭСЗО (1,2 кгс/см2)
float &pnto_t = *(float *)&buf[74];
// Крутизна характеристики ЭСЗО
float &pnto_sl = *(float *)&buf[75];
// Пропорциональный коэффициент регулятора ЭСЗО
float &reg_eszo_k = *(float *)&buf[76];
// Интегральный коэффициент регулятора ЭСЗО
float &reg_eszo_ki = *(float *)&buf[77];
// Крутизна передачи частоты на РМ
//float &n_sl_rm = *(float *)&buf[78];
// Начало зоны нечувствительности по частоте РМ
//float &n_ns_rm_b = *(float *)&buf[79];
// Конец зоны нечувствительности по частоте РМ
//float &n_ns_rm_e = *(float *)&buf[80];
// Крутизна изменения задания РМ по электрической мощности
float &ne_ref_sl = *(float *)&buf[81];
// Задание по электрической мощности РМ
float &ne_ref = *(float *)&buf[82];
// Давлении перед турбиной меньше которого происходит
// блокировка управления <больше> РМ
float &pt_rm_min = *(float *)&buf[83];
// Давлении перед турбиной больше которого происходит
// блокировка управления <меньше> РМ
float &pt_rm_max = *(float *)&buf[84];
// Давлении в регулирующей ступени ЧВД меньше которого
// происходит блокировка управления <меньше> РМ
float &prs_rm_min = *(float *)&buf[85];
// Давлении в регулирующей ступени ЧВД больше которого
// происходит блокировка управления <больше> РМ
float &prs_rm_max = *(float *)&buf[86];
// Частота вращения меньше которой происходит блокировка
// управления <меньше> РМ
float &n_rm_min = *(float *)&buf[87];
// Частота вращения больше которой происходит блокировка
// управления <больше> РМ
float &n_rm_max = *(float *)&buf[88];
// Вакууме в конденсаторе меньше которого происходит блокировка
// управления <больше> РМ
float &vk_rm_min = *(float *)&buf[89];
// Положении СВД больше которого происходит блокировка управления
// <больше> РМ
float &hsvd_rm_max = *(float *)&buf[90];
// Пропорциональный коэффициент РМ
float &reg_rm_k = *(float *)&buf[91];
// Интегральный коэффициент РМ
float &reg_rm_ki = *(float *)&buf[92];
// Задание по температуре РТО1 [С]
float &t_ref1 = *(float *)&buf[93];
// Крутизна изменения задания РТО1 по температуре (0,5C/мин)
float &t_ref1_sl = *(float *)&buf[94];
// Давлении перед турбиной меньше которого происходит блокировка
// управления <больше> РТО1 (125 кгс/см2)
float &pt_rto1_min = *(float *)&buf[95];
// Давлении перед турбиной больше которого происходит блокировка
// управления <меньше> РТО1 (132 кгс/см2)
float &pt_rto1_max = *(float *)&buf[96];
// Давлении в регулирующей ступени ЧВД меньше которого происходит
// блокировка управления <меньше> РТО1 (40 кгс/см2)
float &prs_rto1_min = *(float *)&buf[97];
// Давлении в регулирующей ступени ЧВД больше которого происходит
// блокировка управления <больше> РТО1 (92 кгс/см2)
float &prs_rto1_max = *(float *)&buf[98];
// Частота вращения меньше которой происходит блокировка управления
// <меньше> РТО1 (49 Гц)
float &n_rto1_min = *(float *)&buf[99];
// Частота вращения больше которой происходит блокировка управления
// <больше> РТО1 (50,5 Гц)
float &n_rto1_max = *(float *)&buf[100];
// Нагрев воды в ПСГ1 больше которого происходит блокировка управления
// <больше> РТО1 (50?С)
float &dt1_rto1_max = *(float *)&buf[101];
// Нагрев воды в ПСГ2 больше которого происходит блокировка управления
// <больше> РТО1 (50?С)
float &dt2_rto1_max = *(float *)&buf[102];
// Давлении в НТО больше которого происходит блокировка управления
// <больше> РТО1 (1,1 кгс/см2)
float &pnto_rto1_max = *(float *)&buf[103];
// Пропорциональный коэффициент РТО1
float &reg_rto1_k = *(float *)&buf[104];
// Интегральный коэффициент РТО1
float &reg_rto1_ki = *(float *)&buf[105];
// Выбор температуры которой управляет РТО1. 1 - t1; 2 - t2.
long &t_rto1_sel = buf[106];
// Задание по температуре РТО2 [?С]
float &t_ref2 = *(float *)&buf[107];
// Крутизна изменения задания РТО2 по температуре (0,5?C/мин)
float &t_ref2_sl = *(float *)&buf[108];
// Выбор температуры которой управляет РТО2. 1 - t1; 2 - t2.
long &t_rto2_sel = buf[109];
// Нагрев воды в ПСГ1 больше которого происходит блокировка управления
// <больше> РТО2 (50?С)
float &dt1_rto2_max = *(float *)&buf[110];
// Нагрев воды в ПСГ2 больше которого происходит блокировка управления
// <больше> РТО2 (50?С)
float &dt2_rto2_max = *(float *)&buf[111];
// Давлении в НТО больше которого происходит блокировка управления
// <больше> РТО2 (1,1 кгс/см2)
float &pnto_rto2_max = *(float *)&buf[112];
// Вакууме в конденсаторе меньше которого происходит блокировка управления
// <меньше> РТО2 (0,89 кгс/см2)
float &vk_rto2_min = *(float *)&buf[113];
// Положение ЭСНД меньше которого происходит блокировка управления
// <больше> РТО2 (5 мм)
float &hesnd_rto2_min = *(float *)&buf[114];
// Положение ЭСНД больше которого происходит блокировка управления
// <меньше> РТО2 (110 мм)
float &hesnd_rto2_max = *(float *)&buf[115];
// Пропорциональный коэффициент РТО2
float &reg_rto2_k = *(float *)&buf[116];
// Интегральный коэффициент РТО2
float &reg_rto2_ki = *(float *)&buf[117];
// Задание по температуре РТПВ [?С]
float &tpv_ref = *(float *)&buf[118];
// Крутизна изменения задание по температуре РТПВ [?С/мин]
float &tpv_ref_sl = *(float *)&buf[119];
// Вакууме в конденсаторе меньше которого происходит блокировка
// управления <больше> РТО2 (0,88 кгс/см2)
float &vk_rtpv_min = *(float *)&buf[120];
// Положение ЭСНД меньше которого происходит блокировка управления
// <меньше> РТО2 (5 мм)
float &hesnd_rtpv_min = *(float *)&buf[121];
// Положение ЭСНД больше которого происходит блокировка управления
// <больше> РТО2 (60 мм)
float &hesnd_rtpv_max = *(float *)&buf[122];
// Интервал времени, на который закрывается СВД, после отключения
// генератора от сети. (0 - 10 с)
float &t_cl_svd = *(float *)&buf[123];
// Статус кнопок на пульте и ШУ
long &but_st = buf[124];
// Номер сокета к которому подключен текущий терминал
//long &sock_num = buf[125];
// Положение АСК и СНД
long &ask_snd_st = buf[126];
// Изменения на выходе регулятора РМ
long &rm_izm = buf[127];
// Изменения на выходе регулятора РТО1
long &rto1_izm = buf[128];
// Изменения на выходе регулятора РТО2
long &rto2_izm = buf[129];
// Изменения на выходе регулятора РТПВ
long &rtpv_izm = buf[130];
// Блокировка изменения больше/меньше РМ
long &rm_bl = buf[131];
// Блокировка изменения больше/меньше РТО1
long &rto1_bl = buf[132];
// Блокировка изменения больше/меньше РТО2
long &rto2_bl = buf[133];
// Блокировка изменения больше/меньше РТПВ
long &rtpv_bl = buf[134];
// Текущий фактический угол сервомотора
float &hzu_f = *(float *)&buf[135];
// Вакуум в конденсаторе
float &vk = *(float *)&buf[136];
// Сигналы тревоги серводрайвера
long &alarm_srv = buf[137];
// Неисправность блоков питания
long &bp_bad = buf[138];
// Задание по электрической мощности РМ [МВт] после ограничителя темпа
float &ne_ref_a = *(float *)&buf[139];
// Задание по температуре РТО1 [?С] после ограничителя темпа
float &t_ref1_a = *(float *)&buf[140];
// Задание по температуре РТО2 [?С] после ограничителя темпа
float &t_ref2_a = *(float *)&buf[141];
// Задание по температуре РТПВ [?С] после ограничителя темпа
float &tpv_ref_a = *(float *)&buf[142];
// Коэффициент для коррекции АЦП. Давление импульсного масла (Римп.масла)
float &pim_k = *(float *)&buf[143];
// Смещение для коррекции АЦП. Давление импульсного масла (Римп.масла)
float &pim_b = *(float *)&buf[144];
// Коэффициент для коррекции АЦП. Давление масла на
// регулирование (Ррег.масла)
float &prm_k = *(float *)&buf[145];
// Смещение для коррекции АЦП. Давление масла на регулирование (Ррег.масла)
float &prm_b = *(float *)&buf[146];
// Коэффициент для коррекции АЦП. Давление в регулирующей ступени (Ррс)
float &prs_k = *(float *)&buf[147];
// Смещение для коррекции АЦП. Давление в регулирующей ступени (Ррс)
float &prs_b = *(float *)&buf[148];
// Коэффициент для коррекции АЦП. Давление острого пара (Р'Т)
float &pt_k = *(float *)&buf[149];
// Смещение для коррекции АЦП. Давление острого пара (Р'Т)
float &pt_b = *(float *)&buf[150];
// Коэффициент для коррекции АЦП. Давление в нижнем отборе (Рнто)
float &pnto1_k = *(float *)&buf[151];
// Смещение для коррекции АЦП. Давление в нижнем отборе (Рнто)
float &pnto1_b = *(float *)&buf[152];
// Коэффициент для коррекции АЦП. Давление в нижнем отборе (Рнто)
float &pnto2_k = *(float *)&buf[153];
// Смещение для коррекции АЦП. Давление в нижнем отборе (Рнто)
float &pnto2_b = *(float *)&buf[154];
// Коэффициент для коррекции АЦП. Температура подпиточной воды (tпв)
float &tpv_k = *(float *)&buf[155];
// Смещение для коррекции АЦП. Температура подпиточной воды (tпв)
float &tpv_b = *(float *)&buf[156];
// Коэффициент для коррекции АЦП. Температура сетевой воды 1 (t1)
float &t1_k = *(float *)&buf[157];
// Смещение для коррекции АЦП. Температура сетевой воды 1 (t1)
float &t1_b = *(float *)&buf[158];
// Коэффициент для коррекции АЦП. Температура сетевой воды 2 (t2)
float &t2_k = *(float *)&buf[159];
// Смещение для коррекции АЦП. Температура сетевой воды 2 (t2)
float &t2_b = *(float *)&buf[160];
// Коэффициент для коррекции АЦП. Температура обратной сетевой воды (tобр)
float &tobr_k = *(float *)&buf[161];
// Смещение для коррекции АЦП. Температура обратной сетевой воды (tобр)
float &tobr_b = *(float *)&buf[162];
// Коэффициент для коррекции АЦП. Давление сетевой воды (Рсв)
float &psv_k = *(float *)&buf[163];
// Смещение для коррекции АЦП. Давление сетевой воды (Рсв)
float &psv_b = *(float *)&buf[164];
// Коэффициент для коррекции АЦП. Положение СНД (hэснд)
float &hesnd_k = *(float *)&buf[165];
// Смещение для коррекции АЦП. Положение СНД (hэснд)
float &hesnd_b = *(float *)&buf[166];
// Коэффициент для коррекции АЦП. Положение сервомотора высокого
// давления - hСВД
float &hsvd_k = *(float *)&buf[167];
// Смещение для коррекции АЦП. Положение сервомотора высокого
// давления - hСВД
float &hsvd_b = *(float *)&buf[168];
// Коэффициент для коррекции АЦП. Активная мощность, отдаваемая
// генератором в сеть
float &ne_k = *(float *)&buf[169];
// Смещение для коррекции АЦП. Активная мощность, отдаваемая
// генератором в сеть
float &ne_b = *(float *)&buf[170];
// Коэффициент для коррекции АЦП. Вакуум в конденсаторе
float &vk_k = *(float *)&buf[171];
// Смещение для коррекции АЦП. Вакуум в конденсаторе
float &vk_b = *(float *)&buf[172];
// Пропорциональный коэффициент РТПВ [мм/?С]
float &reg_rtpv_k = *(float *)&buf[173];
// Интегральный коэффициент РТПВ [с-1]
float &reg_rtpv_ki = *(float *)&buf[174];
// Минимально допустимое положение СНД (hэснд) (0 - 250 мм)
float &hesnd_min = *(float *)&buf[175];
// Максимально допустимое положение СНД (hэснд) (0 - 250 мм)
float &hesnd_max = *(float *)&buf[176];
// Производная частоты соответствующая наиболее достоверной частоте
//float &d_n = *(float *)&buf[177];
// Смещение hСВД на входе РЧВТ [об./мин.]
//float &b_hsvd = *(float *)&buf[178];
// Коэффициент для вычисления hzu через hsvd [град./мм]
float &hzu_hsvd_k = *(float *)&buf[179];
// Смещение для вычисления hsvd через hzu [град.]
float &hzu_hsvd_b = *(float *)&buf[180];
// Самоблокировка
long &samo_bl = buf[181];
// Коэффициент преобразования производной Pнто на входе РТО1
//float &k_dpnto_rto1 = *(float *)&buf[182];
// Коэффициент преобразования производной Pнто на входе РТО2
//float &k_dpnto_rto2 = *(float *)&buf[183];
// Начальная частота для проверки каналов защиты по частоте [об./мин.]
float &test_n = *(float *)&buf[184];
// Производная частоты для проверки каналов защиты по частоте [об./(мин.с)]
float &test_dn = *(float *)&buf[185];
// Значение частоты, при которой сработала защита по частоте платы
// ИСВТ1 [об./мин.]
float &n12o = *(float *)&buf[186];
// Значение частоты, при которой сработала защита по частоте платы
// ИСВТ2 [об./мин.]
float &n34o = *(float *)&buf[187];
// Значение частоты, при которой сработала защита по частоте платы
// ИСВТ3 [об./мин.]
float &n56o = *(float *)&buf[188];
// Порог по частоте 1 считанный с ИСВТ (только чтение) [об./мин.]
float &n12tg = *(float *)&buf[189];
// Порог по частоте 2 считанный с ИСВТ (только чтение) [об./мин.]
float &n34tg = *(float *)&buf[190];
// Порог по частоте 3 считанный с ИСВТ (только чтение) [об./мин.]
float &n56tg = *(float *)&buf[191];
// Число зубьев на шестерне, которая используется для измерения частоты
long &n_zub = buf[192];
// Исправность реле ИСВТ
long &isvt_rele = buf[193];
// Ошибка позиционирования серводрайвера
long &srv_bad_pos = buf[194];
// Положение СВД холостого хода, мм
float &scnt_hh = *(float *)&buf[195];
// Состояние контуров управления
long &reg_st = buf[199];
// Пропорциональный коэффициент РЧВТ в обратном направлении [мм/(об./мин.)] ([имп./(об./мин.)])
float &reg_rchvt2_k = *(float *)&buf[203];
// Интегральный коэффициент РЧВТ в обратном направлении [с-1]
float &reg_rchvt2_ki = *(float *)&buf[204];
// Дифференциальный коэффициент РЧВТ в обратном направлении [с]
float &reg_rchvt2_kd = *(float *)&buf[205];
// Коэффициент изодромного звена
float &isodr_k = *(float *)&buf[206];
// Постоянная времени изодромного звена [с]
float &isodr_t = *(float *)&buf[207];
// Наклон нелинейной характеристики по частоте в зоне изменения
// коэффициета передачи
float &nelin_k = *(float *)&buf[208];
// Начало зоны изменения коэффициента передачи [об./мин.]
float &nelin_nb = *(float *)&buf[209];
// Конец зоны изменения коэффициента передачи [об./мин.]
float &nelin_ne = *(float *)&buf[210];
// Минимум диапазона измерения датчика температуры обратной сетевой воды (tobr)
float &tobr_smin = *(float *)&buf[211];
// Максимум диапазона измерения датчика температуры обратной сетевой воды (tobr)
float &tobr_smax = *(float *)&buf[212];
// Минимум диапазона измерения датчика вакуума в конденсаторе (vk)
float &vk_smin = *(float *)&buf[213];
// Максимум диапазона измерения датчика вакуума в конденсаторе (vk)
float &vk_smax = *(float *)&buf[214];
// Минимум диапазона измерения датчика давления сетевой воды (psv)
float &psv_smin = *(float *)&buf[215];
// Максимум диапазона измерения датчика давления сетевой воды (psv)
float &psv_smax = *(float *)&buf[216];
// Минимум диапазона измерения датчика положения электрического
// сервомотора низкого давления (hesnd)
float &hesnd_smin = *(float *)&buf[217];
// Максимум диапазона измерения датчика положения электрического
// сервомотора низкого давления (hesnd)
float &hesnd_smax = *(float *)&buf[218];
// Минимум диапазона измерения датчика положения сервомотора высокого
// давления (hsvd)
float &hsvd_smin = *(float *)&buf[219];
// Максимум диапазона измерения датчика положения сервомотора высокого
// давления (hsvd)
float &hsvd_smax = *(float *)&buf[220];

// Массив коэффициентов для коррекции АЦП
float *ai_k[15] = {&pim_k, &prm_k, &ne_k, &prs_k, &pt_k,
  &pnto1_k, &pnto2_k, &tpv_k, &t1_k, &t2_k, &tobr_k,
  &vk_k, &psv_k, &hesnd_k, &hsvd_k};
// Массив смещений для коррекции АЦП
float *ai_b[15] = {&pim_b, &prm_b, &ne_b, &prs_b, &pt_b,
  &pnto1_b, &pnto2_b, &tpv_b, &t1_b, &t2_b, &tobr_b,
  &vk_b, &psv_b, &hesnd_b, &hsvd_b};
// Массив конечных значений АЦП
float *ai_val[15] = {&pim, &prm, &ne, &prs, &pt,
  &pnto1, &pnto2, &tpv, &t1, &t2, &tobr,
  &vk, &psv, &hesnd, &hsvd};
// Массив порогов по частоте для ИСВТ
float *n12tm[3] = {&n12t, &n34t, &n56t};
// Массив порогов по производной частоты для ИСВТ
//float *d_n12tm[3] = {&d_n12t, &d_n34t, &d_n56t};
// Массив значений при которых сработала защита по частоте ИСВТ
float *n12om[3] = {&n12o, &n34o, &n56o};
// Массив значений при которых сработала защита по производной частоты ИСВТ
//float *d_n12om[3] = {&d_n12o, &d_n34o, &d_n56o};
// Массив текущих частот
float *n12m[3] = {&n12, &n34, &n56};
// Массив текущих производных частот
//float *d_n12m[3] = {&d_n12, &d_n34, &d_n56};
// Массив порогов по частоте считанный с ИСВТ
float *n12tgm[3] = {&n12tg, &n34tg, &n56tg};
// Массивы пределов измерения аналоговых датчиков
float *ai_smin[15] = {&pim_smin, &prm_smin, &ne_smin, &prs_smin, &pt_smin,
  &pnto1_smin, &pnto2_smin, &tpv_smin, &t1_smin, &t2_smin, &tobr_smin,
  &vk_smin, &psv_smin, &hesnd_smin, &hsvd_smin};
float *ai_smax[15] = {&pim_smax, &prm_smax, &ne_smax, &prs_smax, &pt_smax,
  &pnto1_smax, &pnto2_smax, &tpv_smax, &t1_smax, &t2_smax, &tobr_smax,
  &vk_smax, &psv_smax, &hesnd_smax, &hsvd_smax};

// ********************
// Служебные перменные
// Включение управления НД без датчика hesnd
long &nd_nosens = buf[CNT_VAR - 53];
// Частота по первому датчику, об./мин.
float &n1f = *(float *)&buf[CNT_VAR - 52];
// Частота по второму датчику, об./мин.
float &n2f = *(float *)&buf[CNT_VAR - 51];
// Частота по третьему датчику, об./мин.
float &n3f = *(float *)&buf[CNT_VAR - 50];
// Частота по четвертому датчику, об./мин.
float &n4f = *(float *)&buf[CNT_VAR - 49];
// Частота по пятому датчику, об./мин.
float &n5f = *(float *)&buf[CNT_VAR - 48];
// Частота по шестому датчику, об./мин.
float &n6f = *(float *)&buf[CNT_VAR - 47];
// Датчик по которому была зафиксирована аномальная частота
long &ind_nx_anomal = buf[CNT_VAR - 46];
// Время появления аномальной частоты датчика, 0xddhhmmss
long &time_nx_anomal = buf[CNT_VAR - 45];
// Предыдущая аномальная частота датчика, об./мин
float &nx_anomal_prev = *(float *)&buf[CNT_VAR - 44];
// Текущая аномальная частота датчика, об./мин
float &nx_anomal_cur = *(float *)&buf[CNT_VAR - 43];
// Время появления аномальной частоты, 0xddhhmmss
long &time_n_anomal = buf[CNT_VAR - 42];
// Предыдущая аномальная частота, об./мин
float &n_anomal_prev = *(float *)&buf[CNT_VAR - 41];
// Текущая аномальная частота, об./мин
float &n_anomal_cur = *(float *)&buf[CNT_VAR - 40];
// Период импульса НД, с
float &nd_period = *(float *)&buf[CNT_VAR - 39];
// Длина импульса НД, с
float &nd_pulse = *(float *)&buf[CNT_VAR - 38];
// Счетчик для определения состояния сети
long &suc_lan_counter = buf[CNT_VAR - 37];
// Постоянная времени фильтра АЦП в с
float &t_ai_filtr = *(float *)&buf[CNT_VAR - 36];
// Отличие hesnd от hesnd_ref меньше которого, регулирование не происходит, мм
float &hesnd_ref_delta = *(float *)&buf[CNT_VAR - 35];
// Текущее положение сервомотора ВД в отладке, мм
float &dbg_hsvd = *(float *)&buf[CNT_VAR - 34];
// Текущая мощность в отладке, МВт
float &dbg_ne = *(float *)&buf[CNT_VAR - 33];
// Текущая частота в отладке, об./мин.
float &dbg_n = *(float *)&buf[CNT_VAR - 32];
// Текущее положение сервомотора НД, мм
float &dbg_hesnd = *(float *)&buf[CNT_VAR - 31];
// Текущее положение золотника в отладке, имп.
long &dbg_scnt = buf[CNT_VAR - 30];
// Текущий статус дискретных входов в отладке (0: СТОП, 1: АСК,
// 2: ЗЗ1, 3: ЗЗ2, 4: ЗЗ3, 5-6: Ключ ВД, 7: КГ)
long &dbg_state = buf[CNT_VAR - 29];
// Задатчик положения серводрайвера
long &servo_cnt_ref = buf[CNT_VAR - 28];
// Счетчик перезапусков CPU по остановке сети
long &cnt_res_cpu = buf[CNT_VAR - 27];
// Счетчик перезапусков пакетного драйвера
long &cnt_res_pd = buf[CNT_VAR - 21];
// Счетчик перезапусков свитча
long &cnt_res_sw = buf[CNT_VAR - 20];
// Текущее время на контроллере, 0xddhhmmss
long &cur_time = buf[CNT_VAR - 19];
// Число элементов тестового массива
long &n_arr_tst = buf[CNT_VAR - 18];
// Индекс тестового массива
long &i_arr_tst = buf[CNT_VAR - 17];
// Запуск теста
long &start_test = buf[CNT_VAR - 16];
// Шаг записи тестового массива, с
float &dt_test = *(float *)&buf[CNT_VAR - 15];
// Счетчик тревог серводрайвера
long &cnt_alm_srv = buf[CNT_VAR - 14];
// Задание по hesnd
float &hesnd_ref = *(float *)&buf[CNT_VAR - 12];
// Задание по hsvd
float &hsvd_ref = *(float *)&buf[CNT_VAR - 11];
// Пуск турбины
long &start = buf[CNT_VAR - 10];
// Счетчик прерываний
long &count_int = buf[CNT_VAR - 9];
// Флаг аварийной перезагрузки
long &restart_flag = buf[CNT_VAR - 8];
// Счетчик перезагрузок
long &count_restart = buf[CNT_VAR - 7];
// Текущая длительность прерывания [мс]
float &cur_int_time = *(float *)&buf[CNT_VAR - 6];
// Максимальная длительность прерывания [мс]
float &int_time = *(float *)&buf[CNT_VAR - 5];
// Тестовая перменная
long &test_var = buf[CNT_VAR - 4];
// Счетчик циклов
long &count_loop = buf[CNT_VAR - 3];
// Контрольная сумма
long &check_sum = buf[CNT_VAR - 2];
// Шаг по времени выдачи пакетов по дополнительному порту, мс
float &dt_lan2 = *(float *)&buf[CNT_VAR - 1];

// Массив по датчикам
float *nfm[6] = {&n1f, &n2f, &n3f, &n4f, &n5f, &n6f};

// ******************************************************************
// Функции
// ******************************************************************
// Дополнительная фильтрация переменных при инициализации
void filtrbufex()
{
  long lr;
  //float fr;
  //fr = bound(*(float *)&wbuf[135], -100.f, 100.f);
  //sync(hzu_f = fr);
  //servo_cnt_cur = servo_cnt = scnt_to_grad(hzu_f);
  hzu_f = 0.f;
  //hesnd_ref = hesnd_min;
  if (vbuf[181]) {
    //lr = bound(wbuf[181], 0L, 15L);
    lr = wbuf[181];
    sync(samo_bl = lr);
  }
}
// Проверяет массив записи на правильность и корректирует, в случае
// необходимости. Копирует правильные значения в буфер чтения
void filtrbuf()
{
  long lr;
  float fr;

  if (vbuf[1]) {
    fr = bound(*(float *)&wbuf[1], -1e6f, 1e6f);
    sync(pim_smin = fr);
  }
  if (vbuf[2]) {
    fr = bound(*(float *)&wbuf[2], -1e6f, 1e6f);
    sync(pim_smax = fr);
  }
  if (vbuf[3]) {
    fr = bound(*(float *)&wbuf[3], -1e6f, 1e6f);
    sync(prm_smin = fr);
  }
  if (vbuf[4]) {
    fr = bound(*(float *)&wbuf[4], -1e6f, 1e6f);
    sync(prm_smax = fr);
  }
  if (vbuf[5]) {
    fr = bound(*(float *)&wbuf[5], -1e6f, 1e6f);
    sync(ne_smin = fr);
  }
  if (vbuf[6]) {
    fr = bound(*(float *)&wbuf[6], -1e6f, 1e6f);
    sync(ne_smax = fr);
  }
  if (vbuf[7]) {
    fr = bound(*(float *)&wbuf[7], -1e6f, 1e6f);
    sync(prs_smin = fr);
  }
  if (vbuf[8]) {
    fr = bound(*(float *)&wbuf[8], -1e6f, 1e6f);
    sync(prs_smax = fr);
  }
  if (vbuf[9]) {
    fr = bound(*(float *)&wbuf[9], -1e6f, 1e6f);
    sync(pt_smin = fr);
  }
  if (vbuf[10]) {
    fr = bound(*(float *)&wbuf[10], -1e6f, 1e6f);
    sync(pt_smax = fr);
  }
  if (vbuf[11]) {
    fr = bound(*(float *)&wbuf[11], -1e6f, 1e6f);
    sync(pnto1_smin = fr);
  }
  if (vbuf[12]) {
    fr = bound(*(float *)&wbuf[12], -1e6f, 1e6f);
    sync(pnto1_smax = fr);
  }
  if (vbuf[13]) {
    fr = bound(*(float *)&wbuf[13], -1e6f, 1e6f);
    sync(pnto2_smin = fr);
  }
  if (vbuf[14]) {
    fr = bound(*(float *)&wbuf[14], -1e6f, 1e6f);
    sync(pnto2_smax = fr);
  }
  if (vbuf[15]) {
    fr = bound(*(float *)&wbuf[15], -1e6f, 1e6f);
    sync(tpv_smin = fr);
  }
  if (vbuf[16]) {
    fr = bound(*(float *)&wbuf[16], -1e6f, 1e6f);
    sync(tpv_smax = fr);
  }
  if (vbuf[17]) {
    fr = bound(*(float *)&wbuf[17], -1e6f, 1e6f);
    sync(t1_smin = fr);
  }
  if (vbuf[18]) {
    fr = bound(*(float *)&wbuf[18], -1e6f, 1e6f);
    sync(t1_smax = fr);
  }
  if (vbuf[19]) {
    fr = bound(*(float *)&wbuf[19], -1e6f, 1e6f);
    sync(t2_smin = fr);
  }
  if (vbuf[20]) {
    fr = bound(*(float *)&wbuf[20], -1e6f, 1e6f);
    sync(t2_smax = fr);
  }
  if (vbuf[26]) {
    lr = bound(wbuf[26], 0L, 1L);
    sync(mode_hzu = lr);
  }
  if (vbuf[27]) {
    lr = bound(wbuf[27], 0L, 1L);
    sync(start_turbine = lr);
  }
  if (vbuf[28]) {
    lr = bound(wbuf[28], 0L, 7L);
    sync(mode_testn = lr);
  }
  if (vbuf[30]) {
    lr = bound(wbuf[30], 0L, 7L);
    #ifdef N_BAD_BLOCK
    sync(n_bad = 0);
    #else //N_BAD_BLOCK
    sync(n_bad = lr);
    #endif //N_BAD_BLOCK
  }
  if (vbuf[32]) {
    lr = bound(wbuf[32], 0L, 2L);
    sync(pnto_sel = lr);
  }
  if (vbuf[46]) {
    fr = bound(*(float *)&wbuf[46], 0.f, 250.f);
    sync(hsvd_min = fr);
  }
  if (vbuf[47]) {
    fr = bound(*(float *)&wbuf[47], 0.f, 250.f);
    sync(hsvd_max = fr);
  }
  if (vbuf[49]) {
    fr = bound(*(float *)&wbuf[49], -1e6f, 1e6f);
    sync(hzu_min = fr);
    servo_cnt_min = grad_to_scnt(hzu_min);
  }
  if (vbuf[50]) {
    fr = bound(*(float *)&wbuf[50], -1e6f, 1e6f);
    sync(hzu_max = fr);
    servo_cnt_max = grad_to_scnt(hzu_max);
  }
  if (vbuf[57]) {
    fr = bound(*(float *)&wbuf[57], 0.f, 3500.f);
    sync(n12t = fr);
  }
  if (vbuf[58]) {
    fr = bound(*(float *)&wbuf[58], 0.f, 3500.f);
    sync(n34t = fr);
  }
  if (vbuf[59]) {
    fr = bound(*(float *)&wbuf[59], 0.f, 3500.f);
    sync(n56t = fr);
  }
  if (vbuf[60]) {
    fr = bound(*(float *)&wbuf[60], 0.f, 1e6f);
    sync(rchvt_ins = fr);
  }
  if (vbuf[61]) {
    fr = bound(*(float *)&wbuf[61], 0.f, 1e6f);
    sync(rm_ins = fr);
  }
  if (vbuf[62]) {
    fr = bound(*(float *)&wbuf[62], 0.f, 1e6f);
    sync(rto1_ins = fr);
  }
  if (vbuf[63]) {
    fr = bound(*(float *)&wbuf[63], 0.f, 1e6f);
    sync(rto2_ins = fr);
  }
  if (vbuf[64]) {
    fr = bound(*(float *)&wbuf[64], 0.f, 1e6f);
    sync(rtpv_ins = fr);
  }
  if (vbuf[65]) {
    fr = bound(*(float *)&wbuf[65], 0.f, 3000.f);
    sync(n_defense = fr);
  }
  if (vbuf[66]) {
    fr = bound(*(float *)&wbuf[66], n_ref_min, n_ref_max);
    sync(n_ref = fr);
  }
  if (vbuf[67]) {
    fr = bound(*(float *)&wbuf[67], -1e6f, 1e6f);
    sync(reg_rchvt_k = fr);
  }
  if (vbuf[68]) {
    fr = bound(*(float *)&wbuf[68], -1e6f, 1e6f);
    sync(reg_rchvt_ki = fr);
  }
  if (vbuf[69]) {
    fr = bound(*(float *)&wbuf[69], -1e6f, 1e6f);
    sync(reg_rchvt_kd = fr);
  }
  if (vbuf[70]) {
    fr = bound(*(float *)&wbuf[70], 0.f, 58.3f);
    sync(n_ns_rchvt_b = fr);
  }
  if (vbuf[71]) {
    fr = bound(*(float *)&wbuf[71], 0.f, 58.3f);
    sync(n_ns_rchvt_e = fr);
  }
  if (vbuf[73]) {
    fr = bound(*(float *)&wbuf[73], 0.f, 1e6f);
    sync(a_hsvd = fr);
  }
  if (vbuf[74]) {
    fr = bound(*(float *)&wbuf[74], -1.f, 1.5f);
    sync(pnto_t = fr);
  }
  if (vbuf[75]) {
    fr = bound(*(float *)&wbuf[75], 0.f, 100.f);
    sync(pnto_sl = fr);
  }
  if (vbuf[76]) {
    fr = bound(*(float *)&wbuf[76], -1e6f, 1e6f);
    sync(reg_eszo_k = fr);
  }
  if (vbuf[77]) {
    fr = bound(*(float *)&wbuf[77], -1e6f, 1e6f);
    sync(reg_eszo_ki = fr);
  }
  if (vbuf[81]) {
    fr = bound(*(float *)&wbuf[81], 0.f, 1e6f);
    sync(ne_ref_sl = fr);
  }
  if (vbuf[82]) {
    fr = bound(*(float *)&wbuf[82], 0.f, 120.f);
    sync(ne_ref = fr);
  }
  if (vbuf[83]) {
    fr = bound(*(float *)&wbuf[83], -2.f, 140.f);
    sync(pt_rm_min = fr);
  }
  if (vbuf[84]) {
    fr = bound(*(float *)&wbuf[84], -2.f, 140.f);
    sync(pt_rm_max = fr);
  }
  if (vbuf[85]) {
    fr = bound(*(float *)&wbuf[85], -2.f, 92.f);
    sync(prs_rm_min = fr);
  }
  if (vbuf[86]) {
    fr = bound(*(float *)&wbuf[86], -2.f, 92.f);
    sync(prs_rm_max = fr);
  }
  if (vbuf[87]) {
    fr = bound(*(float *)&wbuf[87], 0.f, 58.3f);
    sync(n_rm_min = fr);
  }
  if (vbuf[88]) {
    fr = bound(*(float *)&wbuf[88], 0.f, 58.3f);
    sync(n_rm_max = fr);
  }
  if (vbuf[89]) {
    fr = bound(*(float *)&wbuf[89], -1.f, 0.f);
    sync(vk_rm_min = fr);
  }
  if (vbuf[90]) {
    fr = bound(*(float *)&wbuf[90], 0.f, 250.f);
    sync(hsvd_rm_max = fr);
  }
  if (vbuf[91]) {
    fr = bound(*(float *)&wbuf[91], -1e6f, 1e6f);
    sync(reg_rm_k = fr);
  }
  if (vbuf[92]) {
    fr = bound(*(float *)&wbuf[92], -1e6f, 1e6f);
    sync(reg_rm_ki = fr);
  }
  if (vbuf[93]) {
    fr = bound(*(float *)&wbuf[93], 0.f, 150.f);
    sync(t_ref1 = fr);
  }
  if (vbuf[94]) {
    fr = bound(*(float *)&wbuf[94], 0.f, 1e6f);
    sync(t_ref1_sl = fr);
  }
  if (vbuf[95]) {
    fr = bound(*(float *)&wbuf[95], 0.f, 140.f);
    sync(pt_rto1_min = fr);
  }
  if (vbuf[96]) {
    fr = bound(*(float *)&wbuf[96], 0.f, 140.f);
    sync(pt_rto1_max = fr);
  }
  if (vbuf[97]) {
    fr = bound(*(float *)&wbuf[97], 0.f, 92.f);
    sync(prs_rto1_min = fr);
  }
  if (vbuf[98]) {
    fr = bound(*(float *)&wbuf[98], 0.f, 92.f);
    sync(prs_rto1_max = fr);
  }
  if (vbuf[99]) {
    fr = bound(*(float *)&wbuf[99], 0.f, 58.3f);
    sync(n_rto1_min = fr);
  }
  if (vbuf[100]) {
    fr = bound(*(float *)&wbuf[100], 0.f, 58.3f);
    sync(n_rto1_max = fr);
  }
  if (vbuf[101]) {
    fr = bound(*(float *)&wbuf[101], 0.f, 150.f);
    sync(dt1_rto1_max = fr);
  }
  if (vbuf[102]) {
    fr = bound(*(float *)&wbuf[102], 0.f, 150.f);
    sync(dt2_rto1_max = fr);
  }
  if (vbuf[103]) {
    fr = bound(*(float *)&wbuf[103], 0.f, 1.5f);
    sync(pnto_rto1_max = fr);
  }
  if (vbuf[104]) {
    fr = bound(*(float *)&wbuf[104], -1e6f, 1e6f);
    sync(reg_rto1_k = fr);
  }
  if (vbuf[105]) {
    fr = bound(*(float *)&wbuf[105], -1e6f, 1e6f);
    sync(reg_rto1_ki = fr);
  }
  if (vbuf[106]) {
    lr = bound(wbuf[106], 1L, 2L);
    sync(t_rto1_sel = lr);
  }
  if (vbuf[107]) {
    fr = bound(*(float *)&wbuf[107], 0.f, 150.f);
    sync(t_ref2 = fr);
  }
  if (vbuf[108]) {
    fr = bound(*(float *)&wbuf[108], 0.f, 1e6f);
    sync(t_ref2_sl = fr);
  }
  if (vbuf[109]) {
    lr = bound(wbuf[109], 1L, 2L);
    sync(t_rto2_sel = lr);
  }
  if (vbuf[110]) {
    fr = bound(*(float *)&wbuf[110], 0.f, 150.f);
    sync(dt1_rto2_max = fr);
  }
  if (vbuf[111]) {
    fr = bound(*(float *)&wbuf[111], 0.f, 150.f);
    sync(dt2_rto2_max = fr);
  }
  if (vbuf[112]) {
    fr = bound(*(float *)&wbuf[112], 0.f, 1.5f);
    sync(pnto_rto2_max = fr);
  }
  if (vbuf[113]) {
    fr = bound(*(float *)&wbuf[113], -1.f, 0.f);
    sync(vk_rto2_min = fr);
  }
  if (vbuf[114]) {
    fr = bound(*(float *)&wbuf[114], 0.f, 250.f);
    sync(hesnd_rto2_min = fr);
  }
  if (vbuf[115]) {
    fr = bound(*(float *)&wbuf[115], 0.f, 250.f);
    sync(hesnd_rto2_max = fr);
  }
  if (vbuf[116]) {
    fr = bound(*(float *)&wbuf[116], -1e6f, 1e6f);
    sync(reg_rto2_k = fr);
  }
  if (vbuf[117]) {
    fr = bound(*(float *)&wbuf[117], -1e6f, 1e6f);
    sync(reg_rto2_ki = fr);
  }
  if (vbuf[118]) {
    fr = bound(*(float *)&wbuf[118], 0.f, 50.f);
    sync(tpv_ref = fr);
  }
  if (vbuf[119]) {
    fr = bound(*(float *)&wbuf[119], 0.f, 1e6f);
    sync(tpv_ref_sl = fr);
  }
  if (vbuf[120]) {
    fr = bound(*(float *)&wbuf[120], -1.f, 0.f);
    sync(vk_rtpv_min = fr);
  }
  if (vbuf[121]) {
    fr = bound(*(float *)&wbuf[121], 0.f, 250.f);
    sync(hesnd_rtpv_min = fr);
  }
  if (vbuf[122]) {
    fr = bound(*(float *)&wbuf[122], 0.f, 250.f);
    sync(hesnd_rtpv_max = fr);
  }
  if (vbuf[123]) {
    fr = bound(*(float *)&wbuf[123], 0.f, 10.f);
    sync(t_cl_svd = fr);
  }
  if (vbuf[143]) {
    fr = bound(*(float *)&wbuf[143], -1e6f, 1e6f);
    sync(pim_k = fr);
  }
  if (vbuf[144]) {
    fr = bound(*(float *)&wbuf[144], -20.f, 20.f);
    sync(pim_b = fr);
  }
  if (vbuf[145]) {
    fr = bound(*(float *)&wbuf[145], -1e6f, 1e6f);
    sync(prm_k = fr);
  }
  if (vbuf[146]) {
    fr = bound(*(float *)&wbuf[146], -20.f, 20.f);
    sync(prm_b = fr);
  }
  if (vbuf[147]) {
    fr = bound(*(float *)&wbuf[147], -1e6f, 1e6f);
    sync(prs_k = fr);
  }
  if (vbuf[148]) {
    fr = bound(*(float *)&wbuf[148], -100.f, 100.f);
    sync(prs_b = fr);
  }
  if (vbuf[149]) {
    fr = bound(*(float *)&wbuf[149], -1e6f, 1e6f);
    sync(pt_k = fr);
  }
  if (vbuf[150]) {
    fr = bound(*(float *)&wbuf[150], -150.f, 150.f);
    sync(pt_b = fr);
  }
  if (vbuf[151]) {
    fr = bound(*(float *)&wbuf[151], -1e6f, 1e6f);
    sync(pnto1_k = fr);
  }
  if (vbuf[152]) {
    fr = bound(*(float *)&wbuf[152], -3.f, 3.f);
    sync(pnto1_b = fr);
  }
  if (vbuf[153]) {
    fr = bound(*(float *)&wbuf[153], -1e6f, 1e6f);
    sync(pnto2_k = fr);
  }
  if (vbuf[154]) {
    fr = bound(*(float *)&wbuf[154], -3.f, 3.f);
    sync(pnto2_b = fr);
  }
  if (vbuf[155]) {
    fr = bound(*(float *)&wbuf[155], -1e6f, 1e6f);
    sync(tpv_k = fr);
  }
  if (vbuf[156]) {
    fr = bound(*(float *)&wbuf[156], -50.f, 50.f);
    sync(tpv_b = fr);
  }
  if (vbuf[157]) {
    fr = bound(*(float *)&wbuf[157], -1e6f, 1e6f);
    sync(t1_k = fr);
  }
  if (vbuf[158]) {
    fr = bound(*(float *)&wbuf[158], -150.f, 150.f);
    sync(t1_b = fr);
  }
  if (vbuf[159]) {
    fr = bound(*(float *)&wbuf[159], -1e6f, 1e6f);
    sync(t2_k = fr);
  }
  if (vbuf[160]) {
    fr = bound(*(float *)&wbuf[160], -150.f, 150.f);
    sync(t2_b = fr);
  }
  if (vbuf[161]) {
    fr = bound(*(float *)&wbuf[161], -1e6f, 1e6f);
    sync(tobr_k = fr);
  }
  if (vbuf[162]) {
    fr = bound(*(float *)&wbuf[162], -150.f, 150.f);
    sync(tobr_b = fr);
  }
  if (vbuf[163]) {
    fr = bound(*(float *)&wbuf[163], -1e6f, 1e6f);
    sync(psv_k = fr);
  }
  if (vbuf[164]) {
    fr = bound(*(float *)&wbuf[164], -10.f, 10.f);
    sync(psv_b = fr);
  }
  if (vbuf[165]) {
    fr = bound(*(float *)&wbuf[165], -1e6f, 1e6f);
    sync(hesnd_k = fr);
  }
  if (vbuf[166]) {
    fr = bound(*(float *)&wbuf[166], -300.f, 300.f);
    sync(hesnd_b = fr);
  }
  if (vbuf[167]) {
    fr = bound(*(float *)&wbuf[167], -1e6f, 1e6f);
    sync(hsvd_k = fr);
  }
  if (vbuf[168]) {
    fr = bound(*(float *)&wbuf[168], -300.f, 300.f);
    sync(hsvd_b = fr);
  }
  if (vbuf[169]) {
    fr = bound(*(float *)&wbuf[169], -1e6f, 1e6f);
    sync(ne_k = fr);
  }
  if (vbuf[170]) {
    fr = bound(*(float *)&wbuf[170], -140.f, 140.f);
    sync(ne_b = fr);
  }
  if (vbuf[171]) {
    fr = bound(*(float *)&wbuf[171], -1e6f, 1e6f);
    sync(vk_k = fr);
  }
  if (vbuf[172]) {
    fr = bound(*(float *)&wbuf[172], -2.f, 2.f);
    sync(vk_b = fr);
  }
  if (vbuf[173]) {
    fr = bound(*(float *)&wbuf[173], -1e6f, 1e6f);
    sync(reg_rtpv_k = fr);
  }
  if (vbuf[174]) {
    fr = bound(*(float *)&wbuf[174], -1e6f, 1e6f);
    sync(reg_rtpv_ki = fr);
  }
  if (vbuf[175]) {
    fr = bound(*(float *)&wbuf[175], 0.f, 250.f);
    sync(hesnd_min = fr);
  }
  if (vbuf[176]) {
    fr = bound(*(float *)&wbuf[176], 0.f, 250.f);
    sync(hesnd_max = fr);
  }
  if (vbuf[179]) {
    fr = bound(*(float *)&wbuf[179], 0.f, 1e6f);
    sync(hzu_hsvd_k = fr);
  }
  if (vbuf[180]) {
    fr = bound(*(float *)&wbuf[180], -hsvd_smax, hsvd_smax);
    sync(hzu_hsvd_b = fr);
  }
  if (vbuf[184]) {
    fr = bound(*(float *)&wbuf[184], 0.f, n_max);
    sync(test_n = fr);
  }
  if (vbuf[185]) {
    fr = bound(*(float *)&wbuf[185], 0.f, 1e6f);
    sync(test_dn = fr);
  }
  if (vbuf[186]) {
    fr = bound(*(float *)&wbuf[186], 0.f, n_max);
    sync(n12o = fr);
  }
  if (vbuf[187]) {
    fr = bound(*(float *)&wbuf[187], 0.f, n_max);
    sync(n34o = fr);
  }
  if (vbuf[188]) {
    fr = bound(*(float *)&wbuf[188], 0.f, n_max);
    sync(n56o = fr);
  }
  if (vbuf[192]) {
    lr = bound(wbuf[192], 1L, 255L);
    sync(n_zub = lr);
  }
  if (vbuf[195]) {
    fr = bound(*(float *)&wbuf[195], 0.f, 555.f);
    sync(scnt_hh = fr);
  }
  if (vbuf[203]) {
    fr = bound(*(float *)&wbuf[203], -1e6f, 1e6f);
    sync(reg_rchvt2_k = fr);
  }
  if (vbuf[204]) {
    fr = bound(*(float *)&wbuf[204], -1e6f, 1e6f);
    sync(reg_rchvt2_ki = fr);
  }
  if (vbuf[205]) {
    fr = bound(*(float *)&wbuf[205], -1e6f, 1e6f);
    sync(reg_rchvt2_kd = fr);
  }
  if (vbuf[206]) {
    fr = bound(*(float *)&wbuf[206], -1e6f, 1e6f);
    sync(isodr_k = fr);
  }
  if (vbuf[207]) {
    fr = bound(*(float *)&wbuf[207], -1e6f, 1e6f);
    sync(isodr_t = fr);
  }
  if (vbuf[208]) {
    fr = bound(*(float *)&wbuf[208], -1e6f, 1e6f);
    sync(nelin_k = fr);
  }
  if (vbuf[209]) {
    fr = bound(*(float *)&wbuf[209], -1e6f, 1e6f);
    sync(nelin_nb = fr);
  }
  if (vbuf[210]) {
    fr = bound(*(float *)&wbuf[210], -1e6f, 1e6f);
    sync(nelin_ne = fr);
  }
  if (vbuf[211]) {
    fr = bound(*(float *)&wbuf[211], -1e6f, 1e6f);
    sync(tobr_smin = fr);
  }
  if (vbuf[212]) {
    fr = bound(*(float *)&wbuf[212], -1e6f, 1e6f);
    sync(tobr_smax = fr);
  }
  if (vbuf[213]) {
    fr = bound(*(float *)&wbuf[213], -1e6f, 1e6f);
    sync(vk_smin = fr);
  }
  if (vbuf[214]) {
    fr = bound(*(float *)&wbuf[214], -1e6f, 1e6f);
    sync(vk_smax = fr);
  }
  if (vbuf[215]) {
    fr = bound(*(float *)&wbuf[215], -1e6f, 1e6f);
    sync(psv_smin = fr);
  }
  if (vbuf[216]) {
    fr = bound(*(float *)&wbuf[216], -1e6f, 1e6f);
    sync(psv_smax = fr);
  }
  if (vbuf[217]) {
    fr = bound(*(float *)&wbuf[217], -1e6f, 1e6f);
    sync(hesnd_smin = fr);
  }
  if (vbuf[218]) {
    fr = bound(*(float *)&wbuf[218], -1e6f, 1e6f);
    sync(hesnd_smax = fr);
  }
  if (vbuf[219]) {
    fr = bound(*(float *)&wbuf[219], -1e6f, 1e6f);
    sync(hsvd_smin = fr);
  }
  if (vbuf[220]) {
    fr = bound(*(float *)&wbuf[220], -1e6f, 1e6f);
    sync(hsvd_smax = fr);
  }

  // Служебные переменные
  if (vbuf[CNT_VAR - 53]) {
    sync(nd_nosens = bound(wbuf[CNT_VAR - 53], 0L, 1L));
    //sync(nd_nosens = wbuf[CNT_VAR - 53]);
  }
  if (vbuf[CNT_VAR - 39]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 39], 1.f, 600.f);
    sync(nd_period = fr);
  }
  if (vbuf[CNT_VAR - 38]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 38], .02f, 10.f);
    sync(nd_pulse = fr);
  }
  if (vbuf[CNT_VAR - 37]) {
    sync(suc_lan_counter = wbuf[CNT_VAR - 37]);
  }
  if (vbuf[CNT_VAR - 36]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 36], 0.01f, 100.f);
    sync(t_ai_filtr = fr);
    double n_ai_filtr1 = t_ai_filtr/(16*tm0tos(TIMER0_CNT));
    sync(n_ai_filtr = n_ai_filtr1);
    {for (int i = 0; i < 24; i++) {
      adcf[i].y1 = INT_MIN;
      adcf[i].t = n_ai_filtr;
    }}
  }
  if (vbuf[CNT_VAR - 35]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 35], 0.f, 200.f);
    sync(hesnd_ref_delta = fr);
  }
  if (vbuf[CNT_VAR - 34]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 34], 0.f, 250.f);
    sync(dbg_hsvd = fr);
  }
  if (vbuf[CNT_VAR - 33]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 33], 0.f, 200.f);
    sync(dbg_ne = fr);
  }
  if (vbuf[CNT_VAR - 32]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 32], 3.5f, 10000.f);
    sync(dbg_n = fr);
  }
  if (vbuf[CNT_VAR - 31]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 31], 0.f, 250.f);
    sync(dbg_hesnd = fr);
  }
  if (vbuf[CNT_VAR - 30]) {
    sync(dbg_scnt = bound(wbuf[CNT_VAR - 30], 0L, 512L));
  }
  if (vbuf[CNT_VAR - 29]) {
    sync(dbg_state = wbuf[CNT_VAR - 29]);
  }
  if (vbuf[CNT_VAR - 27]) {
    sync(cnt_res_cpu = wbuf[CNT_VAR - 27]);
  }
  if (vbuf[CNT_VAR - 21]) {
    sync(cnt_res_pd = wbuf[CNT_VAR - 21]);
  }
  if (vbuf[CNT_VAR - 20]) {
    sync(cnt_res_sw = wbuf[CNT_VAR - 20]);
  }
  if (vbuf[CNT_VAR - 16]) {
    sync(start_test = wbuf[CNT_VAR - 16]);
  }
  if (vbuf[CNT_VAR - 15]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 15], -1e6f, 1e6f);
    sync(dt_test = fr);
  }
  if (vbuf[CNT_VAR - 14]) {
    sync(cnt_alm_srv = wbuf[CNT_VAR - 14]);
  }
  if (vbuf[CNT_VAR - 8]) {
    sync(restart_flag = wbuf[CNT_VAR - 8]);
  }
  if (vbuf[CNT_VAR - 7]) {
    sync(count_restart = wbuf[CNT_VAR - 7]);
  }
  if (vbuf[CNT_VAR - 4]) {
    sync(test_var = wbuf[CNT_VAR - 4]);
  }
  if (vbuf[CNT_VAR - 1]) {
    fr = bound(*(float *)&wbuf[CNT_VAR - 1], 1.f, 5000.f);
    sync(dt_lan2 = fr);
  }
}
// Проверяет входит ли переменная x в диапазон min - max и если не входит,
// заменяет ее значение на ближайшую границу диапазона
#ifdef NOP
template <class T>
T bound(T x, T min, T max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}
#endif
int bound(int x, int min, int max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}
long bound(long x, long min, long max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}
float bound(float x, float min, float max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}
double bound(double x, double min, double max)
{
  if (x < min) x = min;
  if (x > max) x = max;
  return x;
}
// Чтение счетчика канала 0 таймера 8254
uint get_timer0()
{
  uint res;
  outp(0x43, 0);
  res = inp(0x40); 
  res |= (inp(0x40) << 8); 
  return res;
}
#define TINT_CORR // Выполнение дополнительной коррекции
// Ф-ция для определения временных интервалов
// step == 0 - начальный момент времени
// step == 1 - конечный
uint get_tint(int step)
{
  static uint t1, t2, dt;
  switch (step) {
    case 0:
      #ifdef TINT_CORR
      get_tint(3);
      get_tint(4);
      #endif //TINT_CORR
      t1 = get_timer0();
      break;
    case 1:
      t2 = get_timer0();
      #ifdef TINT_CORR
      dt = -dt;
      dt += t1 - t2;
      #else //TINT_CORR
      dt = t1 - t2;
      #endif //TINT_CORR
      return dt;
    case 3:
      t1 = get_timer0();
      break;
    case 4:
      t2 = get_timer0();
      dt = t1 - t2;
      return dt;
  }
  return 0;
}
// Перевод показаний таймера 0 из отсчетов в с
double tm0tos(uint cnt)
{
  double k = 1./1.19318/1e6;
  return k*cnt;
}
// Перевод показаний таймера 0 из отсчетов в мс
double tm0toms(uint cnt)
{
  double k = 1./1.19318/1e3;
  return k*cnt;
}
// Первод счетчика в мс
double cnttoms(long cnt)
{
  return cnt*TIMER0_CNT/TACT_T0/1e3;
}
// Перевод мс в счетчик
long mstocnt(double ms)
{
  double r1 = ms*TACT_T0*1e3/TIMER0_CNT;
  long r = bound(r1, 0., (double)LONG_MAX);
  return r;
}
// Устанавливает таймаут t в 0.1 с
// в переменную c
void set_timeout(clock_t &c, long t)
{
  c = clock() + CLK_TCK*t/10;
}
// Проверяет таймаут c
int test_timeout(clock_t c)
{
  return clock() - c > 0;
}
#ifdef USE_CPU_COUNTER
// Установка таймаута t в мс по переменной counter
void set_to_cnt_t(counter_t &to, double t)
{
  set_to_cnt(to, TIME_TO_CNT(t, 1000));
}
// Установка следующего таймаута t в мс по переменной counter
void next_to_cnt_t(counter_t &to, double t)
{
  next_to_cnt(to, TIME_TO_CNT(t, 1000));
}
// Проверка таймаута в мс по переменной counter
int test_to_cnt_t(counter_t to)
{
  return test_to_cnt(to);
}
#else //USE_CPU_COUNTER
// Установка таймаута t в мс по переменной counter
void set_to_cnt_t(long &to, double t)
{
  long _cnt = counter;
        long d = t*TACT_T0*1e3/TIMER0_CNT;
  if (d <= 0) d = 1;
  to = _cnt + d;
}
// Установка следующего таймаута t в мс по переменной counter
void next_to_cnt_t(long &to, double t)
{
        long d = t*TACT_T0*1e3/TIMER0_CNT;
  if (d <= 0) d = 1;
  to += d;
}
// Проверка таймаута в мс по переменной counter
int test_to_cnt_t(long to)
{
  long _cnt = counter;
  return _cnt - to > 0;
}
#endif //USE_CPU_COUNTER
#ifdef TEST
// Инициализация теста
void test_init()
{
  n_arr_tst = size_arr_tst/sizeof(tst_data);
  arr_tst = (tst_data *)farcalloc(n_arr_tst, sizeof(tst_data));
  if (!arr_tst) {
    dputs("Не хватает памяти для arr_tst");
  }
  start_test = 0L;
  dtms_test = dt_test*1000.;
}
// Деинициализация теста
void test_deinit()
{
  farfree(arr_tst);
}
// Запись тестового файла
void test_save()
{
  if (test_save_f) {
    // Конструирование имени файла
    char fn[50] = "";
    int L = strlen(fn);
    strcpy(fn + L, test_path);
    struct time st;
    gettime(&st);
    struct date sd;
    getdate (&sd);
    L = strlen(fn);
    sprintf(fn + L, "%02d", (int)sd.da_day);
    L = strlen(fn);
    sprintf(fn + L, "%02d", (int)st.ti_hour);
    L = strlen(fn);
    sprintf(fn + L, "%02d", (int)st.ti_min);
    L = strlen(fn);
    sprintf(fn + L, "%02d", (int)st.ti_sec);
    strcat(fn, ".csv");

    // Запись файла на диск
    FILE *f = fopen(fn, "wt");
    fprintf(f, "time;hzu;hzu_f;servo_cnt;servo_cnt_ref;servo_cnt_cur;"
      "hsvd;n;n_ref;dtn=%gms "
      "krchvt=%g kirchvt=%g kdhsvd=%g rchvt_type=%s\n",
      dt_test*1e3f, reg_rchvt_k,
      reg_rchvt_ki, reg_rchvt_kd, rchvt_t);
    for (int i = 0; i < i_arr_tst; i++) {
      char s[100];
      sprintf(s, "%g;%g;%g;%ld;%ld;%ld;%g;%g;%g\n", arr_tst[i].time,
        arr_tst[i].hzu, arr_tst[i].hzu_f,
        arr_tst[i].servo_cnt, arr_tst[i].servo_cnt_ref,
        arr_tst[i].servo_cnt_cur, arr_tst[i].hsvd,
        arr_tst[i].n, arr_tst[i].n_ref);

      // Подстановка запятых вместо точек
      char *p = strchr(s, '.');
      for(;;) {
        if (p) {
          *p = ',';
          p = strchr(s, '.');
        } else {
          break;
        }
      }

      fprintf(f, s);
    }
    fclose(f);
    test_save_f = 0;
  }
}
// Обработка теста
void test_proc()
{
  if (arr_tst || !test_save_f) {
    static counter_t to_test = 0;
    static int start_test_s = 0;

    if (chk_event(&start_test_s, start_test, 2)) {
      if (start_test) {
        i_arr_tst = 0;
        st_counter = counter;
        dtms_test = dt_test*1000.;
        set_to_cnt_t(to_test, dtms_test);
      } else {
        test_save_f = 1;
      }
    }

    if (start_test) {
      if (test_to_cnt_t(to_test)) {
        next_to_cnt_t(to_test, dtms_test);
        if (i_arr_tst < n_arr_tst) {
          arr_tst[i_arr_tst].time = cnttoms(counter - st_counter)/1e3;
          arr_tst[i_arr_tst].hzu = hzu;
          arr_tst[i_arr_tst].hzu_f = hzu_f;
          arr_tst[i_arr_tst].servo_cnt = servo_cnt;
          arr_tst[i_arr_tst].servo_cnt_ref = servo_cnt_ref;
          arr_tst[i_arr_tst].servo_cnt_cur = servo_cnt_cur;
          arr_tst[i_arr_tst].hsvd = hsvd;
          arr_tst[i_arr_tst].n = n;
          arr_tst[i_arr_tst].n_ref = n_ref;
          i_arr_tst++;
        } else {
          start_test = 0;
        }
      }
    }
  }
}
#endif //TEST
// Выполнение начальной инициализации
void init()
{
  #ifndef DEBUG
  // Инициализация сокетов
  sock_init();
  #ifdef DPRINTF_UDP
  if (udp_open(&udp_log_sock, MXLOGUDP_PORT, 0xffffffffL, MXLOGUDP_PORT, NULL)) {
    udp_log_opened = 1;
  } else {
    dputs("error udp_open for log");
  }
  //sock_mode(&udp_log_sock, TCP_MODE_ASCII);
  #endif //DPRINTF_UDP
  #ifndef DBGISVT
  // Задержка перед инициализацией
  dputs("Задержка 4 с...");
  delay(4000);
  #endif //DBGISVT
  #endif //DEBUG
  // Инициализация структур для обработки сетевых переменных
  ldata.tact = 200.;
  ldata.port = MXINDTCP_PORT;
  ldata.fn = (void *)lan01;
  ldata.udp_buf = malloc(udp_buf_size);
  if (!ldata.udp_buf) dputs("Ошибка выделения памяти для ldata.udp_buf");
  ldata2.tact = dt_lan2;
  ldata2.port = MXINDTCP_PORT2;
  ldata2.fn = (void *)lan01;
  ldata2.udp_buf = malloc(udp_buf_size);
  if (!ldata2.udp_buf) dputs("Ошибка выделения памяти для ldata2.udp_buf");
  // Чтение сетевых переменных из флэш
  for (int i = 0; i < CNT_VAR; i++) vbuf[i] = 1;
  if (readbuf(wbuf)) {
    filtrbuf();
    #ifndef LAN_SINGLE
    filtrbufex();
    #endif //LAN_SINGLE
  }
  if (restart_flag) count_restart++;
  restart_flag = 1L;
  need_writebuf = 1;
  #ifndef LAN_SINGLE
  // Чтение ini-файла
  #ifndef IGNORE_INI
  readini();
  #endif //IGNORE_INI

  #ifdef NEW_FADC
  // Данные для инерционных звеньев фильтров АЦП
  {for (int i = 0; i < 24; i++) {
    adcf[i].y1 = INT_MIN;
    adcf[i].t = n_ai_filtr;
  }}
  #ifdef DBGONCPU
  dbg_proc_int();
  #endif //DBGONCPU
  #endif //NEW_FADC

  #ifndef DEBUG
  // Инициализация UNIO
  unio_init();
  // Инициализация AI
  ai_init();
  // Инициализация ИСВТ
  isvt_init();
  #endif //DEBUG

  #ifdef TEST
  // Инициализация теста
  test_init();
  #endif //TEST

  // Инициализация ограничителей темпа
  //  РМ
  rm_ot.cur = ne;
  rm_ot.dt = dt;
  rm_ot.slope = ne_ref_sl;
  rm_ot.recalc = 1;
  ogr_temp(&rm_ot, rm_ot.cur);
  //  РТО1
  rto1_ot.cur = get_t_rto1();
  rto1_ot.dt = dt;
  rto1_ot.slope = t_ref1_sl;
  rto1_ot.recalc = 1;
  ogr_temp(&rto1_ot, rto1_ot.cur);
  //  РТО2
  rto2_ot.cur = get_t_rto2();
  rto2_ot.dt = dt;
  rto2_ot.slope = t_ref2_sl;
  rto2_ot.recalc = 1;
  ogr_temp(&rto2_ot, rto2_ot.cur);
  //  РТПВ
  rtpv_ot.cur = tpv;
  rtpv_ot.dt = dt;
  rtpv_ot.slope = tpv_ref_sl;
  rtpv_ot.recalc = 1;
  ogr_temp(&rtpv_ot, rtpv_ot.cur);

  // Инициализация таймаутов
  set_to_cnt_t(to_dt, dtms);

  #ifndef DEBUG
  // Инициализация прерывания по каналу 0 таймера
  #ifndef ADC_IN_LOOP
  init_timer0();
  {
    for(int i = 0; i < 3; i++)
      get_ai_all();        
  }
  #endif //ADC_IN_LOOP
  #ifdef WDT_ON
  // Запуск сторожевого таймера
  outp(0x20C, 1);
  #endif //WDT_ON
  #endif //DEBUG

  #endif //LAN_SINGLE
}
// Выполнение деинициализации
void deinit()
{
  #ifndef LAN_SINGLE
  #ifndef DEBUG
  #ifndef ADC_IN_LOOP
  // Деинициализация прерываний по каналу 0 таймера
  deinit_timer0();
  #endif //ADC_IN_LOOP
  #endif //DEBUG
  #ifdef TEST
  // Деинициализация теста
  test_deinit();
  #endif //TEST
  // Останов сторожевого таймера
  #ifndef DEBUG
  #ifdef WDT_ON
  if (!exit_wdt_on)
    outp(0x20C, 0);
  #endif //WDT_ON
  // Деинициализация AI
  ai_deinit();
  // Деинициализация UNIO
  unio_deinit();
  #endif //DEBUG
  #endif //LAN_SINGLE
  #ifndef DEBUG
  // Освобождение памяти UDP-буферов
  free(ldata.udp_buf);
  free(ldata2.udp_buf);
  #ifdef DPRINTF_UDP
  if (udp_log_opened) sock_close(&udp_log_sock);
  //sock_mode(&udp_log_sock, TCP_MODE_ASCII);
  #endif //DPRINTF_UDP
  #endif //DEBUG
}
// Ф-ции обработки сетевых переменных
void lan01(landata *ld)
{
  if (!udp_open(&ld->us, ld->port, 0xffffffffL, ld->port, NULL))
    dputs("Ошибка в udp_open");
  if (sock_recv_init((void *)&ld->us, (char *)ld->udp_buf, (unsigned)udp_buf_size)) 
    dputs("Error in sock_recv_init");
  set_to_cnt_t(ld->to, ld->tact);
  ld->fn = (void *)lan02;
}
//   Подготовка буфера для UDP
void prepare_ubuf()
{
  ubuf[0] = 0xFF43439A;
  ubuf[1] = 0xE81F3AAD;
  check_sum = 0;
  sync(memcpy(ubuf + 2, buf, 4*CNT_VAR));
  long  cs = 0;
  for (int i = 0; i < CNT_VAR; i++) cs += ubuf[i + 2];
  ubuf[CNT_VAR] = cs;
}
//   Передача на терминал и обработка прочих сетевых операции
void lan02(landata *ld)
{
  DWORD ip;
  WORD port;
  tcp_tick(NULL);
  if (test_to_cnt_t(ld->to)) {
    set_to_cnt_t(ld->to, ld->tact);
    #ifdef LAN_RECEIVE_NOT_BLOCKING
    if (lan_en) {
    #endif //LAN_RECEIVE_NOT_BLOCKING
    prepare_ubuf();
    //int len =
    sock_write((void *)&ld->us, (char *)ubuf, (int)sizeof(ubuf));
    //dprintf("data send %d\n", len);
    #ifdef LAN_RECEIVE_NOT_BLOCKING
    }
    #endif //LAN_RECEIVE_NOT_BLOCKING
  }
  long tbuf[5];
  int res = sock_recv_from((void *)&ld->us, &ip, &port, (char *)tbuf, (unsigned)sizeof(ubuf), 0);
  if (res > 0) {
    //char ips[20];
    char *st = NULL;
    char *s1 = "Неправильное начало пакета";
    char *s2 = "Ошибка контрольной суммы";
    char *s3 = "Номер переменной вне диапазона";
    char *s4 = "Несоответствие размера пакета";
    if (res != 20) {
      st = s4;
    } else if ((tbuf[0] != (long)0xFF43439A) || (tbuf[1] != (long)0xE81F3AAD)) {
      st = s1;
    } else if (tbuf[2] + tbuf[3] != tbuf[4]) {
      st = s2;
    } else if ((tbuf[2] < 0) || (tbuf[2] >= CNT_VAR)) {
      st = s3;
    }
    if (st) {
      struct in_addr ip_addr = {ip};
      dprintf("%s:%hd - %s\n", inet_ntoa(ip_addr), ntohs(port), st);
      //dprintf("%s:%d - %s\n", inet_ntoa(ips, ntohl(ip)), ntohs(port), st);
    } else {
      memset(vbuf, 0, sizeof(vbuf));
      vbuf[tbuf[2]] = 1;
      wbuf[tbuf[2]] = tbuf[3];
      filtrbuf();
      if (tbuf[2] != CNT_VAR - 37)
      {
      need_writebuf = 1;
      struct in_addr ip_addr = {ip};
      dprintf("%s:%hd - %ld: %ld, 0x%08lX, %g\n", inet_ntoa(ip_addr),
        ntohs(port), tbuf[2], tbuf[3], tbuf[3], *(float *)&tbuf[3]);
      //dprintf("%s:%d - %ld: %ld, 0x%08lX, %g\n", inet_ntoa(ips, ntohl(ip)),
        //ntohs(port), tbuf[2], tbuf[3], tbuf[3], *(float *)&tbuf[3]);
      }
    }
  }
}
#ifndef DEBUG
#ifndef ADC_IN_LOOP
// Установка своего прерывания на канал 0 таймера
void init_timer0()
{
  _disable();
  outp(0x40, TIMER0_CNT);
  outp(0x40, TIMER0_CNT >> 8);
  oti0 = _dos_getvect(TIMER0_VECTOR);
  _dos_setvect(TIMER0_VECTOR, ti0);
  _enable();
}
// Восстановление старого прерывания на канале 0 таймера
void deinit_timer0()
{
  _disable();
  outp(0x40, 0);
  outp(0x40, 0);
  _dos_setvect(TIMER0_VECTOR, oti0);
  _enable();
}
#endif //ADC_IN_LOOP
#endif
// Обработка данных плат ИСВТ
int isvt_proc()
{
  int res = 0;

  #ifdef DEBUG

  res = 1;

  #else //DEBUG
  
  int cnt_isvt = 3;

  #ifdef DBGONCPU
  #ifndef DBGISVT
  cnt_isvt = 1;
  #else //DBGISVT
  cnt_isvt = 0;
  #endif //DBGISVT
  #endif //DBGONCPU


  for (int i = 0; i < cnt_isvt; i++) {
    uint ba = *isvt_ba[i];
    byte Status1, Status2;
    uint n12oi;
    int d_n12oi;
    uint n12ti_tmp;
    int d_n12ti_tmp;
    sync_isvt(int r = isvt_proc_r(ba, &Status1, &Status2, n1m[i], &dn1m[i],
        n2m[i], &dn2m[i], &n12ti_tmp, &d_n12ti_tmp, &n12oi, &d_n12oi));
    res |= (r << i);
    // Чтение данных
    // Считанные пороги по частоте
    *n12tgm[i] = n12ti_tmp/10.f;
    // Текущие частота и производная частоты
    if (1 == sensor_select[i]) {
      *n12m[i] = float(*n2m[i]/10.f);
    } else if (0 == sensor_select[i]) {
      *n12m[i] = float(*n1m[i]/10.f);
    } else {
      *n12m[i] = 0.f;
      if ((*n2m[i]/10.f) > 3.f) *n12m[i] = float(*n2m[i]/10.f);
      if ((*n1m[i]/10.f) > 3.f) *n12m[i] = float(*n1m[i]/10.f);
    }
    // Частота при которой сработала защита
    *n12om[i] = float(n12oi/10.f);
    // Статус
    if (Status2&Bit_Status2_Defense_Freq)
      isvt_st |= long(1 << (2*i));
    else
      isvt_st &= 0xFFFFFFFF^long(1 << (2*i));
    if (Status2&Bit_Status2_Defense_Acceleration)
      isvt_st |= long(2 << (2*i));
    else
      isvt_st &= 0xFFFFFFFF^long(2 << (2*i));
    #ifdef OFF_BAD_RELE_ISVT
    isvt_rele = 0x3F;
    #else
    // Исправность реле
    if ((Status1&Bit_Status1_Error_Rele1))
      isvt_rele &= 0xFFFFFFFF^long(1 << (2*i));
    else
      isvt_rele |= long(1 << (2*i));
    if ((Status1&Bit_Status1_Error_Rele2))
      isvt_rele &= 0xFFFFFFFF^long(1 << (2*i + 1));
    else
      isvt_rele |= long(1 << (2*i + 1));
    #endif
  }
  {
    for (int i = 0; i < 6; i++) *nfm[i] = *nm[i]/10.f;
  }

  select_isvt_sensor();

  valve_test();

  #endif //DEBUG

  return res;
}
// Взведение i-го золотника (0-2)
void vzv_zz(int i)
{
  uint ba = *isvt_ba[i];
  uint n12ti = uint(*n12tm[i]*10.f);
  #ifdef SENS_SEL_BLOCK_ON_VALVE_TEST
  // Блокировка переключения датчиков при расхаживании  
  int isvt_sensor = isvt_sensor_from_sensor_select(sensor_select[i]);
  sync_isvt(isvt_init_r(ba, n12ti, isvt_sensor));
  #else //SENS_SEL_BLOCK_ON_VALVE_TEST
  sync_isvt(isvt_init_r(ba, n12ti, 0));
  sensor_select[i] = 0;
  #endif //SENS_SEL_BLOCK_ON_VALVE_TEST
}
// Взведение всех золотников
void vzv_zz_all()
{
  for (int i = 0; i < 3; i++) vzv_zz(i);
}
// Сброс i-го золотника (0-2)
void sbr_zz(int i)
{
  uint ba = *isvt_ba[i];
  sync_isvt(isvt_deinit_r(ba));
}
// Тестирование i-го золотника (0-2)
void tst_zz(int i)
{
  uint ba = *isvt_ba[i];
  uint t_n = uint(test_n*10.f);
  int t_dn = int(test_dn*10.f);
  sync_isvt(isvt_test_init_r(ba, t_n, t_dn));
  //sync_isvt(isvt_test_init_r(ba, 25000u, 2000));
}
// Расхаживание клапанов
void valve_test()
{
  #ifdef DBGONCPU
  double t_srab = 3000.;
  static counter_t to_srab;
  static long mode_testn_prev = 0;
  if (mode_testn != mode_testn_prev) {
    if (1 == mode_testn) {
      testn_st = 2;
      set_to_cnt_t(to_srab, t_srab);
    }
  }
  mode_testn_prev = mode_testn;
  if (1 == mode_testn)
  if (test_to_cnt_t(to_srab)) {
    testn_st = 1;
    mode_testn = 0;
  }
  #else //DBGONCPU
  double t_srab = 10000.;
  #ifdef VALVE_TEST_TIMEOUT_BIGGER
  double t_vzv = 30000.;
  #else //VALVE_TEST_TIMEOUT_BIGGER
  double t_vzv = 3000.;
  #endif //VALVE_TEST_TIMEOUT_BIGGER
  double t_wait = 1000.;
  static int step_test = 0;
  static long mode_testn_prev = 0;
  static counter_t to_srab;
  static counter_t to_vzv;
  static counter_t to_wait;
  static int f_wait = 0;
  if (start) {
    if (mode_testn != mode_testn_prev) {
      if (1 == mode_testn) {
        testn_st = 2;
        step_test = 1;
        valve_test_proc = 1;
      }
      if (0 == mode_testn) {
        step_test = 0;
        reset_fail();
        // Текущая тестируемая плата ИСВТ
        cur_test_isvt = -1;
        f_wait = 1;
        set_to_cnt_t(to_wait, t_wait);
      }
    }
  } else {
    if (1 == mode_testn) {
      testn_st = 2;
      step_test = 0;
      mode_testn = 0;
    }
  }
  mode_testn_prev = mode_testn;
  switch (step_test) {
    case 1:
      if (7 == zz_st&7) {
        cur_test_isvt = 0;
        #ifdef VALVE_GENER
        tst_zz(0);
        #else
        sbr_zz(0);
        #endif
        step_test = 2;
        set_to_cnt_t(to_srab, t_srab);
      } else {
        testn_st = 2;
        mode_testn = 0;
        step_test = 0;
      }
      break;
    case 2:
      if (test_to_cnt_t(to_srab)) {
        if (zz_st&1) {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        } else {
          step_test = 3;
          set_to_cnt_t(to_vzv, t_vzv);
        }
        vzv_zz(0);
        //reset_fail();
      }
      break;
    case 3:
      if (test_to_cnt_t(to_vzv)) {
        if (zz_st&1) {
          step_test = 4;
        } else {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        }
      }
      break;




    case 4:
      if (7 == zz_st&7) {
        cur_test_isvt = 1;
        #ifdef VALVE_GENER
        tst_zz(1);
        #else
        sbr_zz(1);
        #endif
        step_test = 5;
        set_to_cnt_t(to_srab, t_srab);
      } else {
        testn_st = 2;
        mode_testn = 0;
        step_test = 0;
      }
      break;
    case 5:
      if (test_to_cnt_t(to_srab)) {
        if (zz_st&2) {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        } else {
          step_test = 6;
          set_to_cnt_t(to_vzv, t_vzv);
        }
        vzv_zz(1);
        //reset_fail();
      }
      break;
    case 6:
      if (test_to_cnt_t(to_vzv)) {
        if (zz_st&2) {
          step_test = 7;
        } else {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        }
      }
      break;



    case 7:
      if (7 == zz_st&7) {
        cur_test_isvt = 2;
        #ifdef VALVE_GENER
        tst_zz(2);
        #else
        sbr_zz(2);
        #endif
        step_test = 8;
        set_to_cnt_t(to_srab, t_srab);
      } else {
        testn_st = 2;
        mode_testn = 0;
        step_test = 0;
      }
      break;
    case 8:
      if (test_to_cnt_t(to_srab)) {
        if (zz_st&4) {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        } else {
          step_test = 9;
          set_to_cnt_t(to_vzv, t_vzv);
        }
        vzv_zz(2);
        //reset_fail();
      }
      break;
    case 9:
      if (test_to_cnt_t(to_vzv)) {
        if (zz_st&4) {
          testn_st = 1;
          mode_testn = 0;
          step_test = 0;
        } else {
          testn_st = 2;
          mode_testn = 0;
          step_test = 0;
        }
      }
      break;
  }

  if (f_wait)
  if (test_to_cnt_t(to_wait)) {
    f_wait = 0;
    valve_test_proc = 0;
  }
  #endif //DBGONCPU
}
// Выбор текущих датчиков частоты ИСВТ
void select_isvt_sensor()
{
  #ifdef DBGONCPU
  #ifndef DBGISVT
  static int _init = 1;
  if (_init) {
    _init = 0;
    uint ba = *isvt_ba[0];
    sync_isvt(set_isvt_sensor_r(ba, 1));
    n_all_bad_isvt = 0;
    #ifdef PERM_TEST_GENER
    test_n = 3000.f;
    test_dn = 0.f;
    tst_zz(0);
    #endif //PERM_TEST_GENER
  }
  n = *nm[0]/10.;
  #endif //DBGISVT
  #else //DBGONCPU
  
  #ifdef BLOCK_SELECT_N_VALVE_TEST_ISVT
  int schet_sovp[sensor_count];
  memset(schet_sovp, 0, sensor_count*sizeof(int));
  bool blocked_sensors[sensor_count];
  memset(blocked_sensors, 0, sensor_count*sizeof(bool));
  
  // Блокируем датчики с неправильными частотами
  for (int sensor_idx = 0; sensor_idx < sensor_count; sensor_idx++) {
    if ((*nm[sensor_idx] < 30u) || (*nm[sensor_idx] > 40000u)) {
      blocked_sensors[sensor_idx] = true;
    }
  }

  // Блокировка выбора тестируемых датчиков
  if (cur_test_isvt != -1) {
    blocked_sensors[2*cur_test_isvt] = true;
    blocked_sensors[2*cur_test_isvt + 1] = true;
  }

  for (int first_sensor_idx = 0; first_sensor_idx < sensor_count; 
    first_sensor_idx++) {
    if (!blocked_sensors[first_sensor_idx]) {
      int sum = 0;
      for (int second_sensor_idx = 0; second_sensor_idx < sensor_count; 
        second_sensor_idx++) {
        if ((first_sensor_idx != second_sensor_idx) && 
          (!blocked_sensors[second_sensor_idx])) {
          #ifdef SENS_SEL_REDUCE_RANGE_EQUAL
          if (abs(*nm[first_sensor_idx] - *nm[second_sensor_idx]) <= 30u) {
          #else //SENS_SEL_REDUCE_RANGE_EQUAL
          if (abs(*nm[first_sensor_idx] - *nm[second_sensor_idx]) <= 100u) {
          #endif //SENS_SEL_REDUCE_RANGE_EQUAL
            sum++;
          }
        }
      }
      schet_sovp[first_sensor_idx] = sum;
    }
  }
  #else //BLOCK_SELECT_N_VALVE_TEST_ISVT
  int schet_sovp[6] = {1, 1, 1, 1, 1, 1};
  int i, j;
  for (i = 0; i < 6; i++) {
    if ((*nm[i] < 30u) || (*nm[i] > 40000u)) schet_sovp[i] = 0;
  }

  // Блокировка выбора тестируемых датчиков
  if (cur_test_isvt != -1) {
    schet_sovp[2*cur_test_isvt] = 0;
    schet_sovp[2*cur_test_isvt + 1] = 0;
  }

  for (i = 0; i < 6; i++) {
    if (schet_sovp[i] != 0) {
      int sum = 0;
      for (j = 0; j < 6; j++) {
        if ((i != j) && (schet_sovp[j] != 0)) {
          if (abs(*nm[i] - *nm[j]) <= 100u) sum++;
        }
      }
      schet_sovp[i] = sum;
    }
  }
  #endif //BLOCK_SELECT_N_VALVE_TEST_ISVT

  #ifndef NEW_SENS_SEL
  #ifdef SENS_SEL_BLOCK_ON_VALVE_TEST
  if (!valve_test_proc)
  for (int i = 0; i < isvt_freq_count; i++) {
    uint ba = *isvt_ba[i];
    const bool is_sensor_choice = (sensor_none != sensor_select[i]);
    const int schet_sovp_cur_sensor = schet_sovp[2*i + sensor_select[i]];
    const int schet_sovp_first_sensor = schet_sovp[2*i];
    const int schet_sovp_second_sensor = schet_sovp[2*i + 1];
    if (!is_sensor_choice || (schet_sovp_cur_sensor < 2)) {
      if (schet_sovp_first_sensor >= 1) {
        sensor_select[i] = sensor_first;
        sync_isvt(set_isvt_sensor_r(ba, isvt_sensor_first));
      } else if (schet_sovp_second_sensor >= 1) {
        sensor_select[i] = sensor_second;
        sync_isvt(set_isvt_sensor_r(ba, isvt_sensor_second));
      }
    }
  }
  #else //SENS_SEL_BLOCK_ON_VALVE_TEST
  if (!valve_test_proc)
  for (i = 0; i < 3; i++) {
    uint ba = *isvt_ba[i];
    if (!((-1 == sensor_select[i]) && (n_bad&(1 << i)))) {
      if ((-1 == sensor_select[i]) ||
        (schet_sovp[2*i + sensor_select[i]] < 2)) {
        if (schet_sovp[2*i] >= 1) {
          sensor_select[i] = 0;
          sync_isvt(set_isvt_sensor_r(ba, 1));
        } else if (schet_sovp[2*i + 1] >= 1) {
          sensor_select[i] = 1;
          sync_isvt(set_isvt_sensor_r(ba, 2));
        } else {
          sensor_select[i] = -1;
          sync_isvt(set_isvt_sensor_r(ba, 0));
        }
      }
    } else {
      if (n_bad&(1 << i)) {
        sensor_select[i] = -1;
        sync_isvt(set_isvt_sensor_r(ba, 0));
      }
    }
  }
  #endif //SENS_SEL_BLOCK_ON_VALVE_TEST
  #endif //NEW_SENS_SEL

  int n_all_bad_isvt_no_unblocked_sensors = 0;
  #ifdef BLOCK_SELECT_N_VALVE_TEST_ISVT
  int i_sovp = 0;
  int max_sovp = schet_sovp[i_sovp];
  
  // Поиск первого исправного датчика
  n_all_bad_isvt_no_unblocked_sensors = 1; // Если все датчики блокированы
  for (int sensor_idx = 0; sensor_idx < sensor_count; sensor_idx++) {
    if (!blocked_sensors[sensor_idx]) {
      // Если хотя бы один датчик исправен, то
      n_all_bad_isvt_no_unblocked_sensors = 0;
      i_sovp = sensor_idx;
      max_sovp = schet_sovp[i_sovp];
      break;
    }
  }
  
  if (!n_all_bad_isvt_no_unblocked_sensors) {
    // Поиск датчика с максимальным количеством совпадений
    for (int sensor_idx = i_sovp; sensor_idx < sensor_count; sensor_idx++) {
      if ((!blocked_sensors[sensor_idx]) && 
      	((max_sovp < schet_sovp[sensor_idx]))) 
      {
        max_sovp = schet_sovp[sensor_idx];
        i_sovp = sensor_idx;
      }
    }
  }
  #else //BLOCK_SELECT_N_VALVE_TEST_ISVT
  int max_sovp = 0, i_sovp = 0;
  max_sovp = schet_sovp[0];
  i_sovp = 0;
  
  for (i = 1; i < 6; i++) {
    if (max_sovp < schet_sovp[i]) {
      max_sovp = schet_sovp[i];
      i_sovp = i;
    }
  }
  #endif //BLOCK_SELECT_N_VALVE_TEST_ISVT

  #ifdef NEW_SENS_SEL
  if (!valve_test_proc)
  for (i = 0; i < 3; i++) {
    uint ba = *isvt_ba[i];
    int f_old = 0;
    if (-1 == sensor_select[i]) {
      if ((abs(*nm[2*i] - *nm[i_sovp]) <= 100u) &&
        (schet_sovp[2*i] >= 1)) {
        sensor_select[i] = 0;
      } else if ((abs(*nm[2*i + 1] - *nm[i_sovp]) <= 100u) &&
        (schet_sovp[2*i + 1] >= 1)) {
        sensor_select[i] = 1;
      } else {
        f_old = 1;
      }
    } else if (abs(*nm[2*i + sensor_select[i]] - *nm[i_sovp]) > 100u ||
      (schet_sovp[2*i + sensor_select[i]] < 1)) {
      if ((abs(*nm[2*i + 1 - sensor_select[i]] - *nm[i_sovp]) <= 100u) &&
        (schet_sovp[2*i + 1 - sensor_select[i]] >= 1)) {
        sensor_select[i] = 1 - sensor_select[i];
      } else {
        f_old = 1;
      }
    }

    if (f_old) {
      if (!((-1 == sensor_select[i]) && (n_bad&(1 << i)))) {
        if ((-1 == sensor_select[i]) ||
          (schet_sovp[2*i + sensor_select[i]] < 2)) {
          if (schet_sovp[2*i] >= 2) {
            sensor_select[i] = 0;
            sync_isvt(set_isvt_sensor_r(ba, 1));
          } else if (schet_sovp[2*i + 1] >= 2) {
            sensor_select[i] = 1;
            sync_isvt(set_isvt_sensor_r(ba, 2));
          } else {
            sensor_select[i] = -1;
            sync_isvt(set_isvt_sensor_r(ba, 0));
          }
        }
      } else {
        if (n_bad&(1 << i)) {
          sensor_select[i] = -1;
          sync_isvt(set_isvt_sensor_r(ba, 0));
        }
      }
    }
  }
  #endif //NEW_SENS_SEL

  int n_all_bad_isvt_max_sovp_low = 0;
  if (max_sovp < 2) {
    n_all_bad_isvt_max_sovp_low = 1; // Если мало совпадений
  }
  int n_all_bad_isvt_n_over_range = 0;
  float n_for_range_check = *nm[i_sovp]/10.f;
  if (kg_v) {
    if ((3700.f < n_for_range_check) || (n_for_range_check < 2300.f)) {
      n_all_bad_isvt_n_over_range = 1; // Если частота неадекватна
    }
  }
  if (n_all_bad_isvt_max_sovp_low || n_all_bad_isvt_no_unblocked_sensors ||
    n_all_bad_isvt_n_over_range) {
    n_all_bad_isvt = 1; // Все датчики неисправны
  } else {
    n_all_bad_isvt = 0; // Есть исправные датчики
  }
  #ifdef NOP
  // Частота 3000 в работе
  i_sovp = i_sovp;
  n = 3000.f;
  #else //NOP
  if ((!n_all_bad_isvt) || (!kg_v)) {
    n = *nm[i_sovp]/10.f;
  }
  #endif //NOP
  
  #endif //DBGONCPU
  
  #ifdef DETECT_N_ANOMAL
  // Детектирование общей аномальной частоты
  {
    static byte _init = 1;
    static n_anomal_data n_anomal_v;
    if (_init) {
      _init = 0;
      n_anomal_v._init = 1;
    }
    n_anomal_v.detect = 0;
    n_anomal_proc(_in n, _in_out &n_anomal_v);
    if (n_anomal_v.detect) {
      n_anomal_cur = n_anomal_v.n_anomal_cur;
      n_anomal_prev = n_anomal_v.n_anomal_prev;
      time_n_anomal = n_anomal_v.time_n_anomal;
    }
  }
  // Детектирование аномальной частоты по конкретным датчикам
  {
    static byte _init = 1;
    static nx_anomal_data nx_anomal_a;
    nx_anomal_proc(_in (const uint **)nm, _in_out _init, _out nx_anomal_cur,
      _out nx_anomal_prev, _out time_nx_anomal, _out ind_nx_anomal,
      _internal &nx_anomal_a);
  }
  #endif //DETECT_N_ANOMAL
}
// Чтение значения текущего датчика температуры РТО1
float get_t_rto1()
{
  float t12 = t1;
  if (2 == t_rto1_sel) t12 = t2;
  return t12;
}
// Чтение значения текущего датчика температуры РТО2
float get_t_rto2()
{
  float t12 = t1;
  if (2 == t_rto2_sel) t12 = t2;
  return t12;
}
#ifdef RES_PKT_DRV
// Процедура сброса пакетного драйвера
void res_pd_proc()
{
  // Инициализация
  #ifdef RES_PKT_DRV_DBG
  // Интервал перезагрузки пакетного драйвера 10 с для отладки
  #ifdef RES_PKT_SWITCH
  static double t_slc = 40.*1e3;
  static double t_off = 5.*1e3;
  static double t_on = 20.*1e3;
  #else //RES_PKT_SWITCH
  static double t_slc = 10.*1e3;
  #endif //RES_PKT_SWITCH
  #else //RES_PKT_DRV_DBG
  // Интервал перезагрузки пакетного драйвера 1 ч для нормального режима
  static double t_slc = 60.*60.*1e3;
  #ifdef RES_PKT_SWITCH
  static double t_off = 10.*1e3;
  static double t_on = 5.*60.*1e3;
  #endif //RES_PKT_SWITCH
  #endif //RES_PKT_DRV_DBG
  #ifdef RES_PKT_SWITCH
  static counter_t to_off;
  static counter_t to_on;
  static int f_on = 0;
  static int f_off = 0;
  #endif //RES_PKT_SWITCH
  static counter_t to_slc;
  static int _init = 1;
  if (_init) {
    _init = 0;
    set_to_cnt_t(to_slc, t_slc);
  }
  // Проверка состояния счетчика
  static long suc_lan_counter_p = 0;
  if (suc_lan_counter != suc_lan_counter_p) {
    set_to_cnt_t(to_slc, t_slc);
    #ifdef RES_PKT_SWITCH
    f_on = 0;
    #endif //RES_PKT_SWITCH
  }
  suc_lan_counter_p = suc_lan_counter;
  // Перезагрузка пакетного драйвера, через час после остановки счетчика
  if (test_to_cnt_t(to_slc)) {
    set_to_cnt_t(to_slc, t_slc);
    #ifdef RES_PKT_SWITCH
    f_off = 1;
    set_to_cnt_t(to_off, t_off);
    unioset(switch_off, 1);
    f_on = 1;
    set_to_cnt_t(to_on, t_on);
    dputs("switch shutdown");
    #else //RES_PKT_SWITCH
    reinit_packet_drv();
    #endif //RES_PKT_SWITCH
  }
  #ifdef RES_PKT_SWITCH
  if (f_off)
  if (test_to_cnt_t(to_off)) {
    f_off = 0;
    unioset(switch_off, 0);
    dputs("switch power on");
    cnt_res_sw++;
  }
  if (f_on)
  if (test_to_cnt_t(to_on)) {
    f_on = 0;
    reinit_packet_drv();
  }
  #else //RES_PKT_SWITCH
  #endif //RES_PKT_SWITCH
}
// Функция реинициализации пакетного драйвера
void reinit_packet_drv()
{
  #ifdef RES_PKT_CPU
  stop_loop = 1;
  exit_wdt_on = 1;
  cnt_res_cpu++;
  #else //RES_PKT_CPU
  dputs("\nbegin reinit packet driver");
  #ifdef NOP
  // Закрытие сокетов
  sock_close(&ldata.us);
  sock_close(&ldata2.us);
  sock_exit();
  dputs("after sock_exit");
  #else
  //_eth_release();
  //pkt_release_handle(_pkt_inf->handle);
  //pkt_release_handle(0);
  pkt_stop();
  #endif

  #ifndef NOP
  bl_proc = 1;

  #ifdef NOP
  #ifndef RES_PKT_DRV_DELAY_OFF
  {
    counter_t to_wait;
    set_to_cnt_t(to_wait, 500.);
    while(!test_to_cnt_t(to_wait));
  }
  dputs("after wait 01");
  #endif //RES_PKT_DRV_DELAY_OFF
  #endif

  //outp(0x21, 1|inp(0x21));
  //long cnt1 = counter;

  // Деинсталяция пакетного драйвера
  int r = system("c83815 -u > NUL");
  //int r = system("c83815 -u");
  //int r = spawnlp(P_WAIT, "c83815", "-u", NULL);
  if (r == -1) perror("uninstall c83815 error");
  dputs("\nafter c83815 -u");
  // Установка пакетного драйвера
  r = system("c83815 0x7e > NUL");
  //r = system("c83815 0x7e");
  //r = spawnlp(P_WAIT, "c83815", "0x7e", NULL);
  if (r == -1) perror("install c83815 error");
  dputs("after c83815 0x7e");

  //long cnt2 = counter;
  //outp(0x21, 0xFE&inp(0x21));
  //dprintf("cnt1 = %d; cnt2 = %d\n", cnt1, cnt2);

  #ifndef RES_PKT_DRV_DELAY_OFF
  {
    counter_t to_wait;
    set_to_cnt_t(to_wait, 500.);
    while(!test_to_cnt_t(to_wait));
  }
  dputs("after wait");
  #endif //RES_PKT_DRV_DELAY_OFF
  _fpreset();
  dputs("after _fpreset");
  bl_proc = 0;
  #endif

  #ifdef NOP
  // Открытие сокетов
  ldata.fn = (void *)lan01;
  ldata2.fn = (void *)lan01;
  dputs("after lan01");
  sock_init();
  #else
  //_eth_init();
  //pkt_set_access();
  pkt_start();
  #endif
  cnt_res_pd++;
  dprintf("\n%u - end reinit packet driver\n", cnt_res_pd);
  //dputs("\nend reinit packet driver");
  #endif //RES_PKT_CPU
}
#endif //RES_PKT_DRV
#ifdef DBGONCPU
// Модель турбины в режиме отладки
void dbg_proc_int()
{
  static fade_data hzu_fd;
  static fade_data hsvd_fd;
  static fade_data n_fd;
  static fade_data ne_fd;
  // Вакуум в конденсаторе
  float vk_ = -0.5f;
  // Давление в регулирующей ступени
  float prs_ = 1.f;
  // Давление масла на регулирование
  float prm_ = 0.f;
  // Давление импульсного масла
  float pim_ = 0.f;
  // Давление в нижнем отборе 1
  float pnto1_ = 0.f;
  // Давление в нижнем отборе 2
  float pnto2_ = 0.f;
  // Давление сетевой воды
  float psv_ = 0.f;
  // Давление острого пара
  float pt_ = 1.f;
  // Положение СВД
  float hsvd_ = 0.f;
  // Положение СНД
  float hesnd_ = 1.f;
  // Температура подпиточной воды
  float tpv_ = 0.f;
  // Температура сетевой воды 1
  float t1_ = 0.f;
  // Температура сетевой воды 2
  float t2_ = 0.f;
  // Температура обратной сетевой воды
  float tobr_ = 0.f;
  // Электрическая мощность
  float ne_ = 0.f;
  float *fadc[15] = {&pim_, &prm_, &ne_, &prs_, &pt_, &pnto1_, &pnto2_, &tpv_,
    &t1_, &t2_, &tobr_, &vk_, &psv_, &hesnd_, &hsvd_};
  double hsvd_3000 = 50.f;
  double hsvd_110 = 212.;
  double hsvd_0 = 50.;

  // Инициализация дискретных входов при запуске
  static int _init = 1;
  if (_init) {
    _init = 0;
    opt_sw = 1;
    nd_sw = 2;
    srv_alm_v = 0;
    ask_otkr_v = int(1L&(dbg_state >> 1));
    ask_zakr_v = int(1L - (1L&(dbg_state >> 1)));
    k_stop_v = int(1L&dbg_state);
    dzz1_v = int(1L&(dbg_state >> 2));
    dzz2_v = int(1L&(dbg_state >> 3));
    dzz3_v = int(1L&(dbg_state >> 4));
    vd_sw = int(3L&(dbg_state >> 5));
    kg_v = int(1L&(dbg_state >> 7));
    if (vd_sw == 0) vd_sw = 3;
    #ifdef NOP
    if (hzu_hsvd_k != 0) {
      double hzu_ = scnt_to_grad(dbg_scnt);
      hsvd_ = bound(double((hzu_ - hzu_hsvd_b)/hzu_hsvd_k), 0., 212.);
    } else {
      hsvd_ = 0;
    }
    if (kg_v) {
      ne_ = 110.*(hsvd_ - hsvd_0)/(hsvd_110 - hsvd_0);
      n = 3000.f;
    } else {
      n = 3000.*hsvd_/hsvd_3000;
      ne_ = 0.f;
    }
    n_fd.x1 = n;
    n_fd.y1 = n;
    ne_fd.x1 = ne_;
    ne_fd.y1 = ne_;
    #endif
    n_fd.x1 = dbg_n;
    n_fd.y1 = dbg_n;
    ne_fd.x1 = dbg_ne;
    ne_fd.y1 = dbg_ne;
    hsvd_fd.x1 = dbg_hsvd;
    hsvd_fd.y1 = dbg_hsvd;
    hzu_fd.x1 = dbg_scnt;
    hzu_fd.y1 = dbg_scnt;
    servo_cnt_ref = dbg_scnt;
  }

  // Обработка дискретных входов
  if (dbg_k_vzv_ask) {
    dbg_k_vzv_ask = 0;
    dzz1_v = 1;
    dzz2_v = 1;
    dzz3_v = 1;
    ask_otkr_v = 1;
    ask_zakr_v = 0;
  }
  if (dbg_stop) {
    dbg_stop = 0;
    dzz1_v = 0;
    dzz2_v = 0;
    dzz3_v = 0;
    ask_otkr_v = 0;
    ask_zakr_v = 1;
  }
  static int kg_v_s = -1;
  if (chk_event(&kg_v_s, kg_v, 2)) {
    kg_e = 1;
  }

  // Модель системы
  if (vd_sw != 3) {
    dbg_scnt = servo_cnt_ref;
  }
  hzu_fd.t = 0.03/dt;
  servo_cnt = fade(&hzu_fd, dbg_scnt);
  if (hzu_hsvd_k != 0) {
    hzu_f = scnt_to_grad(servo_cnt);
    double hsvd_in = bound(double(hzu_f - hzu_hsvd_b)/hzu_hsvd_k, 0., 212.);
    hsvd_fd.t = 1.5/dt;
    hsvd_ = fade(&hsvd_fd, hsvd_in);
  } else {
    hsvd_ = 0;
  }
  if (kg_v) {
    ne_fd.t = 0.5/dt;
    double ne_in = 110.*(hsvd_ - hsvd_0)/(hsvd_110 - hsvd_0);
    ne_ = bound(fade(&ne_fd, ne_in),
      -5., 200.);
    n12 = n34 = n56 = n = 3000.f;
  } else {
    #ifdef DBGISVT
    //n12 = n34 = n56 = n = 2000.f;
    n_fd.t = 20./dt;
    double n_in = 3000.*hsvd_/hsvd_3000;
    n12 = n34 = n56 = n = bound(fade(&n_fd, n_in),
      3.5, 10000.);
    #endif
    ne_ = 0.f;
  }

  // Сохранение статуса дискретных входов
  dbg_state = (1L&k_stop_v) | ((1L&ask_otkr_v) << 1) | ((1L&dzz1_v) << 2) |
    ((1L&dzz2_v) << 3) | ((1L&dzz3_v) << 4) | ((3L&vd_sw) << 5) |
    ((1L&kg_v) << 7);
  dbg_n = n;
  dbg_ne = ne_;
  dbg_hsvd = hsvd_;

  // Преобразование аналоговых величин в код АЦП
  for (int i = 0; i < NUM_AI; i++)  {
    float smin = *ai_smin[i];
    float smax = *ai_smax[i];
    float k = *ai_k[i];
    float b = *ai_b[i];

    float adci = 0.f;
    if (((smax - smin) != 0) && (k != 0))
      adci = ((*fadc[i] - b)/k - smin)*double(ADC_20mA - ADC_4mA)/
        (smax - smin) + ADC_4mA;
    else
      adci = ADC_4mA;

    int adcx = 0;
    if ((adci <= INT_MAX) && (adci >= INT_MIN)) adcx = adci;
    adc[*num_adc[i]] = adcx;
  }
}
// Функция для задания тестовых значений в режиме отладки на ПК
// При включеном DBGONCPU
void dbg_proc()
{
  static int but = 0;
  static counter_t to_but;
  #ifdef DEBUG
  static double t_but = 10000.;
  #else //DEBUG
  static double t_but = 500.;
  #endif //DEBUG

  dbg_info();

  if (kbhit()) {
    char c = getch();
    switch (c) {
      case 'w':
      case 'W':
        if (dbg_vd_bol != 1) dputs("ВД больше");
        dbg_vd_bol = 1;
        dbg_nd_bol = 0;
        dbg_nd_men = 0;
        dbg_vd_men = 0;
        but = 1;
        set_to_cnt_t(to_but, t_but);
        break;
      case 's':
      case 'S':
        if (dbg_vd_men != 1) dputs("ВД меньше");
        dbg_vd_men = 1;
        dbg_nd_bol = 0;
        dbg_nd_men = 0;
        dbg_vd_bol = 0;
        but = 1;
        set_to_cnt_t(to_but, t_but);
        break;
      case 'e':
      case 'E':
        if (dbg_nd_bol != 1) dputs("НД больше");
        dbg_nd_bol = 1;
        dbg_nd_men = 0;
        dbg_vd_bol = 0;
        dbg_vd_men = 0;
        but = 1;
        set_to_cnt_t(to_but, t_but);
        break;
      case 'd':
      case 'D':
        if (dbg_nd_men != 1) dputs("НД меньше");
        dbg_nd_men = 1;
        dbg_nd_bol = 0;
        dbg_vd_bol = 0;
        dbg_vd_men = 0;
        but = 1;
        set_to_cnt_t(to_but, t_but);
        break;
      case '0':
        kg_v = 1 - kg_v;
        break;
      case '1':
        k_vzv_ask_v = 1;
        k_vzv_ask_e = 1;
        break;
      case '2':
        k_stop_v = 1 - k_stop_v;
        break;
      case '3':
        vd_sw = 1;
        break;
      case '4':
        vd_sw = 2;
        break;
      case '5':
        vd_sw = 3;
        break;
      case 'b':
      case 'B':
        brk_pnt = 1;
        break;
      case 't':
      case 'T':
        {
        uint t = get_timer0();
        dprintf("Time = %u\n", t);
        }
        break;
      #ifdef RES_PKT_DRV
      case 'r':
      case 'R':
        reinit_packet_drv();
        break;
      #endif //RES_PKT_DRV
      default:
        stop_loop = 1;
        break;
    }
  }
  if (but)
  if (test_to_cnt_t(to_but)) {
    but = 0;
    dbg_nd_bol = 0;
    dbg_nd_men = 0;
    dbg_vd_bol = 0;
    dbg_vd_men = 0;
    dputs("Сброс кнопок ВД, НД");
  }
  static int dbg_vd_bol_s = 0;
  static int dbg_vd_men_s = 0;
  static counter_t to_vd;
  if (vd_sw == 3) {
    if (chk_event(&dbg_vd_bol_s, dbg_vd_bol, 0)) {
      set_to_cnt_t(to_vd, 40000./512.);
    }
    if (chk_event(&dbg_vd_men_s, dbg_vd_men, 0)) {
      set_to_cnt_t(to_vd, 40000./512.);
    }
    if (dbg_vd_bol)
    if (test_to_cnt_t(to_vd)) {
      next_to_cnt_t(to_vd, 40000./512.);
      dbg_scnt++;
      if (dbg_scnt > 512L) dbg_scnt = 512L;
    }
    if (dbg_vd_men)
    if (test_to_cnt_t(to_vd)) {
      next_to_cnt_t(to_vd, 40000./512.);
      dbg_scnt--;
      if (dbg_scnt < 0L) dbg_scnt = 0L;
    }
  }

  static int k_stop_v_p = -1;
  if (k_stop_v_p != k_stop_v)
    if (k_stop_v) dputs("Нажата кн. СТОП"); else dputs("Отжата кн. СТОП");
  k_stop_v_p = k_stop_v;
  static int k_vzv_ask_v_p = -1;
  if (k_vzv_ask_v_p != k_vzv_ask_v)
    if (k_vzv_ask_v) dputs("Нажата кн. взв. АСК");// else dputs("Отжата кн. взв. АСК");
  k_vzv_ask_v_p = k_vzv_ask_v;
}
#else //DBGONCPU
#ifdef DEBUG
// Функция для задания тестовых значений в режиме отладки на ПК
// При включеном DEBUG и выключенном DBGONCPU
void dbg_proc()
{
  static int _init = 1;
  //static float _n_1 = 2970.f;
  //static float _dn_1 = 20.f;
  static counter_t to_show;
  if (_init) {
    _init = 0;
    opt_sw = 1;
    vd_sw = 2;
    nd_sw = 2;
    start_test = 1;
    dputs("Тест запущен...");
    start_turbine = 1;
    //n = 2980.f;
    n_ref = 3000.f;
    hsvd = 30.f;
    reg_rchvt_k = 20;
    reg_rchvt_ki = 0;
    reg_rchvt_kd = 0;
    nelin_k = 0.2;
    nelin_nb = 2998.;
    nelin_ne = 3002.;
    isodr_k = 0.1;
    isodr_t = 50.;
    hzu_min = 0.;
    servo_cnt_min = grad_to_scnt(hzu_min);
    dt_test = 0.02;
    //randomize();
    dprintf("Инициализация...\n");
    set_to_cnt_t(to_show, dtms_test);
  }
  static fade_data fd;
  fd.t = 1./dt_dbg;
  static fade_data fd1;
  fd.t = 1.5/dt_dbg;
  static fade_data fd2;
  fd.t = 2./dt_dbg;
  n = 100.*fade(&fd1, fade(&fd2, fade(&fd, hzu)));
  //hzu_f = 30.f;

  if (!init_corr)
  if (test_to_cnt_t(to_show)) {
    next_to_cnt_t(to_show, dtms_test);
    _setcursortype(_NOCURSOR);
    dprintf("hzu = %5.2f; n_ref = %6.2f n = %6.1f", hzu, n_ref, n);
    clreol();
    dprintf("\r");
    //n = _n_1 - _dn_1*random(1000)/1000.;
  }

  _setcursortype(_NORMALCURSOR);
  #ifdef TEST
  static test_save_f_s = -1;
  if (chk_event(&test_save_f_s, test_save_f, 1)) {
    dputs("Тест завершен");
    dprintf("dt = %g\n", dt_test);
  }
  #endif //TEST

  dbg_info();

  if (kbhit()) {
    char c = getch();
    switch (c) {
      case ' ':
        start_test = 0;
        break;
      #ifndef NOP
      case 13:
        dprintf("n_ref = "); clreol(); scanf("%g", &n_ref);
        break;
      #endif
      default:
        stop_loop = 1;
        break;
    }
  }
}
#else //DEBUG
// Функция для задания тестовых значений в режиме отладки на ПК
// При выключеных DEBUG и DBGONCPU
void dbg_proc()
{
  #ifndef NOEXIT
  // Выполнение выхода из программы и реакции на нажатия на клавиши
  if (kbhit()) {
    int c = getch();
    if ((c == 'q') || (c == 'Q')) stop_loop = 1;
    #ifndef NOP
    else if ((c == 'b') || (c == 'B')) {
      brk_pnt = 1;
    }
    #endif
    #ifdef RES_PKT_DRV
    else if ((c == 'r') || (c == 'R')) {
      reinit_packet_drv();
    }
    #endif //RES_PKT_DRV
    else stop_loop = 1;
  } else {
  }
  #endif //NOEXIT
}
#endif //DEBUG
#endif //DBGONCPU
// Инерционное звено
double fade(fade_data *fd, double x)
{
  double y = (x + fd->x1 - (1 - 2*fd->t)*fd->y1)/(1 + 2*fd->t);
  fd->x1 = x;
  fd->y1 = y;
  return y;
}
// Отправка импульсов управления в серводрайвер
void send_srv_pulses(int pulses)
{
  #ifndef DEBUG
  outpw(isvt_a1_11 + 14, pulses);
  #else
  pulses++;
  #endif //DEBUG
}
// Процедура сброса серводрайвера
void reset_servo_proc()
{
  static counter_t to_rst;
  static counter_t to_on;
  static counter_t to_wait;
  static int flag_1 = 0;

  if (vd_sw != 3) {
    if (reset_servo) {
      if (test_to_cnt_t(to_rst)) {
        unioset(srv_alm_rst, 0);
      }
      if (!flag_1)
      if (test_to_cnt_t(to_on)) {
        unioset(srv_s_on, 1);
        flag_1 = 1;
        set_to_cnt_t(to_wait, 1000.);
      }
      if (flag_1)
      if (test_to_cnt_t(to_wait)) {
        flag_1 = 0;
        reset_servo = 0;
        init_corr = 0;
        init_res_servo_f = 0;
      }
    } else {
      if (srv_alm_v) {
        reset_servo = 1;

        #ifdef CORR_HZU_ON
        // Корректировка hzu
        cnt_alm_srv++;
        if (hsvd != -100.f) {
          if (hsvd > 100.f) {
            servo_cnt_ref1 = servo_cnt_ref = servo_cnt_cur = servo_cnt = 555;
            set_servo();
          } else {
            servo_cnt_ref1 = servo_cnt_ref = servo_cnt_cur = servo_cnt = 0;
            set_servo();
          }
        }
        #endif //CORR_HZU_ON

        unioset(srv_s_on, 0);
        unioset(srv_alm_rst, 1);
        set_to_cnt_t(to_rst, 100.);
        set_to_cnt_t(to_on, 500.);
      }
    }
  } else {
    reset_servo = 0;
    init_corr = 0;
    init_res_servo_f = 0;
    flag_1 = 0;
  }
  if (srv_bad_pos)
  if (test_to_cnt_t(to_res_sbp)) {
    srv_bad_pos = 0L;
  }
}
#ifdef CORR_HZU_ON
// Коррекция hzu при инициализации
void corr_hzu_init()
{
  static counter_t to_init = 0L;
  static int _init = 1;
  double t_init_to = 1000.;

  if (!init_corr) return;

  if (_init) {
    _init = 0;
    init_corr = 1;
    set_to_cnt_t(to_init, t_init_to);
  }
  if (!init_res_servo_f)
  if (test_to_cnt_t(to_init)) {

    lan_en = 1;

    if (hesnd != -100.f) {
      hesnd_ref = hesnd;
    }

    #ifdef ISODR
    rchvt_isodr.fd.x1 = n;
    rchvt_isodr.fd.y1 = n;
    #endif //ISODR

    #ifdef DETECT_KG
    filtr_n.x1 = n;
    filtr_n.y1 = n;
    #endif //DETECT_KG

    if (hsvd != -100.f) {
      if ((hsvd < hsvd_min + 3.f)) {
        if (vd_sw != 3) {
          send_srv_pulses(1024);
          servo_cnt_ref = servo_cnt_min;
          #ifdef DBGONCPU
          init_corr = 0;
          #else //DBGONCPU
          init_res_servo_f = 1;
          #endif //DBGONCPU
        } else {
          init_corr = 0;
        }
      } else if (hsvd > hsvd_max - 3.f) {
        if (vd_sw != 3) {
          send_srv_pulses(-1024);
          servo_cnt_ref = servo_cnt_max;
          #ifdef DBGONCPU
          init_corr = 0;
          #else //DBGONCPU
          init_res_servo_f = 1;
          #endif //DBGONCPU
        } else {
          init_corr = 0;
        }
      } else {
        hzu_f = hzu_hsvd_k*hsvd + hzu_hsvd_b;
        servo_cnt = servo_cnt_cur = grad_to_scnt(hzu_f);
        set_servo();
        servo_cnt_ref = servo_cnt_ref1 = servo_cnt;
        init_corr = 0;
      }
    } else {
      init_corr = 0;
    }
  }

  if (init_res_servo_f) reset_servo_proc();
}
// Коррекция hzu по hsvd
void corr_hzu()
{
  static int d_hzu_norm = 0;
  static int _init = 1;
  static float d_hzu_max = 5.f;
  static double t_d_hzu = 3000.;
  static counter_t to_d_hzu;
  static float hzu_mint = hzu, hzu_maxt = hzu;
  if (_init) {
    _init = 0;
    set_to_cnt_t(to_d_hzu, t_d_hzu);
  }
  if (hzu_mint > hzu) hzu_mint = hzu;
  if (hzu_maxt < hzu) hzu_maxt = hzu;
  
  // Поиск интервала времени t_d_hzu в котором hzu меняется незначительно
  // < d_hzu_max. При этом флаг d_hzu_norm выставляется в 1
  if (test_to_cnt_t(to_d_hzu)) {
    set_to_cnt_t(to_d_hzu, t_d_hzu);
    if (hzu_maxt - hzu_mint < d_hzu_max) d_hzu_norm = 1;
    hzu_mint = hzu;
    hzu_maxt = hzu;
  }

  delta_cnt_corr = 0;

  if (!reset_servo && (hsvd != -100.f) && d_hzu_norm) {
    d_hzu_norm = 0;


    if ((hsvd > hsvd_min + 3.f) && (hsvd < hsvd_max - 3.f)) {
      float hzu_r = hzu_hsvd_k*hsvd + hzu_hsvd_b;

      for (;;) {
        if (fabs(hzu_f - hzu_r) > 10.f) {
          hzu_f = hzu_r;
          servo_cnt = grad_to_scnt(hzu_f);
          set_servo();
          break;
        }

        if (3 != vd_sw)
        if (fabs(hzu - hzu_r) > 10.f) {
          corr_hzu_en = 1;
          delta_cnt_corr = grad_to_scnt(hzu_r - hzu);
          servo_cnt_cur += delta_cnt_corr;
          break;
        }

        break;
      }
    } else {
      if ((hsvd >= hsvd_max - 3.f) && (hzu < hzu_min + 1.f)) {
        corr_hzu_en = 1;
        delta_cnt_corr = grad_to_scnt(hzu_max - hzu);
        servo_cnt_cur += delta_cnt_corr;
      }
      if ((hsvd <= hsvd_min + 3.f) && (hzu > hzu_max - 1.f)) {
        corr_hzu_en = 1;
        delta_cnt_corr = grad_to_scnt(hzu_min - hzu);
        servo_cnt_cur += delta_cnt_corr;
      }
    }
  }
}
#endif //CORR_HZU_ON
#ifdef ISODR
// Изодромное звено
// id - структура данных изодромного звена
// x - входное значение
double isodr(isodr_data *id, double x)
{
  return id->k*x + (1 - id->k)*fade(&id->fd, x);
}
#endif //ISODR
#ifdef NELIN
// Нелинейность по частоте перед РЧВТ
// k - коэффициент передачи в зоне нелинейности
// nb - начало зоны нелинейности, об./мин.
// ne - конец зоны нелинейности, об./мин.
// x - текущая частота, об./мин.
double nelin(double k, double nb, double ne, double x)
{
  double y;
  if (x <= nb) {
    y = x + (1. - k)*(ne - nb)/2.;
  } else if (x >= ne) {
    y = x - (1. - k)*(ne - nb)/2.;
  } else {
    y = k*x + (1. - k)*(ne + nb)/2.;
  }
  return y;
}
#endif //NELIN
// Отслеживание изменения переменных
// Если x изменилась, то возвращется 1, иначе 0
// При первом вызове всегда возвращается 0
// _init - должен быть указателем на статическую переменную инциализированую
//   единицей
// xp - должен быть указателем на статическую переменную, ее инициализация
//   не обязательна
// x - проверяемая переменная
int change_var(byte *_init, int *xp, int x)
{
  int res = 0;
  if (*_init) {
    *_init = 0;
    res = 0;
  } else {
    if (x != *xp) res = 1;
    else res = 0;
  }
  *xp = x;
  return res;
}
int change_var(byte *_init, long *xp, long x)
{
  int res = 0;
  if (*_init) {
    *_init = 0;
    res = 0;
  } else {
    if (x != *xp) res = 1;
    else res = 0;
  }
  *xp = x;
  return res;
}
int change_var(byte *_init, float *xp, float x)
{
  int res = 0;
  if (*_init) {
    *_init = 0;
    res = 0;
  } else {
    if (x != *xp) res = 1;
    else res = 0;
  }
  *xp = x;
  return res;
}
int change_var(byte *_init, double *xp, double x)
{
  int res = 0;
  if (*_init) {
    *_init = 0;
    res = 0;
  } else {
    if (x != *xp) res = 1;
    else res = 0;
  }
  *xp = x;
  return res;
}
// Блокировка всех включенных контуров регулирования с самоблокировкой
void samo_bl_all()
{
  if (rm_on) samo_bl |= 1L;
  if (rto1_on) samo_bl |= 2L;
  if (rto2_on) samo_bl |= 4L;
  if (rtpv_on) samo_bl |= 8L;
}
void all_proc()
{
  // Основной алгоритм
  if (test_to_cnt_t(to_dt)) {
    //next_to_cnt_t(to_dt, dtms);
    set_to_cnt_t(to_dt, dtms);

    long servo_cnt_rchvt = 0L;
    long servo_cnt_eszo = 0L;
    long servo_cnt_kch = 0L;
    float hesnd_rto2 = 0.f;
    float hesnd_rtpv = 0.f;

    // Обработка данных по соответствующим платам
    ai_proc();
    unio_proc();
    isvt_proc();

    #ifdef DBGONCPU
    // Модель турбины в режиме отладки
    dbg_proc_int();
    #endif //DBGONCPU

    // Коррекция hzu при инициализации
    #ifdef CORR_HZU_ON
    corr_hzu_init();

    if (!init_corr) {
    #endif

      // Коррекция hzu по hsvd
      #ifdef CORR_HZU_ON
      corr_hzu();
      #endif

      // Проверка на аварии и взведение АСК
      check_fail();
      vzv_ask();
      if (k_vzv_ask_e && k_vzv_ask_v) {
          ask_vzv_err = 0;
          if (hsvd > (hsvd_min+3.f)) ask_vzv_err |= 1L;
          if (k_stop_v) ask_vzv_err |= 2L;
          if (sz_v) ask_vzv_err |= 4L;
      }

      // Обработка теста
      #ifdef TEST
      test_proc();
      #endif //TEST

      // Выбор наиболее достоверного Pнто
      pnto_select();

      #ifdef DETECT_KG
      // Задатчик = 3000 об./мин. и ВД закрывается на t_cl_svd c
      // после отключения генератора
      //static int _kg_prev = 0;
      static int kg_f = 0;

      // Детектирование сброса нагрузки
      #ifdef BLOCK_FILTR_DETECT_KG_ON_N_ALL_BAD_ISVT
      if (!n_all_bad_isvt) {
        static float n_fp = -100.f;
        filtr_n.t = t_filtr_n/dt;
        float n_f = fade(&filtr_n, n);
        if (kg_v)
        if (-100.f != n_fp) {
          if (n_f > 3030.f)
          if ((n_f - n_fp)/dt > 100.f) {
            kg_f = 1;
          }
        }
        n_fp = n_f;
      }
      #else //BLOCK_FILTR_DETECT_KG_ON_N_ALL_BAD_ISVT
      static float n_fp = -100.f;
      filtr_n.t = t_filtr_n/dt;
      float n_f = fade(&filtr_n, n);
      if (kg_v)
      if (!n_all_bad_isvt)
      if (-100.f != n_fp) {
        if (n_f > 3030.f)
        if ((n_f - n_fp)/dt > 100.f) {
          kg_f = 1;
        }
      }
      n_fp = n_f;
      #endif //BLOCK_FILTR_DETECT_KG_ON_N_ALL_BAD_ISVT

      if (kg_e) {
        n_ref = 3000.f;
        need_writebuf = 1;
        if (!kg_v) {
          kg_f = 1;
        }
      }
      static counter_t to_svd_zakr = 0;
      if (kg_f) {
        kg_f = 0;
        set_to_cnt_t(to_svd_zakr, t_cl_svd*1e3f);
        kg_off_svd_zakr = 1;
        kg_off_esnd_zakr = 1;
        kg_off_svd_zakr_e = 1;
        samo_bl &= 0xFFFF00FFL;
        samo_bl |= 0x100L;
        samo_bl_all();
      }
      //_kg_prev = kg_v;
      if (kg_off_svd_zakr) {
        if (test_to_cnt_t(to_svd_zakr)) kg_off_svd_zakr = 0;
      }
      #else //DETECT_KG
      // Задатчик = 3000 об./мин. и ВД закрывается на t_cl_svd c
      // после отключения генератора
      //static int _kg_prev = 0;
      static counter_t to_svd_zakr;
      if (kg_e) {
        n_ref = 3000.f;
        need_writebuf = 1;
        if (!kg_v) {
          set_to_cnt_t(to_svd_zakr, t_cl_svd*1e3f);
          kg_off_svd_zakr = 1;
          kg_off_esnd_zakr = 1;
          kg_off_svd_zakr_e = 1;
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x100L;
          samo_bl_all();
        }
      }
      //_kg_prev = kg_v;
      if (kg_off_svd_zakr) {
        if (test_to_cnt_t(to_svd_zakr)) kg_off_svd_zakr = 0;
      }
      #endif //DETECT_KG

      // Вкл./выкл. регуляторов
      start = start_turbine && !fail;
      rchvt_on = 0;
      eszo_on = 0;
      rm_on = 0;
      rto1_on = 0;
      rto2_on = 0;
      rtpv_on = 0;
      kch_on = 0;
      if (start) {
        if (kg_v) {
          switch (vd_sw) {
            case 1:
              switch (opt_sw) {
                case 1:
                  rm_on = 1;
                  break;
                case 2:
                case 3:
                  rto1_on = 1;
                  break;
              }
            case 2:
              kch_on = 1;
              eszo_on = 1;
              break;
          }
          switch (nd_sw) {
            case 1:
              switch (opt_sw) {
                case 1:
                  rto2_on = 1;
                  break;
                case 3:
                  rtpv_on = 1;
                  break;
              }
              break;
          }
        } else {
          if (vd_sw != 3)
            rchvt_on = 1;
        }
      }
      if (n_all_bad_isvt) {
        kch_on = 0;
        rm_on = 0;
        rto1_on = 0;
        rchvt_on = 0;
      }
      if (kg_off_svd_zakr) {
          rchvt_on = 0;
        kch_on = 0;
        eszo_on = 0;
        rm_on = 0;
        rto1_on = 0;
      }
      if (kg_off_esnd_zakr) {
        rto2_on = 0;
        rtpv_on = 0;
      }
      if (-100.f == pnto) {
        eszo_on = 0;
      }
      if (-100.f == hsvd) {
          rchvt_on = 0;
        kch_on = 0;
        eszo_on = 0;
        rm_on = 0;
        rto1_on = 0;
      }
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (-100.f == hesnd) {
        rto2_on = 0;
        rtpv_on = 0;
      }
      if (n < 3.f) {
        rchvt_on = 0;
      }

      // Самоблокировка
      if (ne == -100.f)
        if (rm_on) {
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x1000L;
          samo_bl |= 1L;
        }
      if ((pnto == -100.f) || ((t1 == -100.f) && (t_rto1_sel == 1)) ||
        ((t2 == -100.f) && (t_rto1_sel == 2)))
        if (rto1_on) {
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x2000L;
          samo_bl |= 2L;
        }
      if ((pnto == -100.f) || ((t1 == -100.f) && (t_rto2_sel == 1)) ||
        ((t2 == -100.f) && (t_rto2_sel == 2)))
        if (rto2_on) {
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x2000L;
          samo_bl |= 4L;
        }
      if (tpv == -100.f)
        if (rtpv_on) {
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x4000L;
          samo_bl |= 8L;
        }
      if (vd_sw == 2) samo_bl &= (0xFFFFFFFF^3L);
      if (nd_sw == 2) samo_bl &= (0xFFFFFFFF^0xCL);

      // Пуск регулирования по ВД
      start_vd = rchvt_on || eszo_on || kch_on || rm_on || rto1_on;

      // Пуск регулирования по НД
      start_nd = rto2_on || rtpv_on;

      // Установка максимума уставки по частоте в зависимости от режима
      if (7 == mode_testn) {
        n_ref_max = 3400.f;
      } else {
        n_ref_max = 3200.f;
      }
      if (n_ref > n_ref_max) {
        need_writebuf = 1;
        n_ref = n_ref_max;
      }

      #ifdef DBGONCPU
      if (brk_pnt) {
        brk_pnt = 0;
        int i;
        i++;
      }
      #endif //DBGONCPU

      #ifdef NOP
      if (brk_pnt) {
        brk_pnt = 0;
        int i;
        i++;
      }
      #endif

      // РЧВТ
      rchvt_isodr.k = isodr_k;
      rchvt_isodr.fd.t = isodr_t*dt;
      rchvt_reg.k = reg_rchvt_k;
      rchvt_reg.ki = reg_rchvt_ki*dt;
      rchvt_reg.kd = reg_rchvt_kd/dt;
      rchvt_reg.min = servo_cnt_min;
      rchvt_reg.max = servo_cnt_max;
      static int rchvt_on_s = 0;
      if (chk_event(&rchvt_on_s, rchvt_on, 0)) {
        #ifdef ISODR
        rchvt_isodr.fd.x1 = n;
        rchvt_isodr.fd.y1 = n;
        #endif //ISODR

        pid_reg_sync(&rchvt_reg, n_ref - n, servo_cnt_ref);
      }
      if (rchvt_on) {
        #ifdef NELIN
        float n_ = nelin(nelin_k, nelin_nb, nelin_ne, n);
        #else //NELIN

        #ifdef ISODR
        float n_;
        if (fabs(n - n_ref) < 10.f) {
          n_ = isodr(&rchvt_isodr, n);
        } else {
          rchvt_isodr.fd.x1 = n;
          rchvt_isodr.fd.y1 = n;
          n_ = n;
        }
        //float n_ = isodr(&rchvt_isodr, n);
        #else  //ISODR
        float n_ = rchvt_insens(n);
        #endif //ISODR

        #endif //NELIN

        // Синхронизация интергатора при изменении коэффициентов
        // и после коррекции
        static int _init = 1;
        static double k_p = 0., ki_p = 0., kd_p = 0.;
        pid_data *reg = &rchvt_reg;
        if (_init) {
          _init = 0;
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        } else {
          if (corr_hzu_en) {
            reg->prev_out += delta_cnt_corr;
          }
          if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd) ||
            corr_hzu_en) {
            #ifdef CORR_HZU_ERR_CORR
            corr_hzu_en = 0;
            #endif //CORR_HZU_ERR_CORR
            pid_reg_sync(reg);
          }
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        }

        #ifdef RCHVT_DEFENSE
        if ((n_ref - n_ < -n_defense) /*|| (n_ref - n_ > n_ref/2.)*/)
          rchvt_reg.int_val = 0.;
        #endif //RCHVT_DEFENSE

        servo_cnt_rchvt = pid_reg(&rchvt_reg, n_ref - n_);
      }

      // КЧ
      if (kch_on) {
          float kch_val = a_hsvd*kch_insens(n);
        servo_cnt_kch = long(bound(kch_val, (float)LONG_MIN, (float)LONG_MAX));
        if (servo_cnt_kch != 0.f) {
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x200L;
          samo_bl_all();
          reg_st |= 0x2;
        } else {
          reg_st &= 0xFFFFFFFF^0x2;
        }
      } else {
        reg_st &= 0xFFFFFFFF^0x2;
      }

      // ЭСЗО
      static int eszo_off = 0;
      static counter_t to_eszo;
      if (eszo_on) {
        eszo_reg.k = reg_eszo_k;
        eszo_reg.ki = reg_eszo_ki*dt;
        eszo_reg.kd = 0.;
        eszo_reg.min = 0.;
        eszo_reg.max = servo_cnt_max - servo_cnt_min;
        float delta_pnto = 0.f;
        if (pnto > pnto_t) {
          delta_pnto = pnto_sl*(pnto - pnto_t);
          samo_bl &= 0xFFFF00FFL;
          samo_bl |= 0x400L;
          samo_bl_all();
          reg_st |= 0x1;
          eszo_off = 1;
          set_to_cnt_t(to_eszo, 1000.);
        }
        servo_cnt_eszo = pid_reg(&eszo_reg, delta_pnto);
      } else {
        eszo_off = 0;
        eszo_reg.prev_e = 0.;
        eszo_reg.int_val = 0.;
        reg_st &= 0xFFFFFFFF^0x1;
      }
      // Выключении ЭСЗО, если pnto <= pnto_t дольше секунды
      if (eszo_off)
      if (test_to_cnt_t(to_eszo)) {
        eszo_off = 0;
        eszo_reg.prev_e = 0.;
        eszo_reg.int_val = 0.;
        servo_cnt_ref1 = bound(servo_cnt_ref1 - servo_cnt_eszo,
          servo_cnt_min, servo_cnt_max);
        servo_cnt_eszo = 0L;
        reg_st &= 0xFFFFFFFF^0x1;
      }

      // Самоблокировка
      if (samo_bl&0x1) rm_on = 0;
      if (samo_bl&0x2) rto1_on = 0;
      if (samo_bl&0x4) rto2_on = 0;
      if (samo_bl&0x8) rtpv_on = 0;

      // РМ
      rm_ot.slope = ne_ref_sl;
      rm_ot.dt = dt;
      rm_reg.k = reg_rm_k;
      rm_reg.ki = reg_rm_ki*dt;
      rm_reg.kd = 0.;
      rm_reg.min = servo_cnt_min;
      rm_reg.max = servo_cnt_max;
      static int rm_on_s = 0;
      if (chk_event(&rm_on_s, rm_on, 0)) {
        ne_ref_a = rm_ot.cur = ne;
        pid_reg_sync(&rm_reg, 0., servo_cnt_ref);
      }
      if (rm_on) {
        ne_ref_a = ogr_temp(&rm_ot, ne_ref);
        float ne_ = rm_insens(ne);

        // Синхронизация интергатора при изменении коэффициентов
        // и после коррекции
        static int _init = 1;
        static double k_p = 0., ki_p = 0., kd_p = 0.;
        pid_data *reg = &rm_reg;
        if (_init) {
          _init = 0;
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        } else {
          if (corr_hzu_en) {
            reg->prev_out += delta_cnt_corr;
          }
          if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd) ||
            corr_hzu_en) {
            #ifdef CORR_HZU_ERR_CORR
            corr_hzu_en = 0;
            #endif //CORR_HZU_ERR_CORR
            pid_reg_sync(reg);
          }
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        }

        float servo_cnt_rm = pid_reg(&rm_reg, ne_ref_a - ne_);
        servo_cnt_ref1 = rm_bol_men(&rm_bol_men_prev, servo_cnt_rm);
      } else {
        rm_izm = 0L;
        rm_bl = 0L;
        if (ne == -100.f) {
          ne_ref = ne_ref_a = rm_ot.cur = 0.f;
        } else {
          ne_ref = ne_ref_a = rm_ot.cur = ne;
        }
      }

      // Выбор текущего датчика температуры для РТО1
      float t12 = get_t_rto1();
      static char t_rto1_sel_prev = 10;
      if (t12 != -100.f)
      if (t_rto1_sel_prev != 10) {
        if (t_rto1_sel_prev != t_rto1_sel) {
          t_ref1 = t_ref1_a = rto1_ot.cur = t12;
          pid_reg_sync(&rto1_reg, 0., servo_cnt_ref);
        }
      }
      t_rto1_sel_prev = (char)t_rto1_sel;

      // РТО1
      rto1_ot.slope = t_ref1_sl;
      rto1_ot.dt = dt;
      rto1_reg.k = reg_rto1_k;
      rto1_reg.ki = reg_rto1_ki*dt;
      rto1_reg.kd = 0.;
      rto1_reg.min = servo_cnt_min;
      rto1_reg.max = servo_cnt_max;
      static int rto1_on_s = 0;
      if (chk_event(&rto1_on_s, rto1_on, 0)) {
        t_ref1_a = rto1_ot.cur = t12;
        pid_reg_sync(&rm_reg, 0., servo_cnt_ref);
      }
      if (rto1_on) {
        t_ref1_a = ogr_temp(&rto1_ot, t_ref1);
        float t12_ = rto1_insens(t12);

        // Синхронизация интергатора при изменении коэффициентов
        // и после коррекции
        static int _init = 1;
        static double k_p = 0., ki_p = 0., kd_p = 0.;
        pid_data *reg = &rto1_reg;
        if (_init) {
          _init = 0;
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        } else {
          if (corr_hzu_en) {
            reg->prev_out += delta_cnt_corr;
          }
          if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd) ||
            corr_hzu_en) {
            #ifdef CORR_HZU_ERR_CORR
            corr_hzu_en = 0;
            #endif //CORR_HZU_ERR_CORR
            pid_reg_sync(reg);
          }
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        }

        float servo_cnt_rto1 = pid_reg(&rto1_reg, t_ref1_a - t12_);
        servo_cnt_ref1 = rto1_bol_men(&rto1_bol_men_prev, servo_cnt_rto1);
      } else {
        rto1_izm = 0L;
        rto1_bl = 0L;
        if (t12 == -100.f) {
          t_ref1 = t_ref1_a = rto1_ot.cur = 0.f;
        } else {
          t_ref1 = t_ref1_a = rto1_ot.cur = t12;
        }
      }

      // Выбор текущего датчика температуры для РТО2
      float t12_2 = get_t_rto2();
      static char t_rto2_sel_prev = 10;
      if (t12_2 != -100.f)
      if (t_rto2_sel_prev != 10) {
        if (t_rto2_sel_prev != t_rto2_sel) {
          t_ref2 = t_ref2_a = rto2_ot.cur = t12_2;
          #ifdef ND_WITHOUT_SENS
          if (!nd_nosens) {
          #endif //ND_WITHOUT_SENS
          pid_reg_sync(&rto2_reg, 0., hesnd_ref);
          #ifdef ND_WITHOUT_SENS
          }
          #endif //ND_WITHOUT_SENS
        }
      }
      t_rto2_sel_prev = (char)t_rto2_sel;

      // РТО2
      rto2_ot.slope = t_ref2_sl;
      rto2_ot.dt = dt;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens) {
      #endif //ND_WITHOUT_SENS
      rto2_reg.k = reg_rto2_k;
      rto2_reg.ki = reg_rto2_ki*dt;
      rto2_reg.kd = 0.;
      rto2_reg.min = hesnd_min;
      rto2_reg.max = hesnd_max;
      #ifdef ND_WITHOUT_SENS
      }
      #endif //ND_WITHOUT_SENS
      static int rto2_on_s = 0;
      if (chk_event(&rto2_on_s, rto2_on, 0)) {
        t_ref2_a = rto2_ot.cur = t12_2;
        #ifdef ND_WITHOUT_SENS
        if (!nd_nosens) {
        #endif //ND_WITHOUT_SENS
          pid_reg_sync(&rto2_reg, 0., hesnd_ref);
        #ifdef ND_WITHOUT_SENS
        }
        #endif //ND_WITHOUT_SENS
      }
      if (rto2_on) {
        t_ref2_a = ogr_temp(&rto2_ot, t_ref2);
        #ifdef ND_WITHOUT_SENS
        float t12_2_ = 0.f;
        if (nd_nosens) {
          t12_2_ = t12_2;
          float e = t12_2_ - t_ref2_a;
          if (fabs(e) < rto2_ins) nd_mode = 1; else nd_mode = 0;
          hesnd_rto2 = e;
        } else {
          t12_2_ = rto2_insens(t12_2);
  
          // Синхронизация интергатора при изменении коэффициентов
          // и после коррекции
          static int _init = 1;
          static double k_p = 0., ki_p = 0., kd_p = 0.;
          pid_data *reg = &rto2_reg;
          if (_init) {
            _init = 0;
            k_p = reg->k;
            ki_p = reg->ki;
            kd_p = reg->kd;
          } else {
            if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd)) {
              pid_reg_sync(reg);
            }
            k_p = reg->k;
            ki_p = reg->ki;
            kd_p = reg->kd;
          }
  
          hesnd_rto2 = pid_reg(&rto2_reg, t12_2_ - t_ref2_a);
        }
        #else //ND_WITHOUT_SENS
        float t12_2_ = rto2_insens(t12_2);

        // Синхронизация интергатора при изменении коэффициентов
        // и после коррекции
        static int _init = 1;
        static double k_p = 0., ki_p = 0., kd_p = 0.;
        pid_data *reg = &rto2_reg;
        if (_init) {
          _init = 0;
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        } else {
          if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd)) {
            pid_reg_sync(reg);
          }
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        }

        hesnd_rto2 = pid_reg(&rto2_reg, t12_2_ - t_ref2_a);
        #endif //ND_WITHOUT_SENS

        hesnd_ref = rto2_bol_men(&rto2_bol_men_prev, hesnd_rto2);
        #ifdef ND_WITHOUT_SENS
        if (nd_nosens) {
          if (hesnd_ref > 0.f) {
            nd_dir = -1;
          } else if (hesnd_ref < 0.f) {
            nd_dir = 1;
          } else {
            nd_dir = 0;
          }
        }
        #endif //ND_WITHOUT_SENS
      } else {
        rto2_izm = 0L;
        rto2_bl = 0L;
        if (t12_2 == -100.f) {
          t_ref2 = t_ref2_a = rto2_ot.cur = 0.f;
        } else {
          t_ref2 = t_ref2_a = rto2_ot.cur = t12_2;
        }
      }

      // РТПВ
      rtpv_ot.slope = tpv_ref_sl;
      rtpv_ot.dt = dt;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens) {
      #endif //ND_WITHOUT_SENS
      rtpv_reg.k = reg_rtpv_k;
      rtpv_reg.ki = reg_rtpv_ki*dt;
      rtpv_reg.kd = 0.;
      rtpv_reg.min = hesnd_min;
      rtpv_reg.max = hesnd_max;
      #ifdef ND_WITHOUT_SENS
      }
      #endif //ND_WITHOUT_SENS
      static int rtpv_on_s = 0;
      if (chk_event(&rtpv_on_s, rtpv_on, 0)) {
        tpv_ref_a = rtpv_ot.cur = tpv;
        #ifdef ND_WITHOUT_SENS
        if (!nd_nosens) {
        #endif //ND_WITHOUT_SENS
        pid_reg_sync(&rtpv_reg, 0., hesnd_ref);
        #ifdef ND_WITHOUT_SENS
        }
        #endif //ND_WITHOUT_SENS
      }
      if (rtpv_on) {
        tpv_ref_a = ogr_temp(&rtpv_ot, tpv_ref);

        #ifdef ND_WITHOUT_SENS
        float tpv_;
        if (nd_nosens) {
          tpv_ = tpv;
          float e = tpv_ref_a - tpv_;
          if (fabs(e) < rtpv_ins) nd_mode = 1; else nd_mode = 0;
          hesnd_rtpv = e;
        } else {
          tpv_ = rtpv_insens(tpv);

          // Синхронизация интергатора при изменении коэффициентов
          // и после коррекции
          static int _init = 1;
          static double k_p = 0., ki_p = 0., kd_p = 0.;
          pid_data *reg = &rtpv_reg;
          if (_init) {
            _init = 0;
            k_p = reg->k;
            ki_p = reg->ki;
            kd_p = reg->kd;
          } else {
            if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd)) {
              pid_reg_sync(reg);
            }
            k_p = reg->k;
            ki_p = reg->ki;
            kd_p = reg->kd;
          }

          hesnd_rtpv = pid_reg(&rtpv_reg, tpv_ref_a - tpv_);
        }
        #else //ND_WITHOUT_SENS
        float tpv_ = rtpv_insens(tpv);

        // Синхронизация интергатора при изменении коэффициентов
        // и после коррекции
        static int _init = 1;
        static double k_p = 0., ki_p = 0., kd_p = 0.;
        pid_data *reg = &rtpv_reg;
        if (_init) {
          _init = 0;
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        } else {
          if ((k_p != reg->k) || (ki_p != reg->ki) || (kd_p != reg->kd)) {
            pid_reg_sync(reg);
          }
          k_p = reg->k;
          ki_p = reg->ki;
          kd_p = reg->kd;
        }

        hesnd_rtpv = pid_reg(&rtpv_reg, tpv_ref_a - tpv_);
        #endif //ND_WITHOUT_SENS

        hesnd_ref = rtpv_bol_men(&rtpv_bol_men_prev, hesnd_rtpv);
        #ifdef ND_WITHOUT_SENS
        if (nd_nosens) {
          if (hesnd_ref > 0.f) {
            nd_dir = -1;
          } else if (hesnd_ref < 0.f) {
            nd_dir = 1;
          } else {
            nd_dir = 0;
          }
        }
        #endif //ND_WITHOUT_SENS
      } else {
        rtpv_izm = 0L;
        rtpv_bl = 0L;
        if (tpv == -100.f) {
          tpv_ref = tpv_ref_a = rtpv_ot.cur = 0.f;
        } else {
          tpv_ref = tpv_ref_a = rtpv_ot.cur = tpv;
        }
      }

      // Запись заданий регуляторов во флэш
      static int _rm_on_s = 0;
      if (chk_event(&_rm_on_s, rm_on, 0)) need_writebuf = 1;
      static int _rto1_on_s = 0;
      if (chk_event(&_rto1_on_s, rto1_on, 0)) need_writebuf = 1;
      static int _rto2_on_s = 0;
      if (chk_event(&_rto2_on_s, rto2_on, 0)) need_writebuf = 1;
      static int _rtpv_on_s = 0;
      if (chk_event(&_rtpv_on_s, rtpv_on, 0)) need_writebuf = 1;

      // Вычисление общего задания по ВД
      if (!reset_servo)
      if (3 == vd_sw) {
        // Не трогаем hzu в аварийном режиме
        servo_cnt_ref1 = servo_cnt_ref = servo_cnt_cur = servo_cnt;
        start_reg_vd = 0;
      } else {
        static int flag = 1;
        if (start) {
          flag = 1;
          if (kg_off_svd_zakr) {
            // Закрытие СВД при сбросах нагрузки
            //servo_cnt_ref1 = servo_cnt_ref = servo_cnt_min;
            if (kg_off_svd_zakr_e) {
              kg_off_svd_zakr_e = 0;
              send_srv_pulses(1024);
            } else {
              servo_cnt_ref1 = servo_cnt_ref = servo_cnt_min;
            }
            start_reg_vd = 1;
          } else {
            if (start_vd) {
              if (kg_v) {
                // Управление hzu с регуляторов и от кнопок
                //if (kg_e && (2 == vd_sw)) servo_cnt_ref1 = scnt_hh;
                if (eszo_on && (servo_cnt_eszo > 0L)) {
                  servo_cnt_ref = bound(servo_cnt_ref1 - servo_cnt_eszo,
                    servo_cnt_min, servo_cnt_max);
                } else if (kch_on) {
                  servo_cnt_ref = bound(servo_cnt_ref1 - servo_cnt_kch,
                    servo_cnt_min, servo_cnt_max);
                } else {
                  servo_cnt_ref = bound(servo_cnt_ref1, servo_cnt_min,
                    servo_cnt_max);
                }
                start_reg_vd = 1;
              } else {
                // Управление hzu от РЧВТ
                servo_cnt_ref1 = servo_cnt_ref = servo_cnt_rchvt;
                start_reg_vd = 1;
              }
            } else {
              // Не трогаем hzu без регуляторов
              #ifndef NOP
              servo_cnt_ref1 = servo_cnt_ref = servo_cnt_cur = servo_cnt;
              start_reg_vd = 0;
              #endif
            }
          }
        } else {
          // Закрываем hzu от защит
          #ifndef NOP
          if (flag) {
            flag = 0;
            send_srv_pulses(1024);
          } else {
            servo_cnt_ref1 = servo_cnt_ref = servo_cnt_min;
          }
          #else
          servo_cnt_ref1 = servo_cnt_ref = servo_cnt_min;
          #endif
          start_reg_vd = 1;
        }
      }

      // Вычисление общего задания по НД
      if (2 == nd_sw) {
        // Не трогаем hesnd в ручном режиме
        #ifdef ND_WITHOUT_SENS
        if (!nd_nosens) {
        #endif //ND_WITHOUT_SENS
        hesnd_ref = hesnd;
        #ifdef ND_WITHOUT_SENS
        }
        #endif //ND_WITHOUT_SENS
        start_reg_nd = 0;
      } else {
        if (start) {
          if (kg_off_esnd_zakr) {
            // Закрытие ЭСНД при сбросах нагрузки
            #ifdef ND_WITHOUT_SENS
            if (nd_nosens) {
              nd_mode = 0;
              nd_dir = 1;
            } else {
              hesnd_ref = hesnd_min;
            }
            #else //ND_WITHOUT_SENS
            hesnd_ref = hesnd_min;
            #endif //ND_WITHOUT_SENS
            start_reg_nd = 1;
          } else {
            if (kg_v) {
              if (start_nd) {
                // Управление hesnd с регуляторов
                start_reg_nd = 1;
              } else {
                // Не трогаем hesnd под нагрузкой без регуляторов
                #ifdef ND_WITHOUT_SENS
                if (!nd_nosens) {
                #endif //ND_WITHOUT_SENS
                hesnd_ref = hesnd;
                #ifdef ND_WITHOUT_SENS
                }
                #endif //ND_WITHOUT_SENS
                start_reg_nd = 0;
              }
            } else {
              // Управление hesnd с кнопок
              start_reg_nd = 1;
            }
          }
        } else {
          // Закрываем hesnd от защит
          #ifdef ND_WITHOUT_SENS
          if (nd_nosens) {
            nd_mode = 0;
            nd_dir = 1;
          } else {
            hesnd_ref = hesnd_min;
          }
          #else //ND_WITHOUT_SENS
          hesnd_ref = hesnd_min;
          #endif //ND_WITHOUT_SENS
          start_reg_nd = 1;
        }
      }

      k_vd_proc();
      k_nd_proc();

      #ifdef CORR_HZU_ON
      // Блокировка ВД на время перелета реле
      static int vd_sw_s = -1;
      static int vd_flag = 0;
      static counter_t to_vd;
      int vd_sw_n;
      if (vd_sw == 3) vd_sw_n = 1; else vd_sw_n = 0;
      if (chk_event(&vd_sw_s, vd_sw_n, 1)) {
        vd_flag = 1;
        set_to_cnt_t(to_vd, 1000.);
      }
      if (vd_flag)
      if (test_to_cnt_t(to_vd)) {
        vd_flag = 0;
      }

      if (!reset_servo)
      if (!vd_flag)
      #endif
      vd_proc();

      nd_proc();

      // Запись переменных изменяемых контроллером во флэш
      static byte _samo_bl_i = 1;
      static long _samo_bl_s = 0L;
      if (change_var(&_samo_bl_i, &_samo_bl_s, samo_bl)) 
        need_writebuf = 1;
      static int _start_turbine_s = -1;
      if (chk_event(&_start_turbine_s, (int)start_turbine, 2)) 
        need_writebuf = 1;
      static byte _mode_testn_i = 1;
      static long _mode_testn_s = 0L;
      if (change_var(&_mode_testn_i, &_mode_testn_s, mode_testn)) 
        need_writebuf = 1;

      #ifndef CORR_HZU_ERR_CORR
      corr_hzu_en = 0;
      #endif //CORR_HZU_ERR_CORR
    #ifdef CORR_HZU_ON
    }
    #endif //CORR_HZU_ON

    #ifdef UNIO_NEW
    unioout();
    #endif //UNIO_NEW

    #ifdef DBGONCPU
    k_vzv_ask_e = 0;
    k_vzv_ask_v = 0;
    kg_e = 0;
    #endif //DBGONCPU

    #ifdef WDT_NEW
    #ifndef DEBUG
    #ifndef LAN_SINGLE
    #ifdef WDT_ON
    // Сброс сторожевого таймера
    inp(0x20C);
    #endif //WDT_ON
    #endif //LAN_SINGLE
    #endif //DEBUG
    #endif //WDT_NEW
  }
}
// Мое прерывания для канала 0 таймера
#ifdef DEBUG
void ti0()
#else //DEBUG
void interrupt far ti0()
#endif //DEBUG
{
  // Счетчик используемый для измерения времени
  counter++;

  #ifndef AI_INT
  #ifndef ADC_IN_LOOP
  // Чтение АЦП
  if (!bl_proc)
    ai_int();
  #endif //ADC_IN_LOOP
  #endif //AI_INT

  #ifndef NO_INT_IN_INT
  #ifndef DEBUG
  #ifdef DISABLE_TICK
  outp(0x20, 0x20);
  #else //DISABLE_TICK
  if (counter%TICK_CNT == 0) {
    oti0();
  } else {
    // Окончание прерывания
    outp(0x20, 0x20);
  }
  #endif //DISABLE_TICK
  // Разрешение прерываний
  _enable();
  #endif //DEBUG
  #endif //NO_INT_IN_INT

  #ifndef PROC_IN_LOOP
  if (!bl_proc)
    all_proc();
  #endif //PROC_IN_LOOP

  #ifndef WDT_NEW
  #ifdef WDT_INT
  #ifndef DEBUG
  #ifndef LAN_SINGLE
  #ifdef WDT_ON
  // Сброс сторожевого таймера
  inp(0x20C);
  #endif //WDT_ON
  #endif //LAN_SINGLE
  #endif //DEBUG
  #endif //WDT_INT
  #endif //WDT_NEW

  #ifdef SHOW_INT_T0
  if (!bl_proc) {
    uint t_int_c = get_timer0();
    if (t_int_c < t_int_t0) t_int_t0 = t_int_c;
    int_time = tm0toms(TIMER0_CNT - t_int_t0);
    cur_int_time = tm0toms(TIMER0_CNT - t_int_c);
  }
  #endif

  #ifdef NO_INT_IN_INT
  #ifndef DEBUG
  #ifdef DISABLE_TICK
  outp(0x20, 0x20);
  #else //DISABLE_TICK
  if (counter%TICK_CNT == 0) {
    oti0();
  } else {
    // Окончание прерывания
    outp(0x20, 0x20);
  }
  #endif //DISABLE_TICK
  #endif //DEBUG
  #endif //NO_INT_IN_INT
}
// Запись сетевых переменных на флэш
void writebuf(long *abuf)
{
  static int step = 0;
  if (step%2 == 0) {
    long tbuf[CNT_VAR];
    sync(memcpy(tbuf, abuf, 4*CNT_VAR));
    char tfn[512];
    strncpy(tfn, lan_var_fn, 512);
    int L = strlen(lan_var_fn);
    if (L > 511) L = 511;
    tfn[L - 1] = '_';
    if (!access(tfn, 0)) {
      remove(tfn);
    }
    int f = open(lan_var_fn, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
      S_IREAD | S_IWRITE);
    //tmp_lan_var_fn
    if (!f) {
      dputs("Ошибка открытия файла в writebuf");
      return;
    }
    int r = write(f, (void *)tbuf, 4*CNT_VAR);
    if (r == -1) {
      dputs("Ошибка записи в writebuf");
    }
    close(f);
    f = open(tfn, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
      S_IREAD | S_IWRITE);
    close(f);
  } else {
    long tbuf[CNT_VAR];
    sync(memcpy(tbuf, abuf, 4*CNT_VAR));
    char tfn[512];
    strncpy(tfn, tmp_lan_var_fn, 512);
    int L = strlen(tmp_lan_var_fn);
    if (L > 511) L = 511;
    tfn[L - 1] = '_';
    if (!access(tfn, 0)) {
      remove(tfn);
    }
    int f = open(tmp_lan_var_fn, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
      S_IREAD | S_IWRITE);
    //tmp_lan_var_fn
    if (!f) {
      dputs("Ошибка открытия файла в writebuf");
      return;
    }
    int r = write(f, (void *)tbuf, 4*CNT_VAR);
    if (r == -1) {
      dputs("Ошибка записи в writebuf");
    }
    close(f);
    f = open(tfn, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
      S_IREAD | S_IWRITE);
    close(f);
  }
  step++;
}
// Чтение сетевых переменных из флэш
int readbuf(long *abuf)
{
  int finit = 0;
  opt_sw = 1;
  vd_sw = 3;
  nd_sw = 2;
  pnto_sel = 1;
  hsvd_min = 0.;
  hsvd_max = 212.;
  hzu_min = -100.f;
  hzu_max = 100.f;
  servo_cnt_min = grad_to_scnt(hzu_min);
  servo_cnt_max = grad_to_scnt(hzu_max);
  n_bad = 0;
  mode_hzu = 0;
  n_defense = 10.f;
  n_ref = 0.f;
  n12t = 3300.f;
  n34t = 3300.f;
  n56t = 3300.f;
  rchvt_ins = 0.f;
  rm_ins = 0.f;
  rto1_ins = 0.f;
  rto2_ins = 0.f;
  rtpv_ins = 0.f;
  reg_rchvt_k = 1.25f;
  reg_rchvt_ki = 0.05f;
  reg_rchvt_kd = 0.8f;
  n_ns_rchvt_b = 49.6f;
  n_ns_rchvt_e = 50.2f;
  ask_vzv_err = 0L;
  a_hsvd = 2.f;
  pnto_t = 1.2f;
  pnto_sl = 1.f;
  reg_eszo_k = 1.f;
  reg_eszo_ki = 0.f;
  //n_sl_rm = 40.f;
  //n_ns_rm_b = 49.6f;
  //n_ns_rm_e = 50.2f;
  ne_ref_sl = 3.f;
  pt_rm_min = 125.f;
  pt_rm_max = 132.f;
  prs_rm_min = 40.f;
  prs_rm_max = 92.f;
  n_rm_min = 49.f;
  n_rm_max = 50.5f;
  vk_rm_min = 0.89f;
  hsvd_rm_max = 212.f;
  reg_rm_k = 1.f;
  reg_rm_ki = 0.f;
  t_ref1_sl = .5f;
  pt_rto1_min = 125.f;
  pt_rto1_max = 132.f;
  prs_rto1_min = 40.f;
  prs_rto1_max = 92.f;
  n_rto1_min = 49.f;
  n_rto1_max = 50.5f;
  dt1_rto1_max = 50.f;
  dt2_rto1_max = 50.f;
  pnto_rto1_max = 1.1f;
  reg_rto1_k = 1.f;
  reg_rto1_ki = 0.f;
  t_rto1_sel = 2L;
  t_ref2_sl = 0.5f;
  t_rto2_sel = 1L;
  dt1_rto2_max = 50.f;
  dt2_rto2_max = 50.f;
  pnto_rto2_max = 1.1f;
  vk_rto2_min = 0.89f;
  hesnd_rto2_min = 5.f;
  hesnd_rto2_max = 110.f;
  reg_rto2_k = 1.f;
  reg_rto2_ki = 0.f;
  tpv_ref_sl = 1.f;
  vk_rtpv_min = 0.88f;
  hesnd_rtpv_min = 5.f;
  hesnd_rtpv_max = 60.f;
  t_cl_svd = 5.f;
  pim_k = 1.f;
  pim_b = 0.f;
  prm_k = 1.f;
  prm_b = 0.f;
  prs_k = 1.f;
  prs_b = 0.f;
  pt_k = 1.f;
  pt_b = 0.f;
  pnto1_k = 1.f;
  pnto1_b = 0.f;
  pnto2_k = 1.f;
  pnto2_b = 0.f;
  tpv_k = 1.f;
  tpv_b = 0.f;
  t1_k = 1.f;
  t1_b = 0.f;
  t2_k = 1.f;
  t2_b = 0.f;
  tobr_k = 1.f;
  tobr_b = 0.f;
  psv_k = 1.f;
  psv_b = 0.f;
  hesnd_k = 1.f;
  hesnd_b = 0.f;
  hsvd_k = 1.f;
  hsvd_b = 0.f;
  ne_k = 1.f;
  ne_b = 0.f;
  vk_k = 1.f;
  vk_b = 0.f;
  reg_rtpv_k = 1.f;
  reg_rtpv_ki = 0.f;
  hesnd_min = 0.f;
  hesnd_max = 250.f;
  //b_hsvd = 0.f;
  hzu_hsvd_k = 1.f;
  hzu_hsvd_b = 0.f;
  samo_bl = 0L;
  //k_dpnto_rto1 = 1.f;
  //k_dpnto_rto2 = 1.f;
  test_n = 2900.f;
  test_dn = 10.f;
  n12o = 0.f;
  n34o = 0.f;
  n56o = 0.f;
  n12tg = 0.f;
  n34tg = 0.f;
  n56tg = 0.f;
  n_zub = 30L;
  isvt_rele = 0x3F;
  srv_bad_pos = 0;
  scnt_hh = 240.f;
  #ifdef NOP
  reg_hsvd_k = 1.f;
  reg_hsvd_ki = 0.f;
  reg_hsvd_kd = 0.f;
  reg_hsvd2_k = 1.f;
  reg_hsvd2_ki = 0.f;
  reg_hsvd2_kd = 0.f;
  #endif
  reg_rchvt2_k = 1.f;
  reg_rchvt2_ki = 0.f;
  reg_rchvt2_kd = 0.f;
  isodr_k = 0.5f;
  isodr_t = 50.f;
  nelin_k = 0.2f;
  nelin_nb = 2998.f;
  nelin_ne = 3002.f;
  test_var = 0L;
  cnt_res_cpu = 0L;
  restart_flag = 0L;
  count_restart = 0L;
  count_int = 0L;
  start = 0L;
        hsvd_ref = hsvd_min;
        hesnd_ref = hesnd_min;
  testn_st = 2L;
  reg_st = 0L;
  nd_nosens = 0L;
  ind_nx_anomal = 0L;
  time_nx_anomal = 0L;
  nx_anomal_prev = 0.f;
  nx_anomal_cur = 0.5;
  time_n_anomal = 0L;
  n_anomal_prev = 0.f;
  n_anomal_cur = 0.f;
  nd_period = 1000.;
  nd_pulse = 100.;
  suc_lan_counter = 0L;
  t_ai_filtr = 1.f;
  hesnd_ref_delta = 5.f;
  dbg_hsvd = 0.f;
  dbg_ne = 0.f;
  dbg_n = 3.5f;
  dbg_hesnd = 0.f;
  dbg_scnt = 0L;
  cnt_alm_srv = 0L;
  start_test = 0L;
  dt_lan2 = 50.f;
  dt_test = 0.01f;
  nd_nosens = 0L;
  cnt_res_pd = 0L;
  cnt_res_sw = 0L;
  cur_time = 0L;
  n_arr_tst = 0L;
  dbg_state = 0L;
  i_arr_tst = 0L;
  servo_cnt_ref = 0L;
  pim_smin = 0.f;
  pim_smax = 20.f;
  prm_smin = 0.f;
  prm_smax = 20.f;
  ne_smin = 0.f;
  ne_smax = 138.f;
  prs_smin = 0.f;
  prs_smax = 92.f;
  pt_smin = 0.f;
  pt_smax = 140.f;
  pnto1_smin = -1.f;
  pnto1_smax = 1.5f;
  pnto2_smin = -1.f;
  pnto2_smax = 1.5f;
  tpv_smin = 0.f;
  tpv_smax = 100.f;
  t1_smin = 0.f;
  t1_smax = 200.f;
  t2_smin = 0.f;
  t2_smax = 200.f;
  tobr_smin = 0.f;
  tobr_smax = 200.f;
  vk_smin = -1.f;
  vk_smax = 0.f;
  psv_smin = 0.f;
  psv_smax = 10.f;
  hesnd_smin = 0.f;
  hesnd_smax = 250.f;
  hsvd_smin = 0.f;
  hsvd_smax = 250.f;

  char tfn[512];
  strncpy(tfn, lan_var_fn, 512);
  int L = strlen(lan_var_fn);
  if (L > 511) L = 511;
  tfn[L - 1] = '_';
  if (!access(tfn, 0)) {
    if (access(lan_var_fn, 0) == 0) {
      int f = open(lan_var_fn, O_RDONLY | O_BINARY);
      if (!f) {
        dputs("Ошибка открытия файла в readbuf");
        finit = 1;
      } else {
        int r = read(f, (void *)abuf, 4*CNT_VAR);
        if (r == -1) {
          dputs("Ошибка чтения в readbuf");
        }
        close(f);
      }
    } else {
      finit = 1;
    }
  } else {
    finit = 1;
  }
  if (finit) {
    finit = 0;
    strncpy(tfn, tmp_lan_var_fn, 512);
    L = strlen(tmp_lan_var_fn);
    if (L > 511) L = 511;
    tfn[L - 1] = '_';
    if (!access(tfn, 0)) {
      if (access(tmp_lan_var_fn, 0) == 0) {
        int f = open(tmp_lan_var_fn, O_RDONLY | O_BINARY);
        if (!f) {
          dputs("Ошибка открытия файла в readbuf");
          finit = 1;
        } else {
          int r = read(f, (void *)abuf, 4*CNT_VAR);
          if (r == -1) {
            dputs("Ошибка чтения в readbuf");
          }
          close(f);
        }
      } else {
        finit = 1;
      }
    } else {
      finit = 1;
    }
  }
  return !finit;
}
#ifndef IGNORE_INI
// Чтение ini-файла
void readini()
{
  //char *inibuf = NULL;
  char *iniline = NULL;
  FILE *f = NULL, *tf = NULL;
  int i;
  int fe = 0;
  iniline = (char *)calloc(sz_ini_line, 1);
  if (!iniline) {
    dputs("Недостаточно памяти для буфера iniline");
    goto err_exit;
  }
  if (!creat_inidata()) {
    dputs("Недостаточно памяти для массива ini_data");
    goto err_exit;
  }
  //tmp_ini_fn
  tf = fopen(tmp_ini_fn, "wt");
  if (!tf) {
    dputs("Ошибка открытия временного ini-файла");
    goto err_exit;
  }
  if (!access(ini_fn, 0)) {
    fe = 1;
    f = fopen(ini_fn, "rt");
    if (!f) {
      dputs("Ошибка открытия ini-файла");
      goto err_exit;
    }
    for(;;) {
      if (!fgets(iniline, sz_ini_line, f)) break;
      int nvar = proc_iniline(iniline);
      if (nvar == -1) {
        fputs(iniline, tf);
      } else {
        if (proc_inivar(nvar)) {
          fprintf(tf, "%s = %s", ini_data[nvar].varname,
            ini_data[nvar].strval);
        } else {
          fputs(iniline, tf);
        }
        ini_data[nvar].flag = 1;
      }
    }
  }
  for (i = 0; i < iv_cnt; i++) {
    if (proc_inivar(i))
    if (!ini_data[i].flag)
      fprintf(tf, "%s = %s\n", ini_data[i].varname,
        ini_data[i].strval);
  }
  if (fe)
  if (remove(ini_fn) == -1) {
    dprintf("Error in line %d\n", __LINE__);
    goto err_exit;
  }
  if (tf) fclose(tf);
  if (rename(tmp_ini_fn, ini_fn) == -1) {
    dprintf("Error in line %d\n", __LINE__);
    switch (errno) {
      case ENOENT:
        dputs("No such file or directory");
        break;
      case EACCES:
        dputs("Permission denied");
        break;
      default:
        dputs("Other");
        break;
    }
    goto err_exit;
  }
  err_exit:
  free_inidata();
  if (f) fclose(f);
  if (tf) fclose(tf);
  //if (inibuf) free(inibuf);
  if (iniline) free(iniline);
}
// Обработка строки ini-файла
int proc_iniline(char *line)
{
  char *name;
  char *val;
  if (line[0] == ';') return -1;
  char *p = strchr(line, '=');
  if (p) {
    int len = int(p - line + 1);
    name = (char *)calloc(len, 1);
    if (!name) {
      dputs("Нет памяти для name в proc_iniline");
      return -1;
    }
    val = (char *)calloc(strlen(p + 1), 1);
    if (!name) {
      free(name);
      dputs("Нет памяти для val в proc_iniline");
      return -1;
    }
    memcpy(name, line, len - 1);
    memcpy(val, p + 1, strlen(p + 1));
    trimstr(name);
    for (int i = 0; i < iv_cnt; i++) {
      if (strcmp(ini_data[i].varname, name) == 0) {
        free(name);
        free(val);
        ini_data[i].strval = trimstr(val);
        return i;
      }
    }
  }
  free(name);
  free(val);
  return -1;
}
// Очистка строки от начальных и конечных пробельных символов
char far *trimstr(char far *s)
{
  size_t n1, n2;
  char far *ch = " \r\n\t";
  n1 = _fstrspn(s, ch);
  n2 = _fstrcspn(s + n1, ch);
  if (n1) memmove(s, s + n1, n2);
  s[n2] = 0;
  return s;
}
// Обработка переменной ini-файла
int proc_inivar(int nv)
{
  char *&sval = ini_data[nv].strval;
  int cv = 0, def;
  long v;
  uint uv;
  int iv;

  // test_var
  if (cv == nv) {
    if (!sval) {
      ini_alloc(sval, 15);
      ltoa(-1, sval, 10);
      return 1;
    }
    v = strtol(sval, NULL, 0);
    if (v == -2) {
      v = -1;
      ini_alloc(sval, 15);
      ltoa(v, sval, 10);
      return 1;
    }
    if (v != -1) {
      sync(test_var = v);
    }
    return 0;
  } cv++; if (cv >= iv_count) goto err_exit;

  //unio_a1_6
  proc_isaba(unio_a1_6);
  //unio_a1_7
  proc_isaba(unio_a1_7);
  //ai_a1_3
  proc_isaba(ai_a1_3);
  //ai_a1_4
  proc_isaba(ai_a1_4);
  //ai_a1_5
  proc_isaba(ai_a1_5);
  //isvt_a1_8
  proc_isaba(isvt_a1_8);
  //isvt_a1_9
  proc_isaba(isvt_a1_9);
  //isvt_a1_10
  proc_isaba(isvt_a1_10);
  //tbi_a2
  proc_tbi(tbi_a2);
  //tbi_a3
  proc_tbi(tbi_a3);
  //tbi_a4
  proc_tbi(tbi_a4);
  //tbi_a5
  proc_tbi(tbi_a5);
  //tbi_a6
  proc_tbi(tbi_a6);
  //tbi_a7
  proc_tbi(tbi_a7);
  //tbi_a8
  proc_tbi(tbi_a8);
  //tbi_a9
  proc_tbi(tbi_a9);

  //Дискретные входы/выходы
  proc_discr(ndb);
  proc_discr(ndm);
  //proc_discr(srv_puls);
  //proc_discr(srv_sign);
  proc_discr(srv_s_on);
  proc_discr(srv_alm_rst);
  proc_discr(avariya);
  proc_discr(srv_alm);
  proc_discr(srv_alo1);
  proc_discr(srv_alo2);
  proc_discr(srv_alo3);
  proc_discr(dzz1);
  proc_discr(dzz2);
  proc_discr(dzz3);
  proc_discr(kg);
  proc_discr(sz);
  proc_discr(snd_otkr);
  proc_discr(snd_zakr);
  proc_discr(ask_otkr);
  proc_discr(ask_zakr);
  proc_discr(k_vd_bol);
  proc_discr(k_vd_men);
  proc_discr(k_stop);
  proc_discr(rezim_o);
  proc_discr(rezim_p);
  proc_discr(rezim_t);
  proc_discr(shu2_220_on);
  proc_discr(k_nd_bol);
  proc_discr(k_nd_men);
  proc_discr(k_vzv_ask);
  proc_discr(shu1_220_on);
  proc_discr(nd_avt);
  proc_discr(nd_ruch);
  proc_discr(vd_avt);
  proc_discr(vd_ruch);
  proc_discr(vd_avar);
  proc_discr(bp24v_shu2);
  proc_discr(bp24v1);
  proc_discr(bp24v2);
  proc_discr(bp24v3);
  proc_discr(bp5v1);
  proc_discr(bp5v2);
  proc_discr(bp5v3);
  proc_discr(sbros_srv);
  proc_discr(boot_sw);

  // Аналоговые входы
  proc_adc(vk_adc);
  proc_adc(prs_adc);
  proc_adc(prm_adc);
  proc_adc(pim_adc);
  proc_adc(pnto1_adc);
  proc_adc(pnto2_adc);
  proc_adc(psv_adc);
  proc_adc(pt_adc);
  proc_adc(hsvd_adc);
  proc_adc(hesnd_adc);
  proc_adc(tpv_adc);
  proc_adc(t1_adc);
  proc_adc(t2_adc);
  proc_adc(tobr_adc);
  proc_adc(ne_adc);

  err_alloc:
  return 0;
  err_exit:
  dputs("Переполнение в proc_inivar");
  return 0;
}
// Инициализация массива ini_data
int creat_inidata()
{
  int cv = 0;
  ini_data = (inidata *)calloc(iv_count, sizeof(ini_data[0]));
  if (!ini_data) {
    return 0;
  }
  // Базовые адреса плат
  ini_data[cv].varname = "test_var"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "unio_a1_6"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "unio_a1_7"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ai_a1_3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ai_a1_4"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ai_a1_5"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "isvt_a1_8"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "isvt_a1_9"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "isvt_a1_10"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a4"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a5"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a6"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a7"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a8"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tbi_a9"; cv++; if (cv >= iv_count) goto err_exit;
  // Дискретные входа/выхода
  ini_data[cv].varname = "ndb"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ndm"; cv++; if (cv >= iv_count) goto err_exit;
  //ini_data[cv].varname = "srv_puls"; cv++; if (cv >= iv_count) goto err_exit;
  //ini_data[cv].varname = "srv_sign"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_s_on"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_alm_rst"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "avariya"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_alm"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_alo1"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_alo2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "srv_alo3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "dzz1"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "dzz2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "dzz3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "kg"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "sz"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "snd_otkr"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "snd_zakr"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ask_otkr"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ask_zakr"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_vd_bol"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_vd_men"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_stop"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "rezim_o"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "rezim_p"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "rezim_t"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "shu2_220_on"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_nd_bol"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_nd_men"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "k_vzv_ask"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "shu1_220_on"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "nd_avt"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "nd_ruch"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "vd_avt"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "vd_ruch"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "vd_avar"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp24v_shu2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp24v1"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp24v2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp24v3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp5v1"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp5v2"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "bp5v3"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "sbros_srv"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "boot_sw"; cv++; if (cv >= iv_count) goto err_exit;
  // Аналоговые входы
  ini_data[cv].varname = "vk_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "prs_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "prm_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "pim_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "pnto1_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "pnto2_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "psv_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "pt_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "hsvd_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "hesnd_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tpv_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "t1_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "t2_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "tobr_adc"; cv++; if (cv >= iv_count) goto err_exit;
  ini_data[cv].varname = "ne_adc"; cv++; if (cv >= iv_count) goto err_exit;
  iv_cnt = cv;
  return 1;
  err_exit:
  dputs("Переполнение в creat_inidata");
  iv_cnt = cv;
  return 1;
}
// Освобождение массива ini_data
void free_inidata()
{
  for (int i = 0; i < iv_cnt; i++)
    if (ini_data[i].strval) free(ini_data[i].strval);
  if (ini_data) free(ini_data);
}
#endif //IGNORE_INI
// Отладочная версия inp
byte dinp(uint port)
{
  //vport = 0x01;
  #ifndef BLOCK_SHOW_DPORT
  dprintf("inpw: 0x%04X\n", port);
  #endif
  port++;
  return vport;
}
// Отладочная версия outp
void doutp(uint port, byte val)
{
  if ((port&0xF) > 2) vport &= ~val;
  else vport = val;
  #ifndef BLOCK_SHOW_DPORT
  dprintf("outpw: 0x%04X\nval port: 0x%02X\n", port, vport);
  #endif
}
// Функция проверки бита дискретного входа
int uniobit(uint N)
{
  byte mask = 1 << (N&7);
  uint tbi_c = tbi[(N >> 8)&7];
  uint unio_ba = unio[(tbi_c >> 2)&1];
  uint mrx_ba = uniom[tbi_c&3];
  uint oct_n = (N >> 3)&3;
  int res_ret = (N >> 12)&1;
  int set_ret = 1 - res_ret;
  #ifdef DEBUG
  if (mask&dinp(unio_ba + mrx_ba + oct_n)) return set_ret;
  else return res_ret;
  #else
  if (mask&(byte)inp(unio_ba + mrx_ba + oct_n)) return set_ret;
  else return res_ret;
  #endif
}
// Установка дискретных выходов UNIO
void unioout()
{
  #ifndef DEBUG
  uint N = avariya;
  uint tbi_c = tbi[(N >> 8)&7];
  uint unio_ba = unio[(tbi_c >> 2)&1];
  uint mrx_ba = uniom[tbi_c&3];
  uint oct_n = (N >> 3)&3;
  uint port = unio_ba + mrx_ba + oct_n;
  //byte vp;

  outp(port, unio_mask);
  #endif //DEBUG
}
// Функция установки состояния дискретного выхода
void unioset(uint N, int bit)
{
  byte mask = 1 << (N&7);
  #ifndef UNIO_NEW
  uint tbi_c = tbi[(N >> 8)&7];
  uint unio_ba = unio[(tbi_c >> 2)&1];
  uint mrx_ba = uniom[tbi_c&3];
  uint oct_n = (N >> 3)&3;
  uint port = unio_ba + mrx_ba + oct_n;
  #endif
  int inv = (N >> 12)&1;
  if (bit) bit = 1;
  if (inv) bit = 1 - bit;
  #ifdef UNIO_NEW
  if (bit) unio_mask |= mask; else unio_mask &= (0xFF^mask);
  #else
  byte vp;
  #ifdef DEBUG
  vp = dinp(port);
  if (bit) vp = vp|mask; else vp &= vp&(0xFF^mask);
  doutp(port, vp);
  #else
  vp = (byte)inp(port);
  if (bit) vp = vp|mask; else vp = vp&(0xFF^mask);
  outp(port, vp);
  #endif
  #endif
}
// Функция чтения событий дискретных входов
int unioev(uint N)
{
  byte mask = 1 << (N&7);
  uint tbi_c = tbi[(N >> 8)&7];
  uint unio_ba = unio[(tbi_c >> 2)&1];
  uint mrx_ba = uniom[tbi_c&3];
  uint oct_n = (N >> 3)&3;
  uint port = unio_ba + mrx_ba + oct_n + 6;
  #ifdef DEBUG
  return mask&dinp(port);
  #else
  return mask&(byte)inp(port);
  #endif
}
// Функция для сброса событий дискретных входов
void uniorev(uint N)
{
  byte mask = 1 << (N&7);
  uint tbi_c = tbi[(N >> 8)&7];
  uint unio_ba = unio[(tbi_c >> 2)&1];
  uint mrx_ba = uniom[tbi_c&3];
  uint oct_n = (N >> 3)&3;
  uint port = unio_ba + mrx_ba + oct_n + 6;
  #ifdef DEBUG
  doutp(port, mask);
  #else
  outp(port, mask);
  #endif
}
// Фунция для проверки макросов для UNIO
int _unio(uint N)
{
  dprintf("inv: %d\nN=0x%04X\n", ((N>>12)&1), N);
  return ( (1 << (N&0x7))&dinp(
    unio[(tbi[(N&0x700) >> 8] >> 2)&1] + uniom[tbi[(N&0x700) >> 8]&3]
    + ((N&0x18) >> 3) ) ) ? ( 1 - ((N>>12)&1) ) : ( 0 + ((N>>12)&1) );
}
// Фунция для проверки макросов для UNIO - 2
void _unios(uint N)
{
  doutp(unio[(tbi[(N&0x700) >> 8] >> 2)&1] + uniom[tbi[(N&0x700) >> 8]&3]
    + ((N&0x18) >> 3) , (~(1 << (N&0x7)))&dinp(
    unio[(tbi[(N&0x700) >> 8] >> 2)&1] + uniom[tbi[(N&0x700) >> 8]&3]
    + ((N&0x18) >> 3) ) );
}
// Деинициализация UNIO
void unio_deinit()
{
  int i_tbi = 3;
  uint tbi_c = tbi[i_tbi];
  uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
  // Сброс регистров входов/выходов
  outp(tbi_ba, 0);
  outp(tbi_ba + 1, 0);
  outp(tbi_ba + 2, 0);

  #ifdef NOP
  // Деинициализация прерывания по UNIO
  i_tbi = 6;
  tbi_c = tbi[i_tbi];
  tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
  // Запрещение прерывания по нулевому счетчику
  outpw(tbi_ba, 0);
  // Запрещаем прерывания от всех матриц
  outp(unio[(tbi_c >> 2)&1] + 0xA00D, 0);
  #ifdef NOP
  // Маскируем IRQ6
  outp(0x21, inp(0x21)|0x40);
  // Восстанавливаем старый вектор для IRQ6
  sync(setvect(0xE, oui));
  // End of interrupt
  outp(0x20, 0x20);
  #endif

  outp(0x21, inp(0x21)|IRQMask);
  sync(setvect(AIVEC, oui));
  // End of interrupt
  outp(0x20, 0x20);
  #endif
}
// Установка счетчика энкодера
void set_servo()
{
  long servo_cnt_tmp;
  uint tbi_c = tbi[2];
  uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
  servo_cnt_tmp = servo_cnt;
  if (tbi_a4&0x200) servo_cnt_tmp = -servo_cnt;
  outpw(tbi_ba + 1, IRS_LOWORD(servo_cnt_tmp));
  outpw(tbi_ba + 3, IRS_HIWORD(servo_cnt_tmp));
}
// Чтение счетчика энкодера
void get_servo()
{
  long servo_cnt_tmp;
  uint tbi_c = tbi[2];
  uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
  IRS_LOWORD(servo_cnt_tmp) = (unsigned short)inpw(tbi_ba + 1);
  IRS_HIWORD(servo_cnt_tmp) = (unsigned short)inpw(tbi_ba + 3);
  if (tbi_a4&0x200) servo_cnt = -servo_cnt_tmp;
}
// Преобразование количества импульсов серводрайвера в градусы
float scnt_to_grad(long scnt)
{
  return scnt/k_cnt_hzu;
}
// Преобразование градусов в количества импульсов серводрайвера
long grad_to_scnt(float grad)
{
  return grad*k_cnt_hzu;
}
// Чтение значений счетчиков из схемы t00
long get_t00(int i)
{
  int i_tbi = 6;
  uint tbi_c = tbi[i_tbi];
  uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
  long r;
  IRS_LOWORD(r) = (unsigned short)inpw(tbi_ba + (i << 1));
  IRS_HIWORD(r) = (unsigned short)inpw(tbi_ba + (i << 1) + 2);
  return r;
}
#ifdef UNIO_SINGLE
// Инициализация UNIO
void unio_init()
{
  byte unio_suc[4] = {0, 0, 0, 0};
  for (int i_tbi = 0; i_tbi < 4; i_tbi++) {
    uint tbi_c = tbi[i_tbi];
    uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
    byte ch = (byte)inp(tbi_ba + 14);
    int num = inp(tbi_ba + 15);
    switch (i_tbi) {
        case 2:
        // Инициализация матрицы с прошивкой x32
        if ((ch == 'x') && (num == 32)) {
          unio_suc[i_tbi] = 1;
          outp(tbi_ba, (byte)(1 + ((tbi_c >> 8)&1)));
          set_servo();
        } else {
          dprintf("Схема соответствующая плате TBI A%d должна быть x32,\n"
          "а фактический: %c%02d\n", i_tbi + 2, ch, num);
        }
        break;
      default:
        // Инициализация матриц с прошивкой p55
        if ((ch == 'p') && (num == 55)) {
          unio_suc[i_tbi] = 1;
          // Инициализация регистра событий и антидребезга входов
          outp(tbi_ba + 4, (byte)(tbi_c >> 8));
          // Сброс регистров входов/выходов
          outp(tbi_ba, 0);
          outp(tbi_ba + 1, 0);
          outp(tbi_ba + 2, 0);
          // Сброс регистров событий
          outp(tbi_ba + 6, 0xFF);
          outp(tbi_ba + 7, 0xFF);
          outp(tbi_ba + 8, 0xFF);
          if ((i_tbi == 3) || (i_tbi == 7)) {
            // Инициализация выходных плат
            outp(tbi_ba + 3, 0);
          } else {
            // Инициализация входных плат
            outp(tbi_ba + 3, 0xFF);
          }
        } else {
          dprintf("Схема соответствующая плате TBI A%d должна быть p55,\n"
          "а фактический: %c%02d\n", i_tbi + 2, ch, num);
        }
        break;
    }
  }
  // Инициализация режима работы с UNIO
  unio_mode = 2;
  if (unio_suc[0] && unio_suc[1] && unio_suc[2] && unio_suc[3]) {
        unio_mode = 0;
  }
  dprintf("unio_mode = %d\n", unio_mode);
  // Включение серводрайвера
  unioset(srv_s_on, 1);
  // Выполнение инициализации переменных связанных с unio
  unio_proc();
}
#else //UNIO_SINGLE
// Инициализация UNIO
void unio_init()
{
  //unio_mode = 0;
  byte unio_suc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i_tbi = 0; i_tbi < 8; i_tbi++) {
    uint tbi_c = tbi[i_tbi];
    uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
    byte ch = (byte)inp(tbi_ba + 14);
    int num = inp(tbi_ba + 15);
    switch (i_tbi) {
        case 2:
        // Инициализация матрицы с прошивкой x32
        if ((ch == 'x') && (num == 32)) {
          unio_suc[i_tbi] = 1;
          outp(tbi_ba, (byte)(1 + ((tbi_c >> 8)&1)));
          set_servo();
        } else {
          dprintf("Схема соответствующая плате TBI A%d должна быть x32,\n"
          "а фактический: %c%02d\n", i_tbi + 2, ch, num);
        }
        break;
      case 6:
        // Инициализация матрицы с прошивкой t00
        if ((ch == 't') && (num == 0)) {
                // Признак успешной проверки матрицы
          unio_suc[i_tbi] = 1;
          // Инициализация регистров максимального значения
          outpw(tbi_ba + 2, 0);
          outpw(tbi_ba + 4, 0);
          outpw(tbi_ba + 6, 0);
          outpw(tbi_ba + 8, 0);
          // Инициализация контрольного регистра
          outpw(tbi_ba, 0x7000);

          #ifdef NOP
          oui = getvect(AIVEC);
          sync(setvect(AIVEC, ui));
          // End of interrupt
          outp (0x20, 0x20);
          // Размаскировать прерывания PC
          outp(0x21, inp(0x21) & ~IRQMask);

          #ifdef NOP
          // Запомнить старое прерывание
          oui = getvect(0xE);
          // Установка своего прерывания int 0xe для IRQ6
          sync(setvect(0xE, ui));
          // End of interrupt
          outp (0x20, 0x20);
          // Размаскировать IRQ6
          //outp(0x21, inp(0x21) & 0xBF);
          byte mask = (byte)inp(0x21) & 0xBF;
          delay(1);
          outp(0x21, mask);
          #endif
          // Маскимум счета нулевого счетчика 50 мкс
          outpw(tbi_ba + 2, 50);
          // Разрешаем прерываю от матрицы со схемой t00 по IRQ6
          // Что соответствует INT 0xE
          //outp(unio[(tbi_c >> 2)&1] + 0xA00D, (1 << ((tbi_c&3) + 3)) + 6);
          outp(unio[(tbi_c >> 2)&1] + 0xA00D, 0x76);
          // Разрешение прерывания по нулевому счетчику
          outpw(tbi_ba, 0x0001);
          #endif
        } else {
          dprintf("Схема соответствующая плате TBI A%d должна быть t00,\n"
          "а фактический: %c%02d\n", i_tbi + 2, ch, num);
        }

        break;
      default:
        // Инициализация матриц с прошивкой p55
        if ((ch == 'p') && (num == 55)) {
          unio_suc[i_tbi] = 1;
          // Инициализация регистра событий и антидребезга входов
          outp(tbi_ba + 4, (byte)(tbi_c >> 8));
          // Сброс регистров входов/выходов
          outp(tbi_ba, 0);
          outp(tbi_ba + 1, 0);
          outp(tbi_ba + 2, 0);
          // Сброс регистров событий
          outp(tbi_ba + 6, 0xFF);
          outp(tbi_ba + 7, 0xFF);
          outp(tbi_ba + 8, 0xFF);
          if ((i_tbi == 3) || (i_tbi == 7)) {
            // Инициализация выходных плат
            outp(tbi_ba + 3, 0);
          } else {
            // Инициализация входных плат
            outp(tbi_ba + 3, 0xFF);
          }
        } else {
          dprintf("Схема соответствующая плате TBI A%d должна быть p55,\n"
          "а фактический: %c%02d\n", i_tbi + 2, ch, num);
        }
        break;
    }
  }
  // Инициализация режима работы с UNIO
  unio_mode = 2;
  if (unio_suc[0] && unio_suc[1] && unio_suc[2] && unio_suc[3]) {
    unio_mode = 1;
    if (unio_suc[4] && unio_suc[5] && unio_suc[6] && unio_suc[7]) {
      unio_mode = 0;
    }
  }
  dprintf("unio_mode = %d\n", unio_mode);
  // Включение серводрайвера
  unioset(srv_s_on, 1);
  // Выполнение инициализации переменных связанных с unio
  unio_proc();
}
#endif //UNIO_SINGLE
// Обработка данных от UNIO
void unio_proc()
{
  #ifdef DBGONCPU
  // -- Работа в режиме эмуляции дискретных входов
  // Золотники защиты
  if (dzz1_v) zz_st |= 1L; else zz_st &= 0xFFFFFFFF^1L;
  if (dzz2_v) zz_st |= 2L; else zz_st &= 0xFFFFFFFF^2L;
  if (dzz3_v) zz_st |= 4L; else zz_st &= 0xFFFFFFFF^4L;
  // Кнопки и др. сигналы
  if (k_vd_bol_v) but_st |= 1L; else but_st &= 0xFFFFFFFF^1L;
  if (k_vd_men_v) but_st |= 2L; else but_st &= 0xFFFFFFFF^2L;
  if (k_nd_bol_v) but_st |= 4L; else but_st &= 0xFFFFFFFF^4L;
  if (k_nd_men_v) but_st |= 8L; else but_st &= 0xFFFFFFFF^8L;
  if (k_stop_v) but_st |= 0x10L; else but_st &= 0xFFFFFFFF^0x10L;
  if (sbros_srv_v) but_st |= 0x20L; else but_st &= 0xFFFFFFFF^0x20L;
  if (k_vzv_ask_v) but_st |= 0x40L; else but_st &= 0xFFFFFFFF^0x40L;
  if (kg_v) but_st |= 0x80L; else but_st &= 0xFFFFFFFF^0x80L;
  if (sz_v) but_st |= 0x100L; else but_st &= 0xFFFFFFFF^0x100L;
  // Положение АСК и СНД
  if (ask_otkr_v) ask_snd_st |= 1L; else ask_snd_st &= 0xFFFFFFFF^1L;
  if (ask_zakr_v) ask_snd_st |= 2L; else ask_snd_st &= 0xFFFFFFFF^2L;
  if (snd_zakr_v) ask_snd_st |= 4L; else ask_snd_st &= 0xFFFFFFFF^4L;
  if (snd_otkr_v) ask_snd_st |= 8L; else ask_snd_st &= 0xFFFFFFFF^8L;
  // Сигналы тревоги серводрайвера
  if (srv_alm_v) alarm_srv |= 1L; else alarm_srv &= 0xFFFFFFFF^1L;
  if (srv_alo1_v) alarm_srv |= 2L; else alarm_srv &= 0xFFFFFFFF^2L;
  if (srv_alo2_v) alarm_srv |= 4L; else alarm_srv &= 0xFFFFFFFF^4L;
  if (srv_alo3_v) alarm_srv |= 8L; else alarm_srv &= 0xFFFFFFFF^8L;
  hzu_f = scnt_to_grad(servo_cnt);
  #else //DBGONCPU
  #ifndef DEBUG
  // -- Работа дискретных входов в нормальном режиме
  if (unio_mode != 2) {
    // Тумблеры
    if (uniobit(rezim_o)) opt_sw = 1;
    if (uniobit(rezim_p)) opt_sw = 2;
    if (uniobit(rezim_t)) opt_sw = 3;
    #ifdef NOP
    vd_sw = 1;
    #else
    if (uniobit(vd_avt)) vd_sw = 1;
    if (uniobit(vd_ruch)) vd_sw = 2;
    if (uniobit(vd_avar)) vd_sw = 3;
    #endif
    if (uniobit(nd_avt)) nd_sw = 1;
    if (uniobit(nd_ruch)) nd_sw = 2;
    // Золотники защиты
    dzz1_v = uniobit(dzz1);
    dzz2_v = uniobit(dzz2);
    dzz3_v = uniobit(dzz3);
    if (dzz1_v) zz_st |= 1L; else zz_st &= 0xFFFFFFFF^1L;
    if (dzz2_v) zz_st |= 2L; else zz_st &= 0xFFFFFFFF^2L;
    if (dzz3_v) zz_st |= 4L; else zz_st &= 0xFFFFFFFF^4L;
    // Кнопки и др. сигналы
    k_vd_bol_v = uniobit(k_vd_bol);
    k_vd_men_v = uniobit(k_vd_men);
    k_nd_bol_v = uniobit(k_nd_bol);
    k_nd_men_v = uniobit(k_nd_men);
    k_stop_v = uniobit(k_stop);
    sbros_srv_v = uniobit(sbros_srv);
    k_vzv_ask_v = uniobit(k_vzv_ask);
    // Включение КГ в работе
    #ifdef NOP
    kg_v = 1;
    #else
    kg_v = uniobit(kg);
    #endif
    sz_v = uniobit(sz);
    k_stop_e = unioev(k_stop);
    sbros_srv_e = unioev(sbros_srv);
    k_vzv_ask_e = unioev(k_vzv_ask);
    kg_e = unioev(kg);
    sz_e = unioev(sz);
    if (k_vd_bol_v) but_st |= 1L; else but_st &= 0xFFFFFFFF^1L;
    if (k_vd_men_v) but_st |= 2L; else but_st &= 0xFFFFFFFF^2L;
    if (k_nd_bol_v) but_st |= 4L; else but_st &= 0xFFFFFFFF^4L;
    if (k_nd_men_v) but_st |= 8L; else but_st &= 0xFFFFFFFF^8L;
    if (k_stop_v) but_st |= 0x10L; else but_st &= 0xFFFFFFFF^0x10L;
    if (sbros_srv_v) but_st |= 0x20L; else but_st &= 0xFFFFFFFF^0x20L;
    if (k_vzv_ask_v) but_st |= 0x40L; else but_st &= 0xFFFFFFFF^0x40L;
    if (kg_v) but_st |= 0x80L; else but_st &= 0xFFFFFFFF^0x80L;
    if (sz_v) but_st |= 0x100L; else but_st &= 0xFFFFFFFF^0x100L;
    // Положение АСК и СНД
    ask_otkr_v = uniobit(ask_otkr);
    ask_zakr_v = uniobit(ask_zakr);
    snd_otkr_v = uniobit(snd_otkr);
    snd_zakr_v = uniobit(snd_zakr);
    ask_otkr_e = unioev(ask_otkr);
    ask_zakr_e = unioev(ask_zakr);
    snd_otkr_e = unioev(snd_otkr);
    snd_zakr_e = unioev(snd_zakr);
    if (ask_otkr_v) ask_snd_st |= 1L; else ask_snd_st &= 0xFFFFFFFF^1L;
    if (ask_zakr_v) ask_snd_st |= 2L; else ask_snd_st &= 0xFFFFFFFF^2L;
    if (snd_zakr_v) ask_snd_st |= 4L; else ask_snd_st &= 0xFFFFFFFF^4L;
    if (snd_otkr_v) ask_snd_st |= 8L; else ask_snd_st &= 0xFFFFFFFF^8L;
    // Сигналы тревоги серводрайвера
    srv_alm_v = uniobit(srv_alm);
    srv_alo1_v = uniobit(srv_alo1);
    srv_alo2_v = uniobit(srv_alo2);
    srv_alo3_v = uniobit(srv_alo3);
    if (srv_alm_v) alarm_srv |= 1L; else alarm_srv &= 0xFFFFFFFF^1L;
    if (srv_alo1_v) alarm_srv |= 2L; else alarm_srv &= 0xFFFFFFFF^2L;
    if (srv_alo2_v) alarm_srv |= 4L; else alarm_srv &= 0xFFFFFFFF^4L;
    if (srv_alo3_v) alarm_srv |= 8L; else alarm_srv &= 0xFFFFFFFF^8L;
    // Неисправность БП
    if (uniobit(bp24v_shu2)) bp_bad &= 0xFFFFFFFF^1L; else bp_bad |= 1L;
    if (uniobit(bp24v1)) bp_bad &= 0xFFFFFFFF^2L; else bp_bad |= 2L;
    if (uniobit(bp24v2)) bp_bad &= 0xFFFFFFFF^4L; else bp_bad |= 4L;
    if (uniobit(bp24v3)) bp_bad &= 0xFFFFFFFF^8L; else bp_bad |= 8L;
    if (uniobit(bp5v1)) bp_bad &= 0xFFFFFFFF^0x10L; else bp_bad |= 0x10L;
    if (uniobit(bp5v2)) bp_bad &= 0xFFFFFFFF^0x20L; else bp_bad |= 0x20L;
    if (uniobit(bp5v3)) bp_bad &= 0xFFFFFFFF^0x40L; else bp_bad |= 0x40L;
    if (uniobit(shu1_220_on)) bp_bad &= 0xFFFFFFFF^0x80L; else bp_bad |= 0x80L;
                if (uniobit(shu2_220_on)) bp_bad &= 0xFFFFFFFF^0x100L; else bp_bad |= 0x100L;
    // Сброс событий UNIO
    uniorev(ask_otkr);
    uniorev(ask_zakr);
    uniorev(snd_otkr);
    uniorev(snd_zakr);
    uniorev(k_stop);
    uniorev(sbros_srv);
    uniorev(k_vzv_ask);
    uniorev(kg);
    uniorev(sz);
    // Положение сервомотора
    //uint tbi_c = tbi[2];
    //uint tbi_ba = unio[(tbi_c >> 2)&1] + uniom[tbi_c&3];
    if (mode_hzu && !start) {
      servo_cnt_cur = servo_cnt = 0;
      set_servo();
      mode_hzu = 0;
    }
    get_servo();
    hzu_f = scnt_to_grad(servo_cnt);
  }
  #endif //DEBUG
  #endif //DBGONCPU
}
// Инициализация плат АЦП AI
void ai_init()
{
  int ai1_suc = 0;
  int ai2_suc = 0;
  int ai3_suc = 0;
  byte ch;
  int num;
  ch = (byte)inp(ai_a1_3 + 14);
  num = inp(ai_a1_3 + 15);
  if ((ch == 'A') && (num == 117)) {
    ai1_suc = 1;
    // Сброс контрольного регистра
    outpw (ai_a1_3 + 0, 0);
    //outpw (ai_a1_3 + 0, 0x0018);
    // Коэф.усиления
    outpw (ai_a1_3 + 6, 0x5555);
    outpw (ai_a1_3 + 8, 0x5555);
    // MAXC =10-65535 мкс
    outpw (ai_a1_3 + 4, dt_tmr_ai);
    // Дифференциальное включение, канал 0
    outp(ai_a1_3 + 2, 0);
  } else {
    dprintf("Идентификатор модуля AI A1.3 должен быть A117,\n"
    "а фактический: %c%03d\n", ch, num);
  }
  ch = (byte)inp(ai_a1_4 + 14);
  num = inp(ai_a1_4 + 15);
  if ((ch == 'A') && (num == 117)) {
    ai2_suc = 1;
    // Сброс контрольного регистра
    outpw (ai_a1_4 + 0, 0);
    // Коэф.усиления
    outpw (ai_a1_4 + 6, 0x5555);
    outpw (ai_a1_4 + 8, 0x5555);
    // MAXC =10-65535 мкс
    outpw (ai_a1_4 + 4, dt_tmr_ai);
    // Дифференциальное включение, канал 0
    outp(ai_a1_4 + 2, 0);
  } else {
    dprintf("Идентификатор модуля AI A1.4 должен быть A117,\n"
    "а фактический: %c%03d\n", ch, num);
  }
  #ifndef NO_AI3
  ch = (byte)inp(ai_a1_5 + 14);
  num = inp(ai_a1_5 + 15);
  if ((ch == 'A') && (num == 117)) {
    ai3_suc = 1;
    // Сброс контрольного регистра
    outpw (ai_a1_5 + 0, 0);
    // Коэф.усиления
    outpw (ai_a1_5 + 6, 0x5555);
    outpw (ai_a1_5 + 8, 0x5555);
    // MAXC =10-65535 мкс
    outpw (ai_a1_5 + 4, dt_tmr_ai);
    // Дифференциальное включение, канал 0
    outp(ai_a1_5 + 2, 0);
  } else {
    dprintf("Идентификатор модуля AI A1.5 должен быть A117,\n"
    "а фактический: %c%03d\n", ch, num);
  }
  #else
  ai3_suc = 1;
  #endif
  ai_mode = 2;
  if (ai1_suc) {
    ai_mode = 1;
    if (ai2_suc && ai3_suc) ai_mode = 0;
  }
  dprintf("ai_mode = %d\n", ai_mode);
  if (ai_mode == 0) {
        //get_ai_all();
    #ifdef AI_INT
    oaii = getvect(AIVEC);
    sync(setvect(AIVEC, aii));
    // End of interrupt
    outp (0x20, 0x20);
    // Размаскировать прерывания PC
    outp(0x21, inp(0x21) & ~IRQMask);
    //outp(ai_a1_3 + 11, RS_FIFO|END);// Reset FIFO
    // Разрешить запись в FIFO, прерывания, таймер и автосканирование
    //outpw (ai_a1_3 + 0, FIFO|TMR|INT|INC);
    // Разрешить прерывания, таймер
    outpw(ai_a1_3 + 0, TMR|INT);
    #endif
  }
  ai_proc();
}
// Деинициализация плат АЦП AI
void ai_deinit()
{
  if (ai_mode != 2) {
    outpw(ai_a1_3, 0);
    if (ai_mode == 0) {
      outpw(ai_a1_4, 0);
      #ifndef NO_AI3
      outpw(ai_a1_5, 0);
      #endif
    }
    #ifdef AI_INT
    outp(0x21, inp(0x21)|IRQMask);
    sync(setvect(AIVEC, oaii));
    // End of interrupt
    outp(0x20, 0x20);
    #endif
  }
}
// Инициализация плат ИСВТ
void isvt_init()
{
  int i;
  int isvt_suc[3] = {0, 0, 0};

  // Проверка наличия плат ИСВТ
  for (i = 0; i < 3; i++) sensor_select[i] = 0;
  int r = isvt_proc();
  for (i = 0; i < 3; i++) {
    if ((r >> i)&1) {
      isvt_suc[i] = 1;
      dprintf("Плата ИСВТ%d присутствует\n", i + 1);
    } else {
      dprintf("Плата ИСВТ%d отсутствует\n", i + 1);
    }
  }
  uint ba = *isvt_ba[3];
  uint temp_ba = inpw(ba + 25);
  if (ba != temp_ba) {
    dprintf("Плата ИСВТ4 отсутствует\n", i + 1);
  } else {
    dprintf("Плата ИСВТ4 присутствует\n", i + 1);
    double x = 62500./2000. - 1.;
    if (x < 0.) x = 0.;
    byte f_isvt = (byte)(x);
    outpw(isvt_a1_11 + 14, 0u);
    outp(isvt_a1_11 + 24, f_isvt);
  }
  isvt_mode = 2;
  if (isvt_suc[0] && isvt_suc[1] && isvt_suc[2]) isvt_mode = 0;
  dprintf("isvt_mode = %d\n", isvt_mode);
}

#ifdef SENS_SEL_BLOCK_ON_VALVE_TEST
// Перевод номера из sensor_select вида в вид ИСВТ
int isvt_sensor_from_sensor_select(int a_sensor_select_num)
{
  int isvt_sensor = 0;
  switch (a_sensor_select_num) {
    case sensor_first: {
      isvt_sensor = isvt_sensor_first;
    } break;
    case sensor_second: {
      isvt_sensor = isvt_sensor_second;
    } break;
    case sensor_none: {
      isvt_sensor = isvt_sensor_first;
    } break;
  }
  return isvt_sensor;
}
#endif //SENS_SEL_BLOCK_ON_VALVE_TEST

// Чтение всех каналов АЦП
void get_ai_all()
{
  #ifndef DEBUG
  #ifndef DBGONCPU
  for (int i = 0; i < 8; i++) {
    outp(ai_a1_3 + 2, i);
    outp(ai_a1_4 + 2, i);
    delay(1);
    outp(ai_a1_3, ST_RDY);
    outp(ai_a1_4, ST_RDY);
    delay(1);
    adc[i] = (int)(inpw(ai_a1_3 + 2));
    adc[i + 8] = (int)(inpw(ai_a1_4 + 2));
  }
  #endif //DBGONCPU
  #endif //DEBUG
}
#ifndef NO_AI_FILTR
#ifdef FADC_IN_LOOP
// Фильтрация АЦП
void filter_ADC()
{
  for (int i = 0; i < 16; i++) {
    if (adc_flags[i]) {
      adc_flags[i] = 0;
      #ifdef NEW_FADC
      if (adcf[i].y1 == INT_MIN) {
        adcf[i].x1 = adcf[i].y1 = adc[i];
      } else {
        fade(&adcf[i], adc[i]);
      }
      #else //NEW_FADC
      adcf[i] += (adc[i] - adcf[i])/n_ai_filtr;
      #endif //NEW_FADC
    }
  }
}
#endif //FADC_IN_LOOP
#endif //NO_AI_FILTR
#ifndef AI_INT
// Чтение АЦП без прерывания
void ai_int()
{
        static int i = 0, j = 8;
        static int step1 = 0;
        static int step2 = 0;
        if (step1) {
    #ifndef DEBUG
    if ((byte)inp(ai_a1_3)&ST_RDY) {
    #endif //DEBUG
                step1 = 0;
      #ifndef DBGONCPU
      #ifndef DEBUG
        adc[i] = (int)(inpw(ai_a1_3 + 2));
      #endif //DEBUG
      #endif //DBGONCPU
      #ifndef NO_AI_FILTR
      #ifndef FADC_IN_LOOP
      #ifdef NEW_FADC
      if (adcf[i].y1 == INT_MIN) {
        adcf[i].x1 = adcf[i].y1 = adc[i];
      } else {
        fade(&adcf[i], adc[i]);
      }
      #else //NEW_FADC
      adcf[i] += (adc[i] - adcf[i])/n_ai_filtr;
      #endif //NEW_FADC
      #else //FADC_IN_LOOP
      adc_flags[i] = 1;
      #endif //FADC_IN_LOOP
      #endif //NO_AI_FILTR
      i++;
      if (i >= 8) i = 0;
      #ifndef DEBUG
      outp(ai_a1_3 + 2, i);
      #endif //DEBUG
    #ifndef DEBUG
    }
    #endif //DEBUG
  } else {
    #ifndef DEBUG
    outp(ai_a1_3, ST_RDY);              // Старт АЦП
    #endif //DEBUG
        step1 = 1;
  }
        if (step2) {
    #ifndef DEBUG
    if ((byte)inp(ai_a1_4)&ST_RDY) {
    #endif //DEBUG
                step2 = 0;
      #ifndef DBGONCPU
      #ifndef DEBUG
        adc[j] = (int)(inpw(ai_a1_4 + 2));
      #endif //DEBUG
      #endif //DBGONCPU
      #ifndef NO_AI_FILTR
      #ifndef FADC_IN_LOOP
      #ifdef NEW_FADC
      if (adcf[j].y1 == INT_MIN) {
        adcf[j].x1 = adcf[j].y1 = adc[j];
      } else {
        fade(&adcf[j], adc[j]);
      }
      #else //NEW_FADC
      adcf[j] += (adc[j] - adcf[j])/n_ai_filtr;
      #endif //NEW_FADC
      #else //FADC_IN_LOOP
      adc_flags[j] = 1;
      #endif //FADC_IN_LOOP
      #endif //NO_AI_FILTR
      j++;
      if (j >= 16) j = 8;
      #ifndef DEBUG
      outp(ai_a1_4 + 2, j - 8);
      #endif //DEBUG
    #ifndef DEBUG
    }
    #endif //DEBUG
  } else {
    #ifndef DEBUG
    outp(ai_a1_4, ST_RDY);              // Старт АЦП
    #endif //DEBUG
        step2 = 1;
  }
}
#endif
// Прерывание UNIO (t00)
void interrupt ui()
{
  //buf[280]++;
  outp (0x20,0x20);      // End of interrupt
}
#ifdef AI_INT
// Прерывание аналоговых плат
void interrupt aii()
{
        static int i = 0, j = 8;
        static int step1 = 0;
        static int step2 = 0;
        if (step1) {
    step1 = 0;
    adc[i] = inpw(ai_a1_3 + 2);
    i++;
    if (i >= 8) i = 0;
    outp(ai_a1_3 + 2, i);
  } else {
        //inpw(ai_a1_3 + 2);
        step1 = 1;
  }
        if (step2) {
    if ((byte)inp(ai_a1_4)& ST_RDY) {
                step2 = 0;
        adc[j] = inpw(ai_a1_4 + 2);
      j++;
      if (j >= 16) j = 8;
      outp(ai_a1_4 + 2, j - 8);
    }
  } else {
    outp(ai_a1_4, ST_RDY);              // Старт АЦП
        step2 = 1;
  }

  outp (0x20,0x20);      // End of interrupt
}
#endif
// Проверка событий unio
// var - переменная для сохранения предыдущего состояния
// signal - проверяемый сигнал
// front - 0 - событие по переднему фронту, 1 - по заднему, 2 - оба фронта
int chk_event(int *var, int signal, int front)
{
        int res = 0;
        switch (front) {
                case 0:
                        if ((0 == *var) && (1 == signal)) res = 1;
                        break;
                case 1:
                        if ((1 == *var) && (0 == signal)) res = 1;
                        break;
                case 2:
                        if ((0 == *var) && (1 == signal)) res = 1;
                        if ((1 == *var) && (0 == signal)) res = 1;
                        break;
        }
        *var = signal;
        return res;
}
// Обработка данных от плат AI
void ai_proc()
{
  if (ai_mode != 2) {
    for (int i = 0; i < NUM_AI; i++) {
      #ifdef NO_AI_FILTR
      int adci = adc[*num_adc[i]];
      #else //NO_AI_FILTR
      #ifdef NEW_FADC
      float adci = adcf[*num_adc[i]].y1;
      #else //NEW_FADC
      float adci = adcf[*num_adc[i]];
      #endif //NEW_FADC
      #endif //NO_AI_FILTR
      float smin = *ai_smin[i];
      float smax = *ai_smax[i];
      float k = *ai_k[i];
      float b = *ai_b[i];
      float aiv = float(double(adci - ADC_4mA)*(smax - smin)/
        double(ADC_20mA - ADC_4mA) + smin);
      aiv = k*aiv + b;
      #ifndef OFF_BAD_AI
      if ((adci < ADC_min) || (adci > ADC_max)) {
        aiv = -100.f;
      }
      #endif //OFF_BAD_AI

      #ifdef NOP
      if (*num_adc[i] == ne_adc) {
        ne = 10.f;
      } else {
        *ai_val[i] = aiv;
      }
      #else
      #ifdef DEBUG
      #ifndef BLOCK_ADC_ON_DBG
      *ai_val[i] = aiv;
      #endif
      #else
      *ai_val[i] = aiv;
      #endif
      #endif
    }
    #ifdef NOP
    if (hsvd != -100.f) hsvd_2 = hsvd;
    else hsvd_2 = hzu_f*hsvd_hzu_k + hsvd_hzu_b;
    #endif
  }
}
// Обработка управления по ВД
void vd_proc()
{
  #ifdef CORR_HZU_ERR_CORR
  if (corr_hzu_en) {
    corr_hzu_en = 0;
    servo_cnt_ref += delta_cnt_corr;
  }
  #endif //CORR_HZU_ERR_CORR
  if (start_reg_vd) {
    int delta_cnt = (int)bound(servo_cnt_ref - servo_cnt_cur, (long)INT_MIN, (long)INT_MAX);
    servo_cnt_cur = servo_cnt_ref;

    #ifndef INV_SRV
    delta_cnt = -delta_cnt;
    #endif

    send_srv_pulses(delta_cnt);
  }

  hzu = scnt_to_grad(servo_cnt_ref);
}
// Включение соответствующих реле управляющих НД
// dir - направление НД: -1 - открытие; 1 - закрытие; 0 - на месте
void nd_exec(int dir)
{
  #ifdef ND_EN_LIM_SW
  if (dir == 1) {
    if (ask_snd_st&4L) dir = 0;
  } else if (dir == -1) {
    if (ask_snd_st&8L) dir = 0;
  }
  #endif
  if (dir == 1) {
    unioset(ndb, 1);
    unioset(ndm, 0);
  } else if (dir == -1) {
    unioset(ndb, 0);
    unioset(ndm, 1);
  } else {
    unioset(ndb, 0);
    unioset(ndm, 0);
  }
}
#ifdef ND_NEW
#ifdef ND_WITHOUT_SENS
// Обработка управления по НД
void nd_proc()
{
  if (nd_nosens) {
    static int _init_nd_pulse = 1;
    static int _init_esnd_zakr = 1;
    int exe_pulse = 0;
    int exe_esnd_zakr = 0;
    if (start_reg_nd) {
      if (kg_off_esnd_zakr) {
        nd_exec(1);
        exe_esnd_zakr = 1;
        #ifdef ND_EN_LIM_SW
        // Закрываем НД до концевика
        if (ask_snd_st&4L) {
          kg_off_esnd_zakr = 0;
        }
        #endif //ND_EN_LIM_SW
        // Без обработки концевиков закрываем НД на 40 с
        static counter_t to_esnd_zakr;
        if (_init_esnd_zakr) {
          _init_esnd_zakr = 0;
          set_to_cnt_t(to_esnd_zakr, 40000.);
        } else {
          if (test_to_cnt_t(to_esnd_zakr)) {
            _init_esnd_zakr = 1;
            kg_off_esnd_zakr = 0;
          }
        }
      } else if (1 == nd_mode) {
        exe_pulse = 1;
        static counter_t to_pulse;
        static counter_t to_period;
        if (_init_nd_pulse) {
          _init_nd_pulse = 0;
          set_to_cnt_t(to_pulse, nd_pulse*1000.);
          set_to_cnt_t(to_period, nd_period*1000.);
          nd_exec(nd_dir);
        } else {
          if (test_to_cnt_t(to_pulse)) {
            nd_exec(0);
          }
          if (test_to_cnt_t(to_period)) {
            set_to_cnt_t(to_pulse, nd_pulse*1000.);
            set_to_cnt_t(to_period, nd_period*1000.);
            nd_exec(nd_dir);
          }
        }
      } else {
        nd_exec(nd_dir);
      }
    } else {
      nd_exec(0);
    }
    if (!exe_pulse) {
      _init_nd_pulse = 1;
    }
    if (!exe_esnd_zakr) {
      kg_off_esnd_zakr = 0;
      _init_esnd_zakr = 1;
    }
  } else {
    static int _init = 1;
    int exe_pulse = 0;
    int exe_esnd_zakr = 0;
    if (start_reg_nd) {
      if (kg_off_esnd_zakr) {
        nd_exec(1);
        exe_esnd_zakr = 1;
        if (hesnd <= hesnd_min) {
          kg_off_esnd_zakr = 0;
        }
      } else if (fabs(hesnd_ref - hesnd) < hesnd_ref_delta) {
        exe_pulse = 1;
        //static int dir = 0;
        static counter_t to_pulse;
        static counter_t to_period;
        if (_init) {
          _init = 0;
          set_to_cnt_t(to_pulse, nd_pulse*1000.);
          set_to_cnt_t(to_period, nd_period*1000.);
          if (hesnd_ref < hesnd) {
            nd_exec(1);
          } else if (hesnd_ref > hesnd) {
            nd_exec(-1);
          } else {
            nd_exec(0);
          }
        } else {
          if (test_to_cnt_t(to_pulse)) {
            nd_exec(0);
          }
          if (test_to_cnt_t(to_period)) {
            set_to_cnt_t(to_pulse, nd_pulse*1000.);
            set_to_cnt_t(to_period, nd_period*1000.);
            if (hesnd_ref < hesnd) {
              nd_exec(1);
            } else if (hesnd_ref > hesnd) {
              nd_exec(-1);
            } else {
              nd_exec(0);
            }
          }
        }
      } else {
        if (hesnd_ref < hesnd) {
          nd_exec(1);
        }
        if (hesnd_ref > hesnd) {
          nd_exec(-1);
        }
      }
    } else {
      nd_exec(0);
    }
    if (!exe_pulse) {
      _init = 1;
    }
    if (!exe_esnd_zakr) {
      kg_off_esnd_zakr = 0;
    }
  }
}
#else //ND_WITHOUT_SENS
// Обработка управления по НД
void nd_proc()
{
  static int _init = 1;
  int exe_pulse = 0;
  int exe_esnd_zakr = 0;
  if (start_reg_nd) {
    if (kg_off_esnd_zakr) {
      nd_exec(1);
      exe_esnd_zakr = 1;
      if (hesnd <= hesnd_min) {
        kg_off_esnd_zakr = 0;
      }
    } else if (fabs(hesnd_ref - hesnd) < hesnd_ref_delta) {
      exe_pulse = 1;
      //static int dir = 0;
      static counter_t to_pulse;
      static counter_t to_period;
      if (_init) {
        _init = 0;
        set_to_cnt_t(to_pulse, nd_pulse*1000.);
        set_to_cnt_t(to_period, nd_period*1000.);
        if (hesnd_ref < hesnd) {
          nd_exec(1);
        } else if (hesnd_ref > hesnd) {
          nd_exec(-1);
        } else {
          nd_exec(0);
        }
      } else {
        if (test_to_cnt_t(to_pulse)) {
          nd_exec(0);
        }
        if (test_to_cnt_t(to_period)) {
          set_to_cnt_t(to_pulse, nd_pulse*1000.);
          set_to_cnt_t(to_period, nd_period*1000.);
          if (hesnd_ref < hesnd) {
            nd_exec(1);
          } else if (hesnd_ref > hesnd) {
            nd_exec(-1);
          } else {
            nd_exec(0);
          }
        }
      }
    } else {
      if (hesnd_ref < hesnd) {
        nd_exec(1);
      }
      if (hesnd_ref > hesnd) {
        nd_exec(-1);
      }
    }
  } else {
    nd_exec(0);
  }
  if (!exe_pulse) {
    _init = 1;
  }
  if (!exe_esnd_zakr) {
    kg_off_esnd_zakr = 0;
  }
}
#endif //ND_WITHOUT_SENS
#else //ND_NEW
void nd_proc()
{
  int exe_esnd_zakr = 0;
  if (start_reg_nd) {
    if (kg_off_esnd_zakr) {
      nd_exec(1);
      exe_esnd_zakr = 1;
      if (hesnd <= hesnd_min) {
        kg_off_esnd_zakr = 0;
      }
    } else if (fabs(hesnd_ref - hesnd) < hesnd_ref_delta) {
      nd_exec(0);
    } else {
      if (hesnd_ref < hesnd) {
        nd_exec(1);
      }
      if (hesnd_ref > hesnd) {
        nd_exec(-1);
      }
    }
  } else {
    nd_exec(0);
  }
  if (!exe_esnd_zakr) {
    kg_off_esnd_zakr = 0;
  }
}
#endif //ND_NEW
// ПИД-регулятор
double pid_reg(pid_data *pd, double e)
{
  double dif_int = pd->prev_e;
  double res = 0;
  if (pd->block_int == 1) if (pd->prev_e > 0) dif_int = 0;
  if (pd->block_int == -1) if (pd->prev_e < 0) dif_int = 0;
  if (pd->block_int_ext == 1) if (pd->prev_e > 0) dif_int = 0;
  if (pd->block_int_ext == -1) if (pd->prev_e < 0) dif_int = 0;
  pd->int_val += dif_int;
  double d_pid = k_d_pid*(pd->max - pd->min);
  if ((pd->k != 0.) && (pd->ki != 0.)) {
    double imin = (pd->min - d_pid)/pd->k/pd->ki;
    double imax = (pd->max + d_pid)/pd->k/pd->ki;
    pd->int_val = bound(pd->int_val, imin, imax);
  }
  double Ires = pd->k*pd->ki*pd->int_val;
  double PDres = pd->k*(e + pd->kd*(e - pd->prev_e));
  res = Ires + PDres;
  if (res > pd->max + d_pid) {
    if (PDres > pd->max || (pd->k == 0.) || (pd->ki == 0.)) {
      pd->int_val = 0.;
      res = PDres;
    } else {
      pd->int_val = (pd->max - PDres)/pd->k/pd->ki;
      res = pd->max;
    }
  }
  if (PDres < -d_pid) {
    if (pd->int_val < 0.) pd->int_val = 0.;
  }
  if (res > pd->max) {
    res = pd->max;
    pd->block_int = 1;
  } else if (res < pd->min) {
    res = pd->min;
    pd->block_int = -1;
  } else {
    pd->block_int = 0;
  }
  res = bound(res, pd->min, pd->max);
  pd->pp_e = pd->prev_e;
  pd->prev_e = e;
  pd->prev_out = res;
  return res;
}
// Синхронизация ПИД-регулятора на текущее значение регулируемого параметра
// при выключенном регуляторе
// pd - Параметры ПИД-регулятора,
// e - ошибка на входе регулятора
// rp - значение выдаваемое на исполнительный механизм на который
// осуществляется синхронизация
void pid_reg_sync(pid_data *pd, double e, double rp)
{
  if ((pd->k == 0.) || (pd->ki == 0.)) {
    pd->int_val = 0.;
  } else {
    double Pres = pd->k*pd->prev_e;
    double d_pid = k_d_pid*(pd->max - pd->min);
    if (fabs(Pres) < d_pid) {
      pd->int_val = (rp - pd->k*(e/* + pd->kd*(e - pd->prev_e)*/))
        /(pd->k*pd->ki);
    } else {
      pd->int_val = 0.;
    }
  }
  pd->pp_e = e;
  pd->prev_e = e;
  pd->prev_out = rp;
}
// Синхронизация ПИД-регулятора на текущее значение регулируемого параметра
// при включенном регуляторе
// pd - Параметры ПИД-регулятора,
void pid_reg_sync(pid_data *pd)
{
  if ((pd->k == 0.) || (pd->ki == 0.)) {
    pd->int_val = 0.;
  } else {
    double PDres = pd->k*(pd->prev_e + pd->kd*(pd->prev_e - pd->pp_e));
    double d_pid = k_d_pid*(pd->max - pd->min);
    if ((PDres > -d_pid) && (PDres < pd->max)) {
      pd->int_val = (pd->prev_out - pd->k*(pd->prev_e +
        pd->kd*(pd->prev_e - pd->pp_e)))
        /(pd->k*pd->ki);
    } else {
      pd->int_val = 0.;
    }
  }
}
// Зона нечувствительности для КЧ
float kch_insens(float v)
{
  //float pl = 3000.f;
  float pl = 0.f;
  if (v < 60.*n_ns_rchvt_b) {
    return v - 60.*n_ns_rchvt_b + pl;
  } if (v > 60.*n_ns_rchvt_e) {
    return v - 60.*n_ns_rchvt_e + pl;
  } else {
    return pl;
  }
}
// Зона нечувствительности для РЧВТ
float rchvt_insens(float n_r)
{
  float pl = n_ref;
  if (fabs(pl - n_r) < rchvt_ins) return pl;
  else return n_r;
}
// Зона нечувствительности для РМ
float rm_insens(float ne_r)
{
  float pl = ne_ref_a;
  if (fabs(pl - ne_r) < rm_ins) return pl;
  else return ne_r;
}
// Зона нечувствительности для РТО1
float rto1_insens(float t_r)
{
  float pl = t_ref1_a;
  if (fabs(pl-t_r) < rto1_ins) return pl;
  else return t_r;
}
// Зона нечувствительности для РТО2
float rto2_insens(float t_r)
{
  float pl = t_ref2_a;
  if (fabs(pl - t_r) < rto2_ins) return pl;
  else return t_r;
}
// Зона нечувствительности для РТПВ
float rtpv_insens(float t_r)
{
  float pl = tpv_ref_a;
  if (fabs(pl - t_r) < rtpv_ins) return pl;
  else return t_r;
}
// Ограничитель темпа
float ogr_temp(ogr_temp_data *otd, float v)
{
  if ((otd->slope != otd->slope_prev) || (otd->dt != otd->dt_prev) ||
    otd->recalc) {
    otd->dl = otd->dt*otd->slope/60.f;
    otd->recalc = 0;
  }
  float sub = v - otd->cur;
  if (sub > otd->dl) otd->cur += otd->dl;
  else if (sub < -otd->dl) otd->cur -= otd->dl;
  else otd->cur = v;
  otd->slope_prev = otd->slope;
  otd->dt_prev = otd->dt;
  return otd->cur;
}
#ifdef RES_BL_BAD_SENS
// Ограничитель по больше/меньше РМ
float rm_bol_men(float *p, float v)
{
  //int rm_izm_ = 0;
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0) {
      long rm_bl_temp = rm_bl;
      rm_bl_temp = 0L;
      if (pt != -100.f) if (pt < pt_rm_min) rm_bl_temp |= 1L;
      if (prs != -100.f) if (prs > prs_rm_max) rm_bl_temp |= 2L;
      if (!n_all_bad_isvt) if (n > n_rm_max*60.f) rm_bl_temp |= 4L;
      if (vk != -100.f) if (vk < vk_rm_min) rm_bl_temp |= 8L;
      if (hsvd != -100.f) if (hsvd > hsvd_rm_max) rm_bl_temp |= 0x10L;
      rm_bl = rm_bl_temp;
      if (rm_bl&0xFF) {
        rm_izm = 0;
        rm_reg.block_int_ext = 1;
        return *p;
      } else {
        rm_reg.block_int_ext = 0;
      }
      rm_izm = 1;
    } else if (d < 0) {
      long rm_bl_temp = rm_bl;
      rm_bl_temp = 0L;
      if (pt != -100.f) if (pt > pt_rm_max) rm_bl_temp |= 0x100L;
      if (prs != -100.f) if (prs < prs_rm_min) rm_bl_temp |= 0x200L;
      if (!n_all_bad_isvt) if (n < n_rm_min*60.f) rm_bl_temp |= 0x400L;
      rm_bl = rm_bl_temp;
      if (rm_bl&0xFF00) {
        rm_izm = 0;
        rm_reg.block_int_ext = -1;
        return *p;
      } else {
        rm_reg.block_int_ext = 0;
      }
      rm_izm = -1;
    } else {
      rm_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТО1
float rto1_bol_men(float *p, float v)
{
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0.f) {
      long rto1_bl_temp = rto1_bl;
      rto1_bl_temp = 0L;
      if (pt != -100.f) if (pt < pt_rto1_min) rto1_bl_temp |= 1L;
      if (prs != -100.f) if (prs > prs_rto1_max) rto1_bl_temp |= 2L;
      if (!n_all_bad_isvt) if (n > n_rto1_max*60.f) rto1_bl_temp |= 4L;
      if ((t1 != -100.f) && (tobr != -100.f)) if (t1 - tobr > dt1_rto1_max) rto1_bl_temp |= 8L;
      if ((t2 != -100.f) && (t1 != -100.f)) if (t2 - t1 > dt2_rto1_max) rto1_bl_temp |= 0x10L;
      if (pnto != -100.f) if (pnto > pnto_rto1_max) rto1_bl_temp |= 0x20L;
      rto1_bl = rto1_bl_temp;
      if (rto1_bl&0xFF) {
        rto1_izm = 0;
        rto1_reg.block_int_ext = 1;
        return *p;
      } else {
        rto1_reg.block_int_ext = 0;
      }
      rto1_izm = 1;
    } else if (d < 0.f) {
      long rto1_bl_temp = rto1_bl;
      rto1_bl_temp = 0L;
      if (pt != -100.f) if (pt > pt_rto1_max) rto1_bl_temp |= 0x100L;
      if (prs != -100.f) if (prs < prs_rto1_min) rto1_bl_temp |= 0x200L;
      if (!n_all_bad_isvt) if (n < n_rto1_min*60.f) rto1_bl_temp |= 0x400L;
      rto1_bl = rto1_bl_temp;
      if (rto1_bl&0xFF00) {
        rto1_izm = 0;
        rto1_reg.block_int_ext = -1;
        return *p;
      } else {
        rto1_reg.block_int_ext = 0;
      }
      rto1_izm = -1;
    } else {
      rto1_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТО2
float rto2_bol_men(float *p, float v)
{
  #ifdef ND_WITHOUT_SENS
  if (nd_nosens)
    *p = 0;
  #endif //ND_WITHOUT_SENS
  if (*p != 1e6f) {
    float d = v - *p;
    if (d < 0.f) {
      long rto2_bl_temp = rto2_bl;
      rto2_bl_temp = 0L;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd < hesnd_rto2_min) rto2_bl_temp |= 1L;
      if ((t1 != -100.f) && (tobr != -100.f))
        if (t1 - tobr > dt1_rto2_max) rto2_bl_temp |= 2L;
      if ((t2 != -100.f) && (t1 != -100.f))
        if (t2 - t1 > dt2_rto2_max) rto2_bl_temp |= 4L;
      if (pnto != -100.f) if (pnto > pnto_rto2_max) rto2_bl_temp |= 8L;
      rto2_bl = rto2_bl_temp;
      if (rto2_bl&0xFF) {
        rto2_izm = 0;
        rto2_reg.block_int_ext = -1;
        return *p;
      } else {
        rto2_reg.block_int_ext = 0;
      }
      rto2_izm = 1;
    } else if (d > 0.f) {
      long rto2_bl_temp = rto2_bl;
      rto2_bl_temp = 0L;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd > hesnd_rto2_max) rto2_bl_temp |= 0x100L;
      if (vk != -100.f) if (vk < vk_rto2_min) rto2_bl_temp |= 0x200L;
      rto2_bl = rto2_bl_temp;
      if (rto2_bl&0xFF00) {
        rto2_izm = 0;
        rto2_reg.block_int_ext = 1;
        return *p;
      } else {
        rto2_reg.block_int_ext = 0;
      }
      rto2_izm = -1;
    } else {
      rto2_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТПВ
float rtpv_bol_men(float *p, float v)
{
  #ifdef ND_WITHOUT_SENS
  if (nd_nosens)
    *p = 0;
  #endif //ND_WITHOUT_SENS
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0.f) {
      long rtpv_bl_temp = rtpv_bl;
      rtpv_bl_temp = 0L;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd > hesnd_rtpv_max) rtpv_bl_temp |= 1L;
      if (vk != -100.f)
      if (vk < vk_rtpv_min) rtpv_bl_temp |= 2L;
      rtpv_bl = rtpv_bl_temp;
      if (rtpv_bl&0xFF) {
        rtpv_izm = 0;
        rtpv_reg.block_int_ext = 1;
        return *p;
      } else rtpv_reg.block_int_ext = 0;
      rtpv_izm = 1;
    } else if (d < 0.f) {
      long rtpv_bl_temp = rtpv_bl;
      rtpv_bl_temp = 0L;
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd < hesnd_rtpv_min) rtpv_bl_temp |= 0x100L;
      rtpv_bl = rtpv_bl_temp;
      if (rtpv_bl&0xFF00) {
        rtpv_izm = 0;
        rtpv_reg.block_int_ext = -1;
        return *p;
      } else {
        rtpv_reg.block_int_ext = 0;
      }
      rtpv_izm = -1;
    } else {
      rtpv_izm = 0;
    }
  }
  *p = v;
  return v;
}
#else //RES_BL_BAD_SENS
// Ограничитель по больше/меньше РМ
float rm_bol_men(float *p, float v)
{
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0) {
      rm_bl &= (0xFFFFFFFF^0xFF00L);
      if (pt != -100.f)
      if (pt < pt_rm_min) rm_bl |= 1L; else rm_bl &= (0xFFFFFFFF^1L);
      if (prs != -100.f)
      if (prs > prs_rm_max) rm_bl |= 2L; else rm_bl &= (0xFFFFFFFF^2L);
      if (!n_all_bad_isvt)
      if (n > n_rm_max*60.f) rm_bl |= 4L; else rm_bl &= (0xFFFFFFFF^4L);
      if (vk != -100.f)
      if (vk < vk_rm_min) rm_bl |= 8L; else rm_bl &= (0xFFFFFFFF^8L);
      if (hsvd != -100.f)
      if (hsvd > hsvd_rm_max) rm_bl |= 0x10L; else rm_bl &= (0xFFFFFFFF^0x10L);
      if (rm_bl&0xFF) {
        rm_izm = 0;
        rm_reg.block_int_ext = 1;
        return *p;
      } else {
        rm_reg.block_int_ext = 0;
      }
      rm_izm = 1;
    } else if (d < 0) {
      rm_bl &= (0xFFFFFFFF^0xFFL);
      if (pt != -100.f)
      if (pt > pt_rm_max) rm_bl |= 0x100L; else rm_bl &= (0xFFFFFFFF^0x100L);
      if (prs != -100.f)
      if (prs < prs_rm_min) rm_bl |= 0x200L; else rm_bl &= (0xFFFFFFFF^0x200L);
      if (!n_all_bad_isvt)
      if (n < n_rm_min*60.f) rm_bl |= 0x400L; else rm_bl &= (0xFFFFFFFF^0x400L);
      if (rm_bl&0xFF00) {
        rm_izm = 0;
        rm_reg.block_int_ext = -1;
        return *p;
      } else {
        rm_reg.block_int_ext = 0;
      }
      rm_izm = -1;
    } else {
      rm_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТО1
float rto1_bol_men(float *p, float v)
{
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0.f) {
      if (pt != -100.f)
      if (pt < pt_rto1_min) rto1_bl |= 1L; else rto1_bl &= ~1L;
      if (prs != -100.f)
      if (prs > prs_rto1_max) rto1_bl |= 2L; else rto1_bl &= ~2L;
      if (!n_all_bad_isvt)
      if (n > n_rto1_max*60.f) rto1_bl |= 4L; else rto1_bl &= ~4L;
      if ((t1 != -100.f) && (tobr != -100.f))
      if (t1 - tobr > dt1_rto1_max) rto1_bl |= 8L; else rto1_bl &= ~8L;
      if ((t2 != -100.f) && (t1 != -100.f))
      if (t2 - t1 > dt2_rto1_max) rto1_bl |= 0x10L; else rto1_bl &= ~0x10L;
      if (pnto != -100.f)
      if (pnto > pnto_rto1_max) rto1_bl |= 0x20L; else rto1_bl &= ~0x20L;
      if (rto1_bl&0xFF) {
        rto1_izm = 0;
        rto1_reg.block_int_ext = 1;
        return *p;
      } else {
        rto1_reg.block_int_ext = 0;
      }
      rto1_izm = 1;
    } else if (d < 0.f) {
      if (pt != -100.f)
      if (pt > pt_rto1_max) rto1_bl |= 0x100L; else rto1_bl &= ~0x100L;
      if (prs != -100.f)
      if (prs < prs_rto1_min) rto1_bl |= 0x200L; else rto1_bl &= ~0x200L;
      if (!n_all_bad_isvt)
      if (n < n_rto1_min*60.f) rto1_bl |= 0x400L; else rto1_bl &= ~0x400L;
      if (rto1_bl&0xFF00) {
        rto1_izm = 0;
        rto1_reg.block_int_ext = -1;
        return *p;
      } else {
        rto1_reg.block_int_ext = 0;
      }
      rto1_izm = -1;
    } else {
      rto1_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТО2
float rto2_bol_men(float *p, float v)
{
  #ifdef ND_WITHOUT_SENS
  if (nd_nosens)
    *p = 0;
  #endif //ND_WITHOUT_SENS
  if (*p != 1e6f) {
    float d = v - *p;
    if (d < 0.f) {
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd < hesnd_rto2_min) rto2_bl |= 1L; else rto2_bl &= ~1L;
      if ((t1 != -100.f) && (tobr != -100.f))
      if (t1 - tobr > dt1_rto2_max) rto2_bl |= 2L; else rto2_bl &= ~2L;
      if ((t2 != -100.f) && (t1 != -100.f))
      if (t2 - t1 > dt2_rto2_max) rto2_bl |= 4L; else rto2_bl &= ~4L;
      if (pnto != -100.f)
      if (pnto > pnto_rto2_max) rto2_bl |= 8L; else rto2_bl &= ~8L;
      if (rto2_bl&0xFF) {
        rto2_izm = 0;
        rto2_reg.block_int_ext = -1;
        return *p;
      } else {
        rto2_reg.block_int_ext = 0;
      }
      rto2_izm = 1;
    } else if (d > 0.f) {
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f) if (hesnd > hesnd_rto2_max) rto2_bl |= 0x100L; else rto2_bl &= ~0x100L;
      if (vk != -100.f)
      if (vk < vk_rto2_min) rto2_bl |= 0x200L; else rto2_bl &= ~0x200L;
      if (rto2_bl&0xFF00) {
        rto2_izm = 0;
        rto2_reg.block_int_ext = 1;
        return *p;
      } else {
        rto2_reg.block_int_ext = 0;
      }
      rto2_izm = -1;
    } else {
      rto2_izm = 0;
    }
  }
  *p = v;
  return v;
}
// Ограничитель по больше/меньше РТПВ
float rtpv_bol_men(float *p, float v)
{
  #ifdef ND_WITHOUT_SENS
  if (nd_nosens)
    *p = 0;
  #endif //ND_WITHOUT_SENS
  if (*p != 1e6f) {
    float d = v - *p;
    if (d > 0.f) {
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f)
      if (hesnd > hesnd_rtpv_max) rtpv_bl |= 1L; else rtpv_bl &= ~1L;
      if (vk != -100.f)
      if (vk < vk_rtpv_min) rtpv_bl |= 2L; else rtpv_bl &= ~2L;
      if (rtpv_bl&0xFF) {
        rtpv_izm = 0;
        rtpv_reg.block_int_ext = 1;
        return *p;
      } else rtpv_reg.block_int_ext = 0;
      rtpv_izm = 1;
    } else if (d < 0.f) {
      #ifdef ND_WITHOUT_SENS
      if (!nd_nosens)
      #endif //ND_WITHOUT_SENS
      if (hesnd != -100.f)
      if (hesnd < hesnd_rtpv_min) rtpv_bl |= 0x100L; else rtpv_bl &= ~0x100L;
      if (rtpv_bl&0xFF00) {
        rtpv_izm = 0;
        rtpv_reg.block_int_ext = -1;
        return *p;
      } else {
        rtpv_reg.block_int_ext = 0;
      }
      rtpv_izm = -1;
    } else {
      rtpv_izm = 0;
    }
  }
  *p = v;
  return v;
}
#endif //RES_BL_BAD_SENS
// Экстренный останов турбины
void stop_turbine()
{
  #ifdef DBGONCPU
  dbg_stop = 1;
  #endif //DBGONCPU
  #ifndef DEBUG
  for (int i = 0; i < 3; i++) {
    uint ba = *isvt_ba[i];
    sync_isvt(isvt_deinit_r(ba));
    sync_isvt(isvt_deinit_r(ba));
    sync_isvt(isvt_deinit_r(ba));
  }
  #endif //DEBUG
}
#ifdef NOP
// Обнуление сброса серводрайвера
void stop_res_servo()
{
  unioset(srv_alm_rst, 0);
  unioset(srv_s_on, 1);
  //srv_alm_rst_flag = 0;
  //srv_s_on_flag = 1;
  reset_servo = 0;
}
#endif
// Проверка на аварийное состояние
void check_fail()
{
  // Процедура сброса серводрайвера
  reset_servo_proc();

  // Обработка аварий по АСК, кнопке Стоп, СЗ, срабатыванию ИСВТ
  if (!fail) {
    #ifndef NOP
    int n_err1 = int((isvt_st&1)|((isvt_st >> 1)&1));
    #ifdef DBGONCPU
    int n_err2 = 0;
    int n_err3 = 0;
    #else //DBGONCPU
    int n_err2 = int(((isvt_st >> 2)&1)|((isvt_st >> 3)&1));
    int n_err3 = int(((isvt_st >> 4)&1)|((isvt_st >> 5)&1));
    #endif //DBGONCPU
    if (n_err1 + n_err2 + n_err3 > 1) n_error = 1;
    #else //NOP
    int n_err1 = 1 - dzz1_v;
    #ifdef DBGONCPU
    int n_err2 = 0;
    int n_err3 = 0;
    #else //DBGONCPU
    int n_err2 = 1 - dzz2_v;
    int n_err3 = 1 - dzz3_v;
    #endif //DBGONCPU
    if (n_err1 + n_err2 + n_err3 > 1) n_error = 1;
    #endif //NOP
    //mode_testn
    if (ask_zakr_v || n_error || k_stop_v || sz_v) {
        //vd_zakr = 1;
      start_turbine = 0;
      if (((ask_zakr_v || n_error) && !valve_test_proc) || k_stop_v || sz_v)
        stop_turbine();
      fail = 1;
      if (k_stop_v) stop_error = 1;
      unioset(avariya, 1);
    }
  }
  if (fail) {
    samo_bl &= 0xFFFF00FFL;
    samo_bl |= 0x800L;
    samo_bl_all();
    n_ref = 0.f;
    ne_ref = 0.f;
    t_ref1 = 0.f;
    t_ref2 = 0.f;
    tpv_ref = 0.f;
  }
  // Запись заданий регуляторов во флэш
  static int fail_s = 0;
  if (chk_event(&fail_s, fail, 0)) need_writebuf = 1;
}
// Взведение АСК
void vzv_ask()
{
  if (ask_otkr_v || ((hsvd < hsvd_min + 3.f) && !k_stop_v && !sz_v))
  if (k_vzv_ask_v && k_vzv_ask_e) {
        reset_fail();
    #ifdef DBGONCPU
    dbg_k_vzv_ask = 1;
    #endif //DBGONCPU
  }
  if (ask_otkr_v) {
    fail = 0;
    n_error = 0;
    start_turbine = 1;
    stop_error = 0;
    unioset(avariya, 0);
  }
}
// Выдача команды ИСВТ на взведение АСК
void reset_fail()
{
  #ifdef SENS_SEL_BLOCK_ON_VALVE_TEST
  mode_testn = 0;
  isvt_st = 0;
  for (int isvt_freq_idx = 0; isvt_freq_idx < isvt_freq_count; 
    isvt_freq_idx++) {
    #ifndef DEBUG
    vzv_zz(isvt_freq_idx);
    #endif //DEBUG
  }
  #else //SENS_SEL_BLOCK_ON_VALVE_TEST
  mode_testn = 0;
  isvt_st = 0;
  for (int i = 0; i < 3; i++) {
    #ifndef DEBUG
    uint ba = *isvt_ba[i];
    uint n12ti = uint(*n12tm[i]*10.f);
    sync_isvt(isvt_init_r(ba, n12ti, 0));
    #endif //DEBUG
    sensor_select[i] = 0;
  }
  #endif //SENS_SEL_BLOCK_ON_VALVE_TEST
}
// Выбор текущего датчика Pнто
void pnto_select()
{
  float pnto_ = 0.f;
  if (1 == pnto_sel) pnto_ = pnto1;
  if (2 == pnto_sel) pnto_ = pnto2;
  if (0 == pnto_sel) pnto_ = -100.f;
  pnto = pnto_;
}
// Обработка кнопок ВД
void k_vd_proc()
{
  static double ref = 0.f;
  int k_vd_mode = 0;
  static int k_vd_mode_prev = 0;
  static int kvdb_prev = 0, kvdm_prev = 0;
  int kvdb, kvdm;
  if (vd_sw != 3)
  if (kg_v) {
        if (vd_sw == 2) {
        k_vd_mode = 4;
    } else {
      if (1 == opt_sw) {
        k_vd_mode = 2;
      } else {
        k_vd_mode = 3;
      }
    }
  } else {
        k_vd_mode = 1;
  }
  if (k_vd_mode) {
    if (k_vd_mode != k_vd_mode_prev) {
      #ifdef NEW_BUT
      vd_dig_f = 0;
      vd_dec_f = 0;
      k_vd_d = 1.f;
      #else //NEW_BUT
      k_vd_par1 = *k_vd_par;
      #endif //NEW_BUT
      switch (k_vd_mode) {
        case 1:
          k_vd_par = &n_ref;
          k_vd_min = &n_ref_min;
          k_vd_max = &n_ref_max;
          #ifndef NEW_BUT
          k_vd_d = (*k_vd_max - *k_vd_min)*k_vd_b;
          k_vd_c = k_vd_d;
          #endif //NEW_BUT
          break;
        case 2:
          k_vd_par = &ne_ref;
          k_vd_min = &ne_smin;
          k_vd_max = &ne_smax;
          #ifndef NEW_BUT
          k_vd_d = (*k_vd_max - *k_vd_min)*k_vd_b;
          k_vd_c = k_vd_d;
          #endif //NEW_BUT
          break;
        case 3:
          k_vd_par = &t_ref1;
          k_vd_min = &t1_smin;
          k_vd_max = &t1_smax;
          #ifndef NEW_BUT
          k_vd_d = (*k_vd_max - *k_vd_min)*k_vd_b;
          k_vd_c = k_vd_d;
          #endif //NEW_BUT
          break;
        case 4:
          ref = servo_cnt_ref1;
          k_vd_d = dt*(servo_cnt_max - servo_cnt_min)/40.f;
          break;
      }
    }
    #ifdef DBGONCPU
    kvdb = dbg_vd_bol;
    kvdm = dbg_vd_men;
    #else //DBGONCPU
    kvdb = k_vd_bol_v;
    kvdm = k_vd_men_v;
    #endif //DBGONCPU
    if (4 == k_vd_mode) {
      if (kvdb) {
        //ref = bound(ref + k_vd_d, double(servo_cnt_min),
          //double(servo_cnt_max));
        ref += k_vd_d;
        if (ref < double(servo_cnt_min)) ref = double(servo_cnt_min);
        if (ref > double(servo_cnt_max)) ref = double(servo_cnt_max);
        servo_cnt_ref1 = ref;
      } else if (kvdm) {
        //ref = bound(ref - k_vd_d, servo_cnt_min,
          //servo_cnt_max);
        ref -= k_vd_d;
        if (ref < double(servo_cnt_min)) ref = double(servo_cnt_min);
        if (ref > double(servo_cnt_max)) ref = double(servo_cnt_max);
        servo_cnt_ref1 = ref;
      }
    } else {
      #ifdef NEW_BUT
      if ((kvdb != kvdb_prev) || (kvdm != kvdm_prev)) {
        if (kvdb || kvdm) {
          vd_dig_f = 1;
          vd_dec_f = 1;
          set_to_cnt_t(vd_dig_to, t_dig);
          set_to_cnt_t(vd_dec_to, t_dec);
        } else {
          vd_dig_f = 0;
          vd_dec_f = 0;
          k_vd_d = 1.f;
          //k_vd_d = 5.f;
        }
      }
      if (vd_dec_f) {
        if (test_to_cnt_t(vd_dec_to)) {
          if (k_vd_d < 1.f) {
            k_vd_d *= 10.f;
            set_to_cnt_t(vd_dec_to, t_dec);
          } else {
            vd_dec_f = 0;
          }
        }
      }
      if (vd_dig_f) {
        if (test_to_cnt_t(vd_dig_to)) {
          if (kvdb) {
            *k_vd_par = bound(*k_vd_par + k_vd_d, *k_vd_min, *k_vd_max);
            set_to_cnt_t(vd_dig_to, t_dig);
          } else if (kvdm) {
            *k_vd_par = bound(*k_vd_par - k_vd_d, *k_vd_min, *k_vd_max);
            set_to_cnt_t(vd_dig_to, t_dig);
          }
        }
      }
      #else //NEW_BUT
      if ((kvdb != kvdb_prev) || (kvdm != kvdm_prev)) {
        if (kvdb || kvdm) {
          k_vd_par1 = *k_vd_par;
        }
      }
      if (kvdb) {
        if (*k_vd_par < *k_vd_max) k_vd_c *= k_vd_k;
        *k_vd_par = bound(k_vd_par1 + k_vd_c - k_vd_d, *k_vd_min, *k_vd_max);
      } else if (kvdm) {
        if (*k_vd_par > *k_vd_min) k_vd_c *= k_vd_k;
        *k_vd_par = bound(k_vd_par1 - k_vd_c + k_vd_d, *k_vd_min, *k_vd_max);
      } else {
        k_vd_c = k_vd_d;
      }
      #endif //NEW_BUT
    }
  }
  k_vd_mode_prev = k_vd_mode;
  kvdb_prev = kvdb; kvdm_prev = kvdm;
}
// Обработка кнопок НД
void k_nd_proc()
{
  int k_nd_mode = 0;
  static int k_nd_mode_prev = 0;
  static int kndb_prev = 0, kndm_prev = 0;
  int kndb, kndm;
  if (1 == nd_sw)
  if (kg_v) {
    if (1 == opt_sw) k_nd_mode = 1;
    if (3 == opt_sw) k_nd_mode = 2;
  } else {
    k_nd_mode = 3;
  }
  if (k_nd_mode) {
    if (k_nd_mode != k_nd_mode_prev) {
      #ifdef NEW_BUT
      nd_dig_f = 0;
      nd_dec_f = 0;
      k_nd_d = 1.f;
      #else //NEW_BUT
      k_nd_par1 = *k_nd_par;
      #endif //NEW_BUT
      switch (k_nd_mode) {
        case 1:
          k_nd_par = &t_ref2;
          k_nd_min = &t1_smin;
          k_nd_max = &t1_smax;
          #ifndef NEW_BUT
          k_nd_d = (*k_nd_max - *k_nd_min)*k_nd_b;
          k_nd_c = k_nd_d;
          #endif //NEW_BUT
          break;
        case 2:
          k_nd_par = &tpv_ref;
          k_nd_min = &tpv_smin;
          k_nd_max = &tpv_smax;
          #ifndef NEW_BUT
          k_nd_d = (*k_nd_max - *k_nd_min)*k_nd_b;
          k_nd_c = k_nd_d;
          #endif //NEW_BUT
          break;
        case 3:
          k_nd_par = &hesnd_ref;
          k_nd_min = &hesnd_min;
          k_nd_max = &hesnd_max;
          k_nd_d = dt*(*k_nd_max - *k_nd_min)/40.f;
          break;
      }
    }
    #ifdef DBGONCPU
    kndb = dbg_nd_bol;
    kndm = dbg_nd_men;
    #else //DBGONCPU
    kndb = k_nd_bol_v;
    kndm = k_nd_men_v;
    #endif //DBGONCPU
    if (3 == k_nd_mode) {
      #ifdef ND_WITHOUT_SENS
      if (nd_nosens) {
        if (kndb) {
          nd_mode = 0;
          nd_dir = 1;
        } else if (kndm) {
          nd_mode = 0;
          nd_dir = -1;
        } else {
          nd_mode = 0;
          nd_dir = 0;
        }
      } else {
        if (kndb) {
          *k_nd_par = bound(*k_nd_par + k_nd_d, *k_nd_min, *k_nd_max);
        } else if (kndm) {
          *k_nd_par = bound(*k_nd_par - k_nd_d, *k_nd_min, *k_nd_max);
        }
      }
      #else //ND_WITHOUT_SENS
      if (kndb) {
        *k_nd_par = bound(*k_nd_par + k_nd_d, *k_nd_min, *k_nd_max);
      } else if (kndm) {
        *k_nd_par = bound(*k_nd_par - k_nd_d, *k_nd_min, *k_nd_max);
      }
      #endif //ND_WITHOUT_SENS
    } else {
      #ifdef NEW_BUT
      if ((kndb != kndb_prev) || (kndm != kndm_prev)) {
        if (kndb || kndm) {
          nd_dig_f = 1;
          nd_dec_f = 1;
          set_to_cnt_t(nd_dig_to, t_dig);
          set_to_cnt_t(nd_dec_to, t_dec);
        } else {
          nd_dig_f = 0;
          nd_dec_f = 0;
          k_nd_d = 1.f;
        }
      }
      if (nd_dec_f) {
        if (test_to_cnt_t(nd_dec_to)) {
          if (k_nd_d < 1.f) {
            k_nd_d *= 10.f;
            set_to_cnt_t(nd_dec_to, t_dec);
          } else {
            nd_dec_f = 0;
          }
        }
      }
      if (nd_dig_f) {
        if (test_to_cnt_t(nd_dig_to)) {
          if (kndb) {
            *k_nd_par = bound(*k_nd_par + k_nd_d, *k_nd_min, *k_nd_max);
            set_to_cnt_t(nd_dig_to, t_dig);
          } else if (kndm) {
            *k_nd_par = bound(*k_nd_par - k_nd_d, *k_nd_min, *k_nd_max);
            set_to_cnt_t(nd_dig_to, t_dig);
          }
        }
      }
      #else //NEW_BUT
      if ((kndb != kndb_prev) || (kndm != kndm_prev)) {
        if (kndb || kndm) {
          k_nd_par1 = *k_nd_par;
        }
      }
      if (kndb) {
        if (*k_nd_par < *k_nd_max) k_nd_c *= k_nd_k;
        *k_nd_par = bound(k_nd_par1 + k_nd_c - k_nd_d, *k_nd_min, *k_nd_max);
      } else if (kndm) {
        if (*k_nd_par > *k_nd_min) k_nd_c *= k_nd_k;
        *k_nd_par = bound(k_nd_par1 - k_nd_c + k_nd_d, *k_nd_min, *k_nd_max);
      } else {
        k_nd_c = k_nd_d;
      }
      #endif //NEW_BUT
    }
  }
  k_nd_mode_prev = k_nd_mode;
  kndb_prev = kndb; kndm_prev = kndm;
}
#ifdef SHOW_TLOOP
// Отображение времени цикла
void show_tloop()
{
  static long prev_cnt;
  static counter_t to_loop;
  static int _init = 1;
  if (_init) {
    _init = 0;
    prev_cnt = counter;
    set_to_cnt_t(to_loop, 3000.);
  } else {
    if (test_to_cnt_t(to_loop)) {
      set_to_cnt_t(to_loop, 3000.);
      dprintf("Длина цикла: %g\n", cnttoms(counter - prev_cnt));
    }
    prev_cnt = counter;
  }
}
#endif //SHOW_TLOOP
// Вывод отладочной информации
void dbg_info()
{
  static int fail_p = -1;
  static int rchvt_on_p = -1;
  static int kch_on_p = -1;
  static int eszo_on_p = -1;
  static int rm_on_p = -1;
  static int rto1_on_p = -1;
  static int rto2_on_p = -1;
  static int rtpv_on_p = -1;
  static int vd_on_p = -1;
  static int nd_on_p = -1;
  //start_reg_vd
  if (fail != fail_p) {
    if (fail) dputs("Fail"); else dputs("не Fail");
  }
  if (rchvt_on_p != rchvt_on) {
    if (rchvt_on) dputs("РЧВТ вкл."); else dputs("РЧВТ выкл.");
  }
  if (kch_on_p != kch_on) {
    if (kch_on) dputs("КЧ вкл."); else dputs("КЧ выкл.");
  }
  if (eszo_on_p != eszo_on) {
    if (eszo_on) dputs("ЭСЗО вкл."); else dputs("ЭСЗО выкл.");
  }
  if (rm_on_p != rm_on) {
    if (rm_on) dputs("РМ вкл."); else dputs("РМ выкл.");
  }
  if (rto1_on_p != rto1_on) {
    if (rto1_on) dputs("РТО1 вкл."); else dputs("РТО1 выкл.");
  }
  if (rto2_on_p != rto2_on) {
    if (rto2_on) dputs("РТО2 вкл."); else dputs("РТО2 выкл.");
  }
  if (rtpv_on_p != rtpv_on) {
    if (rtpv_on) dputs("РТПВ вкл."); else dputs("РТПВ выкл.");
  }
  if (vd_on_p != start_reg_vd) {
    if (start_reg_vd) dputs("ВД вкл."); else dputs("ВД выкл.");
  }
  if (nd_on_p != start_reg_nd) {
    if (start_reg_nd) dputs("НД вкл."); else dputs("НД выкл.");
  }
  fail_p = fail;
  rchvt_on_p = rchvt_on;
  kch_on_p = kch_on;
  eszo_on_p = eszo_on;
  rm_on_p = rm_on;
  rto1_on_p = rto1_on;
  rto2_on_p = rto2_on;
  rtpv_on_p = rtpv_on;
  vd_on_p = start_reg_vd;
  nd_on_p = start_reg_nd;
}
// Выдача текущего времени в формате: 0xddhhmmss
long get_time_to_long()
{
  ulong res = 0uL;
  struct dostime_t tm;
  _dos_gettime(&tm);
  res |= (ulong(tm.second%10) << 0);
  res |= (ulong(tm.second/10) << 4);
  res |= (ulong(tm.minute%10) << 8);
  res |= (ulong(tm.minute/10) << 12);
  res |= (ulong(tm.hour%10) << 16);
  res |= (ulong(tm.hour/10) << 20);
  struct dosdate_t dt;
  _dos_getdate(&dt);
  res |= (ulong(dt.day%10) << 24);
  res |= (ulong(dt.day/10) << 28);
  return res;
}
#ifdef DETECT_N_ANOMAL
// Детектирование общей аномальной частоты
void n_anomal_proc(
  // Исследуемая частота, об./мин.
  _in float an,
  // Входные/выходные данные
  _in_out n_anomal_data *a_n_anomal
)
{
  if (a_n_anomal->_init) {
    a_n_anomal->_init = 0;
    a_n_anomal->n_prev = an;
  } else {
    if (fabs(an - a_n_anomal->n_prev) > 70.f) {
      a_n_anomal->n_anomal_cur = an;
      a_n_anomal->n_anomal_prev = a_n_anomal->n_prev;
      a_n_anomal->time_n_anomal = get_time_to_long();
      a_n_anomal->detect = 1;
    }
    a_n_anomal->n_prev = an;
  }
}
// Детектирование аномальной частоты по конкретным датчикам
void nx_anomal_proc(
  // Массив частот от 6 датчиков, *10 об./мин.
  _in const uint *anm[],
  // При первом вызове функции должна быть единица
  _in_out byte &_init,
  // Текущая аномальная частота датчика, об./мин
  _out float &a_nx_anomal_cur,
  // Предыдущая аномальная частота датчика, об./мин
  _out float &a_nx_anomal_prev,
  // Время появления аномальной частоты датчика, 0xddhhmmss
  _out long &a_time_nx_anomal,
  // Датчик по которому была зафиксирована аномальная частота
  _out long &a_ind_nx_anomal,
  // Внутренние данные
  _internal nx_anomal_data *nx_anomal_a
)
{
  if (_init) {
    _init = 0;
    for (int i = 0; i < 6; i++) {
      nx_anomal_a->n_anomal_v[i]._init = 1;
      n_anomal_proc((*anm[i])/10.f, &nx_anomal_a->n_anomal_v[i]);
    }
  } else {
    for (int i = 0; i < 6; i++) {
      nx_anomal_a->n_anomal_v[i].detect = 0;
      n_anomal_proc((*anm[i])/10.f, &nx_anomal_a->n_anomal_v[i]);
      if (nx_anomal_a->n_anomal_v[i].detect) {
        a_nx_anomal_cur = nx_anomal_a->n_anomal_v[i].n_anomal_cur;
        a_nx_anomal_prev = nx_anomal_a->n_anomal_v[i].n_anomal_prev;
        a_time_nx_anomal = nx_anomal_a->n_anomal_v[i].time_n_anomal;
        a_ind_nx_anomal = i;
      }
    }
  }
}
#endif //DETECT_N_ANOMAL
// Vyvod otladochnoi informacii po tipy puts
void dputs(const char *s)
{
  doutstr(s);
  doutstr("\n");
}
// Vyvod otladochnoi informacii po tipy printf
void dprintf(const char *format, ...)
{
  char *msgbuf;
  int len;
  va_list arglist;
  va_start(arglist, format);
  len = vsnprintf(NULL, 0, format, arglist);
  len++;
  va_end(arglist);
  msgbuf = (char *)malloc(len);
  va_start(arglist, format);
  vsnprintf(msgbuf, len, format, arglist);
  msgbuf[len - 1] = 0;
  va_end(arglist);
  doutstr(msgbuf);
  free(msgbuf);
}
// Vyvod otladochnoi informacii
void doutstr(const char *s)
{
  #ifdef DPRINTF_OFF
  if (s) {
    char dummy = s[0];
    dummy = dummy;
  }
  #else //DPRINTF_OFF
  #ifdef DPRINTF_CON
  printf(s);
  #endif //DPRINTF_CON
  #ifdef DPRINTF_UDP
  if (udp_log_opened)
    sock_printf(&udp_log_sock, s);
  #endif //DPRINTF_UDP
  #endif //DPRINTF_OFF
}

// ******************************************************************
// Точка входа
// ******************************************************************
int main()
{
  #ifdef NOP
  init();
  lan_en = 1;
  for (;;) {
    count_int = counter;
    count_loop++;
    lanfn lfn = (lanfn)ldata.fn;
    if (lfn) lfn(&ldata);
    ldata2.tact = dt_lan2;
    lanfn lfn2 = (lanfn)ldata2.fn;
    if (lfn2) lfn2(&ldata2);
    if (kbhit()) {
      getch();
      break;
    }
    //dputs("for");
    //dprintf("lfn %p %p\n", (lanfn)ldata.fn, (lanfn)ldata2.fn);
    //dprintf("counter = %ld\n", counter);
  }
  deinit();
  exit(0);
  #endif


  #ifdef DEBUG
  dputs("\nПрограмма запущена");
  #endif

  // Инициализация
  init();
  
  // Основной цикл
  for (;;) {
    count_int = (long)(counter_get() >> 16);
    cur_time = get_time_to_long();

    //buf[300] = clock();
    //buf[300] = *(unsigned long far *)0x000046cL;
    
    #ifdef ADC_IN_LOOP
    // Чтение АЦП
    static irs_bool _init_ai_int = irs_true;
    static counter_t to_ai_int;
    const counter_t t_ai_int = TIME_TO_CNT(500, 1000000);
    if (_init_ai_int) {
      _init_ai_int = irs_false;
      set_to_cnt(to_ai_int, t_ai_int);
    }
    if (test_to_cnt(to_ai_int)) {
      set_to_cnt(to_ai_int, t_ai_int);
      ai_int();
    }
    #endif //ADC_IN_LOOP

    #ifndef ADC_IN_LOOP
    #ifndef DEBUG
    #ifdef REST_T0
    // Перезапуск таймера в случае, если его значение испорчено
    if ((long)get_timer0() > (long)TIMER0_CNT) {
      deinit_timer0();
      init_timer0();
    }
    #endif //REST_T0
    #endif //DEBUG
    #endif //ADC_IN_LOOP

    #ifndef NO_AI_FILTR
    #ifdef FADC_IN_LOOP
    // Фильтрация АЦП
    filter_ADC();
    #endif //FADC_IN_LOOP
    #endif //NO_AI_FILTR

    #ifdef PROC_IN_LOOP
    all_proc();
    #endif //PROC_IN_LOOP

    dbg_proc();

    #ifdef TEST
    test_save();
    #endif //TEST

    #ifdef DEBUG
    ti0();
    #endif

    #ifndef WDT_NEW
    #ifndef WDT_INT
    #ifndef DEBUG
    #ifndef LAN_SINGLE
    #ifdef WDT_ON
    // Сброс сторожевого таймера
    inp(0x20C);
    #endif
    #endif
    #endif
    #endif
    #endif //WDT_NEW

    #ifndef DEBUG
    // Выполнение сетевого обмена
    #ifndef DBGONCPU
    #ifndef LAN_RECEIVE_NOT_BLOCKING
    if (lan_en)
    #endif //LAN_RECEIVE_NOT_BLOCKING
    #endif //DBGONCPU
    {
      lanfn lfn = (lanfn)ldata.fn;
      if (lfn) lfn(&ldata);
      ldata2.tact = dt_lan2;
      lanfn lfn2 = (lanfn)ldata2.fn;
      if (lfn2) lfn2(&ldata2);
    }
    #endif //DEBUG

    // Счетчик циклов
    sync(count_loop++);

    #ifdef SHOW_TLOOP
    // Отображение времени цикла
    show_tloop();
    #endif //SHOW_TLOOP

    // Запись сетевых переменных во флэш
    static counter_t to_flash;
    static double t_flash = 1000.;
    static int _init = 1;
    if (_init) {
      _init = 0;
      set_to_cnt_t(to_flash, t_flash);
    } else {
      if (test_to_cnt_t(to_flash)) {
        set_to_cnt_t(to_flash, t_flash);
        if (need_writebuf) {
          need_writebuf = 0;
          writebuf(buf);
          writebuf(buf);
          //dputs("writebuf");
        }
      }
    }

    #ifdef RES_PKT_DRV
    // Процедура сброса пакетного драйвера
    res_pd_proc();
    #endif //RES_PKT_DRV

    // Выход из цикла
    if (stop_loop) {
      restart_flag = 0L;
      writebuf(buf);
      writebuf(buf);
      break;
    }
  }

  // Деинициализация
  deinit();
  // Отображение максимального времени обработки прерывания таймера
  // при выходе
  #ifdef SHOW_INT_T0
  //t_int_t0 TIMER0_CNT
  dprintf("Время обработки прерывания таймера: %g мс\n",
    tm0toms(TIMER0_CNT - t_int_t0));
  #endif //SHOW_INT_T0

  #ifdef DEBUG
  dputs("Программа остановлена");
  #endif

  return 0;
}
