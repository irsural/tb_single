#ifndef ISVTH
#define ISVTH

//#include <dos.h>
#include <conio.h>

unsigned char isvt_init_r(unsigned int ba, unsigned int Limit_Freq,
  signed int Limit_Acceleration);
unsigned char isvt_deinit_r(unsigned int ba);
unsigned char isvt_proc_r(unsigned int ba,
  unsigned char * Ext_Status1,unsigned char * Ext_Status2,
  unsigned int * Freq_Sensor1,signed int * Acceleration_Sensor1,
  unsigned int * Freq_Sensor2,signed int * Acceleration_Sensor2,
  unsigned int * Limit_Freq,signed int * Limit_Acceleration,
  unsigned int * Max_Freq_rotor, signed int * Max_Acceleration_rotor);
unsigned char isvt_test_init_r(unsigned int ba,unsigned int Start_Test_Freq,
  signed int Test_Acceleration);
unsigned char set_isvt_sensor_r(unsigned int ba,unsigned char Sensor);
#endif
