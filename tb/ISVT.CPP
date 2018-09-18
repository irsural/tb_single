#include "isvt.h"

#define R_Bit_Status1_Error_Sensor2  7 //0-исправен, 1-неисправен
#define R_Bit_Status1_Error_Sensor1  6 //0-исправен, 1-неисправен
#define R_Bit_Status1_Sensor2  5 //0-выключено, 1-включено для реакции на рэле
#define R_Bit_Status1_Sensor1  4 //0-выключено, 1-включено для реакции на рэле
#define R_Bit_Status1_Error_Rele2  3 //0-исправно, 1-неисправно
#define R_Bit_Status1_Error_Rele1  2 //0-исправно, 1-неисправно
#define R_Bit_Status1_Rele2_ON  1 //0-выключено, 1-включено
#define R_Bit_Status1_Rele1_ON  0 //0-выключено, 1-включено

#define R_Bit_Status2_Rele1_Enabled  7 //0-выключено, 1-включено для реакции
#define R_Bit_Status2_Rele2_Enabled  6 //0-выключено, 1-включено для реакции
#define R_Bit_Status2_Sensor1_Enabled  5 //0-выключено, 1-включено для расчёта значения частоты и производной по частоте
#define R_Bit_Status2_Sensor2_Enabled  4 //0-выключено, 1-включено для расчёта значения частоты и производной по частоте
#define R_Bit_Status2_Test  3 // 0-рабочий режим, 1 - режим теститрования
#define R_Bit_Status2_Defense_Freq 2 // 1 - сработала защита по частоте
#define R_Bit_Status2_Defense_Acceleration 1 //1 - сработала защита по производной частоты
#define R_Bit_Status2_Defense_Sensor 0  // датчик, по которому сработала защита 0 - по первому, 1 - по второму
//-------------------------------------------------------------------------------------------------------------
unsigned char isvt_init_r(unsigned int ba, unsigned int Limit_Freq,
  signed int Limit_Acceleration)
{
 unsigned char Status1,Status2;
 unsigned int Temp_ba;
 Temp_ba=inpw(ba+25);
 if (ba!=Temp_ba) return 0;
 else
 {
  Status1=0xFF&((1<<R_Bit_Status1_Sensor1)|(1<<R_Bit_Status1_Rele2_ON)|(1<<R_Bit_Status1_Rele1_ON));
  Status2=0xFF&((1<<R_Bit_Status2_Rele1_Enabled)|(1<<R_Bit_Status2_Rele2_Enabled)|(1<<R_Bit_Status2_Sensor1_Enabled)|(1<<R_Bit_Status2_Sensor2_Enabled));
  outpw(ba+12,Limit_Freq);
  outpw(ba+14,Limit_Acceleration);
  outp(ba+0,Status1);
  outp(ba+1,Status2);
  return 1;
 }
}
//-------------------------------------------------------------------------------------------------------------
unsigned char isvt_deinit_r(unsigned int ba)
{
 unsigned char Temp_Status2,Temp_Status1,Status1;
 unsigned int Temp_ba;
 Temp_Status1=(unsigned char)inp(ba);
 Temp_Status2=(unsigned char)inp(ba+1);
 Temp_ba=inpw(ba+25);
 Temp_Status1&=(0xFF^((1<<R_Bit_Status1_Error_Sensor2)|(1<<R_Bit_Status1_Error_Sensor1)|(1<<R_Bit_Status1_Error_Rele2)|(1<<R_Bit_Status1_Error_Rele1)));
 if (ba!=Temp_ba) return 0;
 else
 {
  Status1=Temp_Status1&(0xFF^((1<<R_Bit_Status1_Rele2_ON)|(1<<R_Bit_Status1_Rele1_ON)));
  if(Temp_Status1!=Status1){
   outp(ba,Status1);
   outp(ba+1,Temp_Status2);
  }
  return 1;
 }
}
//-------------------------------------------------------------------------------------------------------------
unsigned char isvt_proc_r(unsigned int ba,
  unsigned char * Ext_Status1,unsigned char * Ext_Status2,
  unsigned int * Freq_Sensor1,signed int * Acceleration_Sensor1,
  unsigned int * Freq_Sensor2,signed int * Acceleration_Sensor2,
  unsigned int * Limit_Freq,signed int * Limit_Acceleration,
  unsigned int * Max_Freq_rotor, signed int * Max_Acceleration_rotor)
{
 unsigned int Temp_ba;
 * Ext_Status1=(unsigned char)inp(ba);
 * Ext_Status2=(unsigned char)inp(ba+1);
 * Freq_Sensor1=inpw(ba+4);
 * Freq_Sensor2=inpw(ba+6);
 * Acceleration_Sensor1=inpw(ba+8);
 * Acceleration_Sensor2=inpw(ba+10);
 * Limit_Freq=inpw(ba+12);
 * Limit_Acceleration=inpw(ba+14);
 * Max_Freq_rotor=inpw(ba+20);
 * Max_Acceleration_rotor=inpw(ba+22);
 Temp_ba=inpw(ba+25);
 if (ba!=Temp_ba) return 0;
 else { return 1; }
}
//-------------------------------------------------------------------------------------------------------------
unsigned char isvt_test_init_r(unsigned int ba,unsigned int Start_Test_Freq,
  signed int Test_Acceleration)
{
 unsigned char Temp_Status1,Temp_Status2,Status2;
 unsigned int Temp_ba;
 Temp_Status1=(unsigned char)inp(ba);
 Temp_Status2=(unsigned char)inp(ba+1);
 Temp_ba=inpw(ba+25);
 if (ba!=Temp_ba) return 0;
 else
 {
  Status2=Temp_Status2|(0xFF&(1<<R_Bit_Status2_Test));
  outpw(ba+16,Start_Test_Freq);
  outpw(ba+18,Test_Acceleration);
  outp(ba,Temp_Status1);
  //if(Temp_Status2!=Status2)
  outp(ba+1,Status2);
  return 1;
 }
}
//-------------------------------------------------------------------------------------------------------------
unsigned char set_isvt_sensor_r(unsigned int ba,unsigned char Sensor)
{
 unsigned char Temp_Status2,Temp_Status1,Status1;
 unsigned int Temp_ba;
 Temp_Status1=(unsigned char)inp(ba);
 Temp_Status2=(unsigned char)inp(ba+1);
 Temp_ba=inpw(ba+25);
 Temp_Status1&=(0xFF^((1<<R_Bit_Status1_Error_Sensor2)|(1<<R_Bit_Status1_Error_Sensor1)|(1<<R_Bit_Status1_Error_Rele2)|(1<<R_Bit_Status1_Error_Rele1)));
 if (ba!=Temp_ba) return 0;
 else
 {
  Status1=Temp_Status1&(0xFF^((1<<R_Bit_Status1_Sensor2)|(1<<R_Bit_Status1_Sensor1)));
  if(Sensor==1)
  {Status1|=(1<<R_Bit_Status1_Sensor1);outp(ba,Status1);outp(ba+1,Temp_Status2);}
  if(Sensor==2)
  {Status1|=(1<<R_Bit_Status1_Sensor2);outp(ba,Status1);outp(ba+1,Temp_Status2);}
//  if(Sensor==0){outp(ba,Status1);}
//  outp(ba,Status1);
  return 1;
 }
}

