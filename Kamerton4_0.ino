/*

 Kamerton4_0_0.ino
 VisualStudio
 
 ��������� ������������ ������ "��������" (������� �������)
 ������:      - 4_0_0
 ����:        - 10.06.2015�.
 �����������: - ��� "������"
 �����:       - �������� �.�.
 ������: ������ ���������� ����� �������� 10.06.2015 �.
 �����������:
 -
 - ���������� 20��,
 - ��������/����� �� ��� �����,
 - ������� ���������� ����, ����� � ����������, 
 - ���������� MCP23017
 - ������ ����, 
 - ������ ���� ������, 
 - ��������� �������� ���������
 - ���������� SD ������
 - ���������� ����, ������, 
 */



#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <MsTimer2.h> 
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include <Wire.h>   
#include "MCP23017.h"


#define  ledPin13  13                               // ���������� ����������� �� �����
#define  ledPin12  12                               // ���������� ����������� �� �����
#define  ledPin11  11                               // ���������� ����������� �� �����
#define  ledPin10  10                               // ���������� ����������� �� �����
#define  Front_led_Blue 14                          // ���������� ����������� �� �������� ������
#define  Front_led_Red  15                          // ���������� ����������� �� �������� ������

//  ����� ���������� ������ ��������
#define DTR  8                                      // DTR out �������� ������  ������������ 0 ��� ������
#define RTS  9                                      // RTS out �������� ������   
#define CTS  5                                      // CTS in  ������� ������  ���� ������� �������� ��������������!!!!
#define DSR  6                                      // DSR in  ������� ������  ���� ������� "����� - ��������"
#define DCD  7                                      // DCD in  ������� ������  ���� ������ ������ � ��������� 

//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // ����� ���������� AD5252  
#define control_word1    0x07                       // ���� ���������� �������� �1
#define control_word2    0x87                       // ���� ���������� �������� �2
byte resistance        = 0x00;                      // ������������� 0x00..0xFF - 0��..100���
//byte level_resist      = 0;                       // ���� ��������� ������ �������� ���������
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
unsigned int Array_volume[514];                     //
unsigned int volume_porog_D       = 30;             // �������� ������ ��� �������� ����������� FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // ����� �������� ��������� �����
unsigned int  voltage10 ;

#define FASTADC 1                                   // ��������� ���������� ����������� �������
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//**************************************************************************
int analog_tok            = 0;       //
int analog_12V            = 1;       //
int analog_tok_x10        = 2;       //
int analog_radio4         = 3;       //
int analog_mag4           = 4;       //
int analog_gg_radio1      = 5;       //
int analog_gg_radio2      = 6;       //
int analog_ggs            = 7;       //
int analog_LineL          = 8;       //
int analog_LineR          = 9;       //
int analog_FrontL        = 10;       //
int analog_FrontR        = 11;       //
int analog_W             = 12;       //
int analog_13            = 13;       //
int analog_14            = 14;       //
int analog_12v_x1        = 15;       //

//************************************************************************************************

RTC_DS1307 RTC;                                     // define the Real Time Clock object


MCP23017 mcp_Klava;                                 // ���������� ������ ���������� MCP23017  2 A - in,  B - Out
MCP23017 mcp_Out1;                                  // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // ���������� ������ ���������� MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // ���������� ������ ���������� MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------


int adr_reg_ind_CTS                 = 10081;        // ����� ����a ��������� ��������� ������� CTS
int adr_reg_ind_DSR                 = 10082;        // ����� ����a ��������� ��������� ������� DSR
int adr_reg_ind_DCD                 = 10083;        // ����� ����a ��������� ��������� ������� DCD

// **************** ������ ������� ������ ��� �������� ����. ����������� ��������������� ����� ����� *************
int adr_temp_day                    = 240;          // ����� �������� ���������� ����
int adr_temp_mon                    = 241;          // ����� �������� ���������� �����
int adr_temp_year                   = 242;          // ����� �������� ���������� ���  
int adr_file_name_count             = 243;          // ����� �������� ���������� �������� ������ �����
//------------------------------------------------------------------------------------------------------------------

//*********************������ � ������ ����� ******************************
char file_name[13] ;
char file_name_txt[5] = ".txt";
byte file_name_count = 0;
char str_day_file[3];
char str_day_file0[3];
char str_day_file10[3];
char str_mon_file[3];
char str_mon_file0[3];
char str_mon_file10[3];
char str_year_file[3];

char str_file_name_count[4];
char str_file_name_count0[4] = "0";
char str0[10];
char str1[10];
char str2[10];
char str3[13];
char str_kamIn[10];

//-------------------------------------------------------------------------

//+++++++++++++++++++++ ��������� ���������� +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//++++++++++++++ ������� ����������, ������������ ������� ���������� SD utility library functions: +++++++++++++++
Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;
const int SD_Select = 53;    
//-----------------------------------------------------------------------------------------
//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

byte regs_in[5];                                    // �������� ������ � ������ �������� CPLL
byte regs_out[4];                                   // �������� ������ � ������ ��������
byte regs_crc[1];                                   // �������� ������ � ������ �������� ����������� �����


byte Stop_Kam = 0;                                  // ���� ��������� ������ ���. �� ���������
byte prer_Kmerton_On = 0;                           // ���� ���������� ���������� ��������
byte prer_Kmerton_Run = 0;                          // ���� ���������� ���������� ��������
#define BUFFER_SIZEK 64                             // ������ ������ �������� �� ����� 128 ����
unsigned char bufferK;                              // ������� ���������� ����������� ����

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned int adr_control_command         = 40120; //����� �������� ������� �� ���������� 

// ������� ����� 
unsigned int adr_kontrol_day             = 40046; // ����� ����
unsigned int adr_kontrol_month           = 40047; // ����� �����
unsigned int adr_kontrol_year            = 40048; // ����� ���
unsigned int adr_kontrol_hour            = 40049; // ����� ���
unsigned int adr_kontrol_minute          = 40050; // ����� ������
unsigned int adr_kontrol_second          = 40051; // ����� �������

// ��������� ������� � �����������
unsigned int adr_set_kontrol_day         = 40052;   // ����� ����
unsigned int adr_set_kontrol_month       = 40053;   // ����� �����
unsigned int adr_set_kontrol_year        = 40054;   // ����� ���
unsigned int adr_set_kontrol_hour        = 40055;   // ����� ���
unsigned int adr_set_kontrol_minute      = 40056;   // ����� ������

// ����� ������ �����
unsigned int adr_Mic_Start_day           = 40096; // ����� ����
unsigned int adr_Mic_Start_month         = 40097; // ����� �����
unsigned int adr_Mic_Start_year          = 40098; // ����� ���
unsigned int adr_Mic_Start_hour          = 40099; // ����� ���
unsigned int adr_Mic_Start_minute        = 40100; // ����� ������
unsigned int adr_Mic_Start_second        = 40101; // ����� �������

// ����� ��������� �����
unsigned int adr_Mic_Stop_day            = 40102; // ����� ����
unsigned int adr_Mic_Stop_month          = 40103; // ����� �����
unsigned int adr_Mic_Stop_year           = 40104; // ����� ���
unsigned int adr_Mic_Stop_hour           = 40105; // ����� ���
unsigned int adr_Mic_Stop_minute         = 40106; // ����� ������
unsigned int adr_Mic_Stop_second         = 40107; // ����� �������

// ����������������� ���������� �����
unsigned int adr_Time_Test_day           = 40108; // ����� ����
unsigned int adr_Time_Test_hour          = 40109; // ����� ���
unsigned int adr_Time_Test_minute        = 40110; // ����� ������
unsigned int adr_Time_Test_second        = 40111; // ����� �������

unsigned int adr_set_time                = 36;    // ����� ���� ���������

//-------------------------------------------------------------------------------------


//========================= ���� �������� ============================================


void flash_time()                                              // ��������� ���������� ���������� 
{ 
	prer_Kmerton_Run = 1;
		digitalWrite(ledPin12,HIGH);
		//digitalWrite(ledPin12,!digitalRead(ledPin12));       // ���� ������� ������ ����������
	    prer_Kamerton();
		digitalWrite(ledPin12,LOW);
	prer_Kmerton_Run = 0;
}

void serialEvent2()
{
	digitalWrite(ledPin13,HIGH);
 // digitalWrite(ledPin13,!digitalRead(ledPin13));               // ���� ������� MODBUS
	while(prer_Kmerton_Run == 1)                               // ��������� ��������� ����������
		{
		}
	slave.run(); 
	digitalWrite(ledPin13,LOW);
}

void dateTime(uint16_t* date, uint16_t* time)                  // ��������� ������ ������� � ���� �����
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
void prer_Kamerton()                                          // ���������� ����� ���������� � ������� ��������
{
	sendPacketK ();  
	// ��������� ���������� � ������ ��������
	waiting_for_replyK();                                  // �������� �������������
}
void sendPacketK () 
{              // ��������� �������� ������ � ��������
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
		}
}
void waiting_for_replyK()                                  // ������ ������ �� ���������
{
	  if (Serial1.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer = 0;                      // ���������� � ������ ������ ������

			while (Serial1.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial1.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (bufferK == BUFFER_SIZEK)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
						regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
			calculateCRC_In();
		   }
	 else 
		{
			Stop_Kam = 0;                                    // ���� ��������. ���. �� ���������
		}
}
void Stop_Kamerton ()                  //���� �� �������� ���������� � ��������� - �������� ��������
  {
	 for (unsigned char i = 0; i <4; i++)
	 regs_in[i]=0;
  }

void calculateCRC_Out()                // ���������� ����������� ����� ������ �����
{ 
  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // ��������  
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 �����
  temp2 = temp1>>4;                    // ����������� ������� ���� � �������
  temp3 = regs_out[2];                 // ��������
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 2 �����
  temp3 = temp3>>4;                    // ����������� ������� ���� � �������
  temp4 = regs_out[2];                 // ��������
  temp4 = temp4&0x0F;                  // �������� ����� F0 �� ������� ���� 2 �����
  crc =  temp2 ^  temp3 ^  temp4  ;
  crc = crc&0x0F;                      // �������� ����� F0 �� ������� ���� 2 �����
  regs_out[1]= temp1 | crc;
}
void calculateCRC_In()                 // ���������� ����������� ����� ������ �����
{ 
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // ��������  
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 �����
  temp1H = temp1>>4;                   // ����������� ������� ���� � �������
  temp1 = regs_in[0];                  // �������� 
  temp1L = temp1&0x0F;                 // �������� ����� 0F �� ������� ���� 1 �����

  temp2 = regs_in[1];                  // ��������  
  temp2 = temp2&0xF0;                  // �������� ����� F0 �� ������� ���� 2 �����
  temp2H = temp2>>4;                   // ����������� ������� ���� � �������
  temp2 = regs_in[1];                  // �������� 
  temp2L = temp2&0x0F;                 // �������� ����� 0F �� ������� ���� 2 �����

  temp3 = regs_in[2];                  // ��������  
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp3H = temp3>>4;                   // ����������� ������� ���� � �������
  temp3 = regs_in[2];                  // �������� 
  temp3L = temp3&0x0F;                 // �������� ����� 0F �� ������� ���� 3 �����

  temp4 = regs_in[3];                  // ��������  
  temp4 = temp4&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp4H = temp4>>4;                   // ����������� ������� ���� � �������
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // �������� ����� F0 �� ������� ���� 4 �����
  regs_crc[0]= crc_in;
}
void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
	int rdata = data;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.write(rdata);
	Wire.endTransmission();
	delay(10);
}
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
	byte rdata = 0xFF;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,1);
	if (Wire.available()) rdata = Wire.read();
	return rdata;
}

void reg_Kamerton()                                       // ������������ �������������� � �������� ������ ��  regs_out[3] � �������� Samkoon
{
	 byte i4,i5;
	 unsigned int  i7=30001;                             //������� �������� ������� ����� ������ ��� �������� � ��������
	//  unsigned int  i7=10224;    

	 regBank.set(40001,regs_out[0]);   
	 regBank.set(40002,regs_out[1]);   
	 regBank.set(40003,regs_out[2]);   
	 regBank.set(40004,regs_in[0]);   
	 regBank.set(40005,regs_in[1]); 
	 regBank.set(40006,regs_in[2]); 
	 regBank.set(40007,regs_in[3]); 


	//i5 = regs_out[0];                                     // 1 ���� ��������
	/*
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+i4,bitRead(i5,i4));            // 1 ���� �������� ��������� � �������� Samkoon
		}
	i5= regs_out[1];                                      // 2 ���� ��������
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+8+i4,bitRead(i5,i4));          // 2 ���� �������� ��������� � �������� Samkoon
		}
	i5= regs_out[2];                                      // 3 ���� ��������
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+16+i4,bitRead(i5,i4));         // 3 ���� �������� ��������� � �������� Samkoon
		}
	
	//********************************************************************************

	i5= regs_in[0];                                      // 1 ���� ���������� �� ���������
		for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+24+i4,bitRead(i5,i4));
		}
	i5= regs_in[1];
	for (i4 = 0; i4 <8; i4++)                              // 2 ���� ���������� �� ���������
		{
			regBank.set(i7+32+i4,bitRead(i5,i4));
		}
	i5= regs_in[2];                                        // 3 ���� ���������� �� ���������
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+40+i4,bitRead(i5,i4));
		}
	i5= regs_in[3];                                        // 4 ���� ���������� �� ���������
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+48+i4,bitRead(i5,i4));
		}
	*/
	regBank.set(adr_reg_ind_CTS, !mcp_Analog.digitalRead(CTS));
	regBank.set(adr_reg_ind_DSR, !mcp_Analog.digitalRead(DSR));
	regBank.set(adr_reg_ind_DCD, !mcp_Analog.digitalRead(DCD));

		 
}
void UpdateRegs()                             // �������� ��������
	// ������������ �������� ������ �� ��������� �� ����� ������
{
	//-----������ ���� ------------
	//-----���������� ��� 0
	//while(prer_Kmerton_Run==1)
	//{
	//}
prer_Kmerton_On = 0;
	  reg_Kamerton();

	  //-----���������� ��� 0
	boolean set_rele = regBank.get(1);
	mcp_Out1.digitalWrite(0, set_rele);                 // ���� RL0 ����  ���� Mic1p ���������

	 //-----���������� ��� 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // ���� RL1 ���� Mic2p  ���������

	 //-----���������� ��� 2
	  set_rele = regBank.get(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // ���� RL2 ���� Mic3p MTT
  
	 //-----���������� ��� 3
	  set_rele = regBank.get(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // ���� RL3 ����

	 //-----���������� ��� 4                            // ���� RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----���������� ��� 5
	  set_rele = regBank.get(6);                        // ���� RL5 ����
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----���������� ��� 6	 
	  set_rele = regBank.get(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // ���� RL6 ����

	 //-----���������� ��� 7
	  set_rele = regBank.get(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // ���� RL7 ������� �����

	 //---- ������ ����----------
	 //-----���������� ��� 8
	  set_rele = regBank.get(9);                        // ���� RL8 ���� �� ��������
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----���������� ��� 9
	  set_rele = regBank.get(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // ���� RL9 XP1 10

	 //-----���������� ��� 10                           // �������� J24 - 2 
	  set_rele = regBank.get(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----���������� ��� 11                            // �������� J24 - 1
	  set_rele = regBank.get(12);
	  mcp_Out1.digitalWrite(11, set_rele);    

	 //-----���������� ��� 12
	  set_rele = regBank.get(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   Sence �������� ������

	 //-----���������� ��� 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT �������� ������

	 //-----���������� ��� 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT ���

	  //-----���������� ��� 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   Sence ���

	  //  Test 3
	 //-----������ ���� ------------
	 //-----���������� ��� 0

	  set_rele = regBank.get(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   ����. �.

	 //-----���������� ��� 1
	  set_rele = regBank.get(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----���������� ��� 2
	  set_rele = regBank.get(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 Sence  ����. �.
  
	//-----���������� ��� 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 ����. �.

	 //-----���������� ��� 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     Sence "���." 

	 //-----���������� ��� 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     Sence "��C."

	 //-----���������� ��� 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     Sence "��-�����1."

	 //-----���������� ��� 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     Sence "��-�����2."

	  // Test 4
	//-----������ ���� ------------
	 //-----���������� ��� 8
	  set_rele = regBank.get(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      ���� ����������� ������  

	  //-----���������� ��� 9
	  set_rele = regBank.get(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR ���.

	  //-----���������� ��� 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    ���� ����������� ��������� ����������� � 2 ����������

	  //-----���������� ��� 11
	  set_rele = regBank.get(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS ���

	  //-----���������� ��� 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    ���� ����������� ��������� ����������� 

	  //-----���������� ��� 13
	  set_rele = regBank.get(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS ���

	  //-----���������� ��� 14
	  set_rele = regBank.get(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    ���� ���������� ��������� ���������� � 2 ����������

	  //-----���������� ��� 15
	  set_rele = regBank.get(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    ���� ���������� ��������� ����������

	  //*******************************************************
/*
// ��������� ��������� ��������� � ����������
  
//	  avto_test  =  regBank.get(33);
	 // start_prer =  regBank.get(34);


	  regBank.set(adr_reg_count_cts0, count_test_CTS);                    // �������� � ������� ��������� �������� �������� ����� CTS
	  regBank.set(adr_reg_err_cts0, err_count_cts);                       // �������� � ������� ��������� �������� ������ CTS
	  regBank.set(adr_reg_err_volume_HaSPTT, err_count_volume_HaSPTT);    // �������� � ������� ��������� �������� ��� ����� HaSPTT
	  regBank.set(adr_reg_err_HaSPTT_ON, err_count_HaSPTT_ON);            // �������� � ������� ��������� �������� ������ HaSPTT_ON
	  regBank.set(adr_reg_err_HaSPTT_OFF, err_count_HaSPTT_OFF);          // �������� � ������� ��������� �������� ������ HaSPTT_OFF
	  regBank.set(adr_reg_err_volume_HeS1PTT, err_count_volume_HeS1PTT);  // �������� � ������� ��������� �������� ��� �����  HeS1PTT
	  regBank.set(adr_reg_err_HeS1PTT_ON, err_count_HeS1PTT_ON);          // �������� � ������� ��������� �������� ������ HeS1PTT_ON
	  regBank.set(adr_reg_err_HeS1PTT_OFF,err_count_HeS1PTT_OFF );        // �������� � ������� ��������� �������� ������ HeS1PTT_OFF
	  regBank.set(adr_reg_err_volume_HeS2PTT,err_count_volume_HeS2PTT);   // �������� � ������� ��������� �������� ��� ����� HeS2PTT
	  regBank.set(adr_reg_err_HeS2PTT_ON,err_count_HeS2PTT_ON);           // �������� � ������� ��������� �������� ������ HeS2PTT_ON     
	  regBank.set(adr_reg_err_HeS2PTT_OFF,err_count_HeS2PTT_OFF);         // �������� � ������� ��������� �������� ������ HeS2PTT_OFF
	  regBank.set(adr_reg_err_volume_MicPTT, err_count_volume_MicPTT);    // �������� � ������� ��������� �������� ��� �����  MicPTT
	  regBank.set(adr_reg_err_MicPTT_ON, err_count_MicPTT_ON);            // �������� � ������� ��������� �������� ������ MicPTT_ON
	  regBank.set(adr_reg_err_MicPTT_OFF,err_count_MicPTT_OFF );          // �������� � ������� ��������� �������� ������ MicPTT_OFF
	  regBank.set(adr_reg_err_volume_TangNPTT,err_count_volume_TangNPTT); // �������� � ������� ��������� �������� ��� ����� HeS2PTT
	  regBank.set(adr_reg_err_TangNPTT_ON,err_count_TangNPTT_ON);         // �������� � ������� ��������� �������� ������ HeS2PTT_ON     
	  regBank.set(adr_reg_err_TangNPTT_OFF,err_count_TangNPTT_OFF);       // �������� � ������� ��������� �������� ������ HeS2PTT_OFF

	 // regBank.set(adr_reg_count_Mic, count_test_Mic);                     // �������� � ������� ��������� �������� �������� ����� Mic
	  regBank.set(adr_reg_err_Mic, err_count_Mic);                        // �������� � ������� ��������� �������� ������ CTS
	  */
	//  time_control();
	  prer_Kmerton_On = 1;
}
void set_clock()
{    
		int day    = regBank.get(adr_set_kontrol_day);  
		int month  = regBank.get(adr_set_kontrol_month);          
		int year   = regBank.get(adr_set_kontrol_year);  
		int hour   = regBank.get(adr_set_kontrol_hour);  
		int minute = regBank.get(adr_set_kontrol_minute);  
		int second = 0;
		DateTime set_time = DateTime(year, month, day, hour, minute, second);
		RTC.adjust(set_time);
		regBank.set(adr_set_time, 0);    
}
void data_clock_exchange()
{
	DateTime now = RTC.now();
	uint16_t year_temp = now.year()-2000;
	uint8_t day_temp = now.day();
	uint8_t mon_temp = now.month();
		 
	byte  b = i2c_eeprom_read_byte(0x50, adr_temp_day); //access an address from the memory
		delay(10);

		if (b != day_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_day, day_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// ��� ����� ���� ������� ������ ����� �������� � "0"
			}

		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon); //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year); //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// ��� ����� ���� ������� ������ ����� �������� � "0"
			}
}
void time_control()
{
	DateTime now = RTC.now();
	regBank.set(adr_kontrol_day  , now.day());
	regBank.set(adr_kontrol_month, now.month());
	regBank.set(adr_kontrol_year, now.year());
	regBank.set(adr_kontrol_hour, now.hour());
	regBank.set(adr_kontrol_minute, now.minute());
	regBank.set(adr_kontrol_second, now.second());
}
void file_print_date()  //���������  ������ ���� � ����
	{
	  DateTime now = RTC.now();
	  myFile.print(now.day(), DEC);
	  myFile.print('/');
	  myFile.print(now.month(), DEC);
	  myFile.print('/');
	  myFile.print(now.year(), DEC);//Serial display time
	  myFile.print(' ');
	  myFile.print(now.hour(), DEC);
	  myFile.print(':');
	  myFile.print(now.minute(), DEC);
	  myFile.print(':');
	  myFile.print(now.second(), DEC);
  }
void serial_print_date()                           // ������ ���� � �������    
{
	  DateTime now = RTC.now();
	  Serial.print(now.day(), DEC);
	  Serial.print('/');
	  Serial.print(now.month(), DEC);
	  Serial.print('/');
	  Serial.print(now.year(), DEC);//Serial display time
	  Serial.print(' ');
	  Serial.print(now.hour(), DEC);
	  Serial.print(':');
	  Serial.print(now.minute(), DEC);
	  Serial.print(':');
	  Serial.print(now.second(), DEC);
}
void resistor(int resist, int valresist)
{
	resistance = valresist;
	switch (resist)
	{
	case 1:
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word1));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	case 2:				
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word2));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	}
			//Wire.requestFrom(address_AD5252, 1, true);  // ������� ��������� ������ ��������� 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
}
void FileOpen()
{
	DateTime now = RTC.now();

			regBank.set(adr_Mic_Start_day , now.day());  // ����� ������ �����
			regBank.set(adr_Mic_Start_month, now.month());
			regBank.set(adr_Mic_Start_year, now.year());
			regBank.set(adr_Mic_Start_hour, now.hour());
			regBank.set(adr_Mic_Start_minute, now.minute());
			regBank.set(adr_Mic_Start_second, now.second());
				
			regBank.set(adr_Time_Test_day, 0); 
			regBank.set(adr_Time_Test_hour, 0); 
			regBank.set(adr_Time_Test_minute, 0); 
			regBank.set(adr_Time_Test_second, 0); 
	

	data_clock_exchange();                               // ��������� �� ���������� �� ����

	file_name_count = i2c_eeprom_read_byte(0x50, adr_file_name_count); // ������� ������� ����� ����� �� ������

	preob_num_str();                                     // ������������ ��� ����� �� ���� � �������� ������
 
//	Serial.println("***********");
	 if (SD.exists(file_name))                           // ��������� ���� �� ����� ����
	  { 

		Serial.print(file_name);
		Serial.println("  OK!.");
	  }
	  else 
	  {
		  Serial.println("");
		  Serial.print(file_name);
		  Serial.println(" doesn't exist.");             // ������ ����� ���
	  }

	  myFile = SD.open(file_name, FILE_WRITE);            // ������� ���� ��� ������ ������
	  myFile.println ("");
	  myFile.print ("Start test   ");
	  file_print_date();
	  myFile.println ("");

	//  UpdateRegs(); 
	  delay(100);
}
void FileClose()
{
	 myFile.println ("");
	 myFile.print ("Stop test   ");
	 file_print_date();
	 myFile.println ("");
	 myFile.close();
	 
	 file_name_count++;

	if(file_name_count > 99)
		{
			file_name_count = 0;
		}
	i2c_eeprom_write_byte(0x50, adr_file_name_count, file_name_count);//

	   Serial.print(file_name);
	   Serial.println(" close.");
	  delay(500);

	  if (SD.exists(file_name))
	  { 
		Serial.print(file_name);
		Serial.println("Close  OK!.");
	  }

	  else 
	  {
		  Serial.print(file_name);
		  Serial.println(" doesn't exist.");  
	  }
 
}

void control_command()
{
	/*
	��� ������ ������������ �������� ���������� �������� ����� �������� �� ������ adr_control_command (40120) 
	��� ��������
	0 -  ���������� ������� ��������
	1 -  ��������� ��� �������
	2 -  �������� ��� �������
	3 -  ���� �����������
	4 -  ���� ����������
	5 -  ���� ���
	6 -  ���� ����.�
	7 -  ���� ��������
	8 -  ���� ���
	9 -  ���� ����� 1
	10 - ���� ����� 2
	11 - ���� ����������
	12 - ������� ����
	13 - ������� ����
	14 - �������� �����


	*/
	UpdateRegs() ;

	int test_n = regBank.get(adr_control_command); //�����  40120

		
	switch (test_n)
	{
		case 1:
			// regBank.set(7,1);      // �������� ������� ��������
			 sence_all_off();                                // ��������� ��� �������
			break;
		case 2:				
			 sence_all_on();                                 // �������� ��� �������
				break;
		case 3:
			test_instruktora();
				break;
		case 4:				
			//test_dispetchera();          //
				break;
		case 5:
			// test_MTT();                  //
				break;
		case 6:				
			// test_tangR();                //
				break;
		case 7:
			// test_mikrophon();             // ������������ ���������
				break;
		case 8:				
			// testGGS();
				break;
		case 9:
			// test_GG_Radio1();
				break;
		case 10:				
			// test_GG_Radio2();
				break;
		case 11:				
			// test_mag();
				break;
		case 12:
			   FileOpen();
				break;
		case 13:				
			  FileClose();
				break;
		case 14:				
			  set_clock();
				break;
		default:
		break;

	 }
	regBank.set(adr_control_command,0);
}

void sence_all_off()
{
  regBank.set(7,1);      //�������� ������� ��������
  regBank.set(20,0);     // * XP2 Sence "���." 
  regBank.set(21,0);     // * XP5 Sence "��C."
  regBank.set(22,0);     // * XP3 Sence "��-�����1."  
  regBank.set(23,0);     // * XP4 Sence "��-�����2."  
  regBank.set(15,0);     // * XS1 6 Sence ����������� ���������

  regBank.set(12,0);     // * XP8 2 Sence �������� ������
  regBank.set(18,0);     // * XP7 2 Sence �������� ������
  regBank.set(19,0);     // XP7 1 PTT1 �������� ������
  regBank.set(16,0);     // XP7 4 PTT2 �������� ������
  regBank.set(17,0);     // XP1 - 20  HandUp


  regBank.set(31,0);     // * XP1- 1  HeS1Ls   Sence ����������� ��������� ����������
  regBank.set(30,0);     // * XP1- 5  HeS1Rs   Sence ����������� ��������� ���������� � 2 ����������
  regBank.set(29,0);     // XP1- 6  HeS1PTT 
  regBank.set(28,0);     // * XP1- 13 HeS2Ls   Sence ����������� ��������� �����������
  regBank.set(27,0);     // XP1- 15 HeS2PTT 

  regBank.set(26,0);     // * XP1- 16 HeS2Rs   Sence ����������� ��������� ����������� � 2 ����������
  regBank.set(25,0);     // XP1- 17 HaSPTT   
  regBank.set(24,1);     // XP1- 19 HaSs
  regBank.set(13,0);     // XP8 - 1  PTT 
  regBank.set(14,0);     // XS1 - 5   PTT  
  UpdateRegs(); 
  delay(100);

  
 //1 ��
	   if(regBank.get(30040) > 0)                  // ���� �������� ������ ������� Sence "���."   ok!
		  {
			int regcount = regBank.get(40126);     // 
			regcount++;
			regBank.set(40126,regcount);           // 
			regBank.set(126,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Sence Mag Error! - ");
			myFile.println(regcount);
		  }
//2 ��
	   if(regBank.get(30046) > 0)                   // XP5 Sence "��C."
		  {
			int regcount = regBank.get(40132);      // 
			regcount++;
			regBank.set(40132,regcount);            // 
			regBank.set(132,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off GGS  Error! - ");
			myFile.println(regcount);
		  }
//3	  �� 
	   if(regBank.get(30025) > 0)                   // XP3 Sence "��-�����1."  
		  {
			int regcount = regBank.get(40121);      // 
			regcount++;
			regBank.set(40121,regcount);            // 
			regBank.set(121,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off GG Radio1 Error! - ");
			myFile.println(regcount);
		  }
//4	��
	   if(regBank.get(30024) > 0)                   // XP3 Sence "��-�����2."  
		  {
			int regcount = regBank.get(40122);      // 
			regcount++;
			regBank.set(40122,regcount);            // 
			regBank.set(122,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off GG Radio2  Error! - ");
			myFile.println(regcount);
		  }
//5 ��
	   if(regBank.get(30026) > 0)                   // XS1 6 Sence ����������� ������
		  {
			int regcount = regBank.get(40123);      // 
			regcount++;
			regBank.set(40123,regcount);            // 
			regBank.set(123,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Trubka  Error! - ");
			myFile.println(regcount);
		  }
//6	���
	   if(regBank.get(30045) > 0)                   // ���� ���������
		  {
			int regcount = regBank.get(40131);      // ��������   ���� ����������� ���������
			regcount++;
			regBank.set(40131,regcount);            // 
			regBank.set(131,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Microphon  Error! - ");
			myFile.println(regcount);
		  }
//7	��  
	   if(regBank.get(30028) > 0)                   // Sence ���� ������ ok!
		  {
			int regcount = regBank.get(40125);      // 
			regcount++;
			regBank.set(40125,regcount);            // 
			regBank.set(125,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Tangenta n.  Error! - ");
			myFile.println(regcount);
		  }
//8	��
	   if(regBank.get(30027) > 0)                   // ��������  ���� ����������� ������ ��������
		  {
			int regcount = regBank.get(40124);      // 
			regcount++;
			regBank.set(40124,regcount);            // 
			regBank.set(124,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Tangenta ruch.  Error! - ");
			myFile.println(regcount);
		  }

//9	��
	   if(regBank.get(30042) > 0)                   // ���� ����������� ��������� �����������
		  {
			int regcount = regBank.get(40128);      // 
			regcount++;
			regBank.set(40128,regcount);            // 
			regBank.set(128,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura instruktora  Error! - ");
			myFile.println(regcount);
		  }

//10 ��
	   if(regBank.get(30041) > 0)                   //��������   ���� ����������� ��������� ����������� 2 ����������
		  {
			int regcount = regBank.get(40127);      // 
			regcount++;
			regBank.set(40127,regcount);            // 
			regBank.set(127,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura instruktora 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }
//11 ��
	   if(regBank.get(30044) > 0)                    //��������   ���� ����������� ��������� ����������
		  {
			int regcount = regBank.get(40130);       // 
			regcount++;
			regBank.set(40130,regcount);             // 
			regBank.set(130,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura dispetchera  Error! - ");
			myFile.println(regcount);
		  }
	 
//12 ��  
	   if(regBank.get(30043) > 0)                    //��������   ���� ����������� ��������� ���������� � 2 ����������
		  {
			int regcount = regBank.get(40129);       // 
			regcount++;
			regBank.set(40129,regcount);             // 
			regBank.set(129,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura dispetchera 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }



// * regBank.add(121);   // ���� �������� ������ ������� Sence "��-�����1."  ok!
// * regBank.add(40121); // ����� �������� ������ ������� Sence "��-�����1."  ok!
// * regBank.add(30025);  // ���� 1 ����� ��� 1 - ��������  ���� ����������� �� �����1


// * regBank.add(122);   // ���� �������� ������ ������� Sence "��-�����2."  ok!
// * regBank.add(40122); // ����� �������� ������ ������� Sence "��-�����2."  ok!
// * regBank.add(30024);  // ���� 1 ����� ��� 0 - ��������  ���� ����������� �� �����2

// * regBank.add(123);   // ���� �������� ������ ������� ����������� ������
// * regBank.add(40123); // ����� �������� ������ ������� ����������� ������
// * regBank.add(30026);  // ���� 1 ����� ��� 2 - ��������  ���� ����������� ������

// * regBank.add(124);   // ���� �������� ������ �������  Sence ����. �. ok!
// * regBank.add(40124); // ����� �������� ������ �������  Sence ����. �. ok!
// * regBank.add(30027);  // ���� 1 ����� ��� 3 - ��������  ���� ����������� ������ ��������

// * regBank.add(125);   // ���� �������� ������ ������� Sence ���� �. ok!
// * regBank.add(40125); // ����� �������� ������ ������� Sence ���� �. ok!
// * regBank.add(30028);  // ���� 1 ����� ��� 4 - ��������  ���� ����������� ������

// * regBank.add(126);   // ���� �������� ������ ������� Sence "���."   ok!
// * regBank.add(40126); // ����� �������� ������ ������� Sence "���."   ok!
// * egBank.add(30040);  // ���� 3 ����� ��� 0 - ��������   ���� ����������� �����������

// * regBank.add(127);   // ���� �������� ������ ������� ��������� ����������� � 2 ����������
// * regBank.add(40127); // ����� �������� ������ ������� ��������� ����������� � 2 ����������
// * regBank.add(30041);  // ���� 3 ����� ��� 1 - ��������   ���� ����������� ��������� ����������� 2 ����������

// * regBank.add(128);   // ���� �������� ������ ������� ��������� �����������
// * regBank.add(40128); // ����� �������� ������ ������� ��������� �����������
// * regBank.add(30042);  // ���� 3 ����� ��� 2 - ��������   ���� ����������� ��������� �����������

// * regBank.add(129);   // ���� �������� ������ ������� ��������� ���������� � 2 ����������
// * regBank.add(40129); // ����� �������� ������ ������� ��������� ���������� � 2 ����������
// * regBank.add(30043);  // ���� 3 ����� ��� 3 - ��������   ���� ����������� ��������� ���������� � 2 ����������


// * regBank.add(130);   // ���� �������� ������ ������� ��������� ����������
// * regBank.add(40130); // ����� �������� ������ ������� ��������� ����������
// * regBank.add(30044);  // ���� 3 ����� ��� 4 - ��������   ���� ����������� ��������� ����������


//regBank.add(131);   // ���� �������� ������ ������� Sence ���. ok!
//regBank.add(40131); // ����� �������� ������ ������� Sence ���. ok!
//regBank.add(30045);  // ���� 3 ����� ��� 5 - ��������   ���� ����������� ��������� XS1 - 6 Sence


// * regBank.add(132);   // ���� �������� ������ ������� Sence "��C."   ok!
// * regBank.add(40132); // ����� �������� ������ ������� Sence "��C."   ok!
// * regBank.add(30046);  // ���� 3 ����� ��� 6 - ��������   ���� ����������� ���


//regBank.add(133);   // ���� �������� ������ ������� 
//regBank.add(134);   // ���� �������� ������ ������� PTT ���� �.  ok!
//regBank.add(135);   // ���� �������� ������ ������� PTT ��� ok!

//regBank.add(136);   // ���� �������� ������  PTT2 ����. �. ok!
//regBank.add(137);   // ���� �������� ������  HangUp  DCD  ok!
//regBank.add(138);   // ���� �������� ������  PTT1 ����. �. ok!



//regBank.add(40133); // ����� �������� ������ ������� 
//regBank.add(40134); // ����� �������� ������ ������� PTT ���� �.  ok!
//regBank.add(40135); // ����� �������� ������ ������� PTT ��� ok!
//
//regBank.add(40136); // ����� �������� ������  PTT2 ����. �. ok!
//regBank.add(40137); // ����� �������� ������  HangUp  DCD  ok!
//regBank.add(40138); // ����� �������� ������  PTT1 ����. �. ok!
//regBank.add(40139); // ����� �������� ������ 
//regBank.add(40140); // ����� �������� ������ 

//regBank.add(30052);  // ���� 4 ����� ��� 4 - ��������    ���� ���������� ��� (Mute)
//regBank.add(30053);  // ���� 4 ����� ��� 5 - ��������    ���� �������������
//regBank.add(30054);  // ���� 4 ����� ��� 6 - ��������    ���� ���������� ����������� ��������
 



   delay(100);
 //  Serial.println("All sensors OFF!.");
}
void sence_all_on()
{
  regBank.set(20,1);     //XP2 "���." 
  regBank.set(21,1);     //XP5 Sence "��C."
  regBank.set(22,1);     //XP3 Sence "��-�����1."  
  regBank.set(23,1);     //XP4 Sence "��-�����2."  
  regBank.set(15,1);     // XS1 6 Sence 
  regBank.set(12,1);     // XP8 2 Sence
  regBank.set(18,1);     // XP7 2 Sence
  regBank.set(19,1);     // XP7 1 PTT1
  regBank.set(16,1);     // XP7 4 PTT2
  regBank.set(17,1);     // XP1 - 20  HandUp
 
  regBank.set(31,1);     // XP1- 1  HeS1Ls 
  regBank.set(30,1);     // XP1- 5  HeS1Rs
  regBank.set(29,1);     // XP1- 6  HeS1PTT 
  regBank.set(28,1);     // XP1- 13 HeS2Ls
  regBank.set(27,1);     // XP1- 15 HeS2PTT

  regBank.set(26,1);     // XP1- 16 HeS2Rs
  regBank.set(25,1);     // XP1- 17 HaSPTT
  regBank.set(24,0);     // XP1- 19 HaSs
  regBank.set(13,1);     // XP8 - 1  PTT 
  regBank.set(14,1);     // XS1 - 5   PTT 
  UpdateRegs(); 
  delay(100);

 //1 ��
	   if(regBank.get(30040) == 0)                  // ���� �������� ������ ������� Sence "���."   ok!
		  {
			int regcount = regBank.get(40126);     // 
			regcount++;
			regBank.set(40126,regcount);           // 
			regBank.set(126,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Sence Mag Error! - ");
			myFile.println(regcount);
		  }
//2 ��
	   if(regBank.get(30046) == 0)                   // XP5 Sence "��C."
		  {
			int regcount = regBank.get(40132);      // 
			regcount++;
			regBank.set(40132,regcount);            // 
			regBank.set(132,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GGS  Error! - ");
			myFile.println(regcount);
		  }
//3	  �� 
	   if(regBank.get(30025) == 0)                   // XP3 Sence "��-�����1."  
		  {
			int regcount = regBank.get(40121);      // 
			regcount++;
			regBank.set(40121,regcount);            // 
			regBank.set(121,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GG Radio1 Error! - ");
			myFile.println(regcount);
		  }
//4	��
	   if(regBank.get(30024) == 0)                   // XP3 Sence "��-�����2."  
		  {
			int regcount = regBank.get(40122);      // 
			regcount++;
			regBank.set(40122,regcount);            // 
			regBank.set(122,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GG Radio2  Error! - ");
			myFile.println(regcount);
		  }
//5 ��
	   if(regBank.get(30026) == 0)                   // XS1 6 Sence ����������� ������
		  {
			int regcount = regBank.get(40123);      // 
			regcount++;
			regBank.set(40123,regcount);            // 
			regBank.set(123,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Trubka  Error! - ");
			myFile.println(regcount);
		  }
//6	���
	   if(regBank.get(30045) == 0)                   // ���� ���������
		  {
			int regcount = regBank.get(40131);      // ��������   ���� ����������� ���������
			regcount++;
			regBank.set(40131,regcount);            // 
			regBank.set(131,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Microphon  Error! - ");
			myFile.println(regcount);
		  }
//7	��  
	   if(regBank.get(30028) == 0)                   // Sence ���� ������ ok!
		  {
			int regcount = regBank.get(40125);      // 
			regcount++;
			regBank.set(40125,regcount);            // 
			regBank.set(125,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Tangenta n.  Error! - ");
			myFile.println(regcount);
		  }
//8	��
	   if(regBank.get(30027) == 0)                   // ��������  ���� ����������� ������ ��������
		  {
			int regcount = regBank.get(40124);      // 
			regcount++;
			regBank.set(40124,regcount);            // 
			regBank.set(124,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Tangenta ruch.  Error! - ");
			myFile.println(regcount);
		  }

//9	��
	   if(regBank.get(30042) == 0)                   // ���� ����������� ��������� �����������
		  {
			int regcount = regBank.get(40128);      // 
			regcount++;
			regBank.set(40128,regcount);            // 
			regBank.set(128,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura instruktora  Error! - ");
			myFile.println(regcount);
		  }

//10 ��
	   if(regBank.get(30041) == 0)                   //��������   ���� ����������� ��������� ����������� 2 ����������
		  {
			int regcount = regBank.get(40127);      // 
			regcount++;
			regBank.set(40127,regcount);            // 
			regBank.set(127,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura instruktora 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }
//11 ��
	   if(regBank.get(30044) == 0)                    //��������   ���� ����������� ��������� ����������
		  {
			int regcount = regBank.get(40130);       // 
			regcount++;
			regBank.set(40130,regcount);             // 
			regBank.set(130,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura dispetchera  Error! - ");
			myFile.println(regcount);
		  }
	 
//12 ��  
	   if(regBank.get(30043) == 0)                    //��������   ���� ����������� ��������� ���������� � 2 ����������
		  {
			int regcount = regBank.get(40129);       // 
			regcount++;
			regBank.set(40129,regcount);             // 
			regBank.set(129,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura dispetchera 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }




 // Serial.println("All sensors ON!.");
}
void test_instruktora()
{
	  myFile.println("");
	  myFile.println("Test_instruktora start!");
	  myFile.println("Komanda sensor Off SpkLout  send!");
	  regBank.set(29,0);                                    // XP1- 13 HeS2Ls  ��������� ������ �����������
	  myFile.println("Komanda sensor Off SpkRout  send!");
	  regBank.set(27,0);                                    // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	  UpdateRegs(); 
	  delay(600);
	  UpdateRegs(); 
	  delay(600);
	  byte i5 = regs_in[2];

		if(bitRead(i5,1) > 0)                              // ��������  ����� ����������� ��������� ����������� 2 ����������
		  {
			int regcount = regBank.get(40127);              // ����� �������� ������ ������� ��������� ����������� � 2 ����������
			regcount++;                                     // ��������� ������� ������
			regBank.set(40127,regcount);                    // ����� �������� ������ ������� ��������� ����������� � 2 ����������
			regBank.set(127,1);                             // ���������� ���� ������
			regBank.set(120,1);                             // ���������� ����� ���� ������
			resistor(1, 255);                               // ���������� ������� �������
			resistor(2, 255);                               // ���������� ������� �������
			myFile.print("Komanda sensor Off SpkLout  Error! - ");
			myFile.println(regcount);
		  }
		else
		  {
			myFile.println("Komanda sensor Off SpkLout  Ok!");
		  }

		if(bitRead(i5,2) > 0)                                // ��������  ����� ����������� ��������� ����������� 
		  {
			 int regcount = regBank.get(40128);              // ����� �������� ������ ������� ��������� �����������
			 regcount++;                                     // ��������� ������� ������
			 regBank.set(40128,regcount);                    // ����� �������� ������ ������� ��������� �����������
			 regBank.set(128,1);                             // ���������� ���� ������
			 regBank.set(120,1);                             // ���������� ����� ���� ������
			 myFile.print("Komanda sensor Off SpkRout  Error! - ");
			 myFile.println(regcount);
		  }
		else
		  {
			 myFile.println("Komanda sensor Off SpkRout  Ok!");
		  }

		  myFile.println("Komanda sensor On  SpkLout  send!");
		  regBank.set(29,1);                             // XP1- 13 HeS2Ls
		  myFile.println("Komanda sensor On  SpkRout  send!");
		  regBank.set(27,1);                             // XP1- 16 HeS2Rs
		  UpdateRegs(); 
		  delay(600);
		  UpdateRegs(); 
		  delay(600);

	 	 if(bitRead(i5,1) > 0)                       // �������� ����� ����������� ��������� ����������� 2 ����������
			{
				int regcount = regBank.get(40127);   // ����� �������� ������ ������� ��������� ����������� � 2 ����������
				regcount++;                          // ��������� ������� ������
				regBank.set(40127,regcount);         // ����� �������� ������ ������� ��������� ����������� � 2 ����������
				regBank.set(127,1);                  // ���������� ���� ������
				regBank.set(120,1);                  // ���������� ����� ���� ������
				myFile.print("Komanda sensor On  SpkLout  Error! - ");
				myFile.println(regcount);
			}
		else
			{
				myFile.println("Komanda sensor On  SpkLout  Ok!");
			}
		 	if(bitRead(i5,2) > 0)                     // �������� ����� ����������� ��������� �����������
	         {
				int regcount = regBank.get(40128);    // ����� �������� ������ ������� ��������� �����������
				regcount++;                           // ��������� ������� ������
				regBank.set(40128,regcount);          // ����� �������� ������ ������� ��������� �����������
				regBank.set(128,1);                   // ���������� ���� ������
				regBank.set(120,1);                   // ���������� ����� ���� ������
				myFile.print("Komanda sensor On  SpkRout  Error! - ");
				myFile.println(regcount);
			 }
		else
			{
				myFile.println("Komanda sensor On  SpkRout  Ok!");
			}
			Serial.println(regBank.get(127));
			Serial.println(regBank.get(40127));
			Serial.println(regBank.get(128));
			Serial.println(regBank.get(40128));



/*
		 regBank.set(4,0);                                      // XP1- 12 HeS2e  ��������� ���� ��  �������� �����������
		 myFile.println("Komanda microphon instr. Off send!");
		 regBank.set(28,0);                                     // XP1- 15 HeS2Ls ��������� PTT �����������
		 myFile.println("Komanda PTT instr. Off  send!");
		 regBank.set(5,1);                                      // ������ ������ �� �������� ��������� �����������
		 UpdateRegs(); 
		 delay(500);
		 UpdateRegs(); 
		 delay(100);
		i5 = regs_in[3];

		if(bitRead(i5,4)!= 0) 		                            // ��������� ���������� ��������� �����������
			{
				int regcount = regBank.get(40139);              // ����� �������� ������ ���������� ��������� ��������� �����������
				regcount++;                                     // ��������� ������� ������
				regBank.set(40139,regcount);                    // ����� �������� ������ ���������� ��������� ��������� �����������
				regBank.set(139,1);                             // ���������� ���� ������ ���������� ��������� ��������� �����������
				regBank.set(120,1);                             // ���������� ����� ���� ������
				myFile.print("Komanda microphon instrukt. Off Error! - ");
				myFile.println(regcount);
				Serial.println(regBank.get(120));
			}
		if(regBank.get(10080)!= 0)                  // ��������� ���������� PTT �����������
			{
				int regcount = regBank.get(40140);    // ����� �������� ������ ���������� PTT ��������� �����������
				regcount++;                           // ��������� ������� ������
				regBank.set(40140,regcount);          // ����� �������� ������ ���������� PTT ��������� �����������
				regBank.set(140,1);                   // ���������� ���� ������ ���������� PTT ��������� �����������
				regBank.set(120,1);                   // ���������� ����� ���� ������
				myFile.print("Komanda PTT instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		delay(1000);

		  // ��������� ����������� ������ ���������
		measure_volume(analog_FrontL);

		if(voltage10 < volume_porog_D)                //volume_porog_D ��������� ����������� ������ ��������� �����������
			{
				int regcount = regBank.get(40141);    // ����� �������� ������ ������ ���������  �����������
				regcount++;                           // ��������� ������� ������ ������ ��������� 
				regBank.set(40141,regcount);          // ����� �������� ������ ������ ���������  �����������
				regBank.set(141,1);                   // ���������� ���� ������  ������ ���������  �����������
				regBank.set(120,1);                   // ���������� ����� ���� ������ 
				myFile.print("Komanda FrontL instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		measure_volume(analog_FrontR);

		if(voltage10 < volume_porog_D)                  // ��������� ����������� ������ ���������  �����������
			{
				int regcount = regBank.get(40142);      // ����� �������� ������ ������ ���������  ��������� �����������
				regcount++;                             // ��������� ������� ������
				regBank.set(40142,regcount);            // ����� �������� ������ ������ ���������  ��������� �����������
				regBank.set(142,1);                     // ���������� ���� ������ ������ ���������  ��������� �����������
				regBank.set(120,1);                     // ���������� ����� ���� ������
				myFile.print("Komanda FrontR instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		*/
		
		delay(100);
		regBank.set(adr_control_command,0);
}

void measure_volume(int analog)
{
		volume1     = 0;
		volume_max  = 0;
		volume_min  = 1023;
		volume_fact = 0;
		int i;
		int i_stop = 255;
		
		for (i = 0;i<= i_stop; i++)
			{
				Array_volume[i] = analogRead(analog);               // ��������� ��������
			}
		for (i = 0; i<= i_stop; i++)
			{
				volume_max = max(volume_max, Array_volume[i]);
				volume_min = min(volume_min, Array_volume[i]);
			}
		
		volume_fact = volume_max - volume_min;
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;

		/* Serial.print("voltage - ");
		Serial.println(voltage10);*/

		//Serial.print(volume_min*(5.0 / 1023.0) );
		//Serial.print("  max - ");
		//Serial.print(volume_max*(5.0 / 1023.0));
		//Serial.print("  fact - ");
		//Serial.println(volume_fact*(5.0 / 1023.0));

		//Serial.print("  *** - ");
		//  for (int i=0;i<= i_stop; i++)
		//   {
		//	   Serial.println(Array_volume[i]*(5.0 / 1023.0));
		//   }
}
void preob_num_str() // ��������� ������������ ����� �����, ���������� �� ������� ���� � �������� ������
{
	DateTime now = RTC.now();
	int year_temp = now.year()-2000;

	itoa (year_temp,str_year_file, 10); // �������������� ���� ��� � ������ ( 10 - ���������� ������) 

	
	if (now.month() <10)
		{
		   itoa (0,str_mon_file0, 10);                   //  �������������� ���� �����  � ������ ( 10 - ���������� ������) 
		   itoa (now.month(),str_mon_file10, 10);        //  �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);  // �������� 2 �����
		}
	else
		{
		   itoa (now.month(),str_mon_file, 10);         // �������������� ����� � ������ ( 10 - ���������� ������) 
		}


	if (now.day() <10)
		{
		   itoa (0,str_day_file0, 10);                  // �������������� ����� � ������ ( 10 - ���������� ������) 
		   itoa (now.day(),str_day_file10, 10);         // �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);  // �������� 2 �����
		}
	else
		{
		itoa (now.day(),str_day_file, 10);                   // �������������� ����� � ������ ( 10 - ���������� ������) 
		}

		 
	if (file_name_count<10)
		{
			itoa (file_name_count,str0, 10);                 // �������������� ����� � ������ ( 10 - ���������� ������) 
			sprintf(str_file_name_count, "%s%s", str_file_name_count0, str0);  // �������� 2 �����
		}
	
	else
		{
			itoa (file_name_count,str_file_name_count, 10);  // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
	sprintf(str1, "%s%s",str_year_file, str_mon_file);       // �������� 2 �����
	sprintf(str2, "%s%s",str1, str_day_file);                // �������� 2 �����
	sprintf(str3, "%s%s", str2, str_file_name_count);        // �������� 2 �����
	sprintf(file_name, "%s%s", str3, file_name_txt);         // ��������� ����� ����� � file_name

}

void setup_mcp()
{
	// ��������� ����������� ������

 // mcp_Klava.begin(2);             // ����� (2)������� ����������� ������ ���������� � ����������
  mcp_Klava.begin(2);             // ����� (2)������� ����������� ������ ���������� � ����������
  mcp_Klava.pinMode(8, OUTPUT);   // ��������� �1
  mcp_Klava.pinMode(9, OUTPUT);   // ��������� �2
  mcp_Klava.pinMode(10, OUTPUT);  // ��������� �3
  mcp_Klava.pinMode(11, OUTPUT);  // ��������� �4
  mcp_Klava.pinMode(12, OUTPUT);  // ��������� �5
  mcp_Klava.pinMode(13, OUTPUT);  // ��������� �6
  mcp_Klava.pinMode(14, OUTPUT);  // ��������� �7
  mcp_Klava.pinMode(15, OUTPUT);  // ��������� �8
 
 
  mcp_Klava.pinMode(0, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(0, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(1, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(1, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(2, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(2, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(3, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(3, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(4, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(4, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(5, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(5, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(6, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(6, HIGH);      // ���������� ���������� �������� 100K � 5�.
  mcp_Klava.pinMode(7, INPUT);    // ����������� ������ �� ����
  mcp_Klava.pullUp(7, HIGH);      // ���������� ���������� �������� 100K � 5�.
  
 
  mcp_Out1.begin(4);              //  ����� (4) �������  ����������� ������
  mcp_Out1.pinMode(0, OUTPUT);    // ���� �0  ����    
  mcp_Out1.pinMode(1, OUTPUT);    // ���� �1  ����    
  mcp_Out1.pinMode(2, OUTPUT);    // ���� �2  ����    
  mcp_Out1.pinMode(3, OUTPUT);    // ���� �3  ����    
  mcp_Out1.pinMode(4, OUTPUT);    // ���� �4  ����   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // ���� �5  ����    
  mcp_Out1.pinMode(6, OUTPUT);    // ���� �6  ����   
  mcp_Out1.pinMode(7, OUTPUT);    // ���� �7  �������� +12�����  ������� ����� ��������
  
  mcp_Out1.pinMode(8, OUTPUT);    //  ���� �8 ���� �� �������� ����.  
  mcp_Out1.pinMode(9, OUTPUT);    // �������� J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // �������� J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // �������� J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  Sence     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 Sence      

	
  mcp_Out2.begin(6);              //  ����� (6) �������  ����������� ������
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 Sence
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    Sence "���."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    Sence "��C." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    Sence "��-�����1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    Sence "��-�����2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  ����� (5)  ����������� ������ 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 ��������
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 ��������
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 ��������
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 ��������
  mcp_Analog.pullUp(0, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(1, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 �������� �������� ����������� ������ �� ���� 
  mcp_Analog.pullUp(2, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 �������� ����������� ������ �� ����
  mcp_Analog.pullUp(3, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(4, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(5, INPUT);   //CTS ����������� ������ �� ����
  mcp_Analog.pullUp(5, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(6, INPUT);   // DSR ����������� ������ �� ����
  mcp_Analog.pullUp(6, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(7, INPUT);   //  DCD ����������� ������ �� ����
  mcp_Analog.pullUp(7, HIGH);     // ���������� ���������� �������� 100K � 5�.

}
void setup_resistor()
{ 
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word1));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word2));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
}
void setup_SD() 
{


  Serial.println("\nInitializing SD card...");
  pinMode(SD_Select, OUTPUT);     // change this to 53 on a mega


  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, SD_Select)) 
	  {
		Serial.println("initialization failed. Things to check:");
		Serial.println("* is a card is inserted?");
		Serial.println("* Is your wiring correct?");
		Serial.println("* did you change the SD_Select pin to match your shield or module?");
		return;
	  }
  else 
	  {
	   Serial.println("Wiring is correct and a card is present."); 
	  }

  // print the type of card
  Serial.print("Card type: ");
  switch(card.type()) {
	case SD_CARD_TYPE_SD1:
	  Serial.println("SD1");
	  break;
	case SD_CARD_TYPE_SD2:
	  Serial.println("SD2");
	  break;
	case SD_CARD_TYPE_SDHC:
	  Serial.println("SDHC");
	  break;
	default:
	  Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
	Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
	return;
  }
   


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is FAT");
  Serial.println(volume.fatType(), DEC);
 // Serial.println();
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
 // Serial3.print("Volume size (bytes): ");
 // Serial3.println(volumesize);
 // Serial3.print("Volume size (Kbytes): ");
  volumesize /= 1024;
 //Serial3.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);

  
  Serial.println("Files found on the card (name, date and size in bytes): ");
  root.openRoot(volume);
  
  // list all files in the card with date and size
  root.ls (LS_R | LS_DATE | LS_SIZE);


}
void setup_SDFile()
{
	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	myFile = SD.open("test.txt", FILE_WRITE);

	// if the file opened okay, write to it:
	if (myFile) 
		{
			Serial.print("Writing to test.txt...");
			myFile.println("testing 1, 2, 3.");
			// close the file:
			myFile.close();
			Serial.println("done.");
		}
	else 
		{
			// if the file didn't open, print an error:
			Serial.println("error opening test.txt");
		}

	// re-open the file for reading:
	myFile = SD.open("test.txt");
	if (myFile) 
		{
			Serial.println("test.txt:");

			// read from the file until there's nothing else in it:
			while (myFile.available()) 
				{
					Serial.write(myFile.read());
				}
			// close the file:
			myFile.close();
		}
	else 
		{
			// if the file didn't open, print an error:
			Serial.println("error opening test.txt");
		}
}
void setup_regModbus()
{

/*
��������� ������ Modbus ���������� ���������� ���������
��� ��, ��� ���������� ��������� ����� ��������, ����� ������ � ������
������������������ ������. � ��������� ����� �������� Modbus Slave ��������� �����
������� ������ ���� ���������� ����������� �� ����.
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Assign the modbus device ID.  
  regBank.setId(1);               // Slave ID 1

/*
modbus registers follow the following format
00001-09999  Digital Outputs, A master device can read and write to these registers
10001-19999  Digital Inputs,  A master device can only read the values from these registers
30001-39999  Analog Inputs,   A master device can only read the values from these registers
40001-49999  Analog Outputs,  A master device can read and write to these registers 
����� �����, ����� ��������� �������� ��� ���� � ������� ������. ���
������������ ����� ����������� ����� � �������� � ��������� ���������� ���������
��������� ������� ��� ���������� ������.
*/
	regBank.add(1);     // ���� RL0 ����
	regBank.add(2);     // ���� RL1 ����
	regBank.add(3);     // ���� RL2 ����
	regBank.add(4);     // ���� RL3 ����
	regBank.add(5);     // ���� RL4 XP1 12  HeS2e 
	regBank.add(6);     // ���� RL5 ����
	regBank.add(7);     // ���� RL6 ����
	regBank.add(8);     // ���� RL7 ������� �����
  
	regBank.add(9);     // ���� RL8 ���� �� ��������
	regBank.add(10);     // ���� RL9 XP1 10
	regBank.add(11);    // �������� J24 - 2 
	regBank.add(12);    // �������� J24 - 1 
	regBank.add(13);    // XP8 - 2   Sence 
	regBank.add(14);    // XP8 - 1   PTT 
	regBank.add(15);    // XS1 - 5   PTT ���
	regBank.add(16);    // XS1 - 6   Sence ���
 
	regBank.add(17);    // J8-12     XP7 4 PTT2   ����. �.
	regBank.add(18);    // XP1 - 20  HangUp  DCD
	regBank.add(19);    // J8-11     XP7 2 Sence  ����. �.
	regBank.add(20);    // J8-23     XP7 1 PTT1 ����. �.
	regBank.add(21);    // XP2-2     Sence "���."  
	regBank.add(22);    // XP5-3     Sence "��C."
	regBank.add(23);    // XP3-3     Sence "��-�����1."
	regBank.add(24);    // XP4-3     Sence "��-�����2."
 
	regBank.add(25);    // XP1- 19 HaSs      Sence ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.add(27);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS ���
	regBank.add(29);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS ���
	regBank.add(31);    // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.add(32);    // XP1- 1  HeS1Ls    Sence ���������� ��������� ����������

	regBank.add(10081);    // ����� ����a ��������� ��������� ������� CTS
	regBank.add(10082);    // ����� ����a ��������� ��������� ������� DSR
	regBank.add(10083);    // ����� ����a ��������� ��������� ������� DCD

	regBank.add(120);   // ���� ��������� ������������� ����� ������
 
	regBank.add(121);   // ���� �������� ������ ������� Sence "��-�����1."  ok!
	regBank.add(122);   // ���� �������� ������ ������� Sence "��-�����2."  ok!
	regBank.add(123);   // ���� �������� ������ ������� ����������� ������
	regBank.add(124);   // ���� �������� ������ �������  Sence ����. �. ok!
	regBank.add(125);   // ���� �������� ������ ������� Sence ���� �. ok!
	regBank.add(126);   // ���� �������� ������ ������� Sence "���."   ok!
	regBank.add(127);   // ���� �������� ������ ������� ��������� ����������� � 2 ����������
	regBank.add(128);   // ���� �������� ������ ������� ��������� �����������
	regBank.add(129);   // ���� �������� ������ ������� ��������� ���������� � 2 ����������
	regBank.add(130);   // ���� �������� ������ ������� ��������� ����������
	regBank.add(131);   // ���� �������� ������ ������� Sence ���. ok!
	regBank.add(132);   // ���� �������� ������ ������� Sence "��C."   ok!

	regBank.add(133);   // ���� �������� ������ ������� 
	regBank.add(134);   // ���� �������� ������ ������� PTT ���� �.  ok!
	regBank.add(135);   // ���� �������� ������ ������� PTT ��� ok!

	regBank.add(136);   // ���� �������� ������  PTT2 ����. �. ok!
	regBank.add(137);   // ���� �������� ������  HangUp  DCD  ok!
	regBank.add(138);   // ���� �������� ������  PTT1 ����. �. ok!
	regBank.add(139);   // ���� ������ ���������� ��������� ��������� �����������
	regBank.add(140);   // ���� ������ ���������� PTT ��������� ����������� 
	regBank.add(141);   // ���� ������ �������� ��������� ����������� FrontL
	regBank.add(142);   // ���� ������ �������� ��������� ����������� FrontR

						 //Add Input registers 30001-30040 to the register bank

	regBank.add(30000);  // ���� 0 ���� ��� 0 - ��������   ��� "D"
	regBank.add(30001);  // ���� 0 ���� ��� 1 - ��������   "1"
	regBank.add(30002);  // ���� 0 ���� ��� 2 - ��������   "0"  
	regBank.add(30003);  // ���� 0 ���� ��� 3 - ��������   "1"
	regBank.add(30004);  // ���� 0 ���� ��� 4 - ��������   "0" ����� ���� ��������� (2)
	regBank.add(30005);  // ���� 0 ���� ��� 5 - ��������   "1" ����� ���� ��������� (2)
	regBank.add(30006);  // ���� 0 ���� ��� 6 - ��������   "0" ����� ���� ��������� (2)
	regBank.add(30007);  // ���� 0 ���� ��� 7 - ��������   "0"
  
	regBank.add(30008);  // ���� 1 ���� ��� 0 - ��������   CRC0
	regBank.add(30009);  // ���� 1 ���� ��� 1 - ��������   CRC1
	regBank.add(30010);  // ���� 1 ���� ��� 2 - ��������   CRC2
	regBank.add(30011);  // ���� 1 ���� ��� 3 - ��������   CRC3
	regBank.add(30012);  // ���� 1 ���� ��� 4 - ��������   ���������� ��� (Mute)
	regBank.add(30013);  // ���� 1 ���� ��� 5 - ��������   �������������
	regBank.add(30014);  // ���� 1 ���� ��� 6 - ��������   ������/ ������ ���� �������
	regBank.add(30015);  // ���� 1 ���� ��� 7 - ��������   "1"
  
	regBank.add(30016);  // ���� 2 ���� ��� 0 - ��������   ��� ������� ������
	regBank.add(30017);  // ���� 2 ���� ��� 1 - ��������   ��� ������� ������
	regBank.add(30018);  // ���� 2 ���� ��� 2 - ��������   ��� ������� ������
	regBank.add(30019);  // ���� 2 ���� ��� 3 - ��������   ��� ������� ������
	regBank.add(30020);  // ���� 2 ���� ��� 4 - ��������   ��� ������� ������
	regBank.add(30021);  // ���� 2 ���� ��� 5 - ��������   ��� ������� ������
	regBank.add(30022);  // ���� 2 ���� ��� 6 - ��������   ��� ������� ������
	regBank.add(30023);  // ���� 2 ���� ��� 7 - ��������   "0" 

						 // ���� ������  ���������� ��  ����� ��������. �������� 4 �����
  
	regBank.add(30024);  // ���� 1 ����� ��� 0 - ��������  ���� ����������� �� �����2
	regBank.add(30025);  // ���� 1 ����� ��� 1 - ��������  ���� ����������� �� �����1
	regBank.add(30026);  // ���� 1 ����� ��� 2 - ��������  ���� ����������� ������
	regBank.add(30027);  // ���� 1 ����� ��� 3 - ��������  ���� ����������� ������ ��������
	regBank.add(30028);  // ���� 1 ����� ��� 4 - ��������  ���� ����������� ������
	regBank.add(30029);  // ���� 1 ����� ��� 5 - ��������   "1"
	regBank.add(30030);  // ���� 1 ����� ��� 6 - ��������   "0" 
	regBank.add(30031);  // ���� 1 ����� ��� 7 - ��������   "1"
  
	regBank.add(30032);  // ���� 2 ����� ��� 0 - ��������   ��� ������� ������
	regBank.add(30033);  // ���� 2 ����� ��� 1 - ��������   ��� ������� ������
	regBank.add(30034);  // ���� 2 ����� ��� 2 - ��������   ��� ������� ������
	regBank.add(30035);  // ���� 2 ����� ��� 3 - ��������   ��� ������� ������
	regBank.add(30036);  // ���� 2 ����� ��� 4 - ��������   ��� ������� ������
	regBank.add(30037);  // ���� 2 ����� ��� 5 - ��������   ��� ������� ������
	regBank.add(30038);  // ���� 2 ����� ��� 6 - ��������   ��� ������� ������
	regBank.add(30039);  // ���� 2 ����� ��� 7 - ��������   "0" 
  
	regBank.add(30040);  // ���� 3 ����� ��� 0 - ��������   ���� ����������� �����������
	regBank.add(30041);  // ���� 3 ����� ��� 1 - ��������   ���� ����������� ��������� ����������� 2 ����������
	regBank.add(30042);  // ���� 3 ����� ��� 2 - ��������   ���� ����������� ��������� �����������
	regBank.add(30043);  // ���� 3 ����� ��� 3 - ��������   ���� ����������� ��������� ���������� � 2 ����������
	regBank.add(30044);  // ���� 3 ����� ��� 4 - ��������   ���� ����������� ��������� ����������
	regBank.add(30045);  // ���� 3 ����� ��� 5 - ��������   ���� ����������� ��������� XS1 - 6 Sence
	regBank.add(30046);  // ���� 3 ����� ��� 6 - ��������   ���� ����������� ���
	regBank.add(30047);  // ���� 3 ����� ��� 7 - ��������   "0" 
  
	regBank.add(30048);  // ���� 4 ����� ��� 0 - ��������   CRC0
	regBank.add(30049);  // ���� 4 ����� ��� 1 - ��������   CRC1
	regBank.add(30050);  // ���� 4 ����� ��� 2 - ��������   CRC2   
	regBank.add(30051);  // ���� 4 ����� ��� 3 - ��������   CRC3   
	regBank.add(30052);  // ���� 4 ����� ��� 4 - ��������   ���� ���������� ��������� �����������
	regBank.add(30053);  // ���� 4 ����� ��� 5 - ��������    ���� �������������
	regBank.add(30054);  // ���� 4 ����� ��� 6 - ��������   ���� ���������� ��������� ����������
	regBank.add(30055);  // ���� 4 ����� ��� 7 - ��������   "0" 



	regBank.add(40000);  // 
	regBank.add(40001);  // 
	regBank.add(40002);  // 
	regBank.add(40003);  // 
	regBank.add(40004);  // 
	regBank.add(40005);  // 
	regBank.add(40006);  // 
	regBank.add(40007);  // 
 // 
	//regBank.add(40008);  // 
	//regBank.add(40009);  // 
	//regBank.add(40010);  // 
	//regBank.add(40011);  // 
	//regBank.add(40012);  // 
	//regBank.add(40013);  // 
	//regBank.add(40014);  // 
	//regBank.add(40015);  // 
	//regBank.add(40016);  // 


						 // ������� ����� 
	regBank.add(40046);  // ����� ���� ������ ����� �����������
	regBank.add(40047);  // ����� ����� ������ ����� �����������
	regBank.add(40048);  // ����� ��� ������ ����� �����������
	regBank.add(40049);  // ����� ��� ������ ����� �����������
	regBank.add(40050);  // ����� ������ ������ ����� �����������
	regBank.add(40051);  // ����� ������� ������ ����� �����������
 
						 // ��������� ������� � �����������
	regBank.add(40052);  // ����� ����
	regBank.add(40053);  // ����� �����
	regBank.add(40054);  // ����� ���
	regBank.add(40055);  // ����� ���
	regBank.add(40056);  // ����� ������
	regBank.add(40057);  // 
	regBank.add(40058);  // 
	regBank.add(40059);  // 
	/*
	regBank.add(40060); // ����� �������� ������
	regBank.add(40061); // ����� �������� ������
	regBank.add(40062); // ����� �������� ������
	regBank.add(40063); // ����� �������� ������
	regBank.add(40064); // ����� ������
	regBank.add(40065); // ����� ������
	regBank.add(40066); // ����� ������
	regBank.add(40067); // ����� ������
	regBank.add(40068); // ����� ������
	regBank.add(40069); // ����� ������
	regBank.add(40070); // ����� ������
	regBank.add(40071); // ����� ������

	regBank.add(40072); // ����� ������ � %
	regBank.add(40073); // ����� ������ � %
	regBank.add(40074); // ����� ������ � %
	regBank.add(40075); // ����� ������ %
	regBank.add(40076); // ����� ������ %
	regBank.add(40077); // ����� ������ %
	regBank.add(40078); // ����� ������ %
	regBank.add(40079); // ����� ������ %
	regBank.add(40080); // ����� ������ %
	regBank.add(40081); // ����� ������ %
	regBank.add(40082); // ����� ������ %
	regBank.add(40083); // ����� ������ %

	// ����� ������ �� ���������
	regBank.add(40084); // ����� ���� adr_Mic_On_day 
	regBank.add(40085); // ����� ����� adr_Mic_On_month  
	regBank.add(40086); // ����� ��� adr_Mic_On_year  
	regBank.add(40087); // ����� ��� adr_Mic_On_hour 
	regBank.add(40088); // ����� ������ adr_Mic_On_minute 
	regBank.add(40089); // ����� �������  adr_Mic_On_second    

	// ����� ������ �� ����������
	regBank.add(40090); // ����� ���� adr_Mic_Off_day    
	regBank.add(40091); // ����� �����  adr_Mic_Off_month 
	regBank.add(40092); // ����� ��� adr_Mic_Off_year  
	regBank.add(40093); // ����� ��� adr_Mic_Off_hour   
	regBank.add(40094); // ����� ������ adr_Mic_Off_minute   
	regBank.add(40095); // ����� ������� adr_Mic_Off_second    
	*/
	// ����� ������ �����
	regBank.add(40096); // ����� ����  adr_Mic_Start_day    
	regBank.add(40097); // ����� ����� adr_Mic_Start_month  
	regBank.add(40098); // ����� ��� adr_Mic_Start_year  
	regBank.add(40099); // ����� ��� adr_Mic_Start_hour 
	regBank.add(40100); // ����� ������ adr_Mic_Start_minute 
	regBank.add(40101); // ����� ������� adr_Mic_Start_second  

	// ����� ��������� �����
	regBank.add(40102); // ����� ���� adr_Mic_Stop_day 
	regBank.add(40103); // ����� ����� adr_Mic_Stop_month 
	regBank.add(40104); // ����� ��� adr_Mic_Stop_year
	regBank.add(40105); // ����� ��� adr_Mic_Stop_hour 
	regBank.add(40106); // ����� ������ adr_Mic_Stop_minute  
	regBank.add(40107); // ����� ������� adr_Mic_Stop_second 

	// ����������������� ���������� �����
	regBank.add(40108); // ����� ���� adr_Time_Test_day 
	regBank.add(40109); // ����� ��� adr_Time_Test_hour 
	regBank.add(40110); // ����� ������ adr_Time_Test_minute
	regBank.add(40111); // ����� ������� adr_Time_Test_second
 
	regBank.add(40120); // adr_control_command ����� �������� ������� �� ����������
	regBank.add(40121); // ����� �������� ������ ������� Sence "��-�����1."  ok!
	regBank.add(40122); // ����� �������� ������ ������� Sence "��-�����2."  ok!
	regBank.add(40123); // ����� �������� ������ ������� ����������� ������
	regBank.add(40124); // ����� �������� ������ �������  Sence ����. �. ok!
	regBank.add(40125); // ����� �������� ������ ������� Sence ���� �. ok!
	regBank.add(40126); // ����� �������� ������ ������� Sence "���."   ok!
	regBank.add(40127); // ����� �������� ������ ������� ��������� ����������� � 2 ����������
	regBank.add(40128); // ����� �������� ������ ������� ��������� �����������
	regBank.add(40129); // ����� �������� ������ ������� ��������� ���������� � 2 ����������

	regBank.add(40130); // ����� �������� ������ ������� ��������� ����������
	regBank.add(40131); // ����� �������� ������ ������� Sence ���. ok!
	regBank.add(40132); // ����� �������� ������ ������� Sence "��C."   ok!
	regBank.add(40133); // ����� �������� ������ ������� 
	regBank.add(40134); // ����� �������� ������ ������� PTT ���� �.  ok!
	regBank.add(40135); // ����� �������� ������ ������� PTT ��� ok!
	regBank.add(40136); // ����� �������� ������  PTT2 ����. �. ok!
	regBank.add(40137); // ����� �������� ������  HangUp  DCD  ok!
	regBank.add(40138); // ����� �������� ������  PTT1 ����. �. ok!
	regBank.add(40139); // ����� �������� ������ ���������� ��������� ��������� ����������� 

	regBank.add(40140); // ����� �������� ������ ���������� PTT ��������� �����������
	regBank.add(40141); // ����� �������� ������ �������� ��������� ����������� FrontL
	regBank.add(40142); // ����� �������� ������ �������� ��������� ����������� FrontR

	slave._device = &regBank;  
}
void test_system()
{
	//prer_Kmerton_On = 0;   
	////reg_Kamerton();
	//Serial.print(regs_in[0],HEX);
	//Serial.print("--");
	//////  Serial.println(regs_out[0],DEC);
	//Serial.print(regs_in[1],HEX);
	//Serial.print("--");
	////  Serial.println(regs_out[1],DEC);
	//Serial.print(regs_in[2],HEX);
	//Serial.print("--");
	//Serial.print(regs_in[3],HEX);

	//Serial.print("-   -");
	//Serial.print(regBank.get(10279),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10278),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10277),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10276),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10275),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10274),HEX);
	//Serial.print("--");
	//Serial.print(regBank.get(10273),HEX);
	//Serial.print("--");
	//Serial.println(regBank.get(10272),HEX);

	//////  Serial.println(regs_out[2],DEC);*/

	//prer_Kmerton_On = 1;   
	////delay(1000);
}
void setup()
{
	Serial.begin(9600);                             // ����������� � USB ��
	Serial1.begin(115200);                          // ����������� � ��������� ������ ��������
	//slave.setBaud(57600);   
	slave.setSerial(2,57600);                       // ����������� � ��������� MODBUS ���������� Serial2 
	Serial3.begin(115200);                          // ��������
	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");
	Wire.begin();
	if (!RTC.begin())                               // ��������� ����� 
		{
			Serial.println("RTC failed");
			while(1);
		};
	
	serial_print_date();
	Serial.println(" ");
	setup_mcp();                                    // ��������� ����� ����������  
	mcp_Analog.digitalWrite(DTR, HIGH);             // ���������� ������ (������)���������� � ����������
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	pinMode(ledPin13, OUTPUT);  
	pinMode(ledPin12, OUTPUT);  
	pinMode(ledPin11, OUTPUT);  
	pinMode(ledPin10, OUTPUT);  
	setup_resistor();                               // ��������� ��������� ���������
	setup_SD();
	if (!SD.begin(SD_Select))                       // ���������� SD ������
		{
			Serial.println("initialization SD failed!");
		}
	SdFile::dateTimeCallback(dateTime);             // ��������� ������� ������ �����
	//setup_SDFile();                                // �������� ������ � �������
	setup_regModbus();                              // ��������� ��������� MODBUS

	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����
	regs_in[0]= 0x00;                               // ��� ������� ����� 
	regs_in[1]= 0x00;                               // 
	regs_in[2]= 0x00;                               // 
	regs_in[3]= 0x00;                               // 
//	reg_Kamerton();
	regBank.set(8,1);                               // �������� ������� ��������
//	sence_all_off();
	UpdateRegs();                                   // �������� ���������� � ���������

	#if FASTADC                                     // �������� ���������� ����������� ������
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif

	for (int i = 120; i < 142; i++)                  // �������� ����� ������
	{
	   regBank.set(i,0);   
	}
	
	for (unsigned int i = 40120; i < 40142; i++)     // �������� ����� ������
	{
	   regBank.set(i,0);   
	}

	MsTimer2::set(30, flash_time);                   // 30ms ������ ������� ���������
	MsTimer2::start();                               // �������� ������ ����������
	resistor(1, 200);                                // ���������� ������� �������
	resistor(2, 200);                                // ���������� ������� �������
	prer_Kmerton_On = 1;                             // ��������� ���������� �� ��������
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	Serial.println(" ");
	Serial.println("System initialization OK!.");
}

void loop()
{
	control_command();
//	delay(100);

/*
	 Serial.print(regs_out[0],HEX);
	 Serial.print("--");
	 Serial.print(regs_out[1],HEX);
	 Serial.print("--");
	 Serial.print(regs_out[2],HEX);
	 Serial.print("    ");
	 */

	 //Serial.print(regs_in[0],BIN); 
	 //Serial.print("--");
	 //Serial.print(regs_in[1],BIN);
	 //Serial.print("--");
	 //Serial.print(regs_in[2],BIN);
	 //Serial.print("--");
	 //Serial.println(regs_in[3],BIN);

	

	 //Serial.print(regBank.get(40004),HEX); 
	 //Serial.print("--");
	 //Serial.print(regBank.get(40005),HEX); 
	 //Serial.print("--");
	 //Serial.print(regBank.get(40006),HEX); 
	 //Serial.print("--");
	 //Serial.println(regBank.get(40007),HEX); 

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
}
