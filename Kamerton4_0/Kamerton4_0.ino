/*

 Kamerton4_0.ino
 VisualStudio
 
 Программа тестирования модуля "Камертон" (Базовый вариант)
 Версия:      - 4_0_0
 Дата:        - 10.06.2015г.
 Организация: - ООО "Децима"
 Автор:       - Мосейчук А.В.
 Версия: Работа продолжена после перерыва 10.06.2015 г.
 Реализовано:
 -
 - прерывание 20мс,
 - передача/прием по СОМ порту,
 - подсчет контролных сумм, связь с Камертоном, 
 - Расширение MCP23017
 - модуль реле, 
 - чтение всех портов, 
 - подключен звуковой генератор
 - Подключена SD память
 - подключены часы, память, 
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


#define  ledPin13  13                               // Назначение светодиодов на плате
#define  ledPin12  12                               // Назначение светодиодов на плате
#define  ledPin11  11                               // Назначение светодиодов на плате
#define  ledPin10  10                               // Назначение светодиодов на плате
#define  Front_led_Blue 14                          // Назначение светодиодов на передней панели
#define  Front_led_Red  15                          // Назначение светодиодов на передней панели

//  Порты управления платой Камертон
#define DTR  8                                      // DTR out выходной сигнал  сформировать 0 для старта
#define RTS  9                                      // RTS out выходной сигнал   
#define CTS  5                                      // CTS in  входной сигнал  флаг нажатия тангенты контролировать!!!!
#define DSR  6                                      // DSR in  входной сигнал  флаг нажатия "Связь - передача"
#define DCD  7                                      // DCD in  входной сигнал  флаг снятия трубки с ложемента 

//+++++++++++++++++++++++ Настройка электронного резистора +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // Адрес микросхемы AD5252  
#define control_word1    0x07                       // Байт инструкции резистор №1
#define control_word2    0x87                       // Байт инструкции резистор №2
byte resistance        = 0x00;                      // Сопротивление 0x00..0xFF - 0Ом..100кОм
//byte level_resist      = 0;                       // Байт считанных данных величины резистора
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
unsigned int Array_volume[514];                     //
unsigned int volume_porog_D = 40;                   // Величина порога при проверке исправности FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // порог величины синусоиды звука
unsigned int  voltage10 ;

#define FASTADC 1                                   // Ускорение считывания аналогового сигнала
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


MCP23017 mcp_Klava;                                 // Назначение портов расширения MCP23017  2 A - in,  B - Out
MCP23017 mcp_Out1;                                  // Назначение портов расширения MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // Назначение портов расширения MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // Назначение портов расширения MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------


int adr_reg_ind_CTS                 = 10081;        // Адрес флагa индикации состояния сигнала CTS
int adr_reg_ind_DSR                 = 10082;        // Адрес флагa индикации состояния сигнала DSR
int adr_reg_ind_DCD                 = 10083;        // Адрес флагa индикации состояния сигнала DCD

// **************** Адреса внешней памяти для хранения даты. Применяется приформировании имени файла *************
int adr_temp_day                    = 240;          // Адрес хранения переменной день
int adr_temp_mon                    = 241;          // Адрес хранения переменной месяц
int adr_temp_year                   = 242;          // Адрес хранения переменной год  
int adr_file_name_count             = 243;          // Адрес хранения переменной счетчика номера файла
//------------------------------------------------------------------------------------------------------------------

//*********************Работа с именем файла ******************************
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

//+++++++++++++++++++++ Установки прерывания +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//++++++++++++++ созданы переменные, использующие функции библиотеки SD utility library functions: +++++++++++++++
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

byte regs_in[5];                                    // Регистры работы с платой Камертон CPLL
byte regs_out[4];                                   // Регистры работы с платой Камертон
byte regs_crc[1];                                   // Регистры работы с платой Камертон контрольная сумма


byte Stop_Kam = 0;                                  // Флаг индикации чтения инф. из Камертона
bool prer_Kmerton_On = true;                        // Флаг разрешение прерывания Камертон
volatile bool prer_Kmerton_Run = false;              // Флаг разрешение прерывания Камертон
#define BUFFER_SIZEK 64                             // Размер буфера Камертон не более 128 байт
unsigned char bufferK;                              // Счетчик количества принимаемых байт

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned int adr_control_command         = 40120; //Адрес передачи комманд на выполнение 

// Текущее время 
unsigned int adr_kontrol_day             = 40046; // адрес день
unsigned int adr_kontrol_month           = 40047; // адрес месяц
unsigned int adr_kontrol_year            = 40048; // адрес год
unsigned int adr_kontrol_hour            = 40049; // адрес час
unsigned int adr_kontrol_minute          = 40050; // адрес минута
unsigned int adr_kontrol_second          = 40051; // адрес секунда

// Установка времени в контроллере
unsigned int adr_set_kontrol_day         = 40052;   // адрес день
unsigned int adr_set_kontrol_month       = 40053;   // адрес месяц
unsigned int adr_set_kontrol_year        = 40054;   // адрес год
unsigned int adr_set_kontrol_hour        = 40055;   // адрес час
unsigned int adr_set_kontrol_minute      = 40056;   // адрес минута

// Время старта теста
unsigned int adr_Mic_Start_day           = 40096; // адрес день
unsigned int adr_Mic_Start_month         = 40097; // адрес месяц
unsigned int adr_Mic_Start_year          = 40098; // адрес год
unsigned int adr_Mic_Start_hour          = 40099; // адрес час
unsigned int adr_Mic_Start_minute        = 40100; // адрес минута
unsigned int adr_Mic_Start_second        = 40101; // адрес секунда

// Время окончания теста
unsigned int adr_Mic_Stop_day            = 40102; // адрес день
unsigned int adr_Mic_Stop_month          = 40103; // адрес месяц
unsigned int adr_Mic_Stop_year           = 40104; // адрес год
unsigned int adr_Mic_Stop_hour           = 40105; // адрес час
unsigned int adr_Mic_Stop_minute         = 40106; // адрес минута
unsigned int adr_Mic_Stop_second         = 40107; // адрес секунда

// Продолжительность выполнения теста
unsigned int adr_Time_Test_day           = 40108; // адрес день
unsigned int adr_Time_Test_hour          = 40109; // адрес час
unsigned int adr_Time_Test_minute        = 40110; // адрес минута
unsigned int adr_Time_Test_second        = 40111; // адрес секунда

unsigned int adr_set_time                = 36;    // адрес флаг установки

//-------------------------------------------------------------------------------------


//========================= Блок программ ============================================


void flash_time()                                              // Программа обработчик прерывания 
{ 
		prer_Kmerton_Run = true;
			digitalWrite(ledPin12,HIGH);
		prer_Kamerton();
		slave.run(); 
			digitalWrite(ledPin12,LOW);
		prer_Kmerton_Run = false;
}

void serialEvent2()
{
	//while(prer_Kmerton_Run == 1) {}                                // Подождать окончания прерывания
	//	digitalWrite(ledPin13,HIGH);
	// // digitalWrite(ledPin13,!digitalRead(ledPin13));               // Сроб импульс MODBUS
	//	UpdateRegs();
	//	slave.run(); 
	//	Serial.println("slave.run");
	////	control_command();
	//	digitalWrite(ledPin13,LOW);
	//while (Serial.available()) 
	//{
	//	char inChar = (char)Serial.read();
	//}
}

void dateTime(uint16_t* date, uint16_t* time)                  // Программа записи времени и даты файла
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}
void prer_Kamerton()                                          // Произвести обмен информации с модулем Камертон
{
	sendPacketK ();  
	// Отправить информацию в модуль Камертон
	waiting_for_replyK();                                  // Получить подтверждение
}
void sendPacketK () 
{              // Программа передачи пакета в Камертон
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
		}
}
void waiting_for_replyK()                                  // Чтение данных из Камертона
{
	  if (Serial1.available())                             // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer = 0;                      // Установить в начало чтения буфера

			while (Serial1.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial1.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (bufferK == BUFFER_SIZEK)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
						}
						regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
			calculateCRC_In();
		   }
	 else 
		{
			Stop_Kam = 0;                                    // Флаг отсутств. инф. из Камертона
		}
}
void Stop_Kamerton ()                  //Если не приходит информация с Камертона - регистры обнулить
  {
	 for (unsigned char i = 0; i <4; i++)
	 regs_in[i]=0;
  }

void calculateCRC_Out()                // Вычисление контрольной суммы ниблов байта
{ 
  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // записать  
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта
  temp2 = temp1>>4;                    // Переместить старший нибл в младший
  temp3 = regs_out[2];                 // записать
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 2 байта
  temp3 = temp3>>4;                    // Переместить старший нибл в младший
  temp4 = regs_out[2];                 // записать
  temp4 = temp4&0x0F;                  // Наложить маску F0 на младший нибл 2 байта
  crc =  temp2 ^  temp3 ^  temp4  ;
  crc = crc&0x0F;                      // Наложить маску F0 на младший нибл 2 байта
  regs_out[1]= temp1 | crc;
}
void calculateCRC_In()                 // Вычисление контрольной суммы ниблов байта
{ 
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // записать  
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта
  temp1H = temp1>>4;                   // Переместить старший нибл в младший
  temp1 = regs_in[0];                  // записать 
  temp1L = temp1&0x0F;                 // Наложить маску 0F на младший нибл 1 байта

  temp2 = regs_in[1];                  // записать  
  temp2 = temp2&0xF0;                  // Наложить маску F0 на старший нибл 2 байта
  temp2H = temp2>>4;                   // Переместить старший нибл в младший
  temp2 = regs_in[1];                  // записать 
  temp2L = temp2&0x0F;                 // Наложить маску 0F на младший нибл 2 байта

  temp3 = regs_in[2];                  // записать  
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp3H = temp3>>4;                   // Переместить старший нибл в младший
  temp3 = regs_in[2];                  // записать 
  temp3L = temp3&0x0F;                 // Наложить маску 0F на младший нибл 3 байта

  temp4 = regs_in[3];                  // записать  
  temp4 = temp4&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp4H = temp4>>4;                   // Переместить старший нибл в младший
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // Наложить маску F0 на младший нибл 4 байта
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

void reg_Kamerton()                                       // Подпрограмма преобразования и переноса данных из  regs_out[3] в регистры Samkoon
{
	 byte i4,i5;
	 unsigned int  i7=30001;                             //Регистр хранения первого байта данных для передачи в Камертон
	//  unsigned int  i7=10224;    

	 regBank.set(40001,regs_out[0]);   
	 regBank.set(40002,regs_out[1]);   
	 regBank.set(40003,regs_out[2]);   
	 regBank.set(40004,regs_in[0]);   
	 regBank.set(40005,regs_in[1]); 
	 regBank.set(40006,regs_in[2]); 
	 regBank.set(40007,regs_in[3]); 


	//i5 = regs_out[0];                                     // 1 байт передачи
	/*
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+i4,bitRead(i5,i4));            // 1 байт передачи перенесен в регистры Samkoon
		}
	i5= regs_out[1];                                      // 2 байт передачи
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+8+i4,bitRead(i5,i4));          // 2 байт передачи перенесен в регистры Samkoon
		}
	i5= regs_out[2];                                      // 3 байт передачи
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+16+i4,bitRead(i5,i4));         // 3 байт передачи перенесен в регистры Samkoon
		}
	
	//********************************************************************************

	i5= regs_in[0];                                      // 1 байт полученный из Камертона
		for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+24+i4,bitRead(i5,i4));
		}
	i5= regs_in[1];
	for (i4 = 0; i4 <8; i4++)                              // 2 байт полученный из Камертона
		{
			regBank.set(i7+32+i4,bitRead(i5,i4));
		}
	i5= regs_in[2];                                        // 3 байт полученный из Камертона
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+40+i4,bitRead(i5,i4));
		}
	i5= regs_in[3];                                        // 4 байт полученный из Камертона
	for (i4 = 0; i4 <8; i4++)
		{
			regBank.set(i7+48+i4,bitRead(i5,i4));
		}
	*/
	regBank.set(adr_reg_ind_CTS, !mcp_Analog.digitalRead(CTS));
	regBank.set(adr_reg_ind_DSR, !mcp_Analog.digitalRead(DSR));
	regBank.set(adr_reg_ind_DCD, !mcp_Analog.digitalRead(DCD));

		 
}
void UpdateRegs()                                        // Обновить регистры
{
	//-----Первый байт ------------
	//-----Установить бит 0
	 while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // Запретить прерывание Камертон ??
	 reg_Kamerton();                                     // Записать данные из Камертон в    регистры 
		// Подпрограмма переноса данных из регистров на порты вывода
	  //-----Установить бит 0
	 set_rele = regBank.get(1);
	 mcp_Out1.digitalWrite(0, set_rele);                 // Реле RL0 Звук  Звук Mic1p Диспетчер

	 //-----Установить бит 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // Реле RL1 Звук Mic2p  Инструкор

	 //-----Установить бит 2
	  set_rele = regBank.get(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // Реле RL2 Звук Mic3p MTT
  
	 //-----Установить бит 3
	  set_rele = regBank.get(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // Реле RL3 Звук

	 //-----Установить бит 4                            // Реле RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----Установить бит 5
	  set_rele = regBank.get(6);                        // Реле RL5 Звук
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----Установить бит 6	 
	  set_rele = regBank.get(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // Реле RL6 Звук

	 //-----Установить бит 7
	  set_rele = regBank.get(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // Реле RL7 Питание платы

	 //---- Второй байт----------
	 //-----Установить бит 8
	  set_rele = regBank.get(9);                        // Реле RL8 Звук на микрофон
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----Установить бит 9
	  set_rele = regBank.get(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // Реле RL9 XP1 10

	 //-----Установить бит 10                           // Свободен J24 - 2 
	  set_rele = regBank.get(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----Установить бит 11                            // Свободен J24 - 1
	  set_rele = regBank.get(12);
	  mcp_Out1.digitalWrite(11, set_rele);    

	 //-----Установить бит 12
	  set_rele = regBank.get(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   Sence Тангента ножная

	 //-----Установить бит 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT Тангента ножная

	 //-----Установить бит 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT Мик

	  //-----Установить бит 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   Sence Мик

	  //  Test 3
	 //-----Первый байт ------------
	 //-----Установить бит 0

	  set_rele = regBank.get(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   Танг. р.

	 //-----Установить бит 1
	  set_rele = regBank.get(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----Установить бит 2
	  set_rele = regBank.get(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 Sence  Танг. р.
  
	//-----Установить бит 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 Танг. р.

	 //-----Установить бит 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     Sence "Маг." 

	 //-----Установить бит 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     Sence "ГГC."

	 //-----Установить бит 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     Sence "ГГ-Радио1."

	 //-----Установить бит 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     Sence "ГГ-Радио2."

	  // Test 4
	//-----Первый байт ------------
	 //-----Установить бит 8
	  set_rele = regBank.get(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      флаг подключения трубки  

	  //-----Установить бит 9
	  set_rele = regBank.get(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR вкл.

	  //-----Установить бит 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками

	  //-----Установить бит 11
	  set_rele = regBank.get(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS вкл

	  //-----Установить бит 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 

	  //-----Установить бит 13
	  set_rele = regBank.get(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS вкл

	  //-----Установить бит 14
	  set_rele = regBank.get(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    Флаг подкючения гарнитуры диспетчера с 2 наушниками

	  //-----Установить бит 15
	  set_rele = regBank.get(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    Флаг подкючения гарнитуры диспетчера

	  //*******************************************************
/*
// перенести состояние регистров в переменные
  
//	  avto_test  =  regBank.get(33);
	 // start_prer =  regBank.get(34);


	  regBank.set(adr_reg_count_cts0, count_test_CTS);                    // записать в регистр показания счетчика проходов теста CTS
	  regBank.set(adr_reg_err_cts0, err_count_cts);                       // записать в регистр показания счетчика ошибок CTS
	  regBank.set(adr_reg_err_volume_HaSPTT, err_count_volume_HaSPTT);    // записать в регистр показания счетчика Нет звука HaSPTT
	  regBank.set(adr_reg_err_HaSPTT_ON, err_count_HaSPTT_ON);            // записать в регистр показания счетчика ошибок HaSPTT_ON
	  regBank.set(adr_reg_err_HaSPTT_OFF, err_count_HaSPTT_OFF);          // записать в регистр показания счетчика ошибок HaSPTT_OFF
	  regBank.set(adr_reg_err_volume_HeS1PTT, err_count_volume_HeS1PTT);  // записать в регистр показания счетчика Нет звука  HeS1PTT
	  regBank.set(adr_reg_err_HeS1PTT_ON, err_count_HeS1PTT_ON);          // записать в регистр показания счетчика ошибок HeS1PTT_ON
	  regBank.set(adr_reg_err_HeS1PTT_OFF,err_count_HeS1PTT_OFF );        // записать в регистр показания счетчика ошибок HeS1PTT_OFF
	  regBank.set(adr_reg_err_volume_HeS2PTT,err_count_volume_HeS2PTT);   // записать в регистр показания счетчика Нет звука HeS2PTT
	  regBank.set(adr_reg_err_HeS2PTT_ON,err_count_HeS2PTT_ON);           // записать в регистр показания счетчика ошибок HeS2PTT_ON     
	  regBank.set(adr_reg_err_HeS2PTT_OFF,err_count_HeS2PTT_OFF);         // записать в регистр показания счетчика ошибок HeS2PTT_OFF
	  regBank.set(adr_reg_err_volume_MicPTT, err_count_volume_MicPTT);    // записать в регистр показания счетчика Нет звука  MicPTT
	  regBank.set(adr_reg_err_MicPTT_ON, err_count_MicPTT_ON);            // записать в регистр показания счетчика ошибок MicPTT_ON
	  regBank.set(adr_reg_err_MicPTT_OFF,err_count_MicPTT_OFF );          // записать в регистр показания счетчика ошибок MicPTT_OFF
	  regBank.set(adr_reg_err_volume_TangNPTT,err_count_volume_TangNPTT); // записать в регистр показания счетчика Нет звука HeS2PTT
	  regBank.set(adr_reg_err_TangNPTT_ON,err_count_TangNPTT_ON);         // записать в регистр показания счетчика ошибок HeS2PTT_ON     
	  regBank.set(adr_reg_err_TangNPTT_OFF,err_count_TangNPTT_OFF);       // записать в регистр показания счетчика ошибок HeS2PTT_OFF

	 // regBank.set(adr_reg_count_Mic, count_test_Mic);                   // записать в регистр показания счетчика проходов теста Mic
	  regBank.set(adr_reg_err_Mic, err_count_Mic);                        // записать в регистр показания счетчика ошибок CTS
	  */
	  time_control();
	  prer_Kmerton_On = true;
}
void set_clock()
{    
		int day    = regBank.get(adr_set_kontrol_day);  
		int month  = regBank.get(adr_set_kontrol_month);          
		int year   = regBank.get(adr_set_kontrol_year);  
		int hour   = regBank.get(adr_set_kontrol_hour);  
		int minute = regBank.get(adr_set_kontrol_minute);  
		int second = 0;
		DateTime set_time = DateTime(year, month, day, hour, minute, second); // Занести данные о времени в строку "set_time"
		RTC.adjust(set_time);                                // Записать время в контроллер часов  
		regBank.set(adr_set_time, 0);                        // Записать в регистр признак окончания выполнения команды
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
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// при смене даты счетчик номера файла сбросить в "0"
			}

		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon); //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// при смене даты счетчик номера файла сбросить в "0"
			}
		
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year); //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);// при смене даты счетчик номера файла сбросить в "0"
			}
}
void time_control() // Программа записи текущего времени в регистры для передачи в ПК
{
	DateTime now = RTC.now();
	regBank.set(adr_kontrol_day  , now.day());
	regBank.set(adr_kontrol_month, now.month());
	regBank.set(adr_kontrol_year, now.year());
	regBank.set(adr_kontrol_hour, now.hour());
	regBank.set(adr_kontrol_minute, now.minute());
	regBank.set(adr_kontrol_second, now.second());
}
void time_control_get()   // Тестовая программа проверки содержания регистров времени
{
  for (unsigned int i = 0; i < 6; i++)     // 
	{
	   Serial.print(regBank.get(40046+i));   
	   Serial.print(" "); 
	}
Serial.println();   
}

void file_print_date()  //программа  записи даты в файл
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
void serial_print_date()                           // Печать даты и времени    
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
			//Wire.requestFrom(address_AD5252, 1, true);  // Считать состояние движка резистора 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
}
void FileOpen()
{
	DateTime now = RTC.now();

			regBank.set(adr_Mic_Start_day , now.day());  // Время старта теста
			regBank.set(adr_Mic_Start_month, now.month());
			regBank.set(adr_Mic_Start_year, now.year());
			regBank.set(adr_Mic_Start_hour, now.hour());
			regBank.set(adr_Mic_Start_minute, now.minute());
			regBank.set(adr_Mic_Start_second, now.second());
				
			regBank.set(adr_Time_Test_day, 0); 
			regBank.set(adr_Time_Test_hour, 0); 
			regBank.set(adr_Time_Test_minute, 0); 
			regBank.set(adr_Time_Test_second, 0); 
	

	data_clock_exchange();                               // Проверить не изменилась ли дата

	file_name_count = i2c_eeprom_read_byte(0x50, adr_file_name_count); // считать текущий номер файла из памяти

	preob_num_str();                                     // сформировать имя файла из даты и счетчика файлов
 
	 if (SD.exists(file_name))                           // проверить есть ли такой файл
	  { 

		Serial.print(file_name);
		Serial.println("  OK!.");
	  }
	  else 
	  {
		  Serial.println("");
		  Serial.print(file_name);
		  Serial.println(" doesn't exist.");             // такого файла нет
	  }

	  myFile = SD.open(file_name, FILE_WRITE);            // открыть файл для записи данных
	  myFile.println ("");
	  myFile.print ("Start test   ");
	  file_print_date();
	  myFile.println ("");
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
	Для вызова подпрограммы проверки необходимо записать номер проверки по адресу adr_control_command (40120) 
	Код проверки
	0 -  Выполнение команды окончено
	1 -  Отключить все сенсоры
	2 -  Включить все сенсоры
	3 -  Тест Инструктора
	4 -  Тест диспетчера
	5 -  Тест МТТ
	6 -  Тест Танг.р
	7 -  Тест Микрофон
	8 -  Тест ГГС
	9 -  Тест Радио 1
	10 - Тест Радио 2
	11 - Тест Магнитофон
	12 - Открыть файл
	13 - Закрыть файл
	14 - Записать время


	*/
	UpdateRegs() ;

	int test_n = regBank.get(adr_control_command); //адрес  40120
	//if (test_n!= 0) Serial.println(test_n);
		
	switch (test_n)
	{
		case 1:
			// regBank.set(7,1);                              // Включить питание Камертон
			// Serial.println(" sence_all_off");
			 sence_all_off();                                // Отключить все сенсоры
			break;
		case 2:		
			// Serial.println(" sence_all_on");
			 sence_all_on();                                 // Включить все сенсоры
				break;
		case 3:
			// Serial.println("test_instruktora");
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
			// test_mikrophon();             // Тестирование микрофона
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
			// Serial.println("FileOpen");
			   FileOpen();
				break;
		case 13:	
			//  Serial.println("FileClose");
			  FileClose();
				break;
		case 14:
			//  Serial.println("Set clock");
			  set_clock();
			//  time_control_get();
				break;
		default:
		break;

	 }
	regBank.set(adr_control_command,0);
}

void sence_all_off()
{
  regBank.set(7,1);      // Включить питание Камертон
  regBank.set(20,0);     // * XP2 Sence "Маг." 
  regBank.set(21,0);     // * XP5 Sence "ГГC."
  regBank.set(22,0);     // * XP3 Sence "ГГ-Радио1."  
  regBank.set(23,0);     // * XP4 Sence "ГГ-Радио2."  
  regBank.set(15,0);     // * XS1 6 Sence подключения микрофона

  regBank.set(12,0);     // * XP8 2 Sence тангента ножная
  regBank.set(18,0);     // * XP7 2 Sence тангента ручная
  regBank.set(19,0);     // XP7 1 PTT1 тангента ручная
  regBank.set(16,0);     // XP7 4 PTT2 тангента ручная
  regBank.set(17,0);     // XP1 - 20  HandUp


  regBank.set(31,0);     // * XP1- 1  HeS1Ls   Sence подключения гарнитуры диспетчера
  regBank.set(30,0);     // * XP1- 5  HeS1Rs   Sence подключения гарнитуры диспетчера с 2 наушниками
  regBank.set(29,0);     // XP1- 6  HeS1PTT 
  regBank.set(28,0);     // * XP1- 13 HeS2Ls   Sence подключения гарнитуры инструктора
  regBank.set(27,0);     // XP1- 15 HeS2PTT 

  regBank.set(26,0);     // * XP1- 16 HeS2Rs   Sence подключения гарнитуры инструктора с 2 наушниками
  regBank.set(25,0);     // XP1- 17 HaSPTT   
  regBank.set(24,1);     // XP1- 19 HaSs
  regBank.set(13,0);     // XP8 - 1  PTT 
  regBank.set(14,0);     // XS1 - 5   PTT  
  UpdateRegs(); 
  delay(100);

  
 //1 ок
	   if(regBank.get(30040) > 0)                  // Флаг счетчика ошибки сенсора Sence "Маг."   ok!
		  {
			int regcount = regBank.get(40126);     // 
			regcount++;
			regBank.set(40126,regcount);           // 
			regBank.set(126,1);
			regBank.set(120,1);
			myFile.print("Komanda sensor Off Sence Mag Error! - ");
			myFile.println(regcount);
		  }
//2 ок
	   if(regBank.get(30046) > 0)                   // XP5 Sence "ГГC."
		  {
			int regcount = regBank.get(40132);      // 
			regcount++;
			regBank.set(40132,regcount);            // 
			regBank.set(132,1);
			regBank.set(120,1);
			myFile.print("Komanda sensor Off GGS  Error! - ");
			myFile.println(regcount);
		  }
//3	  ок 
	   if(regBank.get(30025) > 0)                   // XP3 Sence "ГГ-Радио1."  
		  {
			int regcount = regBank.get(40121);      // 
			regcount++;
			regBank.set(40121,regcount);            // 
			regBank.set(121,1);
			regBank.set(120,1);
			myFile.print("Komanda sensor Off GG Radio1 Error! - ");
			myFile.println(regcount);
		  }
//4	ок
	   if(regBank.get(30024) > 0)                   // XP3 Sence "ГГ-Радио2."  
		  {
			int regcount = regBank.get(40122);      // 
			regcount++;
			regBank.set(40122,regcount);            // 
			regBank.set(122,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off GG Radio2  Error! - ");
			myFile.println(regcount);
		  }
//5 ок
	   if(regBank.get(30026) > 0)                   // XS1 6 Sence подключения трубки
		  {
			int regcount = regBank.get(40123);      // 
			regcount++;
			regBank.set(40123,regcount);            // 
			regBank.set(123,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Trubka  Error! - ");
			myFile.println(regcount);
		  }
//6	нет
	   if(regBank.get(30045) > 0)                   // Флаг микрофона
		  {
			int regcount = regBank.get(40131);      // Камертон   флаг подключения микрофона
			regcount++;
			regBank.set(40131,regcount);            // 
			regBank.set(131,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Microphon  Error! - ");
			myFile.println(regcount);
		  }
//7	ок  
	   if(regBank.get(30028) > 0)                   // Sence Танг ножная ok!
		  {
			int regcount = regBank.get(40125);      // 
			regcount++;
			regBank.set(40125,regcount);            // 
			regBank.set(125,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Tangenta n.  Error! - ");
			myFile.println(regcount);
		  }
//8	ок
	   if(regBank.get(30027) > 0)                   // Камертон  флаг подключения ручной тангенты
		  {
			int regcount = regBank.get(40124);      // 
			regcount++;
			regBank.set(40124,regcount);            // 
			regBank.set(124,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Tangenta ruch.  Error! - ");
			myFile.println(regcount);
		  }

//9	ок
	   if(regBank.get(30042) > 0)                   // флаг подключения гарнитуры инструктора
		  {
			int regcount = regBank.get(40128);      // 
			regcount++;
			regBank.set(40128,regcount);            // 
			regBank.set(128,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura instruktora  Error! - ");
			myFile.println(regcount);
		  }

//10 ок
	   if(regBank.get(30041) > 0)                   //Камертон   флаг подключения гарнитуры инструктора 2 наушниками
		  {
			int regcount = regBank.get(40127);      // 
			regcount++;
			regBank.set(40127,regcount);            // 
			regBank.set(127,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura instruktora 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }
//11 ок
	   if(regBank.get(30044) > 0)                    //Камертон   флаг подключения гарнитуры диспетчера
		  {
			int regcount = regBank.get(40130);       // 
			regcount++;
			regBank.set(40130,regcount);             // 
			regBank.set(130,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura dispetchera  Error! - ");
			myFile.println(regcount);
		  }
	 
//12 ок  
	   if(regBank.get(30043) > 0)                    //Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками
		  {
			int regcount = regBank.get(40129);       // 
			regcount++;
			regBank.set(40129,regcount);             // 
			regBank.set(129,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor Off Garnitura dispetchera 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }



// * regBank.add(121);   // Флаг счетчика ошибки сенсора Sence "ГГ-Радио1."  ok!
// * regBank.add(40121); // адрес счетчика ошибки сенсора Sence "ГГ-Радио1."  ok!
// * regBank.add(30025);  // байт 1 прием бит 1 - Камертон  флаг подключения ГГ Радио1


// * regBank.add(122);   // Флаг счетчика ошибки сенсора Sence "ГГ-Радио2."  ok!
// * regBank.add(40122); // адрес счетчика ошибки сенсора Sence "ГГ-Радио2."  ok!
// * regBank.add(30024);  // байт 1 прием бит 0 - Камертон  флаг подключения ГГ Радио2

// * regBank.add(123);   // Флаг счетчика ошибки сенсора подключения трубки
// * regBank.add(40123); // адрес счетчика ошибки сенсора подключения трубки
// * regBank.add(30026);  // байт 1 прием бит 2 - Камертон  флаг подключения трубки

// * regBank.add(124);   // Флаг счетчика ошибки сенсора  Sence Танг. р. ok!
// * regBank.add(40124); // адрес счетчика ошибки сенсора  Sence Танг. р. ok!
// * regBank.add(30027);  // байт 1 прием бит 3 - Камертон  флаг подключения ручной тангенты

// * regBank.add(125);   // Флаг счетчика ошибки сенсора Sence Танг н. ok!
// * regBank.add(40125); // адрес счетчика ошибки сенсора Sence Танг н. ok!
// * regBank.add(30028);  // байт 1 прием бит 4 - Камертон  флаг подключения педали

// * regBank.add(126);   // Флаг счетчика ошибки сенсора Sence "Маг."   ok!
// * regBank.add(40126); // адрес счетчика ошибки сенсора Sence "Маг."   ok!
// * egBank.add(30040);  // байт 3 прием бит 0 - Камертон   флаг подключения магнитофона

// * regBank.add(127);   // Флаг счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
// * regBank.add(40127); // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
// * regBank.add(30041);  // байт 3 прием бит 1 - Камертон   флаг подключения гарнитуры инструктора 2 наушниками

// * regBank.add(128);   // Флаг счетчика ошибки сенсора гарнитуры инструктора
// * regBank.add(40128); // адрес счетчика ошибки сенсора гарнитуры инструктора
// * regBank.add(30042);  // байт 3 прием бит 2 - Камертон   флаг подключения гарнитуры инструктора

// * regBank.add(129);   // Флаг счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
// * regBank.add(40129); // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
// * regBank.add(30043);  // байт 3 прием бит 3 - Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками


// * regBank.add(130);   // Флаг счетчика ошибки сенсора гарнитуры диспетчера
// * regBank.add(40130); // адрес счетчика ошибки сенсора гарнитуры диспетчера
// * regBank.add(30044);  // байт 3 прием бит 4 - Камертон   флаг подключения гарнитуры диспетчера


//regBank.add(131);   // Флаг счетчика ошибки сенсора Sence Мик. ok!
//regBank.add(40131); // адрес счетчика ошибки сенсора Sence Мик. ok!
//regBank.add(30045);  // байт 3 прием бит 5 - Камертон   флаг подключения микрофона XS1 - 6 Sence


// * regBank.add(132);   // Флаг счетчика ошибки сенсора Sence "ГГC."   ok!
// * regBank.add(40132); // адрес счетчика ошибки сенсора Sence "ГГC."   ok!
// * regBank.add(30046);  // байт 3 прием бит 6 - Камертон   флаг подключения ГГС


//regBank.add(133);   // Флаг счетчика ошибки сенсора 
//regBank.add(134);   // Флаг счетчика ошибки сенсора PTT Танг н.  ok!
//regBank.add(135);   // Флаг счетчика ошибки сенсора PTT Мик ok!

//regBank.add(136);   // Флаг счетчика ошибки  PTT2 Танг. р. ok!
//regBank.add(137);   // Флаг счетчика ошибки  HangUp  DCD  ok!
//regBank.add(138);   // Флаг счетчика ошибки  PTT1 Танг. р. ok!



//regBank.add(40133); // адрес счетчика ошибки сенсора 
//regBank.add(40134); // адрес счетчика ошибки сенсора PTT Танг н.  ok!
//regBank.add(40135); // адрес счетчика ошибки сенсора PTT Мик ok!
//
//regBank.add(40136); // адрес счетчика ошибки  PTT2 Танг. р. ok!
//regBank.add(40137); // адрес счетчика ошибки  HangUp  DCD  ok!
//regBank.add(40138); // адрес счетчика ошибки  PTT1 Танг. р. ok!
//regBank.add(40139); // адрес счетчика ошибки 
//regBank.add(40140); // адрес счетчика ошибки 

//regBank.add(30052);  // байт 4 прием бит 4 - Камертон    флаг выключения ГГС (Mute)
//regBank.add(30053);  // байт 4 прием бит 5 - Камертон    флаг радиопередачи
//regBank.add(30054);  // байт 4 прием бит 6 - Камертон    флаг управления микрофонами гарнитур
 

   delay(100);
}
void sence_all_on()
{
  regBank.set(20,1);     //XP2 "Маг." 
  regBank.set(21,1);     //XP5 Sence "ГГC."
  regBank.set(22,1);     //XP3 Sence "ГГ-Радио1."  
  regBank.set(23,1);     //XP4 Sence "ГГ-Радио2."  
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

 //1 ок
	   if(regBank.get(30040) == 0)                  // Флаг счетчика ошибки сенсора Sence "Маг."   ok!
		  {
			int regcount = regBank.get(40126);     // 
			regcount++;
			regBank.set(40126,regcount);           // 
			regBank.set(126,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Sence Mag Error! - ");
			myFile.println(regcount);
		  }
//2 ок
	   if(regBank.get(30046) == 0)                   // XP5 Sence "ГГC."
		  {
			int regcount = regBank.get(40132);      // 
			regcount++;
			regBank.set(40132,regcount);            // 
			regBank.set(132,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GGS  Error! - ");
			myFile.println(regcount);
		  }
//3	  ок 
	   if(regBank.get(30025) == 0)                   // XP3 Sence "ГГ-Радио1."  
		  {
			int regcount = regBank.get(40121);      // 
			regcount++;
			regBank.set(40121,regcount);            // 
			regBank.set(121,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GG Radio1 Error! - ");
			myFile.println(regcount);
		  }
//4	ок
	   if(regBank.get(30024) == 0)                   // XP3 Sence "ГГ-Радио2."  
		  {
			int regcount = regBank.get(40122);      // 
			regcount++;
			regBank.set(40122,regcount);            // 
			regBank.set(122,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On GG Radio2  Error! - ");
			myFile.println(regcount);
		  }
//5 ок
	   if(regBank.get(30026) == 0)                   // XS1 6 Sence подключения трубки
		  {
			int regcount = regBank.get(40123);      // 
			regcount++;
			regBank.set(40123,regcount);            // 
			regBank.set(123,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Trubka  Error! - ");
			myFile.println(regcount);
		  }
//6	нет
	   if(regBank.get(30045) == 0)                   // Флаг микрофона
		  {
			int regcount = regBank.get(40131);      // Камертон   флаг подключения микрофона
			regcount++;
			regBank.set(40131,regcount);            // 
			regBank.set(131,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Microphon  Error! - ");
			myFile.println(regcount);
		  }
//7	ок  
	   if(regBank.get(30028) == 0)                   // Sence Танг ножная ok!
		  {
			int regcount = regBank.get(40125);      // 
			regcount++;
			regBank.set(40125,regcount);            // 
			regBank.set(125,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Tangenta n.  Error! - ");
			myFile.println(regcount);
		  }
//8	ок
	   if(regBank.get(30027) == 0)                   // Камертон  флаг подключения ручной тангенты
		  {
			int regcount = regBank.get(40124);      // 
			regcount++;
			regBank.set(40124,regcount);            // 
			regBank.set(124,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Tangenta ruch.  Error! - ");
			myFile.println(regcount);
		  }

//9	ок
	   if(regBank.get(30042) == 0)                   // флаг подключения гарнитуры инструктора
		  {
			int regcount = regBank.get(40128);      // 
			regcount++;
			regBank.set(40128,regcount);            // 
			regBank.set(128,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura instruktora  Error! - ");
			myFile.println(regcount);
		  }

//10 ок
	   if(regBank.get(30041) == 0)                   //Камертон   флаг подключения гарнитуры инструктора 2 наушниками
		  {
			int regcount = regBank.get(40127);      // 
			regcount++;
			regBank.set(40127,regcount);            // 
			regBank.set(127,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura instruktora 2 dinamik  Error! - ");
			myFile.println(regcount);
		  }
//11 ок
	   if(regBank.get(30044) == 0)                    //Камертон   флаг подключения гарнитуры диспетчера
		  {
			int regcount = regBank.get(40130);       // 
			regcount++;
			regBank.set(40130,regcount);             // 
			regBank.set(130,1);
			regBank.set(120,1);

			myFile.print("Komanda sensor On Garnitura dispetchera  Error! - ");
			myFile.println(regcount);
		  }
	 
//12 ок  
	   if(regBank.get(30043) == 0)                    //Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками
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
	  regBank.set(29,0);                                                // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	  myFile.println("Komanda sensor Off SpkRout  send!");
	  regBank.set(27,0);                                                // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	  UpdateRegs();                                                     // Выполнить команду отключения сенсоров
	  delay(600);                                                       // 
	  UpdateRegs();                                                     // Выполнить команду отключения сенсоров
	  delay(600);                                                       //
	  byte i5 = regs_in[2];                                             // 

		if(bitRead(i5,1) > 0)                                           // Проверка  флага подключения гарнитуры инструктора 2 наушниками
		  {
			int regcount = regBank.get(40127);                          // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40127,regcount);                                // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
			regBank.set(127,1);                                         // установить флаг ошибки
			regBank.set(120,1);                                         // установить общий флаг ошибки
			resistor(1, 255);                                           // Установить уровень сигнала в исходное состояниу
			resistor(2, 255);                                           // Установить уровень сигнала в исходное состояниу
			myFile.print("Komanda sensor Off SpkLout  Error! - ");      //
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.println("Komanda sensor Off SpkLout  Ok!");
		  }

		if(bitRead(i5,2) > 0)                                           // Проверка  флага подключения гарнитуры инструктора 
		  {
			 int regcount = regBank.get(40128);                         // адрес счетчика ошибки сенсора гарнитуры инструктора
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40128,regcount);                               // адрес счетчика ошибки сенсора гарнитуры инструктора
			 regBank.set(128,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 myFile.print("Komanda sensor Off SpkRout  Error! - ");
			 myFile.println(regcount);
		  }
		else
		  {
			 myFile.println("Komanda sensor Off SpkRout  Ok!");
		  }

		  myFile.println("Komanda sensor On  SpkLout  send!");
		  regBank.set(29,1);                                            // XP1- 13 HeS2Ls
		  myFile.println("Komanda sensor On  SpkRout  send!");
		  regBank.set(27,1);                                            // XP1- 16 HeS2Rs
		  UpdateRegs(); 
		  delay(300);
		  UpdateRegs(); 
		  delay(200);

		 if(bitRead(i5,1) > 0)                                          // Проверка флага подключения гарнитуры инструктора 2 наушниками
			{
				int regcount = regBank.get(40127);                      // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40127,regcount);                            // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
				regBank.set(127,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				myFile.print("Komanda sensor On  SpkLout  Error! - ");
				myFile.println(regcount);
			}
		else
			{
				myFile.println("Komanda sensor On  SpkLout  Ok!");
			}
	   if(bitRead(i5,2) > 0)                                            // Проверка флага подключения гарнитуры инструктора
			 {
				int regcount = regBank.get(40128);                      // адрес счетчика ошибки сенсора гарнитуры инструктора
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40128,regcount);                            // адрес счетчика ошибки сенсора гарнитуры инструктора
				regBank.set(128,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				myFile.print("Komanda sensor On  SpkRout  Error! - ");
				myFile.println(regcount);
			 }
		else
			{
				myFile.println("Komanda sensor On  SpkRout  Ok!");
			}
	//	Serial.println(regBank.get(127));
	//		Serial.println(regBank.get(40127));
	//		Serial.println(regBank.get(128));
	//		Serial.println(regBank.get(40128));




		 regBank.set(4,0);                                             // LFE  Отключить звук на  микрофон инструктора
		 myFile.println("Komanda microphon instr. Off send!");
		 regBank.set(28,0);                                            // XP1- 15 HeS2Ls Отключить PTT инструктора
		 myFile.println("Komanda PTT instr. Off  send!");
		 regBank.set(5,0);                                             // Подать управляющую команду на вывод 12 ХР1 HeS2e
		 UpdateRegs();                                                 // 
		 delay(200);
		 UpdateRegs(); 
		 delay(100);
		 i5 = regs_in[3];

		if(bitRead(i5,4)!= 0) 		                                   // Проверить отключение микрофона инструктора
			{
				int regcount = regBank.get(40139);                     // адрес счетчика ошибки отключения микрофона гарнитуры инструктора
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40139,regcount);                           // адрес счетчика ошибки отключения микрофона гарнитуры инструктора
				regBank.set(139,1);                                    // установить флаг ошибки отключения микрофона гарнитуры инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки
				myFile.print("Komanda microphon instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		if(regBank.get(10080)!= 0)                                     // Проверить отключение PTT инструктора
			{
				int regcount = regBank.get(40140);                     // адрес счетчика ошибки отключения PTT гарнитуры инструктора
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40140,regcount);                           // адрес счетчика ошибки отключения PTT гарнитуры инструктора
				regBank.set(140,1);                                    // установить флаг ошибки отключения PTT гарнитуры инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки
				myFile.print("Komanda PTT instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		delay(1000);

		//++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок +++++++++++++++++++++++++++++++++
		measure_volume(analog_FrontL);                                 // Измерить уровень сигнала на выходе FrontL   

		if(voltage10 > volume_porog_D)                                 // volume_porog_D Проверить исправность канала динамиков инструктора
			{
				int regcount = regBank.get(40141);                     // адрес счетчика ошибки канала динамиков  инструктора
				regcount++;                                            // увеличить счетчик ошибок канала динамиков 
				regBank.set(40141,regcount);                           // адрес счетчика ошибки канала динамиков  инструктора
				regBank.set(141,1);                                    // установить флаг ошибки  канала динамиков  инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки 
				myFile.print("Komanda FrontL instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		                                                               // Проверить исправность канала динамиков на отсутствие наводок
		measure_volume(analog_FrontR);                                 // Измерить уровень сигнала на выходе FrontR

		if(voltage10 > volume_porog_D)                                 // Проверить исправность канала динамиков  инструктора
			{
				int regcount = regBank.get(40142);                     // адрес счетчика ошибки канала динамиков  гарнитуры инструктора
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40142,regcount);                           // адрес счетчика ошибки канала динамиков  гарнитуры инструктора
				regBank.set(142,1);                                    // установить флаг ошибки канала динамиков  гарнитуры инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки
				myFile.print("Komanda FrontR instrukt. Off Error! - ");
				myFile.println(regcount);
			}

		//-------------------------------------------------------------------------------------------------------------------------------

		// +++++++++++++++++++++++++++ Подать сигнал на вход микрофона -----------------------------------------------------------------

		resistor(1, 40);                                           // Установить уровень сигнала 30 мв
		resistor(2, 40);                                           // Установить уровень сигнала 30 мв
		regBank.set(2,1);                                          // Подать сигнал на вход микрофона Mic2p
		UpdateRegs();                                              // Выполнить команду
		delay(300);

		// Заменить на подпрограмму !!
		//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала в динамиках +++++++++++++++++++++++++++++++++
		measure_volume(analog_FrontL);                                 // Измерить уровень сигнала на выходе FrontL   

		if(voltage10 > volume_porog_D)                                 // volume_porog_D Проверить исправность канала динамиков инструктора
			{
				int regcount = regBank.get(40141);                     // адрес счетчика ошибки канала динамиков  инструктора
				regcount++;                                            // увеличить счетчик ошибок канала динамиков 
				regBank.set(40141,regcount);                           // адрес счетчика ошибки канала динамиков  инструктора
				regBank.set(141,1);                                    // установить флаг ошибки  канала динамиков  инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки 
				myFile.print("Komanda FrontL instrukt. Off Error! - ");
				myFile.println(regcount);
			}
		                                                               // Проверить исправность канала динамиков на отсутствие наводок
		measure_volume(analog_FrontR);                                 // Измерить уровень сигнала на выходе FrontR

		if(voltage10 > volume_porog_D)                                 // Проверить исправность канала динамиков  инструктора
			{
				int regcount = regBank.get(40142);                     // адрес счетчика ошибки канала динамиков  гарнитуры инструктора
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40142,regcount);                           // адрес счетчика ошибки канала динамиков  гарнитуры инструктора
				regBank.set(142,1);                                    // установить флаг ошибки канала динамиков  гарнитуры инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки
				myFile.print("Komanda FrontR instrukt. Off Error! - ");
				myFile.println(regcount);
			}

		//-------------------------------------------------------------------------------------------------------------------------------
		 regBank.set(5,1);                                             // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
		 regBank.set(28,1);                                            // XP1- 15 HeS2Ls Отключить PTT инструктора
		 UpdateRegs();                                                 // 
		// delay(3000);




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
				Array_volume[i] = analogRead(analog);               // считываем значение
			}
		for (i = 0; i<= i_stop; i++)
			{
				volume_max = max(volume_max, Array_volume[i]);
				volume_min = min(volume_min, Array_volume[i]);
			}
		
		volume_fact = volume_max - volume_min;
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;

		Serial.print("voltage - ");
		Serial.println(voltage10);

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
void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
{
	DateTime now = RTC.now();
	int year_temp = now.year()-2000;

	itoa (year_temp,str_year_file, 10); // Преобразование даты год в строку ( 10 - десятичный формат) 

	
	if (now.month() <10)
		{
		   itoa (0,str_mon_file0, 10);                   //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
		   itoa (now.month(),str_mon_file10, 10);        //  Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);  // Сложение 2 строк
		}
	else
		{
		   itoa (now.month(),str_mon_file, 10);         // Преобразование числа в строку ( 10 - десятичный формат) 
		}


	if (now.day() <10)
		{
		   itoa (0,str_day_file0, 10);                  // Преобразование числа в строку ( 10 - десятичный формат) 
		   itoa (now.day(),str_day_file10, 10);         // Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);  // Сложение 2 строк
		}
	else
		{
		itoa (now.day(),str_day_file, 10);                   // Преобразование числа в строку ( 10 - десятичный формат) 
		}

		 
	if (file_name_count<10)
		{
			itoa (file_name_count,str0, 10);                 // Преобразование числа в строку ( 10 - десятичный формат) 
			sprintf(str_file_name_count, "%s%s", str_file_name_count0, str0);  // Сложение 2 строк
		}
	
	else
		{
			itoa (file_name_count,str_file_name_count, 10);  // Преобразование числа в строку ( 10 - десятичный формат) 
		}
	sprintf(str1, "%s%s",str_year_file, str_mon_file);       // Сложение 2 строк
	sprintf(str2, "%s%s",str1, str_day_file);                // Сложение 2 строк
	sprintf(str3, "%s%s", str2, str_file_name_count);        // Сложение 2 строк
	sprintf(file_name, "%s%s", str3, file_name_txt);         // Получение имени файла в file_name

}

void setup_mcp()
{
	// Настройка расширителя портов

 // mcp_Klava.begin(2);             // Адрес (2)первого расширителя портов клавиатура и светодиоды
  mcp_Klava.begin(2);             // Адрес (2)первого расширителя портов клавиатура и светодиоды
  mcp_Klava.pinMode(8, OUTPUT);   // Светодиод №1
  mcp_Klava.pinMode(9, OUTPUT);   // Светодиод №2
  mcp_Klava.pinMode(10, OUTPUT);  // Светодиод №3
  mcp_Klava.pinMode(11, OUTPUT);  // Светодиод №4
  mcp_Klava.pinMode(12, OUTPUT);  // Светодиод №5
  mcp_Klava.pinMode(13, OUTPUT);  // Светодиод №6
  mcp_Klava.pinMode(14, OUTPUT);  // Светодиод №7
  mcp_Klava.pinMode(15, OUTPUT);  // Светодиод №8
 
 
  mcp_Klava.pinMode(0, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(0, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(1, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(1, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(2, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(2, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(3, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(3, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(4, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(4, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(5, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(5, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(6, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(6, HIGH);      // Подключить внутренний резистор 100K к 5В.
  mcp_Klava.pinMode(7, INPUT);    // Расширитель портов на ввод
  mcp_Klava.pullUp(7, HIGH);      // Подключить внутренний резистор 100K к 5В.
  
 
  mcp_Out1.begin(4);              //  Адрес (4) второго  расширителя портов
  mcp_Out1.pinMode(0, OUTPUT);    // Реле №0  Звук    
  mcp_Out1.pinMode(1, OUTPUT);    // Реле №1  Звук    
  mcp_Out1.pinMode(2, OUTPUT);    // Реле №2  Звук    
  mcp_Out1.pinMode(3, OUTPUT);    // Реле №3  Звук    
  mcp_Out1.pinMode(4, OUTPUT);    // Реле №4  Звук   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // Реле №5  Звук    
  mcp_Out1.pinMode(6, OUTPUT);    // Реле №6  Звук   
  mcp_Out1.pinMode(7, OUTPUT);    // Реле №7  включить +12вольт  Питание платы Камертон
  
  mcp_Out1.pinMode(8, OUTPUT);    //  Реле №8 Звук на микрофон дифф.  
  mcp_Out1.pinMode(9, OUTPUT);    // Свободен J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // Свободен J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // Свободен J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  Sence     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 Sence      

	
  mcp_Out2.begin(6);              //  Адрес (6) второго  расширителя портов
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 Sence
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    Sence "Маг."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    Sence "ГГC." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    Sence "ГГ-Радио1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    Sence "ГГ-Радио2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  Адрес (5)  расширителя портов 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 Свободен
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 Свободен
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 Свободен
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 Свободен
  mcp_Analog.pullUp(0, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(1, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 Свободен Свободен Расширитель портов на ввод 
  mcp_Analog.pullUp(2, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 Свободен Расширитель портов на ввод
  mcp_Analog.pullUp(3, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(4, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(5, INPUT);   //CTS Расширитель портов на ввод
  mcp_Analog.pullUp(5, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(6, INPUT);   // DSR Расширитель портов на ввод
  mcp_Analog.pullUp(6, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(7, INPUT);   //  DCD Расширитель портов на ввод
  mcp_Analog.pullUp(7, HIGH);     // Подключить внутренний резистор 100K к 5В.

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
Присвоить объект Modbus устройства обработчик протокола
Это то, где обработчик протокола будет смотреть, чтобы читать и писать
зарегистрированные данные. В настоящее время протокол Modbus Slave проводник может
имеется только одно устройство возложенные на него.
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
Лучше всего, чтобы настроить регистры как типа в смежных блоках. это
обеспечивает более эффективный поиск и регистра и уменьшает количество сообщений
требуются мастера для извлечения данных.
*/
	regBank.add(1);     // Реле RL0 Звук
	regBank.add(2);     // Реле RL1 Звук
	regBank.add(3);     // Реле RL2 Звук
	regBank.add(4);     // Реле RL3 Звук  LFE  "Маг."
	regBank.add(5);     // Реле RL4 XP1 12  HeS2e   
	regBank.add(6);     // Реле RL5 Звук
	regBank.add(7);     // Реле RL6 Звук
	regBank.add(8);     // Реле RL7 Питание платы
  
	regBank.add(9);     // Реле RL8 Звук на микрофон
	regBank.add(10);     // Реле RL9 XP1 10
	regBank.add(11);    // Свободен J24 - 2 
	regBank.add(12);    // Свободен J24 - 1 
	regBank.add(13);    // XP8 - 2   Sence 
	regBank.add(14);    // XP8 - 1   PTT 
	regBank.add(15);    // XS1 - 5   PTT Мик
	regBank.add(16);    // XS1 - 6   Sence Мик
 
	regBank.add(17);    // J8-12     XP7 4 PTT2   Танг. р.
	regBank.add(18);    // XP1 - 20  HangUp  DCD
	regBank.add(19);    // J8-11     XP7 2 Sence  Танг. р.
	regBank.add(20);    // J8-23     XP7 1 PTT1 Танг. р.
	regBank.add(21);    // XP2-2     Sence "Маг."  
	regBank.add(22);    // XP5-3     Sence "ГГC."
	regBank.add(23);    // XP3-3     Sence "ГГ-Радио1."
	regBank.add(24);    // XP4-3     Sence "ГГ-Радио2."
 
	regBank.add(25);    // XP1- 19 HaSs      Sence подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.add(27);    // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS вкл
	regBank.add(29);    // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS вкл
	regBank.add(31);    // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.add(32);    // XP1- 1  HeS1Ls    Sence подкючения гарнитуры диспетчера

	regBank.add(10081);    // Адрес флагa индикации состояния сигнала CTS
	regBank.add(10082);    // Адрес флагa индикации состояния сигнала DSR
	regBank.add(10083);    // Адрес флагa индикации состояния сигнала DCD

	regBank.add(120);   // Флаг индикации возникновения любой ошибки
 
	regBank.add(121);   // Флаг счетчика ошибки сенсора Sence "ГГ-Радио1."  ok!
	regBank.add(122);   // Флаг счетчика ошибки сенсора Sence "ГГ-Радио2."  ok!
	regBank.add(123);   // Флаг счетчика ошибки сенсора подключения трубки
	regBank.add(124);   // Флаг счетчика ошибки сенсора  Sence Танг. р. ok!
	regBank.add(125);   // Флаг счетчика ошибки сенсора Sence Танг н. ok!
	regBank.add(126);   // Флаг счетчика ошибки сенсора Sence "Маг."   ok!
	regBank.add(127);   // Флаг счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
	regBank.add(128);   // Флаг счетчика ошибки сенсора гарнитуры инструктора
	regBank.add(129);   // Флаг счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
	regBank.add(130);   // Флаг счетчика ошибки сенсора гарнитуры диспетчера
	regBank.add(131);   // Флаг счетчика ошибки сенсора Sence Мик. ok!
	regBank.add(132);   // Флаг счетчика ошибки сенсора Sence "ГГC."   ok!

	regBank.add(133);   // Флаг счетчика ошибки сенсора 
	regBank.add(134);   // Флаг счетчика ошибки сенсора PTT Танг н.  ok!
	regBank.add(135);   // Флаг счетчика ошибки сенсора PTT Мик ok!

	regBank.add(136);   // Флаг счетчика ошибки  PTT2 Танг. р. ok!
	regBank.add(137);   // Флаг счетчика ошибки  HangUp  DCD  ok!
	regBank.add(138);   // Флаг счетчика ошибки  PTT1 Танг. р. ok!
	regBank.add(139);   // Флаг ошибки отключения микрофона гарнитуры инструктора
	regBank.add(140);   // Флаг ошибки отключения PTT гарнитуры инструктора 
	regBank.add(141);   // Флаг ошибки динамика гарнитуры инструктора FrontL
	regBank.add(142);   // Флаг ошибки динамика гарнитуры инструктора FrontR

						 //Add Input registers 30001-30040 to the register bank

	regBank.add(30000);  // байт 0 отпр бит 0 - Камертон   бит "D"
	regBank.add(30001);  // байт 0 отпр бит 1 - Камертон   "1"
	regBank.add(30002);  // байт 0 отпр бит 2 - Камертон   "0"  
	regBank.add(30003);  // байт 0 отпр бит 3 - Камертон   "1"
	regBank.add(30004);  // байт 0 отпр бит 4 - Камертон   "0" число байт пересылки (2)
	regBank.add(30005);  // байт 0 отпр бит 5 - Камертон   "1" число байт пересылки (2)
	regBank.add(30006);  // байт 0 отпр бит 6 - Камертон   "0" число байт пересылки (2)
	regBank.add(30007);  // байт 0 отпр бит 7 - Камертон   "0"
  
	regBank.add(30008);  // байт 1 отпр бит 0 - Камертон   CRC0
	regBank.add(30009);  // байт 1 отпр бит 1 - Камертон   CRC1
	regBank.add(30010);  // байт 1 отпр бит 2 - Камертон   CRC2
	regBank.add(30011);  // байт 1 отпр бит 3 - Камертон   CRC3
	regBank.add(30012);  // байт 1 отпр бит 4 - Камертон   выключение ГГС (Mute)
	regBank.add(30013);  // байт 1 отпр бит 5 - Камертон   радиопередача
	regBank.add(30014);  // байт 1 отпр бит 6 - Камертон   чтение/ запись кода яркости
	regBank.add(30015);  // байт 1 отпр бит 7 - Камертон   "1"
  
	regBank.add(30016);  // байт 2 отпр бит 0 - Камертон   код яркости экрана
	regBank.add(30017);  // байт 2 отпр бит 1 - Камертон   код яркости экрана
	regBank.add(30018);  // байт 2 отпр бит 2 - Камертон   код яркости экрана
	regBank.add(30019);  // байт 2 отпр бит 3 - Камертон   код яркости экрана
	regBank.add(30020);  // байт 2 отпр бит 4 - Камертон   код яркости экрана
	regBank.add(30021);  // байт 2 отпр бит 5 - Камертон   код яркости экрана
	regBank.add(30022);  // байт 2 отпр бит 6 - Камертон   код яркости экрана
	regBank.add(30023);  // байт 2 отпр бит 7 - Камертон   "0" 

						 // Биты строки  полученные из  платы камертон. Получено 4 байта
  
	regBank.add(30024);  // байт 1 прием бит 0 - Камертон  флаг подключения ГГ Радио2
	regBank.add(30025);  // байт 1 прием бит 1 - Камертон  флаг подключения ГГ Радио1
	regBank.add(30026);  // байт 1 прием бит 2 - Камертон  флаг подключения трубки
	regBank.add(30027);  // байт 1 прием бит 3 - Камертон  флаг подключения ручной тангенты
	regBank.add(30028);  // байт 1 прием бит 4 - Камертон  флаг подключения педали
	regBank.add(30029);  // байт 1 прием бит 5 - Камертон   "1"
	regBank.add(30030);  // байт 1 прием бит 6 - Камертон   "0" 
	regBank.add(30031);  // байт 1 прием бит 7 - Камертон   "1"
  
	regBank.add(30032);  // байт 2 прием бит 0 - Камертон   код яркости экрана
	regBank.add(30033);  // байт 2 прием бит 1 - Камертон   код яркости экрана
	regBank.add(30034);  // байт 2 прием бит 2 - Камертон   код яркости экрана
	regBank.add(30035);  // байт 2 прием бит 3 - Камертон   код яркости экрана
	regBank.add(30036);  // байт 2 прием бит 4 - Камертон   код яркости экрана
	regBank.add(30037);  // байт 2 прием бит 5 - Камертон   код яркости экрана
	regBank.add(30038);  // байт 2 прием бит 6 - Камертон   код яркости экрана
	regBank.add(30039);  // байт 2 прием бит 7 - Камертон   "0" 
  
	regBank.add(30040);  // байт 3 прием бит 0 - Камертон   флаг подключения магнитофона
	regBank.add(30041);  // байт 3 прием бит 1 - Камертон   флаг подключения гарнитуры инструктора 2 наушниками
	regBank.add(30042);  // байт 3 прием бит 2 - Камертон   флаг подключения гарнитуры инструктора
	regBank.add(30043);  // байт 3 прием бит 3 - Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками
	regBank.add(30044);  // байт 3 прием бит 4 - Камертон   флаг подключения гарнитуры диспетчера
	regBank.add(30045);  // байт 3 прием бит 5 - Камертон   флаг подключения микрофона XS1 - 6 Sence
	regBank.add(30046);  // байт 3 прием бит 6 - Камертон   флаг подключения ГГС
	regBank.add(30047);  // байт 3 прием бит 7 - Камертон   "0" 
  
	regBank.add(30048);  // байт 4 прием бит 0 - Камертон   CRC0
	regBank.add(30049);  // байт 4 прием бит 1 - Камертон   CRC1
	regBank.add(30050);  // байт 4 прием бит 2 - Камертон   CRC2   
	regBank.add(30051);  // байт 4 прием бит 3 - Камертон   CRC3   
	regBank.add(30052);  // байт 4 прием бит 4 - Камертон   флаг выключения микрофона инструктора
	regBank.add(30053);  // байт 4 прием бит 5 - Камертон    флаг радиопередачи
	regBank.add(30054);  // байт 4 прием бит 6 - Камертон   флаг выключения микрофона диспетчера
	regBank.add(30055);  // байт 4 прием бит 7 - Камертон   "0" 



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


						 // Текущее время 
	regBank.add(40046);  // адрес день модуля часов контроллера
	regBank.add(40047);  // адрес месяц модуля часов контроллера
	regBank.add(40048);  // адрес год модуля часов контроллера
	regBank.add(40049);  // адрес час модуля часов контроллера
	regBank.add(40050);  // адрес минута модуля часов контроллера
	regBank.add(40051);  // адрес секунда модуля часов контроллера
 
						 // Установка времени в контроллере
	regBank.add(40052);  // адрес день
	regBank.add(40053);  // адрес месяц
	regBank.add(40054);  // адрес год
	regBank.add(40055);  // адрес час
	regBank.add(40056);  // адрес минута
	regBank.add(40057);  // 
	regBank.add(40058);  // 
	regBank.add(40059);  // 
	/*
	regBank.add(40060); // адрес счетчика ошибки
	regBank.add(40061); // адрес счетчика ошибки
	regBank.add(40062); // адрес счетчика ошибки
	regBank.add(40063); // адрес счетчика ошибки
	regBank.add(40064); // адрес ошибки
	regBank.add(40065); // адрес ошибки
	regBank.add(40066); // адрес ошибки
	regBank.add(40067); // адрес ошибки
	regBank.add(40068); // адрес ошибки
	regBank.add(40069); // адрес ошибки
	regBank.add(40070); // адрес ошибки
	regBank.add(40071); // адрес ошибки

	regBank.add(40072); // адрес ошибки в %
	regBank.add(40073); // адрес ошибки в %
	regBank.add(40074); // адрес ошибки в %
	regBank.add(40075); // адрес ошибки %
	regBank.add(40076); // адрес ошибки %
	regBank.add(40077); // адрес ошибки %
	regBank.add(40078); // адрес ошибки %
	regBank.add(40079); // адрес ошибки %
	regBank.add(40080); // адрес ошибки %
	regBank.add(40081); // адрес ошибки %
	regBank.add(40082); // адрес ошибки %
	regBank.add(40083); // адрес ошибки %

	// Время ошибки на включение
	regBank.add(40084); // адрес день adr_Mic_On_day 
	regBank.add(40085); // адрес месяц adr_Mic_On_month  
	regBank.add(40086); // адрес год adr_Mic_On_year  
	regBank.add(40087); // адрес час adr_Mic_On_hour 
	regBank.add(40088); // адрес минута adr_Mic_On_minute 
	regBank.add(40089); // адрес секунда  adr_Mic_On_second    

	// Время ошибки на выключение
	regBank.add(40090); // адрес день adr_Mic_Off_day    
	regBank.add(40091); // адрес месяц  adr_Mic_Off_month 
	regBank.add(40092); // адрес год adr_Mic_Off_year  
	regBank.add(40093); // адрес час adr_Mic_Off_hour   
	regBank.add(40094); // адрес минута adr_Mic_Off_minute   
	regBank.add(40095); // адрес секунда adr_Mic_Off_second    
	*/
	// Время старта теста
	regBank.add(40096); // адрес день  adr_Mic_Start_day    
	regBank.add(40097); // адрес месяц adr_Mic_Start_month  
	regBank.add(40098); // адрес год adr_Mic_Start_year  
	regBank.add(40099); // адрес час adr_Mic_Start_hour 
	regBank.add(40100); // адрес минута adr_Mic_Start_minute 
	regBank.add(40101); // адрес секунда adr_Mic_Start_second  

	// Время окончания теста
	regBank.add(40102); // адрес день adr_Mic_Stop_day 
	regBank.add(40103); // адрес месяц adr_Mic_Stop_month 
	regBank.add(40104); // адрес год adr_Mic_Stop_year
	regBank.add(40105); // адрес час adr_Mic_Stop_hour 
	regBank.add(40106); // адрес минута adr_Mic_Stop_minute  
	regBank.add(40107); // адрес секунда adr_Mic_Stop_second 

	// Продолжительность выполнения теста
	regBank.add(40108); // адрес день adr_Time_Test_day 
	regBank.add(40109); // адрес час adr_Time_Test_hour 
	regBank.add(40110); // адрес минута adr_Time_Test_minute
	regBank.add(40111); // адрес секунда adr_Time_Test_second
 
	regBank.add(40120); // adr_control_command Адрес передачи комманд на выполнение
	regBank.add(40121); // адрес счетчика ошибки сенсора Sence "ГГ-Радио1."  ok!
	regBank.add(40122); // адрес счетчика ошибки сенсора Sence "ГГ-Радио2."  ok!
	regBank.add(40123); // адрес счетчика ошибки сенсора подключения трубки
	regBank.add(40124); // адрес счетчика ошибки сенсора  Sence Танг. р. ok!
	regBank.add(40125); // адрес счетчика ошибки сенсора Sence Танг н. ok!
	regBank.add(40126); // адрес счетчика ошибки сенсора Sence "Маг."   ok!
	regBank.add(40127); // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
	regBank.add(40128); // адрес счетчика ошибки сенсора гарнитуры инструктора
	regBank.add(40129); // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками

	regBank.add(40130); // адрес счетчика ошибки сенсора гарнитуры диспетчера
	regBank.add(40131); // адрес счетчика ошибки сенсора Sence Мик. ok!
	regBank.add(40132); // адрес счетчика ошибки сенсора Sence "ГГC."   ok!
	regBank.add(40133); // адрес счетчика ошибки сенсора 
	regBank.add(40134); // адрес счетчика ошибки сенсора PTT Танг н.  ok!
	regBank.add(40135); // адрес счетчика ошибки сенсора PTT Мик ok!
	regBank.add(40136); // адрес счетчика ошибки  PTT2 Танг. р. ok!
	regBank.add(40137); // адрес счетчика ошибки  HangUp  DCD  ok!
	regBank.add(40138); // адрес счетчика ошибки  PTT1 Танг. р. ok!
	regBank.add(40139); // адрес счетчика ошибки отключения микрофона гарнитуры инструктора 

	regBank.add(40140); // адрес счетчика ошибки отключения PTT гарнитуры инструктора
	regBank.add(40141); // адрес счетчика ошибки динамика гарнитуры инструктора FrontL
	regBank.add(40142); // адрес счетчика ошибки динамика гарнитуры инструктора FrontR

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
	Serial.begin(9600);                             // Подключение к USB ПК
	Serial1.begin(115200);                          // Подключение к звуковому модулю Камертон
	//slave.setBaud(57600);   
	slave.setSerial(2,57600);                       // Подключение к протоколу MODBUS компьютера Serial2 
	Serial3.begin(115200);                          // Свободен
	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println(" ");
	Wire.begin();
	if (!RTC.begin())                               // Настройка часов 
		{
			Serial.println("RTC failed");
			while(1);
		};
	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0); // Занести данные о времени в строку "set_time"
	// RTC.adjust(set_time);                                // Записа
	serial_print_date();
	Serial.println(" ");
	setup_mcp();                                    // Настроить порты расширения  
	mcp_Analog.digitalWrite(DTR, HIGH);             // Разрешение вывода (обмена)информации с Камертоном
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	pinMode(ledPin13, OUTPUT);  
	pinMode(ledPin12, OUTPUT);  
	pinMode(ledPin11, OUTPUT);  
	pinMode(ledPin10, OUTPUT);  
	setup_resistor();                               // Начальные установки резистора
	setup_SD();
	if (!SD.begin(SD_Select))                       // Подключить SD память
		{
			Serial.println("initialization SD failed!");
		}
	SdFile::dateTimeCallback(dateTime);             // Настройка времени записи файла
	//setup_SDFile();                                // Проверка рабрты с файлами
	setup_regModbus();                              // Настройка регистров MODBUS

	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме
	regs_in[0]= 0x00;                               // Код первого байта 
	regs_in[1]= 0x00;                               // 
	regs_in[2]= 0x00;                               // 
	regs_in[3]= 0x00;                               // 
//	reg_Kamerton();
	regBank.set(8,1);                               // Включить питание Камертон
//	sence_all_off();
	UpdateRegs();                                   // Обновить информацию в регистрах

	#if FASTADC                                     // Ускорить считывание аналогового канала
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif

	for (int i = 120; i < 142; i++)                  // Очистить флаги ошибок
	{
	   regBank.set(i,0);   
	}
	
	for (unsigned int i = 40120; i < 40142; i++)     // Очистить флаги ошибок
	{
	   regBank.set(i,0);   
	}

	MsTimer2::set(30, flash_time);                   // 30ms период таймера прерывани
	MsTimer2::start();                               // Включить таймер преравания
	resistor(1, 200);                                // Установить уровень сигнала
	resistor(2, 200);                                // Установить уровень сигнала
	prer_Kmerton_On = true;                          // Разрешить прерывания на камертон
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	Serial.println(" ");
//	time_control_get();
	Serial.println("System initialization OK!.");
}

void loop()
{
	//while(prer_Kmerton_Run == true) {}  // 
	//slave.run(); 
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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
}
