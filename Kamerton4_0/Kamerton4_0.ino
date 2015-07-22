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
#include <avr/pgmspace.h>


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
unsigned int volume_porog_D = 40;                   // Максимальная величина порога при проверке исправности FrontL,FrontR
unsigned int volume_porog_L = 200;                  // Минимальная величина порога при проверке исправности FrontL,FrontR
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
int analog_mag_radio      = 3;       //
int analog_mag_phone      = 4;       //
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
const int adr_reg_ind_CTS      PROGMEM           = 10081;        // Адрес флагa индикации состояния сигнала CTS
const int adr_reg_ind_DSR      PROGMEM           = 10082;        // Адрес флагa индикации состояния сигнала DSR
const int adr_reg_ind_DCD      PROGMEM           = 10083;        // Адрес флагa индикации состояния сигнала DCD

// **************** Адреса внешней памяти для хранения даты. Применяется приформировании имени файла *************
const int adr_temp_day         PROGMEM           = 240;          // Адрес хранения переменной день
const int adr_temp_mon         PROGMEM           = 241;          // Адрес хранения переменной месяц
const int adr_temp_year        PROGMEM           = 242;          // Адрес хранения переменной год  
const int adr_file_name_count  PROGMEM           = 243;          // Адрес хранения переменной счетчика номера файла
//------------------------------------------------------------------------------------------------------------------

//*********************Работа с именем файла ******************************
char file_name[13] ;
const char file_name_txt[5] = ".txt";
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
bool test_repeat     = true;                        // Флаг повторения теста
volatile bool prer_Kmerton_Run = false;              // Флаг разрешение прерывания Камертон
#define BUFFER_SIZEK 64                             // Размер буфера Камертон не более 128 байт
unsigned char bufferK;                              // Счетчик количества принимаемых байт

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const unsigned int adr_control_command         = 40120; //Адрес передачи комманд на выполнение 

// Текущее время 
const unsigned int adr_kontrol_day        PROGMEM      = 40046; // адрес день
const unsigned int adr_kontrol_month      PROGMEM      = 40047; // адрес месяц
const unsigned int adr_kontrol_year       PROGMEM      = 40048; // адрес год
const unsigned int adr_kontrol_hour       PROGMEM      = 40049; // адрес час
const unsigned int adr_kontrol_minute     PROGMEM      = 40050; // адрес минута
const unsigned int adr_kontrol_second     PROGMEM      = 40051; // адрес секунда

// Установка времени в контроллере
const unsigned int adr_set_kontrol_day    PROGMEM      = 40052;   // адрес день
const unsigned int adr_set_kontrol_month  PROGMEM      = 40053;   // адрес месяц
const unsigned int adr_set_kontrol_year   PROGMEM      = 40054;   // адрес год
const unsigned int adr_set_kontrol_hour   PROGMEM      = 40055;   // адрес час
const unsigned int adr_set_kontrol_minute PROGMEM      = 40056;   // адрес минута

// Время старта теста
const unsigned int adr_Mic_Start_day      PROGMEM      = 40096; // адрес день
const unsigned int adr_Mic_Start_month    PROGMEM      = 40097; // адрес месяц
const unsigned int adr_Mic_Start_year     PROGMEM      = 40098; // адрес год
const unsigned int adr_Mic_Start_hour     PROGMEM      = 40099; // адрес час
const unsigned int adr_Mic_Start_minute   PROGMEM      = 40100; // адрес минута
const unsigned int adr_Mic_Start_second   PROGMEM      = 40101; // адрес секунда

// Время окончания теста
const unsigned int adr_Mic_Stop_day       PROGMEM       = 40102; // адрес день
const unsigned int adr_Mic_Stop_month     PROGMEM       = 40103; // адрес месяц
const unsigned int adr_Mic_Stop_year      PROGMEM       = 40104; // адрес год
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 40105; // адрес час
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 40106; // адрес минута
const unsigned int adr_Mic_Stop_second    PROGMEM       = 40107; // адрес секунда

// Продолжительность выполнения теста
const unsigned int adr_Time_Test_day      PROGMEM       = 40108; // адрес день
const unsigned int adr_Time_Test_hour     PROGMEM       = 40109; // адрес час
const unsigned int adr_Time_Test_minute   PROGMEM       = 40110; // адрес минута
const unsigned int adr_Time_Test_second   PROGMEM       = 40111; // адрес секунда

const unsigned int adr_set_time           PROGMEM       = 36;    // адрес флаг установки

//---------------------------Тексты файлов  ---------------------------------------------------
const char  txt_message0[]    PROGMEM            = "    Error! - ";                           
const char  txt_message1[]    PROGMEM            = "Pass";      
const char  txt_message2[]    PROGMEM            = " ****** Test sensor OFF start! ******";    
const char  txt_message3[]    PROGMEM            = " ****** Test sensor ON  start! ******";      
const char  txt_message4[]    PROGMEM            = "Signal headset instructor microphone 30mv     ON"            ;   
const char  txt_message5[]    PROGMEM            = "Microphone headset instructor signal          ON"            ;  
const char  txt_message6[]    PROGMEM            = "Command sensor OFF headset instructor 2          send!"      ;    
const char  txt_message7[]    PROGMEM            = "Command sensor OFF headset instructor            send!"      ;    
const char  txt_message8[]    PROGMEM            = "Command PTT    OFF headset instructor            send!"      ;    
const char  txt_message9[]    PROGMEM            = "Command sensor OFF microphone                    send!"      ;    

const char  txt_message10[]   PROGMEM            = "Command sensor ON  headset instructor 2          send!"      ;     
const char  txt_message11[]   PROGMEM            = "Command sensor ON  headset instructor            send!"      ;     
const char  txt_message12[]   PROGMEM            = "Command        ON  PTT headset instructor (CTS)  send!"      ;  
const char  txt_message13[]   PROGMEM            = "Signal headset dispatcher microphone 30mv     ON"            ;   
const char  txt_message14[]   PROGMEM            = "Microphone headset dispatcher signal          ON"            ;     
const char  txt_message15[]   PROGMEM            = "Command sensor OFF headset dispatcher 2          send!"      ;    
const char  txt_message16[]   PROGMEM            = "Command sensor OFF headset dispatcher            send!"      ;   
const char  txt_message17[]   PROGMEM            = "Command PTT    OFF headset dispatcher            send!"      ;    
const char  txt_message18[]   PROGMEM            = "Command sensor OFF microphone                    send!"      ;   
const char  txt_message19[]   PROGMEM            = "Command sensor ON  headset dispatcher 2          send!"      ;   

const char  txt_message20[]   PROGMEM            = "Command sensor ON  headset dispatcher            send!"      ;    
const char  txt_message21[]   PROGMEM            = "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
const char  txt_message22[]   PROGMEM            = " ****** Test headset instructor start! ******"               ; 
const char  txt_message23[]   PROGMEM            = " ****** Test headset dispatcher start! ******"               ;
const char  txt_message24[]   PROGMEM            = " ****** Test MTT start! ******"                              ;  
const char  txt_message25[]   PROGMEM            = " ****** Test tangenta nognaja start! ********"               ;  
const char  txt_message26[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ********"              ; 
const char  txt_message27[]   PROGMEM            = " "              ;







const char  txt_all_on0[]   PROGMEM            = "";       
const char  txt_all_on1[]   PROGMEM            = " ****** Test sensor ON start! ******";                           
const char  txt_all_on2[]   PROGMEM            = "Sensor MTT                          XP1- 19 HaSs                      Error! - ";                         
const char  txt_all_on3[]   PROGMEM            = "Sensor MTT                          XP1- 19 HaSs            ";
const char  txt_all_on4[]   PROGMEM            = "ON  - Pass"                                                  ;
const char  txt_all_on5[]   PROGMEM            = "Sensor Tangenta ruchnaja            XP7 - 2                           Error! - ";    
const char  txt_all_on6[]   PROGMEM            = "Sensor Tangenta ruchnaja            XP7 - 2                 ";    
const char  txt_all_on7[]   PROGMEM            = "Sensor headset dispatcher           XP1 - 1  HeS1Ls                   Error! - ";         
const char  txt_all_on8[]   PROGMEM            = "Sensor tangenta nognaja             XP8 - 2                           Error! - ";          
const char  txt_all_on9[]   PROGMEM            = "Sensor tang. nogn.                  XP8 - 2                 ";      
const char  txt_all_on10[]  PROGMEM            = "Sensor headset dispatcher           XP1 - 1  HeS1Ls         ";        
const char  txt_all_on11[]  PROGMEM            = "Sensor headset instructor 2         XP1 - 16 HeS2Rs                   Error! - ";      
const char  txt_all_on12[]  PROGMEM            = "Sensor headset instructor 2         XP1 - 16 HeS2Rs         ";                         
const char  txt_all_on13[]  PROGMEM            = "Sensor microphone                   XS1 - 6                           Error! - ";
const char  txt_all_on14[]  PROGMEM            = "Sensor headset instructor           XP1 - 13 HeS2Ls                   Error! - ";
const char  txt_all_on15[]  PROGMEM            = "Sensor headset instructor           XP1 - 13 HeS2Ls         ";    
const char  txt_all_on16[]  PROGMEM            = "Sensor microphone                   XS1 - 6                 ";    
const char  txt_all_on17[]  PROGMEM            = "Sensor headset dispatcher 2         XP1 - 13 HeS2Ls                   Error! - ";         
const char  txt_all_on18[]  PROGMEM            = "Sensor headset dispatcher 2         XP1 - 13 HeS2Ls         ";          
const char  txt_all_on19[]  PROGMEM            = "Microphone headset instructor       XP1 - 12 HeS2e                    Error! - ";        
const char  txt_all_on20[]  PROGMEM            = "Microphone headset instructor       XP1 - 12 HeS2e          ";         
const char  txt_all_on21[]  PROGMEM            = "Radioperedacha ON                                                     Error! - ";          
const char  txt_all_on22[]  PROGMEM            = "Radioperedacha                                              ";        
const char  txt_all_on23[]  PROGMEM            = "Microphone headset dispatcher ON                                      Error! - ";         
const char  txt_all_on24[]  PROGMEM            = "Microphone headset dispatcher       XP1 10                  ";          

const char  txt_all_off0[]   PROGMEM           = "";    
const char  txt_all_off1[]   PROGMEM           = " ****** Test sensor OFF start! ******" ;                           
const char  txt_all_off2[]   PROGMEM           = "Sensor MTT                          XP1- 19 HaSs                      Error! - ";                         
const char  txt_all_off3[]   PROGMEM           = "Sensor MTT                          XP1- 19 HaSs            ";
const char  txt_all_off4[]   PROGMEM           = "OFF - Pass"                                               ;                           
const char  txt_all_off5[]   PROGMEM           = "Sensor tangenta ruchnaja            XP7 - 2                           Error! - ";                         
const char  txt_all_off6[]   PROGMEM           = "Sensor tangenta ruchnaja            XP7 - 2                 ";
const char  txt_all_off7[]   PROGMEM           = "Sensor tangenta nognaja             XP8 - 2                           Error! - ";                           
const char  txt_all_off8[]   PROGMEM           = "Sensor tangenta nognaja             XP8 - 2                 ";                         
const char  txt_all_off9[]   PROGMEM           = "Sensor headset instructor 2         XP1- 16 HeS2Rs                    Error! - ";
const char  txt_all_off10[]  PROGMEM           = "Sensor headset instructor 2         XP1- 16 HeS2Rs          ";                           
const char  txt_all_off11[]  PROGMEM           = "Sensor headset instructor           XP1- 13 HeS2Ls                    Error! - ";                         
const char  txt_all_off12[]  PROGMEM           = "Sensor headset instructor           XP1- 13 HeS2Ls          ";
const char  txt_all_off13[]  PROGMEM           = "Sensor headset dispatcher 2         XP1- 13 HeS2Ls                    Error! - ";                           
const char  txt_all_off14[]  PROGMEM           = "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ";                         
const char  txt_all_off15[]  PROGMEM           = "Sensor headset dispatcher           XP1- 1  HeS1Ls                    Error! - ";
const char  txt_all_off16[]  PROGMEM           = "Sensor headset dispatcher           XP1- 1  HeS1Ls          ";                           
const char  txt_all_off17[]  PROGMEM           = "Sensor microphone                   XS1 - 6                           Error! - ";                         
const char  txt_all_off18[]  PROGMEM           = "Sensor microphone                   XS1 - 6                 ";
const char  txt_all_off19[]  PROGMEM           = "Microphone headset instructor Sw.   XP1 12 HeS2e                      Error! - ";                           
const char  txt_all_off20[]  PROGMEM           = "Microphone headset instructor Sw.   XP1 12 HeS2e            ";                         
const char  txt_all_off21[]  PROGMEM           = "Radioperedacha OFF                                                    Error! - ";
const char  txt_all_off22[]  PROGMEM           = "Radioperedacha                                              ";                           
const char  txt_all_off23[]  PROGMEM           = "Microphone headset dispatcher OFF   XP1 10                            Error! - ";                         
const char  txt_all_off24[]  PROGMEM           = "Microphone headset dispatcher                               ";
//**********************************************************************************************************
const char  txt_test_all0[]  PROGMEM           = " ****** Test headset instructor start! ******";
const char  txt_test_all1[]  PROGMEM           = "Signal headset instructor microphone 30mv     ON"            ;
const char  txt_test_all2[]  PROGMEM           = "Headset instructor Microphone                 ON                      Error! - ";
const char  txt_test_all3[]  PROGMEM           = "Microphone headset instructor                 ON                - Pass";
const char  txt_test_all4[]  PROGMEM           = "Microphone headset instructor signal          ON"            ;
const char  txt_test_all5[]  PROGMEM           = "";
const char  txt_test_all6[]  PROGMEM           = "";
const char  txt_test_all7[]  PROGMEM           = "";
const char  txt_test_all8[]  PROGMEM           = "    Error! - ";
const char  txt_test_all9[]  PROGMEM           = "Pass";

const char  txt_test_all10[]  PROGMEM          = " ****** Test tangenta nognaja start! ******";
const char  txt_test_all11[]  PROGMEM          = "Command sensor OFF nognaja ruchnaja send!"                ;
const char  txt_test_all12[]  PROGMEM          = "Command PTT1   OFF nognaja ruchnaja send!"                ;
const char  txt_test_all13[]  PROGMEM          = "Command sensor ON  nognaja ruchnaja send!"                ;
const char  txt_test_all14[]  PROGMEM          = "Command PTT1   ON  nognaja ruchnaja send!"                ;
const char  txt_test_all15[]  PROGMEM          = "Command PTT1   OFF nognaja ruchnaja (CTS)                             Error! - ";
const char  txt_test_all16[]  PROGMEM          = "Command PTT1   OFF nognaja ruchnaja (CTS)                       - Pass";
const char  txt_test_all17[]  PROGMEM          = "Command PTT1   ON  tangenta ruchnaja (CTS)                            Error! - ";
const char  txt_test_all18[]  PROGMEM          = "Command PTT1   ON  tangenta ruchnaja (CTS)                      - Pass";
const char  txt_test_all19[]  PROGMEM          = "";

const char  txt_test_all20[]  PROGMEM          = " ****** Test headset dispatcher start! ******"          ;
const char  txt_test_all21[]  PROGMEM          = "Signal microphone headset dispatcher 30mv     ON"       ;
const char  txt_test_all22[]  PROGMEM          = "Microphone headset dispatcher                 ON                      Error! - ";
const char  txt_test_all23[]  PROGMEM          = "Microphone headset dispatcher                 ON                - Pass";
const char  txt_test_all24[]  PROGMEM          = "Microphone headset dispatcher signal          ON"       ;
const char  txt_test_all25[]  PROGMEM          = "";
const char  txt_test_all26[]  PROGMEM          = "";
const char  txt_test_all27[]  PROGMEM          = "";
const char  txt_test_all28[]  PROGMEM          = "";
const char  txt_test_all29[]  PROGMEM          = "";

const char  txt_test_all30[]  PROGMEM          = " ****** Test MTT start! ******"                          ;
const char  txt_test_all31[]  PROGMEM          = "Signal microphone MTT 60mv                    ON"        ;
const char  txt_test_all32[]  PROGMEM          = "Microphone MTT                                ON                      Error! - ";
const char  txt_test_all33[]  PROGMEM          = "Microphone MTT                                ON                - Pass";
const char  txt_test_all34[]  PROGMEM          = "Microphone MTT                                ON"        ;
const char  txt_test_all35[]  PROGMEM          = "";
const char  txt_test_all36[]  PROGMEM          = "";
const char  txt_test_all37[]  PROGMEM          = "";
const char  txt_test_all38[]  PROGMEM          = "";
const char  txt_test_all39[]  PROGMEM          = "";

const char  txt_test_all40[]  PROGMEM          = "Signal FrontL     OFF                             - ";
const char  txt_test_all41[]  PROGMEM          = "Signal FrontR     OFF                             - ";
const char  txt_test_all42[]  PROGMEM          = "Signal mag radio  OFF                             - ";
const char  txt_test_all43[]  PROGMEM          = "Signal mag phone  OFF                             - ";
const char  txt_test_all44[]  PROGMEM          = "Signal GGS        OFF                             - ";
const char  txt_test_all45[]  PROGMEM          = "Signal GG Radio1  OFF                             - ";
const char  txt_test_all46[]  PROGMEM          = "Signal GG Radio2  OFF                             - ";
const char  txt_test_all47[]  PROGMEM          = "Signal LineL      OFF                             - ";
const char  txt_test_all48[]  PROGMEM          = "Signal LineR      OFF                             - ";
const char  txt_test_all49[]  PROGMEM          = "    Error! - ";

const char  txt_test_all50[]  PROGMEM          = "Signal FrontL     ON                              - ";
const char  txt_test_all51[]  PROGMEM          = "Signal FrontR     ON                              - ";
const char  txt_test_all52[]  PROGMEM          = "Signal mag radio  ON                              - ";
const char  txt_test_all53[]  PROGMEM          = "Signal mag phone  ON                              - ";
const char  txt_test_all54[]  PROGMEM          = "Signal GGS        ON                              - ";
const char  txt_test_all55[]  PROGMEM          = "Signal GG Radio1  ON                              - ";
const char  txt_test_all56[]  PROGMEM          = "Signal GG Radio2  ON                              - ";
const char  txt_test_all57[]  PROGMEM          = "Signal LineL      ON                              - ";
const char  txt_test_all58[]  PROGMEM          = "Signal LineR      ON                              - ";
const char  txt_test_all59[]  PROGMEM          = "Signal FrontL, FrontR  ON ";

const char  txt_test_all60[]  PROGMEM          = " ****** Test tangenta ruchnaja start! ******";
const char  txt_test_all61[]  PROGMEM          = "Command sensor OFF tangenta ruchnaja send!"               ;
const char  txt_test_all62[]  PROGMEM          = "Command PTT1   OFF tangenta ruchnaja send!"               ;
const char  txt_test_all63[]  PROGMEM          = "Command PTT2   OFF tangenta ruchnaja send!"               ;
const char  txt_test_all64[]  PROGMEM          = "Command sensor ON  tangenta ruchnaja send!"               ;
const char  txt_test_all65[]  PROGMEM          = "Command PTT1   ON  tangenta ruchnaja send!"               ;
const char  txt_test_all66[]  PROGMEM          = "Command PTT2   ON  tangenta ruchnaja send!"               ;
const char  txt_test_all67[]  PROGMEM          = "Command PTT1   OFF tangenta ruchnaja (CTS)                            Error! - ";
const char  txt_test_all68[]  PROGMEM          = "Command PTT1   OFF tangenta ruchnaja (CTS)                      - Pass";
const char  txt_test_all69[]  PROGMEM          = "Command PTT2   OFF tangenta ruchnaja (DCR)                            Error! - ";

const char  txt_test_all70[]  PROGMEM          = "Command PTT2   OFF tangenta ruchnaja (DCR)                      - Pass";
const char  txt_test_all71[]  PROGMEM          = "Command PTT1   ON  tangenta ruchnaja (CTS)                            Error! - ";
const char  txt_test_all72[]  PROGMEM          = "Command PTT1   ON  tangenta ruchnaja (CTS)                      - Pass";
const char  txt_test_all73[]  PROGMEM          = "Command PTT2   ON  tangenta ruchnaja (DCR)                            Error! - ";
const char  txt_test_all74[]  PROGMEM          = "Command PTT2   ON  tangenta ruchnaja (DCR)                      - Pass";
const char  txt_test_all75[]  PROGMEM          = "";
const char  txt_test_all76[]  PROGMEM          = "";
const char  txt_test_all77[]  PROGMEM          = "";
const char  txt_test_all78[]  PROGMEM          = "";
const char  txt_test_all79[]  PROGMEM          = "";





//**********************************************************************************************************
const char  txt_instr_off0[]  PROGMEM          = "Command sensor OFF headset instructor            send!"                   ; // OK
const char  txt_instr_off1[]  PROGMEM          = "Command sensor OFF headset instructor 2          send!"                   ; // OK
const char  txt_instr_off2[]  PROGMEM          = "Command PTT    OFF headset instructor            send!"                   ; // ok
const char  txt_instr_off3[]  PROGMEM          = "Command sensor OFF headset instructor 2                               Error! - ";
const char  txt_instr_off4[]  PROGMEM          = "Command sensor OFF headset instructor 2                         - Pass";
const char  txt_instr_off5[]  PROGMEM          = "Command sensor OFF headset instructor                                 Error! - ";
const char  txt_instr_off6[]  PROGMEM          = "Command sensor OFF headset instructor                           - Pass";
const char  txt_instr_off7[]  PROGMEM          = "Command sensor OFF microphone                                         Error! - ";
const char  txt_instr_off8[]  PROGMEM          = "Command sensor OFF microphone                                   - Pass";
const char  txt_instr_off9[]  PROGMEM          = "Command        OFF PTT headset instructor (CTS)                       Error! - ";
const char  txt_instr_off10[] PROGMEM          = "Command        OFF PTT headset instructor (CTS)                 - Pass";

const char  txt_instr_on0[]  PROGMEM           = "Command sensor ON  headset instructor            send!"                   ; // ok
const char  txt_instr_on1[]  PROGMEM           = "Command sensor ON  headset instructor 2          send!"                   ; // ok
const char  txt_instr_on2[]  PROGMEM           = "Command sensor ON  headset instructor 2                               Error! - ";
const char  txt_instr_on3[]  PROGMEM           = "Command sensor ON  headset instructor 2                         - Pass";
const char  txt_instr_on4[]  PROGMEM           = "Command sensor ON  headset instructor                                 Error! - ";
const char  txt_instr_on5[]  PROGMEM           = "Command sensor ON  headset instructor                           - Pass";
const char  txt_instr_on6[]  PROGMEM           = "Command        ON  PTT headset instructor (CTS)                       Error! - ";
const char  txt_instr_on7[]  PROGMEM           = "Command        ON  PTT headset instructor (CTS)                 - Pass";
const char  txt_instr_on8[]  PROGMEM           = "Command        ON  PTT headset instructor (CTS)  send!";


const char  txt_disp_off0[]  PROGMEM           = "Command sensor OFF headset dispatcher   send!";
const char  txt_disp_off1[]  PROGMEM           = "Command sensor OFF headset dispatcher 2 send!";
const char  txt_disp_off2[]  PROGMEM           = "Command PTT    OFF headset dispatcher   send!";
const char  txt_disp_off3[]  PROGMEM           = "Command sensor OFF headset dispatcher 2                               Error! - ";
const char  txt_disp_off4[]  PROGMEM           = "Command sensor OFF headset dispatcher 2                         - Pass";
const char  txt_disp_off5[]  PROGMEM           = "Command sensor OFF headset dispatcher                                 Error! - ";
const char  txt_disp_off6[]  PROGMEM           = "Command sensor OFF headset dispatcher                           - Pass";
const char  txt_disp_off7[]  PROGMEM           = "Command sensor OFF microphone                                         Error! - ";
const char  txt_disp_off8[]  PROGMEM           = "Command sensor OFF microphone                                   - Pass";
const char  txt_disp_off9[]  PROGMEM           = "Command        OFF PTT headset dispatcher (CTS)                       Error! - ";
const char  txt_disp_off10[] PROGMEM           = "Command        OFF PTT headset dispatcher (CTS)                 - Pass";

const char  txt_disp_on0[]   PROGMEM           = "Command sensor ON  headset dispatcher    send!";
const char  txt_disp_on1[]   PROGMEM           = "Command sensor ON  headset dispatcher 2  send!";
const char  txt_disp_on2[]   PROGMEM           = "Command sensor ON  headset dispatcher 2                               Error! - ";
const char  txt_disp_on3[]   PROGMEM           = "Command sensor ON  headset dispatcher 2                         - Pass";
const char  txt_disp_on4[]   PROGMEM           = "Command sensor ON  headset dispatcher                                 Error! - ";
const char  txt_disp_on5[]   PROGMEM           = "Command sensor ON  headset dispatcher                           - Pass";
const char  txt_disp_on6[]   PROGMEM           = "Command        ON  PTT headset dispatcher (CTS)                       Error! - ";
const char  txt_disp_on7[]   PROGMEM           = "Command        ON  PTT headset dispatcher (CTS)                 - Pass";
const char  txt_disp_on8[]   PROGMEM           = "Command        ON  PTT headset dispatcher (CTS) send!";

const char  txt_mtt_off0[]   PROGMEM           = "Command sensor OFF MTT                   send!";
const char  txt_mtt_off1[]   PROGMEM           = "Command PTT    OFF MTT                   send!";
const char  txt_mtt_off2[]   PROGMEM           = "Command        OFF HangUp MTT            send!";
const char  txt_mtt_off3[]   PROGMEM           = "Command sensor OFF MTT                                                Error! - ";
const char  txt_mtt_off4[]   PROGMEM           = "Command sensor OFF MTT                                          - Pass";
const char  txt_mtt_off5[]   PROGMEM           = "Command sensor OFF microphone                                         Error! - ";
const char  txt_mtt_off6[]   PROGMEM           = "Command sensor OFF microphone                                   - Pass";
const char  txt_mtt_off7[]   PROGMEM           = "Command        OFF PTT  MTT (CTS)                                     Error! - ";
const char  txt_mtt_off8[]   PROGMEM           = "Command        OFF PTT  MTT (CTS)                               - Pass";
const char  txt_mtt_off9[]   PROGMEM           = "Command        OFF PTT  MTT (DSR)                                     Error! - ";
const char  txt_mtt_off10[]  PROGMEM           = "Command        OFF PTT  MTT (DSR)                               - Pass";
const char  txt_mtt_off11[]  PROGMEM           = "Command        OFF HangUp MTT                                         Error! - ";
const char  txt_mtt_off12[]  PROGMEM           = "Command        OFF HangUp MTT                                   - Pass";


const char  txt_mtt_on0[]    PROGMEM           = "Command sensor ON  MTT                   send!";
const char  txt_mtt_on1[]    PROGMEM           = "Command        ON  PTT  MTT (CTS)        send!";
const char  txt_mtt_on2[]    PROGMEM           = "Command        ON  HangUp MTT            send!";
const char  txt_mtt_on3[]    PROGMEM           = "Command sensor ON  MTT                                                Error! - ";
const char  txt_mtt_on4[]    PROGMEM           = "Command sensor ON  MTT                                          - Pass";
const char  txt_mtt_on5[]    PROGMEM           = "Command        ON  PTT  MTT (CTS)                                     Error! - ";
const char  txt_mtt_on6[]    PROGMEM           = "Command        ON  PTT  MTT (CTS)                               - Pass";
const char  txt_mtt_on7[]    PROGMEM           = "Command        ON  PTT  MTT (DSR)                                     Error! - ";
const char  txt_mtt_on8[]    PROGMEM           = "Command        ON  PTT  MTT (DSR)                               - Pass";
const char  txt_mtt_on9[]    PROGMEM           = "Command        ON  HangUp MTT                                         Error! - ";
const char  txt_mtt_on10[]   PROGMEM           = "Command        ON  HangUp MTT                                   - Pass";




const char  txt_error0[]  PROGMEM              = "Sensor MTT                          XP1- 19 HaSs            OFF - ";
const char  txt_error1[]  PROGMEM              = "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
const char  txt_error2[]  PROGMEM              = "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
const char  txt_error3[]  PROGMEM              = "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
const char  txt_error4[]  PROGMEM              = "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
const char  txt_error5[]  PROGMEM              = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          OFF - "; 
const char  txt_error6[]  PROGMEM              = "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
const char  txt_error7[]  PROGMEM              = "Sensor microphone                   XS1 - 6                 OFF - "; 
const char  txt_error8[]  PROGMEM              = "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
const char  txt_error9[]  PROGMEM              = "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

const char  txt_error10[]  PROGMEM             = "Sensor MTT                          XP1- 19 HaSs            ON  - ";
const char  txt_error11[]  PROGMEM             = "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
const char  txt_error12[]  PROGMEM             = "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
const char  txt_error13[]  PROGMEM             = "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
const char  txt_error14[]  PROGMEM             = "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
const char  txt_error15[]  PROGMEM             = "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
const char  txt_error16[]  PROGMEM             = "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
const char  txt_error17[]  PROGMEM             = "Sensor microphone                   XS1 - 6                 ON  - "; 
const char  txt_error18[]  PROGMEM             = "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
const char  txt_error19[]  PROGMEM             = "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
  
const char  txt_error20[]  PROGMEM             = "Command PTT headset instructor (CTS)                        OFF - ";
const char  txt_error21[]  PROGMEM             = "Command PTT headset instructor (CTS)                        ON  - ";
const char  txt_error22[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                        OFF - ";
const char  txt_error23[]  PROGMEM             = "Command PTT headset dispatcher (CTS)                        ON  - ";
const char  txt_error24[]  PROGMEM             = "Test headset instructor ** Signal LineL                     ON  - ";
const char  txt_error25[]  PROGMEM             = "Test headset instructor ** Signal LineR                     ON  - ";   
const char  txt_error26[]  PROGMEM             = "Test headset instructor ** Signal Mag phone                 ON  - ";
const char  txt_error27[]  PROGMEM             = "Test headset dispatcher ** Signal LineL                     ON  - ";
const char  txt_error28[]  PROGMEM             = "Test headset dispatcher ** Signal LineR                     ON  - ";  
const char  txt_error29[]  PROGMEM             = "Test headset dispatcher ** Signal Mag phone                 ON  - ";

const char  txt_error30[]  PROGMEM             = "Test headset instructor ** Signal FrontL                    OFF - ";
const char  txt_error31[]  PROGMEM             = "Test headset instructor ** Signal FrontR                    OFF - ";
const char  txt_error32[]  PROGMEM             = "Test headset instructor ** Signal LineL                     OFF - ";
const char  txt_error33[]  PROGMEM             = "Test headset instructor ** Signal LineR                     OFF - ";
const char  txt_error34[]  PROGMEM             = "Test headset instructor ** Signal mag radio                 OFF - ";
const char  txt_error35[]  PROGMEM             = "Test headset instructor ** Signal mag phone                 OFF - ";
const char  txt_error36[]  PROGMEM             = "Test headset instructor ** Signal GGS                       OFF - ";
const char  txt_error37[]  PROGMEM             = "Test headset instructor ** Signal GG Radio1                 OFF - ";
const char  txt_error38[]  PROGMEM             = "Test headset instructor ** Signal GG Radio2                 OFF - ";
const char  txt_error39[]  PROGMEM             = "";

const char  txt_error40[]  PROGMEM             = "Test headset dispatcher ** Signal FrontL                    OFF - ";
const char  txt_error41[]  PROGMEM             = "Test headset dispatcher ** Signal FrontR                    OFF - ";
const char  txt_error42[]  PROGMEM             = "Test headset dispatcher ** Signal LineL                     OFF - ";
const char  txt_error43[]  PROGMEM             = "Test headset dispatcher ** Signal LineR                     OFF - ";
const char  txt_error44[]  PROGMEM             = "Test headset dispatcher ** Signal mag radio                 OFF - ";
const char  txt_error45[]  PROGMEM             = "Test headset dispatcher ** Signal mag phone                 OFF - ";
const char  txt_error46[]  PROGMEM             = "Test headset dispatcher ** Signal GGS                       OFF - ";
const char  txt_error47[]  PROGMEM             = "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
const char  txt_error48[]  PROGMEM             = "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
const char  txt_error49[]  PROGMEM             = "";

const char  txt_error50[]  PROGMEM             = "";
const char  txt_error51[]  PROGMEM             = "";
const char  txt_error52[]  PROGMEM             = "";
const char  txt_error53[]  PROGMEM             = "";
const char  txt_error54[]  PROGMEM             = "";
const char  txt_error55[]  PROGMEM             = "";
const char  txt_error56[]  PROGMEM             = "";
const char  txt_error57[]  PROGMEM             = "";
const char  txt_error58[]  PROGMEM             = "";
const char  txt_error59[]  PROGMEM             = "";

const char  txt_error60[]  PROGMEM             = "";
const char  txt_error61[]  PROGMEM             = "";
const char  txt_error62[]  PROGMEM             = "";
const char  txt_error63[]  PROGMEM             = "";
const char  txt_error64[]  PROGMEM             = "";
const char  txt_error65[]  PROGMEM             = "";
const char  txt_error66[]  PROGMEM             = "";
const char  txt_error67[]  PROGMEM             = "";
const char  txt_error68[]  PROGMEM             = "";
const char  txt_error69[]  PROGMEM             = "";

const char  txt_error70[]  PROGMEM             = "";
const char  txt_error71[]  PROGMEM             = "";
const char  txt_error72[]  PROGMEM             = "";
const char  txt_error73[]  PROGMEM             = "";
const char  txt_error74[]  PROGMEM             = "";
const char  txt_error75[]  PROGMEM             = "";
const char  txt_error76[]  PROGMEM             = "";
const char  txt_error77[]  PROGMEM             = "";
const char  txt_error78[]  PROGMEM             = "";
const char  txt_error79[]  PROGMEM             = "";

const char  txt_error80[]  PROGMEM             = "";
const char  txt_error81[]  PROGMEM             = ""; // адрес счетчика ошибки включения микрофона инструктора
const char  txt_error82[]  PROGMEM             = ""; // адрес счетчика ошибки включения микрофона диспетчера
const char  txt_error83[]  PROGMEM             = "Sensor MTT                          XP1- 19 HaSs            ON  - ";
const char  txt_error84[]  PROGMEM             = "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
const char  txt_error85[]  PROGMEM             = "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
const char  txt_error86[]  PROGMEM             = "";
const char  txt_error87[]  PROGMEM             = "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
const char  txt_error88[]  PROGMEM             = "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
const char  txt_error89[]  PROGMEM             = "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";    

const char  txt_error90[]  PROGMEM             = "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
const char  txt_error91[]  PROGMEM             = "Sensor microphone                   XS1 - 6                 ON  - "; 
const char  txt_error92[]  PROGMEM             = "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - ";    
const char  txt_error93[]  PROGMEM             = "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  

char buffer[100];  

const char* const table_message[] PROGMEM = 
{
txt_message0,                                 // "    Error! - ";                           
txt_message1,                                 // "Pass";      
txt_message2,                                 // " ****** Test sensor OFF start! ******";    
txt_message3,                                 // " ****** Test sensor ON  start! ******";      
txt_message4,                                 // "Signal headset instructor microphone 30mv     ON"            ;   
txt_message5,                                 // "Microphone headset instructor signal          ON"            ;  
txt_message6,                                 // "Command sensor OFF headset instructor 2          send!"      ; 
txt_message7,                                 // "Command sensor OFF headset instructor            send!"      ;  
txt_message8,                                 // "Command PTT    OFF headset instructor            send!"      ;    
txt_message9,                                 // "Command sensor OFF microphone                    send!"      ;  
txt_message10,                                // "Command sensor ON  headset instructor 2          send!"      ;    
txt_message11,                                // "Command sensor ON  headset instructor            send!"      ;   
txt_message12,                                // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
txt_message13,                                // "Signal headset dispatcher microphone 30mv     ON"            ;   
txt_message14,                                // "Microphone headset dispatcher signal          ON"            ;     
txt_message15,                                // "Command sensor OFF headset dispatcher 2          send!"      ;  
txt_message16,                                // "Command sensor OFF headset dispatcher            send!"      ;  
txt_message17,                                // "Command PTT    OFF headset dispatcher            send!"      ;    
txt_message18,                                // "Command sensor OFF microphone                    send!"      ;   
txt_message19,                                // "Command sensor ON  headset dispatcher 2          send!"      ;  
txt_message20,                                // "Command sensor ON  headset dispatcher            send!"      ;   
txt_message21,                                // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
txt_message22,                                // " ****** Test headset instructor start! ******"               ; 
txt_message23,                                // " ****** Test headset dispatcher start! ******"               ;
txt_message24,                                // " ****** Test MTT start! ******"                              ;  
txt_message25,                                // " ****** Test tangenta nognaja start! ********"               ;  
txt_message26,                                // " ****** Test tangenta ruchnaja start! ********"              ; 
txt_message27                                 // " "              ;
};

const char* const string_table_err[] PROGMEM = 
{
txt_error0,                                   // "Sensor MTT                          XP1- 19 HaSs            OFF - ";
txt_error1,                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
txt_error2,                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
txt_error3,                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
txt_error4,                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
txt_error5,                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
txt_error6,                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
txt_error7,                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
txt_error8,                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
txt_error9,                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - "; 

txt_error10,                                  // "Sensor MTT                          XP1- 19 HaSs            ON  - ";
txt_error11,                                  // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
txt_error12,                                  // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
txt_error13,                                  // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
txt_error14,                                  // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
txt_error15,                                  // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
txt_error16,                                  // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
txt_error17,                                  // "Sensor microphone                   XS1 - 6                 ON  - "; 
txt_error18,                                  // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
txt_error19,                                  // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

txt_error20,                                  // "Command PTT headset instructor (CTS)                        OFF - ";
txt_error21,                                  // "Command PTT headset instructor (CTS)                        ON  - ";
txt_error22,                                  // "Command PTT headset dispatcher (CTS)                        OFF - ";
txt_error23,                                  // "Command PTT headset dispatcher (CTS)                        ON  - ";
txt_error24,                                  // "Test headset instructor ** Signal LineL                     ON  - ";
txt_error25,                                  // "Test headset instructor ** Signal LineR                     ON  - ";   
txt_error26,                                  // "Test headset instructor ** Signal Mag phone                 ON  - ";
txt_error27,                                  // "Test headset dispatcher ** Signal LineL                     ON  - ";
txt_error28,                                  // "Test headset dispatcher ** Signal LineR                     ON  - ";  
txt_error29,                                  // "Test headset dispatcher ** Signal Mag phone                 ON  - ";

txt_error30,                                  // "Test headset instructor ** Signal FrontL                    OFF - ";
txt_error31,                                  // "Test headset instructor ** Signal FrontR                    OFF - ";
txt_error32,                                  // "Test headset instructor ** Signal LineL                     OFF - ";
txt_error33,                                  // "Test headset instructor ** Signal LineR                     OFF - ";
txt_error34,                                  // "Test headset instructor ** Signal mag radio                 OFF - "; 
txt_error35,                                  // "Test headset instructor ** Signal mag phone                 OFF - ";
txt_error36,                                  // "Test headset instructor ** Signal GGS                       OFF - ";
txt_error37,                                  // "Test headset instructor ** Signal GG Radio1                 OFF - ";
txt_error38,                                  // "Test headset instructor ** Signal GG Radio2                 OFF - ";
txt_error39,                                  //

txt_error40,                                  // "Test headset dispatcher ** Signal FrontL                    OFF - ";
txt_error41,                                  // "Test headset dispatcher ** Signal FrontR                    OFF - ";
txt_error42,                                  // "Test headset dispatcher ** Signal LineL                     OFF - "; 
txt_error43,                                  // "Test headset dispatcher ** Signal LineR                     OFF - ";
txt_error44,                                  // "Test headset dispatcher ** Signal mag radio                 OFF - "; 
txt_error45,                                  // "Test headset dispatcher ** Signal mag phone                 OFF - ";
txt_error46,                                  // "Test headset dispatcher ** Signal GGS                       OFF - "; 
txt_error47,                                  // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
txt_error48,                                  // "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
txt_error49,                                  //  

txt_error50,                                  // 
txt_error51,                                  //  
txt_error52,                                  //  
txt_error53,                                  //  
txt_error54,                                  // 
txt_error55,                                  // 
txt_error56,                                  // 
txt_error57,                                  // 
txt_error58,                                  //  
txt_error59,                                  // 

txt_error60,                                  // 
txt_error61,                                  //
txt_error62,                                  //
txt_error63,                                  //
txt_error64,                                  //
txt_error65,                                  //
txt_error66,                                  //
txt_error67,                                  //
txt_error68,                                  //
txt_error69,                                  //

txt_error70,                                  //
txt_error71,                                  //
txt_error72,                                  //
txt_error73,                                  //
txt_error74,                                  //
txt_error75,                                  //
txt_error76,                                  //
txt_error77,                                  //  
txt_error78,                                  //  
txt_error79,                                  // 

txt_error80,                                  // "Test headset dispatcher ** Signal LineR       OFF               - ";
txt_error81,                                  // Резерв адрес счетчика ошибки включения микрофона инструктора
txt_error82,                                  // Резерв адрес счетчика ошибки включения микрофона диспетчера
txt_error83,                                  // "Sensor MTT                          XP1- 19 HaSs            ON  - ";
txt_error84,                                  // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
txt_error85,                                  // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
txt_error86,                                  // "";
txt_error87,                                  // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
txt_error88,                                  // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
txt_error89,                                  // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";    

txt_error90,                                  // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
txt_error91,                                  // "Sensor microphone                   XS1 - 6                 ON  - "; 
txt_error92,                                  // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
txt_error93                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
};

const char* const string_table[] PROGMEM = 
{
txt_all_off0,     // 0
txt_all_off1,     // 1   " ****** Test sensor OFF start! ******" ;                               
txt_all_off2,     // 2  "sensor MTT  XP1- 19 HaSs Error! - ";                         
txt_all_off3,     // 3  "sensor MTT XP1- 19 HaSs ";
txt_all_off4,     // 4   "OFF - Ok!"                                                ;                           
txt_all_off5,     // 5  "Tangenta ruchnaja XP7 2  Error! - ";                         
txt_all_off6,     // 6  "Tangenta ruchnaja XP7 - 2                                 ";
txt_all_off7,     // 7  "XP8 - 2 sensor tangenta nognaja Error! - ";                           
txt_all_off8,     // 8  "XP8 - 2 sensor tangenta nognaja ";                         
txt_all_off9,     // 9  Sensor headset headset instructor 2  XP1- 16 HeS2Rs  Error!   - ";
txt_all_off10,    // 10  "Sensor headset headset instructor 2  XP1- 16 HeS2Rs           ";                           
txt_all_off11,    // 11  "Sensor headset headset instructor XP1- 13 HeS2Ls  Error!    - ";                         
txt_all_off12,    // 12  "Sensor headset headset instructor XP1- 13 HeS2Ls              ";
txt_all_off13,    // 13  "Sensor headset dispatcher 2 XP1- 13 HeS2Ls  Error!  - ";                           
txt_all_off14,    // 14  "Sensor headset dispatcher 2 XP1- 13 HeS2Ls            ";                         
txt_all_off15,    // 15  "Sensor headset dispatcher XP1- 1  HeS1Ls Error!     - ";
txt_all_off16,    // 16  "Sensor headset dispatcher XP1- 1  HeS1Ls              ";                           
txt_all_off17,    // 17  "Sensor microphone  XS1 - 6 Error!                      - ";                         
txt_all_off18,    // 18  "Sensor microphone  XS1 - 6                               ";
txt_all_off19,    // 19  "Microphone headset instructor Sw.  XP1 12 HeS2e Error!        - ";                           
txt_all_off20,    // 20  "Microphone headset instructor Sw.  XP1 12 HeS2e                 ";                         
txt_all_off21,    // 21  "Radioperedacha OFF  Error!                             - ";
txt_all_off22,    // 22  "Radioperedacha           "                                ;                           
txt_all_off23,    // 23  "Microphone dispatcher OFF XP1 10    Error!            - ";                         
txt_all_off24     // 24  "Microphone dispatcher   "                                ;

};

const char* const string_table_On[] PROGMEM = 
{
	txt_all_on0,      // 0  "Test sensor ON start!";                    
	txt_all_on1,      // 1  " ****** Test sensor ON start! ******";               
	txt_all_on2,      // 2  "sensor MTT  XP1- 19 HaSs Error! - ";                         
	txt_all_on3,      // 3  "sensor MTT XP1- 19 HaSs";
	txt_all_on4,      // 4  "ON - Ok!"                                                 ;
	txt_all_on5,      // 5  "XP7 2 Tang. ruch . Error!                              - ";    
	txt_all_on6,      // 6  "XP7 - 2 Tang. ruch .                                     ";    
	txt_all_on7,      // 7  "Sensor headset dispatcher XP1- 1  HeS1Ls Error!     - ";         
	txt_all_on8,      // 8  "XP8 - 2 sensor tang. nogn. Error!                       - ";          
	txt_all_on9,      // 9  "XP8 - 2 sensor tang. nogn.                                ";      
	txt_all_on10,     // 10  "Sensor headset dispatcher XP1- 1  HeS1Ls              ";        
	txt_all_on11,     // 11  "Sensor headset headset instructor 2  XP1- 16 HeS2Rs  Error! - ";      
	txt_all_on12,     // 12  "Sensor headset headset instructor 2  XP1- 16 HeS2Rs           ";                         
	txt_all_on13,     // 13  "Sensor microphone  XS1 - 6 Error! -                      ";
	txt_all_on14,     // 14  "Sensor headset headset instructor XP1- 13 HeS2Ls  Error!    - ";
	txt_all_on15,     // 15  "Sensor headset headset instructor XP1- 13 HeS2Ls              ";    
	txt_all_on16,     // 16  "Sensor microphone  XS1 - 6                               ";    
	txt_all_on17,     // 17  "Sensor headset dispatcher 2 XP1- 13 HeS2Ls  Error!  - ";         
	txt_all_on18,     // 18  "Sensor headset dispatcher 2 XP1- 13 HeS2Ls            ";          
	txt_all_on19,     // 19 "Microphone headset instructor Sw. XP1 12 HeS2e Error! -          ";        
	txt_all_on20,     // 20  "Microphone headset instructor Sw. XP1 12 HeS2e                  ";         
	txt_all_on21,     // 21  "Radioperedacha ON  Error! -                              ";          
	txt_all_on22,     // 22  "Radioperedacha                                           ";        
	txt_all_on23,     // 23  "Microphone dispatcher ON Error! -                       ";         
	txt_all_on24      // 24  "Microphone dispatcher XP1 10                            ";   
};

const char* const table_instr_off[] PROGMEM = 
{
	txt_instr_off0,      //  "Command sensor OFF headset instructor   send!                   ";
	txt_instr_off1,      //  "Command sensor OFF headset instructor 2 send!                   ";
	txt_instr_off2,      //  "Command PTT headset instructor OFF      send!                  ";
	txt_instr_off3,      //  "Command sensor OFF headset instructor 2                Error! - ";
	txt_instr_off4,      //  "Command sensor OFF headset instructor 2 - Ok!                   ";
	txt_instr_off5,      //  "Command sensor OFF headset instructor                  Error! - ";
	txt_instr_off6,      //  "Command sensor OFF headset instructor                         - Ok!";
	txt_instr_off7,      //  "Command sensor microphone OFF                   Error! - ";
	txt_instr_off8,      //  "Command sensor microphone OFF                          - Ok!";
	txt_instr_off9,      //  "Command PTT headset instructor (CTS) OFF              Error! - ";
	txt_instr_off10      //  "Command PTT headset instructor (CTS) OFF                     - Ok!";
};

const char* const table_instr_on[] PROGMEM = 
{
	txt_instr_on0,      // "Command sensor ON headset instructor    send!"                   ;
	txt_instr_on1,      // "Command sensor ON headset instructor 2  send!"                   ;
	txt_instr_on2,      // "Command sensor ON headset instructor 2                 Error! - ";
	txt_instr_on3,      // "Command sensor ON headset instructor 2                        - Ok!";
	txt_instr_on4,      // "Command sensor ON headset instructor                   Error! - ";
	txt_instr_on5,      // "Command sensor ON headset instructor                          - Ok!";
	txt_instr_on6,      // "Command PTT headset instructor(CTS) ON                Error! - ";
	txt_instr_on7,      // "Command PTT headset instructor (CTS) ON                      - Ok!";
	txt_instr_on8       // "Command PTT headset instructor (CTS)  send!"                    ;
};

const char* const table_disp_off[] PROGMEM = 
{
	txt_disp_off0,      //  "Command sensor OFF dispatcher   send!                   ";
	txt_disp_off1,      //  "Command sensor OFF dispatcher 2 send!                   ";
	txt_disp_off2,      //  "Command PTT dispatcher OFF      send!                  ";
	txt_disp_off3,      //  "Command sensor OFF dispatcher 2                Error! - ";
	txt_disp_off4,      //  "Command sensor OFF dispatcher 2 - Ok!                   ";
	txt_disp_off5,      //  "Command sensor OFF dispatcher                  Error! - ";
	txt_disp_off6,      //  "Command sensor OFF dispatcher                         - Ok!";
	txt_disp_off7,      //  "Command sensor microphone OFF                   Error! - ";
	txt_disp_off8,      //  "Command sensor microphone OFF                          - Ok!";
	txt_disp_off9,      //  "Command PTT   OFF dispatcher (CTS)            Error! - ";
	txt_disp_off10      //  "Command PTT   OFF dispatcher (CTS)                   - Ok!";
};

const char* const table_disp_on[] PROGMEM = 
{
	txt_disp_on0,      // "Command sensor ON dispatcher    send!"                   ;
	txt_disp_on1,      // "Command sensor ON dispatcher 2  send!"                   ;
	txt_disp_on2,      // "Command sensor ON dispatcher 2                 Error! - ";
	txt_disp_on3,      // "Command sensor ON dispatcher 2                        - Ok!";
	txt_disp_on4,      // "Command sensor ON dispatcher                   Error! - ";
	txt_disp_on5,      // "Command sensor ON dispatcher                          - Ok!";
	txt_disp_on6,      // "Command PTT dispatcher OFF                    Error! - ";
	txt_disp_on7,      // "Command PTT dispatcher  OFF                          - Ok!";
	txt_disp_on8       // "Command PTT dispatcher (CTS)  send!"                    ;
};

const char* const table_mtt_off[] PROGMEM = 
{
	txt_mtt_off0,      // "Command sensor OFF MTT            send!                  ";
	txt_mtt_off1,      // "Command PTT   OFF MTT            send!                  ";
	txt_mtt_off2,      // "Command       OFF HangUp MTT     send!"                  ;
	txt_mtt_off3,      // "Command sensor OFF MTT                          Error! - ";
	txt_mtt_off4,      // "Command sensor OFF MTT                                 - Ok!";
	txt_mtt_off5,      // "Command sensor OFF microphone                   Error! - ";
	txt_mtt_off6,      // "Command sensor OFF microphone                          - Ok!";
	txt_mtt_off7,      // "Command       OFF PTT  MTT (CTS)               Error! - ";
	txt_mtt_off8,      // "Command       OFF PTT  MTT (CTS)                      - Ok!";
	txt_mtt_off9,      // "Command       OFF PTT  MTT (DSR)               Error! - ";
	txt_mtt_off10,     // "Command       OFF PTT  MTT (DSR)                      - Ok!";
	txt_mtt_off11,     // "Command       OFF HangUp MTT                   Error! - ";
	txt_mtt_off12      // "Command       OFF HangUp MTT                          - Ok!";
};

const char* const table_mtt_on[] PROGMEM = 
{
	txt_mtt_on0,      // "Command sensor ON  MTT            send!"                  ;
	txt_mtt_on1,      // "Command       ON  PTT  MTT (CTS) send!"                  ;
	txt_mtt_on2,      // "Command       ON  HangUp MTT     send!"                  ;
	txt_mtt_on3,      // "Command sensor ON  MTT                          Error! - ";
	txt_mtt_on4,      // "Command sensor ON  MTT                                 - Ok!";
	txt_mtt_on5,      // "Command       ON  PTT  MTT (CTS)               Error! - ";
	txt_mtt_on6,      // "Command       ON  PTT  MTT (CTS)                      - Ok!";
	txt_mtt_on7,      // "Command       ON  PTT  MTT (DSR)               Error! - ";
	txt_mtt_on8,      // "Command       ON  PTT  MTT (DSR)                      - Ok!";
	txt_mtt_on9,      // "Command       ON  HangUp MTT                   Error! - ";
	txt_mtt_on10      // "Command       ON  HangUp MTT                          - Ok!";
};

const char* const table_txt_all[] PROGMEM = 
{
	txt_test_all0,      // 0 "Test headset instructor start!"                                 ;
	txt_test_all1,      // 1 "Signal microphone  headset instructor 30mv  ON"                 ;
	txt_test_all2,      // 2 "Microphone headset instructor ON                      Error! - ";
	txt_test_all3,      // 3 "Microphone headset instructor  ON                         - Ok!";
	txt_test_all4,      // 4 "Microphone headset instructor signal ON"                        ;
	txt_test_all5,      // ;
	txt_test_all6,      // ;
	txt_test_all7,      // ;
	txt_test_all8,      // ; "    Error! - ";
	txt_test_all9,      // ; "Pass";

	txt_test_all10,     // 
	txt_test_all11,     //
	txt_test_all12,     // 
	txt_test_all13,     //
	txt_test_all14,     //
	txt_test_all15,     //
	txt_test_all16,     //
	txt_test_all17,     // ;
	txt_test_all18,     // ;
	txt_test_all19,     // ;

	txt_test_all20,     //  "Test dispatcher start!"                                 ;
	txt_test_all21,     //  "Signal microphone dispatcher 30mv  ON"                  ;
	txt_test_all22,     //  "Microphone dispatcher ON                      Error! - ";
	txt_test_all23,     //  "Microphone dispatcher ON                          - Ok!";
	txt_test_all24,     // ;
	txt_test_all25,     // ;
	txt_test_all26,     // ;
	txt_test_all27,     // ;
	txt_test_all28,     // ;
	txt_test_all29,     // ;

	txt_test_all30,     // ;
	txt_test_all31,     // ;
	txt_test_all32,     // ;
	txt_test_all33,     // ;
	txt_test_all34,     // ;
	txt_test_all35,     // ;
	txt_test_all36,     // ;
	txt_test_all37,     // ;
	txt_test_all38,     // ;
	txt_test_all39,     // ;

	txt_test_all40,     // "Signal FrontL OFF                                     - ";
	txt_test_all41,     // "Signal FrontR OFF                                     - ";
	txt_test_all42,     // "Signal mag radio OFF                                  - ";
	txt_test_all43,     // "Signal mag phone OFF                                  - ";
	txt_test_all44,     // "Signal GGS OFF                                        - ";
	txt_test_all45,     // "Signal GG Radio1 OFF                                  - ";
	txt_test_all46,     // "Signal GG Radio2 OFF                                  - ";
	txt_test_all47,     // "Error! - "                                               ;
	txt_test_all48,     // "";
	txt_test_all49,     // "    Error! - "; 

	txt_test_all50,     // "Signal FrontL OFF                                     - ";
	txt_test_all51,     // "Signal FrontR OFF                                     - ";
	txt_test_all52,     // "Signal FrontL ON                                      - ";
	txt_test_all53,     // "Signal mag radio OFF                                  - ";
	txt_test_all54,     // "Signal mag phone OFF                                  - ";
	txt_test_all55,     // "Signal GGS OFF                                        - ";
	txt_test_all56,     // "Signal GG Radio1 OFF                                  - ";
	txt_test_all57,     // "Signal GG Radio2 OFF                                  - ";
	txt_test_all58,     // "Signal Mag phone on                                   - ";
	txt_test_all59,     // "Signal FrontL, FrontR  ON                             - ";

	txt_test_all60,     // " ****** Test tangenta ruchnaja start! ******";
	txt_test_all61,     // "Command sensor OFF tangenta ruchnaja send!"               ;
	txt_test_all62,     // "Command PTT1   OFF tangenta ruchnaja send!"               ;
	txt_test_all63,     // "Command PTT2   OFF tangenta ruchnaja send!"               ;
	txt_test_all64,     // "Command sensor ON  tangenta ruchnaja send!"               ;
	txt_test_all65,     // "Command PTT1   ON  tangenta ruchnaja send!"               ;
	txt_test_all66,     // "Command PTT2   ON  tangenta ruchnaja send!"               ;
	txt_test_all67,     // "Command PTT1   OFF tangenta ruchnaja (CTS)      Error! - ";
	txt_test_all68,     // "Command PTT1   OFF tangenta ruchnaja (CTS)             - Ok!";
	txt_test_all69,     // "Command PTT2   OFF tangenta ruchnaja (DCR)      Error! - ";

	txt_test_all70,     // "Command PTT2   OFF tangenta ruchnaja (DCR)             - Ok!";
	txt_test_all71,     // "Command PTT1   ON  tangenta ruchnaja (CTS)      Error! - ";
	txt_test_all72,     // "Command PTT1   ON  tangenta ruchnaja (CTS)             - Ok!";
	txt_test_all73,     // "Command PTT2   ON  tangenta ruchnaja (DCR)      Error! - ";
	txt_test_all74,     // "Command PTT2   ON  tangenta ruchnaja (DCR)             - Ok!";              ;
	txt_test_all75,     // ""               ;
	txt_test_all76,     // ""               ;
	txt_test_all77,     // "";
	txt_test_all78,     // "";
	txt_test_all79      // "";
};




// ========================= Блок программ ============================================


void flash_time()                                              // Программа обработчик прерывания 
{ 
		prer_Kmerton_Run = true;
	//		digitalWrite(ledPin12,HIGH);
		prer_Kamerton();
		slave.run(); 
		//	digitalWrite(ledPin12,LOW);
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
	 regBank.set(40001,regs_out[0]);   
	 regBank.set(40002,regs_out[1]);   
	 regBank.set(40003,regs_out[2]);   
	 regBank.set(40004,regs_in[0]);   
	 regBank.set(40005,regs_in[1]); 
	 regBank.set(40006,regs_in[2]); 
	 regBank.set(40007,regs_in[3]); 
	 if (regBank.get(118)== 0)
		 {
			 test_repeat = false;
		 }
	else
		 {
			test_repeat = true;
		 }
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
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor Тангента ножная

	 //-----Установить бит 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT Тангента ножная

	 //-----Установить бит 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT Мик

	  //-----Установить бит 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor Мик

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
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  Танг. р.
  
	//-----Установить бит 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 Танг. р.

	 //-----Установить бит 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "Маг." 

	 //-----Установить бит 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "ГГC."

	 //-----Установить бит 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "ГГ-Радио1."

	 //-----Установить бит 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "ГГ-Радио2."

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
	regBank.set(adr_reg_ind_CTS, !mcp_Analog.digitalRead(CTS));
	regBank.set(adr_reg_ind_DSR, !mcp_Analog.digitalRead(DSR));
	regBank.set(adr_reg_ind_DCD, !mcp_Analog.digitalRead(DCD));

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
			 sensor_all_off();                                // Отключить все сенсоры
			break;
		case 2:		
			 sensor_all_on();                                 // Включить все сенсоры
				break;
		case 3:
			 test_headset_instructor();
				break;
		case 4:				
			 test_headset_dispatcher();          //
				break;
		case 5:
			 test_MTT();                  //
				break;
		case 6:				
			 test_tangR();                //
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
			 test_tangN();
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
		case 15:
			 set_rezistor();
				break;
		default:
		break;

	 }
	regBank.set(adr_control_command,0);
}

void sensor_all_off()
{
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2])));                    //  " ****** Test sensor OFF start! ******" ;      
	myFile.println(buffer);                                                         //  " ****** Test sensor OFF start! ******" ;      
	file_print_date();
	myFile.println();
	regBank.set(8,1);                                                               // Включить питание Камертон
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,0);                                                              // Микрофон диспетчера отключить
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(14,0);                                                              // XP8 - 1   PTT     Тангента ножная
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 тангента ручная DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS вкл
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];    

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки отключен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = regBank.get(40201);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(40201,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(201,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja отключен  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = regBank.get(40202);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(40202,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(202,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  отключен  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor отключен  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40205);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40205,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(205,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 отключен  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40206);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			regBank.set(40206,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(206,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor отключения микрофона
		  {
			regcount = regBank.get(40207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone отключен  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // Реле RL4 XP1 12  HeS2e   Выключение микрофона инструктора
		  {
			regcount = regBank.get(40208);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40208,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(208,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора отключен  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = regBank.get(40209);                                          // адрес счетчика ошибки Выключение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Выключение микрофона диспетчера
			regBank.set(40209,regcount);                                            // адрес счетчика ошибки Выключение микрофона диспетчера
			regBank.set(209,1);                                                     // установить флаг ошибки Выключение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. отключен  - Pass
			   }
		  }

	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void sensor_all_on()
{
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	regBank.set(8,1);                                                               // Включить питание Камертон
	regBank.set(5,1);                                                               // Микрофон инструктора включить
	regBank.set(10,1);                                                              // Микрофон диспетчера включить
	regBank.set(13,1);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];  

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = regBank.get(40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = regBank.get(40211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = regBank.get(40212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  - Pass
			  }
		  }

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
			  }
		  }

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
			  }
		  }

		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }

		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor включения микрофона
		  {
			regcount = regBank.get(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = regBank.get(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = regBank.get(40219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			regBank.set(40219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			regBank.set(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
			   }
		  }
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,0);                                                              // Микрофон диспетчера отключить
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}

void set_rezistor()
{
	int mwt = regBank.get(40010);
	resistor(1, mwt);
	resistor(2, mwt);
}

void test_headset_instructor()
{
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22])));
	myFile.println(buffer);                                                         // " ****** Test headset instructor start! ******"               ; 
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_instr_off();                                                               // Отключить реле и сенсоры, прверить отключение
	test_instr_on();                                                                // Включить необходимые сенсоры, проверить состояние
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                                // Установить уровень сигнала 30 мв
	regBank.set(2,1);                                                               // Подать сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (regBank.get(118)== false) myFile.println(buffer);                           // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,25);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,25);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,25);                                 // Измерить уровень сигнала на выходе LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,25);                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,25);                                 // Измерить уровень сигнала на выходе mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,25);                                 // Измерить уровень сигнала на выходе mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,30);                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,30);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,30);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                             //
	regBank.set(5,1);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT Включить PTT инструктора
	regBank.set(16,0);                                                              // Сенсор микрофона отключить
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(29,1);                                                              // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
	UpdateRegs();                                                                   // 
	delay(200);                                                                     //
	byte i53 = regs_in[3];                                                          // Получить текущее состояние Камертона
		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = regBank.get(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // Микрофон инструктора включен  - Pass
			  }
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    Звуковой сигнал подан на вход микрофона инструктора
	delay(20);
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,200);                                // Измерить уровень сигнала на выходе LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,200);                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,25);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,25);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,25);                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,30);                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,30);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,30);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(5,0);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Выключить микрофон инструктора)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	UpdateRegs();     

	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void test_headset_dispatcher()
 {
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23])));                   // " ****** Test headset dispatcher start! ******"               ;
	myFile.println(buffer);                                                         // " ****** Test headset dispatcher start! ******"               ;
	myFile.println();
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_disp_off();                                                                // Отключить реле и сенсоры, прверить отключение
	test_disp_on();                                                                 // Включить необходимые сенсоры, проверить состояние
	myFile.println("");

		// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                                // Установить уровень сигнала 30 мв
	regBank.set(1,1);                                                               // Подать сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (regBank.get(118)== false) myFile.println(buffer);                           // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,25);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,25);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,25);                                 // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,25);                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,25);                                 // Измерить уровень сигнала на выходе mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,25);                                 // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,30);                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,30);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,30);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                             //
	regBank.set(10,1);                                                              // Подать управляющую команду на вывод XP1 10 Включение микрофона диспетчера
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	regBank.set(16,0);                                                              // Сенсор микрофона отключить
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs();                                                                   // 
	delay(200);                                                                     //
	byte i5 = regs_in[3];                                                           // 
		if(bitRead(i5,6) == 0)                                                      // Проверка  включения микрофона диспетчера
		  {
			regcount = regBank.get(40182);                                          // адрес счетчика ошибки включения микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок включения микрофона диспетчера
			regBank.set(40182,regcount);                                            // адрес счетчика ошибки включения микрофона диспетчера
			regBank.set(182,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			resistor(1, 255);                                                       // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                                       // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[22])));
			myFile.print(buffer);                                                   // "Microphone dispatcher ON  Error! - "
			myFile.println(regcount);                                               // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[23])));
			if (test_repeat == false) myFile.println(buffer);                       //"Microphone dispatcher  ON - Ok!" Микрофон диспетчера включился
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[24])));                   // "Microphone dispatcher signal ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(20);
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,200);                                // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,200);                                // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,25);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,25);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,25);                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,30);                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,30);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,30);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";

	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   Отключить sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   Отключить  sensor подкючения гарнитуры диспетчера
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(10,0);                                                              // Подать управляющую команду на вывод XP1 10  (Выключить микрофон диспетчера)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
 }
void test_MTT()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[30])));
	myFile.println(buffer);                                               // "Test MTT start!"
	myFile.println();
	file_print_date();
	myFile.println("");
//	unsigned int regcount = 0;
	test_MTT_off();                                                       // Отключить реле и сенсоры, прверить отключение
	test_MTT_on();                                                        // Включить необходимые сенсоры, проверить состояние
	myFile.println("");
	regBank.set(25,0);                                                    //  XP1- 19 HaSs  sensor подключения трубки    MTT включить
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[0])));
	myFile.println(buffer);                                               // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD Трубку опустить
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[2])));
	myFile.println(buffer);                                               // "Command  HangUp MTT OFF send!"

	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                       // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                       // Измерить уровень сигнала на выходе FrontR 
	measure_vol_min(analog_mag_phone,40145,145,25);                       // Измерить уровень сигнала на выходе phone  
	measure_vol_min(analog_ggs ,     40146,146,25);                       // Измерить уровень сигнала на выходе GGS 
	measure_vol_min(analog_mag_radio,40144,144,25);                       // Измерить уровень сигнала на выходе mag radio  
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                     // Установить уровень сигнала 60 мв
	resistor(2, 130);                                                     // Установить уровень сигнала 60 мв
	regBank.set(3,1);                                                     // Включить сигнал на вход микрофона трубки Mic3p
	UpdateRegs();                                                         // Выполнить команду
	delay(400);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[31])));       //   
	myFile.println(buffer);                                               // "Signal microphone   MTT 60mv  ON"

	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                       // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                       // Измерить уровень сигнала на выходе FrontR 
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линия Radio +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                       // Измерить уровень сигнала на выходе mag radio  
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++

	measure_vol_min(analog_ggs,      40146,146,35);                       // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                       // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                       // Измерить уровень сигнала на выходе GG Radio2

	// ++++++++++++++++++++++++++++++++++ Проверить наличие сигнала  ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineR,    40171,171,35);                       // Измерить уровень сигнала на выходе LineR
	measure_vol_max(analog_mag_phone,40150,150,90);                       // Измерить уровень сигнала на выходе mag phone
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                     // Отключить сигнал на вход микрофона трубки Mic3p
	regBank.set(6,1);                                                     // Реле RL5. Подать звук Front L, Front R
	UpdateRegs();                                                         // Выполнить команду
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[59])));       //   
	myFile.println(buffer);                                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,      40146,146,35);                       // Измерить уровень сигнала на выходе GGS
	regBank.set(18,1);                                                    // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                         // Выполнить команду
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[2])));
	myFile.println(buffer);                                               // "Command  HangUp MTT ON send!"
	measure_vol_max(analog_ggs,      40172,172,120);                      // Измерить уровень сигнала на выходе mag phone
	regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD ON  Положить трубку
	regBank.set(26,0);                                                    // XP1- 17 HaSPTT    CTS DSR вкл. Отключить PTT MTT
	UpdateRegs();                                                         // Выполнить команду
	regBank.set(adr_control_command,0);                                   // Завершить программу    
	delay(200);
}
void test_tangR()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[60])));
	myFile.println(buffer);                                                 // "Test tangenta ruchnaja start!"
	myFile.println();
	file_print_date();
	myFile.println("");
	regBank.set(17,0);                                                      // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                      // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                      // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                           // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[61])));
	myFile.println(buffer);                                                 // "Command sensor OFF tangenta ruchnaja send!"  
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[62])));
	myFile.println(buffer);                                                 // "Command PTT1  OFF tangenta ruchnaja send!"   
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[63])));
	myFile.println(buffer);                                                 // "Command PTT2  OFF tangenta ruchnaja send!" 
	byte i50 = regs_in[0];    

	if(bitRead(i50,3) != 0)                                                 // J8-11     XP7 2 sensor тангента ручная
		{
		regcount = regBank.get(40124);                                      // адрес счетчика ошибки sensor тангента ручная
		regcount++;                                                         // увеличить счетчик ошибок sensor тангента ручная
		regBank.set(40124,regcount);                                        // адрес счетчика ошибки sensor тангента ручная
		regBank.set(124,1);                                                 // установить флаг ошибки sensor тангента ручная
		regBank.set(120,1);                                                 // установить общий флаг ошибки
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));         //
		myFile.print(buffer);                                               // 
		myFile.println(regcount);                                           // 
		}
	else
		{
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[6])));
		myFile.print(buffer);                                               // 
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
		myFile.println(buffer);                                             //  sensor тангента ручная OK!
		}

	 UpdateRegs(); 
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                               // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS
		  {
			 regcount = regBank.get(40173);                                 // адрес счетчика ошибки PTT  MTT (CTS)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40173,regcount);                                   // адрес счетчика ошибки PTT  MTT (CTS)
			 regBank.set(173,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[67])));
			 myFile.print(buffer);                                          // "Command PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";  
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[68])));// "Command PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
			 myFile.println(buffer);                                        // "Command PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
		  }

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                               // Проверка  на отключение  PTT2 тангента ручная (DSR)
		  {
			 regcount = regBank.get(40174);                                 // адрес счетчика ошибки  PTT  MTT (DSR)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40174,regcount);                                   // адрес счетчика ошибки  PTT  MTT (DSR)
			 regBank.set(174,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[69])));// "Command PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.print(buffer);                                          // "Command PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[70])));// "Command PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
			 myFile.println(buffer);                                        // "Command PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
		  }

	regBank.set(19,1);    // J8-11     XP7 2 sensor тангента ручная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[64])));
	myFile.println(buffer);                                                 // "Command sensor OFF tangenta ruchnaja send!"  
	regBank.set(17,1);    // J8-12     XP7 4 PTT2 тангента ручная DSR


	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[65])));
	myFile.println(buffer);                                                 // "Command PTT1  OFF tangenta ruchnaja send!"   
	regBank.set(20,1);    // J8-23     XP7 1 PTT1 тангента ручная CTS

	strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[66])));
	myFile.println(buffer);                                                 // "Command PTT2  OFF tangenta ruchnaja send!" 

	UpdateRegs();                                                           // Выполнить команду
	delay(400);

			if(bitRead(regs_in[0],3) == 0)                                  // J8-11     XP7 2 sensor тангента ручная
		  {
			regcount = regBank.get(40124);                                  // адрес счетчика ошибки sensor тангента ручная
			regcount++;                                                     // увеличить счетчик ошибок sensor тангента ручная
			regBank.set(40124,regcount);                                    // адрес счетчика ошибки sensor тангента ручная
			regBank.set(124,1);                                             // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                             // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[5])));
			myFile.print(buffer);                                           // 
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[6])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                        //  sensor тангента ручная OK!
		  }
	 UpdateRegs(); 
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                               // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS
		  {
			 regcount = regBank.get(40175);                                 // адрес счетчика ошибки PTT  MTT (CTS)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40175,regcount);                                   // адрес счетчика ошибки PTT  MTT (CTS)
			 regBank.set(175,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[71])));
			 myFile.print(buffer);                                          // "Command PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";  
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[72])));// "Command PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
			 myFile.println(buffer);                                        // "Command PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
		  }

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                               // Проверка  на отключение  PTT2 тангента ручная (DSR)
		  {
			 regcount = regBank.get(40176);                                 // адрес счетчика ошибки  PTT  MTT (DSR)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40176,regcount);                                   // адрес счетчика ошибки  PTT  MTT (DSR)
			 regBank.set(176,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[73])));// "Command PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.print(buffer);                                          // "Command PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_txt_all[74])));// "Command PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
			 myFile.println(buffer);                                        // "Command PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
		  }
	regBank.set(17,0);                                                      // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                      // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                      // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                           // Выполнить команду
	regBank.set(adr_control_command,0);                                     // Завершить программу    
	delay(100);
}
void test_tangN()
{

	regBank.set(adr_control_command,0);                                    // Завершить программу    
	delay(100);
}

void test_instr_off()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[7])));                    // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"                   ;  
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                              // XS1 - 6   sensor Мик
	regBank.set(1,0);                                                               // Реле RL0 Звук
	regBank.set(2,0);                                                               // Реле RL1 Звук
	regBank.set(3,0);                                                               // Реле RL2 Звук
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	regBank.set(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // Реле RL5 Звук
	regBank.set(7,0);                                                               // Реле RL6 Звук
	regBank.set(9,0);                                                               // Реле RL8 Звук на микрофон
	regBank.set(10,0);                                                              // Реле RL9 XP1 10
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(300);
	UpdateRegs(); 
//	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
//  byte i53 = regs_in[3];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры инструктора 2 наушниками
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = regBank.get(40207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40220);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40220,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(220,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  отключен  - Pass
		   }
		 }
}
void test_instr_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[10])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor    send!"
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	regBank.set(16,1);                                                              // XS1 - 6   sensor Мик
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	regBank.set(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(300);
	UpdateRegs(); 
//	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
//	byte i53 = regs_in[3];    

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
			  }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40221);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40221,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(221,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			if (test_repeat == false) myFile.println(buffer);                       // "Command PTT headset instructor (CTS)                        ON  - "  включен  - Pass
		   }
		 }
}

void test_disp_off()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[16])));                   // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"     
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    Отключить сенсор гарнитуры диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(16,0);                                                              // XS1 - 6   sensor Мик
	regBank.set(1,0);                                                               // Реле RL0 Звук
	regBank.set(2,0);                                                               // Реле RL1 Звук
	regBank.set(3,0);                                                               // Реле RL2 Звук
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	regBank.set(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // Реле RL5 Звук
	regBank.set(7,0);                                                               // Реле RL6 Звук
	regBank.set(9,0);                                                               // Реле RL8 Звук на микрофон
	regBank.set(10,0);                                                              // Реле RL9 XP1 10
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(300);
	UpdateRegs(); 
//	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
//  byte i53 = regs_in[3];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40205);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(40205,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(205,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " отключен  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  подключения гарнитуры диспетчера
		  {
			regcount = regBank.get(40206);                                          // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			regBank.set(40206,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regBank.set(206,1);                                                     // установить флаг ошибки sensor подключения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = regBank.get(40207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // Проверить отключения PTT диспетчера   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40222);                                          // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40222,regcount);                                            // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(222,1);                                                     // установить флаг  ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  отключен  - Pass
		   }
		 }
}
void test_disp_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[19])));                   // "Command sensor ON  headset dispatcher 2          send!"                   ;   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher 2          send!"                   ;    
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подключения гарнитуры диспетчера 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	regBank.set(16,1);                                                              // XS1 - 6   sensor Мик
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	regBank.set(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(300);
	UpdateRegs(); 
//	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
//	byte i53 = regs_in[3];    

	  // 3)  Проверка сенсора на подключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  включен  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера 
			regBank.set(216,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " включен  - Pass
			  }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT диспетчера   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40223);                                          // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40223,regcount);                                            // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(223,1);                                                     // установить флаг ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			if (test_repeat == false) myFile.println(buffer);                       // "Command PTT headset dispatcher (CTS)                        ON  - "  включен  - Pass
		   }
		 }
}

void test_MTT_off()
{
	  unsigned int regcount = 0;
	  regBank.set(25,1);                                                    // XP1- 19 HaSs      sensor подключения трубки    MTT  OFF       
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[0])));
	  if (test_repeat == false) myFile.println(buffer);                                               // "Command sensor OFF MTT  send!                  
	  regBank.set(26,0);                                                    // XP1- 17 HaSPTT    CTS DSR вкл. Отключить PTT MTT
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[1])));
	  if (test_repeat == false) myFile.println(buffer);                                               // "Command PTT MTT OFF send!"
	  regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[2])));
	  if (test_repeat == false) myFile.println(buffer);                                               // "Command  HangUp MTT OFF send!"
	  regBank.set(16,0);                                                    // XS1 - 6   sensor Мик
	  regBank.set(1,0);                                                     // Реле RL0 Звук
	  regBank.set(2,0);                                                     // Реле RL1 Звук
	  regBank.set(3,0);                                                     // Реле RL2 Звук
	  regBank.set(4,0);                                                     // Реле RL3 Звук  LFE  "Маг."
	  regBank.set(5,0);                                                     // Реле RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                     // Реле RL5 Звук
	  regBank.set(7,0);                                                     // Реле RL6 Звук
	  regBank.set(9,0);                                                     // Реле RL8 Звук на микрофон
	  regBank.set(10,0);                                                    // Реле RL9 XP1 10
	  UpdateRegs();                                                         // Выполнить команду отключения сенсоров
	  delay(400);                                                           // 
	  // 1)  Проверка сенсора MTT на отключение 
	  byte i5 = regs_in[0];                                                 // 
		if(bitRead(i5,2) != 0)                                              // Проверка  флага на отключение сенсора МТТ
		  {
			regcount = regBank.get(40163);                                  // адрес счетчика ошибки сенсора МТТ
			regcount++;                                                     // увеличить счетчик ошибок
			regBank.set(40163,regcount);                                    // адрес счетчика ошибки сенсора МТТ
			regBank.set(163,1);                                             // установить флаг ошибки
			regBank.set(120,1);                                             // установить общий флаг ошибки
			resistor(1, 255);                                               // Установить уровень сигнала в исходное состояние
			resistor(2, 255);                                               // Установить уровень сигнала в исходное состояние
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[3])));
			myFile.print(buffer);                                           // "Command sensor OFF МТТ Error! - "
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[4])));
			if (test_repeat == false) myFile.println(buffer);                                         // "Command sensor OFF  МТТ - Ok!"
		  }
		   UpdateRegs(); 
	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                            // Проверка  на отключение CTS MTT
		  {
			 regcount = regBank.get(40164);                                 // адрес счетчика ошибки PTT  MTT (CTS)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40164,regcount);                                   // адрес счетчика ошибки PTT  MTT (CTS)
			 regBank.set(164,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[7])));
			 myFile.print(buffer);                                          // "Command       OFF PTT  MTT (CTS)   Error! - 
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[8])));
			 if (test_repeat == false) myFile.println(buffer);                                        // "Command       OFF PTT  MTT (CTS)  - Ok!"
		  }

	 // 3)  Проверка  на отключение PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                            // Проверка  на отключение  PTT  MTT (DSR)
		  {
			 regcount = regBank.get(40165);                                 // адрес счетчика ошибки  PTT  MTT (DSR)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40165,regcount);                                   // адрес счетчика ошибки  PTT  MTT (DSR)
			 regBank.set(165,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[9])));
			 myFile.print(buffer);                                          // "Command       OFF PTT  MTT (DSR)  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[10])));
			 if (test_repeat == false) myFile.println(buffer);                                        // "Command       OFF PTT  MTT (DSR)  - Ok!"
		  }

	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                 // Проверить включение HangUp  DCD
		  {
			regcount = regBank.get(40168);                                  // адрес счетчика ошибки отключения HangUp  DCD
			regcount++;                                                     // увеличить счетчик ошибок
			regBank.set(40169,regcount);                                    // адрес счетчика ошибки отключения HangUp  DCD
			regBank.set(169,1);                                             // установить флаг ошибки отключения HangUp  DCD
			regBank.set(120,1);                                             // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[11])));
			myFile.print(buffer);                                           // "Command HangUp  DCD OFF Error! - "  
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[12])));
			if (test_repeat == false) myFile.println(buffer);                                         // "Command HangUp  DCD OFF- Ok!"
		 }
}
void test_MTT_on()
{
	unsigned int regcount = 0;
	regBank.set(25,0);                                                      //  XP1- 19 HaSs  sensor подключения трубки    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[0])));
	if (test_repeat == false) myFile.println(buffer);                                                 // "Command sensor ON MTT  send!                  
	regBank.set(26,1);                                                      // XP1- 17 HaSPTT    CTS DSR вкл. включить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[1])));
	if (test_repeat == false) myFile.println(buffer);                                                 // "Command PTT MTT ON send!"
	regBank.set(18,1);                                                      // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[2])));
	if (test_repeat == false) myFile.println(buffer);                                                 // "Command  HangUp MTT ON send!"

	UpdateRegs(); 
	delay(300);

	  // 1)  Проверка сенсора MTT на отключение 
	  byte i5 = regs_in[0];                                                 // 
		if(bitRead(i5,2) == 0)                                              // Проверка  флага на включение сенсора МТТ
		  {
			regcount = regBank.get(40170);                                  // адрес счетчика ошибки сенсора МТТ
			regcount++;                                                     // увеличить счетчик ошибок
			regBank.set(40170,regcount);                                    // адрес счетчика ошибки сенсора МТТ
			regBank.set(170,1);                                             // установить флаг ошибки
			regBank.set(120,1);                                             // установить общий флаг ошибки
			resistor(1, 255);                                               // Установить уровень сигнала в исходное состояние
			resistor(2, 255);                                               // Установить уровень сигнала в исходное состояние
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[3])));
			myFile.print(buffer);                                           // "Command sensor ON МТТ Error! - "
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[4])));
			if (test_repeat == false) myFile.println(buffer);                                         // "Command sensor ON  МТТ - Ok!"
		  }
		UpdateRegs(); 
	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                            // Проверка  на включение CTS MTT
		  {
			 regcount = regBank.get(40166);                                 // адрес счетчика ошибки PTT  MTT (CTS)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40166,regcount);                                   // адрес счетчика ошибки PTT  MTT (CTS)
			 regBank.set(166,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[7])));
			 myFile.print(buffer);                                          // "Command       ON PTT  MTT (CTS)   Error! - 
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[8])));
			if (test_repeat == false)  myFile.println(buffer);                                        // "Command       ON PTT  MTT (CTS)  - Ok!"
		  }

	 // 3)  Проверка  на отключение PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                            // Проверка  на отключение  PTT  MTT (DSR)
		  {
			 regcount = regBank.get(40167);                                 // адрес счетчика ошибки  PTT  MTT (DSR)
			 regcount++;                                                    // увеличить счетчик ошибок
			 regBank.set(40167,regcount);                                   // адрес счетчика ошибки  PTT  MTT (DSR)
			 regBank.set(167,1);                                            // установить флаг ошибки
			 regBank.set(120,1);                                            // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[9])));
			 myFile.print(buffer);                                          // "Command       OFF PTT  MTT (DSR)  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[10])));
			 if (test_repeat == false) myFile.println(buffer);                                        // "Command       OFF PTT  MTT (DSR)  - Ok!"
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                 // Проверить включение HangUp  DCD
		  {
			regcount = regBank.get(40169);                                  // адрес счетчика ошибки отключения HangUp  DCD
			regcount++;                                                     // увеличить счетчик ошибок
			regBank.set(40169,regcount);                                    // адрес счетчика ошибки отключения HangUp  DCD
			regBank.set(169,1);                                             // установить флаг ошибки отключения HangUp  DCD
			regBank.set(120,1);                                             // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[11])));
			myFile.print(buffer);                                           // "Command HangUp  DCD OFF Error! - "  
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[12])));
			if (test_repeat == false) myFile.println(buffer);                                         // "Command HangUp  DCD OFF- Ok!"
		 }
}

void measure_vol_min(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                               // Измерить уровень сигнала на выходе
		switch (adr_flagErr) 
		{
			case 230:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[30])));    // "Test headset instructor ** Signal FrontL                    OFF - ";
				break;
			case 231:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[31])));    // "Test headset instructor ** Signal FrontR                    OFF - ";
				break;
			case 232:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[32])));    // "Test headset instructor ** Signal LineL                     OFF - ";
				break;
			case 233:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[33])));    // "Test headset instructor ** Signal LineR                     OFF - ";
				break;
			case 234:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[34])));    // "Test headset instructor ** Signal mag radio                 OFF - ";
				break;
			case 235:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[35])));    // "Test headset instructor ** Signal mag phone                 OFF - ";
				break;
			case 236:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[36])));    // "Test headset instructor ** Signal GGS                       OFF - ";
				break;
			case 237:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[37])));    // "Test headset instructor ** Signal GG Radio1                 OFF - ";
				break;
			case 238:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[38])));    // "Test headset instructor ** Signal GG Radio2                 OFF - ";
				break;
			case 240:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[40])));    // "Test headset dispatcher ** Signal FrontL                    OFF - ";
				break;
			case 241:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[41])));    // "Test headset dispatcher ** Signal FrontR                    OFF - ";
				break;
			case 242:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[42])));    // "Test headset dispatcher ** Signal LineL                     OFF - ";
				break;
			case 243:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[43])));    // "Test headset dispatcher ** Signal LineR                     OFF - ";
				break;
			case 244:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[44])));    // "Test headset dispatcher ** Signal mag radio                 OFF - ";
				break;
			case 245:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[45])));    // "Test headset dispatcher ** Signal mag phone                 OFF - ";
				break;
			case 246:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[46])));    // "Test headset dispatcher ** Signal GGS                       OFF - ";
				break;
			case 247:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[47])));    // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
				break;
			case 248:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[48])));    // "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
				break;
		}
		if(voltage10 >  porogV)                                                     // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = regBank.get(adr_count);                                  // адрес счетчика ошибки 
				regcount++;                                                         // увеличить счетчик ошибок канала 
				regBank.set(adr_count,regcount);                                    // адрес счетчика ошибки канала 
				regBank.set(adr_flagErr,1);                                         // установить флаг ошибки  канала 
				regBank.set(120,1);                                                 // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.println(regcount);                                           // Показания счетчика ошибок
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.println(buffer);                                         // "Pass";
				}
			}                                     
}
void measure_vol_max(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                                  // Измерить уровень сигнала на выходе
		switch (adr_flagErr) 
		{
			case 224:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[24])));    // "Test headset instructor ** Signal LineL                     ON  - ";
				break;
			case 225:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[25])));    // "Test headset instructor ** Signal LineR                     ON  - "; 
				break;
		    case 226:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[26])));    // "Test headset instructor ** Signal Mag phone                 ON  - ";
				break;
			case 227:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[27])));    // "Test headset dispatcher ** Signal LineL                     ON  - ";
				break;
		    case 228:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[28])));    // "Test headset dispatcher ** Signal LineR                     ON  - "; 
				break;
			case 229:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[29])));    // "Test headset dispatcher ** Signal Mag phone                 ON  - ";
				break;
		}

		if(voltage10 <  porogV)                                                     // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = regBank.get(adr_count);                                  // адрес счетчика ошибки 
				regcount++;                                                         // увеличить счетчик ошибок канала 
				regBank.set(adr_count,regcount);                                    // адрес счетчика ошибки канала 
				regBank.set(adr_flagErr,1);                                         // установить флаг ошибки  канала 
				regBank.set(120,1);                                                 // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.println(regcount);                                           // Показания счетчика ошибок
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.println(buffer);                                         // "Pass";
				}
			}    
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
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  Адрес (6) второго  расширителя портов
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "Маг."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "ГГC." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "ГГ-Радио1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "ГГ-Радио2."
  
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
	regBank.add(1);     // Реле RL0 Звук  MIC1P
	regBank.add(2);     // Реле RL1 Звук  MIC2P
	regBank.add(3);     // Реле RL2 Звук  MIC3P
	regBank.add(4);     // Реле RL3 Звук  LFE  "Маг."
	regBank.add(5);     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
	regBank.add(6);     // Реле RL5 Звук Front L, Front R
	regBank.add(7);     // Реле RL6 Звук Center
	regBank.add(8);     // Реле RL7 Питание платы
  
	regBank.add(9);     // Реле RL8 Звук на микрофон
	regBank.add(10);    // Реле RL9 XP1 10 Включение микрофона диспетчера
	regBank.add(11);    // Свободен J24 - 2 
	regBank.add(12);    // Свободен J24 - 1 
	regBank.add(13);    // XP8 - 2   sensor Тангента ножная
	regBank.add(14);    // XP8 - 1   PTT Тангента ножная
	regBank.add(15);    // XS1 - 5   PTT Мик
	regBank.add(16);    // XS1 - 6   sensor Мик
 
	regBank.add(17);    // J8-12     XP7 4 PTT2   Танг. р.
	regBank.add(18);    // XP1 - 20  HangUp  DCD
	regBank.add(19);    // J8-11     XP7 2 sensor  Танг. р.
	regBank.add(20);    // J8-23     XP7 1 PTT1 Танг. р.
	regBank.add(21);    // XP2-2     sensor "Маг."  
	regBank.add(22);    // XP5-3     sensor "ГГC."
	regBank.add(23);    // XP3-3     sensor "ГГ-Радио1."
	regBank.add(24);    // XP4-3     sensor "ГГ-Радио2."
 
	regBank.add(25);    // XP1- 19 HaSs      sensor подключения трубки    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.add(27);    // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.add(29);    // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS вкл
	regBank.add(31);    // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.add(32);    // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	regBank.add(118);   // Флаг индикации многоразовой проверки
	regBank.add(119);   // 
	regBank.add(120);   // Флаг индикации возникновения любой ошибки
 


	regBank.add(200);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(201);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(202);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(203);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(204);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(205);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(206);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(207);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(208);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(209);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(210);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(211);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(212);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(213);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(214);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(215);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(216);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(217);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(218);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(219);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	regBank.add(220);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(221);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(222);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(223);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(224);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(225);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(226);                         // Флаг ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(227);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(228);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(229);                         // Флаг ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(230);                         // Флаг ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(231);                         // Флаг ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(232);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(233);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(234);                         // Флаг ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(235);                         // Флаг ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(236);                         // Флаг ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(237);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(238);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(239);                         //

	regBank.add(240);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(241);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(242);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(243);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(244);                         // Флаг ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(245);                         // Флаг ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(246);                         // Флаг ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(247);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(248);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(249);                         //  


	//regBank.add(121);   // Флаг ошибки сенсора sensor "ГГ-Радио1."  ok!
	//regBank.add(122);   // Флаг ошибки сенсора sensor "ГГ-Радио2."  ok!
	//regBank.add(123);   // Флаг ошибки сенсора подключения трубки
	//regBank.add(124);   // Флаг ошибки сенсора sensor Танг. р.      ok!
	//regBank.add(125);   // Флаг ошибки сенсора sensor Танг н.       ok!
	//regBank.add(126);   // Флаг ошибки сенсора sensor "Маг."        ok!
	//regBank.add(127);   // Флаг ошибки сенсора гарнитуры инструктора с 2 наушниками
	//regBank.add(128);   // Флаг ошибки сенсора гарнитуры инструктора
	//regBank.add(129);   // Флаг ошибки сенсора гарнитуры диспетчера с 2 наушниками

	//regBank.add(130);   // Флаг ошибки сенсора гарнитуры диспетчера
	//regBank.add(131);   // Флаг ошибки сенсора sensor Мик. ok!
	//regBank.add(132);   // Флаг ошибки сенсора sensor "ГГC."   ok!
	//regBank.add(133);   // Флаг ошибки сенсора микрофона
	//regBank.add(134);   // Флаг ошибки сенсора PTT Танг н.  ok!
	//regBank.add(135);   // Флаг ошибки сенсора PTT Мик ok!
	regBank.add(136);   // Флаг ошибки  PTT2 Танг. р. ok!
	regBank.add(137);   // Флаг ошибки  HangUp  DCD  ok!
	regBank.add(138);   // Флаг ошибки  PTT1 Танг. р. ok!
	regBank.add(139);   // Флаг ошибки отключения микрофона гарнитуры инструктора

	//regBank.add(140);   // Флаг ошибки отключения PTT гарнитуры инструктора 
	//regBank.add(141);   // Флаг ошибки динамика гарнитуры инструктора FrontL
	//regBank.add(142);   // Флаг ошибки динамика гарнитуры инструктора FrontR
	//regBank.add(143);   // Флаг ошибки сигнала  LineL гарнитуры инструктора 
	//regBank.add(144);   // Флаг ошибки инструктора analog_mag_radio
	//regBank.add(145);   // Флаг ошибки инструктора analog_mag_phone
	//regBank.add(146);   // Флаг ошибки инструктора analog_ggs OFF
	//regBank.add(147);   // Флаг ошибки инструктора analog_gg_radio1
	//regBank.add(148);   // Флаг ошибки инструктора analog_gg_radio2
	//regBank.add(149);   // Флаг ошибки инструктора Mag phone on

	//regBank.add(150);   // Флаг ошибки отключения PTT гарнитуры диспетчера 
	//regBank.add(151);   // Флаг ошибки динамика гарнитуры диспетчера FrontL
	//regBank.add(152);   // Флаг ошибки динамика гарнитуры диспетчера FrontR
	//regBank.add(153);   // Флаг ошибки сигнала  LineL гарнитуры диспетчера 
	//regBank.add(154);   // Флаг ошибки диспетчера analog_mag_radio
	//regBank.add(155);   // Флаг ошибки диспетчера analog_mag_phone
	//regBank.add(156);   // Флаг ошибки диспетчера analog_ggs OFF
	//regBank.add(157);   // Флаг ошибки диспетчера analog_gg_radio1
	//regBank.add(158);   // Флаг ошибки диспетчера analog_gg_radio2
	//regBank.add(159);   // Флаг ошибки диспетчера Mag phone on


	//regBank.add(150);   // Флаг 
	//regBank.add(151);   // Флаг ошибки включения микрофона инструктора
	//regBank.add(152);   // Флаг ошибки включения микрофона диспетчера
	//regBank.add(153);   // Флаг ошибки XP1 - 20  HangUp  DCD
	//regBank.add(154);   // Флаг ошибки sensor MTT ON
	//regBank.add(155);   // Флаг ошибки отключения микрофона гарнитуры диспетчера
	//regBank.add(156);   // Флаг ошибки отключения радиопередачи
	//regBank.add(157);   // Флаг ошибки включения радиопередачи
	//regBank.add(158);   // Флаг ошибки отключения PTT гарнитуры диспетчера
	//regBank.add(159);   // Флаг ошибки отключения микрофона гарнитуры диспетчера

	//regBank.add(160);   // Флаг ошибки динамика гарнитуры диспетчера FrontL
	//regBank.add(161);   // Флаг ошибки динамика гарнитуры диспетчера FrontR
	//regBank.add(162);   // Флаг ошибки сигнала  LineL гарнитуры диспетчера 
	regBank.add(163);   // Флаг ошибки sensor MTT OFF
	regBank.add(164);   // Флаг ошибки  MTT PTT OFF (CTS) 
	regBank.add(165);   // Флаг ошибки  MTT PTT OFF (DSR) 
	regBank.add(166);   // Флаг ошибки  MTT PTT ON  (CTS) 
	regBank.add(167);   // Флаг ошибки  MTT PTT ON  (DSR)
	regBank.add(168);   // Флаг ошибки  MTT HangUp OFF (DCD)
	regBank.add(169);   // Флаг ошибки  MTT HangUp ON  (DCD)

	regBank.add(170);   // Флаг ошибки  sensor MTT ON
	regBank.add(171);   // Флаг ошибки сигнала  LineL MTT
	regBank.add(172);   // Флаг ошибки analog_ggs ON
	regBank.add(173);   // Флаг ошибки "Command PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(174);   // Флаг ошибки "Command PTT2  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(175);   // Флаг ошибки "Command PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(176);   // Флаг ошибки "Command PTT2  ON  tangenta ruchnaja (CTS)      Error! - ";
	//regBank.add(177);   // Флаг ошибки сигнала  LineL гарнитуры инструктора 
	//regBank.add(178);   // Флаг ошибки сигнала  LineR гарнитуры инструктора 
	//regBank.add(179);   // Флаг ошибки сигнала  LineL гарнитуры диспетчера 

	//regBank.add(180);   // Флаг ошибки сигнала  LineR гарнитуры диспетчера 
	//regBank.add(181);   // Флаг ошибки включения микрофона инструктора
	//regBank.add(182);   // Флаг ошибки включения микрофона диспетчера





	regBank.add(10081);    // Адрес флагa индикации состояния сигнала CTS
	regBank.add(10082);    // Адрес флагa индикации состояния сигнала DSR
	regBank.add(10083);    // Адрес флагa индикации состояния сигнала DCD

						 //Add Input registers 30001-30040 to the register bank

	//regBank.add(30000);  // байт 0 отпр бит 0 - Камертон   бит "D"
	//regBank.add(30001);  // байт 0 отпр бит 1 - Камертон   "1"
	//regBank.add(30002);  // байт 0 отпр бит 2 - Камертон   "0"  
	//regBank.add(30003);  // байт 0 отпр бит 3 - Камертон   "1"
	//regBank.add(30004);  // байт 0 отпр бит 4 - Камертон   "0" число байт пересылки (2)
	//regBank.add(30005);  // байт 0 отпр бит 5 - Камертон   "1" число байт пересылки (2)
	//regBank.add(30006);  // байт 0 отпр бит 6 - Камертон   "0" число байт пересылки (2)
	//regBank.add(30007);  // байт 0 отпр бит 7 - Камертон   "0"
 // 
	//regBank.add(30008);  // байт 1 отпр бит 0 - Камертон   CRC0
	//regBank.add(30009);  // байт 1 отпр бит 1 - Камертон   CRC1
	//regBank.add(30010);  // байт 1 отпр бит 2 - Камертон   CRC2
	//regBank.add(30011);  // байт 1 отпр бит 3 - Камертон   CRC3
	//regBank.add(30012);  // байт 1 отпр бит 4 - Камертон   выключение ГГС (Mute)
	//regBank.add(30013);  // байт 1 отпр бит 5 - Камертон   радиопередача
	//regBank.add(30014);  // байт 1 отпр бит 6 - Камертон   чтение/ запись кода яркости
	//regBank.add(30015);  // байт 1 отпр бит 7 - Камертон   "1"
 // 
	//regBank.add(30016);  // байт 2 отпр бит 0 - Камертон   код яркости экрана
	//regBank.add(30017);  // байт 2 отпр бит 1 - Камертон   код яркости экрана
	//regBank.add(30018);  // байт 2 отпр бит 2 - Камертон   код яркости экрана
	//regBank.add(30019);  // байт 2 отпр бит 3 - Камертон   код яркости экрана
	//regBank.add(30020);  // байт 2 отпр бит 4 - Камертон   код яркости экрана
	//regBank.add(30021);  // байт 2 отпр бит 5 - Камертон   код яркости экрана
	//regBank.add(30022);  // байт 2 отпр бит 6 - Камертон   код яркости экрана
	//regBank.add(30023);  // байт 2 отпр бит 7 - Камертон   "0" 

	//					 // Биты строки  полученные из  платы камертон. Получено 4 байта
 // 
	//regBank.add(30024);  // байт 1 прием бит 0 - Камертон  флаг подключения ГГ Радио2
	//regBank.add(30025);  // байт 1 прием бит 1 - Камертон  флаг подключения ГГ Радио1
	//regBank.add(30026);  // байт 1 прием бит 2 - Камертон  флаг подключения трубки
	//regBank.add(30027);  // байт 1 прием бит 3 - Камертон  флаг подключения ручной тангенты
	//regBank.add(30028);  // байт 1 прием бит 4 - Камертон  флаг подключения педали
	//regBank.add(30029);  // байт 1 прием бит 5 - Камертон   "1"
	//regBank.add(30030);  // байт 1 прием бит 6 - Камертон   "0" 
	//regBank.add(30031);  // байт 1 прием бит 7 - Камертон   "1"
 // 
	//regBank.add(30032);  // байт 2 прием бит 0 - Камертон   код яркости экрана
	//regBank.add(30033);  // байт 2 прием бит 1 - Камертон   код яркости экрана
	//regBank.add(30034);  // байт 2 прием бит 2 - Камертон   код яркости экрана
	//regBank.add(30035);  // байт 2 прием бит 3 - Камертон   код яркости экрана
	//regBank.add(30036);  // байт 2 прием бит 4 - Камертон   код яркости экрана
	//regBank.add(30037);  // байт 2 прием бит 5 - Камертон   код яркости экрана
	//regBank.add(30038);  // байт 2 прием бит 6 - Камертон   код яркости экрана
	//regBank.add(30039);  // байт 2 прием бит 7 - Камертон   "0" 
 // 
	//regBank.add(30040);  // байт 3 прием бит 0 - Камертон   флаг подключения магнитофона
	//regBank.add(30041);  // байт 3 прием бит 1 - Камертон   флаг подключения гарнитуры инструктора 2 наушниками
	//regBank.add(30042);  // байт 3 прием бит 2 - Камертон   флаг подключения гарнитуры инструктора
	//regBank.add(30043);  // байт 3 прием бит 3 - Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками
	//regBank.add(30044);  // байт 3 прием бит 4 - Камертон   флаг подключения гарнитуры диспетчера
	//regBank.add(30045);  // байт 3 прием бит 5 - Камертон   флаг подключения микрофона XS1 - 6 sensor
	//regBank.add(30046);  // байт 3 прием бит 6 - Камертон   флаг подключения ГГС
	//regBank.add(30047);  // байт 3 прием бит 7 - Камертон   "0" 
 // 
	//regBank.add(30048);  // байт 4 прием бит 0 - Камертон   CRC0
	//regBank.add(30049);  // байт 4 прием бит 1 - Камертон   CRC1
	//regBank.add(30050);  // байт 4 прием бит 2 - Камертон   CRC2   
	//regBank.add(30051);  // байт 4 прием бит 3 - Камертон   CRC3   
	//regBank.add(30052);  // байт 4 прием бит 4 - Камертон   флаг выключения микрофона инструктора
	//regBank.add(30053);  // байт 4 прием бит 5 - Камертон    флаг радиопередачи
	//regBank.add(30054);  // байт 4 прием бит 6 - Камертон   флаг выключения микрофона диспетчера
	//regBank.add(30055);  // байт 4 прием бит 7 - Камертон   "0" 



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
	regBank.add(40010);  // 
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
	regBank.add(40096);  // адрес день  adr_Mic_Start_day    
	regBank.add(40097);  // адрес месяц adr_Mic_Start_month  
	regBank.add(40098);  // адрес год adr_Mic_Start_year  
	regBank.add(40099);  // адрес час adr_Mic_Start_hour 
	regBank.add(40100);  // адрес минута adr_Mic_Start_minute 
	regBank.add(40101);  // адрес секунда adr_Mic_Start_second  

	// Время окончания теста
	regBank.add(40102);  // адрес день adr_Mic_Stop_day 
	regBank.add(40103);  // адрес месяц adr_Mic_Stop_month 
	regBank.add(40104);  // адрес год adr_Mic_Stop_year
	regBank.add(40105);  // адрес час adr_Mic_Stop_hour 
	regBank.add(40106);  // адрес минута adr_Mic_Stop_minute  
	regBank.add(40107);  // адрес секунда adr_Mic_Stop_second 

	// Продолжительность выполнения теста
	regBank.add(40108);  // адрес день adr_Time_Test_day 
	regBank.add(40109);  // адрес час adr_Time_Test_hour 
	regBank.add(40110);  // адрес минута adr_Time_Test_minute
	regBank.add(40111);  // адрес секунда adr_Time_Test_second
 
	regBank.add(40120);  // adr_control_command Адрес передачи комманд на выполнение



	regBank.add(40200);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(40201);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(40202);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(40203);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(40204);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(40205);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(40206);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(40207);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(40208);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(40209);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(40210);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(40211);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(40212);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(40213);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(40214);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(40215);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(40216);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(40217);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(40218);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(40219);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	regBank.add(40220);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(40221);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(40222);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(40223);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(40224);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40225);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40226);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40227);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40228);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40229);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40230);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40231);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40232);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40233);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40234);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40235);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40236);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40237);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40238);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40239);                         //

	regBank.add(40240);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40241);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40242);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40243);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40244);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40245);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40246);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40247);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40248);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40249);                         //  



	//regBank.add(40121);  // адрес счетчика ошибки сенсора sensor "ГГ-Радио1."    ok!
	//regBank.add(40122);  // адрес счетчика ошибки сенсора sensor "ГГ-Радио2."    ok!
	//regBank.add(40123);  // адрес счетчика ошибки сенсора подключения трубки                   "Sensor MTT    XP1- 19 HaSs   OFF  - ";  
	//regBank.add(40124);  // адрес счетчика ошибки сенсора                                      "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	//regBank.add(40125);  // адрес счетчика ошибки сенсора                                      "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
	//regBank.add(40126);  // адрес счетчика ошибки сенсора  
	//regBank.add(40127);  // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	//regBank.add(40128);  // адрес счетчика ошибки сенсора гарнитуры инструктора                "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
	//regBank.add(40129);  // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками  "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 

	//regBank.add(40130);  // адрес счетчика ошибки сенсора гарнитуры диспетчера                 "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";  
	//regBank.add(40131);  // адрес счетчика ошибки сенсора sensor Мик.                          "Sensor microphone                   XS1 - 6                 OFF - "; 
	//regBank.add(40132);  // адрес счетчика ошибки отключения микрофона гарнитуры инструктора   "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - ";  
	//regBank.add(40133);  // адрес счетчика ошибки отключения радиопередачи                     "Radioperedacha                                              OFF - ";
	//regBank.add(40134);  // адрес счетчика ошибки отключения микрофона гарнитуры диспетчера    "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
	//regBank.add(40135);  // адрес счетчика ошибки сенсора PTT Мик ok!
	regBank.add(40136);  // адрес счетчика ошибки  PTT2 Танг. р. ok!
	regBank.add(40137);  // адрес счетчика ошибки  HangUp  DCD  ok!
	regBank.add(40138);  // адрес счетчика ошибки  PTT1 Танг. р. ok!
	regBank.add(40139);  // адрес счетчика ошибки отключения микрофона гарнитуры инструктора 

	//regBank.add(130);   // Флаг ошибки сенсора гарнитуры диспетчера
	//regBank.add(131);   // Флаг ошибки сенсора sensor Мик. ok!
	//regBank.add(132);   // Флаг ошибки сенсора sensor "ГГC."   ok!
	//regBank.add(133);   // Флаг ошибки сенсора микрофона
	//regBank.add(134);   // Флаг ошибки сенсора PTT Танг н.  ok!
	//regBank.add(135);   // Флаг ошибки сенсора PTT Мик ok!
	//regBank.add(136);   // Флаг ошибки  PTT2 Танг. р. ok!
	//regBank.add(137);   // Флаг ошибки  HangUp  DCD  ok!
	//regBank.add(138);   // Флаг ошибки  PTT1 Танг. р. ok!
	//regBank.add(139);   // Флаг ошибки отключения микрофона гарнитуры инструктора


	//regBank.add(40140);  // адрес счетчика ошибки отключения PTT гарнитуры инструктора
	//regBank.add(40141);  // адрес счетчика ошибки динамика гарнитуры инструктора FrontL
	//regBank.add(40142);  // адрес счетчика ошибки динамика гарнитуры инструктора FrontR
	//regBank.add(40143);  // адрес счетчика ошибки инструктора LineL
	//regBank.add(40144);  // адрес счетчика ошибки инструктора analog_mag_radio
	//regBank.add(40145);  // адрес счетчика ошибки инструктора analog_mag_phone
	//regBank.add(40146);  // адрес счетчика ошибки инструктора analog_ggs
	//regBank.add(40147);  // адрес счетчика ошибки инструктора analog_gg_radio1
	//regBank.add(40148);  // адрес счетчика ошибки инструктора analog_gg_radio2
	//regBank.add(40149);  // адрес счетчика ошибки инструктора Mag phone on


	//regBank.add(40150);  // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
	//regBank.add(40151);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontL
	//regBank.add(40152);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontR
	//regBank.add(40153);  // адрес счетчика ошибки диспетчера LineL
	//regBank.add(40154);  // адрес счетчика ошибки диспетчера analog_mag_radio
	//regBank.add(40155);  // адрес счетчика ошибки диспетчера analog_mag_phone
	//regBank.add(40156);  // адрес счетчика ошибки диспетчера analog_ggs
	//regBank.add(40157);  // адрес счетчика ошибки диспетчера analog_gg_radio1
	//regBank.add(40158);  // адрес счетчика ошибки диспетчера analog_gg_radio2
	//regBank.add(40159);  // адрес счетчика ошибки диспетчера Mag phone on

	//regBank.add(40160);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontL
	//regBank.add(40161);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontR
	//regBank.add(40162);  // адрес счетчика ошибки сигнала  LineL гарнитуры диспетчера 
	regBank.add(40163);  // адрес счетчика ошибки sensor MTT OFF
	regBank.add(40164);  // адрес счетчика ошибки MTT PTT OFF (CTS) 
	regBank.add(40165);  // адрес счетчика ошибки MTT PTT OFF (DSR) 
	regBank.add(40166);  // адрес счетчика ошибки MTT PTT ON  (CTS) 
	regBank.add(40167);  // адрес счетчика ошибки MTT PTT ON  (DSR)
	regBank.add(40168);  // адрес счетчика ошибки MTT HangUp OFF (DCD)
	regBank.add(40169);  // адрес счетчика ошибки MTT HangUp ON  (DCD)

	regBank.add(40170);  // адрес счетчика ошибки  sensor MTT ON
	regBank.add(40171);  // адрес счетчика ошибки сигнала  LineL MTT
	regBank.add(40172);  // адрес счетчика ошибки analog_ggs ON
	regBank.add(40173);  // адрес счетчика ошибки "Command PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40174);  // адрес счетчика ошибки "Command PTT2  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40175);  // адрес счетчика ошибки "Command PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40176);  // адрес счетчика ошибки "Command PTT2  ON  tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40177);  // адрес счетчика ошибки сигнала  LineL гарнитуры инструктора 
	regBank.add(40178);  // адрес счетчика ошибки сигнала  LineR гарнитуры инструктора 
	regBank.add(40179);  // адрес счетчика ошибки сигнала  LineL гарнитуры диспетчера 

	//regBank.add(40180);  // адрес счетчика ошибки сигнала  LineR гарнитуры диспетчера 
	//regBank.add(40181);  // адрес счетчика ошибки включения микрофона инструктора
	//regBank.add(40182);  // адрес счетчика ошибки включения микрофона диспетчера
	//regBank.add(40183);  // адрес счетчика ошибки отключения микрофона гарнитуры диспетчера
	//regBank.add(40184);  // адрес счетчика ошибки отключения микрофона гарнитуры диспетчера







	
	//regBank.add(40150);  // адрес счетчика ошибки  Mag phone on
	//regBank.add(40153);  // адрес счетчика ошибки XP1 - 20  HangUp  DCD
	//regBank.add(40154);  // адрес счетчика ошибки sensor MTT ON
	//regBank.add(40156);  // адрес счетчика ошибки отключения радиопередачи
	//regBank.add(40157);  // адрес счетчика ошибки включения радиопередачи
	//regBank.add(40158);  // адрес счетчика ошибки отключения PTT гарнитуры диспетчера



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
//	sensor_all_off();
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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
}
