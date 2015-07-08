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
const char  txt_all_on0[]   PROGMEM            = "";       
const char  txt_all_on1[]   PROGMEM            = "Test sensor ON start!"                                   ;                           
const char  txt_all_on2[]   PROGMEM            = "Sence MTT                     XP1- 19 HaSs     Error! - ";                         
const char  txt_all_on3[]   PROGMEM            = "Sence MTT                     XP1- 19 HaSs              ";
const char  txt_all_on4[]   PROGMEM            = "ON - Ok!"                                                ;
const char  txt_all_on5[]   PROGMEM            = "Sence Tangenta ruchnaja       XP7 - 2          Error! - ";    
const char  txt_all_on6[]   PROGMEM            = "Sence Tangenta ruchnaja       XP7 - 2                   ";    
const char  txt_all_on7[]   PROGMEM            = "Sensor garnitura dispetchera  XP1 - 1  HeS1Ls  Error! - ";         
const char  txt_all_on8[]   PROGMEM            = "Sence tangenta nognaja        XP8 - 2          Error! - ";          
const char  txt_all_on9[]   PROGMEM            = "Sence tang. nogn.             XP8 - 2                   ";      
const char  txt_all_on10[]  PROGMEM            = "Sence garnitura dispetchera   XP1 - 1  HeS1Ls           ";        
const char  txt_all_on11[]  PROGMEM            = "Sence garnitura instruktora 2 XP1 - 16 HeS2Rs  Error! - ";      
const char  txt_all_on12[]  PROGMEM            = "Sence garnitura instruktora 2 XP1 - 16 HeS2Rs           ";                         
const char  txt_all_on13[]  PROGMEM            = "Sence microphone              XS1 - 6          Error! - ";
const char  txt_all_on14[]  PROGMEM            = "Sence garnitura instruktora   XP1 - 13 HeS2Ls  Error! - ";
const char  txt_all_on15[]  PROGMEM            = "Sence garnitura instruktora   XP1 - 13 HeS2Ls           ";    
const char  txt_all_on16[]  PROGMEM            = "Sence microphone              XS1 - 6                   ";    
const char  txt_all_on17[]  PROGMEM            = "Sence garnitura dispetchera 2 XP1 - 13 HeS2Ls  Error! - ";         
const char  txt_all_on18[]  PROGMEM            = "Sence garnitura dispetchera 2 XP1 - 13 HeS2Ls           ";          
const char  txt_all_on19[]  PROGMEM            = "Microphone instruktora Sw.    XP1 - 12 HeS2e   Error! - ";        
const char  txt_all_on20[]  PROGMEM            = "Microphone instruktora Sw.    XP1 - 12 HeS2e            ";         
const char  txt_all_on21[]  PROGMEM            = "Radioperedacha ON                              Error! - ";          
const char  txt_all_on22[]  PROGMEM            = "Radioperedacha                                          ";        
const char  txt_all_on23[]  PROGMEM            = "Microphone dispetchera ON                      Error! - ";         
const char  txt_all_on24[]  PROGMEM            = "Microphone dispetchera        XP1 10                    ";          

const char  txt_all_off0[]   PROGMEM           = "";    
const char  txt_all_off1[]   PROGMEM           = "Test sensor OFF start!                                  ";                           
const char  txt_all_off2[]   PROGMEM           = "Sence MTT                     XP1- 19 HaSs     Error! - ";                         
const char  txt_all_off3[]   PROGMEM           = "Sence MTT                     XP1- 19 HaSs              ";
const char  txt_all_off4[]   PROGMEM           = "OFF - Ok!"                                               ;                           
const char  txt_all_off5[]   PROGMEM           = "Tangenta ruchnaja             XP7 - 2          Error! - ";                         
const char  txt_all_off6[]   PROGMEM           = "Tangenta ruchnaja             XP7 - 2                   ";
const char  txt_all_off7[]   PROGMEM           = "Sence tangenta nognaja        XP8 - 2          Error! - ";                           
const char  txt_all_off8[]   PROGMEM           = "Sence tangenta nognaja        XP8 - 2                   ";                         
const char  txt_all_off9[]   PROGMEM           = "Sence garnitura instruktora 2 XP1- 16 HeS2Rs   Error! - ";
const char  txt_all_off10[]  PROGMEM           = "Sence garnitura instruktora 2 XP1- 16 HeS2Rs            ";                           
const char  txt_all_off11[]  PROGMEM           = "Sence garnitura instruktora   XP1- 13 HeS2Ls   Error! - ";                         
const char  txt_all_off12[]  PROGMEM           = "Sence garnitura instruktora   XP1- 13 HeS2Ls            ";
const char  txt_all_off13[]  PROGMEM           = "Sence garnitura dispetchera 2 XP1- 13 HeS2Ls   Error! - ";                           
const char  txt_all_off14[]  PROGMEM           = "Sence garnitura dispetchera 2 XP1- 13 HeS2Ls            ";                         
const char  txt_all_off15[]  PROGMEM           = "Sence garnitura dispetchera   XP1- 1  HeS1Ls   Error! - ";
const char  txt_all_off16[]  PROGMEM           = "Sence garnitura dispetchera   XP1- 1  HeS1Ls            ";                           
const char  txt_all_off17[]  PROGMEM           = "Sence microphone              XS1 - 6          Error! - ";                         
const char  txt_all_off18[]  PROGMEM           = "Sence microphone              XS1 - 6                   ";
const char  txt_all_off19[]  PROGMEM           = "Microphone instruktora Sw.    XP1 12 HeS2e     Error! - ";                           
const char  txt_all_off20[]  PROGMEM           = "Microphone instruktora Sw.    XP1 12 HeS2e              ";                         
const char  txt_all_off21[]  PROGMEM           = "Radioperedacha OFF                             Error! - ";
const char  txt_all_off22[]  PROGMEM           = "Radioperedacha                                          ";                           
const char  txt_all_off23[]  PROGMEM           = "Microphone dispetchera OFF    XP1 10           Error! - ";                         
const char  txt_all_off24[]  PROGMEM           = "Microphone dispetchera                                  ";

const char  txt_test_all0[]  PROGMEM           = "Test instruktora start!"                                 ;
const char  txt_test_all1[]  PROGMEM           = "Signal microphone  instruktora 30mv  ON"                 ;
const char  txt_test_all2[]  PROGMEM           = "Microphone instruktora               ON        Error! - ";
const char  txt_test_all3[]  PROGMEM           = "Microphone instruktora               ON               - Ok!";
const char  txt_test_all4[]  PROGMEM           = "Microphone instruktora signal        ON"                 ;
const char  txt_test_all5[]  PROGMEM           = "";
const char  txt_test_all6[]  PROGMEM           = "";
const char  txt_test_all7[]  PROGMEM           = "";
const char  txt_test_all8[]  PROGMEM           = "";
const char  txt_test_all9[]  PROGMEM           = "";
const char  txt_test_all10[]  PROGMEM          = "";
const char  txt_test_all11[]  PROGMEM          = "";
const char  txt_test_all12[]  PROGMEM          = "";
const char  txt_test_all13[]  PROGMEM          = "";
const char  txt_test_all14[]  PROGMEM          = "";
const char  txt_test_all15[]  PROGMEM          = "";
const char  txt_test_all16[]  PROGMEM          = "";
const char  txt_test_all17[]  PROGMEM          = "";
const char  txt_test_all18[]  PROGMEM          = "";
const char  txt_test_all19[]  PROGMEM          = "";
const char  txt_test_all20[]  PROGMEM          = "Test dispetchera start!"                                 ;
const char  txt_test_all21[]  PROGMEM          = "Signal microphone dispetchera 30mv  ON"                  ;
const char  txt_test_all22[]  PROGMEM          = "Microphone dispetchera              ON         Error! - ";
const char  txt_test_all23[]  PROGMEM          = "Microphone dispetchera              ON                - Ok!";
const char  txt_test_all24[]  PROGMEM          = "Microphone dispetchera signal       ON"                  ;
const char  txt_test_all25[]  PROGMEM          = "";
const char  txt_test_all26[]  PROGMEM          = "";
const char  txt_test_all27[]  PROGMEM          = "";
const char  txt_test_all28[]  PROGMEM          = "";
const char  txt_test_all29[]  PROGMEM          = "";
const char  txt_test_all30[]  PROGMEM          = "Test MTT start!"                                         ;
const char  txt_test_all31[]  PROGMEM          = "Signal microphone MTT 30mv          ON"                  ;
const char  txt_test_all32[]  PROGMEM          = "Microphone MTT                      ON         Error! - ";
const char  txt_test_all33[]  PROGMEM          = "Microphone MTT                      ON                - Ok!";
const char  txt_test_all34[]  PROGMEM          = "Microphone MTT                      ON"                  ;
const char  txt_test_all35[]  PROGMEM          = "";
const char  txt_test_all36[]  PROGMEM          = "";
const char  txt_test_all37[]  PROGMEM          = "";
const char  txt_test_all38[]  PROGMEM          = "";
const char  txt_test_all39[]  PROGMEM          = "";
const char  txt_test_all40[]  PROGMEM          = "Signal FrontL     OFF                                 - ";
const char  txt_test_all41[]  PROGMEM          = "Signal FrontR     OFF                                 - ";
const char  txt_test_all42[]  PROGMEM          = "Signal mag radio  OFF                                 - ";
const char  txt_test_all43[]  PROGMEM          = "Signal mag phone  OFF                                 - ";
const char  txt_test_all44[]  PROGMEM          = "Signal GGS        OFF                                 - ";
const char  txt_test_all45[]  PROGMEM          = "Signal GG Radio1  OFF                                 - ";
const char  txt_test_all46[]  PROGMEM          = "Signal GG Radio2  OFF                                 - ";
const char  txt_test_all47[]  PROGMEM          = "Error! - "                                               ;
const char  txt_test_all48[]  PROGMEM          = "";
const char  txt_test_all49[]  PROGMEM          = "";
const char  txt_test_all50[]  PROGMEM          = "Signal FrontL     OFF                                 - ";
const char  txt_test_all51[]  PROGMEM          = "Signal FrontR     OFF                                 - ";
const char  txt_test_all52[]  PROGMEM          = "Signal FrontL     ON                                  - ";
const char  txt_test_all53[]  PROGMEM          = "Signal mag radio  OFF                                 - ";
const char  txt_test_all54[]  PROGMEM          = "Signal mag phone  OFF                                 - ";
const char  txt_test_all55[]  PROGMEM          = "Signal GGS        OFF                                 - ";
const char  txt_test_all56[]  PROGMEM          = "Signal GG Radio1  OFF                                 - ";
const char  txt_test_all57[]  PROGMEM          = "Signal GG Radio2  OFF                                 - ";
const char  txt_test_all58[]  PROGMEM          = "Mag phone         ON                                  - ";
const char  txt_test_all59[]  PROGMEM          = "";
const char  txt_test_all60[]  PROGMEM          = "";

const char  txt_instr_off0[]  PROGMEM          = "Komanda sence OFF instruktora   send!                   ";
const char  txt_instr_off1[]  PROGMEM          = "Komanda sence OFF instruktora 2 send!                   ";
const char  txt_instr_off2[]  PROGMEM          = "Komanda PTT   OFF instruktora   send!                   ";
const char  txt_instr_off3[]  PROGMEM          = "Komanda sence OFF instruktora 2                Error! - ";
const char  txt_instr_off4[]  PROGMEM          = "Komanda sence OFF instruktora 2                       - Ok!";
const char  txt_instr_off5[]  PROGMEM          = "Komanda sence OFF instruktora                  Error! - ";
const char  txt_instr_off6[]  PROGMEM          = "Komanda sence OFF instruktora                         - Ok!";
const char  txt_instr_off7[]  PROGMEM          = "Komanda sence OFF microphone                   Error! - ";
const char  txt_instr_off8[]  PROGMEM          = "Komanda sence OFF microphone                          - Ok!";
const char  txt_instr_off9[]  PROGMEM          = "Komanda       OFF PTT instruktora (CTS)        Error! - ";
const char  txt_instr_off10[] PROGMEM          = "Komanda       OFF PTT instruktora (CTS)               - Ok!";

const char  txt_instr_on0[]  PROGMEM           = "Komanda sence ON  instruktora    send!"                  ;
const char  txt_instr_on1[]  PROGMEM           = "Komanda sence ON  instruktora 2  send!"                  ;
const char  txt_instr_on2[]  PROGMEM           = "Komanda sence ON  instruktora 2                Error! - ";
const char  txt_instr_on3[]  PROGMEM           = "Komanda sence ON  instruktora 2                       - Ok!";
const char  txt_instr_on4[]  PROGMEM           = "Komanda sence ON  instruktora                  Error! - ";
const char  txt_instr_on5[]  PROGMEM           = "Komanda sence ON  instruktora                         - Ok!";
const char  txt_instr_on6[]  PROGMEM           = "Komanda       ON  PTT instruktora (CTS)        Error! - ";
const char  txt_instr_on7[]  PROGMEM           = "Komanda       ON  PTT instruktora (CTS)               - Ok!";
const char  txt_instr_on8[]  PROGMEM           = "Komanda       ON  PTT instruktora (CTS)  send!"          ;


const char  txt_disp_off0[]  PROGMEM           = "Komanda sence OFF dispetchera   send!                   ";
const char  txt_disp_off1[]  PROGMEM           = "Komanda sence OFF dispetchera 2 send!                   ";
const char  txt_disp_off2[]  PROGMEM           = "Komanda PTT   OFF dispetchera   send!                   ";
const char  txt_disp_off3[]  PROGMEM           = "Komanda sence OFF dispetchera 2                Error! - ";
const char  txt_disp_off4[]  PROGMEM           = "Komanda sence OFF dispetchera 2                       - Ok!";
const char  txt_disp_off5[]  PROGMEM           = "Komanda sence OFF dispetchera                  Error! - ";
const char  txt_disp_off6[]  PROGMEM           = "Komanda sence OFF dispetchera                         - Ok!";
const char  txt_disp_off7[]  PROGMEM           = "Komanda sence OFF microphone                   Error! - ";
const char  txt_disp_off8[]  PROGMEM           = "Komanda sence OFF microphone                          - Ok!";
const char  txt_disp_off9[]  PROGMEM           = "Komanda       OFF PTT dispetchera (CTS)        Error! - ";
const char  txt_disp_off10[] PROGMEM           = "Komanda       OFF PTT dispetchera (CTS)               - Ok!";

const char  txt_disp_on0[]   PROGMEM           = "Komanda sence ON  dispetchera    send!"                  ;
const char  txt_disp_on1[]   PROGMEM           = "Komanda sence ON  dispetchera 2  send!"                  ;
const char  txt_disp_on2[]   PROGMEM           = "Komanda sence ON  dispetchera 2                Error! - ";
const char  txt_disp_on3[]   PROGMEM           = "Komanda sence ON  dispetchera 2                       - Ok!";
const char  txt_disp_on4[]   PROGMEM           = "Komanda sence ON  dispetchera                  Error! - ";
const char  txt_disp_on5[]   PROGMEM           = "Komanda sence ON  dispetchera                         - Ok!";
const char  txt_disp_on6[]   PROGMEM           = "Komanda       ON  PTT dispetchera (CTS)        Error! - ";
const char  txt_disp_on7[]   PROGMEM           = "Komanda       ON  PTT dispetchera (CTS)               - Ok!";
const char  txt_disp_on8[]   PROGMEM           = "Komanda       ON  PTT dispetchera (CTS) send!"           ;

const char  txt_mtt_off0[]   PROGMEM           = "Komanda sence OFF MTT            send!                  ";
const char  txt_mtt_off1[]   PROGMEM           = "Komanda PTT   OFF MTT            send!                  ";
const char  txt_mtt_off2[]   PROGMEM           = "Komanda sence OFF MTT                          Error! - ";
const char  txt_mtt_off3[]   PROGMEM           = "Komanda sence OFF MTT                                 - Ok!";
const char  txt_mtt_off4[]   PROGMEM           = "Komanda sence OFF microphone                   Error! - ";
const char  txt_mtt_off5[]   PROGMEM           = "Komanda sence OFF microphone                          - Ok!";
const char  txt_mtt_off6[]   PROGMEM           = "Komanda       OFF PTT  MTT (CTS)               Error! - ";
const char  txt_mtt_off7[]   PROGMEM           = "Komanda       OFF PTT  MTT (CTS)                      - Ok!";
const char  txt_mtt_off8[]   PROGMEM           = "Komanda       OFF HangUp MTT     send!                   - Ok!";
const char  txt_mtt_off9[]   PROGMEM           = "Komanda       OFF HangUp MTT                   Error! - ";
const char  txt_mtt_off10[]  PROGMEM           = "Komanda       OFF HangUp MTT                          - Ok!";


const char  txt_mtt_on0[]    PROGMEM           = "Komanda sence ON  MTT            send!"                  ;
const char  txt_mtt_on1[]    PROGMEM           = "Komanda sence ON  MTT                          Error! - ";
const char  txt_mtt_on2[]    PROGMEM           = "Komanda sence ON  MTT                                 - Ok!";
const char  txt_mtt_on3[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS)               Error! - ";
const char  txt_mtt_on4[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS)                      - Ok!";
const char  txt_mtt_on5[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS) send!"                  ;
const char  txt_mtt_on6[]    PROGMEM           = "Komanda       ON  HangUp MTT                   Error! - ";
const char  txt_mtt_on7[]    PROGMEM           = "Komanda       ON  HangUp MTT                          - Ok!";
const char  txt_mtt_on8[]    PROGMEM           = "Komanda       ON  HangUp MTT     send!"                  ;






char buffer[60];  
const char* const string_table[] PROGMEM = 
{
txt_all_off0,     // 0
txt_all_off1,     // 1  "Test sensor OFF start!";                           
txt_all_off2,     // 2  "Sence MTT  XP1- 19 HaSs Error! - ";                         
txt_all_off3,     // 3  "Sence MTT XP1- 19 HaSs ";
txt_all_off4,     // 4   "OFF - Ok!"                                                ;                           
txt_all_off5,     // 5  "Tangenta ruchnaja XP7 2  Error! - ";                         
txt_all_off6,     // 6  "Tangenta ruchnaja XP7 - 2                                 ";
txt_all_off7,     // 7  "XP8 - 2 Sence tangenta nognaja Error! - ";                           
txt_all_off8,     // 8  "XP8 - 2 Sence tangenta nognaja ";                         
txt_all_off9,     // 9  Sensor garnitura instruktora 2  XP1- 16 HeS2Rs  Error!   - ";
txt_all_off10,    // 10  "Sensor garnitura instruktora 2  XP1- 16 HeS2Rs           ";                           
txt_all_off11,    // 11  "Sensor garnitura instruktora XP1- 13 HeS2Ls  Error!    - ";                         
txt_all_off12,    // 12  "Sensor garnitura instruktora XP1- 13 HeS2Ls              ";
txt_all_off13,    // 13  "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  Error!  - ";                           
txt_all_off14,    // 14  "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls            ";                         
txt_all_off15,    // 15  "Sensor garnitura dispetchera XP1- 1  HeS1Ls Error!     - ";
txt_all_off16,    // 16  "Sensor garnitura dispetchera XP1- 1  HeS1Ls              ";                           
txt_all_off17,    // 17  "Sensor microphone  XS1 - 6 Error!                      - ";                         
txt_all_off18,    // 18  "Sensor microphone  XS1 - 6                               ";
txt_all_off19,    // 19  "Microphone instruktora Sw.  XP1 12 HeS2e Error!        - ";                           
txt_all_off20,    // 20  "Microphone instruktora Sw.  XP1 12 HeS2e                 ";                         
txt_all_off21,    // 21  "Radioperedacha OFF  Error!                             - ";
txt_all_off22,    // 22  "Radioperedacha           "                                ;                           
txt_all_off23,    // 23  "Microphone dispetchera OFF XP1 10    Error!            - ";                         
txt_all_off24     // 24  "Microphone dispetchera   "                                ;

};

const char* const string_table_On[] PROGMEM = 
{
	txt_all_on0,      // 0  "Test sensor ON start!";                    
	txt_all_on1,      // 1  "Test sensor ON start!";                           
	txt_all_on2,      // 2  "Sence MTT  XP1- 19 HaSs Error! - ";                         
	txt_all_on3,      // 3  "Sence MTT XP1- 19 HaSs";
	txt_all_on4,      // 4  "ON - Ok!"                                                 ;
	txt_all_on5,      // 5  "XP7 2 Tang. ruch . Error!                              - ";    
	txt_all_on6,      // 6  "XP7 - 2 Tang. ruch .                                     ";    
	txt_all_on7,      // 7  "Sensor garnitura dispetchera XP1- 1  HeS1Ls Error!     - ";         
	txt_all_on8,      // 8  "XP8 - 2 Sence tang. nogn. Error!                       - ";          
	txt_all_on9,      // 9  "XP8 - 2 Sence tang. nogn.                                ";      
	txt_all_on10,     // 10  "Sensor garnitura dispetchera XP1- 1  HeS1Ls              ";        
	txt_all_on11,     // 11  "Sensor garnitura instruktora 2  XP1- 16 HeS2Rs  Error! - ";      
	txt_all_on12,     // 12  "Sensor garnitura instruktora 2  XP1- 16 HeS2Rs           ";                         
	txt_all_on13,     // 13  "Sensor microphone  XS1 - 6 Error! -                      ";
	txt_all_on14,     // 14  "Sensor garnitura instruktora XP1- 13 HeS2Ls  Error!    - ";
	txt_all_on15,     // 15  "Sensor garnitura instruktora XP1- 13 HeS2Ls              ";    
	txt_all_on16,     // 16  "Sensor microphone  XS1 - 6                               ";    
	txt_all_on17,     // 17  "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  Error!  - ";         
	txt_all_on18,     // 18  "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls            ";          
	txt_all_on19,     // 19 "Microphone instruktora Sw. XP1 12 HeS2e Error! -          ";        
	txt_all_on20,     // 20  "Microphone instruktora Sw. XP1 12 HeS2e                  ";         
	txt_all_on21,     // 21  "Radioperedacha ON  Error! -                              ";          
	txt_all_on22,     // 22  "Radioperedacha                                           ";        
	txt_all_on23,     // 23  "Microphone dispetchera ON Error! -                       ";         
	txt_all_on24      // 24  "Microphone dispetchera XP1 10                            ";   
};

const char* const table_instr_off[] PROGMEM = 
{
	txt_instr_off0,      //  "Komanda sence OFF instruktora   send!                   ";
	txt_instr_off1,      //  "Komanda sence OFF instruktora 2 send!                   ";
	txt_instr_off2,      //  "Komanda PTT instruktora OFF      send!                  ";
	txt_instr_off3,      //  "Komanda sence OFF instruktora 2                Error! - ";
	txt_instr_off4,      //  "Komanda sence OFF instruktora 2 - Ok!                   ";
	txt_instr_off5,      //  "Komanda sence OFF instruktora                  Error! - ";
	txt_instr_off6,      //  "Komanda sence OFF instruktora                         - Ok!";
	txt_instr_off7,      //  "Komanda sence microphone OFF                   Error! - ";
	txt_instr_off8,      //  "Komanda sence microphone OFF                          - Ok!";
	txt_instr_off9,      //  "Komanda PTT instruktora (CTS) OFF              Error! - ";
	txt_instr_off10      //  "Komanda PTT instruktora (CTS) OFF                     - Ok!";
};

const char* const table_instr_on[] PROGMEM = 
{
	txt_instr_on0,      // "Komanda sence ON instruktora    send!"                   ;
	txt_instr_on1,      // "Komanda sence ON instruktora 2  send!"                   ;
	txt_instr_on2,      // "Komanda sence ON instruktora 2                 Error! - ";
	txt_instr_on3,      // "Komanda sence ON instruktora 2                        - Ok!";
	txt_instr_on4,      // "Komanda sence ON instruktora                   Error! - ";
	txt_instr_on5,      // "Komanda sence ON instruktora                          - Ok!";
	txt_instr_on6,      // "Komanda PTT instruktora(CTS) ON                Error! - ";
	txt_instr_on7,      // "Komanda PTT instruktora (CTS) ON                      - Ok!";
	txt_instr_on8       // "Komanda PTT instruktora (CTS)  send!"                    ;
};

const char* const table_disp_off[] PROGMEM = 
{
	txt_disp_off0,      //  "Komanda sence OFF dispetchera   send!                   ";
	txt_disp_off1,      //  "Komanda sence OFF dispetchera 2 send!                   ";
	txt_disp_off2,      //  "Komanda PTT dispetchera OFF      send!                  ";
	txt_disp_off3,      //  "Komanda sence OFF dispetchera 2                Error! - ";
	txt_disp_off4,      //  "Komanda sence OFF dispetchera 2 - Ok!                   ";
	txt_disp_off5,      //  "Komanda sence OFF dispetchera                  Error! - ";
	txt_disp_off6,      //  "Komanda sence OFF dispetchera                         - Ok!";
	txt_disp_off7,      //  "Komanda sence microphone OFF                   Error! - ";
	txt_disp_off8,      //  "Komanda sence microphone OFF                          - Ok!";
	txt_disp_off9,      //  "Komanda PTT   OFF dispetchera (CTS)            Error! - ";
	txt_disp_off10      //  "Komanda PTT   OFF dispetchera (CTS)                   - Ok!";
};

const char* const table_disp_on[] PROGMEM = 
{
	txt_disp_on0,      // "Komanda sence ON dispetchera    send!"                   ;
	txt_disp_on1,      // "Komanda sence ON dispetchera 2  send!"                   ;
	txt_disp_on2,      // "Komanda sence ON dispetchera 2                 Error! - ";
	txt_disp_on3,      // "Komanda sence ON dispetchera 2                        - Ok!";
	txt_disp_on4,      // "Komanda sence ON dispetchera                   Error! - ";
	txt_disp_on5,      // "Komanda sence ON dispetchera                          - Ok!";
	txt_disp_on6,      // "Komanda PTT dispetchera OFF                    Error! - ";
	txt_disp_on7,      // "Komanda PTT dispetchera  OFF                          - Ok!";
	txt_disp_on8       // "Komanda PTT dispetchera (CTS)  send!"                    ;
};

const char* const table_mtt_off[] PROGMEM = 
{
	txt_mtt_off0,      // "Komanda sence OFF MTT            send!                  ";
	txt_mtt_off1,      // "Komanda PTT   OFF MTT            send!                  ";
	txt_mtt_off2,      // "Komanda sence OFF MTT                          Error! - ";
	txt_mtt_off3,      // "Komanda sence OFF MTT                                 - Ok!";
	txt_mtt_off4,      // "Komanda sence OFF microphone                   Error! - ";
	txt_mtt_off5,      // "Komanda sence OFF microphone                          - Ok!";
	txt_mtt_off6,      // "Komanda       OFF PTT  MTT (CTS)               Error! - ";
	txt_mtt_off7,      // "Komanda       OFF PTT  MTT (CTS)                      - Ok!";
	txt_mtt_off8,      // "Komanda       OFF HangUp MTT     send!                   - Ok!";
	txt_mtt_off9,      // "Komanda       OFF HangUp MTT                   Error! - ";
	txt_mtt_off10      // "Komanda       OFF HangUp MTT                          - Ok!";
};

const char* const table_mtt_on[] PROGMEM = 
{
	txt_mtt_on0,      // "Komanda sence ON  MTT            send!"                  ;
	txt_mtt_on1,      // "Komanda sence ON  MTT                          Error! - ";
	txt_mtt_on2,      // "Komanda sence ON  MTT                                 - Ok!";
	txt_mtt_on3,      // "Komanda       ON  PTT  MTT (CTS)               Error! - ";
	txt_mtt_on4,      // "Komanda       ON  PTT  MTT (CTS)                      - Ok!";
	txt_mtt_on5,      // "Komanda       ON  PTT  MTT (CTS) send!"                  ;
	txt_mtt_on6,      // "Komanda       ON  HangUp MTT                   Error! - ";
	txt_mtt_on7,      // "Komanda       ON  HangUp MTT                          - Ok!";
	txt_mtt_on8       // "Komanda       ON  HangUp MTT     send!"                  ;
};

const char* const table_instr_all[] PROGMEM = 
{
	txt_test_all0,      // 0 "Test instruktora start!"                                 ;
	txt_test_all1,      // 1 "Signal microphone  instruktora 30mv  ON"                 ;
	txt_test_all2,      // 2 "Microphone instruktora ON                      Error! - ";
	txt_test_all3,      // 3 "Microphone instruktora  ON                         - Ok!";
	txt_test_all4,      // 4 "Microphone instruktora signal ON"                        ;
	txt_test_all5,      // ;
	txt_test_all6,      // ;
	txt_test_all7,      // ;
	txt_test_all8,      // ;
	txt_test_all9,      // ;

	txt_test_all10,     // ;
	txt_test_all11,     // ;
	txt_test_all12,     // ;
	txt_test_all13,     // ;
	txt_test_all14,     // ;
	txt_test_all15,     // ;
	txt_test_all16,     // ;
	txt_test_all17,     // ;
	txt_test_all18,     // ;
	txt_test_all19,     // ;

	txt_test_all20,     //  "Test dispetchera start!"                                 ;
	txt_test_all21,     //  "Signal microphone dispetchera 30mv  ON"                  ;
	txt_test_all22,     //  "Microphone dispetchera ON                      Error! - ";
	txt_test_all23,     //  "Microphone dispetchera ON                          - Ok!";
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
	txt_test_all49,     // ""                                               ;

	txt_test_all50,     // "Signal FrontL OFF                                     - ";
	txt_test_all51,     // "Signal FrontR OFF                                     - ";
	txt_test_all52,     // "Signal FrontL ON                                      - ";
	txt_test_all53,     // "Signal mag radio OFF                                  - ";
	txt_test_all54,     // "Signal mag phone OFF                                  - ";
	txt_test_all55,     // "Signal GGS OFF                                        - ";
	txt_test_all56,     // "Signal GG Radio1 OFF                                  - ";
	txt_test_all57,     // "Signal GG Radio2 OFF                                  - ";
	txt_test_all58,     // "Mag phone on                                          - ";
	txt_test_all59,     // ;

	txt_test_all60      // ;
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
			 test_dispetchera();          //
				break;
		case 5:
			 test_MTT();                  //
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
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[1])));
	myFile.println(buffer);
	file_print_date();
	myFile.println();
	regBank.set(8,1);     // Включить питание Камертон
	regBank.set(5,0);     // Микрофон инструктора отключить
	regBank.set(10,0);    // Микрофон диспетчера отключить
	regBank.set(13,0);    // XP8 - 2   Sence Тангента ножная
	regBank.set(14,0);    // XP8 - 1   PTT     Тангента ножная
	regBank.set(15,0);    // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);    // XS1 - 6   Sence подключения микрофона
 
	regBank.set(17,0);    // J8-12    XP7 4 PTT2 тангента ручная DSR
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(19,0);    // J8-11     XP7 2 Sence тангента ручная
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 тангента ручная CTS
	//------------------------- Не используется --------------------
	//regBank.set(21,0);    // XP2-2     Sence "Маг."  
	//regBank.set(22,0);    // XP5-3     Sence "ГГC."
	//regBank.set(23,0);    // XP3-3     Sence "ГГ-Радио1."
	//regBank.set(24,1);    // XP4-3     Sence "ГГ-Радио2."
	//-------------------------------------------------------------------
	regBank.set(25,1);    // XP1- 19 HaSs      Sence подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.set(27,0);    // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS вкл
	regBank.set(29,0);    // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS вкл
	regBank.set(31,0);    // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);    // XP1- 1  HeS1Ls    Sence подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];    


	//regBank.add(134);   // Флаг счетчика ошибки сенсора PTT Танг н.  ok!
	//regBank.add(135);   // Флаг счетчика ошибки сенсора PTT Мик ok!

	//regBank.add(136);   // Флаг счетчика ошибки  PTT2 Танг. р. ok!
	//regBank.add(137);   // Флаг счетчика ошибки  HangUp  DCD  ok!
	//regBank.add(138);   // Флаг счетчика ошибки  PTT1 Танг. р. ok!
	//regBank.add(139);   // Флаг ошибки отключения микрофона гарнитуры инструктора


		if(bitRead(i50,2) != 0)                                         // XP1- 19 HaSs Sence подключения трубки  
		  {
			regcount = regBank.get(40123);                              // адрес счетчика ошибки Sence подключения трубки  
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения трубки  
			regBank.set(40123,regcount);                                // адрес счетчика ошибки Sence подключения трубки  
			regBank.set(123,1);                                         // установить флаг ошибки Sence подключения трубки  
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[2])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[3])));
			myFile.print(buffer);                                        // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                      //  Sence подключения трубки  OK!
		  }
	
		if(bitRead(i50,3) != 0)                                         // J8-11     XP7 2 Sence тангента ручная
		  {
			regcount = regBank.get(40124);                              // адрес счетчика ошибки Sence тангента ручная
			regcount++;                                                 // увеличить счетчик ошибок Sence тангента ручная
			regBank.set(40124,regcount);                                // адрес счетчика ошибки Sence тангента ручная
			regBank.set(124,1);                                         // установить флаг ошибки Sence тангента ручная
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[6])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence тангента ручная OK!
		  }

		if(bitRead(i50,4) != 0)                                         // XP8 - 2   Sence Тангента ножная
		  {
			regcount = regBank.get(40125);                              // адрес счетчика ошибки Sence Тангента ножная
			regcount++;                                                 // увеличить счетчик ошибок  Sence Тангента ножная
			regBank.set(40125,regcount);                                // адрес счетчика ошибки  Sence Тангента ножная
			regBank.set(125,1);                                         // установить флаг ошибки Sence Тангента ножная
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[7])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[8])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // Sence Тангента ножная OK!
		  }

		if(bitRead(i52,1) != 0)                                         // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40127);                              // адрес счетчика ошибки Sence подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40127,regcount);                                // адрес счетчика ошибки Sence подключения гарнитуры инструктора с 2 наушниками
			regBank.set(127,1);                                         // установить флаг ошибки Sence подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[9])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[10])));
			myFile.print(buffer);                                       // "Sensor garnitura instruktora 2  XP1- 16 HeS2Rs"
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  
		  }

		if(bitRead(i52,2) != 0)                                         // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40128);                              // адрес счетчика ошибки Sence подключения гарнитуры инструктора
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения гарнитуры инструктора
			regBank.set(40128,regcount);                                // адрес счетчика ошибки Sence подключения гарнитуры инструктора 
			regBank.set(128,1);                                         // установить флаг ошибки Sence подключения гарнитуры инструктора 
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[11])));
			myFile.print(buffer);                                       // "Sensor garnitura instruktora XP1- 13 HeS2Ls  Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[12])));
			myFile.print(buffer);                                       // "Sensor garnitura instruktora XP1- 13 HeS2Ls    "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //   Sence подключения гарнитуры инструктора OK!
		  }

		if(bitRead(i52,3) != 0)                                         // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40129);                              // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40129,regcount);                                // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(129,1);                                         // установить флаг ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[13])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[14])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence подкючения гарнитуры диспетчера с 2 наушниками OK!
		  }

		
		if(bitRead(i52,4) != 0)                                         // XP1- 1  HeS1Ls   Sence подкючения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40130);                              // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера
			regcount++;                                                 // увеличить счетчик ошибок Sence подкючения гарнитуры диспетчера 
			regBank.set(40130,regcount);                                // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера
			regBank.set(130,1);                                         // установить флаг ошибки Sence подкючения гарнитуры диспетчера
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[15])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera XP1- 1  HeS1Ls Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[16])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera XP1- 1  HeS1Ls    "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence подкючения гарнитуры диспетчера  OK!
		  }

		if(bitRead(i52,5) != 0)                                         // XS1 - 6   Sence отключения микрофона
		  {
			regcount = regBank.get(40131);                              // адрес счетчика ошибки Sence подключения микрофона
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения микрофона
			regBank.set(40131,regcount);                                // адрес счетчика ошибки Sence подключения микрофона
			regBank.set(131,1);                                         // установить флаг ошибки Sence подключения микрофона
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[17])));
			myFile.print(buffer);                                       // "Sensor microphone  XS1 - 6 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[18])));
			myFile.print(buffer);                                       // "Sensor microphone  XS1 - 6  "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence отключения микрофона OK!
		  }

		if(bitRead(i53,4) != 0)                                         // Реле RL4 XP1 12  HeS2e   Выключение микрофона инструктора
		  {
			regcount = regBank.get(40139);                              // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                 // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40139,regcount);                                // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(139,1);                                         // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[19])));
			myFile.print(buffer);                                       // "Microphone instruktora Sw.  XP1 12 HeS2e Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[20])));
			myFile.print(buffer);                                        // "Microphone instruktora Sw.  XP1 12 HeS2e "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // Выключение микрофона инструктора ОК!
		  }
	   // --------------------------------  Включение радиопередачи пока не определил ------------------------------
		//if(bitRead(i53,5) != 0)                                         // Выключение радиопередачи
		//  {
		//	regcount = regBank.get(40156);                              // адрес счетчика ошибки Включение радиопередачи
		//	regcount++;                                                 // увеличить счетчик ошибок Включение радиопередачи
		//	regBank.set(40156,regcount);                                // адрес счетчика ошибки Включение радиопередачи
		//	regBank.set(156,1);                                         // установить флаг ошибки Включение радиопередачи
		//	regBank.set(120,1);                                         // установить общий флаг ошибки
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[21])));
		//	myFile.print(buffer);                                // "Radioperedacha OFF  Error! - "
		//	myFile.println(regcount);                                   // 
		//  }
		//else
		//  {
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[22])));
		//	myFile.print(buffer);                                // "Radioperedacha           "
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
		//	myFile.println(buffer);                               // Выключение радиопередачи ОК!
		//  }

		if(bitRead(i53,6) != 0)                                         // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = regBank.get(40155);                              // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                 // увеличить счетчик ошибок Включение микрофона диспетчера
			regBank.set(40155,regcount);                                // адрес счетчика ошибки Включение микрофона диспетчера
			regBank.set(155,1);                                         // установить флаг ошибки Включение микрофона диспетчера
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[23])));
			myFile.print(buffer);                                       // "Microphone dispetchera OFF XP1 10    Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[24])));
			myFile.print(buffer);                                       // "Microphone dispetchera   "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // Выключение микрофона диспетчера ОК!
		  }


		//-----------------------------------Отключен, в данной версии не применяется ------------------------------------------------
	/*	if(bitRead(i50,0) != 0)                                         // XP4-3     Sence "ГГ-Радио2."
		  {
			regcount = regBank.get(40122);                              // адрес счетчика ошибки Sence "ГГ-Радио2."
			regcount++;                                                 // увеличить счетчик ошибок Sence "ГГ-Радио2."
			regBank.set(40122,regcount);                                // адрес счетчика ошибки Sence "ГГ-Радио2."
			regBank.set(122,1);                                         // установить флаг ошибки Sence "ГГ-Радио2."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence GG Radio 2 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 2 ");                          // 
			myFile.println("OFF - Ok!");                                //  Sence "ГГ-Радио2." OK!
		  }

		if(bitRead(i50,1) != 0)                                         // XP3-3     Sence "ГГ-Радио1."
		  {
			regcount = regBank.get(40121);                              // адрес счетчика ошибки  Sence "ГГ-Радио1."
			regcount++;                                                 // увеличить счетчик ошибок  Sence "ГГ-Радио1."
			regBank.set(40121,regcount);                                // адрес счетчика ошибки  Sence "ГГ-Радио1."
			regBank.set(121,1);                                         // установить флаг ошибки  Sence "ГГ-Радио1."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence GG Radio 1 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 1 ");                          // 
			myFile.println("OFF - Ok!");                                //   Sence "ГГ-Радио1." OK!
		  }
	if(bitRead(i52,0) != 0)                                             // XP2-2 Sence "Маг."  
		  {
			regcount = regBank.get(40126);                              // адрес счетчика ошибки Sence "Маг."  
			regcount++;                                                 // увеличить счетчик ошибок Sence "Маг."  
			regBank.set(40126,regcount);                                // адрес счетчика ошибки Sence  "Маг."  
			regBank.set(126,1);                                         // установить флаг ошибки Sence "Маг."  
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence MAG XP2-2  Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence MAG XP2-2 ");                           // 
			myFile.println("OFF - Ok!");                                //  Sence "Маг."  
		  }
	
		if(bitRead(i52,6) != 0)                                         // XP5-3     Sence "ГГC."
		  {
			regcount = regBank.get(40132);                              // адрес счетчика ошибки Sence "ГГC."
			regcount++;                                                 // увеличить счетчик ошибок Sence "ГГC."
			regBank.set(40132,regcount);                                // адрес счетчика ошибки Sence "ГГC."
			regBank.set(132,1);                                         // установить флаг ошибки Sence "ГГC."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sensor GGS  XP5-3 Error! - ");                // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sensor GGS  XP5-3 Error! - ");                                      // 
			myFile.println("OFF - Ok!");                                //  Sence "ГГC." OK
		  }

*/
  //Serial.println(regBank.get(adr_reg_ind_CTS));
  //Serial.println(mcp_Analog.digitalRead(CTS));
 // delay(200);
}
void sence_all_on()
{
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[1])));
	myFile.println(buffer);  // 
	file_print_date();
	myFile.println();
	regBank.set(8,1);     // Включить питание Камертон
	regBank.set(5,1);     // Микрофон инструктора включить
	regBank.set(10,1);    // Микрофон диспетчера включить
	regBank.set(13,1);    // XP8 - 2   Sence Тангента ножная
	//regBank.set(14,1);    // XP8 - 1   PTT     Тангента ножная
	//regBank.set(15,1);    // XS1 - 5   PTT Мик CTS
	regBank.set(16,1);    // XS1 - 6   Sence подключения микрофона
 
	//regBank.set(17,1);    // J8-12    XP7 4 PTT2 тангента ручная DSR
	//regBank.set(18,1);    // XP1 - 20  HangUp  DCD
	regBank.set(19,1);    // J8-11     XP7 2 Sence тангента ручная
	//regBank.set(20,1);    // J8-23     XP7 1 PTT1 тангента ручная CTS
	//------------------------- Не используется --------------------
	//regBank.set(21,1);    // XP2-2     Sence "Маг."  
	//regBank.set(22,1);    // XP5-3     Sence "ГГC."
	//regBank.set(23,1);    // XP3-3     Sence "ГГ-Радио1."
	//regBank.set(24,1);    // XP4-3     Sence "ГГ-Радио2."
	//-------------------------------------------------------------------
	regBank.set(25,0);    // XP1- 19 HaSs      Sence подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
//	regBank.set(26,1);    // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.set(27,1);    // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
//	regBank.set(28,1);    // XP1- 15 HeS2PTT   CTS вкл
	regBank.set(29,1);    // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
//	regBank.set(30,1);    // XP1- 6  HeS1PTT   CTS вкл
	regBank.set(31,1);    // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);    // XP1- 1  HeS1Ls    Sence подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
	//regBank.add(134);   // Флаг счетчика ошибки сенсора PTT Танг н.  ok!
	//regBank.add(135);   // Флаг счетчика ошибки сенсора PTT Мик ok!
	//regBank.add(136);   // Флаг счетчика ошибки  PTT2 Танг. р. ok!
	//regBank.add(137);   // Флаг счетчика ошибки  HangUp  DCD  ok!
	//regBank.add(138);   // Флаг счетчика ошибки  PTT1 Танг. р. ok!
	//regBank.add(139);   // Флаг ошибки отключения микрофона гарнитуры инструктора

		if(bitRead(regs_in[0],2) == 0)                                  // XP1- 19 HaSs Sence подключения трубки  
		  {
			regcount = regBank.get(40123);                              // адрес счетчика ошибки Sence подключения трубки  
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения трубки  
			regBank.set(40123,regcount);                                // адрес счетчика ошибки Sence подключения трубки  
			regBank.set(123,1);                                         // установить флаг ошибки Sence подключения трубки  
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[2])));
			myFile.print(buffer);                                  // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[3])));
			myFile.print(buffer);                                  // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                //  Sence подключения трубки  OK!
		  }
  
		if(bitRead(regs_in[0],3) == 0)                                  // J8-11     XP7 2 Sence тангента ручная
		  {
			regcount = regBank.get(40124);                              // адрес счетчика ошибки Sence тангента ручная
			regcount++;                                                 // увеличить счетчик ошибок Sence тангента ручная
			regBank.set(40124,regcount);                                // адрес счетчика ошибки Sence тангента ручная
			regBank.set(124,1);                                         // установить флаг ошибки Sence тангента ручная
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[5])));
			myFile.print(buffer);               // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[6])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence тангента ручная OK!
		  }

		if(bitRead(regs_in[0],4) == 0)                                  // XP8 - 2   Sence Тангента ножная
		  {
			regcount = regBank.get(40125);                              // адрес счетчика ошибки Sence Тангента ножная
			regcount++;                                                 // увеличить счетчик ошибок  Sence Тангента ножная
			regBank.set(40125,regcount);                                // адрес счетчика ошибки  Sence Тангента ножная
			regBank.set(125,1);                                         // установить флаг ошибки Sence Тангента ножная
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[8])));
			myFile.print(buffer);        // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[9])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 // Sence Тангента ножная OK!
		  }


		if(bitRead(regs_in[2],1) == 0)                                  // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40127);                              // адрес счетчика ошибки Sence подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40127,regcount);                                // адрес счетчика ошибки Sence подключения гарнитуры инструктора с 2 наушниками
			regBank.set(127,1);                                         // установить флаг ошибки Sence подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[11])));
			myFile.print(buffer); // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[12])));
			myFile.print(buffer);          // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  
		  }

		if(bitRead(regs_in[2],2) == 0)                                  // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40128);                              // адрес счетчика ошибки Sence подключения гарнитуры инструктора
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения гарнитуры инструктора
			regBank.set(40128,regcount);                                // адрес счетчика ошибки Sence подключения гарнитуры инструктора 
			regBank.set(128,1);                                         // установить флаг ошибки Sence подключения гарнитуры инструктора 
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[14])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[15])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //   Sence подключения гарнитуры инструктора OK!
		  }

		if(bitRead(regs_in[2],3) == 0)                                  // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40129);                              // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40129,regcount);                                // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(129,1);                                         // установить флаг ошибки Sence подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[17])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[18])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence подкючения гарнитуры диспетчера с 2 наушниками OK!
		  }

		if(bitRead(regs_in[2],4) == 0)                                  // XP1- 1  HeS1Ls  Sence подкючения гарнитуры диспетчера
		  {
			regcount = regBank.get(40130);                              // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера
			regcount++;                                                 // увеличить счетчик ошибок Sence подкючения гарнитуры диспетчера
			regBank.set(40130,regcount);                                // адрес счетчика ошибки Sence подкючения гарнитуры диспетчера 
			regBank.set(130,1);                                         // установить флаг ошибки Sence подкючения гарнитуры диспетчера
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[7])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[10])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence подкючения гарнитуры диспетчера OK!
		  }

		if(bitRead(regs_in[2],5) == 0)                                  // XS1 - 6   Sence подключения микрофона
		  {
			regcount = regBank.get(40131);                              // адрес счетчика ошибки Sence подключения микрофона
			regcount++;                                                 // увеличить счетчик ошибок Sence подключения микрофона
			regBank.set(40131,regcount);                                // адрес счетчика ошибки Sence подключения микрофона
			regBank.set(131,1);                                         // установить флаг ошибки Sence подключения микрофона
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[13])));
			myFile.print(buffer);                                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[16])));
			myFile.print(buffer);                                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                //  Sence подключения микрофона OK!
		  }
	
		if(bitRead(regs_in[3],4) == 0)                                  // Реле RL4 XP1 12  HeS2e   Включение микрофона гарнитуры инструктора
		  {
			regcount = regBank.get(40139);                              // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                 // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40139,regcount);                                // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(139,1);                                         // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[19])));
			myFile.print(buffer);                                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[20])));
			myFile.print(buffer);                                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                // Включение микрофона инструктора ОК!
		  }
		// --------------------------------  Включение радиопередачи пока не определил ------------------------------
		//if(bitRead(regs_in[3],5) == 0)                                  // Включение радиопередачи
		//  {
		//	regcount = regBank.get(40156);                              // адрес счетчика ошибки Включение радиопередачи
		//	regcount++;                                                 // увеличить счетчик ошибок Включение радиопередачи
		//	regBank.set(40156,regcount);                                // адрес счетчика ошибки Включение радиопередачи
		//	regBank.set(156,1);                                         // установить флаг ошибки Включение радиопередачи
		//	regBank.set(120,1);                                         // установить общий флаг ошибки
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[21])));
		//	myFile.print(buffer);                                 // 
		//	myFile.println(regcount);                                   // 
		//  }
		//else
		//  {
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[22])));
		//	myFile.print(buffer);                                 // 
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
		//	myFile.println(buffer);                                // Включение радиопередачи ОК!
		//  }
	

	   if(bitRead(regs_in[3],6) == 0)                                   // Реле RL9 XP1 10 Включение микрофона диспетчера
		  {
			regcount = regBank.get(40155);                              // адрес счетчика ошибки Включение радиопередачи
			regcount++;                                                 // увеличить счетчик ошибок Включение радиопередачи
			regBank.set(40155,regcount);                                // адрес счетчика ошибки Включение радиопередачи
			regBank.set(155,1);                                         // установить флаг ошибки Включение радиопередачи
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[23])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[24])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                     // Включение радиопередачи ОК!
		  }
	
		//-----------------------------------Отключен ------------------------------------------------------------------
	/*	if(bitRead(regs_in[0],0) == 0)                                  // // XP4-3     Sence "ГГ-Радио2."
		  {
			regcount = regBank.get(40122);                              // адрес счетчика ошибки Sence "ГГ-Радио2."
			regcount++;                                                 // увеличить счетчик ошибок Sence "ГГ-Радио2."
			regBank.set(40122,regcount);                                // адрес счетчика ошибки Sence "ГГ-Радио2."
			regBank.set(122,1);                                         // установить флаг ошибки Sence "ГГ-Радио2."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence GG Radio 2 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 2 ");                          // 
			myFile.println(txt_all_on4);                                 //  Sence "ГГ-Радио2." OK!
		  }

		if(bitRead(regs_in[0],1) == 0)                                  // XP3-3     Sence "ГГ-Радио1."
		  {
			regcount = regBank.get(40121);                              // адрес счетчика ошибки  Sence "ГГ-Радио1."
			regcount++;                                                 // увеличить счетчик ошибок  Sence "ГГ-Радио1."
			regBank.set(40121,regcount);                                // адрес счетчика ошибки  Sence "ГГ-Радио1."
			regBank.set(121,1);                                         // установить флаг ошибки  Sence "ГГ-Радио1."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence GG Radio 1 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 1 ");                          // 
			myFile.println(txt_all_on4);                                 //   Sence "ГГ-Радио1." OK!
		  }

		if(bitRead(regs_in[2],0) == 0)                                  // XP2-2 Sence "Маг."  
		  {
			regcount = regBank.get(40126);                              // адрес счетчика ошибки Sence "Маг."  
			regcount++;                                                 // увеличить счетчик ошибок Sence "Маг."  
			regBank.set(40126,regcount);                                // адрес счетчика ошибки Sence  "Маг."  
			regBank.set(126,1);                                         // установить флаг ошибки Sence "Маг."  
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sence MAG XP2-2  Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence MAG XP2-2 ");                           // 
			myFile.println(txt_all_on4);                                 //  Sence "Маг."  
		  }
		if(bitRead(regs_in[2],6) == 0)                                  // XP5-3     Sence "ГГC."
		  {
			regcount = regBank.get(40132);                              // адрес счетчика ошибки Sence "ГГC."
			regcount++;                                                 // увеличить счетчик ошибок Sence "ГГC."
			regBank.set(40132,regcount);                                // адрес счетчика ошибки Sence "ГГC."
			regBank.set(132,1);                                         // установить флаг ошибки Sence "ГГC."
			regBank.set(120,1);                                         // установить общий флаг ошибки
			myFile.print("Sensor GGS  XP5-3 Error! - ");                // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sensor GGS  XP5-3 Error! - ");                                      // 
			myFile.println(txt_all_on4);                                //  Sence "ГГC." OK
		  }
  */

	regBank.set(5,0);     // Микрофон инструктора отключить
	regBank.set(10,0);    // Микрофон диспетчера отключить
	UpdateRegs(); 
	delay(100);
}

void test_instruktora()
{
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[0])));
	myFile.println(buffer);                                              // "Test instruktora start!"
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_instr_off();                                                    // Отключить реле и сенсоры, прверить отключение
	test_instr_on();                                                     // Включить необходимые сенсоры, проверить состояние
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,40141,141,25);                         // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,40142,142,25);                         // Измерить уровень сигнала на выходе FrontR 
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                     // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                     // Установить уровень сигнала 30 мв
	regBank.set(2,1);                                                    // Подать сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();                                                        // Выполнить команду
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[1])));
	myFile.println(buffer);                                              // "Signal microphone  instruktora 30mv  ON"
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                      // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                      // Измерить уровень сигнала на выходе FrontR 
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                      // Измерить уровень сигнала на выходе mag radio  
	measure_vol_min(analog_mag_phone,40145,145,25);                      // Измерить уровень сигнала на выходе mag phone
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40146,146,30);                      // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                      // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                      // Измерить уровень сигнала на выходе GG Radio2

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                  //
	regBank.set(5,1);                                                    // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
	regBank.set(28,1);                                                   // XP1- 15 HeS2PTT Включить PTT инструктора
	regBank.set(16,0);                                                   // Сенсор микрофона отключить
	regBank.set(15,0);                                                   // РТТ микрофона отключить
	regBank.set(29,1);                                                   // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
	UpdateRegs();                                                        // 
	delay(200);                                                          //
	byte i5 = regs_in[3];                                                // 
		if(bitRead(i5,4) == 0)                                           // Проверка  включения микрофона инструктора
		  {
			regcount = regBank.get(40151);                               // адрес счетчика ошибки включения микрофона инструктора
			regcount++;                                                  // увеличить счетчик ошибок включения микрофона инструктора
			regBank.set(40151,regcount);                                 // адрес счетчика ошибки включения микрофона инструктора
			regBank.set(151,1);                                          // установить флаг ошибки
			regBank.set(120,1);                                          // установить общий флаг ошибки
			resistor(1, 255);                                            // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                            // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[2])));
			myFile.print(buffer);                                        // "Microphone instruktora ON  Error! - "
			myFile.println(regcount);                                    // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[3])));
			myFile.println(buffer);                                     //"Microphone instruktora  ON - Ok!" Микрофона инструктора включился
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[4])));
	myFile.println(buffer);                                             // "Microphone instruktora signal ON"  Звуковой сигнал подан на вход микрофона инструктора
	delay(20);
	//++++++++++++++++++++++++++++++++++ Проверить наличие сигнала на линиях FrontL    ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40143,143,200);                    // Измерить уровень сигнала на выходе LineL
	measure_vol_max(analog_mag_phone,40150,150,200);                    // Измерить уровень сигнала на выходе mag phone
   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                     // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                     // Измерить уровень сигнала на выходе FrontR 
	measure_vol_min(analog_ggs,      40146,146,30);                     // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                     // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                     // Измерить уровень сигнала на выходе GG Radio2

	regBank.set(29,0);                                                  // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	regBank.set(27,0);                                                  // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                  // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(15,0);                                                  // РТТ микрофона отключить
	regBank.set(5,0);                                                   // Подать управляющую команду на вывод 12 ХР1 HeS2e (Выключить микрофон инструктора)
	regBank.set(28,0);                                                  // XP1- 15 HeS2Ls Отключить PTT инструктора
	UpdateRegs();     


	regBank.set(adr_control_command,0);                                 // Завершить программу    
	delay(100);
}
void test_dispetchera()
 {
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[20])));
	myFile.println(buffer);                                               // "Test dispetchera start!"
	myFile.println();
	unsigned int regcount = 0;
	test_disp_off();                                                     // Отключить реле и сенсоры, прверить отключение
	test_disp_on();                                                      // Включить необходимые сенсоры, проверить состояние
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,40161,160,25);                         // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,40162,161,25);                         // Измерить уровень сигнала на выходе FrontR 
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                     // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                     // Установить уровень сигнала 30 мв
	regBank.set(1,1);                                                    // Подать сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();                                                        // Выполнить команду
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[21])));      //   
	myFile.println(buffer);                                              // "Signal microphone   dispetchera 30mv  ON"
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                      // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                      // Измерить уровень сигнала на выходе FrontR 
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                      // Измерить уровень сигнала на выходе mag radio  
	measure_vol_min(analog_mag_phone,40145,145,25);                      // Измерить уровень сигнала на выходе mag phone
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40146,146,30);                      // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                      // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                      // Измерить уровень сигнала на выходе GG Radio2

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                  //
	regBank.set(10,1);                                                   // Подать управляющую команду на вывод XP1 10 Включение микрофона диспетчера
	regBank.set(30,1);                                                   // XP1- 6  HeS1PTT   Включить PTT диспетчера
	regBank.set(16,0);                                                   // Сенсор микрофона отключить
	regBank.set(15,0);                                                   // РТТ микрофона отключить
	regBank.set(31,1);                                                   // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                   // XP1- 1  HeS1Ls    Sence подкючения гарнитуры диспетчера

	UpdateRegs();                                                        // 
	delay(200);                                                          //
	byte i5 = regs_in[3];                                                // 
		if(bitRead(i5,6) == 0)                                           // Проверка  включения микрофона диспетчера
		  {
			regcount = regBank.get(40151);                               // адрес счетчика ошибки включения микрофона диспетчера
			regcount++;                                                  // увеличить счетчик ошибок включения микрофона диспетчера
			regBank.set(40151,regcount);                                 // адрес счетчика ошибки включения микрофона диспетчера
			regBank.set(151,1);                                          // установить флаг ошибки
			regBank.set(120,1);                                          // установить общий флаг ошибки
			resistor(1, 255);                                            // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                            // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[22])));
			myFile.print(buffer);                                        // "Microphone dispetchera ON  Error! - "
			myFile.println(regcount);                                    // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[23])));
			myFile.println(buffer);                                     //"Microphone dispetchera  ON - Ok!" Микрофон диспетчера включился
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[24])));     // "Microphone dispetchera signal ON" 
	myFile.println(buffer);                                             // "Microphone dispetchera signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(20);
	//++++++++++++++++++++++++++++++++++ Проверить наличие сигнала на линиях FrontL    ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40143,143,200);                    // Измерить уровень сигнала на выходе LineL
	measure_vol_max(analog_mag_phone,40150,150,200);                    // Измерить уровень сигнала на выходе mag phone
   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                     // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                     // Измерить уровень сигнала на выходе FrontR 
	measure_vol_min(analog_ggs,      40146,146,30);                     // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                     // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                     // Измерить уровень сигнала на выходе GG Radio2
	regBank.set(31,0);                                                  // XP1- 5  HeS1Rs   Отключить Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                  // XP1- 1  HeS1Ls   Отключить  Sence подкючения гарнитуры диспетчера
	regBank.set(15,0);                                                  // РТТ микрофона отключить
	regBank.set(10,0);                                                  // Подать управляющую команду на вывод XP1 10  (Выключить микрофон диспетчера)
	regBank.set(30,0);                                                  // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	regBank.set(28,0);                                                  // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                 // Завершить программу    
	delay(100);
 }
void test_MTT()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[30])));
	myFile.println(buffer);                                               // "Test MTT start!"
	myFile.println();
	unsigned int regcount = 0;
	test_MTT_off();                                                      // Отключить реле и сенсоры, прверить отключение
	test_MTT_on();                                                       // Включить необходимые сенсоры, проверить состояние
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,40161,160,25);                         // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,40162,161,25);                         // Измерить уровень сигнала на выходе FrontR 
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                    // Установить уровень сигнала 30 мв
	resistor(2, 130);                                                    // Установить уровень сигнала 30 мв
	regBank.set(3,1);                                                    // Включить сигнал на вход микрофона трубки Mic3p
	UpdateRegs();                                                        // Выполнить команду
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[31])));      //   
	myFile.println(buffer);                                              // "Signal microphone   MTT 30mv  ON"


	//!!!!!!!!!!!!!!!!!!!!!!


	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                      // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                      // Измерить уровень сигнала на выходе FrontR 
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                      // Измерить уровень сигнала на выходе mag radio  
	measure_vol_min(analog_mag_phone,40145,145,25);                      // Измерить уровень сигнала на выходе mag phone
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40146,146,30);                      // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                      // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                      // Измерить уровень сигнала на выходе GG Radio2

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                  //
	regBank.set(10,1);                                                   // Подать управляющую команду на вывод XP1 10 Включение микрофона MTT
	regBank.set(30,1);                                                   // XP1- 6  HeS1PTT   Включить PTT MTT
	regBank.set(16,0);                                                   // Сенсор микрофона отключить
	regBank.set(15,0);                                                   // РТТ микрофона отключить
	regBank.set(32,1);                                                   // XP1- 1  HeS1Ls    Sence подкючения MTT

	UpdateRegs();                                                        // 
	delay(200);                                                          //
	byte i5 = regs_in[3];                                                // 
		if(bitRead(i5,6) == 0)                                           // Проверка  включения микрофона MTT
		  {
			regcount = regBank.get(40151);                               // адрес счетчика ошибки включения микрофона MTT
			regcount++;                                                  // увеличить счетчик ошибок включения микрофона MTT
			regBank.set(40151,regcount);                                 // адрес счетчика ошибки включения микрофона MTT
			regBank.set(151,1);                                          // установить флаг ошибки
			regBank.set(120,1);                                          // установить общий флаг ошибки
			resistor(1, 255);                                            // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                            // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[22])));
			myFile.print(buffer);                                        // "Microphone MTT ON  Error! - "
			myFile.println(regcount);                                    // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[23])));
			myFile.println(buffer);                                     //"Microphone MTT  ON - Ok!" Микрофон диспетчера включился
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[24])));     // "Microphone MTT signal ON" 
	myFile.println(buffer);                                             // "Microphone MTT signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(20);
	//++++++++++++++++++++++++++++++++++ Проверить наличие сигнала на линиях FrontL    ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40143,143,200);                    // Измерить уровень сигнала на выходе LineL
	measure_vol_max(analog_mag_phone,40150,150,200);                    // Измерить уровень сигнала на выходе mag phone
   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                     // Измерить уровень сигнала на выходе FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                     // Измерить уровень сигнала на выходе FrontR 
	measure_vol_min(analog_ggs,      40146,146,30);                     // Измерить уровень сигнала на выходе GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                     // Измерить уровень сигнала на выходе GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                     // Измерить уровень сигнала на выходе GG Radio2
	regBank.set(32,0);                                                  // XP1- 1  HeS1Ls   Отключить  Sence подкючения MTT
	regBank.set(15,0);                                                  // РТТ микрофона отключить
	regBank.set(10,0);                                                  // Подать управляющую команду на вывод XP1 10  (Выключить микрофон MTT)
	regBank.set(30,0);                                                  // XP1- 6  HeS1PTT   Отключить PTT MTT
	regBank.set(28,0);                                                  // XP1- 15 HeS2PTT   CTS вкл PTT MTT
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                 // Завершить программу    
	delay(100);


	//myFile.println("");
	//regBank.set(10,0);                                                  // Подать управляющую команду на вывод XP1 10 Звук на микрофон диспетчера  отключить
	//regBank.set(1,0);                                                   // Отключить сигнал на вход микрофона диспетчера Mic1p
	//regBank.set(2,0);                                                   // Отключить сигнал на вход микрофона диспетчера Mic2p
	//regBank.set(3,1);                                                   // Включить сигнал на вход микрофона трубки Mic3p
	//regBank.set(5,0);                                                   // Подать управляющую команду на вывод 12 ХР1 HeS2e (отключить микрофон инструктора)
	//regBank.set(28,0);                                                  // XP1- 15 HeS2PTT Отключить PTT инструктора
	//regBank.set(16,0);                                                  // Сенсор микрофона отключить
	//regBank.set(15,0);                                                  // РТТ микрофона отключить
	//regBank.set(18,0);                                                  // XP1 - 20  HangUp  DCD
	//regBank.set(25,1);                                                  // XP1- 19 HaSs      Sence подключения трубки                                         
	//UpdateRegs();                                                       // 
	//delay(200);

	//	if(regBank.get(10083)!= 0)                                      // Проверить XP1 - 20  HangUp  DCD
	//		{
	//			regcount = regBank.get(40153);                          // адрес счетчика ошибки отключения XP1 - 20  HangUp  DCD
	//			regcount++;                                             // увеличить счетчик ошибок
	//			regBank.set(40153,regcount);                            // адрес счетчика ошибки отключения XP1 - 20  HangUp  DCD
	//			regBank.set(153,1);                                     // установить флаг ошибки отключения XP1 - 20  HangUp  DCD
	//			regBank.set(120,1);                                     // установить общий флаг ошибки
	//			myFile.print("Komanda XP1 - 20  HangUp  DCD OFF Error! - ");   //XP1 - 20  HangUp  DCD
	//			myFile.println(regcount);
	//		}
	//	else
	//		{
	//		   myFile.println("Komanda XP1 - 20  HangUp  DCD OFF- Ok!"); // XP1 - 20  HangUp  DCD
	//		}

	//	byte i5 = regs_in[1];                                      // 
	//	if(bitRead(i5,2) == 0)                                           // Проверка  подключения сенсора трубки
	//	  {
	//		regcount = regBank.get(40154);                               // адрес счетчика ошибки подключения сенсора трубки
	//		regcount++;                                                  // увеличить счетчик ошибок подключения сенсора трубки
	//		regBank.set(40154,regcount);                                 // адрес счетчика ошибки подключения сенсора трубки
	//		regBank.set(154,1);                                          // установить флаг ошибки
	//		regBank.set(120,1);                                          // установить общий флаг ошибки
	//		resistor(1, 255);                                            // Установить уровень сигнала в исходное состояниу
	//		resistor(2, 255);                                            // Установить уровень сигнала в исходное состояниу
	//		myFile.print("Sence MTT ON  Error! - ");    // 
	//		myFile.println(regcount);                                    // 
	//	  }
	//	else
	//	  {
	//		myFile.println("Sence MTT ON - Ok!");                        // Сенсор трубки включился
	//	  }
}


void test_instr_off()
{
	  unsigned int regcount = 0;
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[0])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF instruktora   send!"
	  regBank.set(29,0);                                                // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[1])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF instruktora 2 send!"
	  regBank.set(27,0);                                                // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	  regBank.set(16,0);                                                // XS1 - 6   Sence Мик
	  regBank.set(1,0);                                                 // Реле RL0 Звук
	  regBank.set(2,0);                                                 // Реле RL1 Звук
	  regBank.set(3,0);                                                 // Реле RL2 Звук
	  regBank.set(4,0);                                                 // Реле RL3 Звук  LFE  "Маг."
	  regBank.set(5,0);                                                 // Реле RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                 // Реле RL5 Звук
	  regBank.set(7,0);                                                 // Реле RL6 Звук
	  regBank.set(9,0);                                                 // Реле RL8 Звук на микрофон
	  regBank.set(10,0);                                                // Реле RL9 XP1 10
	  regBank.set(28,0);                                                // XP1- 15 HeS2Ls Отключить PTT инструктора
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[2])));
	  myFile.println(buffer);                                           // "Komanda PTT instruktora OFF      send!"

	  UpdateRegs();                                                     // Выполнить команду отключения сенсоров
	  delay(400);                                                       // 
	 
	  // 1)  Проверка сенсора на отключение гарнитуры инструктора 2 наушниками
	  byte i5 = regs_in[2];                                             // 
		if(bitRead(i5,1) > 0)                                           // Проверка  флага на отключение гарнитуры инструктора 2 наушниками
		  {
			regcount = regBank.get(40127);                              // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40127,regcount);                                // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
			regBank.set(127,1);                                         // установить флаг ошибки
			regBank.set(120,1);                                         // установить общий флаг ошибки
			resistor(1, 255);                                           // Установить уровень сигнала в исходное состояниу
			resistor(2, 255);                                           // Установить уровень сигнала в исходное состояниу
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[3])));
			myFile.print(buffer);                                       // "Komanda sensor OFF instruktora 2 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[4])));
			myFile.println(buffer);                                     // "Komanda sensor OFF instruktora 2 - Ok!"
		  }

	  // 2)  Проверка сенсора на отключение гарнитуры инструктора

		if(bitRead(i5,2) > 0)                                           // Проверка  флага на отключение гарнитуры инструктора 
		  {
			 regcount = regBank.get(40128);                             // адрес счетчика ошибки сенсора гарнитуры инструктора
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40128,regcount);                               // адрес счетчика ошибки сенсора гарнитуры инструктора
			 regBank.set(128,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[5])));
			 myFile.print(buffer);                                      // "Komanda sensor OFF instruktora  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[6])));
			 myFile.println(buffer);                                    // "Komanda sensor OFF instruktora   - Ok!"
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i5,5) > 0)                                           // Проверка  флага на отключение микрофона
		  {
			 regcount = regBank.get(40149);                             // адрес счетчика ошибки сенсора микрофона 
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40149,regcount);                               // адрес счетчика ошибки сенсора микрофона
			 regBank.set(149,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[7])));
			 myFile.print(buffer);                                      // "Komanda sensor microphone OFF Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[8])));
			 myFile.println(buffer);                                    // "Komanda sensor microphone OFF    - Ok!"
		  }
		UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS)!= 0)                                       // Проверить включение PTT инструктора   CTS
		  {
			regcount = regBank.get(40140);                              // адрес счетчика ошибки отключения PTT гарнитуры инструктора
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40140,regcount);                                // адрес счетчика ошибки отключения PTT гарнитуры инструктора
			regBank.set(140,1);                                         // установить флаг ошибки отключения PTT гарнитуры инструктора
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[9])));
			myFile.print(buffer);                                       // "Komanda PTT instruktora(CTS) OFF Error! - " Микрофон не отключился 
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[10])));
			myFile.println(buffer);                                     // "Komanda PTT instruktora (CTS) OFF- Ok!"
		 }
}
void test_instr_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[0])));
	myFile.println(buffer);                                             // "Komanda sensor ON instruktora    send!"
	regBank.set(29,1);                                                  // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[1])));
	myFile.println(buffer);                                             // "Komanda sensor ON instruktora 2  send!"
	regBank.set(27,1);                                                  // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
	regBank.set(19,1);                                                  // J8-11     XP7 2 Sence  Танг. р.
	regBank.set(16,1);                                                  // XS1 - 6   Sence Мик
	regBank.set(25,1);                                                  // XP1- 19 HaSs      Sence подключения трубки      
	regBank.set(13,1);                                                  // XP8 - 2           Sence Тангента ножная
	regBank.set(28,1);                                                  // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[8])));
	myFile.println(buffer);                                             // "Komanda PTT ON instruktora  send!"

	UpdateRegs(); 
	delay(300);

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
		 byte i5 = regs_in[2];   
		 if(bitRead(i5,1) == 0)                                         // Проверка флага подключения гарнитуры инструктора 2 наушниками
			{
				regcount = regBank.get(40127);                          // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40127,regcount);                            // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
				regBank.set(127,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[2])));
				myFile.print(buffer);                                   // "Komanda sensor ON instruktora 2 Error! - "
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[3])));
				myFile.println(buffer);                                 // "Komanda sensor ON  instruktora 2 - Ok!"
			}
	  // 4)  Проверка сенсора на подключения гарнитуры инструктора

	   if(bitRead(i5,2) == 0)                                            // Проверка флага подключения гарнитуры инструктора
			 {
				regcount = regBank.get(40128);                          // адрес счетчика ошибки сенсора гарнитуры инструктора
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40128,regcount);                            // адрес счетчика ошибки сенсора гарнитуры инструктора
				regBank.set(128,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[4])));
				myFile.print(buffer);                                   // "Komanda sensor ON  instruktora  Error! - "
				myFile.println(regcount);
			 }
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[5])));
				myFile.println(buffer);                                // "Komanda sensor ON instruktora    - Ok!"
			}
		UpdateRegs(); 

		 if(regBank.get(adr_reg_ind_CTS)== 0)                          // Проверить включение PTT инструктора   CTS
			{
				regcount = regBank.get(40140);                         // адрес счетчика ошибки отключения PTT гарнитуры инструктора
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40140,regcount);                           // адрес счетчика ошибки отключения PTT гарнитуры инструктора
				regBank.set(140,1);                                    // установить флаг ошибки отключения PTT гарнитуры инструктора
				regBank.set(120,1);                                    // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[6])));
				myFile.print(buffer);                                  // "Komanda PTT instruktora(CTS) ON Error! - " Микрофон не отключился 
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[7])));
				myFile.println(buffer);                               // "Komanda PTT instruktora (CTS) ON- Ok!"
			}
}

void test_disp_off()
{
	  unsigned int regcount = 0;
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[0])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF диспетчера  send!"
	  regBank.set(32,0);                                                // XP1- 1  HeS1Ls    Отключить сенсор гарнитуры диспетчера
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[1])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF диспетчера 2 send!"
	  regBank.set(31,0);                                                // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	  regBank.set(16,0);                                                // XS1 - 6   Sence Мик
	  regBank.set(1,0);                                                 // Реле RL0 Звук
	  regBank.set(2,0);                                                 // Реле RL1 Звук
	  regBank.set(3,0);                                                 // Реле RL2 Звук
	  regBank.set(4,0);                                                 // Реле RL3 Звук  LFE  "Маг."
	  regBank.set(5,0);                                                 // Реле RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                 // Реле RL5 Звук
	  regBank.set(7,0);                                                 // Реле RL6 Звук
	  regBank.set(9,0);                                                 // Реле RL8 Звук на микрофон
	  regBank.set(10,0);                                                // Реле RL9 XP1 10
	  regBank.set(30,0);                                                // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[2])));
	  myFile.println(buffer);                                           // "Komanda PTT диспетчера OFF send!"

	  UpdateRegs();                                                     // Выполнить команду отключения сенсоров
	  delay(400);                                                       // 
	 
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
	  byte i5 = regs_in[2];                                             // 
		if(bitRead(i5,3) > 0)                                           // Проверка  флага на отключение гарнитуры диспетчера 2 наушниками
		  {
			regcount = regBank.get(40129);                              // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40129,regcount);                                // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
			regBank.set(129,1);                                         // установить флаг ошибки
			regBank.set(120,1);                                         // установить общий флаг ошибки
			resistor(1, 255);                                           // Установить уровень сигнала в исходное состояниу
			resistor(2, 255);                                           // Установить уровень сигнала в исходное состояниу
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[3])));
			myFile.print(buffer);                                       // "Komanda sensor OFF dispetchera 2 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[4])));
			myFile.println(buffer);                                     // "Komanda sensor OFF dispetchera 2 - Ok!"
		  }

	  // 2)  Проверка сенсора на отключение гарнитуры диспетчера

		if(bitRead(i5,4) > 0)                                           // Проверка  флага на отключение гарнитуры диспетчера 
		  {
			 regcount = regBank.get(40130);                             // адрес счетчика ошибки сенсора гарнитуры диспетчера
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40130,regcount);                               // адрес счетчика ошибки сенсора гарнитуры диспетчера
			 regBank.set(130,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[5])));
			 myFile.print(buffer);                                      // "Komanda sensor OFF dispetchera  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[6])));
			 myFile.println(buffer);                                    // "Komanda sensor OFF dispetchera   - Ok!"
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i5,5) > 0)                                           // Проверка  флага на отключение микрофона
		  {
			 regcount = regBank.get(40149);                             // адрес счетчика ошибки сенсора микрофона 
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40149,regcount);                               // адрес счетчика ошибки сенсора микрофона
			 regBank.set(149,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[7])));
			 myFile.print(buffer);                                      // "Komanda sensor microphone OFF Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[8])));
			 myFile.println(buffer);                                    // "Komanda sensor microphone OFF    - Ok!"
		  }

	   UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS)!= 0)                             // Проверить включение PTT инструктора   CTS
		  {
			regcount = regBank.get(40158);                              // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40158,regcount);                                // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
			regBank.set(158,1);                                         // установить флаг ошибки отключения PTT гарнитуры диспетчера
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[9])));
			myFile.print(buffer);                                       // "Komanda PTT dispetchera (CTS) OFF Error! - " Микрофон не отключился 
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[10])));
			myFile.println(buffer);                                     // "Komanda PTT dispetchera (CTS) OFF- Ok!"
		 }
}
void test_disp_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[0])));
	myFile.println(buffer);                                       // "Komanda sensor ON dispetchera send!"
	regBank.set(32,1);                                            // XP1- 1  HeS1Ls    Sence подключения гарнитуры диспетчера 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[1])));
	myFile.println(buffer);                                       // "Komanda sensor ON dispetchera 2  send!"
	regBank.set(31,1);                                            // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(19,1);                                            // J8-11     XP7 2 Sence  Танг. р.
	regBank.set(16,1);                                            // XS1 - 6   Sence Мик
	regBank.set(25,1);                                            // XP1- 19 HaSs      Sence подключения трубки      
	regBank.set(13,1);                                            // XP8 - 2           Sence Тангента ножная
	regBank.set(30,1);                                            // XP1- 6  HeS1PTT   Включить PTT диспетчера
	UpdateRegs(); 
	delay(300);

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
		 byte i5 = regs_in[2];   
		 if(bitRead(i5,3) == 0)                                         // Проверка флага подключения гарнитуры диспетчера 2 наушниками
			{
				regcount = regBank.get(40129);                          // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40129,regcount);                            // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
				regBank.set(129,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[2])));
				myFile.print(buffer);                                   // "Komanda sensor ON dispetchera 2 Error! - "
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[3])));
				myFile.println(buffer);                                 // "Komanda sensor ON  dispetchera 2 - Ok!"
			}
	  // 4)  Проверка сенсора на подключения гарнитуры инструктора

	   if(bitRead(i5,4) == 0)                                            // Проверка флага подключения гарнитуры диспетчера
			 {
				regcount = regBank.get(40130);                          // адрес счетчика ошибки сенсора гарнитуры диспетчера
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40130,regcount);                            // адрес счетчика ошибки сенсора гарнитуры диспетчера
				regBank.set(130,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[4])));
				myFile.print(buffer);                                   // "Komanda sensor ON  dispetchera  Error! - "
				myFile.println(regcount);
			 }
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[5])));
				myFile.println(buffer);                                // "Komanda sensor ON dispetchera    - Ok!"
			}

		 UpdateRegs(); 
		 if(regBank.get(adr_reg_ind_CTS)== 0)                          // Проверить включение PTT диспетчера   CTS
			{
				regcount = regBank.get(40158);                         // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40158,regcount);                           // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
				regBank.set(158,1);                                    // установить флаг ошибки отключения PTT гарнитуры диспетчера
				regBank.set(120,1);                                    // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[6])));
				myFile.print(buffer);                                  // "Komanda PTT dispetchera OFF Error! - " Микрофон не отключился 
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[7])));
				myFile.println(buffer);                               // "Komanda PTT dispetchera OFF- Ok!"
			}
	//	delay(100);
}

void test_MTT_off()
{
	  unsigned int regcount = 0;
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[0])));
	  myFile.println(buffer);                                           // "Komanda sence OFF MTT            send!                  
	  regBank.set(25,0);                                                // XP1- 1  HeS1Ls    Отключить сенсор гарнитуры диспетчера
	  regBank.set(16,0);                                                // XS1 - 6   Sence Мик
	  regBank.set(1,0);                                                 // Реле RL0 Звук
	  regBank.set(2,0);                                                 // Реле RL1 Звук
	  regBank.set(3,0);                                                 // Реле RL2 Звук
	  regBank.set(4,0);                                                 // Реле RL3 Звук  LFE  "Маг."
	  regBank.set(5,0);                                                 // Реле RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                 // Реле RL5 Звук
	  regBank.set(7,0);                                                 // Реле RL6 Звук
	  regBank.set(9,0);                                                 // Реле RL8 Звук на микрофон
	  regBank.set(10,0);                                                // Реле RL9 XP1 10
	  regBank.set(18,0);                                                // XP1 - 20  HangUp  DCD
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[2])));
	  myFile.println(buffer);                                           // "Komanda PTT диспетчера OFF send!"

	  UpdateRegs();                                                     // Выполнить команду отключения сенсоров
	  delay(400);                                                       // 
	 
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
	  byte i5 = regs_in[2];                                             // 
		if(bitRead(i5,3) > 0)                                           // Проверка  флага на отключение гарнитуры диспетчера 2 наушниками
		  {
			regcount = regBank.get(40129);                              // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40129,regcount);                                // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
			regBank.set(129,1);                                         // установить флаг ошибки
			regBank.set(120,1);                                         // установить общий флаг ошибки
			resistor(1, 255);                                           // Установить уровень сигнала в исходное состояниу
			resistor(2, 255);                                           // Установить уровень сигнала в исходное состояниу
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[3])));
			myFile.print(buffer);                                       // "Komanda sensor OFF dispetchera 2 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[4])));
			myFile.println(buffer);                                     // "Komanda sensor OFF dispetchera 2 - Ok!"
		  }

	  // 2)  Проверка сенсора на отключение гарнитуры диспетчера

		if(bitRead(i5,4) > 0)                                           // Проверка  флага на отключение гарнитуры диспетчера 
		  {
			 regcount = regBank.get(40130);                             // адрес счетчика ошибки сенсора гарнитуры диспетчера
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40130,regcount);                               // адрес счетчика ошибки сенсора гарнитуры диспетчера
			 regBank.set(130,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[5])));
			 myFile.print(buffer);                                      // "Komanda sensor OFF dispetchera  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[6])));
			 myFile.println(buffer);                                    // "Komanda sensor OFF dispetchera   - Ok!"
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i5,5) > 0)                                           // Проверка  флага на отключение микрофона
		  {
			 regcount = regBank.get(40149);                             // адрес счетчика ошибки сенсора микрофона 
			 regcount++;                                                // увеличить счетчик ошибок
			 regBank.set(40149,regcount);                               // адрес счетчика ошибки сенсора микрофона
			 regBank.set(149,1);                                        // установить флаг ошибки
			 regBank.set(120,1);                                        // установить общий флаг ошибки
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[7])));
			 myFile.print(buffer);                                      // "Komanda sensor microphone OFF Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[8])));
			 myFile.println(buffer);                                    // "Komanda sensor microphone OFF    - Ok!"
		  }

	   UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS)!= 0)                             // Проверить включение PTT инструктора   CTS
		  {
			regcount = regBank.get(40158);                              // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
			regcount++;                                                 // увеличить счетчик ошибок
			regBank.set(40158,regcount);                                // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
			regBank.set(158,1);                                         // установить флаг ошибки отключения PTT гарнитуры диспетчера
			regBank.set(120,1);                                         // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[9])));
			myFile.print(buffer);                                       // "Komanda PTT dispetchera (CTS) OFF Error! - " Микрофон не отключился 
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[10])));
			myFile.println(buffer);                                     // "Komanda PTT dispetchera (CTS) OFF- Ok!"
		 }
}
void test_MTT_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[0])));
	myFile.println(buffer);                                       // "Komanda sensor ON dispetchera send!"
	regBank.set(32,1);                                            // XP1- 1  HeS1Ls    Sence подключения гарнитуры диспетчера 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[1])));
	myFile.println(buffer);                                       // "Komanda sensor ON dispetchera 2  send!"
	regBank.set(31,1);                                            // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(19,1);                                            // J8-11     XP7 2 Sence  Танг. р.
	regBank.set(16,1);                                            // XS1 - 6   Sence Мик
	regBank.set(25,1);                                            // XP1- 19 HaSs      Sence подключения трубки      
	regBank.set(13,1);                                            // XP8 - 2           Sence Тангента ножная
	regBank.set(30,1);                                            // XP1- 6  HeS1PTT   Включить PTT диспетчера
	UpdateRegs(); 
	delay(300);

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
		 byte i5 = regs_in[2];   
		 if(bitRead(i5,3) == 0)                                         // Проверка флага подключения гарнитуры диспетчера 2 наушниками
			{
				regcount = regBank.get(40129);                          // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40129,regcount);                            // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками
				regBank.set(129,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[2])));
				myFile.print(buffer);                                   // "Komanda sensor ON dispetchera 2 Error! - "
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[3])));
				myFile.println(buffer);                                 // "Komanda sensor ON  dispetchera 2 - Ok!"
			}
	  // 4)  Проверка сенсора на подключения гарнитуры инструктора

	   if(bitRead(i5,4) == 0)                                            // Проверка флага подключения гарнитуры диспетчера
			 {
				regcount = regBank.get(40130);                          // адрес счетчика ошибки сенсора гарнитуры диспетчера
				regcount++;                                             // увеличить счетчик ошибок
				regBank.set(40130,regcount);                            // адрес счетчика ошибки сенсора гарнитуры диспетчера
				regBank.set(130,1);                                     // установить флаг ошибки
				regBank.set(120,1);                                     // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[4])));
				myFile.print(buffer);                                   // "Komanda sensor ON  dispetchera  Error! - "
				myFile.println(regcount);
			 }
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[5])));
				myFile.println(buffer);                                // "Komanda sensor ON dispetchera    - Ok!"
			}

		 UpdateRegs(); 
		 if(regBank.get(adr_reg_ind_CTS)== 0)                          // Проверить включение PTT диспетчера   CTS
			{
				regcount = regBank.get(40158);                         // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
				regcount++;                                            // увеличить счетчик ошибок
				regBank.set(40158,regcount);                           // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
				regBank.set(158,1);                                    // установить флаг ошибки отключения PTT гарнитуры диспетчера
				regBank.set(120,1);                                    // установить общий флаг ошибки
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[6])));
				myFile.print(buffer);                                  // "Komanda PTT dispetchera OFF Error! - " Микрофон не отключился 
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[7])));
				myFile.println(buffer);                               // "Komanda PTT dispetchera OFF- Ok!"
			}
	//	delay(100);
}


void measure_vol_min(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                               // Измерить уровень сигнала на выходе
		switch (adr_flagErr) 
		{
			case 141:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[40])));  // "Signal FrontL OFF    - "
				//myFile.print(buffer);              // "Signal FrontL OFF    - "
				break;
			case 142:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[41])));  // "Signal FrontR OFF    - "
				//myFile.print(buffer);              // "Signal FrontR OFF    - "
				break;
			case 144:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[42])));  // "Signal mag radio OFF - "
				//myFile.print(buffer);              // "Signal mag radio OFF - "
				break;
			case 145:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[43])));  // "Signal mag phone OFF - "
				//myFile.print(buffer);              // "Signal mag phone OFF - "
				break;
			case 146:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[44])));   // "Signal GGS OFF       - "
				//myFile.print(buffer);              // "Signal GGS OFF       - "
				break;
			case 147:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[45])));  // "Signal GG Radio1 OFF - "
				//print(buffer);              // "Signal GG Radio1 OFF - "
				break;
			case 148:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[46])));   // "Signal GG Radio2 OFF - "
				//myFile.print(buffer);              // "Signal GG Radio2 OFF - "
				break;
			case 160:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[40])));   // "Signal FrontL OFF    - "
				break;
			case 161:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[41])));   // "Signal FrontR OFF    - "
				break;
			case 10:
				//выполняется 
				break;
			case 11:
				//выполняется
				break;
			case 12:
				//выполняется
				break;
			case 13:
				//выполняется 
				break;



		}

		myFile.print(buffer);                                          // 

		if(voltage10 >  porogV)                                        // Проверить исправность канала
			{
				regcount = regBank.get(adr_count);                     // адрес счетчика ошибки 
				regcount++;                                            // увеличить счетчик ошибок канала 
				regBank.set(adr_count,regcount);                       // адрес счетчика ошибки канала 
				regBank.set(adr_flagErr,1);                            // установить флаг ошибки  канала 
				regBank.set(120,1);                                    // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[47])));   // "Error! - "
				myFile.print(buffer);                                  // "Error! - "
				myFile.println(regcount);
			}
		else
			{
				myFile.println("Ok!");
			}                                     
}
void measure_vol_max(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                                  // Измерить уровень сигнала на выходе
		switch (adr_flagErr) 
		{
			case 141:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[50])));     // "Signal FrontL OFF    - "
			//	myFile.print(buffer);                                            
				break;
			case 142:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[51])));     // "Signal FrontR OFF    - "
				//myFile.print(buffer);                                             // "Signal FrontR OFF    - "
				break;
			case 143:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[52])));     // "Signal FrontL ON     - "
			//	myFile.print(buffer);                                               // "Signal FrontL ON     - "
				break;
			case 144:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[53])));     // "Signal mag radio OFF - "
				//myFile.print(buffer);                                               // "Signal mag radio OFF - "
				break;
			case 145:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[54])));     // "Signal mag phone OFF - "
			//	myFile.print(buffer);                                               // "Signal mag phone OFF - "
				break;
			case 146:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[55])));     // "Signal GGS OFF       - "
			//	myFile.print(buffer);                                               // "Signal GGS OFF       - "
				break;
			case 147:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[56])));     // "Signal GG Radio1 OFF - "
			//	myFile.print(buffer);                                               // "Signal GG Radio1 OFF - "
				break;
			case 148:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[57])));     // "Signal GG Radio2 OFF - "
			//	myFile.print(buffer);                                               // "Signal GG Radio2 OFF - "
				break;
			case 150:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[58])));     // "Mag phone on         - "
		   //  myFile.print(buffer);                                                // "Mag phone on         - "
				break;
			 case 160:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[50])));     // "Signal FrontL OFF    - "
				break;
			case 161:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[51])));     // "Signal FrontR OFF    - "
				break;
			case 9:
				//выполняется, когда var равно 1
				break;
			case 10:
				//выполняется когда  var равно 2
				break;
		}
		myFile.print(buffer);    
		if(voltage10 <  porogV)                                       // Проверить исправность канала
			{
				regcount = regBank.get(adr_count);                     // адрес счетчика ошибки 
				regcount++;                                            // увеличить счетчик ошибок канала 
				regBank.set(adr_count,regcount);                       // адрес счетчика ошибки канала 
				regBank.set(adr_flagErr,1);                            // установить флаг ошибки  канала 
				regBank.set(120,1);                                    // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[47])));
				myFile.print(buffer);                                      // "Error! - "
				myFile.println(regcount);
			}
		else
			{
				myFile.println("Ok!");
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
	regBank.add(13);    // XP8 - 2   Sence Тангента ножная
	regBank.add(14);    // XP8 - 1   PTT Тангента ножная
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
 
	regBank.add(25);    // XP1- 19 HaSs      Sence подключения трубки    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.add(27);    // XP1- 16 HeS2Rs    Sence подключения гарнитуры инструктора с 2 наушниками
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.add(29);    // XP1- 13 HeS2Ls    Sence подключения гарнитуры инструктора 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS вкл
	regBank.add(31);    // XP1- 5  HeS1Rs    Sence подкючения гарнитуры диспетчера с 2 наушниками
	regBank.add(32);    // XP1- 1  HeS1Ls    Sence подкючения гарнитуры диспетчера

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
	regBank.add(143);   // Флаг ошибки сигнала  LineL гарнитуры инструктора 
	regBank.add(144);   // Флаг ошибки analog_mag_radio
	regBank.add(145);   // Флаг ошибки analog_mag_phone
	regBank.add(146);   // Флаг ошибки analog_ggs
	regBank.add(147);   // Флаг ошибки analog_gg_radio1
	regBank.add(148);   // Флаг ошибки analog_gg_radio2
	regBank.add(149);   // Флаг ошибки сенсора микрофона
	regBank.add(150);   // Флаг ошибки Mag phone on
	regBank.add(151);   // Флаг ошибки включения микрофона инструктора
	regBank.add(152);   // Флаг ошибки включения микрофона диспетчера
	regBank.add(153);   // Флаг ошибки XP1 - 20  HangUp  DCD
	regBank.add(154);   // Флаг ошибки Sence MTT ON
	regBank.add(155);   // Флаг ошибки отключения микрофона гарнитуры диспетчера
	regBank.add(156);   // адрес счетчика ошибки отключения радиопередачи
	regBank.add(157);   // адрес счетчика ошибки включения радиопередачи
	regBank.add(158);   // Флаг ошибки отключения PTT гарнитуры диспетчера
	regBank.add(159);   // Флаг ошибки отключения микрофона гарнитуры диспетчера
	regBank.add(160);   // Флаг ошибки динамика гарнитуры диспетчера FrontL
	regBank.add(161);   // Флаг ошибки динамика гарнитуры диспетчера FrontR
	regBank.add(162);   // Флаг ошибки сигнала  LineL гарнитуры диспетчера 


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
	//regBank.add(30045);  // байт 3 прием бит 5 - Камертон   флаг подключения микрофона XS1 - 6 Sence
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
	regBank.add(40121);  // адрес счетчика ошибки сенсора Sence "ГГ-Радио1."  ok!
	regBank.add(40122);  // адрес счетчика ошибки сенсора Sence "ГГ-Радио2."  ok!
	regBank.add(40123);  // адрес счетчика ошибки сенсора подключения трубки
	regBank.add(40124);  // адрес счетчика ошибки сенсора  Sence Танг. р. ok!
	regBank.add(40125);  // адрес счетчика ошибки сенсора Sence Танг н. ok!
	regBank.add(40126);  // адрес счетчика ошибки сенсора Sence "Маг."   ok!
	regBank.add(40127);  // адрес счетчика ошибки сенсора гарнитуры инструктора с 2 наушниками
	regBank.add(40128);  // адрес счетчика ошибки сенсора гарнитуры инструктора
	regBank.add(40129);  // адрес счетчика ошибки сенсора гарнитуры диспетчера с 2 наушниками

	regBank.add(40130);  // адрес счетчика ошибки сенсора гарнитуры диспетчера
	regBank.add(40131);  // адрес счетчика ошибки сенсора Sence Мик. ok!
	regBank.add(40132);  // адрес счетчика ошибки сенсора Sence "ГГC."   ok!
	regBank.add(40133);  // адрес счетчика ошибки сенсора 
	regBank.add(40134);  // адрес счетчика ошибки сенсора PTT Танг н.  ok!
	regBank.add(40135);  // адрес счетчика ошибки сенсора PTT Мик ok!
	regBank.add(40136);  // адрес счетчика ошибки  PTT2 Танг. р. ok!
	regBank.add(40137);  // адрес счетчика ошибки  HangUp  DCD  ok!
	regBank.add(40138);  // адрес счетчика ошибки  PTT1 Танг. р. ok!
	regBank.add(40139);  // адрес счетчика ошибки отключения микрофона гарнитуры инструктора 

	regBank.add(40140);  // адрес счетчика ошибки отключения PTT гарнитуры инструктора
	regBank.add(40141);  // адрес счетчика ошибки динамика гарнитуры инструктора FrontL
	regBank.add(40142);  // адрес счетчика ошибки динамика гарнитуры инструктора FrontR
	regBank.add(40143);  // адрес счетчика ошибки LineL
	regBank.add(40144);  // адрес счетчика ошибки analog_mag_radio
	regBank.add(40145);  // адрес счетчика ошибки analog_mag_phone
	regBank.add(40146);  // адрес счетчика ошибки analog_ggs
	regBank.add(40147);  // адрес счетчика ошибки analog_gg_radio1
	regBank.add(40148);  // адрес счетчика ошибки analog_gg_radio2
	regBank.add(40149);  // адрес счетчика ошибки сенсора микрофона
	regBank.add(40150);  // адрес счетчика ошибки  Mag phone on
	regBank.add(40151);  // адрес счетчика ошибки включения микрофона инструктора 
	regBank.add(40152);  // адрес счетчика ошибки включения микрофона диспетчера
	regBank.add(40153);  // адрес счетчика ошибки XP1 - 20  HangUp  DCD
	regBank.add(40154);  // адрес счетчика ошибки Sence MTT ON
	regBank.add(40155);  // адрес счетчика ошибки отключения микрофона гарнитуры диспетчера
	regBank.add(40156);  // адрес счетчика ошибки отключения радиопередачи
	regBank.add(40157);  // адрес счетчика ошибки включения радиопередачи
	regBank.add(40158);  // адрес счетчика ошибки отключения PTT гарнитуры диспетчера
	regBank.add(40159);  // адрес счетчика ошибки отключения микрофона гарнитуры диспетчера
	regBank.add(40160);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontL
	regBank.add(40161);  // адрес счетчика ошибки динамика гарнитуры диспетчера FrontR
	regBank.add(40162);  // адрес счетчика ошибки сигнала  LineL гарнитуры диспетчера 

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
