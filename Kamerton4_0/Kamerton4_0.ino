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
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Wire.h> 
#include <RTClib.h>
#include <MsTimer2.h> 
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
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

//-----------------------------------------------------------------------------------------------

uint8_t second = 0;       //Initialization time
uint8_t minute = 0;
uint8_t hour   = 0;
uint8_t dow    = 1;
uint8_t day    = 1;
uint8_t month  = 1;
uint16_t year  = 15 ;

//------------------------------------------------------------------------------------------------------------


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
int regcount_err        = 0;                                     // Переменная для хранения всех ошибок



//++++++++++++++++++++++ Работа с файлами +++++++++++++++++++++++++++++++++++++++
#define chipSelect SS
SdFat sd;
File myFile;
SdFile file;
// созданы переменные, использующие функции библиотеки SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;

//++++++++++++++++++++ Назначение имени файла ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.TXT";
//------------------------------------------------------------------------------

char c;  // Для ввода символа с ком порта


// Serial output stream
ArduinoOutStream cout(Serial);

//*********************Работа с именем файла ******************************

//byte file_name_count = 0;
char str_day_file[3];
char str_day_file0[3];
char str_day_file10[3];
char str_mon_file[3];
char str_mon_file0[3];
char str_mon_file10[3];
char str_year_file[3];

char str0[10];
char str1[10];
char str2[10];

//+++++++++++++++++++++ Установки прерывания +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

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
volatile bool prer_Kmerton_Run = false;             // Флаг разрешение прерывания Камертон
#define BUFFER_SIZEK 64                             // Размер буфера Камертон не более 128 байт
unsigned char bufferK;                              // Счетчик количества принимаемых байт

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

const unsigned int adr_reg_temp_year      PROGMEM       = 40112; // Регистр хранения переменной год  
const unsigned int adr_reg_temp_mon       PROGMEM       = 40113; // Регистр хранения переменной месяц
const unsigned int adr_reg_temp_day       PROGMEM       = 40114; // Регистр хранения переменной день 
const unsigned int adr_reg_file_name      PROGMEM       = 40115; // Регистр хранения счетчик файлов  
const unsigned int adr_reg_file_tek       PROGMEM       = 40116; // Регистр хранения счетчик файлов  


const unsigned int adr_control_command    PROGMEM       = 40120; //Адрес передачи комманд на выполнение 
const unsigned int adr_reg_count_err      PROGMEM       = 40121; // Регистр хранения счетчик файлов  

const unsigned int adr_set_time           PROGMEM       = 36;    // адрес флаг установки

//---------------------------Тексты сообщений   ---------------------------------------------------
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
const char  txt_message13[]   PROGMEM            = "Signal headset dispatcher microphone 30 mV    ON"            ;   
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
const char  txt_message27[]   PROGMEM            = "Command sensor OFF MTT                           send! "     ;
const char  txt_message28[]   PROGMEM            = "Command PTT    OFF MTT                           send! "     ;
const char  txt_message29[]   PROGMEM            = "Command HangUp OFF MTT                           send! "     ;

const char  txt_message30[]   PROGMEM            = "Command sensor ON  MTT                           send!"      ;
const char  txt_message31[]   PROGMEM            = "Command PTT    ON  MTT                           send!"      ;
const char  txt_message32[]   PROGMEM            = "Command HangUp ON  MTT                           send!"      ;
const char  txt_message33[]   PROGMEM            = "Signal MTT microphone 30 mV                   ON"            ;
const char  txt_message34[]   PROGMEM            = "Microphone MTT signal                         ON"            ;  
const char  txt_message35[]   PROGMEM            = "Signal FrontL, FrontR                         ON "           ;
const char  txt_message36[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ******"                ;
const char  txt_message37[]   PROGMEM            = "Command sensor OFF tangenta ruchnaja             send!"      ;
const char  txt_message38[]   PROGMEM            = "Command PTT1   OFF tangenta ruchnaja             send!"      ;
const char  txt_message39[]   PROGMEM            = "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

const char  txt_message40[]   PROGMEM            = "Command sensor ON  tangenta ruchnaja             send!"      ;
const char  txt_message41[]   PROGMEM            = "Command PTT1   ON  tangenta ruchnaja             send!"      ;
const char  txt_message42[]   PROGMEM            = "Command PTT2   ON  tangenta ruchnaja             send!"      ;
const char  txt_message43[]   PROGMEM            = " ****** Test tangenta nognaja start! ******"                 ;
const char  txt_message44[]   PROGMEM            = "Command sensor OFF tangenta nognaja              send!"      ;
const char  txt_message45[]   PROGMEM            = "Command PTT    OFF tangenta nognaja              send!"      ;
const char  txt_message46[]   PROGMEM            = "Command sensor ON  tangenta nognaja              send!"      ;
const char  txt_message47[]   PROGMEM            = "Command PTT    ON  tangenta nognaja              send!"      ;
const char  txt_message48[]   PROGMEM            = " ****** Test GGS start! ******"      ;
const char  txt_message49[]   PROGMEM            = "Signal GGS  FrontL, FrontR   0,7V             ON"            ;

const char  txt_message50[]   PROGMEM            = " ****** Test Radio1 start! ******"                           ;
const char  txt_message51[]   PROGMEM            = "Signal Radio1 300 mV    LFE                   ON"            ;
const char  txt_message52[]   PROGMEM            = " ****** Test Radio2 start! ******"      ;
const char  txt_message53[]   PROGMEM            = "Signal Radio1 300 mV    Center                ON"            ;
const char  txt_message54[]   PROGMEM            = " ****** Test microphone start! ******"                       ;
const char  txt_message55[]   PROGMEM            = "Signal miсrophone 30  mV                      ON"            ;
const char  txt_message56[]   PROGMEM            = "Command PTT    OFF microphone                    send!"      ;
const char  txt_message57[]   PROGMEM            = "Command PTT    ON  microphone                    send!"      ;
const char  txt_message58[]   PROGMEM            = "Command sensor OFF microphone                    send!"      ;  
const char  txt_message59[]   PROGMEM            = "Command sensor ON  microphone                    send!"      ;

//++++++++++++++++++++++++++++++ Тексты ошибок ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
const char  txt_error19[]  PROGMEM             = "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
  
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

const char  txt_error50[]  PROGMEM             = "Test MTT ** Signal FrontL                                   OFF - ";
const char  txt_error51[]  PROGMEM             = "Test MTT ** Signal FrontR                                   OFF - ";
const char  txt_error52[]  PROGMEM             = "Test MTT ** Signal LineL                                    OFF - ";
const char  txt_error53[]  PROGMEM             = "Test MTT ** Signal LineR                                    OFF - ";
const char  txt_error54[]  PROGMEM             = "Test MTT ** Signal mag radio                                OFF - ";
const char  txt_error55[]  PROGMEM             = "Test MTT ** Signal mag phone                                OFF - ";
const char  txt_error56[]  PROGMEM             = "Test MTT ** Signal GGS                                      OFF - ";
const char  txt_error57[]  PROGMEM             = "Test MTT ** Signal GG Radio1                                OFF - ";
const char  txt_error58[]  PROGMEM             = "Test MTT ** Signal GG Radio2                                OFF - ";
const char  txt_error59[]  PROGMEM             = "Test MTT ** Signal GGS                                      ON  - ";

const char  txt_error60[]  PROGMEM             = "Test MTT ** Signal LineL                                    ON  - ";
const char  txt_error61[]  PROGMEM             = "Test MTT ** Signal LineR                                    ON  - ";  
const char  txt_error62[]  PROGMEM             = "Test MTT ** Signal Mag phone                                ON  - ";
const char  txt_error63[]  PROGMEM             = "Test MTT PTT    (CTS)                                       OFF - ";
const char  txt_error64[]  PROGMEM             = "Test microphone PTT  (CTS)                                  OFF - ";
const char  txt_error65[]  PROGMEM             = "Test MTT PTT    (CTS)                                       ON  - ";
const char  txt_error66[]  PROGMEM             = "Test microphone PTT  (CTS)                                  ON  - ";
const char  txt_error67[]  PROGMEM             = "Test MTT HangUp (DCD)                                       OFF - ";
const char  txt_error68[]  PROGMEM             = "Test MTT HangUp (DCD)                                       ON  - ";
const char  txt_error69[]  PROGMEM             = "";

const char  txt_error70[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
const char  txt_error71[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
const char  txt_error72[]  PROGMEM             = "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
const char  txt_error73[]  PROGMEM             = "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
const char  txt_error74[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2                 OFF - ";
const char  txt_error75[]  PROGMEM             = "Command sensor tangenta ruchnaja    XP7 - 2                 ON  - ";
const char  txt_error76[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2                 OFF - ";
const char  txt_error77[]  PROGMEM             = "Command sensor tangenta nognaja     XP8 - 2                 ON  - ";
const char  txt_error78[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1                 OFF - ";
const char  txt_error79[]  PROGMEM             = "Command PTT tangenta nognaja (CTS)  XP8 - 1                 ON  - ";

const char  txt_error80[]  PROGMEM             = "Test GGS ** Signal FrontL                                   OFF - ";
const char  txt_error81[]  PROGMEM             = "Test GGS ** Signal FrontR                                   OFF - ";
const char  txt_error82[]  PROGMEM             = "Test GGS ** Signal LineL                                    OFF - ";
const char  txt_error83[]  PROGMEM             = "Test GGS ** Signal LineR                                    OFF - ";
const char  txt_error84[]  PROGMEM             = "Test GGS ** Signal mag radio                                OFF - ";
const char  txt_error85[]  PROGMEM             = "Test GGS ** Signal mag phone                                OFF - ";
const char  txt_error86[]  PROGMEM             = "Test GGS ** Signal GGS                                      OFF - ";
const char  txt_error87[]  PROGMEM             = "Test GGS ** Signal GG Radio1                                OFF - ";
const char  txt_error88[]  PROGMEM             = "Test GGS ** Signal GG Radio2                                OFF - ";
const char  txt_error89[]  PROGMEM             = "Test GGS ** Signal GGS                                      ON  - ";

const char  txt_error90[]  PROGMEM             = "Test GGS ** Signal FrontL                                   ON  - ";
const char  txt_error91[]  PROGMEM             = "Test GGS ** Signal FrontR                                   ON  - ";
const char  txt_error92[]  PROGMEM             = "Test GGS ** Signal mag phone                                ON  - ";
const char  txt_error93[]  PROGMEM             = "";  
const char  txt_error94[]  PROGMEM             = "";
const char  txt_error95[]  PROGMEM             = ""; 
const char  txt_error96[]  PROGMEM             = "";    
const char  txt_error97[]  PROGMEM             = "";  
const char  txt_error98[]  PROGMEM             = "Test Microphone ** Signal mag phone                         ON  - ";  
const char  txt_error99[]  PROGMEM             = "Test Microphone ** Signal LineL                             ON  - "; 

const char  txt_error100[]  PROGMEM            = "Test Radio1 ** Signal FrontL                                OFF - ";
const char  txt_error101[]  PROGMEM            = "Test Radio1 ** Signal FrontR                                OFF - ";
const char  txt_error102[]  PROGMEM            = "Test Radio1 ** Signal LineL                                 OFF - ";
const char  txt_error103[]  PROGMEM            = "Test Radio1 ** Signal LineR                                 OFF - ";
const char  txt_error104[]  PROGMEM            = "Test Radio1 ** Signal mag radio                             OFF - ";
const char  txt_error105[]  PROGMEM            = "Test Radio1 ** Signal mag phone                             OFF - ";
const char  txt_error106[]  PROGMEM            = "Test Radio1 ** Signal GGS                                   OFF - ";
const char  txt_error107[]  PROGMEM            = "Test Radio1 ** Signal GG Radio1                             OFF - ";
const char  txt_error108[]  PROGMEM            = "Test Radio1 ** Signal GG Radio2                             OFF - ";
const char  txt_error109[]  PROGMEM            = "Test Radio1 ** Signal Radio1                                ON  - ";

const char  txt_error110[]  PROGMEM            = "Test Radio2 ** Signal FrontL                                OFF - ";
const char  txt_error111[]  PROGMEM            = "Test Radio2 ** Signal FrontR                                OFF - ";
const char  txt_error112[]  PROGMEM            = "Test Radio2 ** Signal LineL                                 OFF - ";
const char  txt_error113[]  PROGMEM            = "Test Radio2 ** Signal LineR                                 OFF - ";
const char  txt_error114[]  PROGMEM            = "Test Radio2 ** Signal mag radio                             OFF - ";
const char  txt_error115[]  PROGMEM            = "Test Radio2 ** Signal mag phone                             OFF - ";
const char  txt_error116[]  PROGMEM            = "Test Radio2 ** Signal GGS                                   OFF - ";
const char  txt_error117[]  PROGMEM            = "Test Radio2 ** Signal GG Radio1                             OFF - ";
const char  txt_error118[]  PROGMEM            = "Test Radio2 ** Signal GG Radio2                             OFF - ";
const char  txt_error119[]  PROGMEM            = "Test Radio2 ** Signal Radio2                                ON  - ";

const char  txt_error120[]  PROGMEM            = "Test Microphone ** Signal FrontL                            OFF - ";
const char  txt_error121[]  PROGMEM            = "Test Microphone ** Signal FrontR                            OFF - ";
const char  txt_error122[]  PROGMEM            = "Test Microphone ** Signal LineL                             OFF - ";
const char  txt_error123[]  PROGMEM            = "Test Microphone ** Signal LineR                             OFF - ";
const char  txt_error124[]  PROGMEM            = "Test Microphone ** Signal mag radio                         OFF - ";
const char  txt_error125[]  PROGMEM            = "Test Microphone ** Signal mag phone                         OFF - ";
const char  txt_error126[]  PROGMEM            = "Test Microphone ** Signal GGS                               OFF - ";
const char  txt_error127[]  PROGMEM            = "Test Microphone ** Signal GG Radio1                         OFF - ";
const char  txt_error128[]  PROGMEM            = "Test Microphone ** Signal GG Radio2                         OFF - ";
const char  txt_error129[]  PROGMEM            = "";

char buffer[130];  

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
txt_message27,                                // "Command sensor OFF MTT                           send! "     ;
txt_message28,                                // "Command PTT    OFF MTT                           send! "     ;
txt_message29,                                // "Command HangUp OFF MTT                           send! "     ;
txt_message30,                                // "Command sensor ON  MTT                           send!"      ;
txt_message31,                                // "Command PTT    ON  MTT                           send!"      ;
txt_message32,                                // "Command HangUp ON  MTT                           send!"      ;
txt_message33,                                // "Signal MTT microphone 30mv                    ON"            ;
txt_message34,                                // "Microphone MTT signal                         ON"            ;  
txt_message35,                                // "Signal FrontL, FrontR                         ON "           ;
txt_message36,                                // " ****** Test tangenta ruchnaja start! ******"                ;
txt_message37,                                // "Command sensor OFF tangenta ruchnaja             send!"      ;
txt_message38,                                // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
txt_message39,                                // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
txt_message40,                                // "Command sensor ON  tangenta ruchnaja             send!"      ;
txt_message41,                                // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
txt_message42,                                // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
txt_message43,                                // " ****** Test tangenta nognaja start! ******"                 ;
txt_message44,                                // "Command sensor OFF tangenta nognaja              send!"      ;
txt_message45,                                // "Command PTT    OFF tangenta nognaja              send!"      ;
txt_message46,                                // "Command sensor ON  tangenta nognaja              send!"      ;
txt_message47,                                // "Command PTT    ON  tangenta nognaja              send!"      ;
txt_message48,                                // " ****** Test GGS start! ******"      ;
txt_message49,                                // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;

txt_message50,                                // " ****** Test Radio1 start! ******"      ;
txt_message51,                                // "Signal Radio1 200 mV    LFE                   ON"            ;
txt_message52,                                //" ****** Test Radio2 start! ******"      ;
txt_message53,                                // "Signal Radio1 300 mV    Center                ON"            ;
txt_message54,                                // " ****** Test miсrophone start! ******"                       ;
txt_message55,                                // "Signal miсrophone 30  mV                      ON"            ;
txt_message56,                                // "Command PTT    OFF microphone                    send!"      ;
txt_message57,                                // "Command PTT    ON  microphone                    send!"      ;
txt_message58,                                // "Command sensor OFF microphone                    send!"      ;  
txt_message59                                 // "Command sensor ON  microphone                    send!"      ;

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
txt_error19,                                  // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 

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

txt_error50,                                  // "Test MTT ** Signal FrontL                                   OFF - ";
txt_error51,                                  // "Test MTT ** Signal FrontR                                   OFF - ";
txt_error52,                                  // "Test MTT ** Signal LineL                                    OFF - ";
txt_error53,                                  // "Test MTT ** Signal LineR                                    OFF - "; 
txt_error54,                                  // "Test MTT ** Signal mag radio                                OFF - ";
txt_error55,                                  // "Test MTT ** Signal mag phone                                OFF - ";
txt_error56,                                  // "Test MTT ** Signal GGS                                      OFF - ";
txt_error57,                                  // "Test MTT ** Signal GG Radio1                                OFF - ";
txt_error58,                                  // "Test MTT ** Signal GG Radio2                                OFF - "; 
txt_error59,                                  // "Test MTT ** Signal GGS                                      ON  - ";

txt_error60,                                  // "Test MTT ** Signal LineL                                    ON  - ";
txt_error61,                                  // "Test MTT ** Signal LineR                                    ON  - ";  
txt_error62,                                  // "Test MTT ** Signal Mag phone                                ON  - ";
txt_error63,                                  // "Test MTT PTT    (CTS)                                       OFF - ";
txt_error64,                                  // "Test microphone PTT  (CTS)                                  OFF - ";
txt_error65,                                  // "Test MTT PTT    (CTS)                                       ON  - ";
txt_error66,                                  // "Test microphone PTT  (CTS)                                  ON  - ";
txt_error67,                                  // "Test MTT HangUp (DCD)                                       OFF - ";
txt_error68,                                  // "Test MTT HangUp (DCD)                                       ON  - ";
txt_error69,                                  //

txt_error70,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
txt_error71,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
txt_error72,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
txt_error73,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
txt_error74,                                  // "Command sensor tangenta ruchnaja                            OFF - ";
txt_error75,                                  // "Command sensor tangenta ruchnaja                            ON  - ";
txt_error76,                                  // "Command sensor tangenta nognaja     XP8 - 2                 OFF - ";
txt_error77,                                  // "Command sensor tangenta nognaja     XP8 - 2                 ON  - ";
txt_error78,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 OFF - ";  
txt_error79,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 ON  - "; 

txt_error80,                                  // "Test GGS ** Signal FrontL                                   OFF - ";
txt_error81,                                  // "Test GGS ** Signal FrontR                                   OFF - ";
txt_error82,                                  // "Test GGS ** Signal LineL                                    OFF - ";
txt_error83,                                  // "Test GGS ** Signal LineR                                    OFF - ";
txt_error84,                                  // "Test GGS ** Signal mag radio                                OFF - ";
txt_error85,                                  // "Test GGS ** Signal mag phone                                OFF - ";
txt_error86,                                  // "Test GGS ** Signal GGS                                      OFF - ";
txt_error87,                                  // "Test GGS ** Signal GG Radio1                                OFF - ";
txt_error88,                                  // "Test GGS ** Signal GG Radio2                                OFF - ";
txt_error89,                                  // "Test GGS ** Signal GGS                                      ON  - ";

txt_error90,                                  // "Test GGS ** Signal FrontL                                   ON  - ";
txt_error91,                                  // "Test GGS ** Signal FrontR                                   ON  - ";
txt_error92,                                  // "Test GGS ** Signal mag phone                                ON  - ";
txt_error93,                                  // 
txt_error94,                                  // 
txt_error95,                                  // 
txt_error96,                                  // 
txt_error97,                                  // 
txt_error98,                                  // "Test Microphone ** Signal mag phone                         ON  - ";   
txt_error99,                                  // "Test Microphone ** Signal LineL                             ON  - "; 

txt_error100,                                 // "Test Radio1 ** Signal FrontL                                OFF - ";
txt_error101,                                 // "Test Radio1 ** Signal FrontR                                OFF - ";
txt_error102,                                 // "Test Radio1 ** Signal LineL                                 OFF - ";
txt_error103,                                 // "Test Radio1 ** Signal LineR                                 OFF - ";
txt_error104,                                 // "Test Radio1 ** Signal mag radio                             OFF - ";
txt_error105,                                 // "Test Radio1 ** Signal mag phone                             OFF - ";
txt_error106,                                 // "Test Radio1 ** Signal GGS                                   OFF - ";
txt_error107,                                 // "Test Radio1 ** Signal GG Radio1                             OFF - ";
txt_error108,                                 // "Test Radio1 ** Signal GG Radio2                             OFF - ";
txt_error109,                                 // "Test Radio1 ** Signal Radio1                                ON  - ";

txt_error110,                                 // "Test Radio2 ** Signal FrontL                                OFF - ";
txt_error111,                                 // "Test Radio2 ** Signal FrontR                                OFF - ";
txt_error112,                                 // "Test Radio2 ** Signal LineL                                 OFF - ";
txt_error113,                                 // "Test Radio2 ** Signal LineR                                 OFF - ";
txt_error114,                                 // "Test Radio2 ** Signal mag radio                             OFF - ";
txt_error115,                                 // "Test Radio2 ** Signal mag phone                             OFF - ";
txt_error116,                                 // "Test Radio2 ** Signal GGS                                   OFF - ";
txt_error117,                                 // "Test Radio2 ** Signal GG Radio1                             OFF - ";
txt_error118,                                 // "Test Radio2 ** Signal GG Radio2                             OFF - ";
txt_error119,                                 // "Test Radio2 ** Signal Radio2                                ON  - ";

txt_error120,                                 // "Test Microphone ** Signal FrontL                            OFF - ";
txt_error121,                                 // "Test Microphone ** Signal FrontR                            OFF - ";
txt_error122,                                 // "Test Microphone ** Signal LineL                             OFF - ";
txt_error123,                                 // "Test Microphone ** Signal LineR                             OFF - ";
txt_error124,                                 // "Test Microphone ** Signal mag radio                         OFF - ";
txt_error125,                                 // "Test Microphone ** Signal mag phone                         OFF - ";
txt_error126,                                 // "Test Microphone ** Signal GGS                               OFF - ";
txt_error127,                                 // "Test Microphone ** Signal GG Radio1                         OFF - ";
txt_error128,                                 // "Test Microphone ** Signal GG Radio2                         OFF - ";
txt_error129,                                 // ";

};



// ========================= Блок программ ============================================


void dateTime(uint16_t* date, uint16_t* time)                  // Программа записи времени и даты файла
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
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

const int8_t ERROR_LED_PIN = 13;


//------------------------------------------------------------------------------

// Size of file base name.  Must not be larger than six.
//Размер базовой части имени файла. Должно быть не больше, чем шесть.
//const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

//void file_name()
//{
//
//   preob_num_str();
//
//  while (sd.exists(fileName)) 
//  {
//	if (fileName[BASE_NAME_SIZE + 1] != '9') 
//	{
//	  fileName[BASE_NAME_SIZE + 1]++;
//	}
//	else if (fileName[BASE_NAME_SIZE] != '9') 
//	{
//	  fileName[BASE_NAME_SIZE + 1] = '0';
//	  fileName[BASE_NAME_SIZE]++;
//	}
//	else 
//	{
////	  sdError("Can't create file name");
//	}
//  }
//  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
//  //do {
//  //  delay(10);
//  // } while (Serial.read() >= 0);
//  //
//  Serial.print(F("Logging to: "));
//  Serial.println(fileName);
//  myFile.close();
//  Serial.println("done.");
//} 
//
//void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
//{
//	DateTime now = RTC.now();
//
//	day   = now.day();
//	month = now.month();
//	year  = now.year();
//
//
//	int year_temp = year-2000;
//
//	itoa (year_temp,str_year_file, 10);                                        // Преобразование даты год в строку ( 10 - десятичный формат) 
//
//	
//	if (month <10)
//		{
//		   itoa (0,str_mon_file0, 10);                                         //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
//		   itoa (month,str_mon_file10, 10);                                    //  Преобразование числа в строку ( 10 - десятичный формат) 
//		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // Сложение 2 строк
//		}
//	else
//		{
//		   itoa (month,str_mon_file, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
//		}
//
//
//	if (day <10)
//		{
//		   itoa (0,str_day_file0, 10);                                         // Преобразование числа в строку ( 10 - десятичный формат) 
//		   itoa (day,str_day_file10, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
//		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // Сложение 2 строк
//		}
//	else
//		{
//		itoa (day,str_day_file, 10);                                           // Преобразование числа в строку ( 10 - десятичный формат) 
//		}
//		 
//	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // Сложение 2 строк
//	sprintf(str2, "%s%s",str1, str_day_file);                                  // Сложение 2 строк
//	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // Получение имени файла в file_name
//	//Serial.println(fileName);
//}

//+++++++++++++++++++++++++++++++++++++++++++++Конецы  Форматирование SD ++++++++++++++++++++++++++


void set_time()
{
	RTC.adjust(DateTime(__DATE__, __TIME__));
//	DateTime now = RTC.now();
//	second = now.second();       //Initialization time
//	minute = now.minute();
//	hour   = now.hour();
//	day    =  now.day();
//	day++;
//	if(day > 31)day = 1;
//	month  = now.month();
//	year   = now.year();
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // Занести данные о времени в строку "set_time"
//	RTC.adjust(set_time);             
}

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
void Reg_count_clear()
{
	for(unsigned int i = 40200; i<=40319;i++)
	{
		regBank.set(i,0);
	}
		for(unsigned int i = 40400; i<=40519;i++)
	{
		regBank.set(i,0);
	}
	for(int k = 200; k<=319;k++)
	{
		regBank.set(k,false);
	}
	regBank.set(40121,0);
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
		RTC.adjust(set_time);                                                 // Записать время в контроллер часов  
		Serial.println("set_clock");
		regBank.set(adr_set_time, 0);                                         // Записать в регистр признак окончания выполнения команды
}
void data_clock_exchange()
{
	DateTime now = RTC.now();
	uint16_t year_temp = now.year()-2000;
	uint8_t day_temp = now.day();
	uint8_t mon_temp = now.month();

	byte  b = i2c_eeprom_read_byte(0x50, adr_temp_day);                             //access an address from the memory
		delay(10);

		if (b != day_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_day, day_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // Регистр  хранения переменной номер файла

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

// ++++++++++++++++++++++ Форматирование SD ++++++++++++++++++++++++++++


Sd2Card card;
uint32_t cardSizeBlocks;
uint16_t cardCapacityMB;

// cache for SD block
cache_t cache;

// MBR information
uint8_t partType;
uint32_t relSector;
uint32_t partSize;

// Fake disk geometry
uint8_t numberOfHeads;
uint8_t sectorsPerTrack;

// FAT parameters
uint16_t reservedSectors;
uint8_t sectorsPerCluster;
uint32_t fatStart;
uint32_t fatSize;
uint32_t dataStart;

// constants for file system structure
uint16_t const BU16 = 128;
uint16_t const BU32 = 8192;

//  strings needed in file system structures
char noName[] = "NO NAME    ";
char fat16str[] = "FAT16   ";
char fat32str[] = "FAT32   ";
//------------------------------------------------------------------------------
#define sdError(msg) sdError_F(F(msg))

void sdError_F(const __FlashStringHelper* str) {
  cout << F("error: ");
  cout << str << endl;
  if (card.errorCode()) {
	cout << F("SD error: ") << hex << int(card.errorCode());
	cout << ',' << int(card.errorData()) << dec << endl;
  }
  while (1);
}
//------------------------------------------------------------------------------
#if DEBUG_PRINT
void debugPrint() {
  cout << F("FreeRam: ") << FreeRam() << endl;
  cout << F("partStart: ") << relSector << endl;
  cout << F("partSize: ") << partSize << endl;
  cout << F("reserved: ") << reservedSectors << endl;
  cout << F("fatStart: ") << fatStart << endl;
  cout << F("fatSize: ") << fatSize << endl;
  cout << F("dataStart: ") << dataStart << endl;
  cout << F("clusterCount: ");
  cout << ((relSector + partSize - dataStart)/sectorsPerCluster) << endl;
  cout << endl;
  cout << F("Heads: ") << int(numberOfHeads) << endl;
  cout << F("Sectors: ") << int(sectorsPerTrack) << endl;
  cout << F("Cylinders: ");
  cout << cardSizeBlocks/(numberOfHeads*sectorsPerTrack) << endl;
}
#endif  // DEBUG_PRINT
//------------------------------------------------------------------------------
// write cached block to the card
uint8_t writeCache(uint32_t lbn) {
  return card.writeBlock(lbn, cache.data);
}
//------------------------------------------------------------------------------
// initialize appropriate sizes for SD capacity
void initSizes() {
  if (cardCapacityMB <= 6) {
	sdError("Card is too small.");
  } else if (cardCapacityMB <= 16) {
	sectorsPerCluster = 2;
  } else if (cardCapacityMB <= 32) {
	sectorsPerCluster = 4;
  } else if (cardCapacityMB <= 64) {
	sectorsPerCluster = 8;
  } else if (cardCapacityMB <= 128) {
	sectorsPerCluster = 16;
  } else if (cardCapacityMB <= 1024) {
	sectorsPerCluster = 32;
  } else if (cardCapacityMB <= 32768) {
	sectorsPerCluster = 64;
  } else {
	// SDXC cards
	sectorsPerCluster = 128;
  }

  cout << F("Blocks/Cluster: ") << int(sectorsPerCluster) << endl;
  // set fake disk geometry
  sectorsPerTrack = cardCapacityMB <= 256 ? 32 : 63;

  if (cardCapacityMB <= 16) {
	numberOfHeads = 2;
  } else if (cardCapacityMB <= 32) {
	numberOfHeads = 4;
  } else if (cardCapacityMB <= 128) {
	numberOfHeads = 8;
  } else if (cardCapacityMB <= 504) {
	numberOfHeads = 16;
  } else if (cardCapacityMB <= 1008) {
	numberOfHeads = 32;
  } else if (cardCapacityMB <= 2016) {
	numberOfHeads = 64;
  } else if (cardCapacityMB <= 4032) {
	numberOfHeads = 128;
  } else {
	numberOfHeads = 255;
  }
}
//------------------------------------------------------------------------------
// zero cache and optionally set the sector signature
void clearCache(uint8_t addSig) {
  memset(&cache, 0, sizeof(cache));
  if (addSig) {
	cache.mbr.mbrSig0 = BOOTSIG0;
	cache.mbr.mbrSig1 = BOOTSIG1;
  }
}
//------------------------------------------------------------------------------
// zero FAT and root dir area on SD
void clearFatDir(uint32_t bgn, uint32_t count) {
  clearCache(false);
  if (!card.writeStart(bgn, count)) {
	sdError("Clear FAT/DIR writeStart failed");
  }
  for (uint32_t i = 0; i < count; i++) {
	if ((i & 0XFF) == 0) {
	  cout << '.';
	}
	if (!card.writeData(cache.data)) {
	  sdError("Clear FAT/DIR writeData failed");
	}
  }
  if (!card.writeStop()) {
	sdError("Clear FAT/DIR writeStop failed");
  }
  cout << endl;
}
//------------------------------------------------------------------------------
// return cylinder number for a logical block number
uint16_t lbnToCylinder(uint32_t lbn) {
  return lbn / (numberOfHeads * sectorsPerTrack);
}
//------------------------------------------------------------------------------
// return head number for a logical block number
uint8_t lbnToHead(uint32_t lbn) {
  return (lbn % (numberOfHeads * sectorsPerTrack)) / sectorsPerTrack;
}
//------------------------------------------------------------------------------
// return sector number for a logical block number
uint8_t lbnToSector(uint32_t lbn) {
  return (lbn % sectorsPerTrack) + 1;
}
//------------------------------------------------------------------------------
// format and write the Master Boot Record
void writeMbr() {
  clearCache(true);
  part_t* p = cache.mbr.part;
  p->boot = 0;
  uint16_t c = lbnToCylinder(relSector);
  if (c > 1023) {
	sdError("MBR CHS");
  }
  p->beginCylinderHigh = c >> 8;
  p->beginCylinderLow = c & 0XFF;
  p->beginHead = lbnToHead(relSector);
  p->beginSector = lbnToSector(relSector);
  p->type = partType;
  uint32_t endLbn = relSector + partSize - 1;
  c = lbnToCylinder(endLbn);
  if (c <= 1023) {
	p->endCylinderHigh = c >> 8;
	p->endCylinderLow = c & 0XFF;
	p->endHead = lbnToHead(endLbn);
	p->endSector = lbnToSector(endLbn);
  } else {
	// Too big flag, c = 1023, h = 254, s = 63
	p->endCylinderHigh = 3;
	p->endCylinderLow = 255;
	p->endHead = 254;
	p->endSector = 63;
  }
  p->firstSector = relSector;
  p->totalSectors = partSize;
  if (!writeCache(0)) {
	sdError("write MBR");
  }
}
//------------------------------------------------------------------------------
// generate serial number from card size and micros since boot
uint32_t volSerialNumber() {
  return (cardSizeBlocks << 8) + micros();
}
//------------------------------------------------------------------------------
// format the SD as FAT16
void makeFat16() {
  uint32_t nc;
  for (dataStart = 2 * BU16;; dataStart += BU16) {
	nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
	fatSize = (nc + 2 + 255)/256;
	uint32_t r = BU16 + 1 + 2 * fatSize + 32;
	if (dataStart < r) {
	  continue;
	}
	relSector = dataStart - r + BU16;
	break;
  }
  // check valid cluster count for FAT16 volume
  if (nc < 4085 || nc >= 65525) {
	sdError("Bad cluster count");
  }
  reservedSectors = 1;
  fatStart = relSector + reservedSectors;
  partSize = nc * sectorsPerCluster + 2 * fatSize + reservedSectors + 32;
  if (partSize < 32680) {
	partType = 0X01;
  } else if (partSize < 65536) {
	partType = 0X04;
  } else {
	partType = 0X06;
  }
  // write MBR
  writeMbr();
  clearCache(true);
  fat_boot_t* pb = &cache.fbs;
  pb->jump[0] = 0XEB;
  pb->jump[1] = 0X00;
  pb->jump[2] = 0X90;
  for (uint8_t i = 0; i < sizeof(pb->oemId); i++) {
	pb->oemId[i] = ' ';
  }
  pb->bytesPerSector = 512;
  pb->sectorsPerCluster = sectorsPerCluster;
  pb->reservedSectorCount = reservedSectors;
  pb->fatCount = 2;
  pb->rootDirEntryCount = 512;
  pb->mediaType = 0XF8;
  pb->sectorsPerFat16 = fatSize;
  pb->sectorsPerTrack = sectorsPerTrack;
  pb->headCount = numberOfHeads;
  pb->hidddenSectors = relSector;
  pb->totalSectors32 = partSize;
  pb->driveNumber = 0X80;
  pb->bootSignature = EXTENDED_BOOT_SIG;
  pb->volumeSerialNumber = volSerialNumber();
  memcpy(pb->volumeLabel, noName, sizeof(pb->volumeLabel));
  memcpy(pb->fileSystemType, fat16str, sizeof(pb->fileSystemType));
  // write partition boot sector
  if (!writeCache(relSector)) {
	sdError("FAT16 write PBS failed");
  }
  // clear FAT and root directory
  clearFatDir(fatStart, dataStart - fatStart);
  clearCache(false);
  cache.fat16[0] = 0XFFF8;
  cache.fat16[1] = 0XFFFF;
  // write first block of FAT and backup for reserved clusters
  if (!writeCache(fatStart)
	  || !writeCache(fatStart + fatSize)) {
	sdError("FAT16 reserve failed");
  }
}
//------------------------------------------------------------------------------
// format the SD as FAT32
void makeFat32() {
  uint32_t nc;
  relSector = BU32;
  for (dataStart = 2 * BU32;; dataStart += BU32) {
	nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
	fatSize = (nc + 2 + 127)/128;
	uint32_t r = relSector + 9 + 2 * fatSize;
	if (dataStart >= r) {
	  break;
	}
  }
  // error if too few clusters in FAT32 volume
  if (nc < 65525) {
	sdError("Bad cluster count");
  }
  reservedSectors = dataStart - relSector - 2 * fatSize;
  fatStart = relSector + reservedSectors;
  partSize = nc * sectorsPerCluster + dataStart - relSector;
  // type depends on address of end sector
  // max CHS has lbn = 16450560 = 1024*255*63
  if ((relSector + partSize) <= 16450560) {
	// FAT32
	partType = 0X0B;
  } else {
	// FAT32 with INT 13
	partType = 0X0C;
  }
  writeMbr();
  clearCache(true);

  fat32_boot_t* pb = &cache.fbs32;
  pb->jump[0] = 0XEB;
  pb->jump[1] = 0X00;
  pb->jump[2] = 0X90;
  for (uint8_t i = 0; i < sizeof(pb->oemId); i++) {
	pb->oemId[i] = ' ';
  }
  pb->bytesPerSector = 512;
  pb->sectorsPerCluster = sectorsPerCluster;
  pb->reservedSectorCount = reservedSectors;
  pb->fatCount = 2;
  pb->mediaType = 0XF8;
  pb->sectorsPerTrack = sectorsPerTrack;
  pb->headCount = numberOfHeads;
  pb->hidddenSectors = relSector;
  pb->totalSectors32 = partSize;
  pb->sectorsPerFat32 = fatSize;
  pb->fat32RootCluster = 2;
  pb->fat32FSInfo = 1;
  pb->fat32BackBootBlock = 6;
  pb->driveNumber = 0X80;
  pb->bootSignature = EXTENDED_BOOT_SIG;
  pb->volumeSerialNumber = volSerialNumber();
  memcpy(pb->volumeLabel, noName, sizeof(pb->volumeLabel));
  memcpy(pb->fileSystemType, fat32str, sizeof(pb->fileSystemType));
  // write partition boot sector and backup
  if (!writeCache(relSector)
	  || !writeCache(relSector + 6)) {
	sdError("FAT32 write PBS failed");
  }
  clearCache(true);
  // write extra boot area and backup
  if (!writeCache(relSector + 2)
	  || !writeCache(relSector + 8)) {
	sdError("FAT32 PBS ext failed");
  }
  fat32_fsinfo_t* pf = &cache.fsinfo;
  pf->leadSignature = FSINFO_LEAD_SIG;
  pf->structSignature = FSINFO_STRUCT_SIG;
  pf->freeCount = 0XFFFFFFFF;
  pf->nextFree = 0XFFFFFFFF;
  // write FSINFO sector and backup
  if (!writeCache(relSector + 1)
	  || !writeCache(relSector + 7)) {
	sdError("FAT32 FSINFO failed");
  }
  clearFatDir(fatStart, 2 * fatSize + sectorsPerCluster);
  clearCache(false);
  cache.fat32[0] = 0x0FFFFFF8;
  cache.fat32[1] = 0x0FFFFFFF;
  cache.fat32[2] = 0x0FFFFFFF;
  // write first block of FAT and backup for reserved clusters
  if (!writeCache(fatStart)
	  || !writeCache(fatStart + fatSize)) {
	sdError("FAT32 reserve failed");
  }
}
//------------------------------------------------------------------------------
// flash erase all data
//uint32_t const ERASE_SIZE = 262144L;
//void eraseCard() {
//  cout << endl << F("Erasing\n");
//  uint32_t firstBlock = 0;
//  uint32_t lastBlock;
//  uint16_t n = 0;
//
//  do {
//    lastBlock = firstBlock + ERASE_SIZE - 1;
//    if (lastBlock >= cardSizeBlocks) {
//      lastBlock = cardSizeBlocks - 1;
//    }
//    if (!card.erase(firstBlock, lastBlock)) {
//      sdError("erase failed");
//    }
//    cout << '.';
//    if ((n++)%32 == 31) {
//      cout << endl;
//    }
//    firstBlock += ERASE_SIZE;
//  } while (firstBlock < cardSizeBlocks);
//  cout << endl;
//
//  if (!card.readBlock(0, cache.data)) {
//    sdError("readBlock");
//  }
//  cout << hex << showbase << setfill('0') << internal;
//  cout << F("All data set to ") << setw(4) << int(cache.data[0]) << endl;
//  cout << dec << noshowbase << setfill(' ') << right;
//  cout << F("Erase done\n");
//}
//------------------------------------------------------------------------------
void formatCard() {
  cout << endl;
  cout << F("Formatting\n");
  initSizes();
  if (card.type() != SD_CARD_TYPE_SDHC) {
	cout << F("FAT16\n");
	makeFat16();
  } else {
	cout << F("FAT32\n");
	makeFat32();
  }
#if DEBUG_PRINT
  debugPrint();
#endif  // DEBUG_PRINT
  cout << F("Format done\n");
}

void list_file()
{
  while (file.openNext(sd.vwd(), O_READ))
  {
	file.printName(&Serial);
	Serial.write(' ');
	file.printModifyDateTime(&Serial);
	Serial.write(' ');
	file.printFileSize(&Serial);
	if (file.isDir()) {
	  // Indicate a directory.
	  Serial.write('/');
	}
	Serial.println();
	file.close();
  }
  Serial.println("Done!");

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
//void serial_print_date()                           // Печать даты и времени    
//{
//	  DateTime now = RTC.now();
//	  Serial.print(now.day(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.month(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.year(), DEC);//Serial display time
//	  Serial.print(' ');
//	  Serial.print(now.hour(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.minute(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.second(), DEC);
//}
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

  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
//	  sdError("Can't create file name");
	}
  }
  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {

  }
  else
  {
	Serial.println(fileName);
	Serial.print(F(" Open Ok!: "));
  }


	DateTime now = RTC.now();

			regBank.set(adr_Mic_Start_day , now.day());           // Время старта теста
			regBank.set(adr_Mic_Start_month, now.month());
			regBank.set(adr_Mic_Start_year, now.year());
			regBank.set(adr_Mic_Start_hour, now.hour());
			regBank.set(adr_Mic_Start_minute, now.minute());
			regBank.set(adr_Mic_Start_second, now.second());
	// Уточнить 			
			regBank.set(adr_Time_Test_day, 0); 
			regBank.set(adr_Time_Test_hour, 0); 
			regBank.set(adr_Time_Test_minute, 0); 
			regBank.set(adr_Time_Test_second, 0); 
	
	/*  myFile = sd.open(fileName, FILE_WRITE);            // открыть файл для записи данных
	  myFile.println ("");
	  myFile.print ("Start test   ");
	  file_print_date();
	  myFile.println ("");
	  delay(100);
	  regBank.set(adr_control_command,0);

	
	data_clock_exchange();                                                          // Проверить не изменилась ли дата

	file_name_count = i2c_eeprom_read_byte(0x50, adr_file_name_count);              // считать текущий номер файла из памяти
	regBank.set(adr_reg_file_name,file_name_count);                                 // Регистр  хранения переменной номер файла
	preob_num_str();                                                                // сформировать имя файла из даты и счетчика файлов
 //   regBank.set(adr_reg_file_tek,file_name_count);                                  // Регистр  хранения переменной номер файла
 */
 //if (sd.exists(file_name))                                                      // проверить есть ли такой файл
	//  { 
	//	Serial.print(file_name);
	//	Serial.println("  Open  OK!.");
	//  }
	//  else 
	//  {
	//	  Serial.println("");
	//	  Serial.print(file_name);
	//	  Serial.println(" doesn't exist.");             // такого файла нет
	//  }

//	  myFile = sd.open(file_name, FILE_WRITE);            // открыть файл для записи данных
	  myFile.println ("");
	  myFile.print ("Start test   ");
	  file_print_date();
	  myFile.println ("");
	  delay(100);
	  regBank.set(adr_control_command,0);
	  
}
void FileClose()
{
	
	 myFile.println ("");
	 myFile.print ("Stop test   ");
	 file_print_date();
	 myFile.println ("");
	 myFile.close();
	// 
	// file_name_count++;

	//if(file_name_count > 99)
	//	{
	//		file_name_count = 0;
	//	}
	//i2c_eeprom_write_byte(0x50, adr_file_name_count, file_name_count);//

	   Serial.print(fileName);
	   Serial.println(" close.");
	  delay(100);

	 // if (SD.exists(file_name))
	 // { 
		//Serial.print(file_name);
		//Serial.println("  Close  OK!.");
	 // }
	 // else 
	 // {
		  //Serial.print(fileName);
		  //Serial.println(" doesn't exist.");  
	  //}
	regBank.set(adr_control_command,0);
	
}

void file_name()
{

   preob_num_str();

  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
//	  sdError("Can't create file name");
	}
  }
  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  //do {
  //  delay(10);
  // } while (Serial.read() >= 0);
  //
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  myFile.close();
  Serial.println("done.");
  
} 
void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
{
	DateTime now = RTC.now();

	day   = now.day();
	month = now.month();
	year  = now.year();


	int year_temp = year-2000;

	itoa (year_temp,str_year_file, 10);                                        // Преобразование даты год в строку ( 10 - десятичный формат) 

	
	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
		   itoa (month,str_mon_file10, 10);                                    //  Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // Сложение 2 строк
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		}


	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // Преобразование числа в строку ( 10 - десятичный формат) 
		   itoa (day,str_day_file10, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // Сложение 2 строк
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // Преобразование числа в строку ( 10 - десятичный формат) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // Сложение 2 строк
	sprintf(str2, "%s%s",str1, str_day_file);                                  // Сложение 2 строк
	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // Получение имени файла в file_name
	//Serial.println(fileName);
}

void set_namber_file_zero()
{
   i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                              // счетчик номера файла сбросить в "0"
   data_clock_exchange();
   regBank.set(adr_control_command,0);
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
	15 - Установить уровень сигнала

	*/
	UpdateRegs() ;

	int test_n = regBank.get(adr_control_command);                                  //адрес  40120
		
	switch (test_n)
	{
		case 1:
			 sensor_all_off();                                                      // Отключить все сенсоры
			break;
		case 2:		
			 sensor_all_on();                                                       // Включить все сенсоры
				break;
		case 3:
			 test_headset_instructor();
				break;
		case 4:	
			 test_headset_dispatcher();                                             //
				break;
		case 5:
			 test_MTT();                                                            //
				break;
		case 6:	
			 test_tangR();                                                          //
				break;
		case 7:
			test_tangN();
				break;
		case 8:				
			 testGGS();
				break;
		case 9:
			 test_GG_Radio1();
				break;
		case 10:	
			 test_GG_Radio2();
				break;
		case 11:				
			 test_mikrophon();                                                      // Тестирование микрофона
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
		case 16:
				Reg_count_clear();			                                        // Сброс счетчиков ошибок                    
				break;
		case 17:
			 set_namber_file_zero();              //
				break;
		case 18:
			// format_SD();                       //
				break;
		default:
		//	regBank.set(adr_control_command,0);
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
	UpdateRegs(); 
	delay(1000);
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
	delay(1000);
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ Начало проверки ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	UpdateRegs(); 
	delay(1000);

	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];  

   if (i50 == 0xA3)                                                       
	   {
		if(i52 == 0x43 )
			{
			if(i53 == 0x02 )
				{
				if (test_repeat == false)
					{
						strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
						myFile.print(buffer);                                               // 
						strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
					}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)

	   {
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x45 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x49 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок                                            // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	//Serial.println(i50 ,HEX);
	//Serial.println(i52 ,HEX);
	//Serial.println(i53 ,HEX);

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x51 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибокчетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки        
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA7)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки        
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xAB)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(13,1);                                                              // XP8 - 2   sensor Тангента ножная
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xB3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x61 )
			   {
				if(i53 == 0x02 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }



	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона отключить
	regBank.set(5,1);                                                               // Микрофон инструктора включить "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x11 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,true);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,1);                                                              // Микрофон диспетчера включить
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x44 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
						}
				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			regBank.set(40219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			regBank.set(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

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
	delay(1000);
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}

void set_rezistor()
{
	int mwt = regBank.get(40010);             // Адрес хранения величины сигнала
	resistor(1, mwt);
	resistor(2, mwt);
	regBank.set(adr_control_command,0);
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
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                                // Установить уровень сигнала 30 мв
	regBank.set(2,1);                                                               // Подать сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                           // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,35);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,35);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,35);                                 // Измерить уровень сигнала на выходе LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,35);                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,35);                                 // Измерить уровень сигнала на выходе mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,35);                                 // Измерить уровень сигнала на выходе mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,35);                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,35);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,35);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
	//myFile.println("");                                                             //
	regBank.set(5,1);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT Включить PTT инструктора
	regBank.set(16,0);                                                              // Сенсор микрофона отключить
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(29,1);                                                              // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
	UpdateRegs();                                                                   // 
	delay(400);                                                                     //
	byte i53 = regs_in[3];                                                          // Получить текущее состояние Камертона
		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = regBank.get(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // Микрофон инструктора включен  - Pass
			  }
		  }
//	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    Звуковой сигнал подан на вход микрофона инструктора
	delay(20);
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,150);                                // Измерить уровень сигнала на выходе LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,150);                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,35);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,35);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,35);                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,35);                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,35);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,35);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(5,0);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Выключить микрофон инструктора)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	regBank.set(2,0);                                                               // Выключить сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();     

	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void test_headset_dispatcher()
 {
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23])));                   // " ****** Test headset dispatcher start! ******"               ;
	myFile.println(buffer);                                                         // " ****** Test headset dispatcher start! ******"               ;
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_disp_off();                                                                // Отключить реле и сенсоры, прверить отключение
	test_disp_on();                                                                 // Включить необходимые сенсоры, проверить состояние
	//myFile.println("");

		// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // Установить уровень сигнала 30 мв
	resistor(2, 30);                                                                // Установить уровень сигнала 30 мв
	regBank.set(1,1);                                                               // Подать сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,35);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,35);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,35);                                 // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,35);                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,35);                                 // Измерить уровень сигнала на выходе mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,35);                                 // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,35);                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,35);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,35);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
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
			regcount = regBank.get(40182);                                          // адрес счетчика ошибки включения микрофона диспетчера          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок включения микрофона диспетчера       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regBank.set(40182,regcount);                                            // адрес счетчика ошибки включения микрофона диспетчера
			regBank.set(182,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			resistor(1, 255);                                                       // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                                       // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - ";  - Pass
			   }
		  }
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14])));                   // "Microphone headset dispatcher signal          ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(20);
	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,200);                                // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,200);                                // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,35);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,35);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,35);                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,35);                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,35);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,35);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";

	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   Отключить sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   Отключить  sensor подкючения гарнитуры диспетчера
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(10,0);                                                              // Подать управляющую команду на вывод XP1 10  (Выключить микрофон диспетчера)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.set(1,0);                                                               // Отключить сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
 }
void test_MTT()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24])));                   // " ****** Test MTT start! ******"                              ; 
	myFile.println(buffer);                                                         // " ****** Test MTT start! ******"                              ; 
	file_print_date();
	myFile.println("");
	test_MTT_off();                                                                 // Отключить реле и сенсоры, прверить отключение
	test_MTT_on();                                                                  // Включить необходимые сенсоры, проверить состояние
//	myFile.println("");
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT включить должно быть в "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD Трубку поднять DCD должно быть в "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"

	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,35);                                // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,35);                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40252,252,35);                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40253,253,35);                                // Измерить уровень сигнала на выходе LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,35);                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40255,255,35);                                // Измерить уровень сигнала на выходе LineR     "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,35);                                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,35);                                // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,35);                                // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                               // Установить уровень сигнала 60 мв
	resistor(2, 130);                                                               // Установить уровень сигнала 60 мв
	regBank.set(3,1);                                                               // Включить сигнал на вход микрофона трубки Mic3p
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                   // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;

	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,35);                                // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,35);                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	//measure_vol_min(analog_LineL,     40252,252,25);                              // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,35);                                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,35);                                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,35);                                // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,35);                                // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ Проверить наличие сигнала  ++++++++++++++++++++++++++++++++++++
//	measure_vol_max(analog_LineL,    40260,260,35);                                 // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,    40261,261,35);                                 // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone,40262,262,90);                                 // Измерить уровень сигнала на выходе mag phone  "Test MTT ** Signal Mag phone                                ON  - ";

	// +++++++++++++++++++++ Проверка реагирования вывода ГГС на сигнал HangUp  DCD ON +++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                               // Отключить сигнал на вход микрофона трубки Mic3p
	regBank.set(6,1);                                                               // Реле RL5. Подать звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                   //   
	if (test_repeat == false) myFile.println(buffer);                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       40256,256,30);                                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                   // Выполнить команду
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,      40259,259,120);                                //  Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS             On      
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD ON  Положить трубку
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл. Отключить PTT MTT
	regBank.set(25,1);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT отключить должно быть в "1"
	regBank.set(6,0);                                                               // Реле RL5. Отключить звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(200);
}
void test_tangR()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[36])));                   // " ****** Test tangenta ruchnaja start! ******"                ;
	myFile.println(buffer);                              //
	file_print_date();
	myFile.println("");
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = regs_in[0];    


	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor тангента ручная               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = regBank.get(40274);                                          // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(40274,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(274,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            OFF - ";  - Pass
		  }
		}

	 UpdateRegs(); 
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40270);                                          // адрес счетчика ошибки                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40270,regcount);                                            // адрес счетчика ошибки 
			regBank.set(270,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";  - Pass
		  }
		 }

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = regBank.get(40271);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40271,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regBank.set(271,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";  - Pass
		   }
		  }

	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(17,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	regBank.set(20,1);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

			if(bitRead(regs_in[0],3) == 0)                                          // J8-11     XP7 2 sensor тангента ручная             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = regBank.get(40275);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta ruchnaja                            ON  - ";
			regBank.set(40275,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			regBank.set(275,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            ON  - "; - Pass
		  }
		  }
	 UpdateRegs(); 
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40272);                                          // адрес счетчика ошибки PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40272,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS)
			regBank.set(272,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - "; - Pass
		   }
		  }

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = regBank.get(40273);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40273,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			regBank.set(273,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";  - Pass
		   }
		  }
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void test_tangN()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));                   // " ****** Test tangenta nognaja start! ******"                 ;
	myFile.println(buffer);                                                         // "Command sensor OFF tangenta nognaja              send!"      ;
	file_print_date();
	myFile.println("");
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(14,0);                                                              // XP8 - 1   PTT Тангента ножная
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //

	byte i50 = regs_in[0];    
	
	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor тангента                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = regBank.get(40276);                                          // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(40276,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(276,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             OFF - "; - Pass
		  }
		}

	 UpdateRegs(); 
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента ножная CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP8 1 PTT1 тангента   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = regBank.get(40278);                                          // адрес счетчика ошибки 
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40278,regcount);                                            // адрес счетчика ошибки                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			regBank.set(278,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  }
		 }


	regBank.set(13,1);                                                              // XP8 2 sensor тангента ножная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(14,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;


	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

			if(bitRead(regs_in[0],4) == 0)                                          // J8-11     XP7 2 sensor тангента                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = regBank.get(40277);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta nognaja                             ON  - ";
			regBank.set(40277,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			regBank.set(277,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             ON  - ";
		   }
		  }
	 UpdateRegs(); 
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента  CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = regBank.get(40279);                                          // адрес счетчика ошибки                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40279,regcount);                                            // адрес счетчика ошибки                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			regBank.set(279,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          ON  - ";
		   }
		  }

	regBank.set(14,0);                                                              //   XP8 1 PTT тангента  
	regBank.set(13,0);                                                              //   XP8 2 sensor тангента  
	UpdateRegs();                                                                   // Выполнить команду
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void test_mikrophon()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test miсrophone start! ******"                       ;
	myFile.println(buffer);                                                         // " ****** Test miсrophone start! ******"                       ;
	file_print_date();
	myFile.println("");
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона

	regBank.set(14,0);    // XP8 - 1   PTT Тангента ножная
	regBank.set(17,0);    // J8-12     XP7 4 PTT2   Танг. р.
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 Танг. р.
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR вкл.  
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера


	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   //"Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на отключение сенсора и  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	byte i52 = regs_in[2];    
			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor отключения микрофона
		  {
			regcount = regBank.get(40207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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

	 UpdateRegs(); 
	  // 2)  Проверка  на отключение PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = regBank.get(40264);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40264,regcount);                                            // адрес счетчика ошибки 
			regBank.set(264,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  OFF - ";  - Pass
		  }
		 }

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение сенсора  microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	i52 = regs_in[2];    

	  if(bitRead(i52,5) == 0)                                                 // XS1 - 6   sensor отключения микрофона
		  {
			regcount = regBank.get(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(15,1);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	i52 = regs_in[2];    

	if(bitRead(i52,5) == 0)                                             // XS1 - 6   sensor отключения микрофона
		  {
			regcount = regBank.get(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }

	 UpdateRegs(); 
	  // 2)  Проверка  на включение  PTT microphone
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = regBank.get(40266);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40266,regcount);                                            // адрес счетчика ошибки 
			regBank.set(266,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  ON  - ";
		  }
		 }

	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала  на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40320,320,30);                                // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,30);                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40322,322,30);                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40323,323,30);                                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,30);                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40325,325,30);                                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,30);                                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,30);                                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,30);                                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 60);                                                                // Установить уровень сигнала 60 мв
	resistor(2, 60);                                                                // Установить уровень сигнала 60 мв
	regBank.set(9,1);                                                               // Включить сигнал на вход микрофона Реле RL8 Звук на микрофон
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	UpdateRegs();                                                                   // Выполнить команду
	delay(1400);

	Serial.println("Signal Mag phone");
	measure_vol_max(analog_mag_phone,40298,298,180);                                // Измерить уровень сигнала на выходе mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,    40299,299,180);                                // Измерить уровень сигнала на выходе "Test Microphone ** Signal LineL                      ON  - ";  
	
	measure_vol_min(analog_FrontL,    40320,320,30);                                // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,30);                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     40323,323,30);                                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,30);                                // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,30);                                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,30);                                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,30);                                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     

	regBank.set(9,0);                                                               // Отключить сигнал на вход микрофона Реле RL8 Звук на микрофон
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // Завершить программу    
}
void testGGS()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[48])));                   // " ****** Test GGS start! ******"      ;
	myFile.println(buffer);                                                         // " ****** Test GGS start! ******"      ;
	file_print_date();
	myFile.println("");
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки  
	regBank.set(6,0);                                                               // Реле RL5 Звук Front L, Front R
	resistor(1, 220);                                                               // Установить уровень сигнала 60 мв
	resistor(2, 220);                                                               // Установить уровень сигнала 60 мв
	//UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
		//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40280,280,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40281,281,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40285,285,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40286,286,30);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	regBank.set(6,1);                                                               // Реле RL5 Звук Front L, Front R
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

	measure_vol_max(analog_FrontL,    40290,290,40);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,40);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,50);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       40289,289,160);                               // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      ON  - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки          
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	UpdateRegs(); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;

	measure_vol_max(analog_FrontL,    40290,290,40);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,40);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,50);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_min(analog_ggs,       40286,286,30);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(6,0);                                                               // Реле RL5 Звук Front L, Front R
	UpdateRegs();    
	delay(300);
	regBank.set(adr_control_command,0);                                             // Завершить программу    
}
void test_GG_Radio1()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[50])));                   // " ****** Test Radio1 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio1 start! ******"                           ;
	file_print_date();
	myFile.println("");
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1, 110);                                                               // Установить уровень сигнала 300 мв
	resistor(2, 110);                                                               // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	UpdateRegs(); 
	delay(300);
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40300,300,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,30);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40307,307,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40308,308,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(4,1);                                                               //  Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

	measure_vol_min(analog_FrontL,    40300,300,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,30);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, 40309,309,220);                               // Измерить уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, 40308,308,25);                                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";


	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // Завершить программу    
}
void test_GG_Radio2()
{

	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));                   // " ****** Test Radio2 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio2 start! ******"                           ;
	file_print_date();
	myFile.println("");
	regBank.set(7,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1, 110);                                                               // Установить уровень сигнала 300 мв
	resistor(2, 110);                                                               // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	UpdateRegs(); 
	delay(300);
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40310,310,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,30);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40318,318,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(7,1);                                                               //  Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);

	measure_vol_min(analog_FrontL,    40310,310,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,30);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,25);                                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, 40319,319,250);                               // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";


	regBank.set(7,0);                                                               // Реле RL6 Звук Center
	UpdateRegs();     
	regBank.set(adr_control_command,0);    
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
	byte i52 = regs_in[2];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры инструктора 2 наушниками
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
	byte i52 = regs_in[2];    
	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             // "Sensor headset instructor включен  - Pass
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  включен  - Pass
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
	byte i52 = regs_in[2];    
	 
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40205);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(40205,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(205,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " отключен  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  подключения гарнитуры диспетчера
		  {
			regcount = regBank.get(40206);                                          // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			regBank.set(40206,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regBank.set(206,1);                                                     // установить флаг ошибки sensor подключения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
	byte i52 = regs_in[2];    

	  // 3)  Проверка сенсора на подключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  включен  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера 
			regBank.set(216,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  включен  - Pass
		   }
		 }
}

void test_MTT_off()
{
	unsigned int regcount = 0;
	regBank.set(25,1);                                                              // "Command sensor OFF MTT                           send! "     ;     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                   // "Command sensor OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF MTT                           send! "     ;      
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS  вкл. Отключить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
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
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(300);
	UpdateRegs(); 
	delay(100);
	byte i50 = regs_in[0];    


		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             //  sensor  трубки отключен  - Pass
			   }
		  }
		   UpdateRegs(); 

	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение CTS MTT
		  {
			regcount = regBank.get(40263);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40263,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regBank.set(263,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));    // "Test MTT PTT    (CTS)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT PTT    (CTS)                                       OFF - ";
			   }                   
		  }


	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // Проверить включение HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = regBank.get(40267);                                          // адрес счетчика ошибки отключения HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40267,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // установить флаг ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));    // "Test MTT HangUp (DCD)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT HangUp (DCD)                                       OFF - ";
			   }             
		 }
}
void test_MTT_on()
{
	delay(600);
	unsigned int regcount = 0;
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	regBank.set(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR вкл. включить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(600);
	UpdateRegs(); 
	  // 1)  Проверка сенсора MTT на включение 
	byte i50 = regs_in[0];    
		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = regBank.get(40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
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
				myFile.println(buffer);                                             //  sensor  трубки включен  - Pass
			   }
		  }

		//delay(300);
		//UpdateRegs(); 
		delay(300);
		UpdateRegs(); 
	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение CTS MTT
		  {
			regcount = regBank.get(40265);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40265,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regBank.set(265,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
			Serial.print("adr_reg_ind_CTS - ");                                      //  
			Serial.println(regBank.get(adr_reg_ind_CTS));
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));    // "Test MTT PTT    (CTS)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " трубки включен  - Pass
			   }
		  }


	   if(regBank.get(adr_reg_ind_DCD)== 0)                                         // Проверить включение HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = regBank.get(40268);                                          // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(40268,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                     // установить флаг ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";трубки включен  - Pass
			   }
		 }
}

void measure_vol_min(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int _istochnik          = istochnik;
		unsigned int _adr_count = adr_count;
		int _adr_flagErr        = adr_flagErr;
		unsigned int _porogV    = porogV;
		int regcount = 0;
		measure_volume(_istochnik);                                                 // Измерить уровень сигнала на выходе
		switch (_adr_flagErr) 
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
			case 250:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[50])));    //  "Test MTT ** Signal FrontL                                   OFF - ";
				break;
			case 251:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[51])));    // "Test MTT ** Signal FrontR                                   OFF - ";
				break;
			case 252:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[52])));    // "Test MTT ** Signal LineL                                    OFF - ";
				break;
			case 253:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[53])));    // "Test MTT ** Signal LineR                                    OFF - ";
				break;
			case 254:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[54])));    // "Test MTT ** Signal mag radio                                OFF - ";
				break;
			case 255:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[55])));    // "Test MTT ** Signal mag phone                                OFF - ";
				break;
			case 256:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[56])));    // "Test MTT ** Signal GGS                                      OFF - ";
				break;
			case 257:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[57])));    // "Test MTT ** Signal GG Radio1                                OFF - ";
				break;
			case 258:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[58])));    // "Test MTT ** Signal GG Radio2                                OFF - ";
				break;
			case 280:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ошибки "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ошибки "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ошибки "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ошибки "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ошибки "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ошибки "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ошибки "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
				break;
			case 288:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[88])));    // "Test GGS ** Signal GG Radio2                                OFF - ";
				break;
			case 300:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[100])));    //  "Test Radio1 ** Signal FrontL                                OFF - ";
				break;
			case 301:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[101])));    // "Test Radio1 ** Signal FrontR                                OFF - ";
				break;
			case 302:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[102])));    // "Test Radio1 ** Signal LineL                                 OFF - ";
				break;
			case 303:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[103])));    // "Test Radio1 ** Signal LineR                                 OFF - ";
				break;
			case 304:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[104])));    // "Test Radio1 ** Signal mag radio                             OFF - ";
				break;
			case 305:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[105])));    // "Test Radio1 ** Signal mag phone                             OFF - ";
				break;
			case 306:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[106])));    // "Test Radio1 ** Signal GGS                                   OFF - ";
				break;
			case 307:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[107])));    // "Test Radio1 ** Signal GG Radio1                             OFF - ";
				break;
			case 308:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[108])));    // "Test Radio1 ** Signal GG Radio2                             OFF - ";
				break;
			case 310:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[110])));    //  "Test Radio2 ** Signal FrontL                                OFF - ";
				break;
			case 311:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[111])));    // "Test Radio2 ** Signal FrontR                                OFF - ";
				break;
			case 312:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[112])));    // "Test Radio2 ** Signal LineL                                 OFF - ";
				break;
			case 313:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[113])));    // "Test Radio2 ** Signal LineR                                 OFF - ";
				break;
			case 314:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[114])));    // "Test Radio2 ** Signal mag radio                             OFF - ";
				break;
			case 315:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[115])));    // "Test Radio2 ** Signal mag phone                             OFF - ";
				break;
			case 316:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[116])));    // "Test Radio2 ** Signal GGS                                   OFF - ";
				break;
			case 317:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[117])));    // "Test Radio2 ** Signal GG Radio1                             OFF - ";
				break;
			case 318:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[118])));    // "Test Radio2 ** Signal GG Radio2                             OFF - ";
				break;
			case 320:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[120])));    //  "Test Microphone ** Signal FrontL                                   OFF - ";
				break;
			case 321:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[121])));    // "Test Microphone ** Signal FrontR                                   OFF - ";
				break;
			case 322:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[122])));    // "Test Microphone ** Signal LineL                                    OFF - ";
				break;
			case 323:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[123])));    // "Test Microphone ** Signal LineR                                    OFF - ";
				break;
			case 324:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[124])));    // "Test Microphone ** Signal mag radio                                OFF - ";
				break;
			case 325:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[125])));    // "Test Microphone ** Signal mag phone                                OFF - ";
				break;
			case 326:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[126])));    // "Test Microphone ** Signal GGS                                      OFF - ";
				break;
			case 327:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[127])));    // "Test Microphone ** Signal GG Radio1                                OFF - ";
				break;
			case 328:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[128])));    // "Test Microphone ** Signal GG Radio2                                OFF - ";
				break;

		}
		if(voltage10 > _porogV)                                                      // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = regBank.get(_adr_count);                                  // адрес счетчика ошибки 
				regcount++;                                                          // увеличить счетчик ошибок канала 
				regBank.set(_adr_count,regcount);                                    // адрес счетчика ошибки канала 
				regBank.set(_adr_count+200,voltage10);                               // адрес данных ошибки канала 
				regBank.set(_adr_flagErr,1);                                         // установить флаг ошибки  канала 
				regcount_err = regBank.get(adr_reg_count_err);                       // Получить данные счетчика всех ошибок
				regcount_err++;                                                      // увеличить счетчик всех ошибок 
				regBank.set(adr_reg_count_err,regcount_err);                         // Сохранить данные счетчика всех ошибок
				regBank.set(120,1);                                                  // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // Показания счетчика ошибок
				myFile.print("     ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print("     ");  
					myFile.print(voltage); 
					myFile.println(" V");
				}
			}  
	delay(100);
}
void measure_vol_max(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
	int _istochnik          = istochnik;
	unsigned int _adr_count = adr_count;
	int _adr_flagErr        = adr_flagErr;
	unsigned int _porogV    = porogV;
	int regcount            = 0;

	measure_volume(_istochnik);                                                 // Измерить уровень сигнала на выходе
	switch (_adr_flagErr) 
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
			case 259:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[59])));    // "Test MTT ** Signal GGS                                      ON  - ";
				break;
			case 260:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[60])));    // "Test MTT ** Signal LineL                                    ON  - ";
				break;
			case 261:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[61])));    // "Test MTT ** Signal LineR                                    ON  - ";  
				break;
			case 262:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[62])));    // "Test MTT ** Signal Mag phone                                ON  - ";
				break;
			case 289:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ошибки "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ошибки "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ошибки "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ошибки "Test GGS ** Signal mag phone                                ON  - ";
				break;
			case 298:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[98])));    // "Test Microphone ** Signal mag phone                         ON  - ";      
				break;
			case 299:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[99])));    // "Test Microphone ** Signal LineL                             ON  - ";  
				break;
			case 309:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[109])));   // "Test Radio1 ** Signal Radio1                                ON  - ";
				break;
			case 319:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[119])));   // "Test Radio1 ** Signal Radio2                                ON  - ";
				break;
		}
	
		if(voltage10 < _porogV)                                                     // Проверить исправность канала
			{
				myFile.print(buffer); 
				regcount = regBank.get(_adr_count);                                 // адрес счетчика ошибки 
				regcount++;                                                         // увеличить счетчик ошибок канала 
				regBank.set(_adr_count, regcount);                                  // адрес счетчика ошибки канала 
				regBank.set(_adr_count+200,voltage10);                              // адрес данных ошибки канала 
				regBank.set(_adr_flagErr,1);                                        // установить флаг ошибки  канала 
				regcount_err = regBank.get(adr_reg_count_err);                       // Получить данные счетчика всех ошибок
				regcount_err++;                                                      // увеличить счетчик всех ошибок 
				regBank.set(adr_reg_count_err,regcount_err);                         // Сохранить данные счетчика всех ошибок
				regBank.set(120,1);                                                 // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // Показания счетчика ошибок
				myFile.print("     ");  
				myFile.print(voltage); 
				myFile.println(" V");
				//Serial.println(regBank.get(_adr_count));
				//Serial.println(regBank.get(_adr_count+200));
				//Serial.println(regBank.get(_adr_flagErr));
  /*              do
				{
					Serial.println(regBank.get(_adr_flagErr));

				} while (regBank.get(_adr_flagErr) != 0);
*/
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // Наименование проверки
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print("     ");  
					myFile.print(voltage); 
					myFile.println(" V");
				}
			} 
		delay(100);
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
//void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
//{
//	DateTime now = RTC.now();
//	int year_temp = now.year()-2000;
//
//	itoa (year_temp,str_year_file, 10); // Преобразование даты год в строку ( 10 - десятичный формат) 
//
//	
//	if (now.month() <10)
//		{
//		   itoa (0,str_mon_file0, 10);                   //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
//		   itoa (now.month(),str_mon_file10, 10);        //  Преобразование числа в строку ( 10 - десятичный формат) 
//		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);  // Сложение 2 строк
//		}
//	else
//		{
//		   itoa (now.month(),str_mon_file, 10);         // Преобразование числа в строку ( 10 - десятичный формат) 
//		}
//
//
//	if (now.day() <10)
//		{
//		   itoa (0,str_day_file0, 10);                  // Преобразование числа в строку ( 10 - десятичный формат) 
//		   itoa (now.day(),str_day_file10, 10);         // Преобразование числа в строку ( 10 - десятичный формат) 
//		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);  // Сложение 2 строк
//		}
//	else
//		{
//		itoa (now.day(),str_day_file, 10);                   // Преобразование числа в строку ( 10 - десятичный формат) 
//		}
//
//		 
//	if (file_name_count<10)
//		{
//			itoa (file_name_count,str0, 10);                 // Преобразование числа в строку ( 10 - десятичный формат) 
//			sprintf(str_file_name_count, "%s%s", str_file_name_count0, str0);  // Сложение 2 строк
//		}
//	
//	else
//		{
//			itoa (file_name_count,str_file_name_count, 10);  // Преобразование числа в строку ( 10 - десятичный формат) 
//		}
//	sprintf(str1, "%s%s",str_year_file, str_mon_file);       // Сложение 2 строк
//	sprintf(str2, "%s%s",str1, str_day_file);                // Сложение 2 строк
//	sprintf(str3, "%s%s", str2, str_file_name_count);        // Сложение 2 строк
//	sprintf(file_name, "%s%s", str3, file_name_txt);         // Получение имени файла в file_name
//
//}

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

/*
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
  */

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
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера
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

	regBank.add(250);                         // Флаг ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(251);                         // Флаг ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(252);                         // Флаг ошибки "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(253);                         // Флаг ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(254);                         // Флаг ошибки "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(255);                         // Флаг ошибки "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(256);                         // Флаг ошибки "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(257);                         // Флаг ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(258);                         // Флаг ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(259);                         // Флаг ошибки "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(260);                         // Флаг ошибки "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(261);                         // Флаг ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(262);                         // Флаг ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(263);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(264);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(265);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(266);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(267);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(268);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(269);                         //  

	regBank.add(270);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(271);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(272);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(273);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(274);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(275);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(276);                         // Флаг ошибки "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(277);                         // Флаг ошибки "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(278);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(279);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(280);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(281);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(282);                         // Флаг ошибки "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(283);                         // Флаг ошибки "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(284);                         // Флаг ошибки "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(285);                         // Флаг ошибки "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(286);                         // Флаг ошибки "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(287);                         // Флаг ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(288);                         // Флаг ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(289);                         // Флаг ошибки "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(290);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(291);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(292);                         // Флаг ошибки "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(293);                         // 
	regBank.add(294);                         // 
	regBank.add(295);                         // 
	regBank.add(296);                         // 
	regBank.add(297);                         // 
	regBank.add(298);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         ON  - ";      
	regBank.add(299);                         // Флаг ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(300);                         // Флаг ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(301);                         // Флаг ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(302);                         // Флаг ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(303);                         // Флаг ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(304);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(305);                         // Флаг ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(306);                         // Флаг ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(307);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(308);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(309);                         // Флаг ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(310);                         // Флаг ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(311);                         // Флаг ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(312);                         // Флаг ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(313);                         // Флаг ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(314);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(315);                         // Флаг ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(316);                         // Флаг ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(317);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(318);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(319);                         // Флаг ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(320);                         // Флаг ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(321);                         // Флаг ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(322);                         // Флаг ошибки "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(323);                         // Флаг ошибки "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(324);                         // Флаг ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(325);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(326);                         // Флаг ошибки "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(327);                         // Флаг ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(328);                         // Флаг ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(329);                         // 

	regBank.add(330);                         // 




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
	regBank.add(40010);    // Адрес хранения величины сигнала резисторами
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

	regBank.add(40112);  // Адрес хранения переменной год 
	regBank.add(40113);  // Адрес хранения переменной месяц 
	regBank.add(40114);  // Адрес хранения переменной день
	regBank.add(40115);  // Адрес хранения переменной счетчика последнего номера файла
	regBank.add(40116);  // Адрес хранения переменной счетчика текущего номера файла

	regBank.add(40120);  // adr_control_command Адрес передачи комманд на выполнение
	regBank.add(40121);  // Адрес счетчика всех ошибок
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

	regBank.add(40250);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40251);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40252);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40253);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40254);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40255);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40256);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40257);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40258);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40259);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40260);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40261);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40262);                         // Aдрес счетчика ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40263);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40264);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(40265);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40266);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(40267);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40268);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40269);                         //  

	regBank.add(40270);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40271);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40272);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40273);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40274);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40275);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40276);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40277);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40278);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40279);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40280);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40281);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40282);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40283);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40284);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40285);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40286);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40287);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40288);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40289);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40290);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40291);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40292);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40293);                         // 
	regBank.add(40294);                         // 
	regBank.add(40295);                         // 
	regBank.add(40296);                         // 
	regBank.add(40297);                         // 
	regBank.add(40298);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         ON  - ";    
	regBank.add(40299);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40300);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40301);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40302);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40303);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40304);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40305);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40306);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40307);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40308);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40309);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40310);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40311);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40312);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40313);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40314);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40315);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40316);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40317);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40318);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40319);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40320);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40321);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40322);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40323);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40324);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40325);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40326);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40327);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40328);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40329);                         // 
	regBank.add(40330);                         // 

	
	// ++++++++++++++++++++++ Регистры хранения данных при проверке модулей ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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




	regBank.add(40420);                         // Aдрес  ;
	regBank.add(40421);                         // Aдрес  ;
	regBank.add(40422);                         // Aдрес  ;
	regBank.add(40423);                         // Aдрес  ;
	regBank.add(40424);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40425);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40426);                         // Aдрес данных измерения "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40427);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40428);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40429);                         // Aдрес данных измерения "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40430);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40431);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40432);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40433);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40434);                         // Aдрес данных измерения "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40435);                         // Aдрес данных измерения "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40436);                         // Aдрес данных измерения "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40437);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40438);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40439);                         //

	regBank.add(40440);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40441);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40442);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40443);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40444);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40445);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40446);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40447);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40448);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40449);                         //  

	regBank.add(40450);                         // Aдрес данных измерения "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40451);                         // Aдрес данных измерения "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40452);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40453);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40454);                         // Aдрес данных измерения "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40455);                         // Aдрес данных измерения "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40456);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40457);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40458);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40459);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40460);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40461);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40462);                         // Aдрес данных измерения "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40463);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40464);                         // 
	regBank.add(40465);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40466);                         // 
	regBank.add(40467);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40468);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40469);                         //  

	regBank.add(40470);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40471);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40472);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40473);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40474);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40475);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40476);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40477);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40478);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40479);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40480);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40481);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40482);                         // Aдрес данных измерения "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40483);                         // Aдрес данных измерения "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40484);                         // Aдрес данных измерения "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40485);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40486);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40487);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40488);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40489);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40490);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40491);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40492);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40493);                         // 
	regBank.add(40494);                         // 
	regBank.add(40495);                         // 
	regBank.add(40496);                         // 
	regBank.add(40497);                         // 
	regBank.add(40498);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         ON  - "; 
	regBank.add(40499);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40500);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40501);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40502);                         // Aдрес данных измерения "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40503);                         // Aдрес данных измерения "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40504);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40505);                         // Aдрес данных измерения "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40506);                         // Aдрес данных измерения "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40507);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40508);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40509);                         // Aдрес данных измерения "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40510);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40511);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40512);                         // Aдрес данных измерения "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40513);                         // Aдрес данных измерения "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40514);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40515);                         // Aдрес данных измерения "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40516);                         // Aдрес данных измерения "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40517);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40518);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40519);                         // Aдрес данных измерения "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40520);                         // Aдрес данных измерения "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40521);                         // Aдрес данных измерения "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40522);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40523);                         // Aдрес данных измерения "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40524);                         // Aдрес данных измерения "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40525);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40526);                         // Aдрес данных измерения "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40527);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40528);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40529);                         // 
	regBank.add(40530);                         // 


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

		Serial.print("Initializing SD card...");
	// On the Ethernet Shield, CS is pin 4. It's set as an output by default.
	// Note that even if it's not used as the CS pin, the hardware SS pin
	// (10 on most Arduino boards, 53 on the Mega) must be left as an output
	// or the SD library functions will not work.
	pinMode(53, OUTPUT);

	if (!sd.begin(chipSelect)) 
		{
		Serial.println("initialization failed!");
		//return;
		}
	Serial.println("initialization done.");
	if (!card.begin(chipSelect, spiSpeed)) {
	cout << F(
		   "\nSD initialization failure!\n"
		   "Is the SD card inserted correctly?\n"
		   "Is chip select correct at the top of this program?\n");
	sdError("card.begin failed");
  }

	 cardSizeBlocks = card.cardSize();
  if (cardSizeBlocks == 0) {
	sdError("cardSize");
  }
  cardCapacityMB = (cardSizeBlocks + 2047)/2048;

  cout << F("Card Size: ") << cardCapacityMB;
  cout << F(" MB, (MB = 1,048,576 bytes)") << endl;


  Serial.println("Files found on the card (name, date and size in bytes): ");

  // list all files in the card with date and size
  sd.ls (LS_R | LS_DATE | LS_SIZE);


	preob_num_str();
	//setup_SD();
	//if (!SD.begin(SD_Select))                       // Подключить SD память
	//	{
	//		Serial.println("initialization SD failed!");
	//	}
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
	data_clock_exchange();
	UpdateRegs();                                   // Обновить информацию в регистрах

	#if FASTADC                                     // Ускорить считывание аналогового канала
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif

	for (int i = 200; i <= 330; i++)                  // Очистить флаги ошибок
	{
	   regBank.set(i,0);   
	}
	
	for (unsigned int i = 40200; i <= 40330; i++)     // Очистить флаги ошибок
	{
	   regBank.set(i,0);   
	}
		for (unsigned int i = 40400; i <= 40530; i++)     // Очистить флаги ошибок
	{
	   regBank.set(i,0);   
	}

	//for (int i = 200; i <= 330; i++)                  // Очистить флаги ошибок
	//{
	//	Serial.print(i);
	//	Serial.print(" - ");
	//	Serial.print(regBank.get(i+40000));
	//	Serial.print(" - ");
	//	Serial.println(regBank.get(i));
	//}



	regBank.set(120,0);
	regBank.set(40120,0);
	regBank.set(adr_reg_count_err,0);                // Обнулить данные счетчика всех ошибок
	MsTimer2::set(30, flash_time);                   // 30ms период таймера прерывани
	MsTimer2::start();                               // Включить таймер преравания
	resistor(1, 200);                                // Установить уровень сигнала
	resistor(2, 200);                                // Установить уровень сигнала
	prer_Kmerton_On = true;                          // Разрешить прерывания на камертон

	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	Serial.println(" ");
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
