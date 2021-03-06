/*

 Kamerton4_0.ino
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
unsigned int volume_porog_D = 40;                   // ������������ �������� ������ ��� �������� ����������� FrontL,FrontR
unsigned int volume_porog_L = 200;                  // ����������� �������� ������ ��� �������� ����������� FrontL,FrontR
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


MCP23017 mcp_Klava;                                 // ���������� ������ ���������� MCP23017  2 A - in,  B - Out
MCP23017 mcp_Out1;                                  // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                  // ���������� ������ ���������� MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                // ���������� ������ ���������� MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------
const int adr_reg_ind_CTS      PROGMEM           = 10081;        // ����� ����a ��������� ��������� ������� CTS
const int adr_reg_ind_DSR      PROGMEM           = 10082;        // ����� ����a ��������� ��������� ������� DSR
const int adr_reg_ind_DCD      PROGMEM           = 10083;        // ����� ����a ��������� ��������� ������� DCD

// **************** ������ ������� ������ ��� �������� ����. ����������� ��������������� ����� ����� *************
const int adr_temp_day         PROGMEM           = 240;          // ����� �������� ���������� ����
const int adr_temp_mon         PROGMEM           = 241;          // ����� �������� ���������� �����
const int adr_temp_year        PROGMEM           = 242;          // ����� �������� ���������� ���  
const int adr_file_name_count  PROGMEM           = 243;          // ����� �������� ���������� �������� ������ �����
//------------------------------------------------------------------------------------------------------------------
int regcount_err        = 0;                                     // ���������� ��� �������� ���� ������



//++++++++++++++++++++++ ������ � ������� +++++++++++++++++++++++++++++++++++++++
#define chipSelect SS
SdFat sd;
File myFile;
SdFile file;
// ������� ����������, ������������ ������� ���������� SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;

//++++++++++++++++++++ ���������� ����� ����� ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.TXT";
//------------------------------------------------------------------------------

char c;  // ��� ����� ������� � ��� �����


// Serial output stream
ArduinoOutStream cout(Serial);

//*********************������ � ������ ����� ******************************

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

//+++++++++++++++++++++ ��������� ���������� +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

byte regs_in[5];                                    // �������� ������ � ������ �������� CPLL
byte regs_out[4];                                   // �������� ������ � ������ ��������
byte regs_crc[1];                                   // �������� ������ � ������ �������� ����������� �����


byte Stop_Kam = 0;                                  // ���� ��������� ������ ���. �� ���������
bool prer_Kmerton_On = true;                        // ���� ���������� ���������� ��������
bool test_repeat     = true;                        // ���� ���������� �����
volatile bool prer_Kmerton_Run = false;             // ���� ���������� ���������� ��������
#define BUFFER_SIZEK 64                             // ������ ������ �������� �� ����� 128 ����
unsigned char bufferK;                              // ������� ���������� ����������� ����

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ������� ����� 
const unsigned int adr_kontrol_day        PROGMEM      = 40046; // ����� ����
const unsigned int adr_kontrol_month      PROGMEM      = 40047; // ����� �����
const unsigned int adr_kontrol_year       PROGMEM      = 40048; // ����� ���
const unsigned int adr_kontrol_hour       PROGMEM      = 40049; // ����� ���
const unsigned int adr_kontrol_minute     PROGMEM      = 40050; // ����� ������
const unsigned int adr_kontrol_second     PROGMEM      = 40051; // ����� �������

// ��������� ������� � �����������
const unsigned int adr_set_kontrol_day    PROGMEM      = 40052;   // ����� ����
const unsigned int adr_set_kontrol_month  PROGMEM      = 40053;   // ����� �����
const unsigned int adr_set_kontrol_year   PROGMEM      = 40054;   // ����� ���
const unsigned int adr_set_kontrol_hour   PROGMEM      = 40055;   // ����� ���
const unsigned int adr_set_kontrol_minute PROGMEM      = 40056;   // ����� ������

// ����� ������ �����
const unsigned int adr_Mic_Start_day      PROGMEM      = 40096; // ����� ����
const unsigned int adr_Mic_Start_month    PROGMEM      = 40097; // ����� �����
const unsigned int adr_Mic_Start_year     PROGMEM      = 40098; // ����� ���
const unsigned int adr_Mic_Start_hour     PROGMEM      = 40099; // ����� ���
const unsigned int adr_Mic_Start_minute   PROGMEM      = 40100; // ����� ������
const unsigned int adr_Mic_Start_second   PROGMEM      = 40101; // ����� �������

// ����� ��������� �����
const unsigned int adr_Mic_Stop_day       PROGMEM       = 40102; // ����� ����
const unsigned int adr_Mic_Stop_month     PROGMEM       = 40103; // ����� �����
const unsigned int adr_Mic_Stop_year      PROGMEM       = 40104; // ����� ���
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 40105; // ����� ���
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 40106; // ����� ������
const unsigned int adr_Mic_Stop_second    PROGMEM       = 40107; // ����� �������

// ����������������� ���������� �����
const unsigned int adr_Time_Test_day      PROGMEM       = 40108; // ����� ����
const unsigned int adr_Time_Test_hour     PROGMEM       = 40109; // ����� ���
const unsigned int adr_Time_Test_minute   PROGMEM       = 40110; // ����� ������
const unsigned int adr_Time_Test_second   PROGMEM       = 40111; // ����� �������

const unsigned int adr_reg_temp_year      PROGMEM       = 40112; // ������� �������� ���������� ���  
const unsigned int adr_reg_temp_mon       PROGMEM       = 40113; // ������� �������� ���������� �����
const unsigned int adr_reg_temp_day       PROGMEM       = 40114; // ������� �������� ���������� ���� 
const unsigned int adr_reg_file_name      PROGMEM       = 40115; // ������� �������� ������� ������  
const unsigned int adr_reg_file_tek       PROGMEM       = 40116; // ������� �������� ������� ������  


const unsigned int adr_control_command    PROGMEM       = 40120; //����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 40121; // ������� �������� ������� ������  

const unsigned int adr_set_time           PROGMEM       = 36;    // ����� ���� ���������

//---------------------------������ ���������   ---------------------------------------------------
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
const char  txt_message55[]   PROGMEM            = "Signal mi�rophone 30  mV                      ON"            ;
const char  txt_message56[]   PROGMEM            = "Command PTT    OFF microphone                    send!"      ;
const char  txt_message57[]   PROGMEM            = "Command PTT    ON  microphone                    send!"      ;
const char  txt_message58[]   PROGMEM            = "Command sensor OFF microphone                    send!"      ;  
const char  txt_message59[]   PROGMEM            = "Command sensor ON  microphone                    send!"      ;

//++++++++++++++++++++++++++++++ ������ ������ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
txt_message54,                                // " ****** Test mi�rophone start! ******"                       ;
txt_message55,                                // "Signal mi�rophone 30  mV                      ON"            ;
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

// ========================= ���� �������� ============================================

void dateTime(uint16_t* date, uint16_t* time)                  // ��������� ������ ������� � ���� �����
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
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

const int8_t ERROR_LED_PIN = 13;


//------------------------------------------------------------------------------

// Size of file base name.  Must not be larger than six.
//������ ������� ����� ����� �����. ������ ���� �� ������, ��� �����.
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
//void preob_num_str() // ��������� ������������ ����� �����, ���������� �� ������� ���� � �������� ������
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
//	itoa (year_temp,str_year_file, 10);                                        // �������������� ���� ��� � ������ ( 10 - ���������� ������) 
//
//	
//	if (month <10)
//		{
//		   itoa (0,str_mon_file0, 10);                                         //  �������������� ���� �����  � ������ ( 10 - ���������� ������) 
//		   itoa (month,str_mon_file10, 10);                                    //  �������������� ����� � ������ ( 10 - ���������� ������) 
//		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // �������� 2 �����
//		}
//	else
//		{
//		   itoa (month,str_mon_file, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
//		}
//
//
//	if (day <10)
//		{
//		   itoa (0,str_day_file0, 10);                                         // �������������� ����� � ������ ( 10 - ���������� ������) 
//		   itoa (day,str_day_file10, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
//		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // �������� 2 �����
//		}
//	else
//		{
//		itoa (day,str_day_file, 10);                                           // �������������� ����� � ������ ( 10 - ���������� ������) 
//		}
//		 
//	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // �������� 2 �����
//	sprintf(str2, "%s%s",str1, str_day_file);                                  // �������� 2 �����
//	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // ��������� ����� ����� � file_name
//	//Serial.println(fileName);
//}

//+++++++++++++++++++++++++++++++++++++++++++++������  �������������� SD ++++++++++++++++++++++++++


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
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
//	RTC.adjust(set_time);             
}

void flash_time()                                              // ��������� ���������� ���������� 
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
	//while(prer_Kmerton_Run == 1) {}                                // ��������� ��������� ����������
	//	digitalWrite(ledPin13,HIGH);
	// // digitalWrite(ledPin13,!digitalRead(ledPin13));               // ���� ������� MODBUS
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
void UpdateRegs()                                        // �������� ��������
{
	//-----������ ���� ------------
	//-----���������� ��� 0
	 while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // ��������� ���������� �������� ??
	 reg_Kamerton();                                     // �������� ������ �� �������� �    �������� 
		// ������������ �������� ������ �� ��������� �� ����� ������
	  //-----���������� ��� 0
	 set_rele = regBank.get(1);
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
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor �������� ������

	 //-----���������� ��� 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT �������� ������

	 //-----���������� ��� 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT ���

	  //-----���������� ��� 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor ���

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
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  ����. �.
  
	//-----���������� ��� 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 ����. �.

	 //-----���������� ��� 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "���." 

	 //-----���������� ��� 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "��C."

	 //-----���������� ��� 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "��-�����1."

	 //-----���������� ��� 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "��-�����2."

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

	 // regBank.set(adr_reg_count_Mic, count_test_Mic);                   // �������� � ������� ��������� �������� �������� ����� Mic
	  regBank.set(adr_reg_err_Mic, err_count_Mic);                        // �������� � ������� ��������� �������� ������ CTS
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
		DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
		RTC.adjust(set_time);                                                 // �������� ����� � ���������� �����  
		Serial.println("set_clock");
		regBank.set(adr_set_time, 0);                                         // �������� � ������� ������� ��������� ���������� �������
}
void data_clock_exchange()
{
	/*
	DateTime now = RTC.now();
	uint16_t year_temp = now.year()-2000;
	uint8_t day_temp = now.day();
	uint8_t mon_temp = now.month();

	byte  b = i2c_eeprom_read_byte(0x50, adr_temp_day);                             //access an address from the memory
		delay(10);

		if (b != day_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_day, day_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // �������  �������� ���������� ����� �����
		  */
}
void time_control() // ��������� ������ �������� ������� � �������� ��� �������� � ��
{
	DateTime now = RTC.now();
	regBank.set(adr_kontrol_day  , now.day());
	regBank.set(adr_kontrol_month, now.month());
	regBank.set(adr_kontrol_year, now.year());
	regBank.set(adr_kontrol_hour, now.hour());
	regBank.set(adr_kontrol_minute, now.minute());
	regBank.set(adr_kontrol_second, now.second());
}
void time_control_get()   // �������� ��������� �������� ���������� ��������� �������
{
  for (unsigned int i = 0; i < 6; i++)     // 
	{
	   Serial.print(regBank.get(40046+i));   
	   Serial.print(" "); 
	}
Serial.println();   
}

// ++++++++++++++++++++++ �������������� SD ++++++++++++++++++++++++++++


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
void formatCard() 
{
  cout << endl;
  cout << F("Formatting\n");
  initSizes();
  if (card.type() != SD_CARD_TYPE_SDHC) 
  {
	cout << F("FAT16\n");
	makeFat16();
  }
  else 
  {
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
//void serial_print_date()                           // ������ ���� � �������    
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
			//Wire.requestFrom(address_AD5252, 1, true);  // ������� ��������� ������ ��������� 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
}
void FileOpen()
{
	int temp_file_name = 0;
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

 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48);
  regBank.set(adr_reg_file_name,temp_file_name);      


  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {

  }
  else
  {
	Serial.print(fileName);
	Serial.println(F("  Open Ok!"));

	DateTime now = RTC.now();

	regBank.set(adr_Mic_Start_day , now.day());           // ����� ������ �����
	regBank.set(adr_Mic_Start_month, now.month());
	regBank.set(adr_Mic_Start_year, now.year());
	regBank.set(adr_Mic_Start_hour, now.hour());
	regBank.set(adr_Mic_Start_minute, now.minute());
	regBank.set(adr_Mic_Start_second, now.second());
	// �������� 			
	regBank.set(adr_Time_Test_day, 0); 
	regBank.set(adr_Time_Test_hour, 0); 
	regBank.set(adr_Time_Test_minute, 0); 
	regBank.set(adr_Time_Test_second, 0); 
	myFile.println ("");
	myFile.print ("Start test   ");
	file_print_date();
	myFile.println ("");
	delay(100);
   }
  regBank.set(adr_control_command,0);  
}
void FileClose()
{
	
	myFile.println ("");
	myFile.print ("Stop test   ");
	file_print_date();
	myFile.println ("");
	myFile.close();

	if (sd.exists(fileName))
		{ 
        Serial.println();
		Serial.print(fileName);
		Serial.println("  Close  OK!.");
		}
	else 
		{
		Serial.println();
		Serial.print(fileName);
		Serial.println(" doesn't exist.");  
		}
	regBank.set(adr_control_command,0);
}

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
//  
//} 
void preob_num_str() // ��������� ������������ ����� �����, ���������� �� ������� ���� � �������� ������
{
	DateTime now = RTC.now();
	day   = now.day();
	month = now.month();
	year  = now.year();
	int year_temp = year-2000;
	itoa (year_temp,str_year_file, 10);                                        // �������������� ���� ��� � ������ ( 10 - ���������� ������) 

	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  �������������� ���� �����  � ������ ( 10 - ���������� ������) 
		   itoa (month,str_mon_file10, 10);                                    //  �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // �������� 2 �����
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // �������������� ����� � ������ ( 10 - ���������� ������) 
		   itoa (day,str_day_file10, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // �������� 2 �����
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // �������� 2 �����
	sprintf(str2, "%s%s",str1, str_day_file);                                  // �������� 2 �����
	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // ��������� ����� ����� � file_name
	//Serial.println(fileName);

	regBank.set(adr_reg_temp_day, day);  
	regBank.set(adr_reg_temp_mon, month); 
	regBank.set(adr_reg_temp_year, year-2000); 
}

void set_namber_file_zero()
{
   //i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                              // ������� ������ ����� �������� � "0"
   //data_clock_exchange();
   //regBank.set(adr_control_command,0);
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
	15 - ���������� ������� �������

	*/
	UpdateRegs() ;

	int test_n = regBank.get(adr_control_command);                                  //�����  40120
		
	switch (test_n)
	{
		case 1:
			 sensor_all_off();                                                      // ��������� ��� �������
			break;
		case 2:		
			 sensor_all_on();                                                       // �������� ��� �������
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
			 test_mikrophon();                                                      // ������������ ���������
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
				Reg_count_clear();			                                        // ����� ��������� ������                    
				break;
		case 17:
			 formatCard();              //
				break;
		case 18:
			                         //
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
	regBank.set(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(1000);
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT     �������� ������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs(); 
	delay(1000);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];    

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = regBank.get(40201);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(40201,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(201,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja ��������  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = regBank.get(40202);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(40202,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(202,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  ��������  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor ��������  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40205);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(40205,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(205,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 ��������  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = regBank.get(40206);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			regBank.set(40206,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(206,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor ���������� ���������
		  {
			regcount = regBank.get(40207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // ���� RL4 XP1 12  HeS2e   ���������� ��������� �����������
		  {
			regcount = regBank.get(40208);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40208,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(208,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� ��������  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = regBank.get(40209);                                          // ����� �������� ������ ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ���������� ��������� ����������
			regBank.set(40209,regcount);                                            // ����� �������� ������ ���������� ��������� ����������
			regBank.set(209,1);                                                     // ���������� ���� ������ ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. ��������  - Pass
			   }
		  }

	regBank.set(adr_control_command,0);                                             // ��������� ���������    
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
	regBank.set(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ ������ �������� ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
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
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
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
			regcount = regBank.get(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
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
			regcount = regBank.get(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
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
			regcount = regBank.get(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������                                            // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
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
			regcount = regBank.get(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			regBank.set(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������        
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
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
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
			regcount = regBank.get(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������        
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
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
			regcount = regBank.get(40211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(13,1);                                                              // XP8 - 2   sensor �������� ������
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  -
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
			regcount = regBank.get(40212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
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
			regcount = regBank.get(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }



	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ��������� ���������
	regBank.set(5,1);                                                               // �������� ����������� �������� "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
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
							if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
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
			regcount = regBank.get(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,true);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,1);                                                              // �������� ���������� ��������
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
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
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
			regcount = regBank.get(40219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			regBank.set(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT     �������� ������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs(); 
	delay(1000);
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}

void set_rezistor()
{
	int mwt = regBank.get(40010);             // ����� �������� �������� �������
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
	test_instr_off();                                                               // ��������� ���� � �������, �������� ����������
	test_instr_on();                                                                // �������� ����������� �������, ��������� ���������
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // ���������� ������� ������� 30 ��
	resistor(2, 30);                                                                // ���������� ������� ������� 30 ��
	regBank.set(2,1);                                                               // ������ ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();                                                                   // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                           // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,35);                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,35);                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,35);                                 // �������� ������� ������� �� ������ LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,35);                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,35);                                 // �������� ������� ������� �� ������ mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,35);                                 // �������� ������� ������� �� ������ mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,35);                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,35);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,35);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
                                                   //
	regBank.set(5,1);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (�������� ��������)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT �������� PTT �����������
	regBank.set(16,0);                                                              // ������ ��������� ���������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(29,1);                                                              // ��� XP1- 13 HeS2Ls ������  ��� ���� ����������� ��������� ����������� 
	UpdateRegs();                                                                   // 
	delay(400);                                                                     //
	byte i53 = regs_in[3];                                                          // �������� ������� ��������� ���������
		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = regBank.get(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // �������� ����������� �������  - Pass
			  }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    �������� ������ ����� �� ���� ��������� �����������
	delay(20);
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,150);                                // �������� ������� ������� �� ������ LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,150);                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,35);                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,35);                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,35);                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,35);                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,35);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,35);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(5,0);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (��������� �������� �����������)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	regBank.set(2,0);                                                               // ��������� ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();     

	regBank.set(adr_control_command,0);                                             // ��������� ���������    
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
	test_disp_off();                                                                // ��������� ���� � �������, �������� ����������
	test_disp_on();                                                                 // �������� ����������� �������, ��������� ���������
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // ���������� ������� ������� 30 ��
	resistor(2, 30);                                                                // ���������� ������� ������� 30 ��
	regBank.set(1,1);                                                               // ������ ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();                                                                   // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,35);                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,35);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,35);                                 // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,35);                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,35);                                 // �������� ������� ������� �� ������ mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,35);                                 // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,35);                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,35);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,35);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
	regBank.set(10,1);                                                              // ������ ����������� ������� �� ����� XP1 10 ��������� ��������� ����������
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	regBank.set(16,0);                                                              // ������ ��������� ���������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs();                                                                   // 
	delay(200);                                                                     //
	byte i5 = regs_in[3];                                                           // 
		if(bitRead(i5,6) == 0)                                                      // ��������  ��������� ��������� ����������
		  {
			regcount = regBank.get(40182);                                          // ����� �������� ������ ��������� ��������� ����������          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regBank.set(40182,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			regBank.set(182,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			resistor(1, 255);                                                       // ���������� ������� ������� � �������� ��������e
			resistor(2, 255);                                                       // ���������� ������� ������� � �������� ��������e
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14])));                   // "Microphone headset dispatcher signal          ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  �������� ������ ����� �� ���� ��������� ����������
	delay(20);
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,200);                                // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,200);                                // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,35);                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,35);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,35);                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,35);                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,35);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,35);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";

	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   ��������� sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   ���������  sensor ���������� ��������� ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(10,0);                                                              // ������ ����������� ������� �� ����� XP1 10  (��������� �������� ����������)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.set(1,0);                                                               // ��������� ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
 }
void test_MTT()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24])));                   // " ****** Test MTT start! ******"                              ; 
	myFile.println(buffer);                                                         // " ****** Test MTT start! ******"                              ; 
	file_print_date();
	myFile.println("");
	test_MTT_off();                                                                 // ��������� ���� � �������, �������� ����������
	test_MTT_on();                                                                  // �������� ����������� �������, ��������� ���������
//	myFile.println("");
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT �������� ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD ������ ������� DCD ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"
	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,35);                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,35);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40252,252,35);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40253,253,35);                                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,35);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40255,255,35);                                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,35);                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,35);                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,35);                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                               // ���������� ������� ������� 60 ��
	resistor(2, 130);                                                               // ���������� ������� ������� 60 ��
	regBank.set(3,1);                                                               // �������� ������ �� ���� ��������� ������ Mic3p
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                   // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,35);                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,35);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	//measure_vol_min(analog_LineL,     40252,252,25);                              // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,35);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,35);                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,35);                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,35);                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ ��������� ������� �������  ++++++++++++++++++++++++++++++++++++
//	measure_vol_max(analog_LineL,    40260,260,35);                                 // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,    40261,261,35);                                 // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone,40262,262,90);                                 // �������� ������� ������� �� ������ mag phone  "Test MTT ** Signal Mag phone                                ON  - ";
	// +++++++++++++++++++++ �������� ������������ ������ ��� �� ������ HangUp  DCD ON +++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                               // ��������� ������ �� ���� ��������� ������ Mic3p
	regBank.set(6,1);                                                               // ���� RL5. ������ ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                   //   
	if (test_repeat == false) myFile.println(buffer);                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       40256,256,30);                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                   // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,      40259,259,120);                                //  �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS             On      
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD ON  �������� ������
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
	regBank.set(25,1);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT ��������� ������ ���� � "1"
	regBank.set(6,0);                                                               // ���� RL5. ��������� ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
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
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = regs_in[0];    
	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor �������� ������               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = regBank.get(40274);                                          // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(40274,regcount);                                            // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(274,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40270);                                          // ����� �������� ������                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40270,regcount);                                            // ����� �������� ������ 
			regBank.set(270,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = regBank.get(40271);                                          // ����� �������� ������  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40271,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regBank.set(271,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(17,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	regBank.set(20,1);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // ��������� �������
	delay(400);
		if(bitRead(regs_in[0],3) == 0)                                          // J8-11     XP7 2 sensor �������� ������             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = regBank.get(40275);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta ruchnaja                            ON  - ";
			regBank.set(40275,regcount);                                            // ����� �������� ������ sensor �������� ������
			regBank.set(275,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40272);                                          // ����� �������� ������ PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40272,regcount);                                            // ����� �������� ������ PTT  MTT (CTS)
			regBank.set(272,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = regBank.get(40273);                                          // ����� �������� ������  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40273,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			regBank.set(273,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
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
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT �������� ������
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //

	byte i50 = regs_in[0];    
	
	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor ��������                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = regBank.get(40276);                                          // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(40276,regcount);                                            // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(276,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	  // 2)  ��������  �� ����������  XP8 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP8 1 PTT1 ��������   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = regBank.get(40278);                                          // ����� �������� ������ 
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40278,regcount);                                            // ����� �������� ������                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			regBank.set(278,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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


	regBank.set(13,1);                                                              // XP8 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(14,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;


	UpdateRegs();                                                                   // ��������� �������
	delay(400);

			if(bitRead(regs_in[0],4) == 0)                                          // J8-11     XP7 2 sensor ��������                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = regBank.get(40277);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta nognaja                             ON  - ";
			regBank.set(40277,regcount);                                            // ����� �������� ������ sensor �������� ������
			regBank.set(277,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	  // 2)  ��������  �� ����������  XP8 1 PTT1 ��������  CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = regBank.get(40279);                                          // ����� �������� ������                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40279,regcount);                                            // ����� �������� ������                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			regBank.set(279,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	regBank.set(14,0);                                                              //   XP8 1 PTT ��������  
	regBank.set(13,0);                                                              //   XP8 2 sensor ��������  
	UpdateRegs();                                                                   // ��������� �������
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}
void test_mikrophon()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test mi�rophone start! ******"                       ;
	myFile.println(buffer);                                                         // " ****** Test mi�rophone start! ******"                       ;
	file_print_date();
	myFile.println("");
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������

	regBank.set(14,0);    // XP8 - 1   PTT �������� ������
	regBank.set(17,0);    // J8-12     XP7 4 PTT2   ����. �.
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 ����. �.
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR ���.  
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS ���   ��� ����������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   //"Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(400);

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������� ������� �  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	byte i52 = regs_in[2];    
			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor ���������� ���������
		  {
			regcount = regBank.get(40207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

	 UpdateRegs(); 
	  // 2)  ��������  �� ���������� PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = regBank.get(40264);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40264,regcount);                                            // ����� �������� ������ 
			regBank.set(264,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ��������� �������  microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	i52 = regs_in[2];    

	  if(bitRead(i52,5) == 0)                                                 // XS1 - 6   sensor ���������� ���������
		  {
			regcount = regBank.get(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(15,1);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	i52 = regs_in[2];    

	if(bitRead(i52,5) == 0)                                             // XS1 - 6   sensor ���������� ���������
		  {
			regcount = regBank.get(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }

	 UpdateRegs(); 
	  // 2)  ��������  �� ���������  PTT microphone
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = regBank.get(40266);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40266,regcount);                                            // ����� �������� ������ 
			regBank.set(266,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������  �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40320,320,30);                                // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,30);                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40322,322,30);                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40323,323,30);                                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,30);                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40325,325,30);                                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,30);                                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,30);                                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,30);                                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 60);                                                                // ���������� ������� ������� 60 ��
	resistor(2, 60);                                                                // ���������� ������� ������� 60 ��
	regBank.set(9,1);                                                               // �������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	UpdateRegs();                                                                   // ��������� �������
	delay(1400);

	Serial.println("Signal Mag phone");
	measure_vol_max(analog_mag_phone,40298,298,180);                                // �������� ������� ������� �� ������ mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,    40299,299,180);                                // �������� ������� ������� �� ������ "Test Microphone ** Signal LineL                      ON  - ";  
	
	measure_vol_min(analog_FrontL,    40320,320,30);                                // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40321,321,30);                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     40323,323,30);                                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40324,324,30);                                // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       40326,326,30);                                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40327,327,30);                                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40328,328,30);                                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     

	regBank.set(9,0);                                                               // ��������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void testGGS()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[48])));                   // " ****** Test GGS start! ******"      ;
	myFile.println(buffer);                                                         // " ****** Test GGS start! ******"      ;
	file_print_date();
	myFile.println("");
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������  
	regBank.set(6,0);                                                               // ���� RL5 ���� Front L, Front R
	resistor(1, 220);                                                               // ���������� ������� ������� 60 ��
	resistor(2, 220);                                                               // ���������� ������� ������� 60 ��
	//UpdateRegs();                                                                   // ��������� �������
	delay(400);
	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
		//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40280,280,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40281,281,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40285,285,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40286,286,30);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;
	regBank.set(6,1);                                                               // ���� RL5 ���� Front L, Front R
	UpdateRegs();                                                                   // ��������� �������
	delay(400);

	measure_vol_max(analog_FrontL,    40290,290,40);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,40);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,50);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       40289,289,160);                               // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      ON  - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������          
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	UpdateRegs(); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;

	measure_vol_max(analog_FrontL,    40290,290,40);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    40291,291,40);                                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	measure_vol_min(analog_LineL,     40282,282,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40283,283,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40284,284,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_max(analog_mag_phone, 40292,292,50);                                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_min(analog_ggs,       40286,286,30);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40287,287,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40288,288,25);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(6,0);                                                               // ���� RL5 ���� Front L, Front R
	UpdateRegs();    
	delay(300);
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio1()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[50])));                   // " ****** Test Radio1 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio1 start! ******"                           ;
	file_print_date();
	myFile.println("");
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1, 110);                                                               // ���������� ������� ������� 300 ��
	resistor(2, 110);                                                               // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	UpdateRegs(); 
	delay(300);
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40300,300,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,30);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40307,307,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40308,308,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(4,1);                                                               //  ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(400);

	measure_vol_min(analog_FrontL,    40300,300,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40301,301,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40302,302,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40303,303,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40304,304,30);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40305,305,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40306,306,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, 40309,309,220);                               // �������� ������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, 40308,308,25);                                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio2()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));                   // " ****** Test Radio2 start! ******"                           ;
	myFile.println(buffer);                                                         // " ****** Test Radio2 start! ******"                           ;
	file_print_date();
	myFile.println("");
	regBank.set(7,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1, 110);                                                               // ���������� ������� ������� 300 ��
	resistor(2, 110);                                                               // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	UpdateRegs(); 
	delay(300);
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40310,310,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,30);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, 40318,318,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(7,1);                                                               //  ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(400);

	measure_vol_min(analog_FrontL,    40310,310,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    40311,311,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     40312,312,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     40313,313,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, 40314,314,30);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, 40315,315,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       40316,316,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, 40317,317,25);                                // �������� ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, 40319,319,250);                               // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";

	regBank.set(7,0);                                                               // ���� RL6 ���� Center
	UpdateRegs();     
	regBank.set(adr_control_command,0);    
}

void test_instr_off()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[7])));                    // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"                   ;  
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(300);
	UpdateRegs(); 
	byte i52 = regs_in[2];    
	 
	  // 1)  �������� ������� �� ���������� ��������� ����������� 2 ����������
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = regBank.get(40207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40207,regcount);                                            // ����� �������� ������ ������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40220);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40220,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(220,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
}
void test_instr_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[10])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor    send!"
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	regBank.set(16,1);                                                              // XS1 - 6   sensor ���
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	regBank.set(13,1);                                                              // XP8 - 2           sensor �������� ������
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(300);
	UpdateRegs(); 
	byte i52 = regs_in[2];    
	  // 3)  �������� ������� �� ����������� ��������� ����������� 2 ����������
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor �������  - Pass
			  }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40221);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40221,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(221,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  �������  - Pass
		   }
		 }
}

void test_disp_off()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[16])));                   // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"     
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    ��������� ������ ��������� ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(300);
	UpdateRegs(); 
	byte i52 = regs_in[2];    
	 
	  // 1)  �������� ������� �� ���������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40205);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(40205,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(205,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " ��������  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  ����������� ��������� ����������
		  {
			regcount = regBank.get(40206);                                          // ����� �������� ������ sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			regBank.set(40206,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������
			regBank.set(206,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = regBank.get(40207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40207,regcount);                                            // ����� �������� ������ ������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 
	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // ��������� ���������� PTT ����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = regBank.get(40222);                                          // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40222,regcount);                                            // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(222,1);                                                     // ���������� ����  ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
}
void test_disp_on()
{
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[19])));                   // "Command sensor ON  headset dispatcher 2          send!"                   ;   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher 2          send!"                   ;    
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ����������� ��������� ���������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	regBank.set(16,1);                                                              // XS1 - 6   sensor ���
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	regBank.set(13,1);                                                              // XP8 - 2           sensor �������� ������
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(300);
	UpdateRegs(); 
	byte i52 = regs_in[2];    

	  // 3)  �������� ������� �� ����������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40215);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(40215,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  �������  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ���������� 
		  {
			regcount = regBank.get(40216);                                          // ����� �������� ������    sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			regBank.set(40216,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� 
			regBank.set(216,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " �������  - Pass
			  }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT ����������   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = regBank.get(40223);                                          // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40223,regcount);                                            // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(223,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  �������  - Pass
		   }
		 }
}

void test_MTT_off()
{
	unsigned int regcount = 0;
	regBank.set(25,1);                                                              // "Command sensor OFF MTT                           send! "     ;     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                   // "Command sensor OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF MTT                           send! "     ;      
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS  ���. ��������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(300);
	UpdateRegs(); 
	delay(100);
	byte i50 = regs_in[0];    

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = regBank.get(40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ ��������  - Pass
			   }
		  }
		   UpdateRegs(); 

	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� CTS MTT
		  {
			regcount = regBank.get(40263);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40263,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regBank.set(263,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = regBank.get(40267);                                          // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40267,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	regBank.set(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR ���. �������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(600);
	UpdateRegs(); 
	  // 1)  �������� ������� MTT �� ��������� 
	byte i50 = regs_in[0];    
		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = regBank.get(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ �������  - Pass
			   }
		  }

		delay(300);
		UpdateRegs(); 
	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ��������� CTS MTT
		  {
			regcount = regBank.get(40265);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40265,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regBank.set(265,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " ������ �������  - Pass
			   }
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                         // ��������� ��������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = regBank.get(40268);                                          // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40268,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";������ �������  - Pass
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
		measure_volume(_istochnik);                                                 // �������� ������� ������� �� ������
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ������ "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ������ "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ������ "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ������ "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ������ "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ������ "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ������ "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ������ "Test GGS ** Signal GG Radio1                                OFF - ";
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
		if(voltage10 > _porogV)                                                      // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = regBank.get(_adr_count);                                  // ����� �������� ������ 
				regcount++;                                                          // ��������� ������� ������ ������ 
				regBank.set(_adr_count,regcount);                                    // ����� �������� ������ ������ 
				regBank.set(_adr_count+200,voltage10);                               // ����� ������ ������ ������ 
				regBank.set(_adr_flagErr,1);                                         // ���������� ���� ������  ������ 
				regcount_err = regBank.get(adr_reg_count_err);                       // �������� ������ �������� ���� ������
				regcount_err++;                                                      // ��������� ������� ���� ������ 
				regBank.set(adr_reg_count_err,regcount_err);                         // ��������� ������ �������� ���� ������
				regBank.set(120,1);                                                  // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // ��������� �������� ������
				myFile.print("     ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
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

	measure_volume(_istochnik);                                                 // �������� ������� ������� �� ������
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ������ "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ������ "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ������ "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ������ "Test GGS ** Signal mag phone                                ON  - ";
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
	
		if(voltage10 < _porogV)                                                     // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = regBank.get(_adr_count);                                 // ����� �������� ������ 
				regcount++;                                                         // ��������� ������� ������ ������ 
				regBank.set(_adr_count, regcount);                                  // ����� �������� ������ ������ 
				regBank.set(_adr_count+200,voltage10);                              // ����� ������ ������ ������ 
				regBank.set(_adr_flagErr,1);                                        // ���������� ���� ������  ������ 
				regcount_err = regBank.get(adr_reg_count_err);                       // �������� ������ �������� ���� ������
				regcount_err++;                                                      // ��������� ������� ���� ������ 
				regBank.set(adr_reg_count_err,regcount_err);                         // ��������� ������ �������� ���� ������
				regBank.set(120,1);                                                 // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // ��������� �������� ������
				myFile.print("     ");  
				myFile.print(voltage); 
				myFile.println(" V");
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
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

		Serial.print("voltage - ");
		Serial.println(voltage10);
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
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  ����� (6) �������  ����������� ������
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "���."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "��C." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "��-�����1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "��-�����2."
  
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
	regBank.add(1);     // ���� RL0 ����  MIC1P
	regBank.add(2);     // ���� RL1 ����  MIC2P
	regBank.add(3);     // ���� RL2 ����  MIC3P
	regBank.add(4);     // ���� RL3 ����  LFE  "���."
	regBank.add(5);     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
	regBank.add(6);     // ���� RL5 ���� Front L, Front R
	regBank.add(7);     // ���� RL6 ���� Center
	regBank.add(8);     // ���� RL7 ������� �����
  
	regBank.add(9);     // ���� RL8 ���� �� ��������
	regBank.add(10);    // ���� RL9 XP1 10 ��������� ��������� ����������
	regBank.add(11);    // �������� J24 - 2 
	regBank.add(12);    // �������� J24 - 1 
	regBank.add(13);    // XP8 - 2   sensor �������� ������
	regBank.add(14);    // XP8 - 1   PTT �������� ������
	regBank.add(15);    // XS1 - 5   PTT ���
	regBank.add(16);    // XS1 - 6   sensor ���
 
	regBank.add(17);    // J8-12     XP7 4 PTT2   ����. �.
	regBank.add(18);    // XP1 - 20  HangUp  DCD
	regBank.add(19);    // J8-11     XP7 2 sensor  ����. �.
	regBank.add(20);    // J8-23     XP7 1 PTT1 ����. �.
	regBank.add(21);    // XP2-2     sensor "���."  
	regBank.add(22);    // XP5-3     sensor "��C."
	regBank.add(23);    // XP3-3     sensor "��-�����1."
	regBank.add(24);    // XP4-3     sensor "��-�����2."
 
	regBank.add(25);    // XP1- 19 HaSs      sensor ����������� ������    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR ���.  
	regBank.add(27);    // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.add(29);    // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS ���   ��� ����������
	regBank.add(31);    // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.add(32);    // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	regBank.add(118);   // ���� ��������� ������������ ��������
	regBank.add(119);   // 
	regBank.add(120);   // ���� ��������� ������������� ����� ������
 


	regBank.add(200);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(201);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(202);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(203);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(204);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(205);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(206);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(207);                         // ���� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(208);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(209);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(210);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(211);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(212);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(213);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(214);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(215);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(216);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(217);                         // ���� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(218);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(219);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	regBank.add(220);                         // ���� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(221);                         // ���� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(222);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(223);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(224);                         // ���� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(225);                         // ���� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(226);                         // ���� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(227);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(228);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(229);                         // ���� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(230);                         // ���� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(231);                         // ���� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(232);                         // ���� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(233);                         // ���� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(234);                         // ���� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(235);                         // ���� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(236);                         // ���� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(237);                         // ���� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(238);                         // ���� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(239);                         //

	regBank.add(240);                         // ���� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(241);                         // ���� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(242);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(243);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(244);                         // ���� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(245);                         // ���� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(246);                         // ���� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(247);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(248);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(249);                         //  

	regBank.add(250);                         // ���� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(251);                         // ���� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(252);                         // ���� ������ "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(253);                         // ���� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(254);                         // ���� ������ "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(255);                         // ���� ������ "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(256);                         // ���� ������ "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(257);                         // ���� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(258);                         // ���� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(259);                         // ���� ������ "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(260);                         // ���� ������ "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(261);                         // ���� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(262);                         // ���� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(263);                         // ���� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(264);                         // ���� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(265);                         // ���� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(266);                         // ���� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(267);                         // ���� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(268);                         // ���� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(269);                         //  

	regBank.add(270);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(271);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(272);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(273);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(274);                         // ���� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(275);                         // ���� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(276);                         // ���� ������ "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(277);                         // ���� ������ "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(278);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(279);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(280);                         // ���� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(281);                         // ���� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(282);                         // ���� ������ "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(283);                         // ���� ������ "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(284);                         // ���� ������ "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(285);                         // ���� ������ "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(286);                         // ���� ������ "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(287);                         // ���� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(288);                         // ���� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(289);                         // ���� ������ "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(290);                         // ���� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(291);                         // ���� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(292);                         // ���� ������ "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(293);                         // 
	regBank.add(294);                         // 
	regBank.add(295);                         // 
	regBank.add(296);                         // 
	regBank.add(297);                         // 
	regBank.add(298);                         // ���� ������ "Test Microphone ** Signal mag phone                         ON  - ";      
	regBank.add(299);                         // ���� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(300);                         // ���� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(301);                         // ���� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(302);                         // ���� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(303);                         // ���� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(304);                         // ���� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(305);                         // ���� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(306);                         // ���� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(307);                         // ���� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(308);                         // ���� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(309);                         // ���� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(310);                         // ���� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(311);                         // ���� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(312);                         // ���� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(313);                         // ���� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(314);                         // ���� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(315);                         // ���� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(316);                         // ���� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(317);                         // ���� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(318);                         // ���� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(319);                         // ���� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(320);                         // ���� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(321);                         // ���� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(322);                         // ���� ������ "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(323);                         // ���� ������ "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(324);                         // ���� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(325);                         // ���� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(326);                         // ���� ������ "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(327);                         // ���� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(328);                         // ���� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(329);                         // 

	regBank.add(330);                         // 




	regBank.add(10081);    // ����� ����a ��������� ��������� ������� CTS
	regBank.add(10082);    // ����� ����a ��������� ��������� ������� DSR
	regBank.add(10083);    // ����� ����a ��������� ��������� ������� DCD

						 //Add Input registers 30001-30040 to the register bank

	//regBank.add(30000);  // ���� 0 ���� ��� 0 - ��������   ��� "D"
	//regBank.add(30001);  // ���� 0 ���� ��� 1 - ��������   "1"
	//regBank.add(30002);  // ���� 0 ���� ��� 2 - ��������   "0"  
	//regBank.add(30003);  // ���� 0 ���� ��� 3 - ��������   "1"
	//regBank.add(30004);  // ���� 0 ���� ��� 4 - ��������   "0" ����� ���� ��������� (2)
	//regBank.add(30005);  // ���� 0 ���� ��� 5 - ��������   "1" ����� ���� ��������� (2)
	//regBank.add(30006);  // ���� 0 ���� ��� 6 - ��������   "0" ����� ���� ��������� (2)
	//regBank.add(30007);  // ���� 0 ���� ��� 7 - ��������   "0"
 // 
	//regBank.add(30008);  // ���� 1 ���� ��� 0 - ��������   CRC0
	//regBank.add(30009);  // ���� 1 ���� ��� 1 - ��������   CRC1
	//regBank.add(30010);  // ���� 1 ���� ��� 2 - ��������   CRC2
	//regBank.add(30011);  // ���� 1 ���� ��� 3 - ��������   CRC3
	//regBank.add(30012);  // ���� 1 ���� ��� 4 - ��������   ���������� ��� (Mute)
	//regBank.add(30013);  // ���� 1 ���� ��� 5 - ��������   �������������
	//regBank.add(30014);  // ���� 1 ���� ��� 6 - ��������   ������/ ������ ���� �������
	//regBank.add(30015);  // ���� 1 ���� ��� 7 - ��������   "1"
 // 
	//regBank.add(30016);  // ���� 2 ���� ��� 0 - ��������   ��� ������� ������
	//regBank.add(30017);  // ���� 2 ���� ��� 1 - ��������   ��� ������� ������
	//regBank.add(30018);  // ���� 2 ���� ��� 2 - ��������   ��� ������� ������
	//regBank.add(30019);  // ���� 2 ���� ��� 3 - ��������   ��� ������� ������
	//regBank.add(30020);  // ���� 2 ���� ��� 4 - ��������   ��� ������� ������
	//regBank.add(30021);  // ���� 2 ���� ��� 5 - ��������   ��� ������� ������
	//regBank.add(30022);  // ���� 2 ���� ��� 6 - ��������   ��� ������� ������
	//regBank.add(30023);  // ���� 2 ���� ��� 7 - ��������   "0" 

	//					 // ���� ������  ���������� ��  ����� ��������. �������� 4 �����
 // 
	//regBank.add(30024);  // ���� 1 ����� ��� 0 - ��������  ���� ����������� �� �����2
	//regBank.add(30025);  // ���� 1 ����� ��� 1 - ��������  ���� ����������� �� �����1
	//regBank.add(30026);  // ���� 1 ����� ��� 2 - ��������  ���� ����������� ������
	//regBank.add(30027);  // ���� 1 ����� ��� 3 - ��������  ���� ����������� ������ ��������
	//regBank.add(30028);  // ���� 1 ����� ��� 4 - ��������  ���� ����������� ������
	//regBank.add(30029);  // ���� 1 ����� ��� 5 - ��������   "1"
	//regBank.add(30030);  // ���� 1 ����� ��� 6 - ��������   "0" 
	//regBank.add(30031);  // ���� 1 ����� ��� 7 - ��������   "1"
 // 
	//regBank.add(30032);  // ���� 2 ����� ��� 0 - ��������   ��� ������� ������
	//regBank.add(30033);  // ���� 2 ����� ��� 1 - ��������   ��� ������� ������
	//regBank.add(30034);  // ���� 2 ����� ��� 2 - ��������   ��� ������� ������
	//regBank.add(30035);  // ���� 2 ����� ��� 3 - ��������   ��� ������� ������
	//regBank.add(30036);  // ���� 2 ����� ��� 4 - ��������   ��� ������� ������
	//regBank.add(30037);  // ���� 2 ����� ��� 5 - ��������   ��� ������� ������
	//regBank.add(30038);  // ���� 2 ����� ��� 6 - ��������   ��� ������� ������
	//regBank.add(30039);  // ���� 2 ����� ��� 7 - ��������   "0" 
 // 
	//regBank.add(30040);  // ���� 3 ����� ��� 0 - ��������   ���� ����������� �����������
	//regBank.add(30041);  // ���� 3 ����� ��� 1 - ��������   ���� ����������� ��������� ����������� 2 ����������
	//regBank.add(30042);  // ���� 3 ����� ��� 2 - ��������   ���� ����������� ��������� �����������
	//regBank.add(30043);  // ���� 3 ����� ��� 3 - ��������   ���� ����������� ��������� ���������� � 2 ����������
	//regBank.add(30044);  // ���� 3 ����� ��� 4 - ��������   ���� ����������� ��������� ����������
	//regBank.add(30045);  // ���� 3 ����� ��� 5 - ��������   ���� ����������� ��������� XS1 - 6 sensor
	//regBank.add(30046);  // ���� 3 ����� ��� 6 - ��������   ���� ����������� ���
	//regBank.add(30047);  // ���� 3 ����� ��� 7 - ��������   "0" 
 // 
	//regBank.add(30048);  // ���� 4 ����� ��� 0 - ��������   CRC0
	//regBank.add(30049);  // ���� 4 ����� ��� 1 - ��������   CRC1
	//regBank.add(30050);  // ���� 4 ����� ��� 2 - ��������   CRC2   
	//regBank.add(30051);  // ���� 4 ����� ��� 3 - ��������   CRC3   
	//regBank.add(30052);  // ���� 4 ����� ��� 4 - ��������   ���� ���������� ��������� �����������
	//regBank.add(30053);  // ���� 4 ����� ��� 5 - ��������    ���� �������������
	//regBank.add(30054);  // ���� 4 ����� ��� 6 - ��������   ���� ���������� ��������� ����������
	//regBank.add(30055);  // ���� 4 ����� ��� 7 - ��������   "0" 



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
	regBank.add(40010);    // ����� �������� �������� ������� �����������
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
	regBank.add(40096);  // ����� ����  adr_Mic_Start_day    
	regBank.add(40097);  // ����� ����� adr_Mic_Start_month  
	regBank.add(40098);  // ����� ��� adr_Mic_Start_year  
	regBank.add(40099);  // ����� ��� adr_Mic_Start_hour 
	regBank.add(40100);  // ����� ������ adr_Mic_Start_minute 
	regBank.add(40101);  // ����� ������� adr_Mic_Start_second  

	// ����� ��������� �����
	regBank.add(40102);  // ����� ���� adr_Mic_Stop_day 
	regBank.add(40103);  // ����� ����� adr_Mic_Stop_month 
	regBank.add(40104);  // ����� ��� adr_Mic_Stop_year
	regBank.add(40105);  // ����� ��� adr_Mic_Stop_hour 
	regBank.add(40106);  // ����� ������ adr_Mic_Stop_minute  
	regBank.add(40107);  // ����� ������� adr_Mic_Stop_second 

	// ����������������� ���������� �����
	regBank.add(40108);  // ����� ���� adr_Time_Test_day 
	regBank.add(40109);  // ����� ��� adr_Time_Test_hour 
	regBank.add(40110);  // ����� ������ adr_Time_Test_minute
	regBank.add(40111);  // ����� ������� adr_Time_Test_second

	regBank.add(40112);  // ����� �������� ���������� ��� 
	regBank.add(40113);  // ����� �������� ���������� ����� 
	regBank.add(40114);  // ����� �������� ���������� ����
	regBank.add(40115);  // ����� �������� ���������� �������� ���������� ������ �����
	regBank.add(40116);  // ����� �������� ���������� �������� �������� ������ �����

	regBank.add(40120);  // adr_control_command ����� �������� ������� �� ����������
	regBank.add(40121);  // ����� �������� ���� ������
	regBank.add(40200);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(40201);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(40202);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(40203);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(40204);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(40205);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(40206);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(40207);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(40208);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(40209);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(40210);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(40211);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(40212);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(40213);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(40214);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(40215);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(40216);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(40217);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(40218);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(40219);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	regBank.add(40220);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(40221);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(40222);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(40223);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(40224);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40225);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40226);                         // A���� �������� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40227);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40228);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40229);                         // A���� �������� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40230);                         // A���� �������� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40231);                         // A���� �������� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40232);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40233);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40234);                         // A���� �������� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40235);                         // A���� �������� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40236);                         // A���� �������� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40237);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40238);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40239);                         //

	regBank.add(40240);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40241);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40242);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40243);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40244);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40245);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40246);                         // A���� �������� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40247);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40248);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40249);                         //  

	regBank.add(40250);                         // A���� �������� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40251);                         // A���� �������� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40252);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40253);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40254);                         // A���� �������� ������ "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40255);                         // A���� �������� ������ "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40256);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40257);                         // A���� �������� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40258);                         // A���� �������� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40259);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40260);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40261);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40262);                         // A���� �������� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40263);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40264);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(40265);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40266);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(40267);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40268);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40269);                         //  

	regBank.add(40270);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40271);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40272);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40273);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40274);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40275);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40276);                         // A���� �������� ������ "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40277);                         // A���� �������� ������ "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40278);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40279);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40280);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40281);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40282);                         // A���� �������� ������ "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40283);                         // A���� �������� ������ "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40284);                         // A���� �������� ������ "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40285);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40286);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40287);                         // A���� �������� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40288);                         // A���� �������� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40289);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40290);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40291);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40292);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40293);                         // 
	regBank.add(40294);                         // 
	regBank.add(40295);                         // 
	regBank.add(40296);                         // 
	regBank.add(40297);                         // 
	regBank.add(40298);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         ON  - ";    
	regBank.add(40299);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40300);                         // A���� �������� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40301);                         // A���� �������� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40302);                         // A���� �������� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40303);                         // A���� �������� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40304);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40305);                         // A���� �������� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40306);                         // A���� �������� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40307);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40308);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40309);                         // A���� �������� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40310);                         // A���� �������� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40311);                         // A���� �������� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40312);                         // A���� �������� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40313);                         // A���� �������� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40314);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40315);                         // A���� �������� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40316);                         // A���� �������� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40317);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40318);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40319);                         // A���� �������� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40320);                         // A���� �������� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40321);                         // A���� �������� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40322);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40323);                         // A���� �������� ������ "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40324);                         // A���� �������� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40325);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40326);                         // A���� �������� ������ "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40327);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40328);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40329);                         // 
	regBank.add(40330);                         // 

	
	// ++++++++++++++++++++++ �������� �������� ������ ��� �������� ������� ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	regBank.add(40200);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(40201);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(40202);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(40203);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(40204);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(40205);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(40206);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(40207);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(40208);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(40209);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(40210);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(40211);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(40212);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(40213);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(40214);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(40215);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(40216);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(40217);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(40218);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(40219);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 




	regBank.add(40420);                         // A����  ;
	regBank.add(40421);                         // A����  ;
	regBank.add(40422);                         // A����  ;
	regBank.add(40423);                         // A����  ;
	regBank.add(40424);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40425);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40426);                         // A���� ������ ��������� "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40427);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40428);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40429);                         // A���� ������ ��������� "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40430);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40431);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40432);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40433);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40434);                         // A���� ������ ��������� "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40435);                         // A���� ������ ��������� "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40436);                         // A���� ������ ��������� "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40437);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40438);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40439);                         //

	regBank.add(40440);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40441);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40442);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40443);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40444);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40445);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40446);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40447);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40448);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40449);                         //  

	regBank.add(40450);                         // A���� ������ ��������� "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40451);                         // A���� ������ ��������� "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40452);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40453);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40454);                         // A���� ������ ��������� "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40455);                         // A���� ������ ��������� "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40456);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40457);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40458);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40459);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40460);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40461);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40462);                         // A���� ������ ��������� "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40463);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40464);                         // 
	regBank.add(40465);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40466);                         // 
	regBank.add(40467);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40468);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40469);                         //  

	regBank.add(40470);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40471);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40472);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40473);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40474);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40475);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40476);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40477);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40478);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40479);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40480);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40481);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40482);                         // A���� ������ ��������� "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40483);                         // A���� ������ ��������� "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40484);                         // A���� ������ ��������� "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40485);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40486);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40487);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40488);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40489);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40490);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40491);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40492);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40493);                         // 
	regBank.add(40494);                         // 
	regBank.add(40495);                         // 
	regBank.add(40496);                         // 
	regBank.add(40497);                         // 
	regBank.add(40498);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         ON  - "; 
	regBank.add(40499);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40500);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40501);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40502);                         // A���� ������ ��������� "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40503);                         // A���� ������ ��������� "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40504);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40505);                         // A���� ������ ��������� "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40506);                         // A���� ������ ��������� "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40507);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40508);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40509);                         // A���� ������ ��������� "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40510);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40511);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40512);                         // A���� ������ ��������� "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40513);                         // A���� ������ ��������� "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40514);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40515);                         // A���� ������ ��������� "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40516);                         // A���� ������ ��������� "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40517);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40518);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40519);                         // A���� ������ ��������� "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40520);                         // A���� ������ ��������� "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40521);                         // A���� ������ ��������� "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40522);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40523);                         // A���� ������ ��������� "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40524);                         // A���� ������ ��������� "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40525);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40526);                         // A���� ������ ��������� "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40527);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40528);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio2                         OFF - ";
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
	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0); // ������� ������ � ������� � ������ "set_time"
	// RTC.adjust(set_time);                                // ������
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
//	sensor_all_off();
//	data_clock_exchange();
	UpdateRegs();                                   // �������� ���������� � ���������

	#if FASTADC                                     // �������� ���������� ����������� ������
	// set prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	#endif

	for (int i = 200; i <= 330; i++)                  // �������� ����� ������
	{
	   regBank.set(i,0);   
	}
	
	for (unsigned int i = 40200; i <= 40330; i++)     // �������� ����� ������
	{
	   regBank.set(i,0);   
	}
		for (unsigned int i = 40400; i <= 40530; i++)     // �������� ����� ������
	{
	   regBank.set(i,0);   
	}
	regBank.set(120,0);
	regBank.set(40120,0);
	regBank.set(adr_reg_count_err,0);                // �������� ������ �������� ���� ������
	//MsTimer2::set(30, flash_time);                   // 30ms ������ ������� ���������
	//MsTimer2::start();                               // �������� ������ ����������
	resistor(1, 200);                                // ���������� ������� �������
	resistor(2, 200);                                // ���������� ������� �������
	prer_Kmerton_On = true;                          // ��������� ���������� �� ��������

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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
}
