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

//*********************������ � ������ ����� ******************************
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
bool prer_Kmerton_On = true;                        // ���� ���������� ���������� ��������
bool test_repeat     = true;                        // ���� ���������� �����
volatile bool prer_Kmerton_Run = false;              // ���� ���������� ���������� ��������
#define BUFFER_SIZEK 64                             // ������ ������ �������� �� ����� 128 ����
unsigned char bufferK;                              // ������� ���������� ����������� ����

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const unsigned int adr_control_command         = 40120; //����� �������� ������� �� ���������� 

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
const char  txt_message27[]   PROGMEM            = "Command sensor OFF MTT                           send! "     ;
const char  txt_message28[]   PROGMEM            = "Command PTT    OFF MTT                           send! "     ;
const char  txt_message29[]   PROGMEM            = "Command HangUp OFF MTT                           send! "     ;

const char  txt_message30[]   PROGMEM            = "Command sensor ON  MTT                           send!"      ;
const char  txt_message31[]   PROGMEM            = "Command PTT    ON  MTT                           send!"      ;
const char  txt_message32[]   PROGMEM            = "Command HangUp ON  MTT                           send!"      ;
const char  txt_message33[]   PROGMEM            = "Signal MTT microphone 30mv                    ON"            ;
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
const char  txt_error64[]  PROGMEM             = "Test MTT PTT    (DSR)                                       OFF - ";
const char  txt_error65[]  PROGMEM             = "Test MTT PTT    (CTS)                                       ON  - ";
const char  txt_error66[]  PROGMEM             = "Test MTT PTT    (DSR)                                       ON  - ";
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

const char  txt_error80[]  PROGMEM             = "";
const char  txt_error81[]  PROGMEM             = "";  
const char  txt_error82[]  PROGMEM             = "";  
const char  txt_error83[]  PROGMEM             = "";
const char  txt_error84[]  PROGMEM             = "";
const char  txt_error85[]  PROGMEM             = "";   
const char  txt_error86[]  PROGMEM             = "";
const char  txt_error87[]  PROGMEM             = "";
const char  txt_error88[]  PROGMEM             = "";   
const char  txt_error89[]  PROGMEM             = "";    

const char  txt_error90[]  PROGMEM             = "";
const char  txt_error91[]  PROGMEM             = ""; 
const char  txt_error92[]  PROGMEM             = "";    
const char  txt_error93[]  PROGMEM             = "";  

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
txt_message47                                 // "Command PTT    ON  tangenta nognaja              send!"      ;

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
txt_error64,                                  // "Test MTT PTT    (DSR)                                       OFF - ";
txt_error65,                                  // "Test MTT PTT    (CTS)                                       ON  - ";
txt_error66,                                  // "Test MTT PTT    (DSR)                                       ON  - ";
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

txt_error80,                                  //  
txt_error81,                                  // 
txt_error82,                                  //  
txt_error83,                                  //  
txt_error84,                                  // 
txt_error85,                                  // 
txt_error86,                                  //  
txt_error87,                                  //  
txt_error88,                                  //  
txt_error89,                                  //  

txt_error90,                                  //  
txt_error91,                                  // 
txt_error92,                                  // 
txt_error93                                   // 
};

// ========================= ���� �������� ============================================


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
void set_clock()
{    
		int day    = regBank.get(adr_set_kontrol_day);  
		int month  = regBank.get(adr_set_kontrol_month);          
		int year   = regBank.get(adr_set_kontrol_year);  
		int hour   = regBank.get(adr_set_kontrol_hour);  
		int minute = regBank.get(adr_set_kontrol_minute);  
		int second = 0;
		DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
		RTC.adjust(set_time);                                // �������� ����� � ���������� �����  
		regBank.set(adr_set_time, 0);                        // �������� � ������� ������� ��������� ���������� �������
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
	

	data_clock_exchange();                                                          // ��������� �� ���������� �� ����

	file_name_count = i2c_eeprom_read_byte(0x50, adr_file_name_count);              // ������� ������� ����� ����� �� ������
	regBank.set(adr_reg_file_name,file_name_count);                                 // �������  �������� ���������� ����� �����
	preob_num_str();                                                                // ������������ ��� ����� �� ���� � �������� ������
 //   regBank.set(adr_reg_file_tek,file_name_count);                                  // �������  �������� ���������� ����� �����
	 if (SD.exists(file_name))                                                      // ��������� ���� �� ����� ����
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
	regBank.set(adr_control_command,0);
}

void set_namber_file_zero()
{
   i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                              // ������� ������ ����� �������� � "0"
   data_clock_exchange();
   regBank.set(adr_control_command,0);
 }
void format_SD()
{


   regBank.set(adr_control_command,0);
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
			 test_mikrophon();                                                      // ������������ ���������
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
		case 16:
							  //
				break;
		case 17:
			 set_namber_file_zero();            //
				break;
		case 18:
			 format_SD();                       //
				break;
		default:
			regBank.set(adr_control_command,0);
		break;

	 }
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
	delay(300);
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
	regBank.set(5,1);                                                               // �������� ����������� ��������
	regBank.set(10,1);                                                              // �������� ���������� ��������
	regBank.set(13,1);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];  

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = regBank.get(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = regBank.get(40211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = regBank.get(40212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  - Pass
			  }
		  }

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
			  }
		  }

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
			  }
		  }

		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = regBank.get(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			regBank.set(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }

		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor ��������� ���������
		  {
			regcount = regBank.get(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = regBank.get(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = regBank.get(40219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			regBank.set(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
			   }
		  }
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}

void set_rezistor()
{
	int mwt = regBank.get(40010);
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
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                    // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                           // "Signal headset instructor microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,25);                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,25);                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40232,232,25);                                 // �������� ������� ������� �� ������ LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40233,233,25);                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40234,234,25);                                 // �������� ������� ������� �� ������ mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,40235,235,25);                                 // �������� ������� ������� �� ������ mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,30);                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,30);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,30);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
	//myFile.println("");                                                             //
	regBank.set(5,1);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (�������� ��������)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT �������� PTT �����������
	regBank.set(16,0);                                                              // ������ ��������� ���������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(29,1);                                                              // ��� XP1- 13 HeS2Ls ������  ��� ���� ����������� ��������� ����������� 
	UpdateRegs();                                                                   // 
	delay(200);                                                                     //
	byte i53 = regs_in[3];                                                          // �������� ������� ��������� ���������
		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = regBank.get(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
//	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    �������� ������ ����� �� ���� ��������� �����������
	delay(20);
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40224,224,200);                                // �������� ������� ������� �� ������ LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40226,226,200);                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40230,230,25);                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40231,231,25);                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40233,233,25);                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40236,236,30);                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40237,237,30);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40238,238,30);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(5,0);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (��������� �������� �����������)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
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
	//myFile.println("");

		// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                                // ���������� ������� ������� 30 ��
	resistor(2, 30);                                                                // ���������� ������� ������� 30 ��
	regBank.set(1,1);                                                               // ������ ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();                                                                   // ��������� �������
	delay(200);
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,25);                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,25);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    40242,242,25);                                 // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    40243,243,25);                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,40244,244,25);                                 // �������� ������� ������� �� ������ mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,40245,245,25);                                 // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40246,246,30);                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,30);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,30);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
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
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14])));                   // "Microphone headset dispatcher signal          ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  �������� ������ ����� �� ���� ��������� ����������
	delay(20);
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40227,227,200);                                // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,40229,229,200);                                // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40240,240,25);                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   40241,241,25);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    40243,243,25);                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      40246,246,30);                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,40247,247,30);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,40248,248,30);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";

	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   ��������� sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   ���������  sensor ���������� ��������� ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(10,0);                                                              // ������ ����������� ������� �� ����� XP1 10  (��������� �������� ����������)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
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
//	unsigned int regcount = 0;
	test_MTT_off();                                                                 // ��������� ���� � �������, �������� ����������
	test_MTT_on();                                                                  // �������� ����������� �������, ��������� ���������
//	myFile.println("");
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT �������� ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                                               // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD ������ ������� DCD ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                                               // "Command  HangUp MTT OFF send!"

	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,25);                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,25);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     40252,252,25);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     40253,253,25);                                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,25);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, 40255,255,25);                                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,30);                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,30);                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,30);                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                               // ���������� ������� ������� 60 ��
	resistor(2, 130);                                                               // ���������� ������� ������� 60 ��
	regBank.set(3,1);                                                               // �������� ������ �� ���� ��������� ������ Mic3p
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	//myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                   // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;

	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    40250,250,25);                                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    40251,251,25);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	//measure_vol_min(analog_LineL,     40252,252,25);                              // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal LineL                                    OFF - ";
	//measure_vol_min(analog_LineR,     40253,253,25);                              // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, 40254,254,25);                                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal mag radio                                OFF - ";
	//measure_vol_min(analog_mag_phone, 40255,255,25);                              // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       40256,256,30);                                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, 40257,257,30);                                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, 40258,258,30);                                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

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
	
	
	
	
	
	
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void testGGS()
{



	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio1()
{




	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio2()
{


	UpdateRegs();     
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
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
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
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

	 // 3)  ��������  �� ���������� PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                       // ��������  �� ����������  PTT  MTT (DSR) "Test MTT PTT    (DSR)                                       OFF - ";
		  {
			regcount = regBank.get(40264);                                          // ����� �������� ������  PTT  MTT (DSR)   "Test MTT PTT    (DSR)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40264,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)   "Test MTT PTT    (DSR)                                       OFF - ";
			regBank.set(265,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test MTT PTT    (DSR)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (DSR)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));    // "Test MTT PTT    (DSR)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT PTT    (DSR)                                       OFF - ";
			   }             
		  }

	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = regBank.get(40267);                                          // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40267,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
	delay(300);

	  // 1)  �������� ������� MTT �� ��������� 
	byte i50 = regs_in[0];    
		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = regBank.get(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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

		UpdateRegs(); 
	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ��������� CTS MTT
		  {
			regcount = regBank.get(40265);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40265,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regBank.set(265,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
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

	 // 3)  ��������  �� ���������� PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                       // ��������  �� ����������  PTT  MTT (DSR)
		  {
			regcount = regBank.get(40266);                                          // ����� �������� ������  PTT  MTT (DSR) "Test MTT PTT    (DSR)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40266,regcount);                                            // ����� �������� ������  PTT  MTT (DSR) "Test MTT PTT    (DSR)                                       ON  - ";
			regBank.set(266,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test MTT PTT    (DSR)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (DSR)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));    // "Test MTT PTT    (DSR)                                       ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (DSR)                                       ON  - " ������ �������  - Pass
			   }
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                         // ��������� ��������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = regBank.get(40268);                                          // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(40268,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
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
		int regcount = 0;
		measure_volume(istochnik);                                               // �������� ������� ������� �� ������
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
		}
		if(voltage10 >  porogV)                                                     // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = regBank.get(adr_count);                                  // ����� �������� ������ 
				regcount++;                                                         // ��������� ������� ������ ������ 
				regBank.set(adr_count,regcount);                                    // ����� �������� ������ ������ 
				regBank.set(adr_flagErr,1);                                         // ���������� ���� ������  ������ 
				regBank.set(120,1);                                                 // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.println(regcount);                                           // ��������� �������� ������
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.println(buffer);                                         // "Pass";
				}
			}                                     
}
void measure_vol_max(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                                  // �������� ������� ������� �� ������
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
		}
		
		if(voltage10 <  porogV)                                                     // ��������� ����������� ������
			{
				myFile.print(buffer); 
				regcount = regBank.get(adr_count);                                  // ����� �������� ������ 
				regcount++;                                                         // ��������� ������� ������ ������ 
				regBank.set(adr_count,regcount);                                    // ����� �������� ������ ������ 
				regBank.set(adr_flagErr,1);                                         // ���������� ���� ������  ������ 
				regBank.set(120,1);                                                 // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.println(regcount);                                           // ��������� �������� ������
			}
		else
			{
			if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
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
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS ���
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
	regBank.add(264);                         // ���� ������ "Test MTT PTT    (DSR)                                       OFF - ";
	regBank.add(265);                         // ���� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(266);                         // ���� ������ "Test MTT PTT    (DSR)                                       ON  - ";
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
	regBank.add(40010);  // 
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
	regBank.add(40264);                         // A���� �������� ������ "Test MTT PTT    (DSR)                                       OFF - ";
	regBank.add(40265);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40266);                         // A���� �������� ������ "Test MTT PTT    (DSR)                                       ON  - ";
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





	////regBank.add(40121);  // ����� �������� ������ ������� sensor "��-�����1."    ok!
	////regBank.add(40122);  // ����� �������� ������ ������� sensor "��-�����2."    ok!
	////regBank.add(40123);  // ����� �������� ������ ������� ����������� ������                   "Sensor MTT    XP1- 19 HaSs   OFF  - ";  
	////regBank.add(40124);  // ����� �������� ������ �������                                      "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	////regBank.add(40125);  // ����� �������� ������ �������                                      "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
	////regBank.add(40126);  // ����� �������� ������ �������  
	////regBank.add(40127);  // ����� �������� ������ ������� ��������� ����������� � 2 ���������� "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	////regBank.add(40128);  // ����� �������� ������ ������� ��������� �����������                "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
	////regBank.add(40129);  // ����� �������� ������ ������� ��������� ���������� � 2 ����������  "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 

	////regBank.add(40130);  // ����� �������� ������ ������� ��������� ����������                 "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";  
	////regBank.add(40131);  // ����� �������� ������ ������� sensor ���.                          "Sensor microphone                   XS1 - 6                 OFF - "; 
	////regBank.add(40132);  // ����� �������� ������ ���������� ��������� ��������� �����������   "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - ";  
	////regBank.add(40133);  // ����� �������� ������ ���������� �������������                     "Radioperedacha                                              OFF - ";
	////regBank.add(40134);  // ����� �������� ������ ���������� ��������� ��������� ����������    "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
	////regBank.add(40135);  // ����� �������� ������ ������� PTT ��� ok!
	//regBank.add(40136);  // ����� �������� ������  PTT2 ����. �. ok!
	//regBank.add(40137);  // ����� �������� ������  HangUp  DCD  ok!
	//regBank.add(40138);  // ����� �������� ������  PTT1 ����. �. ok!
	//regBank.add(40139);  // ����� �������� ������ ���������� ��������� ��������� ����������� 

	////regBank.add(130);   // ���� ������ ������� ��������� ����������
	////regBank.add(131);   // ���� ������ ������� sensor ���. ok!
	////regBank.add(132);   // ���� ������ ������� sensor "��C."   ok!
	////regBank.add(133);   // ���� ������ ������� ���������
	////regBank.add(134);   // ���� ������ ������� PTT ���� �.  ok!
	////regBank.add(135);   // ���� ������ ������� PTT ��� ok!
	////regBank.add(136);   // ���� ������  PTT2 ����. �. ok!
	////regBank.add(137);   // ���� ������  HangUp  DCD  ok!
	////regBank.add(138);   // ���� ������  PTT1 ����. �. ok!
	////regBank.add(139);   // ���� ������ ���������� ��������� ��������� �����������


	////regBank.add(40140);  // ����� �������� ������ ���������� PTT ��������� �����������
	////regBank.add(40141);  // ����� �������� ������ �������� ��������� ����������� FrontL
	////regBank.add(40142);  // ����� �������� ������ �������� ��������� ����������� FrontR
	////regBank.add(40143);  // ����� �������� ������ ����������� LineL
	////regBank.add(40144);  // ����� �������� ������ ����������� analog_mag_radio
	////regBank.add(40145);  // ����� �������� ������ ����������� analog_mag_phone
	////regBank.add(40146);  // ����� �������� ������ ����������� analog_ggs
	////regBank.add(40147);  // ����� �������� ������ ����������� analog_gg_radio1
	////regBank.add(40148);  // ����� �������� ������ ����������� analog_gg_radio2
	////regBank.add(40149);  // ����� �������� ������ ����������� Mag phone on


	////regBank.add(40150);  // ����� �������� ������ ���������� PTT ��������� ����������
	////regBank.add(40151);  // ����� �������� ������ �������� ��������� ���������� FrontL
	////regBank.add(40152);  // ����� �������� ������ �������� ��������� ���������� FrontR
	////regBank.add(40153);  // ����� �������� ������ ���������� LineL
	////regBank.add(40154);  // ����� �������� ������ ���������� analog_mag_radio
	////regBank.add(40155);  // ����� �������� ������ ���������� analog_mag_phone
	////regBank.add(40156);  // ����� �������� ������ ���������� analog_ggs
	////regBank.add(40157);  // ����� �������� ������ ���������� analog_gg_radio1
	////regBank.add(40158);  // ����� �������� ������ ���������� analog_gg_radio2
	////regBank.add(40159);  // ����� �������� ������ ���������� Mag phone on

	////regBank.add(40160);  // ����� �������� ������ �������� ��������� ���������� FrontL
	////regBank.add(40161);  // ����� �������� ������ �������� ��������� ���������� FrontR
	////regBank.add(40162);  // ����� �������� ������ �������  LineL ��������� ���������� 
	//regBank.add(40163);  // ����� �������� ������ sensor MTT OFF
	//regBank.add(40164);  // ����� �������� ������ MTT PTT OFF (CTS) 
	//regBank.add(40165);  // ����� �������� ������ MTT PTT OFF (DSR) 
	//regBank.add(40166);  // ����� �������� ������ MTT PTT ON  (CTS) 
	//regBank.add(40167);  // ����� �������� ������ MTT PTT ON  (DSR)
	//regBank.add(40168);  // ����� �������� ������ MTT HangUp OFF (DCD)
	//regBank.add(40169);  // ����� �������� ������ MTT HangUp ON  (DCD)

	//regBank.add(40170);  // ����� �������� ������  sensor MTT ON
	//regBank.add(40171);  // ����� �������� ������ �������  LineL MTT
	//regBank.add(40172);  // ����� �������� ������ analog_ggs ON
	//regBank.add(40173);  // ����� �������� ������ "Command PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	//regBank.add(40174);  // ����� �������� ������ "Command PTT2  OFF tangenta ruchnaja (CTS)      Error! - ";
	//regBank.add(40175);  // ����� �������� ������ "Command PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	//regBank.add(40176);  // ����� �������� ������ "Command PTT2  ON  tangenta ruchnaja (CTS)      Error! - ";
	//regBank.add(40177);  // ����� �������� ������ �������  LineL ��������� ����������� 
	//regBank.add(40178);  // ����� �������� ������ �������  LineR ��������� ����������� 
	//regBank.add(40179);  // ����� �������� ������ �������  LineL ��������� ���������� 

	//regBank.add(40180);  // ����� �������� ������ �������  LineR ��������� ���������� 
	//regBank.add(40181);  // ����� �������� ������ ��������� ��������� �����������
	//regBank.add(40182);  // ����� �������� ������ ��������� ��������� ����������
	//regBank.add(40183);  // ����� �������� ������ ���������� ��������� ��������� ����������
	//regBank.add(40184);  // ����� �������� ������ ���������� ��������� ��������� ����������







	
	//regBank.add(40150);  // ����� �������� ������  Mag phone on
	//regBank.add(40153);  // ����� �������� ������ XP1 - 20  HangUp  DCD
	//regBank.add(40154);  // ����� �������� ������ sensor MTT ON
	//regBank.add(40156);  // ����� �������� ������ ���������� �������������
	//regBank.add(40157);  // ����� �������� ������ ��������� �������������
	//regBank.add(40158);  // ����� �������� ������ ���������� PTT ��������� ����������



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
//	sensor_all_off();
	data_clock_exchange();
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
	prer_Kmerton_On = true;                          // ��������� ���������� �� ��������
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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
}
