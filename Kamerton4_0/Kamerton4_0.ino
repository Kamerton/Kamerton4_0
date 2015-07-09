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

const unsigned int adr_set_time           PROGMEM       = 36;    // ����� ���� ���������

//---------------------------������ ������  ---------------------------------------------------
const char  txt_all_on0[]   PROGMEM            = "";       
const char  txt_all_on1[]   PROGMEM            = " ****** Test sensor ON start! ******"                    ;                           
const char  txt_all_on2[]   PROGMEM            = "Sence MTT                     XP1- 19 HaSs     Error! - ";                         
const char  txt_all_on3[]   PROGMEM            = "Sence MTT                     XP1- 19 HaSs         ";
const char  txt_all_on4[]   PROGMEM            = "ON - Ok!"                                                ;
const char  txt_all_on5[]   PROGMEM            = "Sence Tangenta ruchnaja       XP7 - 2          Error! - ";    
const char  txt_all_on6[]   PROGMEM            = "Sence Tangenta ruchnaja       XP7 - 2              ";    
const char  txt_all_on7[]   PROGMEM            = "Sensor garnitura dispetchera  XP1 - 1  HeS1Ls  Error! - ";         
const char  txt_all_on8[]   PROGMEM            = "Sence tangenta nognaja        XP8 - 2          Error! - ";          
const char  txt_all_on9[]   PROGMEM            = "Sence tang. nogn.             XP8 - 2              ";      
const char  txt_all_on10[]  PROGMEM            = "Sence garnitura dispetchera   XP1 - 1  HeS1Ls      ";        
const char  txt_all_on11[]  PROGMEM            = "Sence garnitura instruktora 2 XP1 - 16 HeS2Rs  Error! - ";      
const char  txt_all_on12[]  PROGMEM            = "Sence garnitura instruktora 2 XP1 - 16 HeS2Rs      ";                         
const char  txt_all_on13[]  PROGMEM            = "Sence microphone              XS1 - 6          Error! - ";
const char  txt_all_on14[]  PROGMEM            = "Sence garnitura instruktora   XP1 - 13 HeS2Ls  Error! - ";
const char  txt_all_on15[]  PROGMEM            = "Sence garnitura instruktora   XP1 - 13 HeS2Ls      ";    
const char  txt_all_on16[]  PROGMEM            = "Sence microphone              XS1 - 6              ";    
const char  txt_all_on17[]  PROGMEM            = "Sence garnitura dispetchera 2 XP1 - 13 HeS2Ls  Error! - ";         
const char  txt_all_on18[]  PROGMEM            = "Sence garnitura dispetchera 2 XP1 - 13 HeS2Ls      ";          
const char  txt_all_on19[]  PROGMEM            = "Microphone instruktora Sw.    XP1 - 12 HeS2e   Error! - ";        
const char  txt_all_on20[]  PROGMEM            = "Microphone instruktora Sw.    XP1 - 12 HeS2e       ";         
const char  txt_all_on21[]  PROGMEM            = "Radioperedacha ON                              Error! - ";          
const char  txt_all_on22[]  PROGMEM            = "Radioperedacha                                     ";        
const char  txt_all_on23[]  PROGMEM            = "Microphone dispetchera ON                      Error! - ";         
const char  txt_all_on24[]  PROGMEM            = "Microphone dispetchera        XP1 10               ";          

const char  txt_all_off0[]   PROGMEM           = "";    
const char  txt_all_off1[]   PROGMEM           = " ****** Test sensor OFF start! ******                   ";                           
const char  txt_all_off2[]   PROGMEM           = "Sence MTT                     XP1- 19 HaSs     Error! - ";                         
const char  txt_all_off3[]   PROGMEM           = "Sence MTT                     XP1- 19 HaSs        ";
const char  txt_all_off4[]   PROGMEM           = "OFF - Ok!"                                               ;                           
const char  txt_all_off5[]   PROGMEM           = "Sence tangenta ruchnaja       XP7 - 2          Error! - ";                         
const char  txt_all_off6[]   PROGMEM           = "Sence tangenta ruchnaja       XP7 - 2             ";
const char  txt_all_off7[]   PROGMEM           = "Sence tangenta nognaja        XP8 - 2          Error! - ";                           
const char  txt_all_off8[]   PROGMEM           = "Sence tangenta nognaja        XP8 - 2             ";                         
const char  txt_all_off9[]   PROGMEM           = "Sence garnitura instruktora 2 XP1- 16 HeS2Rs   Error! - ";
const char  txt_all_off10[]  PROGMEM           = "Sence garnitura instruktora 2 XP1- 16 HeS2Rs      ";                           
const char  txt_all_off11[]  PROGMEM           = "Sence garnitura instruktora   XP1- 13 HeS2Ls   Error! - ";                         
const char  txt_all_off12[]  PROGMEM           = "Sence garnitura instruktora   XP1- 13 HeS2Ls      ";
const char  txt_all_off13[]  PROGMEM           = "Sence garnitura dispetchera 2 XP1- 13 HeS2Ls   Error! - ";                           
const char  txt_all_off14[]  PROGMEM           = "Sence garnitura dispetchera 2 XP1- 13 HeS2Ls      ";                         
const char  txt_all_off15[]  PROGMEM           = "Sence garnitura dispetchera   XP1- 1  HeS1Ls   Error! - ";
const char  txt_all_off16[]  PROGMEM           = "Sence garnitura dispetchera   XP1- 1  HeS1Ls      ";                           
const char  txt_all_off17[]  PROGMEM           = "Sence microphone              XS1 - 6          Error! - ";                         
const char  txt_all_off18[]  PROGMEM           = "Sence microphone              XS1 - 6             ";
const char  txt_all_off19[]  PROGMEM           = "Microphone instruktora Sw.    XP1 12 HeS2e     Error! - ";                           
const char  txt_all_off20[]  PROGMEM           = "Microphone instruktora Sw.    XP1 12 HeS2e        ";                         
const char  txt_all_off21[]  PROGMEM           = "Radioperedacha OFF                             Error! - ";
const char  txt_all_off22[]  PROGMEM           = "Radioperedacha                                    ";                           
const char  txt_all_off23[]  PROGMEM           = "Microphone dispetchera OFF    XP1 10           Error! - ";                         
const char  txt_all_off24[]  PROGMEM           = "Microphone dispetchera                            ";
//**********************************************************************************************************
const char  txt_test_all0[]  PROGMEM           = " ****** Test instruktora start! ******"                  ;
const char  txt_test_all1[]  PROGMEM           = "Signal microphone  instruktora 30mv  ON"                 ;
const char  txt_test_all2[]  PROGMEM           = "Microphone instruktora               ON        Error! - ";
const char  txt_test_all3[]  PROGMEM           = "Microphone instruktora               ON               - Ok!";
const char  txt_test_all4[]  PROGMEM           = "Microphone instruktora signal        ON"                 ;
const char  txt_test_all5[]  PROGMEM           = "";
const char  txt_test_all6[]  PROGMEM           = "";
const char  txt_test_all7[]  PROGMEM           = "";
const char  txt_test_all8[]  PROGMEM           = "";
const char  txt_test_all9[]  PROGMEM           = "";

const char  txt_test_all10[]  PROGMEM          = " ****** Test tangenta nognaja start! ******";
const char  txt_test_all11[]  PROGMEM          = "Komanda sence OFF nognaja ruchnaja send!"                ;
const char  txt_test_all12[]  PROGMEM          = "Komanda PTT1  OFF nognaja ruchnaja send!"                ;
const char  txt_test_all13[]  PROGMEM          = "Komanda sence ON  nognaja ruchnaja send!"                ;
const char  txt_test_all14[]  PROGMEM          = "Komanda PTT1  ON  nognaja ruchnaja send!"                ;
const char  txt_test_all15[]  PROGMEM          = "Komanda PTT1  OFF nognaja ruchnaja (CTS)       Error! - ";
const char  txt_test_all16[]  PROGMEM          = "Komanda PTT1  OFF nognaja ruchnaja (CTS)              - Ok!";
const char  txt_test_all17[]  PROGMEM          = "Komanda PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
const char  txt_test_all18[]  PROGMEM          = "Komanda PTT1  ON  tangenta ruchnaja (CTS)             - Ok!";
const char  txt_test_all19[]  PROGMEM          = "";

const char  txt_test_all20[]  PROGMEM          = " ****** Test dispetchera start! ******"                  ;
const char  txt_test_all21[]  PROGMEM          = "Signal microphone dispetchera 30mv  ON"                  ;
const char  txt_test_all22[]  PROGMEM          = "Microphone dispetchera              ON         Error! - ";
const char  txt_test_all23[]  PROGMEM          = "Microphone dispetchera              ON                - Ok!";
const char  txt_test_all24[]  PROGMEM          = "Microphone dispetchera signal       ON"                  ;
const char  txt_test_all25[]  PROGMEM          = "";
const char  txt_test_all26[]  PROGMEM          = "";
const char  txt_test_all27[]  PROGMEM          = "";
const char  txt_test_all28[]  PROGMEM          = "";
const char  txt_test_all29[]  PROGMEM          = "";

const char  txt_test_all30[]  PROGMEM          = " ****** Test MTT start! ******"                          ;
const char  txt_test_all31[]  PROGMEM          = "Signal microphone MTT 60mv          ON"                  ;
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
const char  txt_test_all49[]  PROGMEM          = "Signal GGS        ON                                  - ";

const char  txt_test_all50[]  PROGMEM          = "Signal FrontL     OFF                                 - ";
const char  txt_test_all51[]  PROGMEM          = "Signal FrontR     OFF                                 - ";
const char  txt_test_all52[]  PROGMEM          = "Signal FrontL     ON                                  - ";
const char  txt_test_all53[]  PROGMEM          = "Signal mag radio  OFF                                 - ";
const char  txt_test_all54[]  PROGMEM          = "Signal mag phone  OFF                                 - ";
const char  txt_test_all55[]  PROGMEM          = "Signal GGS        OFF                                 - ";
const char  txt_test_all56[]  PROGMEM          = "Signal GG Radio1  OFF                                 - ";
const char  txt_test_all57[]  PROGMEM          = "Signal GG Radio2  OFF                                 - ";
const char  txt_test_all58[]  PROGMEM          = "Signal Mag phone  ON                                  - ";
const char  txt_test_all59[]  PROGMEM          = "Signal FrontL, FrontR  ON                             - ";

const char  txt_test_all60[]  PROGMEM          = " ****** Test tangenta ruchnaja start! ******";
const char  txt_test_all61[]  PROGMEM          = "Komanda sence OFF tangenta ruchnaja send!"               ;
const char  txt_test_all62[]  PROGMEM          = "Komanda PTT1  OFF tangenta ruchnaja send!"               ;
const char  txt_test_all63[]  PROGMEM          = "Komanda PTT2  OFF tangenta ruchnaja send!"               ;
const char  txt_test_all64[]  PROGMEM          = "Komanda sence ON  tangenta ruchnaja send!"               ;
const char  txt_test_all65[]  PROGMEM          = "Komanda PTT1  ON  tangenta ruchnaja send!"               ;
const char  txt_test_all66[]  PROGMEM          = "Komanda PTT2  ON  tangenta ruchnaja send!"               ;
const char  txt_test_all67[]  PROGMEM          = "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
const char  txt_test_all68[]  PROGMEM          = "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
const char  txt_test_all69[]  PROGMEM          = "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";

const char  txt_test_all70[]  PROGMEM          = "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
const char  txt_test_all71[]  PROGMEM          = "Komanda PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
const char  txt_test_all72[]  PROGMEM          = "Komanda PTT1  ON  tangenta ruchnaja (CTS)             - Ok!";
const char  txt_test_all73[]  PROGMEM          = "Komanda PTT2  ON  tangenta ruchnaja (DCR)      Error! - ";
const char  txt_test_all74[]  PROGMEM          = "Komanda PTT2  ON  tangenta ruchnaja (DCR)             - Ok!";
const char  txt_test_all75[]  PROGMEM          = "";
const char  txt_test_all76[]  PROGMEM          = "";
const char  txt_test_all77[]  PROGMEM          = "";
const char  txt_test_all78[]  PROGMEM          = "";
const char  txt_test_all79[]  PROGMEM          = "";





//**********************************************************************************************************
const char  txt_instr_off0[]  PROGMEM          = "Komanda sence OFF instruktora   send!"                   ;
const char  txt_instr_off1[]  PROGMEM          = "Komanda sence OFF instruktora 2 send!"                   ;
const char  txt_instr_off2[]  PROGMEM          = "Komanda PTT   OFF instruktora   send!"                   ;
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

const char  txt_mtt_off0[]   PROGMEM           = "Komanda sence OFF MTT            send!"                  ;
const char  txt_mtt_off1[]   PROGMEM           = "Komanda PTT   OFF MTT            send!"                  ;
const char  txt_mtt_off2[]   PROGMEM           = "Komanda       OFF HangUp MTT     send!"                  ;
const char  txt_mtt_off3[]   PROGMEM           = "Komanda sence OFF MTT                          Error! - ";
const char  txt_mtt_off4[]   PROGMEM           = "Komanda sence OFF MTT                                 - Ok!";
const char  txt_mtt_off5[]   PROGMEM           = "Komanda sence OFF microphone                   Error! - ";
const char  txt_mtt_off6[]   PROGMEM           = "Komanda sence OFF microphone                          - Ok!";
const char  txt_mtt_off7[]   PROGMEM           = "Komanda       OFF PTT  MTT (CTS)               Error! - ";
const char  txt_mtt_off8[]   PROGMEM           = "Komanda       OFF PTT  MTT (CTS)                      - Ok!";
const char  txt_mtt_off9[]   PROGMEM           = "Komanda       OFF PTT  MTT (DSR)               Error! - ";
const char  txt_mtt_off10[]  PROGMEM           = "Komanda       OFF PTT  MTT (DSR)                      - Ok!";
const char  txt_mtt_off11[]  PROGMEM           = "Komanda       OFF HangUp MTT                   Error! - ";
const char  txt_mtt_off12[]  PROGMEM           = "Komanda       OFF HangUp MTT                          - Ok!";


const char  txt_mtt_on0[]    PROGMEM           = "Komanda sence ON  MTT            send!"                  ;
const char  txt_mtt_on1[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS) send!"                  ;
const char  txt_mtt_on2[]    PROGMEM           = "Komanda       ON  HangUp MTT     send!"                  ;
const char  txt_mtt_on3[]    PROGMEM           = "Komanda sence ON  MTT                          Error! - ";
const char  txt_mtt_on4[]    PROGMEM           = "Komanda sence ON  MTT                                 - Ok!";
const char  txt_mtt_on5[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS)               Error! - ";
const char  txt_mtt_on6[]    PROGMEM           = "Komanda       ON  PTT  MTT (CTS)                      - Ok!";
const char  txt_mtt_on7[]    PROGMEM           = "Komanda       ON  PTT  MTT (DSR)               Error! - ";
const char  txt_mtt_on8[]    PROGMEM           = "Komanda       ON  PTT  MTT (DSR)                      - Ok!";
const char  txt_mtt_on9[]    PROGMEM           = "Komanda       ON  HangUp MTT                   Error! - ";
const char  txt_mtt_on10[]   PROGMEM           = "Komanda       ON  HangUp MTT                          - Ok!";







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
	txt_mtt_off2,      // "Komanda       OFF HangUp MTT     send!"                  ;
	txt_mtt_off3,      // "Komanda sence OFF MTT                          Error! - ";
	txt_mtt_off4,      // "Komanda sence OFF MTT                                 - Ok!";
	txt_mtt_off5,      // "Komanda sence OFF microphone                   Error! - ";
	txt_mtt_off6,      // "Komanda sence OFF microphone                          - Ok!";
	txt_mtt_off7,      // "Komanda       OFF PTT  MTT (CTS)               Error! - ";
	txt_mtt_off8,      // "Komanda       OFF PTT  MTT (CTS)                      - Ok!";
	txt_mtt_off9,      // "Komanda       OFF PTT  MTT (DSR)               Error! - ";
	txt_mtt_off10,     // "Komanda       OFF PTT  MTT (DSR)                      - Ok!";
	txt_mtt_off11,     // "Komanda       OFF HangUp MTT                   Error! - ";
	txt_mtt_off12      // "Komanda       OFF HangUp MTT                          - Ok!";
};

const char* const table_mtt_on[] PROGMEM = 
{
	txt_mtt_on0,      // "Komanda sence ON  MTT            send!"                  ;
	txt_mtt_on1,      // "Komanda       ON  PTT  MTT (CTS) send!"                  ;
	txt_mtt_on2,      // "Komanda       ON  HangUp MTT     send!"                  ;
	txt_mtt_on3,      // "Komanda sence ON  MTT                          Error! - ";
	txt_mtt_on4,      // "Komanda sence ON  MTT                                 - Ok!";
	txt_mtt_on5,      // "Komanda       ON  PTT  MTT (CTS)               Error! - ";
	txt_mtt_on6,      // "Komanda       ON  PTT  MTT (CTS)                      - Ok!";
	txt_mtt_on7,      // "Komanda       ON  PTT  MTT (DSR)               Error! - ";
	txt_mtt_on8,      // "Komanda       ON  PTT  MTT (DSR)                      - Ok!";
	txt_mtt_on9,      // "Komanda       ON  HangUp MTT                   Error! - ";
	txt_mtt_on10      // "Komanda       ON  HangUp MTT                          - Ok!";
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
	txt_test_all49,     // ""                                                        ;

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
	txt_test_all61,     // "Komanda sence OFF tangenta ruchnaja send!"               ;
	txt_test_all62,     // "Komanda PTT1  OFF tangenta ruchnaja send!"               ;
	txt_test_all63,     // "Komanda PTT2  OFF tangenta ruchnaja send!"               ;
	txt_test_all64,     // "Komanda sence ON  tangenta ruchnaja send!"               ;
	txt_test_all65,     // "Komanda PTT1  ON  tangenta ruchnaja send!"               ;
	txt_test_all66,     // "Komanda PTT2  ON  tangenta ruchnaja send!"               ;
	txt_test_all67,     // "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	txt_test_all68,     // "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
	txt_test_all69,     // "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";

	txt_test_all70,     // "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
	txt_test_all71,     // "Komanda PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	txt_test_all72,     // "Komanda PTT1  ON  tangenta ruchnaja (CTS)             - Ok!";
	txt_test_all73,     // "Komanda PTT2  ON  tangenta ruchnaja (DCR)      Error! - ";
	txt_test_all74,     // "Komanda PTT2  ON  tangenta ruchnaja (DCR)             - Ok!";              ;
	txt_test_all75,     // ""               ;
	txt_test_all76,     // ""               ;
	txt_test_all77,     // "";
	txt_test_all78,     // "";
	txt_test_all79      // "";
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
	

	data_clock_exchange();                               // ��������� �� ���������� �� ����

	file_name_count = i2c_eeprom_read_byte(0x50, adr_file_name_count); // ������� ������� ����� ����� �� ������

	preob_num_str();                                     // ������������ ��� ����� �� ���� � �������� ������
 
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
	//if (test_n!= 0) Serial.println(test_n);
		
	switch (test_n)
	{
		case 1:
			 sence_all_off();                                // ��������� ��� �������
			break;
		case 2:		
			 sence_all_on();                                 // �������� ��� �������
				break;
		case 3:
			 test_instruktora();
				break;
		case 4:				
			 test_dispetchera();          //
				break;
		case 5:
			 test_MTT();                  //
				break;
		case 6:				
			 test_tangR();                //
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
	regBank.set(8,1);     // �������� ������� ��������
	regBank.set(5,0);     // �������� ����������� ���������
	regBank.set(10,0);    // �������� ���������� ���������
	regBank.set(13,0);    // XP8 - 2   Sence �������� ������
	regBank.set(14,0);    // XP8 - 1   PTT     �������� ������
	regBank.set(15,0);    // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);    // XS1 - 6   Sence ����������� ���������
 
	regBank.set(17,0);    // J8-12    XP7 4 PTT2 �������� ������ DSR
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(19,0);    // J8-11     XP7 2 Sence �������� ������
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 �������� ������ CTS
	//------------------------- �� ������������ --------------------
	//regBank.set(21,0);    // XP2-2     Sence "���."  
	//regBank.set(22,0);    // XP5-3     Sence "��C."
	//regBank.set(23,0);    // XP3-3     Sence "��-�����1."
	//regBank.set(24,1);    // XP4-3     Sence "��-�����2."
	//-------------------------------------------------------------------
	regBank.set(25,1);    // XP1- 19 HaSs      Sence ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,0);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,0);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,0);    // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);    // XP1- 1  HeS1Ls    Sence ���������� ��������� ����������

	UpdateRegs(); 
	delay(300);
	UpdateRegs(); 
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];    


	//regBank.add(134);   // ���� �������� ������ ������� PTT ���� �.  ok!
	//regBank.add(135);   // ���� �������� ������ ������� PTT ��� ok!

	//regBank.add(136);   // ���� �������� ������  PTT2 ����. �. ok!
	//regBank.add(137);   // ���� �������� ������  HangUp  DCD  ok!
	//regBank.add(138);   // ���� �������� ������  PTT1 ����. �. ok!
	//regBank.add(139);   // ���� ������ ���������� ��������� ��������� �����������


		if(bitRead(i50,2) != 0)                                         // XP1- 19 HaSs Sence ����������� ������  
		  {
			regcount = regBank.get(40123);                              // ����� �������� ������ Sence ����������� ������  
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ������  
			regBank.set(40123,regcount);                                // ����� �������� ������ Sence ����������� ������  
			regBank.set(123,1);                                         // ���������� ���� ������ Sence ����������� ������  
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[2])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[3])));
			myFile.print(buffer);                                        // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                      //  Sence ����������� ������  OK!
		  }
	
		if(bitRead(i50,3) != 0)                                         // J8-11     XP7 2 Sence �������� ������
		  {
			regcount = regBank.get(40124);                              // ����� �������� ������ Sence �������� ������
			regcount++;                                                 // ��������� ������� ������ Sence �������� ������
			regBank.set(40124,regcount);                                // ����� �������� ������ Sence �������� ������
			regBank.set(124,1);                                         // ���������� ���� ������ Sence �������� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[6])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence �������� ������ OK!
		  }

		if(bitRead(i50,4) != 0)                                         // XP8 - 2   Sence �������� ������
		  {
			regcount = regBank.get(40125);                              // ����� �������� ������ Sence �������� ������
			regcount++;                                                 // ��������� ������� ������  Sence �������� ������
			regBank.set(40125,regcount);                                // ����� �������� ������  Sence �������� ������
			regBank.set(125,1);                                         // ���������� ���� ������ Sence �������� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[7])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[8])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // Sence �������� ������ OK!
		  }

		if(bitRead(i52,1) != 0)                                         // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40127);                              // ����� �������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regBank.set(40127,regcount);                                // ����� �������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regBank.set(127,1);                                         // ���������� ���� ������ Sence ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                         // ���������� ����� ���� ������
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

		if(bitRead(i52,2) != 0)                                         // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40128);                              // ����� �������� ������ Sence ����������� ��������� �����������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ��������� �����������
			regBank.set(40128,regcount);                                // ����� �������� ������ Sence ����������� ��������� ����������� 
			regBank.set(128,1);                                         // ���������� ���� ������ Sence ����������� ��������� ����������� 
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[11])));
			myFile.print(buffer);                                       // "Sensor garnitura instruktora XP1- 13 HeS2Ls  Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[12])));
			myFile.print(buffer);                                       // "Sensor garnitura instruktora XP1- 13 HeS2Ls    "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //   Sence ����������� ��������� ����������� OK!
		  }

		if(bitRead(i52,3) != 0)                                         // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40129);                              // ����� �������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(40129,regcount);                                // ����� �������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(129,1);                                         // ���������� ���� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[13])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[14])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera 2 XP1- 13 HeS2Ls  "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence ���������� ��������� ���������� � 2 ���������� OK!
		  }

		
		if(bitRead(i52,4) != 0)                                         // XP1- 1  HeS1Ls   Sence ���������� ��������� ���������� 
		  {
			regcount = regBank.get(40130);                              // ����� �������� ������ Sence ���������� ��������� ����������
			regcount++;                                                 // ��������� ������� ������ Sence ���������� ��������� ���������� 
			regBank.set(40130,regcount);                                // ����� �������� ������ Sence ���������� ��������� ����������
			regBank.set(130,1);                                         // ���������� ���� ������ Sence ���������� ��������� ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[15])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera XP1- 1  HeS1Ls Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[16])));
			myFile.print(buffer);                                       // "Sensor garnitura dispetchera XP1- 1  HeS1Ls    "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence ���������� ��������� ����������  OK!
		  }

		if(bitRead(i52,5) != 0)                                         // XS1 - 6   Sence ���������� ���������
		  {
			regcount = regBank.get(40131);                              // ����� �������� ������ Sence ����������� ���������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ���������
			regBank.set(40131,regcount);                                // ����� �������� ������ Sence ����������� ���������
			regBank.set(131,1);                                         // ���������� ���� ������ Sence ����������� ���������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[17])));
			myFile.print(buffer);                                       // "Sensor microphone  XS1 - 6 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[18])));
			myFile.print(buffer);                                       // "Sensor microphone  XS1 - 6  "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     //  Sence ���������� ��������� OK!
		  }

		if(bitRead(i53,4) != 0)                                         // ���� RL4 XP1 12  HeS2e   ���������� ��������� �����������
		  {
			regcount = regBank.get(40139);                              // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                 // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40139,regcount);                                // ����� �������� ������ ��������� ��������� �����������
			regBank.set(139,1);                                         // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[19])));
			myFile.print(buffer);                                       // "Microphone instruktora Sw.  XP1 12 HeS2e Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[20])));
			myFile.print(buffer);                                        // "Microphone instruktora Sw.  XP1 12 HeS2e "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // ���������� ��������� ����������� ��!
		  }
	   // --------------------------------  ��������� ������������� ���� �� ��������� ------------------------------
		//if(bitRead(i53,5) != 0)                                         // ���������� �������������
		//  {
		//	regcount = regBank.get(40156);                              // ����� �������� ������ ��������� �������������
		//	regcount++;                                                 // ��������� ������� ������ ��������� �������������
		//	regBank.set(40156,regcount);                                // ����� �������� ������ ��������� �������������
		//	regBank.set(156,1);                                         // ���������� ���� ������ ��������� �������������
		//	regBank.set(120,1);                                         // ���������� ����� ���� ������
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[21])));
		//	myFile.print(buffer);                                // "Radioperedacha OFF  Error! - "
		//	myFile.println(regcount);                                   // 
		//  }
		//else
		//  {
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[22])));
		//	myFile.print(buffer);                                // "Radioperedacha           "
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
		//	myFile.println(buffer);                               // ���������� ������������� ��!
		//  }

		if(bitRead(i53,6) != 0)                                         // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = regBank.get(40155);                              // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                 // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40155,regcount);                                // ����� �������� ������ ��������� ��������� ����������
			regBank.set(155,1);                                         // ���������� ���� ������ ��������� ��������� ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[23])));
			myFile.print(buffer);                                       // "Microphone dispetchera OFF XP1 10    Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[24])));
			myFile.print(buffer);                                       // "Microphone dispetchera   "
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
			myFile.println(buffer);                                     // ���������� ��������� ���������� ��!
		  }


		//-----------------------------------��������, � ������ ������ �� ����������� ------------------------------------------------
	/*	if(bitRead(i50,0) != 0)                                         // XP4-3     Sence "��-�����2."
		  {
			regcount = regBank.get(40122);                              // ����� �������� ������ Sence "��-�����2."
			regcount++;                                                 // ��������� ������� ������ Sence "��-�����2."
			regBank.set(40122,regcount);                                // ����� �������� ������ Sence "��-�����2."
			regBank.set(122,1);                                         // ���������� ���� ������ Sence "��-�����2."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence GG Radio 2 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 2 ");                          // 
			myFile.println("OFF - Ok!");                                //  Sence "��-�����2." OK!
		  }

		if(bitRead(i50,1) != 0)                                         // XP3-3     Sence "��-�����1."
		  {
			regcount = regBank.get(40121);                              // ����� �������� ������  Sence "��-�����1."
			regcount++;                                                 // ��������� ������� ������  Sence "��-�����1."
			regBank.set(40121,regcount);                                // ����� �������� ������  Sence "��-�����1."
			regBank.set(121,1);                                         // ���������� ���� ������  Sence "��-�����1."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence GG Radio 1 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 1 ");                          // 
			myFile.println("OFF - Ok!");                                //   Sence "��-�����1." OK!
		  }
	if(bitRead(i52,0) != 0)                                             // XP2-2 Sence "���."  
		  {
			regcount = regBank.get(40126);                              // ����� �������� ������ Sence "���."  
			regcount++;                                                 // ��������� ������� ������ Sence "���."  
			regBank.set(40126,regcount);                                // ����� �������� ������ Sence  "���."  
			regBank.set(126,1);                                         // ���������� ���� ������ Sence "���."  
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence MAG XP2-2  Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence MAG XP2-2 ");                           // 
			myFile.println("OFF - Ok!");                                //  Sence "���."  
		  }
	
		if(bitRead(i52,6) != 0)                                         // XP5-3     Sence "��C."
		  {
			regcount = regBank.get(40132);                              // ����� �������� ������ Sence "��C."
			regcount++;                                                 // ��������� ������� ������ Sence "��C."
			regBank.set(40132,regcount);                                // ����� �������� ������ Sence "��C."
			regBank.set(132,1);                                         // ���������� ���� ������ Sence "��C."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sensor GGS  XP5-3 Error! - ");                // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sensor GGS  XP5-3 Error! - ");                                      // 
			myFile.println("OFF - Ok!");                                //  Sence "��C." OK
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
	regBank.set(8,1);     // �������� ������� ��������
	regBank.set(5,1);     // �������� ����������� ��������
	regBank.set(10,1);    // �������� ���������� ��������
	regBank.set(13,1);    // XP8 - 2   Sence �������� ������
	//regBank.set(14,1);    // XP8 - 1   PTT     �������� ������
	//regBank.set(15,1);    // XS1 - 5   PTT ��� CTS
	regBank.set(16,1);    // XS1 - 6   Sence ����������� ���������
 
	//regBank.set(17,1);    // J8-12    XP7 4 PTT2 �������� ������ DSR
	//regBank.set(18,1);    // XP1 - 20  HangUp  DCD
	regBank.set(19,1);    // J8-11     XP7 2 Sence �������� ������
	//regBank.set(20,1);    // J8-23     XP7 1 PTT1 �������� ������ CTS
	//------------------------- �� ������������ --------------------
	//regBank.set(21,1);    // XP2-2     Sence "���."  
	//regBank.set(22,1);    // XP5-3     Sence "��C."
	//regBank.set(23,1);    // XP3-3     Sence "��-�����1."
	//regBank.set(24,1);    // XP4-3     Sence "��-�����2."
	//-------------------------------------------------------------------
	regBank.set(25,0);    // XP1- 19 HaSs      Sence ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
//	regBank.set(26,1);    // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,1);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
//	regBank.set(28,1);    // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,1);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
//	regBank.set(30,1);    // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,1);    // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);    // XP1- 1  HeS1Ls    Sence ���������� ��������� ����������

	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
	//regBank.add(134);   // ���� �������� ������ ������� PTT ���� �.  ok!
	//regBank.add(135);   // ���� �������� ������ ������� PTT ��� ok!
	//regBank.add(136);   // ���� �������� ������  PTT2 ����. �. ok!
	//regBank.add(137);   // ���� �������� ������  HangUp  DCD  ok!
	//regBank.add(138);   // ���� �������� ������  PTT1 ����. �. ok!
	//regBank.add(139);   // ���� ������ ���������� ��������� ��������� �����������

		if(bitRead(regs_in[0],2) == 0)                                     // XP1- 19 HaSs Sence ����������� ������  
		  {
			regcount = regBank.get(40123);                                 // ����� �������� ������ Sence ����������� ������  
			regcount++;                                                    // ��������� ������� ������ Sence ����������� ������  
			regBank.set(40123,regcount);                                   // ����� �������� ������ Sence ����������� ������  
			regBank.set(123,1);                                            // ���������� ���� ������ Sence ����������� ������  
			regBank.set(120,1);                                            // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[2])));
			myFile.print(buffer);                                          // 
			myFile.println(regcount);                                      // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[3])));
			myFile.print(buffer);                                  // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                //  Sence ����������� ������  OK!
		  }
  
		if(bitRead(regs_in[0],3) == 0)                                  // J8-11     XP7 2 Sence �������� ������
		  {
			regcount = regBank.get(40124);                              // ����� �������� ������ Sence �������� ������
			regcount++;                                                 // ��������� ������� ������ Sence �������� ������
			regBank.set(40124,regcount);                                // ����� �������� ������ Sence �������� ������
			regBank.set(124,1);                                         // ���������� ���� ������ Sence �������� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[5])));
			myFile.print(buffer);               // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[6])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence �������� ������ OK!
		  }

		if(bitRead(regs_in[0],4) == 0)                                  // XP8 - 2   Sence �������� ������
		  {
			regcount = regBank.get(40125);                              // ����� �������� ������ Sence �������� ������
			regcount++;                                                 // ��������� ������� ������  Sence �������� ������
			regBank.set(40125,regcount);                                // ����� �������� ������  Sence �������� ������
			regBank.set(125,1);                                         // ���������� ���� ������ Sence �������� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[8])));
			myFile.print(buffer);        // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[9])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 // Sence �������� ������ OK!
		  }


		if(bitRead(regs_in[2],1) == 0)                                  // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40127);                              // ����� �������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regBank.set(40127,regcount);                                // ����� �������� ������ Sence ����������� ��������� ����������� � 2 ����������
			regBank.set(127,1);                                         // ���������� ���� ������ Sence ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                         // ���������� ����� ���� ������
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

		if(bitRead(regs_in[2],2) == 0)                                  // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40128);                              // ����� �������� ������ Sence ����������� ��������� �����������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ��������� �����������
			regBank.set(40128,regcount);                                // ����� �������� ������ Sence ����������� ��������� ����������� 
			regBank.set(128,1);                                         // ���������� ���� ������ Sence ����������� ��������� ����������� 
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[14])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[15])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //   Sence ����������� ��������� ����������� OK!
		  }

		if(bitRead(regs_in[2],3) == 0)                                  // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40129);                              // ����� �������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(40129,regcount);                                // ����� �������� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(129,1);                                         // ���������� ���� ������ Sence ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[17])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[18])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence ���������� ��������� ���������� � 2 ���������� OK!
		  }

		if(bitRead(regs_in[2],4) == 0)                                  // XP1- 1  HeS1Ls  Sence ���������� ��������� ����������
		  {
			regcount = regBank.get(40130);                              // ����� �������� ������ Sence ���������� ��������� ����������
			regcount++;                                                 // ��������� ������� ������ Sence ���������� ��������� ����������
			regBank.set(40130,regcount);                                // ����� �������� ������ Sence ���������� ��������� ���������� 
			regBank.set(130,1);                                         // ���������� ���� ������ Sence ���������� ��������� ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[7])));
			myFile.print(buffer);                          // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[10])));
			myFile.print(buffer);                                      // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                 //  Sence ���������� ��������� ���������� OK!
		  }

		if(bitRead(regs_in[2],5) == 0)                                  // XS1 - 6   Sence ����������� ���������
		  {
			regcount = regBank.get(40131);                              // ����� �������� ������ Sence ����������� ���������
			regcount++;                                                 // ��������� ������� ������ Sence ����������� ���������
			regBank.set(40131,regcount);                                // ����� �������� ������ Sence ����������� ���������
			regBank.set(131,1);                                         // ���������� ���� ������ Sence ����������� ���������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[13])));
			myFile.print(buffer);                                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[16])));
			myFile.print(buffer);                                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                //  Sence ����������� ��������� OK!
		  }
	
		if(bitRead(regs_in[3],4) == 0)                                  // ���� RL4 XP1 12  HeS2e   ��������� ��������� ��������� �����������
		  {
			regcount = regBank.get(40139);                              // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                 // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40139,regcount);                                // ����� �������� ������ ��������� ��������� �����������
			regBank.set(139,1);                                         // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[19])));
			myFile.print(buffer);                                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[20])));
			myFile.print(buffer);                                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                // ��������� ��������� ����������� ��!
		  }
		// --------------------------------  ��������� ������������� ���� �� ��������� ------------------------------
		//if(bitRead(regs_in[3],5) == 0)                                  // ��������� �������������
		//  {
		//	regcount = regBank.get(40156);                              // ����� �������� ������ ��������� �������������
		//	regcount++;                                                 // ��������� ������� ������ ��������� �������������
		//	regBank.set(40156,regcount);                                // ����� �������� ������ ��������� �������������
		//	regBank.set(156,1);                                         // ���������� ���� ������ ��������� �������������
		//	regBank.set(120,1);                                         // ���������� ����� ���� ������
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[21])));
		//	myFile.print(buffer);                                 // 
		//	myFile.println(regcount);                                   // 
		//  }
		//else
		//  {
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[22])));
		//	myFile.print(buffer);                                 // 
		//	strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
		//	myFile.println(buffer);                                // ��������� ������������� ��!
		//  }
	

	   if(bitRead(regs_in[3],6) == 0)                                   // ���� RL9 XP1 10 ��������� ��������� ����������
		  {
			regcount = regBank.get(40155);                              // ����� �������� ������ ��������� �������������
			regcount++;                                                 // ��������� ������� ������ ��������� �������������
			regBank.set(40155,regcount);                                // ����� �������� ������ ��������� �������������
			regBank.set(155,1);                                         // ���������� ���� ������ ��������� �������������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[23])));
			myFile.print(buffer);                                       // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[24])));
			myFile.print(buffer);                                       // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                     // ��������� ������������� ��!
		  }
	
		//-----------------------------------�������� ------------------------------------------------------------------
	/*	if(bitRead(regs_in[0],0) == 0)                                  // // XP4-3     Sence "��-�����2."
		  {
			regcount = regBank.get(40122);                              // ����� �������� ������ Sence "��-�����2."
			regcount++;                                                 // ��������� ������� ������ Sence "��-�����2."
			regBank.set(40122,regcount);                                // ����� �������� ������ Sence "��-�����2."
			regBank.set(122,1);                                         // ���������� ���� ������ Sence "��-�����2."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence GG Radio 2 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 2 ");                          // 
			myFile.println(txt_all_on4);                                 //  Sence "��-�����2." OK!
		  }

		if(bitRead(regs_in[0],1) == 0)                                  // XP3-3     Sence "��-�����1."
		  {
			regcount = regBank.get(40121);                              // ����� �������� ������  Sence "��-�����1."
			regcount++;                                                 // ��������� ������� ������  Sence "��-�����1."
			regBank.set(40121,regcount);                                // ����� �������� ������  Sence "��-�����1."
			regBank.set(121,1);                                         // ���������� ���� ������  Sence "��-�����1."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence GG Radio 1 Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence GG Radio 1 ");                          // 
			myFile.println(txt_all_on4);                                 //   Sence "��-�����1." OK!
		  }

		if(bitRead(regs_in[2],0) == 0)                                  // XP2-2 Sence "���."  
		  {
			regcount = regBank.get(40126);                              // ����� �������� ������ Sence "���."  
			regcount++;                                                 // ��������� ������� ������ Sence "���."  
			regBank.set(40126,regcount);                                // ����� �������� ������ Sence  "���."  
			regBank.set(126,1);                                         // ���������� ���� ������ Sence "���."  
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sence MAG XP2-2  Error! - ");                 // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sence MAG XP2-2 ");                           // 
			myFile.println(txt_all_on4);                                 //  Sence "���."  
		  }
		if(bitRead(regs_in[2],6) == 0)                                  // XP5-3     Sence "��C."
		  {
			regcount = regBank.get(40132);                              // ����� �������� ������ Sence "��C."
			regcount++;                                                 // ��������� ������� ������ Sence "��C."
			regBank.set(40132,regcount);                                // ����� �������� ������ Sence "��C."
			regBank.set(132,1);                                         // ���������� ���� ������ Sence "��C."
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			myFile.print("Sensor GGS  XP5-3 Error! - ");                // 
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			myFile.print("Sensor GGS  XP5-3 Error! - ");                                      // 
			myFile.println(txt_all_on4);                                //  Sence "��C." OK
		  }
  */

	regBank.set(5,0);     // �������� ����������� ���������
	regBank.set(10,0);    // �������� ���������� ���������
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
	test_instr_off();                                                    // ��������� ���� � �������, �������� ����������
	test_instr_on();                                                     // �������� ����������� �������, ��������� ���������
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,40141,141,25);                         // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,40142,142,25);                         // �������� ������� ������� �� ������ FrontR 
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                     // ���������� ������� ������� 30 ��
	resistor(2, 30);                                                     // ���������� ������� ������� 30 ��
	regBank.set(2,1);                                                    // ������ ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();                                                        // ��������� �������
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[1])));
	myFile.println(buffer);                                              // "Signal microphone  instruktora 30mv  ON"
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                      // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                      // �������� ������� ������� �� ������ FrontR 
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                      // �������� ������� ������� �� ������ mag radio  
	measure_vol_min(analog_mag_phone,40145,145,25);                      // �������� ������� ������� �� ������ mag phone
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40146,146,30);                      // �������� ������� ������� �� ������ GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                      // �������� ������� ������� �� ������ GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                      // �������� ������� ������� �� ������ GG Radio2

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                  //
	regBank.set(5,1);                                                    // ������ ����������� ������� �� ����� 12 ��1 HeS2e (�������� ��������)
	regBank.set(28,1);                                                   // XP1- 15 HeS2PTT �������� PTT �����������
	regBank.set(16,0);                                                   // ������ ��������� ���������
	regBank.set(15,0);                                                   // ��� ��������� ���������
	regBank.set(29,1);                                                   // ��� XP1- 13 HeS2Ls ������  ��� ���� ����������� ��������� ����������� 
	UpdateRegs();                                                        // 
	delay(200);                                                          //
	byte i5 = regs_in[3];                                                // 
		if(bitRead(i5,4) == 0)                                           // ��������  ��������� ��������� �����������
		  {
			regcount = regBank.get(40151);                               // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                  // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40151,regcount);                                 // ����� �������� ������ ��������� ��������� �����������
			regBank.set(151,1);                                          // ���������� ���� ������
			regBank.set(120,1);                                          // ���������� ����� ���� ������
			resistor(1, 255);                                            // ���������� ������� ������� � �������� ��������e
			resistor(2, 255);                                            // ���������� ������� ������� � �������� ��������e
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[2])));
			myFile.print(buffer);                                        // "Microphone instruktora ON  Error! - "
			myFile.println(regcount);                                    // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[3])));
			myFile.println(buffer);                                     //"Microphone instruktora  ON - Ok!" ��������� ����������� ���������
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[4])));
	myFile.println(buffer);                                             // "Microphone instruktora signal ON"  �������� ������ ����� �� ���� ��������� �����������
	delay(20);
	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40143,143,200);                    // �������� ������� ������� �� ������ LineL
	measure_vol_max(analog_mag_phone,40150,150,200);                    // �������� ������� ������� �� ������ mag phone
   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                     // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                     // �������� ������� ������� �� ������ FrontR 
	measure_vol_min(analog_ggs,      40146,146,30);                     // �������� ������� ������� �� ������ GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                     // �������� ������� ������� �� ������ GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                     // �������� ������� ������� �� ������ GG Radio2

	regBank.set(29,0);                                                  // XP1- 13 HeS2Ls  ��������� ������ �����������
	regBank.set(27,0);                                                  // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                  // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(15,0);                                                  // ��� ��������� ���������
	regBank.set(5,0);                                                   // ������ ����������� ������� �� ����� 12 ��1 HeS2e (��������� �������� �����������)
	regBank.set(28,0);                                                  // XP1- 15 HeS2Ls ��������� PTT �����������
	UpdateRegs();     

	regBank.set(adr_control_command,0);                                 // ��������� ���������    
	delay(100);
}
void test_dispetchera()
 {
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[20])));
	myFile.println(buffer);                                               // "Test dispetchera start!"
	myFile.println();
	file_print_date();
	myFile.println("");
	unsigned int regcount = 0;
	test_disp_off();                                                     // ��������� ���� � �������, �������� ����������
	test_disp_on();                                                      // �������� ����������� �������, ��������� ���������
	myFile.println("");
	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,40161,160,25);                         // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,40162,161,25);                         // �������� ������� ������� �� ������ FrontR 
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 30);                                                     // ���������� ������� ������� 30 ��
	resistor(2, 30);                                                     // ���������� ������� ������� 30 ��
	regBank.set(1,1);                                                    // ������ ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();                                                        // ��������� �������
	delay(200);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[21])));      //   
	myFile.println(buffer);                                              // "Signal microphone   dispetchera 30mv  ON"
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                      // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                      // �������� ������� ������� �� ������ FrontR 
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                      // �������� ������� ������� �� ������ mag radio  
	measure_vol_min(analog_mag_phone,40145,145,25);                      // �������� ������� ������� �� ������ mag phone
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      40146,146,30);                      // �������� ������� ������� �� ������ GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                      // �������� ������� ������� �� ������ GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                      // �������� ������� ������� �� ������ GG Radio2

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
	myFile.println("");                                                  //
	regBank.set(10,1);                                                   // ������ ����������� ������� �� ����� XP1 10 ��������� ��������� ����������
	regBank.set(30,1);                                                   // XP1- 6  HeS1PTT   �������� PTT ����������
	regBank.set(16,0);                                                   // ������ ��������� ���������
	regBank.set(15,0);                                                   // ��� ��������� ���������
	regBank.set(31,1);                                                   // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                   // XP1- 1  HeS1Ls    Sence ���������� ��������� ����������

	UpdateRegs();                                                        // 
	delay(200);                                                          //
	byte i5 = regs_in[3];                                                // 
		if(bitRead(i5,6) == 0)                                           // ��������  ��������� ��������� ����������
		  {
			regcount = regBank.get(40151);                               // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                  // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40151,regcount);                                 // ����� �������� ������ ��������� ��������� ����������
			regBank.set(151,1);                                          // ���������� ���� ������
			regBank.set(120,1);                                          // ���������� ����� ���� ������
			resistor(1, 255);                                            // ���������� ������� ������� � �������� ��������e
			resistor(2, 255);                                            // ���������� ������� ������� � �������� ��������e
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[22])));
			myFile.print(buffer);                                        // "Microphone dispetchera ON  Error! - "
			myFile.println(regcount);                                    // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[23])));
			myFile.println(buffer);                                     //"Microphone dispetchera  ON - Ok!" �������� ���������� ���������
			delay(20);
		  }
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[24])));     // "Microphone dispetchera signal ON" 
	myFile.println(buffer);                                             // "Microphone dispetchera signal ON"  �������� ������ ����� �� ���� ��������� ����������
	delay(20);
	//++++++++++++++++++++++++++++++++++ ��������� ������� ������� �� ������ FrontL    ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    40143,143,200);                    // �������� ������� ������� �� ������ LineL
	measure_vol_max(analog_mag_phone,40150,150,200);                    // �������� ������� ������� �� ������ mag phone
   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                     // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                     // �������� ������� ������� �� ������ FrontR 
	measure_vol_min(analog_ggs,      40146,146,30);                     // �������� ������� ������� �� ������ GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                     // �������� ������� ������� �� ������ GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                     // �������� ������� ������� �� ������ GG Radio2
	regBank.set(31,0);                                                  // XP1- 5  HeS1Rs   ��������� Sence ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                  // XP1- 1  HeS1Ls   ���������  Sence ���������� ��������� ����������
	regBank.set(15,0);                                                  // ��� ��������� ���������
	regBank.set(10,0);                                                  // ������ ����������� ������� �� ����� XP1 10  (��������� �������� ����������)
	regBank.set(30,0);                                                  // XP1- 6  HeS1PTT   ��������� PTT ����������
	regBank.set(28,0);                                                  // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	UpdateRegs();     
	regBank.set(adr_control_command,0);                                 // ��������� ���������    
	delay(100);
 }
void test_MTT()
{
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[30])));
	myFile.println(buffer);                                               // "Test MTT start!"
	myFile.println();
	file_print_date();
	myFile.println("");
//	unsigned int regcount = 0;
	test_MTT_off();                                                       // ��������� ���� � �������, �������� ����������
	test_MTT_on();                                                        // �������� ����������� �������, ��������� ���������
	myFile.println("");
	regBank.set(25,0);                                                    //  XP1- 19 HaSs  Sence ����������� ������    MTT ��������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[0])));
	myFile.println(buffer);                                               // "Komanda sence ON MTT  send!         
	regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD ������ ��������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[2])));
	myFile.println(buffer);                                               // "Komanda  HangUp MTT OFF send!"

	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                       // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                       // �������� ������� ������� �� ������ FrontR 
	measure_vol_min(analog_mag_phone,40145,145,25);                       // �������� ������� ������� �� ������ phone  
	measure_vol_min(analog_ggs ,     40146,146,25);                       // �������� ������� ������� �� ������ GGS 
	measure_vol_min(analog_mag_radio,40144,144,25);                       // �������� ������� ������� �� ������ mag radio  
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, 130);                                                     // ���������� ������� ������� 60 ��
	resistor(2, 130);                                                     // ���������� ������� ������� 60 ��
	regBank.set(3,1);                                                     // �������� ������ �� ���� ��������� ������ Mic3p
	UpdateRegs();                                                         // ��������� �������
	delay(400);
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[31])));       //   
	myFile.println(buffer);                                               // "Signal microphone   MTT 60mv  ON"

	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   40141,141,25);                       // �������� ������� ������� �� ������ FrontL   
	measure_vol_min(analog_FrontR,   40142,142,25);                       // �������� ������� ������� �� ������ FrontR 
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ����� Radio +++++++++++++++++++++++++++
	measure_vol_min(analog_mag_radio,40144,144,25);                       // �������� ������� ������� �� ������ mag radio  
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++

	measure_vol_min(analog_ggs,      40146,146,35);                       // �������� ������� ������� �� ������ GGS
	measure_vol_min(analog_gg_radio1,40147,147,30);                       // �������� ������� ������� �� ������ GG Radio1
	measure_vol_min(analog_gg_radio2,40148,148,30);                       // �������� ������� ������� �� ������ GG Radio2

	// ++++++++++++++++++++++++++++++++++ ��������� ������� �������  ++++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineR,    40171,171,35);                       // �������� ������� ������� �� ������ LineL
	measure_vol_max(analog_mag_phone,40150,150,90);                       // �������� ������� ������� �� ������ mag phone
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                     // ��������� ������ �� ���� ��������� ������ Mic3p
	regBank.set(6,1);                                                     // ���� RL5. ������ ���� Front L, Front R
	UpdateRegs();                                                         // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[59])));       //   
	myFile.println(buffer);                                               // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,      40146,146,35);                       // �������� ������� ������� �� ������ GGS
	regBank.set(18,1);                                                    // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                         // ��������� �������
	delay(200);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[2])));
	myFile.println(buffer);                                               // "Komanda  HangUp MTT ON send!"
	measure_vol_max(analog_ggs,      40172,172,120);                      // �������� ������� ������� �� ������ mag phone
	regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD ON  �������� ������
	regBank.set(26,0);                                                    // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
	UpdateRegs();                                                         // ��������� �������
	
	delay(200);
}
void test_tangR()
{
	unsigned int regcount = 0;
	myFile.println(""); 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[60])));
	myFile.println(buffer);                                                 // "Test tangenta ruchnaja start!"
	myFile.println();
	file_print_date();
	myFile.println("");
	regBank.set(17,0);                                                      // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                      // J8-11     XP7 2 Sence �������� ������
	regBank.set(20,0);                                                      // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                           // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[61])));
	myFile.println(buffer);                                                 // "Komanda sence OFF tangenta ruchnaja send!"  
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[62])));
	myFile.println(buffer);                                                 // "Komanda PTT1  OFF tangenta ruchnaja send!"   
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[63])));
	myFile.println(buffer);                                                 // "Komanda PTT2  OFF tangenta ruchnaja send!" 
	byte i50 = regs_in[0];    

	if(bitRead(i50,3) != 0)                                                 // J8-11     XP7 2 Sence �������� ������
		{
		regcount = regBank.get(40124);                                      // ����� �������� ������ Sence �������� ������
		regcount++;                                                         // ��������� ������� ������ Sence �������� ������
		regBank.set(40124,regcount);                                        // ����� �������� ������ Sence �������� ������
		regBank.set(124,1);                                                 // ���������� ���� ������ Sence �������� ������
		regBank.set(120,1);                                                 // ���������� ����� ���� ������
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[5])));         //
		myFile.print(buffer);                                               // 
		myFile.println(regcount);                                           // 
		}
	else
		{
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[6])));
		myFile.print(buffer);                                               // 
		strcpy_P(buffer, (char*)pgm_read_word(&(string_table[4])));
		myFile.println(buffer);                                             //  Sence �������� ������ OK!
		}

	 UpdateRegs(); 
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                               // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS
		  {
			 regcount = regBank.get(40173);                                 // ����� �������� ������ PTT  MTT (CTS)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40173,regcount);                                   // ����� �������� ������ PTT  MTT (CTS)
			 regBank.set(173,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[67])));
			 myFile.print(buffer);                                          // "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";  
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[68])));// "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
			 myFile.println(buffer);                                        // "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
		  }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                               // ��������  �� ����������  PTT2 �������� ������ (DSR)
		  {
			 regcount = regBank.get(40174);                                 // ����� �������� ������  PTT  MTT (DSR)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40174,regcount);                                   // ����� �������� ������  PTT  MTT (DSR)
			 regBank.set(174,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[69])));// "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.print(buffer);                                          // "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[70])));// "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
			 myFile.println(buffer);                                        // "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
		  }

	regBank.set(19,1);    // J8-11     XP7 2 Sence �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[64])));
	myFile.println(buffer);                                                 // "Komanda sence OFF tangenta ruchnaja send!"  
	regBank.set(17,1);    // J8-12     XP7 4 PTT2 �������� ������ DSR


	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[65])));
	myFile.println(buffer);                                                 // "Komanda PTT1  OFF tangenta ruchnaja send!"   
	regBank.set(20,1);    // J8-23     XP7 1 PTT1 �������� ������ CTS

	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[66])));
	myFile.println(buffer);                                                 // "Komanda PTT2  OFF tangenta ruchnaja send!" 

	UpdateRegs();                                                           // ��������� �������
	delay(400);

			if(bitRead(regs_in[0],3) == 0)                                  // J8-11     XP7 2 Sence �������� ������
		  {
			regcount = regBank.get(40124);                                  // ����� �������� ������ Sence �������� ������
			regcount++;                                                     // ��������� ������� ������ Sence �������� ������
			regBank.set(40124,regcount);                                    // ����� �������� ������ Sence �������� ������
			regBank.set(124,1);                                             // ���������� ���� ������ Sence �������� ������
			regBank.set(120,1);                                             // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[5])));
			myFile.print(buffer);                                           // 
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[6])));
			myFile.print(buffer);                 // 
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_On[4])));
			myFile.println(buffer);                                        //  Sence �������� ������ OK!
		  }
	 UpdateRegs(); 
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                               // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS
		  {
			 regcount = regBank.get(40175);                                 // ����� �������� ������ PTT  MTT (CTS)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40175,regcount);                                   // ����� �������� ������ PTT  MTT (CTS)
			 regBank.set(175,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[71])));
			 myFile.print(buffer);                                          // "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";  
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[72])));// "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
			 myFile.println(buffer);                                        // "Komanda PTT1  OFF tangenta ruchnaja (CTS)             - Ok!";
		  }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                               // ��������  �� ����������  PTT2 �������� ������ (DSR)
		  {
			 regcount = regBank.get(40176);                                 // ����� �������� ������  PTT  MTT (DSR)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40176,regcount);                                   // ����� �������� ������  PTT  MTT (DSR)
			 regBank.set(176,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[73])));// "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.print(buffer);                                          // "Komanda PTT2  OFF tangenta ruchnaja (DCR)      Error! - ";
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[74])));// "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
			 myFile.println(buffer);                                        // "Komanda PTT2  OFF tangenta ruchnaja (DCR)             - Ok!";
		  }
	regBank.set(17,0);                                                      // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                      // J8-11     XP7 2 Sence �������� ������
	regBank.set(20,0);                                                      // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                           // ��������� �������
	delay(100);
}
void test_tangN()
{

}

void test_instr_off()
{
	  unsigned int regcount = 0;
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[0])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF instruktora   send!"
	  regBank.set(29,0);                                                // XP1- 13 HeS2Ls  ��������� ������ �����������
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[1])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF instruktora 2 send!"
	  regBank.set(27,0);                                                // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	  regBank.set(16,0);                                                // XS1 - 6   Sence ���
	  regBank.set(1,0);                                                 // ���� RL0 ����
	  regBank.set(2,0);                                                 // ���� RL1 ����
	  regBank.set(3,0);                                                 // ���� RL2 ����
	  regBank.set(4,0);                                                 // ���� RL3 ����  LFE  "���."
	  regBank.set(5,0);                                                 // ���� RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                 // ���� RL5 ����
	  regBank.set(7,0);                                                 // ���� RL6 ����
	  regBank.set(9,0);                                                 // ���� RL8 ���� �� ��������
	  regBank.set(10,0);                                                // ���� RL9 XP1 10
	  regBank.set(28,0);                                                // XP1- 15 HeS2Ls ��������� PTT �����������
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[2])));
	  myFile.println(buffer);                                           // "Komanda PTT instruktora OFF      send!"

	  UpdateRegs();                                                     // ��������� ������� ���������� ��������
	  delay(400);                                                       // 
	 
	  // 1)  �������� ������� �� ���������� ��������� ����������� 2 ����������
	  byte i5 = regs_in[2];                                             // 
		if(bitRead(i5,1) > 0)                                           // ��������  ����� �� ���������� ��������� ����������� 2 ����������
		  {
			regcount = regBank.get(40127);                              // ����� �������� ������ ������� ��������� ����������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������
			regBank.set(40127,regcount);                                // ����� �������� ������ ������� ��������� ����������� � 2 ����������
			regBank.set(127,1);                                         // ���������� ���� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			resistor(1, 255);                                           // ���������� ������� ������� � �������� ���������
			resistor(2, 255);                                           // ���������� ������� ������� � �������� ���������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[3])));
			myFile.print(buffer);                                       // "Komanda sensor OFF instruktora 2 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[4])));
			myFile.println(buffer);                                     // "Komanda sensor OFF instruktora 2 - Ok!"
		  }

	  // 2)  �������� ������� �� ���������� ��������� �����������

		if(bitRead(i5,2) > 0)                                           // ��������  ����� �� ���������� ��������� ����������� 
		  {
			 regcount = regBank.get(40128);                             // ����� �������� ������ ������� ��������� �����������
			 regcount++;                                                // ��������� ������� ������
			 regBank.set(40128,regcount);                               // ����� �������� ������ ������� ��������� �����������
			 regBank.set(128,1);                                        // ���������� ���� ������
			 regBank.set(120,1);                                        // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[5])));
			 myFile.print(buffer);                                      // "Komanda sensor OFF instruktora  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[6])));
			 myFile.println(buffer);                                    // "Komanda sensor OFF instruktora   - Ok!"
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i5,5) > 0)                                           // ��������  ����� �� ���������� ���������
		  {
			 regcount = regBank.get(40149);                             // ����� �������� ������ ������� ��������� 
			 regcount++;                                                // ��������� ������� ������
			 regBank.set(40149,regcount);                               // ����� �������� ������ ������� ���������
			 regBank.set(149,1);                                        // ���������� ���� ������
			 regBank.set(120,1);                                        // ���������� ����� ���� ������
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
	   if(regBank.get(adr_reg_ind_CTS)!= 0)                                       // ��������� ��������� PTT �����������   CTS
		  {
			regcount = regBank.get(40140);                              // ����� �������� ������ ���������� PTT ��������� �����������
			regcount++;                                                 // ��������� ������� ������
			regBank.set(40140,regcount);                                // ����� �������� ������ ���������� PTT ��������� �����������
			regBank.set(140,1);                                         // ���������� ���� ������ ���������� PTT ��������� �����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_off[9])));
			myFile.print(buffer);                                       // "Komanda PTT instruktora(CTS) OFF Error! - " �������� �� ���������� 
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
	regBank.set(29,1);                                                  // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[1])));
	myFile.println(buffer);                                             // "Komanda sensor ON instruktora 2  send!"
	regBank.set(27,1);                                                  // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	regBank.set(19,1);                                                  // J8-11     XP7 2 Sence  ����. �.
	regBank.set(16,1);                                                  // XS1 - 6   Sence ���
	regBank.set(25,1);                                                  // XP1- 19 HaSs      Sence ����������� ������      
	regBank.set(13,1);                                                  // XP8 - 2           Sence �������� ������
	regBank.set(28,1);                                                  // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[8])));
	myFile.println(buffer);                                             // "Komanda PTT ON instruktora  send!"

	UpdateRegs(); 
	delay(300);

	  // 3)  �������� ������� �� ����������� ��������� ����������� 2 ����������
		 byte i5 = regs_in[2];   
		 if(bitRead(i5,1) == 0)                                         // �������� ����� ����������� ��������� ����������� 2 ����������
			{
				regcount = regBank.get(40127);                          // ����� �������� ������ ������� ��������� ����������� � 2 ����������
				regcount++;                                             // ��������� ������� ������
				regBank.set(40127,regcount);                            // ����� �������� ������ ������� ��������� ����������� � 2 ����������
				regBank.set(127,1);                                     // ���������� ���� ������
				regBank.set(120,1);                                     // ���������� ����� ���� ������
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[2])));
				myFile.print(buffer);                                   // "Komanda sensor ON instruktora 2 Error! - "
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[3])));
				myFile.println(buffer);                                 // "Komanda sensor ON  instruktora 2 - Ok!"
			}
	  // 4)  �������� ������� �� ����������� ��������� �����������

	   if(bitRead(i5,2) == 0)                                            // �������� ����� ����������� ��������� �����������
			 {
				regcount = regBank.get(40128);                          // ����� �������� ������ ������� ��������� �����������
				regcount++;                                             // ��������� ������� ������
				regBank.set(40128,regcount);                            // ����� �������� ������ ������� ��������� �����������
				regBank.set(128,1);                                     // ���������� ���� ������
				regBank.set(120,1);                                     // ���������� ����� ���� ������
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

		 if(regBank.get(adr_reg_ind_CTS)== 0)                          // ��������� ��������� PTT �����������   CTS
			{
				regcount = regBank.get(40140);                         // ����� �������� ������ ���������� PTT ��������� �����������
				regcount++;                                            // ��������� ������� ������
				regBank.set(40140,regcount);                           // ����� �������� ������ ���������� PTT ��������� �����������
				regBank.set(140,1);                                    // ���������� ���� ������ ���������� PTT ��������� �����������
				regBank.set(120,1);                                    // ���������� ����� ���� ������
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_on[6])));
				myFile.print(buffer);                                  // "Komanda PTT instruktora(CTS) ON Error! - " �������� �� ���������� 
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
	  myFile.println(buffer);                                           // "Komanda sensor OFF ����������  send!"
	  regBank.set(32,0);                                                // XP1- 1  HeS1Ls    ��������� ������ ��������� ����������
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[1])));
	  myFile.println(buffer);                                           // "Komanda sensor OFF ���������� 2 send!"
	  regBank.set(31,0);                                                // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	  regBank.set(16,0);                                                // XS1 - 6   Sence ���
	  regBank.set(1,0);                                                 // ���� RL0 ����
	  regBank.set(2,0);                                                 // ���� RL1 ����
	  regBank.set(3,0);                                                 // ���� RL2 ����
	  regBank.set(4,0);                                                 // ���� RL3 ����  LFE  "���."
	  regBank.set(5,0);                                                 // ���� RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                 // ���� RL5 ����
	  regBank.set(7,0);                                                 // ���� RL6 ����
	  regBank.set(9,0);                                                 // ���� RL8 ���� �� ��������
	  regBank.set(10,0);                                                // ���� RL9 XP1 10
	  regBank.set(30,0);                                                // XP1- 6  HeS1PTT   ��������� PTT ����������
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[2])));
	  myFile.println(buffer);                                           // "Komanda PTT ���������� OFF send!"

	  UpdateRegs();                                                     // ��������� ������� ���������� ��������
	  delay(400);                                                       // 
	 
	  // 1)  �������� ������� �� ���������� ��������� ���������� 2 ����������
	  byte i5 = regs_in[2];                                             // 
		if(bitRead(i5,3) > 0)                                           // ��������  ����� �� ���������� ��������� ���������� 2 ����������
		  {
			regcount = regBank.get(40129);                              // ����� �������� ������ ������� ��������� ���������� � 2 ����������
			regcount++;                                                 // ��������� ������� ������
			regBank.set(40129,regcount);                                // ����� �������� ������ ������� ��������� ���������� � 2 ����������
			regBank.set(129,1);                                         // ���������� ���� ������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			resistor(1, 255);                                           // ���������� ������� ������� � �������� ���������
			resistor(2, 255);                                           // ���������� ������� ������� � �������� ���������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[3])));
			myFile.print(buffer);                                       // "Komanda sensor OFF dispetchera 2 Error! - "
			myFile.println(regcount);                                   // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[4])));
			myFile.println(buffer);                                     // "Komanda sensor OFF dispetchera 2 - Ok!"
		  }

	  // 2)  �������� ������� �� ���������� ��������� ����������

		if(bitRead(i5,4) > 0)                                           // ��������  ����� �� ���������� ��������� ���������� 
		  {
			 regcount = regBank.get(40130);                             // ����� �������� ������ ������� ��������� ����������
			 regcount++;                                                // ��������� ������� ������
			 regBank.set(40130,regcount);                               // ����� �������� ������ ������� ��������� ����������
			 regBank.set(130,1);                                        // ���������� ���� ������
			 regBank.set(120,1);                                        // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[5])));
			 myFile.print(buffer);                                      // "Komanda sensor OFF dispetchera  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[6])));
			 myFile.println(buffer);                                    // "Komanda sensor OFF dispetchera   - Ok!"
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i5,5) > 0)                                           // ��������  ����� �� ���������� ���������
		  {
			 regcount = regBank.get(40149);                             // ����� �������� ������ ������� ��������� 
			 regcount++;                                                // ��������� ������� ������
			 regBank.set(40149,regcount);                               // ����� �������� ������ ������� ���������
			 regBank.set(149,1);                                        // ���������� ���� ������
			 regBank.set(120,1);                                        // ���������� ����� ���� ������
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
	   if(regBank.get(adr_reg_ind_CTS)!= 0)                             // ��������� ��������� PTT �����������   CTS
		  {
			regcount = regBank.get(40158);                              // ����� �������� ������ ���������� PTT ��������� ����������
			regcount++;                                                 // ��������� ������� ������
			regBank.set(40158,regcount);                                // ����� �������� ������ ���������� PTT ��������� ����������
			regBank.set(158,1);                                         // ���������� ���� ������ ���������� PTT ��������� ����������
			regBank.set(120,1);                                         // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_off[9])));
			myFile.print(buffer);                                       // "Komanda PTT dispetchera (CTS) OFF Error! - " �������� �� ���������� 
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
	regBank.set(32,1);                                            // XP1- 1  HeS1Ls    Sence ����������� ��������� ���������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[1])));
	myFile.println(buffer);                                       // "Komanda sensor ON dispetchera 2  send!"
	regBank.set(31,1);                                            // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.set(19,1);                                            // J8-11     XP7 2 Sence  ����. �.
	regBank.set(16,1);                                            // XS1 - 6   Sence ���
	regBank.set(25,1);                                            // XP1- 19 HaSs      Sence ����������� ������      
	regBank.set(13,1);                                            // XP8 - 2           Sence �������� ������
	regBank.set(30,1);                                            // XP1- 6  HeS1PTT   �������� PTT ����������
	UpdateRegs(); 
	delay(300);

	  // 3)  �������� ������� �� ����������� ��������� ����������� 2 ����������
		 byte i5 = regs_in[2];   
		 if(bitRead(i5,3) == 0)                                         // �������� ����� ����������� ��������� ���������� 2 ����������
			{
				regcount = regBank.get(40129);                          // ����� �������� ������ ������� ��������� ���������� � 2 ����������
				regcount++;                                             // ��������� ������� ������
				regBank.set(40129,regcount);                            // ����� �������� ������ ������� ��������� ���������� � 2 ����������
				regBank.set(129,1);                                     // ���������� ���� ������
				regBank.set(120,1);                                     // ���������� ����� ���� ������
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[2])));
				myFile.print(buffer);                                   // "Komanda sensor ON dispetchera 2 Error! - "
				myFile.println(regcount);
			}
		else
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[3])));
				myFile.println(buffer);                                 // "Komanda sensor ON  dispetchera 2 - Ok!"
			}
	  // 4)  �������� ������� �� ����������� ��������� �����������

	   if(bitRead(i5,4) == 0)                                            // �������� ����� ����������� ��������� ����������
			 {
				regcount = regBank.get(40130);                          // ����� �������� ������ ������� ��������� ����������
				regcount++;                                             // ��������� ������� ������
				regBank.set(40130,regcount);                            // ����� �������� ������ ������� ��������� ����������
				regBank.set(130,1);                                     // ���������� ���� ������
				regBank.set(120,1);                                     // ���������� ����� ���� ������
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
		 if(regBank.get(adr_reg_ind_CTS)== 0)                          // ��������� ��������� PTT ����������   CTS
			{
				regcount = regBank.get(40158);                         // ����� �������� ������ ���������� PTT ��������� ����������
				regcount++;                                            // ��������� ������� ������
				regBank.set(40158,regcount);                           // ����� �������� ������ ���������� PTT ��������� ����������
				regBank.set(158,1);                                    // ���������� ���� ������ ���������� PTT ��������� ����������
				regBank.set(120,1);                                    // ���������� ����� ���� ������
				strcpy_P(buffer, (char*)pgm_read_word(&(table_disp_on[6])));
				myFile.print(buffer);                                  // "Komanda PTT dispetchera OFF Error! - " �������� �� ���������� 
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
	  regBank.set(25,1);                                                    // XP1- 19 HaSs      Sence ����������� ������    MTT  OFF       
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[0])));
	  myFile.println(buffer);                                               // "Komanda sence OFF MTT  send!                  
	  regBank.set(26,0);                                                    // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[1])));
	  myFile.println(buffer);                                               // "Komanda PTT MTT OFF send!"
	  regBank.set(18,0);                                                    // XP1 - 20  HangUp  DCD
	  strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[2])));
	  myFile.println(buffer);                                               // "Komanda  HangUp MTT OFF send!"
	  regBank.set(16,0);                                                    // XS1 - 6   Sence ���
	  regBank.set(1,0);                                                     // ���� RL0 ����
	  regBank.set(2,0);                                                     // ���� RL1 ����
	  regBank.set(3,0);                                                     // ���� RL2 ����
	  regBank.set(4,0);                                                     // ���� RL3 ����  LFE  "���."
	  regBank.set(5,0);                                                     // ���� RL4 XP1 12  HeS2e 
	  regBank.set(6,0);                                                     // ���� RL5 ����
	  regBank.set(7,0);                                                     // ���� RL6 ����
	  regBank.set(9,0);                                                     // ���� RL8 ���� �� ��������
	  regBank.set(10,0);                                                    // ���� RL9 XP1 10
	  UpdateRegs();                                                         // ��������� ������� ���������� ��������
	  delay(400);                                                           // 
	  // 1)  �������� ������� MTT �� ���������� 
	  byte i5 = regs_in[0];                                                 // 
		if(bitRead(i5,2) != 0)                                              // ��������  ����� �� ���������� ������� ���
		  {
			regcount = regBank.get(40163);                                  // ����� �������� ������ ������� ���
			regcount++;                                                     // ��������� ������� ������
			regBank.set(40163,regcount);                                    // ����� �������� ������ ������� ���
			regBank.set(163,1);                                             // ���������� ���� ������
			regBank.set(120,1);                                             // ���������� ����� ���� ������
			resistor(1, 255);                                               // ���������� ������� ������� � �������� ���������
			resistor(2, 255);                                               // ���������� ������� ������� � �������� ���������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[3])));
			myFile.print(buffer);                                           // "Komanda sensor OFF ��� Error! - "
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[4])));
			myFile.println(buffer);                                         // "Komanda sensor OFF  ��� - Ok!"
		  }
		   UpdateRegs(); 
	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                            // ��������  �� ���������� CTS MTT
		  {
			 regcount = regBank.get(40164);                                 // ����� �������� ������ PTT  MTT (CTS)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40164,regcount);                                   // ����� �������� ������ PTT  MTT (CTS)
			 regBank.set(164,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[7])));
			 myFile.print(buffer);                                          // "Komanda       OFF PTT  MTT (CTS)   Error! - 
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[8])));
			 myFile.println(buffer);                                        // "Komanda       OFF PTT  MTT (CTS)  - Ok!"
		  }

	 // 3)  ��������  �� ���������� PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                            // ��������  �� ����������  PTT  MTT (DSR)
		  {
			 regcount = regBank.get(40165);                                 // ����� �������� ������  PTT  MTT (DSR)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40165,regcount);                                   // ����� �������� ������  PTT  MTT (DSR)
			 regBank.set(165,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[9])));
			 myFile.print(buffer);                                          // "Komanda       OFF PTT  MTT (DSR)  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[10])));
			 myFile.println(buffer);                                        // "Komanda       OFF PTT  MTT (DSR)  - Ok!"
		  }

	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                 // ��������� ��������� HangUp  DCD
		  {
			regcount = regBank.get(40168);                                  // ����� �������� ������ ���������� HangUp  DCD
			regcount++;                                                     // ��������� ������� ������
			regBank.set(40169,regcount);                                    // ����� �������� ������ ���������� HangUp  DCD
			regBank.set(169,1);                                             // ���������� ���� ������ ���������� HangUp  DCD
			regBank.set(120,1);                                             // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[11])));
			myFile.print(buffer);                                           // "Komanda HangUp  DCD OFF Error! - "  
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[12])));
			myFile.println(buffer);                                         // "Komanda HangUp  DCD OFF- Ok!"
		 }
}
void test_MTT_on()
{
	unsigned int regcount = 0;
	regBank.set(25,0);                                                      //  XP1- 19 HaSs  Sence ����������� ������    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[0])));
	myFile.println(buffer);                                                 // "Komanda sence ON MTT  send!                  
	regBank.set(26,1);                                                      // XP1- 17 HaSPTT    CTS DSR ���. �������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[1])));
	myFile.println(buffer);                                                 // "Komanda PTT MTT ON send!"
	regBank.set(18,1);                                                      // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[2])));
	myFile.println(buffer);                                                 // "Komanda  HangUp MTT ON send!"

	UpdateRegs(); 
	delay(300);

	  // 1)  �������� ������� MTT �� ���������� 
	  byte i5 = regs_in[0];                                                 // 
		if(bitRead(i5,2) == 0)                                              // ��������  ����� �� ��������� ������� ���
		  {
			regcount = regBank.get(40170);                                  // ����� �������� ������ ������� ���
			regcount++;                                                     // ��������� ������� ������
			regBank.set(40170,regcount);                                    // ����� �������� ������ ������� ���
			regBank.set(170,1);                                             // ���������� ���� ������
			regBank.set(120,1);                                             // ���������� ����� ���� ������
			resistor(1, 255);                                               // ���������� ������� ������� � �������� ���������
			resistor(2, 255);                                               // ���������� ������� ������� � �������� ���������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[3])));
			myFile.print(buffer);                                           // "Komanda sensor ON ��� Error! - "
			myFile.println(regcount);                                       // 
		  }
		else
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_on[4])));
			myFile.println(buffer);                                         // "Komanda sensor ON  ��� - Ok!"
		  }
		UpdateRegs(); 
	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                            // ��������  �� ��������� CTS MTT
		  {
			 regcount = regBank.get(40166);                                 // ����� �������� ������ PTT  MTT (CTS)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40166,regcount);                                   // ����� �������� ������ PTT  MTT (CTS)
			 regBank.set(166,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[7])));
			 myFile.print(buffer);                                          // "Komanda       ON PTT  MTT (CTS)   Error! - 
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[8])));
			 myFile.println(buffer);                                        // "Komanda       ON PTT  MTT (CTS)  - Ok!"
		  }

	 // 3)  ��������  �� ���������� PTT  MTT (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                            // ��������  �� ����������  PTT  MTT (DSR)
		  {
			 regcount = regBank.get(40167);                                 // ����� �������� ������  PTT  MTT (DSR)
			 regcount++;                                                    // ��������� ������� ������
			 regBank.set(40167,regcount);                                   // ����� �������� ������  PTT  MTT (DSR)
			 regBank.set(167,1);                                            // ���������� ���� ������
			 regBank.set(120,1);                                            // ���������� ����� ���� ������
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[9])));
			 myFile.print(buffer);                                          // "Komanda       OFF PTT  MTT (DSR)  Error! - "
			 myFile.println(regcount);
		  }
		else
		  {
			 strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[10])));
			 myFile.println(buffer);                                        // "Komanda       OFF PTT  MTT (DSR)  - Ok!"
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                 // ��������� ��������� HangUp  DCD
		  {
			regcount = regBank.get(40169);                                  // ����� �������� ������ ���������� HangUp  DCD
			regcount++;                                                     // ��������� ������� ������
			regBank.set(40169,regcount);                                    // ����� �������� ������ ���������� HangUp  DCD
			regBank.set(169,1);                                             // ���������� ���� ������ ���������� HangUp  DCD
			regBank.set(120,1);                                             // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[11])));
			myFile.print(buffer);                                           // "Komanda HangUp  DCD OFF Error! - "  
			myFile.println(regcount);
		 }
	  else
		 {
			strcpy_P(buffer, (char*)pgm_read_word(&(table_mtt_off[12])));
			myFile.println(buffer);                                         // "Komanda HangUp  DCD OFF- Ok!"
		 }
}

void measure_vol_min(int istochnik, unsigned int adr_count, int adr_flagErr, unsigned int porogV)
{
		int regcount = 0;
		measure_volume(istochnik);                                               // �������� ������� ������� �� ������
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
				//����������� 
				break;
			case 11:
				//�����������
				break;
			case 12:
				//�����������
				break;
			case 13:
				//����������� 
				break;



		}

		myFile.print(buffer);                                          // 

		if(voltage10 >  porogV)                                        // ��������� ����������� ������
			{
				regcount = regBank.get(adr_count);                     // ����� �������� ������ 
				regcount++;                                            // ��������� ������� ������ ������ 
				regBank.set(adr_count,regcount);                       // ����� �������� ������ ������ 
				regBank.set(adr_flagErr,1);                            // ���������� ���� ������  ������ 
				regBank.set(120,1);                                    // ���������� ����� ���� ������ 
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
		measure_volume(istochnik);                                                  // �������� ������� ������� �� ������
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
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[58])));     // "Signal Mag phone on         - "
		   //  myFile.print(buffer);                                                // "Signal Mag phone on         - "
				break;
			 case 160:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[50])));     // "Signal FrontL OFF    - "
				break;
			case 161:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[51])));     // "Signal FrontR OFF    - "
				break;
			case 172:
				strcpy_P(buffer, (char*)pgm_read_word(&(table_instr_all[49])));     // "Signal FrontR OFF    - "
				break;
			case 10:
				//����������� �����  var ����� 2
				break;
		}
		myFile.print(buffer);    
		if(voltage10 <  porogV)                                       // ��������� ����������� ������
			{
				regcount = regBank.get(adr_count);                     // ����� �������� ������ 
				regcount++;                                            // ��������� ������� ������ ������ 
				regBank.set(adr_count,regcount);                       // ����� �������� ������ ������ 
				regBank.set(adr_flagErr,1);                            // ���������� ���� ������  ������ 
				regBank.set(120,1);                                    // ���������� ����� ���� ������ 
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
	regBank.add(13);    // XP8 - 2   Sence �������� ������
	regBank.add(14);    // XP8 - 1   PTT �������� ������
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
 
	regBank.add(25);    // XP1- 19 HaSs      Sence ����������� ������    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);    // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.add(27);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	regBank.add(28);    // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.add(29);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
	regBank.add(30);    // XP1- 6  HeS1PTT   CTS ���
	regBank.add(31);    // XP1- 5  HeS1Rs    Sence ���������� ��������� ���������� � 2 ����������
	regBank.add(32);    // XP1- 1  HeS1Ls    Sence ���������� ��������� ����������

	regBank.add(120);   // ���� ��������� ������������� ����� ������
 
	regBank.add(121);   // ���� �������� ������ ������� Sence "��-�����1."  ok!
	regBank.add(122);   // ���� �������� ������ ������� Sence "��-�����2."  ok!
	regBank.add(123);   // ���� �������� ������ ������� ����������� ������
	regBank.add(124);   // ���� �������� ������ ������� Sence ����. �. ok!
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
	regBank.add(143);   // ���� ������ �������  LineL ��������� ����������� 
	regBank.add(144);   // ���� ������ analog_mag_radio
	regBank.add(145);   // ���� ������ analog_mag_phone
	regBank.add(146);   // ���� ������ analog_ggs OFF
	regBank.add(147);   // ���� ������ analog_gg_radio1
	regBank.add(148);   // ���� ������ analog_gg_radio2
	regBank.add(149);   // ���� ������ ������� ���������
	regBank.add(150);   // ���� ������ Mag phone on
	regBank.add(151);   // ���� ������ ��������� ��������� �����������
	regBank.add(152);   // ���� ������ ��������� ��������� ����������
	regBank.add(153);   // ���� ������ XP1 - 20  HangUp  DCD
	regBank.add(154);   // ���� ������ Sence MTT ON
	regBank.add(155);   // ���� ������ ���������� ��������� ��������� ����������
	regBank.add(156);   // ����� �������� ������ ���������� �������������
	regBank.add(157);   // ����� �������� ������ ��������� �������������
	regBank.add(158);   // ���� ������ ���������� PTT ��������� ����������
	regBank.add(159);   // ���� ������ ���������� ��������� ��������� ����������
	regBank.add(160);   // ���� ������ �������� ��������� ���������� FrontL
	regBank.add(161);   // ���� ������ �������� ��������� ���������� FrontR
	regBank.add(162);   // ���� ������ �������  LineL ��������� ���������� 
	regBank.add(163);   // ���� ������ Sence MTT OFF
	regBank.add(164);   // ���� ������  MTT PTT OFF (CTS) 
	regBank.add(165);   // ���� ������  MTT PTT OFF (DSR) 
	regBank.add(166);   // ���� ������  MTT PTT ON  (CTS) 
	regBank.add(167);   // ���� ������  MTT PTT ON  (DSR)
	regBank.add(168);   // ���� ������  MTT HangUp OFF (DCD)
	regBank.add(169);   // ���� ������  MTT HangUp ON  (DCD)
	regBank.add(170);   // ���� ������  Sence MTT ON
	regBank.add(171);   // ���� ������ �������  LineL MTT
	regBank.add(172);   // ���� ������ analog_ggs ON
	regBank.add(173);   // ���� ������ "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(174);   // ���� ������ "Komanda PTT2  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(175);   // ���� ������ "Komanda PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(176);   // ���� ������ "Komanda PTT2  ON  tangenta ruchnaja (CTS)      Error! - ";





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
	//regBank.add(30045);  // ���� 3 ����� ��� 5 - ��������   ���� ����������� ��������� XS1 - 6 Sence
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
 
	regBank.add(40120);  // adr_control_command ����� �������� ������� �� ����������
	regBank.add(40121);  // ����� �������� ������ ������� Sence "��-�����1."  ok!
	regBank.add(40122);  // ����� �������� ������ ������� Sence "��-�����2."  ok!
	regBank.add(40123);  // ����� �������� ������ ������� ����������� ������
	regBank.add(40124);  // ����� �������� ������ �������  Sence ����. �. ok!
	regBank.add(40125);  // ����� �������� ������ ������� Sence ���� �. ok!
	regBank.add(40126);  // ����� �������� ������ ������� Sence "���."   ok!
	regBank.add(40127);  // ����� �������� ������ ������� ��������� ����������� � 2 ����������
	regBank.add(40128);  // ����� �������� ������ ������� ��������� �����������
	regBank.add(40129);  // ����� �������� ������ ������� ��������� ���������� � 2 ����������

	regBank.add(40130);  // ����� �������� ������ ������� ��������� ����������
	regBank.add(40131);  // ����� �������� ������ ������� Sence ���. ok!
	regBank.add(40132);  // ����� �������� ������ ������� Sence "��C."   ok!
	regBank.add(40133);  // ����� �������� ������ ������� 
	regBank.add(40134);  // ����� �������� ������ ������� PTT ���� �.  ok!
	regBank.add(40135);  // ����� �������� ������ ������� PTT ��� ok!
	regBank.add(40136);  // ����� �������� ������  PTT2 ����. �. ok!
	regBank.add(40137);  // ����� �������� ������  HangUp  DCD  ok!
	regBank.add(40138);  // ����� �������� ������  PTT1 ����. �. ok!
	regBank.add(40139);  // ����� �������� ������ ���������� ��������� ��������� ����������� 

	regBank.add(40140);  // ����� �������� ������ ���������� PTT ��������� �����������
	regBank.add(40141);  // ����� �������� ������ �������� ��������� ����������� FrontL
	regBank.add(40142);  // ����� �������� ������ �������� ��������� ����������� FrontR
	regBank.add(40143);  // ����� �������� ������ LineL
	regBank.add(40144);  // ����� �������� ������ analog_mag_radio
	regBank.add(40145);  // ����� �������� ������ analog_mag_phone
	regBank.add(40146);  // ����� �������� ������ analog_ggs
	regBank.add(40147);  // ����� �������� ������ analog_gg_radio1
	regBank.add(40148);  // ����� �������� ������ analog_gg_radio2
	regBank.add(40149);  // ����� �������� ������ ������� ���������
	regBank.add(40150);  // ����� �������� ������  Mag phone on
	regBank.add(40151);  // ����� �������� ������ ��������� ��������� ����������� 
	regBank.add(40152);  // ����� �������� ������ ��������� ��������� ����������
	regBank.add(40153);  // ����� �������� ������ XP1 - 20  HangUp  DCD
	regBank.add(40154);  // ����� �������� ������ Sence MTT ON
	regBank.add(40155);  // ����� �������� ������ ���������� ��������� ��������� ����������
	regBank.add(40156);  // ����� �������� ������ ���������� �������������
	regBank.add(40157);  // ����� �������� ������ ��������� �������������
	regBank.add(40158);  // ����� �������� ������ ���������� PTT ��������� ����������
	regBank.add(40159);  // ����� �������� ������ ���������� ��������� ��������� ����������
	regBank.add(40160);  // ����� �������� ������ �������� ��������� ���������� FrontL
	regBank.add(40161);  // ����� �������� ������ �������� ��������� ���������� FrontR
	regBank.add(40162);  // ����� �������� ������ �������  LineL ��������� ���������� 
	regBank.add(40163);  // ����� �������� ������ Sence MTT OFF
	regBank.add(40164);  // ����� ��������  MTT PTT OFF (CTS) 
	regBank.add(40165);  // ����� ��������  MTT PTT OFF (DSR) 
	regBank.add(40166);  // ����� ��������  MTT PTT ON  (CTS) 
	regBank.add(40167);  // ����� ��������  MTT PTT ON  (DSR)
	regBank.add(40168);  // ����� ��������  MTT HangUp OFF (DCD)
	regBank.add(40169);  // ����� ��������  MTT HangUp ON  (DCD)
	regBank.add(40170);  // ����� �������� ������  Sence MTT ON
	regBank.add(40171);  // ����� �������� ������ �������  LineL MTT
	regBank.add(40172);  // ����� �������� ������ analog_ggs ON
	regBank.add(40173);  // ���� ������ "Komanda PTT1  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40174);  // ���� ������ "Komanda PTT2  OFF tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40175);  // ���� ������ "Komanda PTT1  ON  tangenta ruchnaja (CTS)      Error! - ";
	regBank.add(40176);  // ���� ������ "Komanda PTT2  ON  tangenta ruchnaja (CTS)      Error! - ";


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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    Sence ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    Sence ����������� ��������� ����������� 
}
