#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>


modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

int test_n=0;
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
	setup_regModbus();
}

void loop()
{
////put some data into the registers
//  regBank.set(1, 1);  
//  regBank.set(2, 1);  
//  regBank.set(3, 0);  
//  regBank.set(4, 1);  
//  regBank.set(5, 1);  
//  regBank.set(6, 0);  
//  regBank.set(7, 1);  
//  regBank.set(8, 0);  
//
//  regBank.set(10001, 1);
//  regBank.set(10002, 1);  
//  regBank.set(10003, 1);  
//  regBank.set(10004, 1);  
//  regBank.set(10005, 0);  
//  regBank.set(10006, 0);  
//  regBank.set(10007, 0);  
//  regBank.set(10008, 0);  
//
//
//  regBank.set(30001,1);
//  regBank.set(30002,2);
//  regBank.set(30003,3);
//  regBank.set(30004,4);
//  regBank.set(30005,5);
//  regBank.set(30006,6);
//  regBank.set(30007,7);
//  regBank.set(30008,8);
//  regBank.set(30009,9);
//  regBank.set(30010,10);
//
//  regBank.set(40001,1);
//  regBank.set(40002,2);
//  regBank.set(40003,2);
//  regBank.set(40004,4);
//  regBank.set(40005,5);
//  regBank.set(40006,6);
//  regBank.set(40007,7);
//  regBank.set(40008,8);
//  regBank.set(40009,9);
//  regBank.set(40010,10);
  
 while(1)
  {
	////put a random number into registers 1, 10001, 30001 and 40001
	//regBank.set(1, (byte) random(0, 2));
	//regBank.set(10001, (byte) random(0, 2));
	//regBank.set(30001, (word) random(0, 32767));
	//regBank.set(40001, (word) random(0, 32767));
	
	 slave.run();  
	 test_n = regBank.get(adr_control_command); //�����  40120
	 if (test_n!= 0) Serial.println(test_n);
  }
}
