#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>


modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

int test_n=0;
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
	regBank.add(4);     // Реле RL3 Звук
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
	 test_n = regBank.get(adr_control_command); //адрес  40120
	 if (test_n!= 0) Serial.println(test_n);
  }
}
