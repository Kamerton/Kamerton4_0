
#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <RTClib.h>
#include <Wire.h>   

RTC_DS1307 RTC;                                     // define the Real Time Clock object

//-----------------------------------------------------------------------------------------------

uint8_t second = 0;       //Initialization time
uint8_t minute = 0;
uint8_t hour   = 0;
uint8_t dow    = 1;
uint8_t day    = 1;
uint8_t month  = 1;
uint16_t year  = 15 ;

// Adafruit SD shields and modules: pin 10
//const uint8_t chipSelect = 53;
const uint8_t chipSelect = SS;
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;

// Serial output stream
ArduinoOutStream cout(Serial);

Sd2Card card;
SdFat sd;
SdFile myFile;
File root;

//*********************Работа с именем файла ******************************
//char file_name[13] ;
//char file_name_txt[5] = ".txt";
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
//char list_files_tab[200][13];
//uint32_t size_files_tab[200] ;
//int set_files = 0;

char c;  // Для ввода символа с ком порта

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

//++++++++++++++++++++ Назначение имени файла ++++++++++++++++++++++++++++++++++++++++++++
const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "150101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13] = FILE_BASE_NAME "00.TXT";
//------------------------------------------------------------------------------
const int8_t ERROR_LED_PIN = 13;

//==============================================================================
// Error messages stored in flash.
#define error(msg) error_P(PSTR(msg))
//------------------------------------------------------------------------------
void error_P(const char* msg) {
  sd.errorHalt_P(msg);
}

//------------------------------------------------------------------------------

// Size of file base name.  Must not be larger than six.
//Размер базовой части имени файла. Должно быть не больше, чем шесть.
//const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

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
      error("Can't create file name");
    }
  }
  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) error("file.open");
  //do {
  //  delay(10);
  // } while (Serial.read() >= 0);
  //
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  myFile.close();
  Serial.println("done.");




 // Serial.println(F("Type any character to stop"));
} 

void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
{
	DateTime now = RTC.now();
	//second = now.second();       //Initialization time
	//minute = now.minute();
	//hour   = now.hour();
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
		 
	if (file_name_count<10)
		{
			itoa (file_name_count,str0, 10);                                   // Преобразование числа в строку ( 10 - десятичный формат) 
			sprintf(str_file_name_count, "%s%s", str_file_name_count0, str0);  // Сложение 2 строк
		}
	
	else
		{
			itoa (file_name_count,str_file_name_count, 10);                    // Преобразование числа в строку ( 10 - десятичный формат) 
		}
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // Сложение 2 строк
	sprintf(str2, "%s%s",str1, str_day_file);                                  // Сложение 2 строк
	sprintf(fileName, "%s%s", str2, "00.TXT");                                 // Получение имени файла в file_name
}

//+++++++++++++++++++++++++++++++++++++++++++++ Форматирование SD ++++++++++++++++++++++++++
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
#define sdError(msg) sdError_P(PSTR(msg))

void sdError_P(const char* str) {
  cout << pstr("error: ");
  cout << pgm(str) << endl;
  if (card.errorCode()) {
    cout << pstr("SD error: ") << hex << int(card.errorCode());
    cout << ',' << int(card.errorData()) << dec << endl;
  }
  while (1);
}
//------------------------------------------------------------------------------
// write cached block to the card
uint8_t writeCache(uint32_t lbn) 
{
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

  cout << pstr("Blocks/Cluster: ") << int(sectorsPerCluster) << endl;
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
    if ((i & 0XFF) == 0) cout << '.';
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
uint16_t lbnToCylinder(uint32_t lbn) 
{
  return lbn / (numberOfHeads * sectorsPerTrack);
}
//------------------------------------------------------------------------------
// return head number for a logical block number
uint8_t lbnToHead(uint32_t lbn) 
{
  return (lbn % (numberOfHeads * sectorsPerTrack)) / sectorsPerTrack;
}
//------------------------------------------------------------------------------
// return sector number for a logical block number
uint8_t lbnToSector(uint32_t lbn) 
{
  return (lbn % sectorsPerTrack) + 1;
}
//------------------------------------------------------------------------------
// format and write the Master Boot Record
void writeMbr() 
{
  clearCache(true);
  part_t* p = cache.mbr.part;
  p->boot = 0;
  uint16_t c = lbnToCylinder(relSector);
  if (c > 1023) sdError("MBR CHS");
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
  if (!writeCache(0)) sdError("write MBR");
}
//------------------------------------------------------------------------------
// generate serial number from card size and micros since boot
uint32_t volSerialNumber() 
{
  return (cardSizeBlocks << 8) + micros();
}
//------------------------------------------------------------------------------
// format the SD as FAT16
void makeFat16() 
{
  uint32_t nc;
  for (dataStart = 2 * BU16;; dataStart += BU16) {
    nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
    fatSize = (nc + 2 + 255)/256;
    uint32_t r = BU16 + 1 + 2 * fatSize + 32;
    if (dataStart < r) continue;
    relSector = dataStart - r + BU16;
    break;
  }
  // check valid cluster count for FAT16 volume
  if (nc < 4085 || nc >= 65525) sdError("Bad cluster count");
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
void makeFat32() 
{
  uint32_t nc;
  relSector = BU32;
  for (dataStart = 2 * BU32;; dataStart += BU32) {
    nc = (cardSizeBlocks - dataStart)/sectorsPerCluster;
    fatSize = (nc + 2 + 127)/128;
    uint32_t r = relSector + 9 + 2 * fatSize;
    if (dataStart >= r) break;
  }
  // error if too few clusters in FAT32 volume
  if (nc < 65525) sdError("Bad cluster count");
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

void formatCard() 
{
  cout << endl;
  cout << pstr("Formatting\n");
  initSizes();
  if (card.type() != SD_CARD_TYPE_SDHC) 
  {
   // cout << pstr("FAT16\n");
    makeFat16();
  }
  else 
  {
   // cout << pstr("FAT32\n");
    makeFat32();
  }
#if DEBUG_PRINT
  debugPrint();
#endif  // DEBUG_PRINT
  cout << pstr("Format done\n");
}
//+++++++++++++++++++++++++++++++++++++++++++++Конецы  Форматирование SD ++++++++++++++++++++++++++

void test_file()
{
	file_name();
	// open the file for write at end like the Native SD library

    if (!myFile.open(fileName, O_RDWR | O_CREAT | O_AT_END)) 
	//if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) 
	{
		sd.errorHalt("opening test.txt for write failed");
	}
	// if the file opened okay, write to it:
	Serial.print("Writing to test.txt...");
	myFile.println("testing 1, 2, 3.");

	// close the file:
	myFile.close();
	Serial.println("done.");

	// re-open the file for reading:
	if (!myFile.open(fileName, O_READ)) 
	{
		sd.errorHalt("opening test.txt for read failed");
	}
	Serial.println(fileName);

	// read from the file until there's nothing else in it:
	int data;
	while ((data = myFile.read()) >= 0) Serial.write(data);
	// close the file:
	myFile.close();
}
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

void com_port_in_menu()
{
  while (1) 
  {
 	  cout << pstr(
		"\n"
		"Options are:\n"
		"F or f - erase and then format the card. (recommended)\n"
		"T or t - test file.\n"
		"P or p - print date.\n"
		"N or n - set file name.\n"
		"S or s - set time.\n"
		"\n"
		"Enter option: ");
    
	  while (!Serial.available()) {}
	  c = Serial.read();
	  cout << c << endl;
	  if (!strchr("FfPpTtNnSs", c)) 
	  {
		cout << pstr("Quiting, invalid option entered.") << endl;
		return;
	  }

	  if (!card.init(spiSpeed, chipSelect)) 
	  {
		cout << pstr(
		 "\nSD initialization failure!\n"
		 "Is the SD card inserted correctly?\n"
		 "Is chip select correct at the top of this sketch?\n");
		sdError("card.init failed");
	  }
	  if (c == 'F' || c == 'f') 
	  {
		formatCard();
	  }
		if (c == 'T'|| c == 't') 
	  {
		test_file();
	  }
	   if (c == 'P'|| c == 'p') 
	  {
		serial_print_date();
	    Serial.println();
	  }

	   if (c == 'N'|| c == 'n') 
	  {
	   file_name();
	  }
	   	   if (c == 'S'|| c == 's') 
	  {
	    set_time();
	  }
  }
}

//------------------------------------------------------------------------------
void setup() 
{

	Serial.begin(9600);
	Wire.begin();

	if (!RTC.begin())                               // Настройка часов 
		{
			Serial.println("RTC failed");
			while(1);
		};

	SdFile::dateTimeCallback(dateTime);             // Настройка времени записи файла

	delay(400);  // catch Due reset problem
  
	// Initialize SdFat or print a detailed error message and halt
	// Use half speed like the native library.
	// change to SPI_FULL_SPEED for more performance.
	if (!sd.begin(chipSelect, SPI_HALF_SPEED)) sd.initErrorHalt();
	cardSizeBlocks = card.cardSize();
	if (cardSizeBlocks == 0) sdError("cardSize");
	cardCapacityMB = (cardSizeBlocks + 2047)/2048;

	cout << pstr("Card Size: ") << cardCapacityMB;
	cout << pstr(" MB, (MB = 1,048,576 bytes)") << endl;
	preob_num_str();
	// rtc.adjust(DateTime(__DATE__, __TIME__));

}
//------------------------------------------------------------------------------
void loop() 
{
	com_port_in_menu();
	delay(500);

}
