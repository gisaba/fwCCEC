#include <Arduino.h>
#ifndef F_CPU               // if F_CPU was not defined in Project -> Properties
#define F_CPU 8000000UL    // define it now as 8 MHz unsigned long
#endif

/*********************************************************************************************/
#include <avr/io.h>       // this is always included in AVR programs
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <LiquidCrystal_I2C.h>
#include <DS3231M.h>
#include <SPIMemory.h>
/*********************************************************************************************/

static const uint8_t D42 = 42;

static const uint8_t IRQ_Module = PORTB1;
static const uint8_t IRQ_TASTIERA = PORTB2;
static const uint8_t IRQ_RTC = PORTC5;
static const uint8_t IRQ_NFC = PORTD4;

static const uint8_t DTR_GSM = PORTB0;

static const uint8_t MOS_GSM = PORTD5;
static const uint8_t EN_WIFI = PORTB3;
static const uint8_t SS_LORA = PORTA4;
static const uint8_t SS_FLASH = PORTB4;
static const uint8_t CS_W5500 = PORTC4;

static const uint8_t Distributore1 = PORTD7;
static const uint8_t Distributore2 = PORTD6;
static const uint8_t PISTOLA_1 = PORTA1;
static const uint8_t PISTOLA_2 = PORTA2;
static const uint8_t BUZZER = PORTC6;
static const uint8_t ADC_IN = PORTA0;
static const uint8_t DIGITAL_OUT = PORTA3;

static const uint8_t PULSER1 = PORTA5;
static const uint8_t PULSER2 = PORTA6;

static const uint8_t RELE1 = PORTC7;
static const uint8_t RELE2 = PORTA7;

static inline void initSS_FLASH()  { DDRB |= (1 << PB4); } // set DDRB bit 4, sets PB4 for output
static inline void initSS_ETH()    { DDRC |= (1 << PC4); } // set DDRC bit 4, sets PC4 for output
static inline void enable_ETH()    { PORTC &= ~(1 << PC4); } // Set 0 Bit 4 PORTC Register
static inline void enable_FLASH()  { PORTB &= ~(1 << PB4); } // Set 0 Bit 4 PORTB Register
static inline void disable_ETH()   { PORTC |= (1 << PC4);  } // Set 1 Bit 4 PORTC Register
static inline void disable_FLASH() { PORTB |= (1 << PB4);  } // Set 1 Bit 4 PORTB Register

const byte PCA9534_I2C_ADDRESS   =  0x20;
const byte PCA9534_I2C_ADDRESS_R =  0x41;

/***********************************************************************************************/

// these macros make checking if bits are set or clear easier and more readable

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))      
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))
#define SET_BIT(byte, bit) (byte |= (1 << bit))
#define CLEAR_BIT(byte, bit) (byte &= ~(1 << bit))
#define TOGGLE_BIT(byte, bit) (byte ^= (1 << bit))

#define PN532_IRQ   (4)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield
#define NUM_OF_CONSECUTIVE_PRESSES 1
#define NUM_OF_CONSECUTIVE_NON_PRESSES 2

#define PCA9534_IP_REGISTER     0x00
#define PCA9534_OP_REGISTER     0x01
#define PCA9534_INV_REGISTER    0x02
#define PCA9534_CONF_REGISTER   0x03

volatile int intConsecutivePresses = 0;
volatile int intConsecutiveNonPresses = 0;

struct mezzoType {
  String Carb;
  String TARGA;
  uint8_t KM;
};

mezzoType mezzo;

uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };       // Buffer dove memorizzo la UID del Badge
uint8_t uidLength;                             // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
uint8_t uidMezzo[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer dove memorizzo la UID del Badge Mezzo
boolean alreadyTimbrata = false;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
uint8_t Clessidra[8] = {0x1F, 0x11, 0x0A, 0x04, 0x04, 0x0E, 0x1F, 0x1F,}; 
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

String StatoAttuale ="START...";
String strURLAPI = "";

unsigned long updatedisplayLCD = 0;

int Litri = 0;              // Variabile per il conteggio dei litri erogati
int Connected = 0;
int Autenticato = LOW;      // Semaforo Autenticazione Operatore
int InviaTarga = LOW;       // Semaforo Validazione Targa/Km Mezzo
int SgancioPistola = HIGH;  // Stato Contatto Pistola
int GO = LOW;               // Semaforo Generale
int stato_procedura = 0;    // Stato del Sistema
volatile int impulsi = 0;   // Variabile per il conteggio degli impulsi generati dal pulser

int tentativiGET = 0;
int lengthHttpResponse = 0;
int tentativi = 0;
int HTTP_len_response = 12;

String RispostaHTTP = "";
String RaccoltaDati[] = {"","","","","",""};
String Carburante = "X";
String SequenzaKeypad[] = {"X","X"};
String Risposta = "";
String Messaggio = "";
String righeDisplay[] = {"X","X","X","X"};
char MessaggioToServer[100] = "";

/************* PULSER *************************/
/**/    int ImpulsiLitro = 50;              /**/
/**/    double debounceDelay = 4.20;        /**/   // ms  debounce time; incrementare se l'output oscilla troppo
/**/    double debounceDelayBenzina = 4.20; /**/   // ms  debounce time; incrementare se l'output oscilla troppo
/**********************************************/

/********************************************************************************************/
/*                    Configurazione Rete                       */
/********************************************************************************************/
IPAddress servizio(192, 168, 5, 9);    // IP Macchina dove risiede il servizio TCP
byte ip[] = { 192, 168, 0, 50 };
IPAddress myDns(192,168,1, 21); // DNS
IPAddress gateway(192, 168, 0, 1); // GATEWAY
IPAddress subnet(255, 255, 0, 0); // SUBNET

char serverATE[] = "wbpate-test.dipvvf.it";
char serverGAC[] = "gacweb-test.dipvvf.it";

EthernetClient clientREST;

byte mac[] = {0x00, 0x0E, 0x0C, 0xB0, 0x25, 0x6F};

EthernetClient  clientToServizio; // Client Verso Servizio
EthernetServer  server(31001);    // Instanza Server CCEC
EthernetClient client;
/********************************************************************************************/

DS3231M_Class DS3231M;  
const uint8_t SPRINTF_BUFFER_SIZE =     32;  
char          inputBuffer[SPRINTF_BUFFER_SIZE];  
unsigned long secs;                                                        // store the seconds value          //
unsigned long UltimoPassaggioStato = 0;        // Timer Stati Procedura
unsigned long Timer = 0;                       // Timer
DateTime nowTimer;

// Timer Max per avanzamento stati
//**********************************
unsigned long TverificaBadge = 5;        // 5 secondi
unsigned long TinputTarga = 30;          // 30 Secondi
unsigned long TselDistributore = 30;     // 30 Secondi
unsigned long TsgancioPistola = 60;      // 60 secondi
unsigned long TmaxErogazione = 180;      // 3 minuti
unsigned long TmaxInviodati = 6;         // 6 Secondi
unsigned long TmaxProgrammingMode = 15;  // 15 Secondi
// *********************************

byte key_idx[] = {0,0};

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

byte premuto = 0;
byte prolungato = 0;
char *customKey;
String InputKey = "";
String TARGA = "";

char MappaKeys[ROWS][COLS] = { // Tastierino Definitivo
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','.'}
};
  
void InizializzaEthernet()
{
  /*****************************/
  // inizializzo ethernet shield
  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  // Ascolto presenza client
  server.begin();
  if (clientToServizio.connect(servizio, 11001)) {
    Serial.println("Connesso Al Server");
    Connected = clientToServizio.connected();
  }
  else {Serial.println("In Attesa di Client");}
  /*****************************/
}

void initIOExpander()
{
  // Write the configuration of the individual pins as inputs or outputs
  
  Wire.begin();         // join i2c bus (address optional for master)

  Wire.beginTransmission(PCA9534_I2C_ADDRESS);
  Wire.write(PCA9534_OP_REGISTER);
  Wire.write(0xC3);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9534_I2C_ADDRESS);
  Wire.write(PCA9534_INV_REGISTER);
  Wire.write(0x00);
  Wire.endTransmission();

  _delay_ms(10);
  
  Wire.beginTransmission(PCA9534_I2C_ADDRESS);
  Wire.write(PCA9534_CONF_REGISTER);
  Wire.write(0x3C); // Identifico la Riga
  //Wire.write(0xFF); // Identifico la Colonna
  Wire.endTransmission();
  
  _delay_ms(10);
  
  Wire.beginTransmission(PCA9534_I2C_ADDRESS);
  Wire.write(PCA9534_IP_REGISTER);
  Wire.endTransmission();

  Wire.begin();
}

/************************************************************/
uint32_t addr = 16101;

#define TRUE 1
#define FALSE 0


void printLine() {
  Serial.println();
  for (uint8_t i = 0; i < 125; i++) {
    Serial.print("-");
  }
  Serial.println();
}

void printTab(uint8_t _times) {
  for (uint8_t i = 0; i < _times; i++) {
    Serial.print("\t");
  }
}

void pass(bool _status) {
  printTab(1);
  Serial.print("   ");
  if (_status) {
    Serial.print("PASS");
  }
  else {
    Serial.print("FAIL");
  }
  printTab(1);
}

SPIFlash flash;

void printUniqueID(void) {
  long long _uniqueID = flash.getUniqueID();
  if (_uniqueID) {
    Serial.print("Unique ID: ");
    Serial.print(uint32_t(_uniqueID / 1000000L));
    Serial.print(uint32_t(_uniqueID % 1000000L));
    Serial.print(", ");
    Serial.print("0x");
    Serial.print(uint32_t(_uniqueID >> 32), HEX);
    Serial.print(uint32_t(_uniqueID), HEX);
  }
   printLine();
}

void FlasheraseSector(uint32_t _addr) {
	uint32_t _time /*_addr*/;
	//_addr = random(0, 0xFFFFF);
	printTab(3);
	Serial.print("Erase 4KB");
	printTab(1);
	if (flash.eraseSector(_addr)) {
		_time = flash.functionRunTime();
		pass(TRUE);

		Serial.print("Flash Diagnostic ");
		Serial.println("Erase 4KB OK");
		printTab(1);
		printLine();
		_delay_ms(1000);
	}
	else {
		pass(FALSE);
	}
}
void erogazioniSaver(uint32_t _addr,String e) {
  
  #define ARRAYSIZE 30
 
  struct Erogazioni {
    uint16_t n;
    String e1[ARRAYSIZE];
  };
  
  Erogazioni _d;
 
  _d.n = 30;
  _d.e1[1] = e; // "000;AABBCCDD;26555;D;;26.30";  
     
  uint32_t wTime = 0;
  uint32_t addr, rTime;

  addr = _addr; //random(0, 0xFFFFF);

 _delay_ms(5); 
 FlasheraseSector(addr);
 _delay_ms(5);  

 if (flash.writeAnything(addr, _d)) {
   // wTime = flash.functionRunTime();  
   Serial.println("Scrittura in memoria eseguita");  
   printLine();
  }
   
}

void erogazioniRead(uint32_t _addr) {
  
  #define ARRAYSIZE 30
 
  struct Erogazioni {
    uint16_t n;
    String e1[ARRAYSIZE];
  };

  Erogazioni _data;
     
  addr = _addr; //random(0, 0xFFFFF);
  
  /**********************************************/
  if (flash.readAnything(addr, _data))
  {
  
  printTab(3);
  Serial.print ("n° Erogazioni:" );
  printLine();
  printTab(2);  
  Serial.println("0x");
  Serial.print(_data.n,HEX); 
  Serial.println("Erogazione 1 : ");
  Serial.print(_data.e1[1]);
  pass(TRUE);
  //return _data.e1[1];
  }

  pass(FALSE);
  //return "ERROR";
  /***********************************************/
  printLine();
}

void FlashpowerDown() {
  uint32_t _time;
  printTab(3);
  Serial.print("Power Down");
  printTab(1);
  if (flash.powerDown()) {
    //_time = flash.functionRunTime();
    pass(TRUE);
  //  printTime(_time, 0);
  
  Serial.print("Flash Diagnostic ");
  Serial.print("Power Down OK");  
  _delay_ms(1000);
  printTab(1);
  printLine();  
  }
  else {
    pass(FALSE);
    printTab(2);
    Serial.print("Not all chips support power down. Check your datasheet.");
  }
}

void FlashpowerUp() {
  uint32_t _time;
  printTab(3);
  Serial.print("Power Up");
  printTab(1);
  if (flash.powerUp()) {
 //   _time = flash.functionRunTime();
    pass(TRUE);
 //   printTime(_time, 0);
  Serial.print("Flash Diagnostic ");
  Serial.print("Power Up OK");
  _delay_ms(1000);
  printTab(1);
  printLine(); 
  }
  else {
    pass(FALSE);
    printTab(2);
  }
}



void eraseChipTest() {
  uint32_t _time;
  printTab(3);
  Serial.print("Erase Chip");
  printTab(1);
  if (flash.eraseChip()) {
    //_time = flash.functionRunTime();
    pass(TRUE);
    // printTime(_time, 0);
  Serial.print("Flash Diagnostic ");
  Serial.print("Erase Chip OK");
  _delay_ms(1000);
  printTab(1);
  printLine(); 
  }
  else {
    pass(FALSE);
  }
}

bool getID() {
  Serial.println();
  uint32_t JEDEC = flash.getJEDECID();
  if (!JEDEC) {
    Serial.println("No comms. Check wiring. Is chip supported? If unable to fix, raise an issue on Github");
    return false;
  }
  else {
    Serial.print("JEDEC ID: 0x");
    Serial.println(JEDEC, HEX);
    Serial.print("Man ID: 0x");
    Serial.println(uint8_t(JEDEC >> 16), HEX);
    Serial.print("Memory ID: 0x");
    Serial.println(uint8_t(JEDEC >> 8), HEX);
    Serial.print("Capacity: ");
    Serial.println(flash.getCapacity());
    Serial.print("Max Pages: ");
    Serial.println(flash.getMaxPage());
    printUniqueID();

  lcd.clear();
  lcd.print("JEDEC ID: 0x" );
  lcd.print(JEDEC, HEX);
  lcd.setCursor(0,1);
  lcd.print("Cap: ");
  lcd.print(flash.getCapacity());
  lcd.setCursor(0,2);
  lcd.print("Memory ID: 0x");
  lcd.print(uint8_t(JEDEC >> 8), HEX);
  lcd.setCursor(0,3);
  lcd.print("Max Pages: ");
  lcd.print(flash.getMaxPage());
  _delay_ms(3000);
  }
  return true;
}

/********************************************************************************************/

void setup() {

   //Serial.begin(9600);
   Serial.println(" inizio Setup ......");
 
  initSS_ETH();
  disable_ETH();
  initSS_FLASH();
  disable_FLASH();
  _delay_ms(1);
  
  DDRC |= (1 << RELE1);  // Rele1
  DDRA |= (1 << RELE2);  // Rele2   // set PA7 e PC7 come output 

  _delay_ms(10);
  SET_BIT(PORTC,RELE1); // Apri RELE1
  _delay_ms(10);
  SET_BIT(PORTA,RELE2); // Apri RELE2
  
    
  /***************************LCD******************************/
  lcd.begin(20,4);         // Inizializza display LCD 20x4 e accendi e spegni 2 volte

  // ------- 2 blinks -------------
  for(int i = 0; i< 2; i++)
  {
    lcd.backlight();
    _delay_ms(250);
    lcd.noBacklight();
    _delay_ms(250);
  }

  lcd.createChar(1, Clessidra); // Creo CHAR Clessidra
  _delay_ms(20);
  lcd.backlight();

  for (int r=0;r<4;r++)
    righeDisplay[r]="";

  /***************************NFC*************************/ 
  nfc.begin(); // Inizializza Modulo NFC 
  
  _delay_ms(50);

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Modulo PN532 non trovato");
    lcd.backlight();
    _delay_ms(250);
    lcd.clear();
    lcd.print("Modulo PN532 non trovato ");    
    _delay_ms(2000);                            
    StatoAttuale  = " CHIAMA ASSISTENZA";   
    //while (1); // halt  // Aspetta per sempre
  } else {Serial.println("Modulo NFC OK ......");}
  
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  nfc.setPassiveActivationRetries(0xFF);   
  nfc.SAMConfig();

  /***************************SPY FLASH*************************/  

  enable_FLASH();
  
  if (flash.error()) {
    Serial.println(flash.error(VERBOSE));
  }

  flash.begin();

  if (getID()) {

    printLine();
    printTab(7);
    Serial.print("Testing library code");
    printLine();
    printTab(3);
    Serial.print("Function");
    printLine();
    printTab(2);   
    
    FlashpowerUp();
    Serial.println();    
    Serial.println();   
    eraseChipTest();
    Serial.println();   
    erogazioniSaver(addr,"Prima");     
    FlashpowerDown();
    Serial.println();
  }
  /*************************** RTC ************************/
  while (!DS3231M.begin()) {                                                  // Initialize RTC communications    //
    Serial.println(F("Unable to find DS3231MM. Checking again in 3s."));      // Show error text                  //
    _delay_ms(1000);                                                              // wait a second                    //
  } 
  _delay_ms(50);

  Serial.println(F("DS3231M initialized."));                                  //                                  //
  DS3231M.adjust();

  /*************************** POTENZIOMETRI ************************/
  Serial.println(" /*************************** POTENZIOMETRI ************************/");
  Serial.println("Inizializzo POTENZIOMETRI per livello pulser.......");
  
  Wire.begin(); // join i2c bus (address optional for master) 
  
  Wire.beginTransmission(0x28);  // (0x50) POTENZIOMETRO U11
  Wire.write(byte(0x00));        // Wiper Register
  Wire.write(50);                // Valore del potenziomentro circa 6 volt
  Wire.endTransmission(); 
   
   _delay_ms(50);     
   
   Wire.beginTransmission(0x52);  // (0x52) POTENZIOMETRO U12
   Wire.write(byte(0x00));        // Wiper Register
   Wire.write(50);                // Valore del potenziomentro circa 6 volt
   Wire.endTransmission(); 
  
   Wire.end();

   Serial.println("POTENZIOMETRI OK");

  /*************************KEYPAD*********************/
  Serial.println("/*************************KEYPAD*********************/");
  Serial.println("Inizializzo KEYPAD .......");
 
  _delay_ms(50);
  
  initIOExpander();
  
  _delay_ms(50);

  /**************** SETTING INIZIALI ******************/
      
  stato_procedura = -2; // set stato iniziale
  StatoAttuale = "Starting ...."; 

  Serial.println(StatoAttuale);
}

/********************************END SETUP ***************************************/

void my_delay_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(1);
    --ms;
  }
}

void Buzzer(uint8_t p_ripeti,uint32_t p_delay_suono) {
  
  uint32_t del = p_delay_suono;
  
  for(int volte = 0;volte<p_ripeti;volte++)
  {
    DDRC |= (1 << PC6);     // set PC6 for output
    TOGGLE_BIT(PORTC,PC6);
    my_delay_ms(p_delay_suono);
    TOGGLE_BIT(PORTC,PC6);
    /*my_delay_ms(p_delay_suono);
    TOGGLE_BIT(PORTC,PC6);
    my_delay_ms(p_delay_suono);
    TOGGLE_BIT(PORTC,PC6);*/
  }
}

/************************************KEYPAD***************************/

 void leggiRighe()
 {
   
   Wire.beginTransmission(PCA9534_I2C_ADDRESS);
   Wire.write(PCA9534_CONF_REGISTER);
   Wire.write(0xC3); // Identifico la riga
   Wire.endTransmission();
   
   _delay_ms(5);   
 }

 void leggiColonne()
 {
   Wire.beginTransmission(PCA9534_I2C_ADDRESS);
   Wire.write(PCA9534_CONF_REGISTER);
   Wire.write(0x3C); // Identifico la Colonna
   Wire.endTransmission();
   
   _delay_ms(5);
 }

 void leggi_IP_REGISTER()
 {
   Wire.beginTransmission(PCA9534_I2C_ADDRESS);
   Wire.write(PCA9534_IP_REGISTER);
   Wire.endTransmission();

   _delay_ms(5);
 }
 void leggi_OP_REGISTER()
 {
   Wire.beginTransmission(PCA9534_I2C_ADDRESS);
   Wire.write(PCA9534_OP_REGISTER);
   Wire.endTransmission();

   _delay_ms(5);
 }

void displayLCD(String righe[],int stato,int delay_lcd)                                               // get the current time             //
{
  //DateTime now = DS3231M.now();
  lcd.clear();
  //sprintf(inputBuffer,"%04d-%02d-%02d %02d:%02d:%02d", now.year(),       // Use sprintf() to pretty print    //
  //now.month(), now.day(), now.hour(), now.minute(), now.second());      // date/time with leading zeros     //
  //Serial.println(inputBuffer);                        // Display the current date/time    //
  //lcd.print(inputBuffer);
  if (stato > 2)
    lcd.print("Tempo: " + String((UltimoPassaggioStato+Timer-secs-1))+ " sec ");
  
  lcd.print((char)1);  // STAMPA LA CLESSIDRA
  lcd.setCursor(0,1);
  lcd.print(righe[1]);
  lcd.setCursor(0,2);
  lcd.print(righe[2]);
  lcd.setCursor(0,3);
  lcd.print(righe[3]);
  
  my_delay_ms(delay_lcd);
}

void avanzaStato(unsigned long p_timer) {
  Timer = p_timer;
  UltimoPassaggioStato = nowTimer.secondstime();
  stato_procedura++;
}

void getTastoPremuto_x_targa()
{
  Wire.flush();
  Wire.requestFrom(0x20, 1);
  
  String tasto = "";
  
  while(Wire.available())    // slave may send less than requested
  {
    uint8_t c = Wire.read();    // receive a byte as character
    
    //lcd.clear();
    //lcd.setCursor(0,3);
    //lcd.print("KEY : " + String(byte(c)));         // print the character
    //_delay_ms(500);

    byte key = byte(c);
    
    switch (key)
    {
      case 60:
      { /*Serial.println("RIPOSO"); delay(10); Serial.println("KEY : " + String(byte(c)));*/} // se a riposo per troppo reset premuto e prolungato
      break;
      case 195:
      { /*Serial.println("RIPOSO"); delay(10); Serial.println("KEY : " + String(byte(c)));*/} // se a riposo per troppo reset premuto e prolungato
      break;
      case 194:
      { key_idx[0] = 1; _delay_ms(5); leggiColonne(); premuto++; } // Riga 1
      break;
      case 193:
      { key_idx[0] = 2; _delay_ms(5); leggiColonne(); premuto++;}  // Riga 2
      break;
      case 67:
      { key_idx[0] = 3; _delay_ms(5); leggiColonne(); premuto++;}  // Riga 3
      break;
      case 131:
      { key_idx[0] = 4; _delay_ms(5); leggiColonne(); premuto++;}  // Riga 4
      break;
      case 199:
      { key_idx[1] = 1; _delay_ms(5); leggiRighe();  leggi_OP_REGISTER();  } // Colonna 1
      break;
      case 203:
      { key_idx[1] = 2; _delay_ms(5); leggiRighe();  leggi_OP_REGISTER();  } // Colonna 2
      break;
      case 211:
      { key_idx[1] = 3; _delay_ms(5); leggiRighe();  leggi_OP_REGISTER();  } // Colonna 3
      break;
      case 227:
      { key_idx[1] = 4; _delay_ms(5); leggiRighe();  leggi_OP_REGISTER();  } // Colonna 4
      break;
      default: leggiColonne(); break;
    }
    
    leggi_IP_REGISTER();
    
    if (premuto > 0)
    {
      InputKey = MappaKeys[key_idx[0]-1];

      if ((key_idx[0] == 4) && (key_idx[1] == 3)) // # Conferma Dati inseriti
      {
        prolungato++;
        
        if ((prolungato > 2) && (TARGA.length() == 5))
        {
          Buzzer(1,100);
          avanzaStato(TselDistributore);
        }
      }
      else if ((key_idx[0] == 4) && (key_idx[1] == 1)) // * Cancella dati digitati
      {
        prolungato++;       

        if (prolungato > 2)
        {       
          Buzzer(1,150);    
          TARGA = "";
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] = "TARGA:" + TARGA;
          lcd.clear();
          lcd.setCursor(0,3);
          lcd.print("TARGA:" + TARGA);
        }
      }
      else if ((key_idx[0] == 1) && (key_idx[1] == 4)) // A
      {
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] = "TARGA:" + TARGA;       
      }
      else if ((key_idx[0] == 2) && (key_idx[1] == 4)) // B
      {
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] = "TARGA:" + TARGA;
      }
      else if ((key_idx[0] == 3) && (key_idx[1] == 4)) // C
      {
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] = "TARGA:" + TARGA;
      }
      else if ((key_idx[0] == 4) && (key_idx[1] == 4)) // .
      {
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] =  "TARGA:" + TARGA;
      }
      else if (premuto < 2) {
          
          TARGA += InputKey.substring(key_idx[1]-1,key_idx[1]);
          righeDisplay[1] =  "****** TARGA ******";
          righeDisplay[2] =  "";
          righeDisplay[3] = "TARGA:" + TARGA;
          prolungato = 0;
          Buzzer(1,10);         
      }
      //tasto = InputKey.substring(key_idx[1]-1,key_idx[1]);      
      premuto = 0;
      //displayLCD(righeDisplay,stato_procedura,50);
    }
    _delay_ms(2); // dealy necessario per non leggere pressioni conscutive
  } 
  
  lcd.setCursor(0,0);
  lcd.print("Tempo: " + String((UltimoPassaggioStato+Timer-secs-1))+ " sec ");
  lcd.print((char)1);  // STAMPA LA CLESSIDRA
  lcd.setCursor(0,3);
  lcd.print("TARGA:" + TARGA);
}

/********************************************************************/


bool InviaRifornimento(int P_stato,int p_connesso, char P_datiVerifica[],int P_l_buffer,String P_prefisso)
{ 
  //Risposta = "999";
  Serial.println("START InviaRifornimento !!");
  
  if ((p_connesso))// && (P_stato == 7 ))
  {
    Serial.println("Connected to Server -- Invio Erogazione !!");
    stato_procedura++;
    String TX =  String(P_datiVerifica);
    char Invio[P_l_buffer];
    TX.toCharArray(Invio,P_l_buffer);
    clientToServizio.write(Invio);
    _delay_ms(20);
    clientToServizio.flush();
    //clientToServizio.stop();
    return true;
  }
  else {return false;};
}


String leggiTAG_Mezzo(bool scrivi)
{
 String app;

  
 /* app.Carb = "X";
  app.TARGA = "NULL";
  app.KM = 0;*/
  
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidMezzo, &uidLength,200);
  
  if (success) {
    
    Buzzer(2,30);
    
    if (uidLength == 4)
    {
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      success = nfc.mifareclassic_AuthenticateBlock(uidMezzo, uidLength, 4, 0, keya);

      if (success)
      {
        uint8_t data[16];
        
        // If you want to write something to block 4 to test with, uncomment
        // the following line and this text should be read back in a minute
        
        if (scrivi)
        { memcpy(data, (const uint8_t[]){ '0', '0', '0', '0', '1', 'D',0,0,0,0,0,0,0,0,0,0 }, sizeof data);
         success = nfc.mifareclassic_WriteDataBlock (4, data);
        }
         
        // Try to read the contents of block 4
        success = nfc.mifareclassic_ReadDataBlock(4, data);
        
        if (success)
        {
          Serial.println("LETTURA TARGA");
          
          String TargaRFID = (char*)data;

          Serial.println(TargaRFID);
          
          if (TargaRFID.length() == 6)
          {             
            return TargaRFID;
          }
          else
          {}
          // Aspetta prima di leggere una nuova CARD NFC
          _delay_ms(100);
        }
        else
        {
          lcd.clear();
          lcd.setCursor(0,1);
          lcd.print("Blocco KO");
        }
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0,1);
        lcd.print("Auth KO");
      }
    }
    Serial.println("");
  }
  return app;
}

String GetCodeRfidATe()
{
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;
  String Codice = "ERRORE";                       
  
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  //success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,300);
  
  if (success) {
    
    String hexCode = nfc.GetHexCode(uid, uidLength);
    hexCode.trim(); 
    Codice = hexCode.substring(0,8);
    alreadyTimbrata = true; 
  }

  return Codice;
}

String GetHTTPResponseCode(String HttpResp)
{
  String Cod_Conn_KO = "CONN KO";

  //Serial.println("VerificaHTTPResponse...");
  if (HttpResp.length() == HTTP_len_response){
    String CodHTTP = HttpResp.substring(HttpResp.length()-3);
    return CodHTTP;
  }
  else
  {return Cod_Conn_KO;}
}

uint8_t GetAteValidation(int Port,char serverWEB[],EthernetClient ClientHTTP,String ATeCode)
{
 int valida = 0;

 if ( (ClientHTTP.connect(serverWEB, Port))) 
  {      
    //strURLAPI = "GET /api/ATe/GetAssociazioneByRfidCode?rfidCode=63cfe34d HTTP/1.1\r\n";
    //strURLAPI = "GET /api/ATe/GetAssociazioneByRfidCode?rfidCode=63CFE34D HTTP/1.1\r\n";
    strURLAPI = "GET /api/ATe/GetAssociazioneByRfidCode?rfidCode=" + ATeCode +"  HTTP/1.1\r\n";
    strURLAPI += "Host: wbpate-test.dipvvf.it";
    strURLAPI += "\r\n";
    strURLAPI += "Accept: application/json";
    strURLAPI += "\r\n";
    strURLAPI += "accept-encoding: gzip, deflate";
    strURLAPI += "\r\n";
    strURLAPI += "accept-language: en-US,en;q=0.8";
    strURLAPI += "\r\n";
    strURLAPI += "user-agent: Mozilla/5.0 (Windows NT 6.3; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) advanced-rest-client/12.1.4 Chrome/61.0.3163.100 Electron/2.0.2 Safari/537.36";
    strURLAPI += "\r\n";
    strURLAPI += "content-type: application/json";
    strURLAPI += "\r\n";
  
    ClientHTTP.print(strURLAPI);
  
    _delay_ms(80);
    ClientHTTP.println("Connection: close");
    ClientHTTP.println();
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Connessione Fallita.");
    lcd.setCursor(0,3);
    lcd.print("Verificare....");
    _delay_ms(1000);
  }

  _delay_ms(100);

  while (ClientHTTP.available()) {
    char c = ClientHTTP.read();
    RispostaHTTP = RispostaHTTP + c;
    //lcd.setCursor(0,2);
    //lcd.print(RispostaHTTP);
    if (RispostaHTTP.length() == HTTP_len_response)
    {
      String rispostaGetTimbrature = GetHTTPResponseCode(RispostaHTTP);
      _delay_ms(80);      
      /*lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("COD HTTP:");
      lcd.print(rispostaGetTimbrature);*/
    
      if (rispostaGetTimbrature == "200"){ valida = 1; }
      _delay_ms(80);
    }
  }
  return valida;
}

uint8_t GetMezzoValidation(int Port,char serverWEB[],EthernetClient ClientHTTP,String P_Targa)
{
 int valida = 0;

 if ( (ClientHTTP.connect(serverWEB, Port))) 
  {
    /*lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Server OK");
    lcd.setCursor(0,3);
    lcd.print(P_Targa);
    _delay_ms(200);*/
        
    //_delay_ms(80);
  
    strURLAPI = "GET /gac-servizi/integrazione/SO115/AnagraficaMezzi/prova HTTP/1.0\r\n";
    strURLAPI += "Host: gacweb-test.dipvvf.it";
    strURLAPI += "\r\n";
    strURLAPI += "Authorization: fjhkhk";
    strURLAPI += "\r\n";    
    strURLAPI += "Accept: application/json";
    strURLAPI += "\r\n";
    strURLAPI += "accept-encoding: gzip, deflate";
    strURLAPI += "\r\n";
    strURLAPI += "accept-language: en-US,en;q=0.8";
    strURLAPI += "\r\n";
    strURLAPI += "user-agent: Mozilla/5.0 (Windows NT 6.3; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) advanced-rest-client/12.1.4 Chrome/61.0.3163.100 Electron/2.0.2 Safari/537.36";
    strURLAPI += "\r\n";
    strURLAPI += "content-type: application/json";
    strURLAPI += "\r\n";
  
    ClientHTTP.print(strURLAPI);
  
    _delay_ms(80);
    ClientHTTP.println("Connection: close");
    ClientHTTP.println();
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Connessione Fallita.");
    lcd.setCursor(0,3);
    lcd.print("Verificare....");
    _delay_ms(1000);
  }

  _delay_ms(100);

  while (ClientHTTP.available()) {
    char c = ClientHTTP.read();
    RispostaHTTP = RispostaHTTP + c;
    if (RispostaHTTP.length() == HTTP_len_response)
    {
      String rispostaGetTimbrature = GetHTTPResponseCode(RispostaHTTP);
      _delay_ms(80);    
      
      /*lcd.clear();
      lcd.setCursor(0,2);
      lcd.print("COD HTTP:");
      lcd.print(rispostaGetTimbrature);*/ 
        
      if (rispostaGetTimbrature == "200"){ valida = 1; }
      _delay_ms(80);
    }
  }
  return valida;
}

void abilitaPulsanti(){
  /****************ABILITO PULSANTI**************/
  DDRD &= ~(1 << PD7);    // sets PD7 for input Distributore1
  DDRD &= ~(1 << PD6);    // sets PD6 for input Distributore2
  PCICR =  0b00001000;
  PCMSK3 = 0b11000000;
  sei();            // enable interrupts
  /**********************************************/
}

void abilitaContattiPistola(){
  /****************ABILITO PULSANTI**************/
  DDRA &= ~(1 << PISTOLA_1);    // sets PA1 for input 
  DDRA &= ~(1 << PISTOLA_2);    // sets PA2 for input 
  //PCICR =  0b00000001;
  //PCMSK0 = 0b00000110;
  //sei();            // enable interrupts
  /**********************************************/
}

void abilitaPulser(char p_carburante)
{
  PCICR =  0b00000001;
  
  if (p_carburante == 'D')
  {
    DDRA &= ~(1 << PA5);  // PULSER 1 clear DDRA bit 5, sets PA5 for input
    PCMSK0 = 0b00100000;  // pulser 1 PCINT5
  }
  else
  {
    DDRA &= ~(1 << PA6);  // PULSER 2 clear DDRA bit 5, sets PA5 for input
    PCMSK0 = 0b01000000;  // pulser 2 PCINT6
  }
  
  sei();            // enable interrupts
};

void ContaImpulsi()
{
  //if (((millis() - lastDebounceTime) > debounceDelay))
  {
    impulsi++;
    //lastDebounceTime = millis();
    _delay_ms(4);
  }
}

double impulsiToLitri(int P_impulsi)
{
  //double imp = (double)(P_impulsi-1);
  double imp = (double)(P_impulsi);
  if (imp < 0) {imp = 0;}
  double lt = (imp / ImpulsiLitro);
  double totale = lt;
  return totale;
}

void Rele_Abilitazione1(int p_azione,int p_bit) {
  
  DDRC |= (1 << PC7);  // Rele1 
  
  switch (p_azione) {
    case 0: // chiudi relè
    {     
      CLEAR_BIT(PORTC,PC7); // Rele1*/
      _delay_ms(50);
    }
    break;
    case 1: // apri relè
    {
      SET_BIT(PORTC,p_bit); // Rele1
      _delay_ms(50);      
    }
    break;
    case 2: // chiudi e apri relè
    {
      CLEAR_BIT(PORTC,p_bit); // Rele1
      _delay_ms(300);
      SET_BIT(PORTC,p_bit); // Rele1
    }
    break;
  }
}

void Rele_Abilitazione2(int p_azione,int p_bit) {
  
  DDRA |= (1 << PA7);  // Rele2
  
  switch (p_azione) {
    case 0:
    {
      CLEAR_BIT(PORTA,p_bit); // Rele2
      _delay_ms(50);
    }
    break;
    case 1:
    {
      SET_BIT(PORTA,p_bit); // Rele2
      _delay_ms(50);
    }
    break;
    case 2:
    {
      CLEAR_BIT(PORTA,p_bit); // Rele2
      _delay_ms(300);
      SET_BIT(PORTA,p_bit); // Rele2      
    }
    break;
  }
}

void Control_WIFI(int azione) {
   DDRB |= (1 << PB3);  // set DDRB bit 3,  sets PB3 for output
   _delay_ms(30);
   if (azione == 1) {SET_BIT(PORTB,3);} else if (azione == 0) { CLEAR_BIT(PORTB,3); }
}

void Azzera()
{
   RispostaHTTP = "";
   impulsi = 0;
   alreadyTimbrata = false;
   
   Carburante = "X";
   SequenzaKeypad[0] = "";
   SequenzaKeypad[1] = "";
   
   Rele_Abilitazione1(1,7);
   Rele_Abilitazione2(1,7);
   Control_WIFI(0);
   
   clientToServizio.flush();
   clientToServizio.stop();
   clientREST.flush();
   clientREST.stop();
   Connected = false;
   
   enable_FLASH();
   _delay_ms(5);
   disable_FLASH();
   _delay_ms(5);
   disable_ETH();
   _delay_ms(5); 
   enable_ETH(); 
     
   SET_BIT(PORTA,A1);
   SET_BIT(PORTA,A2);
   
   righeDisplay[1] =  "";
   righeDisplay[2] =  "";
   righeDisplay[2] =  "";
   
   lcd.noBacklight();
   lcd.noDisplay();
   secs = 0;
   UltimoPassaggioStato = 0;
   
   printLine();
   Serial.println("Azzera....... OK");
   printLine();
   
   stato_procedura = -2;
}

void CompletoRifornimentoPerInvioDati(int P_stato)
{
    impulsi = 0;
    Litri = 0;
    Messaggio.concat("\r\n");
    _delay_ms(100);
    Messaggio.toCharArray(MessaggioToServer, 100);  
}

/**************************LOOP PROCEDURA************************************/
void loop() {
      
  switch (stato_procedura) {
    case -2:
    { //cli(); // disable interrupt      
            
      righeDisplay[1] =  "";
        righeDisplay[2] = "Setting....";
      righeDisplay[3] =  "";
      
      displayLCD(righeDisplay,stato_procedura,100);
      
      _delay_ms(200);     
      stato_procedura++;
    }
    break;
    case -1:
    {   
      righeDisplay[1] =  "";
      righeDisplay[2] =  "";
      righeDisplay[3] =  "";
      
      abilitaPulsanti();
      _delay_ms(20);
      abilitaContattiPistola();
      stato_procedura++;
    }
    break;
    case 0:
    { 
      righeDisplay[1] =  "";
      righeDisplay[2] =  "";
      righeDisplay[3] =  "";
          
      displayLCD(righeDisplay,stato_procedura,100);
      _delay_ms(200);
      alreadyTimbrata = false;  
      stato_procedura++;
    }
    break;
    case 1:
    {   
      righeDisplay[1] = " * AUTENTICAZIONE *";
      righeDisplay[2] = "";
      righeDisplay[3] = "    Avvicina ATE  ";
     
      
      displayLCD(righeDisplay,stato_procedura,100);     
      
      String ATe = "ERRORE";

      if (!alreadyTimbrata) {ATe = GetCodeRfidATe(); Buzzer(2,100);}
      
        if ((ATe != "ERRORE") && (BIT_IS_CLEAR(PORTC,4)))
        { 
           Serial.println("");
           Serial.print("***************************************************************");
           Serial.println(" Tessera ID : " + ATe);
           Serial.print("***************************************************************");
           Serial.println("Riconoscimento Tessera .............");
                     
           RaccoltaDati[0] = ATe;
           
           lcd.backlight();
           lcd.display();          
           _delay_ms(10);
            
           righeDisplay[1] = " * AUTENTICAZIONE *";
           righeDisplay[2] = "";
           righeDisplay[3] = "   Rfid: " + ATe;
         
             displayLCD(righeDisplay,stato_procedura,100);
           Ethernet.begin(mac, ip, myDns, gateway, subnet);
            // give the WIZ5500 a second to initialize
            _delay_ms(1000);
         }

         // Effettua chiamata REST per validare CARD NFC
         // Se la CARD è valida memorizza in memorria l'operazione e prosegui
         // Altrimenti Memorizza in Memoria e Azzera la procedura.
      
         stato_procedura++; // da commentare
         
         /****************************************************
         if (GetAteValidation(80,serverATE,clientREST,ATe)) 
          { 
            SET_BIT(PORTC,PC4);
            
            //Buzzer(1,400); 
            
            righeDisplay[1] =  "****** ESITO *****";
            righeDisplay[2] =  "";
            righeDisplay[3] = "Utente Riconosciuto";
            
            displayLCD(righeDisplay,stato_procedura,100);
            _delay_ms(1000);
            
            avanzaStato(TinputTarga); 
          } 
         else 
          { 
            //Buzzer(3,200);
            
            righeDisplay[1] =  "****** ESITO *****";
            righeDisplay[2] =  "";
            righeDisplay[3] = "Utente Sconosciuto";
            
            displayLCD(righeDisplay,stato_procedura,100);
            _delay_ms(1000);
            Azzera();
           }   
          *****************************************************/
    }
    break;
    case 2:
    {   
      disable_ETH();
      _delay_ms(2);
      enable_ETH();

      // da commentare
      // Carburante = "D"; // Simulo Abilitazione Diesel
      // da commentare
      // Carburante = "B"; // Simulo Abilitazione Benzina
      
      TARGA = "";
      righeDisplay[1] =  "****** TARGA ******";
      righeDisplay[2] =  "";
      righeDisplay[3] = "TARGA:";
      displayLCD(righeDisplay,stato_procedura,10);
      
      avanzaStato(TinputTarga);
    }
    break;
    case 3:
    {       
       getTastoPremuto_x_targa();
      String mezzoString = leggiTAG_Mezzo(false); // con TRUE scrive sul blocco 4 della card NFC
      _delay_ms(10);

      Serial.println(mezzoString);
      
      mezzo.Carb = mezzoString.substring(5);
      mezzo.TARGA = mezzoString.substring(0,5);
      mezzo.KM = 0;

      Serial.println("MEZZO INSERITO : ");
      Serial.println("TIPO CARBURANTE: " + mezzo.Carb);    
      Serial.println("TARGA: " + mezzo.TARGA);              

      Carburante = mezzo.Carb;                 
      if ((mezzo.Carb == "B") || (mezzo.Carb == "D")) {
        RaccoltaDati[1] = mezzo.TARGA;
        RaccoltaDati[2] = mezzo.Carb;
        avanzaStato(TselDistributore); 
      }     
      
      // da commentare
      // avanzaStato(TselDistributore);  
    }
    break;
    case 4:
    { 
    righeDisplay[1] =  "****** DISTRIBUTORE ******";
    righeDisplay[2] =  "";
    righeDisplay[3] = "**** SCEGLI ****";

    displayLCD(righeDisplay,stato_procedura,100);

    // Verifica scelta distributore
    
      if (mezzo.Carb == "B")
      {
        abilitaPulser('B');
        Rele_Abilitazione2(0,7); // chiudi relè
        StatoAttuale = "BENZINA";
        avanzaStato(10);
      }
      else if (mezzo.Carb == "D")
      {
        abilitaPulser('D');
        Rele_Abilitazione1(0,7); // chiudi relè
        StatoAttuale = "GASOLIO";
        avanzaStato(10);
      }                          
    }
    break;
    case 5:
    {             
      // VALIDA MEZZO CON WBSERVICES
      /*clientREST.stop();
      clientREST.flush();

      if (GetMezzoValidation(443,serverGAC,clientREST,mezzo.TARGA)) 
          { 
            SET_BIT(PORTC,PC4);
            
            Buzzer(1,400); 
            
            righeDisplay[1] =  "****** ESITO *****";
            righeDisplay[2] =  "";
            righeDisplay[3] = "TARGA Riconosciuta";
            
            displayLCD(righeDisplay,stato_procedura,100);
            _delay_ms(1000);
            
            avanzaStato(TmaxErogazione); 
          } 
         else 
          { 
            Buzzer(3,100);
            
            righeDisplay[1] =  "****** ESITO *****";
            righeDisplay[2] =  "";
            righeDisplay[3] = "TARGA Sconosciuta";
            
            displayLCD(righeDisplay,stato_procedura,100);
            _delay_ms(1000);
            Azzera();
           }   
      */
      
      impulsi = 0;
      
      righeDisplay[1] = "**** Distributore ****";
      righeDisplay[2] =  "";
      righeDisplay[3] = "***** " + StatoAttuale + " *****";
      
      displayLCD(righeDisplay,stato_procedura,100);

      avanzaStato(20);
    }
    break;
    case 6:
    { 
      double lt = impulsiToLitri(impulsi);      
      
      righeDisplay[1] = "LITRI :" + String(lt);
      righeDisplay[2] = "";     
      righeDisplay[3] = "Erogazione: " + StatoAttuale;
      
      displayLCD(righeDisplay,stato_procedura,100);   

      /* CONTATTO PISTOLA DIESEL*/
      
      if ((PINA & _BV(PA1)) && (Carburante == "D"))
      {       
        RaccoltaDati[3] = String(lt);
        
        StatoAttuale = "STOP EROGAZIONE";
        Rele_Abilitazione2(1,7); //  apri relè
        Rele_Abilitazione1(1,7); //  apri relè  
        TOGGLE_BIT(PORTA,1);      
        avanzaStato(10);
      }
      
      /* CONTATTO PISTOLA BENZINA*/
      
      if  ((PINB & _BV(PB1)) && (Carburante == "B"))
      {
        RaccoltaDati[3] = String(lt);
        
        StatoAttuale = "STOP EROGAZIONE";
        Rele_Abilitazione2(1,7); //  apri relè
        Rele_Abilitazione1(1,7); //  apri relè        
        avanzaStato(10);
      }     
    }
    break;
    case 7 :
    {       
      /**************************************
       Control_WIFI(1);
       _delay_ms(2000);
      /**************************************/
      
      righeDisplay[1] =  "";      
      righeDisplay[2] = "Invio........";
      righeDisplay[3] =  "";
          
      if (BIT_IS_CLEAR(PORTC,4)) 
      {
        displayLCD(righeDisplay,stato_procedura,10);  
        InizializzaEthernet();
        _delay_ms(1000);
        //Control_WIFI(0);
        
        Messaggio = ""; 
        
        for (int k = 0;k<4;k++)
          Messaggio.concat(RaccoltaDati[k]+";");        
        
        //Messaggio = "000;2149016745;00001;2658;Diesel;70.00";
        CompletoRifornimentoPerInvioDati(stato_procedura);
        
        if(InviaRifornimento(stato_procedura,Connected,MessaggioToServer,100,""))
        { 
          // SET_BIT(PORTC,PC4);

          disable_ETH();
          
          righeDisplay[1] = "";
          righeDisplay[2] = " Dati Inviati ";
          righeDisplay[3] =  "";
          
          displayLCD(righeDisplay,stato_procedura,100);
          
          _delay_ms(20);     
          
          Azzera();
        }
        else { stato_procedura++; }        
      }
    }
    break;
    case 8:
    { 
          righeDisplay[1] =  "";
          righeDisplay[2] = "Salvo Dati........";
          righeDisplay[3] =  "";  
          displayLCD(righeDisplay,stato_procedura,10);
          /*******************************/
          _delay_ms(5);
          disable_ETH();
          Serial.println("ETH Disabilitata");
          /*******************************/
          _delay_ms(50);
          enable_FLASH();
          Serial.println("FLASH Ablitata");
          printLine();
          /******************************/        
          FlashpowerUp(); 
          _delay_ms(5); 
          //FlasheraseSector(addr);
          //_delay_ms(5);   
          erogazioniSaver(addr,Messaggio);
          _delay_ms(5);
          FlashpowerDown();          
          printLine();
          /*****************************/
          Azzera();
    }
    break;
    case 9:
    {   
    }
    break;
    case 100:
    {   
    }
    break;
    case 101:
    {      
    }
    break;
  }

  nowTimer = DS3231M.now();
  secs = nowTimer.secondstime();
  if ((UltimoPassaggioStato+Timer-secs) <= 1) Azzera();

}

/********************FINE LOOP PROCEDURA************************************/

// interrupt per conteggio impulsi

ISR(PCINT0_vect) {
       if (PINA & _BV(PA5)){
              impulsi++;
       }
       if (PINA & _BV(PA6)){
              impulsi++;
       }
}

// interrupt per pulsanti abilitazione diesele benzina

ISR(PCINT3_vect) {    
  
  if (stato_procedura == 4)
  {
    if (PIND & _BV(PD6))
    {   
      intConsecutivePresses++;                      // increment counter for number of presses
      if(intConsecutivePresses >= NUM_OF_CONSECUTIVE_PRESSES) 
      {     // if enough presses to constitute a press
        abilitaPulser('B');
        Rele_Abilitazione2(0,7); // chiudi relè
        Carburante = "B";
        StatoAttuale = "BENZINA";
        //stato_procedura++;
        avanzaStato(10);                            
        intConsecutivePresses = 0;                    // and reset press counts
        intConsecutiveNonPresses = 0;
      }
    }
    else  {           // else if button is not pressed (logic low)
    intConsecutiveNonPresses++;
    if(intConsecutiveNonPresses >= NUM_OF_CONSECUTIVE_NON_PRESSES) {
      intConsecutivePresses = 0;                      // reset press counts
      intConsecutiveNonPresses = 0;
    }
    }     
      
    if (PIND & _BV(PD7))
    {     
      intConsecutivePresses++;                      // increment counter for number of presses
      if(intConsecutivePresses >= NUM_OF_CONSECUTIVE_PRESSES) 
      {     // if enough presses to constitute a press        
        abilitaPulser('D');
        Rele_Abilitazione1(0,7); // chiudi relè
        Carburante = "D";
        StatoAttuale = "GASOLIO";
        //stato_procedura++;
        avanzaStato(10);
        intConsecutivePresses = 0;                    // and reset press counts
        intConsecutiveNonPresses = 0;
        }
     }
     else  {            // else if button is not pressed (logic low)
     intConsecutiveNonPresses++;
     if(intConsecutiveNonPresses >= NUM_OF_CONSECUTIVE_NON_PRESSES) {
       intConsecutivePresses = 0;                     // reset press counts
       intConsecutiveNonPresses = 0;
     }
    }
  }               
}