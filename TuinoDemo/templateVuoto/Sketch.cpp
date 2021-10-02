#include <Arduino.h>
#ifndef F_CPU               // if F_CPU was not defined in Project -> Properties
#define F_CPU 8000000UL    // define it now as 8 MHz unsigned long
#endif

/*********************************************************************************************/
#include <avr/io.h>       // this is always included in AVR programs
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
/*********************************************************************************************/
#include <SPI.h>
#include <Wire.h>
#include <Ethernet2.h>
#include <NFC_PN532.h>
#include <LiquidCrystal_I2C.h>
#include <DS3231M.h>
#include <PCA9534.h>
#include <my_EEPROM.h>
/*********************************************************************************************/

static const uint8_t D42 = 42;

static const uint8_t IRQ_Module = PORTB1;
static const uint8_t IRQ_TASTIERA = PORTB2;
static const uint8_t IRQ_RTC = PORTC5;
static const uint8_t IRQ_NFC = PORTD4;
/*********************************************************************************************/
static const uint8_t DTR_GSM = PORTB0;
static const uint8_t MOS_GSM = PORTD5;
static const uint8_t EN_WIFI = PORTB3;
static const uint8_t SS_LORA = PORTA4;
static const uint8_t SS_FLASH = PORTB4;
static const uint8_t CS_W5500 = PORTC4;
/*********************************************************************************************/
static const uint8_t Distributore1 = PORTD7;
static const uint8_t Distributore2 = PORTD6;
static const uint8_t PISTOLA_1 = PORTA1;
static const uint8_t PISTOLA_2 = PORTA2;
static const uint8_t BUZZER = PORTC6;
static const uint8_t ADC_IN = PORTA0;
static const uint8_t DIGITAL_OUT = PORTA3;
/*********************************************************************************************/
static const uint8_t PULSER1 = PORTA5;
static const uint8_t PULSER2 = PORTA6;
/*********************************************************************************************/
static const uint8_t RELE1 = PORTC7;
static const uint8_t RELE2 = PORTA7;
/*********************************************************************************************/
static inline void initSS_FLASH()  { DDRB |= (1 << SS_FLASH); } // set DDRB bit 4, sets PB4 for output
static inline void initSS_ETH()    { DDRC |= (1 << CS_W5500); } // set DDRC bit 4, sets PC4 for output
static inline void initSS_LORA()   { DDRA |= (1 << SS_LORA); }
static inline void initSS_MOSGSM() { DDRD |= (1 << MOS_GSM); }
static inline void initSS_DTRGSM() { DDRB |= (1 << DTR_GSM); }
static inline void initSS_WIFI()   { DDRB |= (1 << EN_WIFI); }
	
static inline void enable_ETH()    { PORTC &= ~(1 << CS_W5500); } // Set 0 Bit 4 PORTC Register
static inline void enable_FLASH()  { PORTB &= ~(1 << SS_FLASH); } // Set 0 Bit 4 PORTB Register

static inline void disable_ETH()   { PORTC |= (1 << CS_W5500);	} // Set 1 Bit 4 PORTC Register
static inline void disable_FLASH() { PORTB |= (1 << SS_FLASH);  }
static inline void disable_LORA()  { PORTA |= (1 << SS_LORA);	}
static inline void disable_MOSGSM(){ PORTD &= ~(1 << MOS_GSM);	}
static inline void disable_DTRGSM(){ PORTB |= (1 << DTR_GSM);	}
static inline void disable_WIFI()  { PORTB &= ~(1 << EN_WIFI);  }	
	
/***********************************************************************************************/
#define testbit(var, bit)	( (var & (1 <<(bit)))!=0 )
#define bitislow(var, bit)	( (var & (1 <<(bit)))==0 )
#define bitishigh(var, bit)	( (var & (1 <<(bit)))==1 )

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))
#define SET_BIT(byte, bit) (byte |= (1 << bit))
#define CLEAR_BIT(byte, bit) (byte &= ~(1 << bit))
#define TOGGLE_BIT(byte, bit) (byte ^= (1 << bit))

#define PN532_IRQ   (4)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield
#define NUM_OF_CONSECUTIVE_PRESSES 1
#define NUM_OF_CONSECUTIVE_NON_PRESSES 2

#define stato_distributore 4 // Stato sistema pr la scelta del distributore
#define stato_erogazione 7	 // Stato Sistema in fase di erogazione

volatile int intConsecutivePresses = 0;
volatile int intConsecutiveNonPresses = 0;
const uint8_t I2C_PCA9534_ADDR = 0x20;

struct mezzoType {
  String Carb;
  String TARGA;
  String KM;
};

mezzoType mezzo;

uint8_t success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };       // Buffer dove memorizzo la UID del Badge
uint8_t uidLength;                             // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
uint8_t uidMezzo[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer dove memorizzo la UID del Badge Mezzo
boolean alreadyTimbrata = false;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
uint8_t Clessidra[8] = {0x1F, 0x11, 0x0A, 0x04, 0x04, 0x0E, 0x1F, 0x1F,};
NFC_PN532 nfc(PN532_IRQ, PN532_RESET);

String StatoAttuale = "START...";
String strURLAPI = "";

unsigned long updatedisplayLCD = 0;

int Litri = 0;              // Variabile per il conteggio dei litri erogati
//int Connected = 0; // elimina
//int Autenticato = LOW;      // Semaforo Autenticazione Operatore // elimina
int InviaTarga = LOW;       // Semaforo Validazione Targa/Km Mezzo
int SgancioPistola = HIGH;  // Stato Contatto Pistola
// int GO = LOW;               // Semaforo Generale // elimina
int stato_procedura = 0;    // Stato del Sistema
volatile uint16_t impulsi = 0;   // Variabile per il conteggio degli impulsi generati dal pulser

uint8_t distr_selezionato = 0;
/*** GESTIONE HTTP REQUEST ***/

int HTTP_len_response = 12;
String RispostaHTTP = "";
// Ate;TARGA;CARB;LT;KM;
String RaccoltaDati[] = {"", "", "", "", "", ""};
// String Carburante = "X";
String Risposta = "";
String Messaggio = "";
String righeDisplay[] = {"X", "X", "X", "X"};

/************* PULSER *************************/
 int ImpulsiLitro = 100;	                /**/
 uint16_t MaxErogabile = 100000;	   	    /**/
/**********************************************/
char CodSede[] = "SA1001";
/********************************************************************************************/
/*                    Configurazione Rete                       */
/********************************************************************************************/

IPAddress ipCCEC(192, 168, 3, 100);
//IPAddress ipCCEC(192, 168, 0, 50);
IPAddress myDns(192, 168, 1, 21); // DNS
IPAddress gateway(192, 168, 0, 1); // GATEWAY
IPAddress subnet(255, 255, 0, 0); // SUBNET

char serverATE[]  = "wbpate-test.dipvvf.it";
char serverGAC[]  = "gacweb-test.dipvvf.it";
char serverREST[] = "ccec.no.dipvvf.it";
//char serverREST[] = "ccec.sa.dipvvf.it";

EthernetClient clientLOCAL;
EthernetClient clientATE;

byte mac[] = {0x00, 0x0E, 0x0C, 0xB0, 0x25, 0x6F};
/********************************************************************************************/
/*                    Real Time Clock                       */
/********************************************************************************************/
DS3231M_Class DS3231M;
const uint8_t SPRINTF_BUFFER_SIZE =     32;
char          inputBuffer[SPRINTF_BUFFER_SIZE];
unsigned long secs;                            // store the seconds value
unsigned long UltimoPassaggioStato = 0;        // Timer Stati Procedura
unsigned long Timer = 0;                       // Timer
DateTime nowTimer;
uint32_t temperatura = 0;
/********************************************************************************************/
/*                     Timer avanzamento stati
/********************************************************************************************/
unsigned long TverificaBadge = 60;        // 60 secondi
unsigned long TinputTarga = 120;          // 120 Secondi
unsigned long TinputKM = 120;             // 120 Secondi
unsigned long TselDistributore = 120;     // 120 Secondi
unsigned long TsgancioPistola = 60;       // 60 secondi
unsigned long TmaxErogazione = 600;       // 10 minuti
unsigned long TmaxInviodati = 30;         // 30 Secondi
unsigned long TmaxProgrammingMode = 30;   // 30 Secondi
unsigned long TmaxSalvataggio = 30;       // 30 Secondi
/********************************************************************************************/

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns

String TARGA = "";
String KM = "";

/******** MAPPA TASTIERINO ************/

char MappaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', '.'}
};

PCA9534 gpio;
/********************************************************************************************/
void InizializzaEthernet()
{
  /***********************************/
  // inizializzo ethernet MICRO W5500
  /***********************************/
  Ethernet.begin(mac, ipCCEC, myDns, gateway, subnet);

}

/************************************************************/
#define TRUE 1
#define FALSE 0

void my_delay_ms(int ms)
{
  while (0 < ms)
  {
    _delay_ms(1);
    --ms;
  }
}
/********************UTILITY x DEBUG Seriale ****************************************/
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

bool write_eeprom_string_struct(ParametriCCEC_TypeDef dato) {

	int lunBuffer = dato.da_memorizzare.length();
	char buf[lunBuffer];
	dato.da_memorizzare.toCharArray(buf, lunBuffer + 1);
	bool out = false;
	int i = 0;

	Serial.println(" len: " + String(lunBuffer));
	Serial.println("Eseguo scrittura nella EEPROM");
	Serial.println(" ");

	for (int ind = dato.startIND ; ind < (dato.startIND + lunBuffer); ind++) {
		if (buf[i] != 0) {
			EEPROM.write(ind, buf[i]);
			//Serial.print(" " + String(buf[i]));
		}
		i++;
	}
	Serial.println(" ");

	return true;
}

void clearEEPROM(int ind_from,int ind_to) {
	Serial.println(" ");
	Serial.println(" ");
	Serial.println("Eseguo la clear della EEPROM");
	for (int i = ind_from ; i < ind_to ; i++) {
		EEPROM.write(i, 0);
	}
	Serial.println("Clear completata");
	Serial.println(" ");
	Serial.println(" ");
}

void WDT_off(void){
	cli();
	// wdt_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE *//* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */WDTCSR = 0x00;
	sei();
}

void WDT_Prescaler_8Sec(void) {
	cli();
	// wdt_reset();
	/* Start timed  equence */
	WDTCSR |= (1<<WDCE) | (1<<WDE) | (1<<WDIE);
	/* Set new prescaler(time-out) value = 1024K cycles (~8 s) */
	WDTCSR  = (1<<WDE) | (1<<WDIE) | (1<<WDP3) | (1<<WDP0);
	sei();
	}
/************************************************************/

void setup() {
	
	WDT_off();
	
	/************************************************************/
	/*  INIT PIN PERIFERICHE										*/
	/************************************************************/
	initSS_LORA();
	_delay_ms(10);
	initSS_MOSGSM();
	_delay_ms(10);
	initSS_FLASH;
	_delay_ms(10);
	initSS_WIFI();
	_delay_ms(10);
	initSS_ETH();
	_delay_ms(10);
	initSS_MOSGSM();
	_delay_ms(10);
	initSS_DTRGSM();

   /************************************************************/
   /*  DISABILITO PERIFERICHE								   */
   /************************************************************/
   _delay_ms(100);
   disable_ETH();
   _delay_ms(100);
   disable_FLASH();
   _delay_ms(100);	
   disable_DTRGSM();
   _delay_ms(100);
   disable_FLASH();
   _delay_ms(100);
   disable_LORA();
   _delay_ms(100);
   disable_WIFI();
   _delay_ms(100);
   /*******************************************************************************************/
  // Serial.begin(115200);
  _delay_ms(100);
  Serial.println(" inizio Setup ......");
  printLine();

  /*******************************************************************************************/
  ParametriCCEC = Parametri;  
  printLine();
  Serial.print("EEPROM utilizzata (byte): ");
  Serial.print(EEPROM.length());
  Serial.println(" ");
  Serial.print("Scrittura Parametri CCEC");

  String app = "";

 clearEEPROM(0,EEPROM.length());
 if (write_eeprom_string_struct(ParametriCCEC[0])) { Serial.println("WRITE OK");}
 if (write_eeprom_string_struct(ParametriCCEC[1])) { Serial.println("WRITE OK");}
 if (write_eeprom_string_struct(ParametriCCEC[2])) { Serial.println("WRITE OK");}
 if (write_eeprom_string_struct(ParametriCCEC[3])) { Serial.println("WRITE OK");}
 if (write_eeprom_string_struct(ParametriCCEC[4])) { Serial.println("WRITE OK");}
 printLine();  
  
/*******************************************************************************************/
  DDRC |= (1 << BUZZER); // set pin BUZZER (PC6) for output
  DDRC |= (1 << RELE1);  // Rele1
  DDRA |= (1 << RELE2);  // Rele2   // set PA7 e PC7 come output
  _delay_ms(10);
  SET_BIT(PORTC, RELE1); // Apri RELE1
  _delay_ms(10);
  SET_BIT(PORTA, RELE2); // Apri RELE2
  printLine();
  /***************************LCD******************************/
  lcd.begin(20, 4);	 // Inizializza display LCD 20x4 e accendi e spegni 2 volte

  // ------- 2 blinks -------------
  for (int i = 0; i < 2; i++)
  {
    lcd.backlight();
    _delay_ms(250);
    lcd.noBacklight();
    _delay_ms(250);
  }

  lcd.createChar(1, Clessidra); // Creo CHAR Clessidra
  _delay_ms(20);
  lcd.backlight();

  for (int r = 0; r < 4; r++)
    righeDisplay[r] = "";

  printLine();

  /***************************NFC*************************/

  nfc.begin();

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
  } else {
    Serial.println("Modulo NFC OK ......");
  }

  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);

  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();
  printLine();
  
  /*************************** RTC **********************************/
  while (!DS3231M.begin()) {
    Serial.println(F("non trovo modulo RTC DS3231MM. Riprovo tra 3s."));
    _delay_ms(1000);
  }
  _delay_ms(50);

  Serial.println(F("RTC chip DS3231M initialized."));
  DS3231M.adjust();
  printLine();
  /*************************** POTENZIOMETRI ************************/
  Serial.println("Inizializzo POTENZIOMETRI per livello pulser.......");

  Wire.begin(); // join i2c bus (address optional for master)
  Wire.beginTransmission(0x28);  // (0x50) POTENZIOMETRO U11
  Wire.write(byte(0x00));        // Wiper Register
  Wire.write(80);              // Valore del potenziomentro
  Wire.endTransmission();

  _delay_ms(50);

  Wire.beginTransmission(0x52);  // (0x52) POTENZIOMETRO U12
  Wire.write(byte(0x00));        // Wiper Register
  Wire.write(80);                // Valore del potenziomentro circa 6 volt
  Wire.endTransmission();
  Wire.end();
  Serial.println("POTENZIOMETRI OK");
  printLine();
  /*************************KEYPAD*********************/
  gpio.begin(I2C_PCA9534_ADDR);
  // set REG IOexpander OPREG 11000011,INVREG 00000000,CONFREG 00111100
  gpio.setporteIoExp(0xC3, 0x00, 0x3C);
  /**************** SETTING INIZIALI ******************/

  stato_procedura = - 2; // set stato di partenza
  StatoAttuale = "Starting ....";
  Serial.println(StatoAttuale);
  printLine();
  printLine();
  printLine();  
  
  // wdt_enable(WDTO_8S); /*Watchdog Reset after 8Sec*/
  WDT_Prescaler_8Sec();
}

/********************************END SETUP ***************************************/

void Buzzer(uint8_t p_ripeti, uint32_t p_delay_suono) {

  uint32_t del = p_delay_suono;

  for (int volte = 0; volte < p_ripeti; volte++)
  {
    TOGGLE_BIT(PORTC,BUZZER);
    my_delay_ms(p_delay_suono);
    TOGGLE_BIT(PORTC,BUZZER);
  }
}

char getCharKeypad(int _ioexpanderByte)
{
  int r = 0;
  int c = 0;

  switch (_ioexpanderByte) {

    /**********RIGA 1*************/
    case (5): {
        Serial.print(MappaKeys[0, 0][0]);
        r = 0;
        c = 0;
      } break;
    case (9): {
        Serial.print(MappaKeys[0, 0][1]);
        r = 0;
        c = 1;
      } break;
    case (17): {
        Serial.print(MappaKeys[0, 0][2]);
        r = 0;
        c = 2;
      } break;
    case (33): {
        Serial.print(MappaKeys[0, 0][3]);
        r = 0;
        c = 3;
      } break;
    /****************************/
    /**********RIGA 2*************/
    case (6): {
        Serial.print(MappaKeys[0, 1][0]);
        r = 1;
        c = 0;
      } break;
    case (10): {
        Serial.print(MappaKeys[0, 1][1]);
        r = 1;
        c = 1;
      } break;
    case (18): {
        Serial.print(MappaKeys[0, 1][2]);
        r = 1;
        c = 2;
      } break;
    case (34): {
        Serial.print(MappaKeys[0, 1][3]);
        r = 1;
        c = 3;
      } break;
    /****************************/
    /**********RIGA 3*************/
    case (132): {
        Serial.print(MappaKeys[0, 2][0]);
        r = 2;
        c = 0;
      } break;
    case (136): {
        Serial.print(MappaKeys[0, 2][1]);
        r = 2;
        c = 1;
      } break;
    case (144): {
        Serial.print(MappaKeys[0, 2][2]);
        r = 2;
        c = 2;
      } break;
    case (160): {
        Serial.print(MappaKeys[0, 2][3]);
        r = 2;
        c = 3;
      } break;
    /****************************/
    /**********RIGA 4*************/
    case (68): {
        Serial.print(MappaKeys[0, 3][0]);
        r = 3;
        c = 0;
      } break;
    case (72): {
        Serial.print(MappaKeys[0, 3][1]);
        r = 3;
        c = 1;
      } break;
    case (80): {
        Serial.print(MappaKeys[0, 3][2]);
        r = 3;
        c = 2;
      } break;
    case (96): {
        Serial.print(MappaKeys[0, 3][3]);
        r = 3;
        c = 3;
      } break;
    /****************************/
    default: {
        return 'N';
      } break;
  }
  // _delay_ms(20);
  return MappaKeys[0, r][c];
}

void displayLCD(String righe[], int stato, int delay_lcd)                                             // get the current time             //
{
  lcd.clear();
  //DateTime now = DS3231M.now();
  
  //sprintf(inputBuffer,"%04d-%02d-%02d %02d:%02d:%02d", now.year(),       // Use sprintf() to pretty print    //
  //now.month(), now.day(), now.hour(), now.minute(), now.second());      // date/time with leading zeros     //
  //Serial.println(inputBuffer);                        // Display the current date/time    //
  //lcd.print(inputBuffer);

  //if (stato > 1)
  // lcd.print("Tempo: " + String((UltimoPassaggioStato+Timer-secs-1))+ " sec ");

  lcd.print((char)1);  // STAMPA LA CLESSIDRA
  lcd.setCursor(0, 1);
  lcd.print(righe[1]);
  lcd.setCursor(0, 2);
  lcd.print(righe[2]);
  lcd.setCursor(0, 3);
  lcd.print(righe[3]);
  // my_delay_ms(delay_lcd);
}

void avanzaStato(unsigned long p_timer) {  
  lcd.clear();
  displayLCD(righeDisplay, stato_procedura, 10);
  Timer = p_timer;
  _delay_ms(5);
  UltimoPassaggioStato = nowTimer.secondstime();
  _delay_ms(5);
  stato_procedura++;
}

String scrivi_TAG_Mezzo(char * w_TARGA)
{
  String OUT;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidMezzo, &uidLength, 200);

  if (success) {

    if (uidLength == 4)
    {
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      success = nfc.mifareclassic_AuthenticateBlock(uidMezzo, uidLength, 4, 0, keya);

      if (success)
      {
        uint8_t data[16];

        memcpy(data,w_TARGA, sizeof data); // 1000030442
        success = nfc.mifareclassic_WriteDataBlock (4, data);        
        _delay_ms(100);
		        
        Serial.println("LETTURA TARGA");
        String TargaRFID = (char*)data;

        if (success)
        {  
          if (TargaRFID.length() == 6) {return TargaRFID; }
          // Aspetta prima di leggere una nuova CARD NFC
          _delay_ms(100);
        }
        else
        {
          lcd.clear();
          lcd.print("RITENTA !!");
        }
      }
      else
      {
        lcd.clear();
        lcd.print("RITENTA !!!");
      }
    }
    Serial.println("");
  }
  return OUT;
}

String leggiTAG_Mezzo(bool scrivi)
{
  String app;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uidMezzo, &uidLength, 200);

  if (success) {

    Buzzer(2, 30);

    if (uidLength == 4)
    {
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      success = nfc.mifareclassic_AuthenticateBlock(uidMezzo, uidLength, 4, 0, keya);

      if (success)
      {
        uint8_t data[16];

//         // Se il parametro scrivi è true scriviamo sul blocco 4 del TAG
//         if (scrivi)
//         { memcpy(data, (const uint8_t[]) {
//             '2', '8', '5', '3', '1', 'D', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
//           }, sizeof data); // 1000030442
//           success = nfc.mifareclassic_WriteDataBlock (4, data);
//         }

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
          lcd.setCursor(0, 1);
          lcd.print("RITENTA !!!!");
        }
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("RITENTA !!");
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

  //success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength,1000);

  if (success) {

    String hexCode = nfc.GetHexCode(uid, uidLength);
    hexCode.trim();
    Codice = hexCode.substring(0, 8);
    alreadyTimbrata = true;
  }

  return Codice;
}

String GetHTTPResponseCode(String HttpResp)
{
  String Cod_Conn_KO = "CONN KO";

  printLine();
  Serial.println("VerificaHTTPResponse...");
  if (HttpResp.length() == HTTP_len_response) {
    String CodHTTP = HttpResp.substring(HttpResp.length() - 3);
    //pass(true);
    Serial.println("CodHTTP:" + CodHTTP);
    return CodHTTP;
  }
  else
  {
    pass(false);
    printLine();
    return Cod_Conn_KO;
  }
}

uint8_t GetAteValidation(int Port, char serverWEB[], EthernetClient ClientHTTP, String ATeCode)
{
  int valida = 0;

  printLine();
  printTab(1);
  Serial.print("Calling Webservice ATe ..............");

  if ( (ClientHTTP.connect(serverWEB, Port)))
  {
    strURLAPI = "GET /api/ATe/GetAssociazioneByRfidCode?rfidCode=" + ATeCode + "  HTTP/1.1\r\n";
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
    lcd.setCursor(0, 1);
    lcd.print("Connessione Fallita.");
    lcd.setCursor(0, 3);
    lcd.print("Verificare....");
    _delay_ms(1000);
  }

  _delay_ms(100);
  while ((ClientHTTP.available()) && (RispostaHTTP.length() < HTTP_len_response)) {
    String rispostaGetAteValidation = "99999";
    char c = ClientHTTP.read();
    RispostaHTTP = RispostaHTTP + c;

    if ((RispostaHTTP.length() == HTTP_len_response) && (rispostaGetAteValidation == "99999"))
    {
      rispostaGetAteValidation = GetHTTPResponseCode(RispostaHTTP);
      _delay_ms(80);

      if (rispostaGetAteValidation == "200") {
        valida = true;
        pass(true);
      }
      _delay_ms(80);
    }
  }
  // _delay_ms(50);
  return valida;
}

bool GetAteCheck(int Port, char serverREST[], EthernetClient ClientHTTP, String _idAte)
{
  bool valida = false;

  printLine();
  printTab(1);

  Serial.print("Call Webservice GET GetAssociazioneByRfidCode ..............");

  if ( (ClientHTTP.connect(serverREST, Port)))  // Chiamata al Rest server per interfacciamento al GAC
  {
    _delay_ms(100);
    strURLAPI =  "GET /GetAssociazioneByRfidCode.php?rfidCode=" + _idAte + " HTTP/1.1\r\n";
    strURLAPI += "Host: " + String(serverREST);
    strURLAPI += "\r\n";
    strURLAPI += "user-agent: Mozilla/5.0 (Windows NT 6.3; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) advanced-rest-client/12.1.4 Chrome/61.0.3163.100 Electron/2.0.2 Safari/537.36";
    strURLAPI += "\r\n";
    strURLAPI += "content-type: application/json";
    strURLAPI += "\r\n";
    strURLAPI += "Accept: */*";
    strURLAPI += "\r\n";
    strURLAPI += "\r\n";
    Serial.println(strURLAPI);    
    ClientHTTP.print(strURLAPI);
     _delay_ms(100);
  
    ClientHTTP.println("Connection: close");
    ClientHTTP.println();        
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("connect failed");
    lcd.setCursor(0, 3);
    lcd.print("Errore Connessione");
    _delay_ms(1000);
    return false;
  }

  _delay_ms(10);

  String rispostaGetAte = "X";
  
  while ((ClientHTTP.available()) && (RispostaHTTP.length() < HTTP_len_response)) {
    char c = ClientHTTP.read();
    RispostaHTTP = RispostaHTTP + c;
  }

  rispostaGetAte = GetHTTPResponseCode(RispostaHTTP);
   _delay_ms(80);

   if (rispostaGetAte == "200") {
       valida = true;
       pass(true);
       printLine();
    }       
  return valida;
}

bool PostErogazioneGAC(int Port, char serverREST[], EthernetClient ClientHTTP, String _erogazione)
{
  bool valida = false;

  printLine();
  printTab(1);

  Serial.print("Call Webservice POST Erogazione Rifornimento.php ..............");

  if ( (ClientHTTP.connect(serverREST, Port)))  // Chiamata al Rest server per interfacciamento al GAC
  {
    _delay_ms(100);
    strURLAPI = "POST /Rifornimento.php HTTP/1.1\r\n";
    strURLAPI += "Host: " + String(serverREST);
    strURLAPI += "\r\n";
    strURLAPI += "user-agent: Mozilla/5.0 (Windows NT 6.3; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) advanced-rest-client/12.1.4 Chrome/61.0.3163.100 Electron/2.0.2 Safari/537.36";
    strURLAPI += "\r\n";
    strURLAPI += "content-type: application/json";
    strURLAPI += "\r\n";
    strURLAPI += "Accept: */*";
    strURLAPI += "\r\n";
    //strURLAPI += "Content-Length: 94";
    strURLAPI += "Content-Length: " + String(_erogazione.length() + 21);
    strURLAPI += "\r\n";
    strURLAPI += "\r\n";
    strURLAPI += _erogazione;
    strURLAPI += "\r\n";
    Serial.println(strURLAPI);
    
    ClientHTTP.print(strURLAPI);
	_delay_ms(100);

    ClientHTTP.println("Connection: close");
    ClientHTTP.println();        
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Connessione Fallita.");
    lcd.setCursor(0, 3);
    lcd.print("Verificare....");
    _delay_ms(1000);
    return false;
  }

  _delay_ms(10);

  String rispostaPOSTGAC = "X";
  
  while ((ClientHTTP.available()) && (RispostaHTTP.length() < HTTP_len_response)) {
    char c = ClientHTTP.read();
    RispostaHTTP = RispostaHTTP + c;
  }

  rispostaPOSTGAC = GetHTTPResponseCode(RispostaHTTP);
   _delay_ms(80);

   if (rispostaPOSTGAC == "200") {
       valida = true;
       pass(true);
       printLine();
    }     
  return valida;
}

void abilitaPulsanti() {
  /****************ABILITO PULSANTI**************/
  DDRD &= ~(1 << PD7);    // sets PD7 for input Distributore1
  DDRD &= ~(1 << PD6);    // sets PD6 for input Distributore2
  PCICR =  0b00001000;
  PCMSK3 = 0b11000000;
  sei();            // enable interrupts
  /**********************************************/
}

void abilitaContattiPistola() {
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
   _delay_ms(800);

  PCICR =  0b00000001;
 
  if (p_carburante == 'D')
  {
	DDRA &= ~(1 << PULSER1);  // PULSER 1 clear DDRA bit 5, sets PA5 for input
    PCMSK0 = 0b00100000;  // pulser 1 PCINT5
  }
  else
  {
	DDRA &= ~(1 << PULSER2);  // PULSER 1 clear DDRA bit 5, sets PA5 for input
    PCMSK0 = 0b01000000;  // pulser 2 PCINT6
  }
  sei();            // enable interrupts
};

void disabilitaPulser(char p_carburante)
{
	_delay_ms(100);

	PCICR =  0b00000001;
	
	if (p_carburante == 'D')
	{
		DDRA &= ~(1 << PULSER1);  // PULSER 1 clear DDRA bit 5, sets PA5 for input
		PCMSK0 = 0b00000000;  // pulser 1 PCINT5
	}
	else
	{
		DDRA &= ~(1 << PULSER2);  // PULSER 1 clear DDRA bit 5, sets PA5 for input
		PCMSK0 = 0b00000000;  // pulser 2 PCINT6
	}
	cli();            // enable interrupts
};


double impulsiToLitri(int P_impulsi)
{
  //double imp = (double)(P_impulsi-1);
  double imp = (double)(P_impulsi);
  if (imp < 0) {
    imp = 0;
  }
  double lt = (imp/ImpulsiLitro);
  //double totale = lt;
  return lt;
}

void Rele_Abilitazione1(int p_azione, int p_bit) {

  DDRC |= (1 << PC7);  // Rele1

  switch (p_azione) {
    case 0: // chiudi relè
      {
        CLEAR_BIT(PORTC, PC7); // Rele1*/
        //_delay_ms();
      }
      break;
    case 1: // apri relè
      {
        SET_BIT(PORTC, p_bit); // Rele1
        //_delay_ms(50);
      }
      break;
    case 2: // chiudi e apri relè
      {
        CLEAR_BIT(PORTC, p_bit); // Rele1
        _delay_ms(300);
        SET_BIT(PORTC, p_bit); // Rele1
      }
      break;
  }
}

void Rele_Abilitazione2(int p_azione, int p_bit) {

  DDRA |= (1 << PA7);  // Rele2

  switch (p_azione) {
    case 0:
      {
        CLEAR_BIT(PORTA, p_bit); // Rele2
        //_delay_ms(50);
      }
      break;
    case 1:
      {
        SET_BIT(PORTA, p_bit); // Rele2
        //_delay_ms(50);
      }
      break;
    case 2:
      {
        CLEAR_BIT(PORTA, p_bit); // Rele2
        _delay_ms(300);
        SET_BIT(PORTA, p_bit); // Rele2
      }
      break;
  }
}

void Azzera()
{
  
  RispostaHTTP = "";
  impulsi = 0;
  alreadyTimbrata = false;
  //Carburante = "X";
  mezzo.Carb = "X";
  mezzo.TARGA = "X";
  mezzo.KM = "0";
  distr_selezionato = 0;

  Rele_Abilitazione1(1, 7);
  Rele_Abilitazione2(1, 7);
  _delay_ms(5);
  clientATE.flush();
  clientATE.stop();
  _delay_ms(5);
  clientLOCAL.flush();
  clientLOCAL.stop();
  //Connected = false; // Eliminato
  _delay_ms(5);
  
  enable_FLASH();
  _delay_ms(5);
  disable_FLASH();
  _delay_ms(5);
  
  disable_ETH();
  _delay_ms(5);
  enable_ETH();

  SET_BIT(PORTA, A1);
  SET_BIT(PORTA, A2);

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
  TARGA = "";
  //wdt_enable(WDTO_8S);
  WDT_Prescaler_8Sec();
  //while(1); // Verifico WDT
  stato_procedura = -1;
}

void inputTarga(char T) {

  switch (T) {
    case ('N'): { NULL;
        // Serial.print("NIENTE");
      }
      break;
    case ('A'): {
        if (TARGA.length() == 5) {
          char buf[6];     
          String w_TARGA = TARGA + "D";   
          w_TARGA.toCharArray(buf, 7);
          String mezzoString = scrivi_TAG_Mezzo(buf);
          _delay_ms(10);       
		  Buzzer(1,10);
		  _delay_ms(10);
		  Buzzer(1,10); 
		  _delay_ms(10);  
        }       
      }
      break;
    case ('B'): {
        if ((TARGA.length() == 5) && (TARGA.length() < 8)) {
          char buf[6];     
          String w_TARGA = TARGA + "B";   
          w_TARGA.toCharArray(buf, 7);
          String mezzoString = scrivi_TAG_Mezzo(buf);
          _delay_ms(10);
          Buzzer(1,10);
          _delay_ms(10);
          Buzzer(1,10);
          _delay_ms(10);
        }
      }
      break;
    case ('*'): {
        TARGA = "";
        righeDisplay[1] =  "AVVICINA TAG MEZZO"; // Set display per stato successivo
        righeDisplay[2] =  "";
        righeDisplay[3] = "";        
        avanzaStato(TselDistributore);
      }
      break;
    case ('C'): {
        if (TARGA.length() > 0) {
          TARGA = TARGA.substring(0, TARGA.length() - 1);
          righeDisplay[1] =  "** TARGA MEZZO **";
          righeDisplay[2] = "TARGA:" + TARGA;
          righeDisplay[3] = "#:Conferma *:Usa TAG";
          displayLCD(righeDisplay, stato_procedura, 10);
        }
      }
      break;
    case ('#'): {
        if (TARGA.length() == 5) {
          mezzo.TARGA = TARGA;
          RaccoltaDati[1] = mezzo.TARGA;
          righeDisplay[1] = "SCEGLI DISTRIBUTORE";
          righeDisplay[2] = "NERO:  POMPA 1";
          righeDisplay[3] = "VERDE: POMPA 2";
          avanzaStato(TselDistributore);
        }
      }
      break;
    default:  {
      if (TARGA.length() < 5) {
		TARGA += String(T);
		Buzzer(1,10);
        _delay_ms(10);
        
		righeDisplay[1] =  "** TARGA MEZZO **";
        righeDisplay[2] = "TARGA:" + TARGA;
        righeDisplay[3] = "#:Conferma *:Usa TAG";
        displayLCD(righeDisplay, stato_procedura, 10);
	  }
      }
      break;
  }
}

void inputKM(char KM_input) {

  switch (KM_input) {
    case ('N'): { NULL;
        //  Serial.print("NIENTE");        
      }
      break;
    case ('A'): { NULL;
        //  Serial.print("NIENTE");
      }
    break;
    case ('B'): { NULL;
        //  Serial.print("NIENTE");
      }
    break;
    case ('.'): { NULL;
        //  Serial.print("NIENTE");
      }
    break;
    case ('*'): { NULL;
        //  Serial.print("NIENTE");
      }
    break;

    case ('C'): {
        if (KM.length() > 0)
          KM = KM.substring(0, KM.length() - 1);
        righeDisplay[1] =  "****** KM ******";
        righeDisplay[2] = "KM:" + KM;
        righeDisplay[3] = "#:Conferma";
        displayLCD(righeDisplay, stato_procedura, 10);
      }
      break;
    case ('#'): {
        if (KM.length() == 4) {
       	  if ( mezzo.Carb == "D" ) {
				 Rele_Abilitazione1(0, 7);
				 //abilitaPulser('D');
				 righeDisplay[3] = "POMPA 1";
			} // chiudi relè
          else if ( mezzo.Carb == "B" ) {
				 Rele_Abilitazione2(0, 7); 
				 //abilitaPulser('B');
			     righeDisplay[3] = "POMPA 2";
			} // chiudi relè
          
	      mezzo.KM = KM;
          RaccoltaDati[4] = mezzo.KM;
		  righeDisplay[1] = "SGANCIA PISTOLA";
		  righeDisplay[2] = "";
         /*****************************************************************/
          disable_ETH();
          _delay_ms(2);
          enable_ETH();
          /*****************************************************************/
          avanzaStato(TsgancioPistola);
        }
      }
      break;
    default:  {
       if (KM.length() < 4) {
	    KM += String(KM_input);
		Buzzer(1,10);
        //_delay_ms(20);
        righeDisplay[1] =  "****** KM ******";
        righeDisplay[2] = "KM:" + KM;
        righeDisplay[3] = "#:Conferma";
        displayLCD(righeDisplay, stato_procedura, 10);
	   }
      }
      break;
  }
}

/******************************* EERPOM ***************************************/
String read_eeprom_string_struct(ParametriCCEC_TypeDef dato) {

  String Salvata = "OK";
  int lunBuffer = dato.da_memorizzare.length();
  uint8_t buf[lunBuffer];
  int i = 0;

  for (int ind = dato.startIND ; ind < (dato.startIND + lunBuffer); ind++) {
    buf[i] = EEPROM.read(ind);
    //if (buf[i] != 0) {
    //  Serial.print(" " +  String(buf[i]));
    // }
    i++;
  }

  Serial.println(" ");
  Serial.println(" ");
  Salvata = String((char *)buf);
  Serial.print(dato.descrizione + ": ");
  Serial.println(Salvata);
  return Salvata;
}

String read_eeprom_string(int lunBuffer,int start_ind) {

 String Salvata = "OK";
 uint8_t buf[lunBuffer];
 int i = 0;

 Serial.println(" ");
 Serial.println("\r\n Fase di lettura");
 Serial.println(" ");
 
 for (int ind = start_ind ;ind < (start_ind + lunBuffer);ind++) {
    buf[i] = EEPROM.read(ind);
    if (buf[i] != 0) {
      Serial.print(" " +  String(buf[i])); 
    }
    i++;
  }

 Serial.println(" ");
 Serial.println(" ");
 Salvata = String((char *)buf);
 Serial.print("Salvata nella EEPROM: ");
 Serial.println(Salvata);
 return Salvata;
}

String read_string_from_eeprom(int start_ind) {

 String Salvata = "";
 int ind = start_ind;
 Serial.println(" ");
 Serial.println("\r\n Fase di lettura");
 Serial.println(" ");
 

 char c = EEPROM.read(ind);
 Salvata = Salvata + c;
 while(c != '\0') {
    c = EEPROM.read(ind);
    Salvata = Salvata + c;    
    ind++;
 }

 Serial.println(" ");
 Serial.println(" ");
 Serial.print("Salvata nella EEPROM: ");
 Serial.println(Salvata);
 return Salvata;
}

bool write_eeprom_string(String erog,int lunBuffer,int start_ind) {
 
 char buf[lunBuffer];
 erog.toCharArray(buf, erog.length()+1);
 bool out = false;
 int i = 0;
 
 Serial.println(" len: " + String(lunBuffer));
 Serial.println("Eseguo scrittura nella EEPROM");
 Serial.println(" ");

  for (int ind = start_ind ; ind < (start_ind + lunBuffer) ; ind++) {
     if (buf[i] != 0) {
      EEPROM.write(ind, buf[i]);
      Serial.print(" " + String(buf[i]));
     }
       i++;
  }
 Serial.println(" ");
 
 return true;
}

void timer_stato(){
  lcd.setCursor(0, 0);
  lcd.print((char)1);  // STAMPA LA CLESSIDRA
  lcd.print("Time: " + String((UltimoPassaggioStato + Timer - secs - 1)) + " sec ");// + String(temperatura/100.0) + "°C");  
}


/**************************LOOP PROCEDURA************************************/
void loop() {

  // while(1); // Verifico WDT 
		
  switch (stato_procedura) {
    case -2:
      { //cli(); // disable interrupt        
        printLine();
        Serial.print("Parametri CCEC da EEPROM");
        String ServerCCEC = read_eeprom_string_struct(ParametriCCEC[0]);
        ServerCCEC.toCharArray(serverREST,ServerCCEC.length()+1);
        printLine();
		String IPCCEC = read_eeprom_string_struct(ParametriCCEC[1]); 				
        printLine();
        String Start_save = read_eeprom_string_struct(ParametriCCEC[2]);
        printLine();
        String CSEDE = read_eeprom_string_struct(ParametriCCEC[3]);
        CSEDE.toCharArray(CodSede,CSEDE.length()+1);        
        printLine();
		ImpulsiLitro = read_eeprom_string_struct(ParametriCCEC[4]).toInt();
        stato_procedura++;
      }
      break;
    case -1:
      {       
        _delay_ms(20);
        abilitaPulsanti();
        _delay_ms(20);
        abilitaContattiPistola();
        _delay_ms(20);
        stato_procedura++;
      }
      break;
    case 0:
      {
		lcd.noBacklight();
        alreadyTimbrata = false;
        enable_ETH();
		_delay_ms(10);
        /************************************************/
        righeDisplay[1] = " * AUTENTICAZIONE *";
        righeDisplay[2] = "";
        righeDisplay[3] = "    Avvicina ATE  ";
        displayLCD(righeDisplay, stato_procedura, 50);
        /************************************************/
		sei();
		_delay_ms(1000);
        //stato_procedura++;
		avanzaStato(TverificaBadge);
        //while (1);
      }
      break;
    case 1:
      {
        TARGA = "";
        KM = "";

        /*****************************************************/
        String ATe = "ERRORE";
        
        if (!alreadyTimbrata) {
          ATe = GetCodeRfidATe();          
        }

        if ((ATe != "ERRORE") && (bitislow(PORTC, CS_W5500))) //&& (BIT_IS_CLEAR(PORTC, 4)))
        {
          Serial.println("");
          Serial.print("***************************************************************");
          Serial.println(" Tessera ID : " + ATe);
          Serial.print("***************************************************************");
          Serial.println("Riconoscimento Tessera .............");

          RaccoltaDati[0] = ATe;
          // RaccoltaDati[0] = "DD92743A";
          // RaccoltaDati[5] = "000";

          lcd.backlight();
          lcd.display();
          _delay_ms(10);

          righeDisplay[1] = "  RICONOSCIMENTO ";
          righeDisplay[2] = ".....In Corso.....";
          righeDisplay[3] = "   Rfid: " + ATe; // COMMENTA in produzione
          //righeDisplay[3] = "Attendere.........";
          displayLCD(righeDisplay, stato_procedura, 100);
          InizializzaEthernet();
          _delay_ms(1000); // tempo per inizializzare la ethernet
       //}

        // Effettua chiamata REST per validare CARD NFC
        
        righeDisplay[1] =  "** TARGA MEZZO **";
        righeDisplay[2] = "TARGA:";
        righeDisplay[3] = "#:Conferma *:Usa TAG";

		//if (GetAteValidation(80,serverATE,clientATE,ATe)) // Server Centrale
		
        // bool GetAteCheck(int Port, char serverREST[], EthernetClient ClientHTTP, String _idAte)
        if (GetAteCheck(80,serverREST,clientATE,ATe)) 
        {
                //SET_BIT(PORTC,PC4);
				SET_BIT(PORTC,CS_W5500);
                RaccoltaDati[5] = "000";               
                Buzzer(1,200);
				_delay_ms(200);
                avanzaStato(TinputTarga);
         } 
         else 
         {
                //SET_BIT(PORTC,PC4);
				SET_BIT(PORTC,CS_W5500);
                RaccoltaDati[5] = "111";                
                lcd.clear();
                righeDisplay[1] = "***** ERRORE ******";
                righeDisplay[2] = "  Ate NON VALIDA ";
                righeDisplay[3] = " Problema di Rete";
                displayLCD(righeDisplay,stato_procedura,10);
                _delay_ms(1000);
                Azzera();
         }
		}    
      }
      break;
    case 2:
      {
        timer_stato();

        /*****************************************************************/
        // da commentare
        // Carburante = "D"; // Simulo Abilitazione Diesel
        // da commentare
        // Carburante = "B"; // Simulo Abilitazione Benzina
        /*****************************************************************/
        gpio.setCONFREG(0x3C);
        uint8_t c = gpio.Read_IP_REGISTER();
        char buf[8];
        itoa(c, buf, 2);
        gpio.setCONFREG(0xC3);
        uint8_t r = gpio.Read_IP_REGISTER();
        char bufr[8];
        itoa(r, bufr, 2);
        char ris[8];
        uint8_t z = (r ^ c);
        itoa(z, ris, 2);
        char T = getCharKeypad(int(z));
        _delay_ms(20);
        /*****************************************************************/
        inputTarga(T);
      }
      break;
    case 3:
      {
        timer_stato();

        if (TARGA.length() == 5)
        {
          mezzo.Carb = "X";
          mezzo.TARGA = TARGA;
          mezzo.KM = "0";
          distr_selezionato = 0;
          avanzaStato(TselDistributore);
        }
        else {
          String mezzoString = leggiTAG_Mezzo(false); // con TRUE scrive sul blocco 4 della card NFC DEL MEZZO
          _delay_ms(10);

          Serial.println(mezzoString);

          mezzo.Carb = mezzoString.substring(5);
          mezzo.TARGA = mezzoString.substring(0, 5);
          mezzo.KM = "0";

          Serial.println("TIPO CARBURANTE: " + mezzo.Carb);
          Serial.println("TARGA: " + mezzo.TARGA);

          if ((mezzo.Carb == "B") || (mezzo.Carb == "D")) {
            RaccoltaDati[1] = mezzo.TARGA;
            RaccoltaDati[2] = mezzo.Carb;
            righeDisplay[3] = "TARGA: " +  mezzo.TARGA;
            _delay_ms(500);
            distr_selezionato = 0;
            avanzaStato(TselDistributore);
          }
        }
      }
      break;
    case 4:
      {
        timer_stato();

        // Verifica scelta distributore

        if ((mezzo.Carb == "B") || (distr_selezionato == 2))
        {
          mezzo.Carb = "B";
          StatoAttuale = "POMPA 2";
          RaccoltaDati[2] = mezzo.Carb;
          righeDisplay[1] =  "****** KM ******";
          righeDisplay[2] = "KM:";
          righeDisplay[3] = "#:Conferma";       
          _delay_ms(100);    
          avanzaStato(TinputKM);
        }
        else if ((mezzo.Carb == "D") || (distr_selezionato == 1))
        {
          mezzo.Carb = "D";
          StatoAttuale = "POMPA 1";
          RaccoltaDati[2] = mezzo.Carb;
          righeDisplay[1] =  "****** KM ******";
          righeDisplay[2] = "KM:";
          righeDisplay[3] = "#:Conferma";      
          _delay_ms(100);     
         avanzaStato(TinputKM);
        }
      }
      break;
    case 5:
      {
		timer_stato();
        
		// RaccoltaDati[4] = "1234";        
        /*****************************************************************/
        gpio.setCONFREG(0x3C);
        uint8_t c = gpio.Read_IP_REGISTER();
        char buf[8];
        itoa(c, buf, 2);
        gpio.setCONFREG(0xC3);
        uint8_t r = gpio.Read_IP_REGISTER();
        char bufr[8];
        itoa(r, bufr, 2);
        char ris[8];
        uint8_t z = (r ^ c);
        itoa(z, ris, 2);
        char K = getCharKeypad(int(z));
        _delay_ms(20);
        /*****************************************************************/

        inputKM(K);
        impulsi = 0;
      }
      break;
	case 6: 
	{ 
	    timer_stato();
		 
		if (testbit(PINA,1) && (mezzo.Carb == "D"))
		{
			cli(); // GLOBAL INTERRUPT DISABLE
            righeDisplay[1] = "LITRI : 0.00";
			righeDisplay[2] = "TARGA:" + mezzo.TARGA;
 		    // righeDisplay[2] = "imp :" + String(impulsi);
 			righeDisplay[3] = "Erogazione: " + StatoAttuale;
			abilitaPulser('D');
			impulsi = 0;
			avanzaStato(TmaxErogazione); 
		}
		
		// CONTATTO PISTOLA BENZINA

		if  (testbit(PINB,1) && (mezzo.Carb == "B"))
		{
			 cli(); // GLOBAL INTERRUPT DISABLE
             righeDisplay[1] = "LITRI : 0.00";
			 righeDisplay[2] = "TARGA:" + mezzo.TARGA;
             // righeDisplay[2] = "imp :" + String(impulsi);
             righeDisplay[3] = "Erogazione: " + StatoAttuale;
			 abilitaPulser('B');
			 impulsi = 0;
             avanzaStato(TmaxErogazione);
		}
		 
	}
	break;
    case 7:
      {
        timer_stato();

        double lt = impulsiToLitri(impulsi);

        righeDisplay[1] = "LITRI :" + String(lt);
		righeDisplay[2] = "TARGA:" + mezzo.TARGA;
        //righeDisplay[2] = "imp :" + String(impulsi);
        righeDisplay[3] = "Erogazione: " + StatoAttuale;

        lcd.setCursor(0, 1);
        lcd.print(righeDisplay[1]);
        lcd.setCursor(0, 2);
        lcd.print(righeDisplay[2]);
        lcd.setCursor(0, 3);
        lcd.print(righeDisplay[3]);

        // CONTATTO PISTOLA DIESEL

 		if (!testbit(PINA,1) && (mezzo.Carb == "D"))
        {
          RaccoltaDati[3] = String(lt);
          StatoAttuale = "STOP EROGAZIONE";
          Rele_Abilitazione2(1, 7); //  apri relè
          Rele_Abilitazione1(1, 7); //  apri relè
          //TOGGLE_BIT(PORTA, 1);
          avanzaStato(TmaxInviodati);
        }

        // CONTATTO PISTOLA BENZINA

		if (!testbit(PINB,1) && (mezzo.Carb == "B"))
        {
          RaccoltaDati[3] = String(lt);
          StatoAttuale = "STOP EROGAZIONE";
          Rele_Abilitazione2(1, 7); //  apri relè
          Rele_Abilitazione1(1, 7); //  apri relè
          avanzaStato(TmaxInviodati);
        }        
      }
      break;
    case 8 :
      {
        righeDisplay[1] =  StatoAttuale;
        righeDisplay[2] = "Invio........";
        righeDisplay[3] =  "";      
        displayLCD(righeDisplay, stato_procedura, 100);
        Messaggio = "";

        for (int k = 0; k < 6; k++)
          Messaggio.concat(RaccoltaDati[k] + ";");

        Messaggio.concat(CodSede);
        Serial.println("Messaggio:" + Messaggio);
        // Messaggio = "DD92743A;28530;D;15.03;1234;000;SA10012";
        /*****************************************************************/
        disable_ETH();
        _delay_ms(2);
        enable_ETH();
        /*****************************************************************/
        _delay_ms(500); // prima era _delay_ms(1000);
		lcd.noBacklight();
        avanzaStato(30); 
      }
      break;
    case 9:
      {        
        //if (BIT_IS_CLEAR(PORTC, 4))
		if (bitislow(PORTC,CS_W5500))
        {
          displayLCD(righeDisplay, stato_procedura, 10);
          Messaggio = "";

          for (int k = 0; k < 6; k++)
            Messaggio.concat(RaccoltaDati[k] + ";");

          Messaggio.concat(CodSede);
          Serial.println("Messaggio:" + Messaggio);

          // Messaggio = "DD92743A;28530;D;15.03;1234;000;SA10012";

          _delay_ms(200); // prima era _delay_ms(1000);

          if (PostErogazioneGAC(80, serverREST, clientLOCAL, Messaggio))
          {
            disable_ETH();
            _delay_ms(200);
            Serial.println("PostErogazioneGAC - OK" );   
            _delay_ms(2);
            enable_ETH();
            _delay_ms(2);
            avanzaStato(20);                                
          }
          else
          {			 
             disable_ETH();   
             String ultima_indirizzo  = read_eeprom_string(4,1035);
             int indirizzo = ultima_indirizzo.toInt();
            // bool write_eeprom_string(String erog,int lunBuffer,int start_ind)
            if (write_eeprom_string(Messaggio,Messaggio.length(),indirizzo))
              {
                Serial.println("WRITE OK AT address :" + String(indirizzo));
                indirizzo = indirizzo + 50;  
                if (indirizzo > 3000) {indirizzo = 2000; }   
                String update_ultima_indirizzo = String(indirizzo);
                if (write_eeprom_string(update_ultima_indirizzo,update_ultima_indirizzo.length(),1035))
                  Serial.println("UPDATE OK address :" + String(indirizzo));  
              }                
              Azzera();              
          }                  
        }
      }
      break;
    case 10:
      {
		 
			 Serial.println("Tento la ritrasmisssione di erogazioni salvate non trasmesse" );             
			 Buzzer(1,100);
			 _delay_ms(100);
			 Buzzer(1,100);
			 _delay_ms(100);
			 Buzzer(1,100);
			 _delay_ms(100);
			 
 			 disabilitaPulser('D');
 			 disabilitaPulser('B');

			 WDT_off();
			 
             String str_indirizzo  = read_eeprom_string(4,1035);
             int ultimo_indirizzo = (str_indirizzo.toInt());
             int start = 2000;
             bool tx = false;
             
             while ((start < ultimo_indirizzo))
             {
               String e  = read_eeprom_string(50,start);
               Serial.println("DA TRASMETTERE: " + e);
               tx = PostErogazioneGAC(80, serverREST, clientLOCAL, e);
               start = start + 50;    
                _delay_ms(200); // Attendo un pochino tra un atrasmissione e l'altra           
               printLine();                
             }
             
             if (start >  2000){
                write_eeprom_string("2000",4,1035);
                clearEEPROM(2000,start);
              }              
        Azzera();
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
  
  temperatura = DS3231M.temperature();
  nowTimer = DS3231M.now();
  secs = nowTimer.secondstime();  
  if (((UltimoPassaggioStato + Timer - secs) <= 1) && (stato_procedura != stato_erogazione)) Azzera();
  else if (((UltimoPassaggioStato + Timer - secs) <= 1) && (stato_procedura == stato_erogazione)) avanzaStato(TmaxInviodati);
  
  wdt_reset(); // Reset watchdog Timer
}

/********************FINE LOOP PROCEDURA************************************/

// interrupt per conteggio impulsi
/***********************************************************************
  Esempio
  100 impulsi/litro
  73 litri al minuto
  73/60 = 1,22 lt/sec
  1,22*100 = 122 Hz
***********************************************************************/

ISR(PCINT0_vect) {
  if ((stato_procedura == stato_erogazione) && (impulsi < MaxErogabile))
  {
	  
	  if ((PINA & _BV(PA5)) && (mezzo.Carb == "D")) {
		impulsi++;
	  }
	  
      if ((PINA & _BV(PA6)) && (mezzo.Carb == "B")) {
		impulsi++;
	  }
 }
}
/***********************************************************************/

// interrupt per pulsanti abilitazione diesele benzina

ISR(PCINT3_vect) {

  if (stato_procedura == stato_distributore)
  {
    if (PIND & _BV(PD6))
    {
      intConsecutivePresses++;  // increment counter for number of presses           
      if (intConsecutivePresses >= NUM_OF_CONSECUTIVE_PRESSES)
      { // if enough presses to constitute a press
        intConsecutivePresses = 0;                    // and reset press counts
        intConsecutiveNonPresses = 0;        
        distr_selezionato = 2;
      }
    }
    else  {           // else if button is not pressed (logic low)
      intConsecutiveNonPresses++;
      if (intConsecutiveNonPresses >= NUM_OF_CONSECUTIVE_NON_PRESSES) {
        intConsecutivePresses = 0;                      // reset press counts
        intConsecutiveNonPresses = 0;
      }
    }

    if (PIND & _BV(PD7))
    {      
      intConsecutivePresses++;  // increment counter for number of presses
      if (intConsecutivePresses >= NUM_OF_CONSECUTIVE_PRESSES)
      { // if enough presses to constitute a press
        intConsecutivePresses = 0;                    // and reset press counts
        intConsecutiveNonPresses = 0;        
        distr_selezionato = 1;
      }
    }
    else  {            // else if button is not pressed (logic low)
      intConsecutiveNonPresses++;
      if (intConsecutiveNonPresses >= NUM_OF_CONSECUTIVE_NON_PRESSES) {
        intConsecutivePresses = 0;                     // reset press counts
        intConsecutiveNonPresses = 0;
      }
    }
  }
}

ISR(WDT_vect)
{
  RispostaHTTP = "";
  impulsi = 0;
  alreadyTimbrata = false;
  //Carburante = "X";
  mezzo.Carb = "X";
  mezzo.TARGA = "X";
  mezzo.KM = "0";
  distr_selezionato = 0;

  Rele_Abilitazione1(1, 7);
  Rele_Abilitazione2(1, 7);
  
  disable_ETH();
  enable_ETH();

  SET_BIT(PORTA, A1);
  SET_BIT(PORTA, A2);

  righeDisplay[1] =  "";
  righeDisplay[2] =  "";
  righeDisplay[2] =  "";
 
  secs = 0;
  UltimoPassaggioStato = 0;

  TARGA = "";
  
  // WDT_Prescaler_8Sec(); // 
  //while(1); // Verifico WDT
  stato_procedura = -1;
}