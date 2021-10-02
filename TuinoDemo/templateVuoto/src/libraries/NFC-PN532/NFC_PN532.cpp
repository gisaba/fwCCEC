/**************************************************************************/
/*!

*/
/**************************************************************************/

#include "Arduino.h"
// #include "WProgram.h"

#include <util/delay.h>
#include <Wire.h>
#define WIRE Wire

#include "NFC_PN532.h"

byte pn532ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
byte pn532response_firmwarevers[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

// If using Native Port on Arduino Zero or Due define as SerialUSB
#define PN532DEBUGPRINT Serial
//#define PN532DEBUGPRINT SerialUSB

#define PN532_PACKBUFFSIZ 64
byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

#ifndef _BV
    #define _BV(bit) (1<<(bit))
#endif

/**************************************************************************/
/*!
    @brief  Sends a single byte via I2C

    @param  x    The byte to send
*/
/**************************************************************************/
static inline void i2c_send(uint8_t x)
{ 
    WIRE.write((uint8_t)x);
}

/**************************************************************************/
/*!
    @brief  Reads a single byte via I2C
*/
/**************************************************************************/
static inline uint8_t i2c_recv(void)
{
    return WIRE.read();
}

/**************************************************************************/
/*!
    @brief  Instantiates a new PN532 class using I2C.

    @param  irq       Location of the IRQ pin
    @param  reset     Location of the RSTPD_N pin
*/
/**************************************************************************/
NFC_PN532::NFC_PN532(uint8_t irq, uint8_t reset):
  _irq(irq),
  _reset(reset)
{
  pinMode(_irq, INPUT);
  pinMode(_reset, OUTPUT);
}

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
void NFC_PN532::begin() {
 
    // I2C initialization.
    WIRE.begin();

   // Reset the PN532
//     digitalWrite(_reset, HIGH);
//     digitalWrite(_reset, LOW);
//     delay(400);
//     digitalWrite(_reset, HIGH);
    _delay_ms(10);  // Small delay required before taking other actions after reset.
                // See timing diagram on page 209 of the datasheet, section 12.23.
}

/**************************************************************************/
/*!
    @brief  Prints a hexadecimal value in plain characters

    @param  data      Pointer to the byte data
    @param  numBytes  Data length in bytes
*/
/**************************************************************************/
void NFC_PN532::PrintHex(const byte * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    PN532DEBUGPRINT.print(F("0x"));
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos]&0xff, HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.println();
}

void NFC_PN532::PrintHexChar(const byte * data, const uint32_t numBytes)
{
  uint32_t szPos;
  for (szPos=0; szPos < numBytes; szPos++)
  {
    // Append leading 0 for small values
    if (data[szPos] <= 0xF)
      PN532DEBUGPRINT.print(F("0"));
    PN532DEBUGPRINT.print(data[szPos], HEX);
    if ((numBytes > 1) && (szPos != numBytes - 1))
    {
      PN532DEBUGPRINT.print(F(" "));
    }
  }
  PN532DEBUGPRINT.print(F("  "));
  for (szPos=0; szPos < numBytes; szPos++)
  {
    if (data[szPos] <= 0x1F)
      PN532DEBUGPRINT.print(F("."));
    else
      PN532DEBUGPRINT.print((char)data[szPos]);
  }
  PN532DEBUGPRINT.println();
}
/**************************************************************************/

String NFC_PN532::GetHexCode(const byte * data, const uint32_t numBytes)
{
  String UID = "";

  uint32_t szPos;
  
  for (szPos=0; szPos < numBytes; szPos++)
  {
    if (data[szPos] <= 0xF)
      UID =  UID  + "0" + String(data[szPos],HEX);
    else
      UID =  UID  + String(data[szPos],HEX); 
  }
  PN532DEBUGPRINT.println();
  return UID;
}

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

    @returns  The chip's firmware version and ID
*/
/**************************************************************************/
uint32_t NFC_PN532::getFirmwareVersion(void) {
  uint32_t response;

  pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

  if (! sendCommandCheckAck(pn532_packetbuffer, 1)) {
    return 0;
  }

  // read data packet
  readdata(pn532_packetbuffer, 12);

  // check some basic stuff
  if (0 != strncmp((char *)pn532_packetbuffer, (char *)pn532response_firmwarevers, 6)) {
#ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Firmware doesn't match!"));
#endif
    return 0;
  }

  //int offset = _usingSPI ? 6 : 7;  // Skip a response byte when using I2C to ignore extra data.
  int offset = 6;
  response = pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];
  response <<= 8;
  response |= pn532_packetbuffer[offset++];

  return response;
}


/**************************************************************************/
/*!
    @brief  Sends a command and waits a specified period for the ACK

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    The size of the command in bytes
    @param  timeout   timeout before giving up

    @returns  1 if everything is OK, 0 if timeout occured before an
              ACK was recieved
*/
/**************************************************************************/
// default timeout of one second
bool NFC_PN532::sendCommandCheckAck(uint8_t *cmd, uint8_t cmdlen, uint16_t timeout) {
  uint16_t timer = 0;

  // write the command
  writecommand(cmd, cmdlen);

  // Wait for chip to say its ready!
  if (!waitready(timeout)) {
    return false;
  }

  #ifdef PN532DEBUG
    if (!_usingSPI) {
      PN532DEBUGPRINT.println(F("IRQ received"));
    }
  #endif

  // read acknowledgement
  if (!readack()) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("No ACK frame received!"));
    #endif
    return false;
  }

  return true; // ack'd command
}

/**************************************************************************/
/*!
    Writes an 8-bit value that sets the state of the PN532's GPIO pins

    @warning This function is provided exclusively for board testing and
             is dangerous since it will throw an error if any pin other
             than the ones marked "Can be used as GPIO" are modified!  All
             pins that can not be used as GPIO should ALWAYS be left high
             (value = 1) or the system will become unstable and a HW reset
             will be required to recover the PN532.

             pinState[0]  = P30     Can be used as GPIO
             pinState[1]  = P31     Can be used as GPIO
             pinState[2]  = P32     *** RESERVED (Must be 1!) ***
             pinState[3]  = P33     Can be used as GPIO
             pinState[4]  = P34     *** RESERVED (Must be 1!) ***
             pinState[5]  = P35     Can be used as GPIO

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool NFC_PN532::writeGPIO(uint8_t pinstate) {
  uint8_t errorbit;

  // Make sure pinstate does not try to toggle P32 or P34
  pinstate |= (1 << PN532_GPIO_P32) | (1 << PN532_GPIO_P34);

  // Fill command buffer
  pn532_packetbuffer[0] = PN532_COMMAND_WRITEGPIO;
  pn532_packetbuffer[1] = PN532_GPIO_VALIDATIONBIT | pinstate;  // P3 Pins
  pn532_packetbuffer[2] = 0x00;    // P7 GPIO Pins (not used ... taken by SPI)

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Writing P3 GPIO: ")); PN532DEBUGPRINT.println(pn532_packetbuffer[1], HEX);
  #endif

  // Send the WRITEGPIO command (0x0E)
  if (! sendCommandCheckAck(pn532_packetbuffer, 3))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0F) DATACHECKSUM 00)
  readdata(pn532_packetbuffer, 8);

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Received: "));
    PrintHex(pn532_packetbuffer, 8);
    PN532DEBUGPRINT.println();
  #endif

  //int offset = _usingSPI ? 5 : 6;
  int offset = 5;
  return  (pn532_packetbuffer[offset] == 0x0F);
}

/**************************************************************************/
/*!
    Reads the state of the PN532's GPIO pins

    @returns An 8-bit value containing the pin state where:

             pinState[0]  = P30
             pinState[1]  = P31
             pinState[2]  = P32
             pinState[3]  = P33
             pinState[4]  = P34
             pinState[5]  = P35
*/
/**************************************************************************/
uint8_t NFC_PN532::readGPIO(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_READGPIO;

  // Send the READGPIO command (0x0C)
  if (! sendCommandCheckAck(pn532_packetbuffer, 1))
    return 0x0;

  // Read response packet (00 FF PLEN PLENCHECKSUM D5 CMD+1(0x0D) P3 P7 IO1 DATACHECKSUM 00)
  readdata(pn532_packetbuffer, 11);

  /* READGPIO response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..5           Frame header and preamble (with I2C there is an extra 0x00)
    b6              P3 GPIO Pins
    b7              P7 GPIO Pins (not used ... taken by SPI)
    b8              Interface Mode Pins (not used ... bus select pins)
    b9..10          checksum */

  //int p3offset = _usingSPI ? 6 : 7;
  int p3offset = 6;

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("Received: "));
    PrintHex(pn532_packetbuffer, 11);
    PN532DEBUGPRINT.println();
    PN532DEBUGPRINT.print(F("P3 GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset],   HEX);
    PN532DEBUGPRINT.print(F("P7 GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset+1], HEX);
    PN532DEBUGPRINT.print(F("IO GPIO: 0x")); PN532DEBUGPRINT.println(pn532_packetbuffer[p3offset+2], HEX);
    // Note: You can use the IO GPIO value to detect the serial bus being used
    switch(pn532_packetbuffer[p3offset+2])
    {
      case 0x00:    // Using UART
        PN532DEBUGPRINT.println(F("Using UART (IO = 0x00)"));
        break;
      case 0x01:    // Using I2C
        PN532DEBUGPRINT.println(F("Using I2C (IO = 0x01)"));
        break;
      case 0x02:    // Using SPI
        PN532DEBUGPRINT.println(F("Using SPI (IO = 0x02)"));
        break;
    }
  #endif

  return pn532_packetbuffer[p3offset];
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
bool NFC_PN532::SAMConfig(void) {
  pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
  pn532_packetbuffer[1] = 0x01; // normal mode;
  pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
  pn532_packetbuffer[3] = 0x01; // use IRQ pin!

  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
    return false;

  // read data packet
  readdata(pn532_packetbuffer, 8);

  //int offset = _usingSPI ? 5 : 6;
  int offset = 5;
  return  (pn532_packetbuffer[offset] == 0x15);
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation byte of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool NFC_PN532::setPassiveActivationRetries(uint8_t maxRetries) {
  pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
  pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
  pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
  pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
  pn532_packetbuffer[4] = maxRetries;

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Setting MxRtyPassiveActivation to ")); PN532DEBUGPRINT.print(maxRetries, DEC); PN532DEBUGPRINT.println(F(" "));
  #endif

  if (! sendCommandCheckAck(pn532_packetbuffer, 5))
    return 0x0;  // no ACK

  return 1;
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field

    @param  cardBaudRate  Baud rate of the card
    @param  uid           Pointer to the array that will be populated
                          with the card's UID (up to 7 bytes)
    @param  uidLength     Pointer to the variable that will hold the
                          length of the card's UID.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
bool NFC_PN532::readPassiveTargetID(uint8_t cardbaudrate, uint8_t * uid, uint8_t * uidLength, uint16_t timeout) {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
  pn532_packetbuffer[2] = cardbaudrate;

  if (!sendCommandCheckAck(pn532_packetbuffer, 3, timeout))
  {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("No card(s) read"));
    #endif
    return 0x0;  // no cards read
  }

  // wait for a card to enter the field (only possible with I2C)
  
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Waiting for IRQ (indicates card presence)"));
    #endif
    if (!waitready(timeout)) {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("IRQ Timeout"));
      #endif
      return 0x0;
    }
  

  // read data packet
  readdata(pn532_packetbuffer, 20);
  // check some basic stuff

  /* ISO14443A card response should be in the following format:

    byte            Description
    -------------   ------------------------------------------
    b0..6           Frame header and preamble
    b7              Tags Found
    b8              Tag Number (only one used in this example)
    b9..10          SENS_RES
    b11             SEL_RES
    b12             NFCID Length
    b13..NFCIDLen   NFCID                                      */

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Found ")); PN532DEBUGPRINT.print(pn532_packetbuffer[7], DEC); PN532DEBUGPRINT.println(F(" tags"));
  #endif
  if (pn532_packetbuffer[7] != 1)
    return 0;

  uint16_t sens_res = pn532_packetbuffer[9];
  sens_res <<= 8;
  sens_res |= pn532_packetbuffer[10];
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("ATQA: 0x"));  PN532DEBUGPRINT.println(sens_res, HEX);
    PN532DEBUGPRINT.print(F("SAK: 0x"));  PN532DEBUGPRINT.println(pn532_packetbuffer[11], HEX);
  #endif

  /* Card appears to be Mifare Classic */
  *uidLength = pn532_packetbuffer[12];
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("UID:"));
  #endif
  for (uint8_t i=0; i < pn532_packetbuffer[12]; i++)
  {
    uid[i] = pn532_packetbuffer[13+i];
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.print(F(" 0x"));PN532DEBUGPRINT.print(uid[i], HEX);
    #endif
  }
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.println();
  #endif

  return 1;
}

/**************************************************************************/
/*!
    @brief  Exchanges an APDU with the currently inlisted peer

    @param  send            Pointer to data to send
    @param  sendLength      Length of the data to send
    @param  response        Pointer to response data
    @param  responseLength  Pointer to the response data length
*/
/**************************************************************************/
bool NFC_PN532::inDataExchange(uint8_t * send, uint8_t sendLength, uint8_t * response, uint8_t * responseLength) {
  if (sendLength > PN532_PACKBUFFSIZ-2) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("APDU length too long for packet buffer"));
    #endif
    return false;
  }
  uint8_t i;

  pn532_packetbuffer[0] = 0x40; // PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = _inListedTag;
  for (i=0; i<sendLength; ++i) {
    pn532_packetbuffer[i+2] = send[i];
  }

  if (!sendCommandCheckAck(pn532_packetbuffer,sendLength+2,1000)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Could not send APDU"));
    #endif
    return false;
  }

  if (!waitready(1000)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Response never received for APDU..."));
    #endif
    return false;
  }

  readdata(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Length check invalid"));
        PN532DEBUGPRINT.println(length,HEX);
        PN532DEBUGPRINT.println((~length)+1,HEX);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INDATAEXCHANGE) {
      if ((pn532_packetbuffer[7] & 0x3f)!=0) {
        #ifdef PN532DEBUG
          PN532DEBUGPRINT.println(F("Status code indicates an error"));
        #endif
        return false;
      }

      length -= 3;

      if (length > *responseLength) {
        length = *responseLength; // silent truncation...
      }

      for (i=0; i<length; ++i) {
        response[i] = pn532_packetbuffer[8+i];
      }
      *responseLength = length;

      return true;
    }
    else {
      PN532DEBUGPRINT.print(F("Don't know how to handle this command: "));
      PN532DEBUGPRINT.println(pn532_packetbuffer[6],HEX);
      return false;
    }
  }
  else {
    PN532DEBUGPRINT.println(F("Preamble missing"));
    return false;
  }
}

/**************************************************************************/
/*!
    @brief  'InLists' a passive target. PN532 acting as reader/initiator,
            peer acting as card/responder.
*/
/**************************************************************************/
bool NFC_PN532::inListPassiveTarget() {
  pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
  pn532_packetbuffer[1] = 1;
  pn532_packetbuffer[2] = 0;

  #ifdef PN532DEBUG
    PN532DEBUGPRINT.print(F("About to inList passive target"));
  #endif

  if (!sendCommandCheckAck(pn532_packetbuffer,3,1000)) {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Could not send inlist message"));
    #endif
    return false;
  }

  if (!waitready(30000)) {
    return false;
  }

  readdata(pn532_packetbuffer,sizeof(pn532_packetbuffer));

  if (pn532_packetbuffer[0] == 0 && pn532_packetbuffer[1] == 0 && pn532_packetbuffer[2] == 0xff) {
    uint8_t length = pn532_packetbuffer[3];
    if (pn532_packetbuffer[4]!=(uint8_t)(~length+1)) {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Length check invalid"));
        PN532DEBUGPRINT.println(length,HEX);
        PN532DEBUGPRINT.println((~length)+1,HEX);
      #endif
      return false;
    }
    if (pn532_packetbuffer[5]==PN532_PN532TOHOST && pn532_packetbuffer[6]==PN532_RESPONSE_INLISTPASSIVETARGET) {
      if (pn532_packetbuffer[7] != 1) {
        #ifdef PN532DEBUG
        PN532DEBUGPRINT.println(F("Unhandled number of targets inlisted"));
        #endif
        PN532DEBUGPRINT.println(F("Number of tags inlisted:"));
        PN532DEBUGPRINT.println(pn532_packetbuffer[7]);
        return false;
      }

      _inListedTag = pn532_packetbuffer[8];
      PN532DEBUGPRINT.print(F("Tag number: "));
      PN532DEBUGPRINT.println(_inListedTag);

      return true;
    } else {
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F("Unexpected response to inlist passive host"));
      #endif
      return false;
    }
  }
  else {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println(F("Preamble missing"));
    #endif
    return false;
  }

  return true;
}


/***** Mifare Classic Functions ******/

/**************************************************************************/
/*!
      Indicates whether the specified block number is the first block
      in the sector (block 0 relative to the current sector)
*/
/**************************************************************************/
bool NFC_PN532::mifareclassic_IsFirstBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock) % 4 == 0);
  else
    return ((uiBlock) % 16 == 0);
}

/**************************************************************************/
/*!
      Indicates whether the specified block number is the sector trailer
*/
/**************************************************************************/
bool NFC_PN532::mifareclassic_IsTrailerBlock (uint32_t uiBlock)
{
  // Test if we are in the small or big sectors
  if (uiBlock < 128)
    return ((uiBlock + 1) % 4 == 0);
  else
    return ((uiBlock + 1) % 16 == 0);
}

/**************************************************************************/
/*!
    Tries to authenticate a block of memory on a MIFARE card using the
    INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
    for more information on sending MIFARE and other commands.

    @param  uid           Pointer to a byte array containing the card UID
    @param  uidLen        The length (in bytes) of the card's UID (Should
                          be 4 for MIFARE Classic)
    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  keyNumber     Which key type to use during authentication
                          (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
    @param  keyData       Pointer to a byte array containing the 6 byte
                          key value

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t NFC_PN532::mifareclassic_AuthenticateBlock (uint8_t * uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t * keyData)
{
  uint8_t len;
  uint8_t i;

  // Hang on to the key and uid data
  memcpy (_key, keyData, 6);
  memcpy (_uid, uid, uidLen);
  _uidLen = uidLen;

  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to authenticate card "));
    NFC_PN532::PrintHex(_uid, _uidLen);
    PN532DEBUGPRINT.print(F("Using authentication KEY "));PN532DEBUGPRINT.print(keyNumber ? 'B' : 'A');PN532DEBUGPRINT.print(F(": "));
    NFC_PN532::PrintHex(_key, 6);
  #endif

  // Prepare the authentication command //
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;   /* Data Exchange Header */
  pn532_packetbuffer[1] = 1;                              /* Max card numbers */
  pn532_packetbuffer[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;
  pn532_packetbuffer[3] = blockNumber;                    /* Block Number (1K = 0..63, 4K = 0..255 */
  memcpy (pn532_packetbuffer+4, _key, 6);
  for (i = 0; i < _uidLen; i++)
  {
    pn532_packetbuffer[10+i] = _uid[i];                /* 4 byte card ID */
  }

  if (! sendCommandCheckAck(pn532_packetbuffer, 10+_uidLen))
    return 0;

  // Read the response packet
  readdata(pn532_packetbuffer, 12);

  // check if the response is valid and we are authenticated???
  // for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
  // Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("Authentification failed: "));
      NFC_PN532::PrintHexChar(pn532_packetbuffer, 12);
    #endif
    return 0;
  }

  return 1;
}

/**************************************************************************/
/*!
    Tries to read an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          Pointer to the byte array that will hold the
                          retrieved data (if any)

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t NFC_PN532::mifareclassic_ReadDataBlock (uint8_t blockNumber, uint8_t * data)
{
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Trying to read 16 bytes from block "));PN532DEBUGPRINT.println(blockNumber);
  #endif

  /* Prepare the command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 4))
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Failed to receive ACK for read command"));
    #endif
    return 0;
  }

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);

  /* If byte 8 isn't 0x00 we probably have an error */
  if (pn532_packetbuffer[7] != 0x00)
  {
    #ifdef MIFAREDEBUG
      PN532DEBUGPRINT.println(F("Unexpected response"));
      NFC_PN532::PrintHexChar(pn532_packetbuffer, 26);
    #endif
    return 0;
  }

  /* Copy the 16 data bytes to the output buffer        */
  /* Block content starts at byte 9 of a valid response */
  memcpy (data, pn532_packetbuffer+8, 16);

  /* Display data for debug if requested */
  #ifdef MIFAREDEBUG
    PN532DEBUGPRINT.print(F("Block "));
    PN532DEBUGPRINT.println(blockNumber);
    NFC_PN532::PrintHexChar(data, 16);
  #endif

  return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 16-byte data block at the specified block
    address.

    @param  blockNumber   The block number to authenticate.  (0..63 for
                          1KB cards, and 0..255 for 4KB cards).
    @param  data          The byte array that contains the data to write.

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t NFC_PN532::mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t * data)
{
//   #ifdef MIFAREDEBUG
//     PN532DEBUGPRINT.print(F("Trying to write 16 bytes to block "));PN532DEBUGPRINT.println(blockNumber);
//   #endif

  /* Prepare the first command */
  pn532_packetbuffer[0] = PN532_COMMAND_INDATAEXCHANGE;
  pn532_packetbuffer[1] = 1;                      /* Card number */
  pn532_packetbuffer[2] = MIFARE_CMD_WRITE;       /* Mifare Write command = 0xA0 */
  pn532_packetbuffer[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */
  memcpy (pn532_packetbuffer+4, data, 16);        /* Data Payload */

  /* Send the command */
  if (! sendCommandCheckAck(pn532_packetbuffer, 20))
  {
//     #ifdef MIFAREDEBUG
//       PN532DEBUGPRINT.println(F("Failed to receive ACK for write command"));
//     #endif
    return 0;
  }
  _delay_ms(10);

  /* Read the response packet */
  readdata(pn532_packetbuffer, 26);

  return 1;
}

/************** high level communication functions (handles both I2C and SPI) */

/**************************************************************************/
/*!
    @brief  Tries to read the I2C ACK signal
*/
/**************************************************************************/
bool NFC_PN532::readack() {
  uint8_t ackbuff[6];

  readdata(ackbuff, 6);

  return (0 == strncmp((char *)ackbuff, (char *)pn532ack, 6));
}


/**************************************************************************/
/*!
    @brief  Return true if the PN532 is ready with a response.
*/
/**************************************************************************/
bool NFC_PN532::isready() {
  
    // I2C check if status is ready by IRQ line being pulled low.
    uint8_t x = digitalRead(_irq);
    return x == 0;
  
}

/**************************************************************************/
/*!
    @brief  Waits until the PN532 is ready.

    @param  timeout   Timeout before giving up
*/
/**************************************************************************/
bool NFC_PN532::waitready(uint16_t timeout) {
  uint16_t timer = 0;
  while(!isready()) {
    if (timeout != 0) {
      timer += 10;
      if (timer > timeout) {
        //PN532DEBUGPRINT.println("TIMEOUT!");
        return false;
      }
    }
    _delay_ms(10);
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Reads n bytes of data from the PN532 via SPI or I2C.

    @param  buff      Pointer to the buffer where data will be written
    @param  n         Number of bytes to be read
*/
/**************************************************************************/
void NFC_PN532::readdata(uint8_t* buff, uint8_t n) {
 
     // I2C write.
    uint16_t timer = 0;

    _delay_ms(2);

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("Reading: "));
    #endif
    // Start read (n+1 to take into account leading 0x01 with I2C)
    WIRE.requestFrom((uint8_t)PN532_I2C_ADDRESS, (uint8_t)(n+2));
    // Discard the leading 0x01
    i2c_recv();
    for (uint8_t i=0; i<n; i++) {
      _delay_ms(1);
      buff[i] = i2c_recv();
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F(" 0x"));
        PN532DEBUGPRINT.print(buff[i], HEX);
      #endif
    }
    // Discard trailing 0x00 0x00
    // i2c_recv();

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.println();
    #endif
}

/**************************************************************************/
/*!
    @brief  Writes a command to the PN532, automatically inserting the
            preamble and required frame details (checksum, len, etc.)

    @param  cmd       Pointer to the command buffer
    @param  cmdlen    Command length in bytes
*/
/**************************************************************************/
void NFC_PN532::writecommand(uint8_t* cmd, uint8_t cmdlen) {
  
    // I2C command write.
    uint8_t checksum;

    cmdlen++;

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F("\nSending: "));
    #endif

    _delay_ms(2);     // or whatever the delay is for waking up the board

    // I2C START
    WIRE.beginTransmission(PN532_I2C_ADDRESS);
    checksum = PN532_PREAMBLE + PN532_PREAMBLE + PN532_STARTCODE2;
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_PREAMBLE);
    i2c_send(PN532_STARTCODE2);

    i2c_send(cmdlen);
    i2c_send(~cmdlen + 1);

    i2c_send(PN532_HOSTTOPN532);
    checksum += PN532_HOSTTOPN532;

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)PN532_PREAMBLE, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)PN532_PREAMBLE, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)PN532_STARTCODE2, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)cmdlen, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)(~cmdlen + 1), HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)PN532_HOSTTOPN532, HEX);
    #endif

    for (uint8_t i=0; i<cmdlen-1; i++) {
      i2c_send(cmd[i]);
      checksum += cmd[i];
      #ifdef PN532DEBUG
        PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)cmd[i], HEX);
      #endif
    }

    i2c_send((byte)~checksum);
    i2c_send((byte)PN532_POSTAMBLE);

    // I2C STOP
    WIRE.endTransmission();

    #ifdef PN532DEBUG
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)~checksum, HEX);
      PN532DEBUGPRINT.print(F(" 0x")); PN532DEBUGPRINT.print((byte)PN532_POSTAMBLE, HEX);
      PN532DEBUGPRINT.println();
    #endif
}