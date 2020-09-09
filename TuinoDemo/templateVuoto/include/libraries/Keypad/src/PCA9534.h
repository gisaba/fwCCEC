/* 
* PCA9534.h
*
* Created: 03/09/2020 17:05:08
* Author: Administrator
*/

#include "Arduino.h"

const uint8_t PCA9534_I2C_ADDRESS    =  0x20;
const uint8_t PCA9534_IP_REGISTER    =  0x00;
const uint8_t PCA9534_OP_REGISTER    =  0x01;
const uint8_t PCA9534_INV_REGISTER   =  0x02;
const uint8_t PCA9534_CONF_REGISTER  =  0x03;
const uint8_t INPUT_INVERTED         =  0x04;


class PCA9534 {
public:
  /**
   * Constructor
   * Creates a new PCA9534 class to manage a PCA9534 chip.
   */
  PCA9534();

  /**
   * Initializes the device and performs initial I2C setup.
   * This method should be called before any others are used.
   *
   * @param {uint8_t} i2caddr - Sets the slave address of the PCA9534,
   * defaults to 0x20.
   */
  void begin(uint8_t i2caddr = PCA9534_I2C_ADDRESS);

  void setCONFREG(uint8_t CONFREG);

  void setporteIoExp(uint8_t OPREG,uint8_t INVREG,uint8_t CONFREG);

  uint8_t getIo();

  uint8_t Read_IP_REGISTER();

  /**
   * Configures the specified pin to behave either as an input, inverted input,
   * or output.
   *
   * @param {uint8_t} pin - Pin number whose mode you wish to set.
   * @param {uint8_t} mode - Pin mode one of: INPUT, INPUT_INVERTED, or OUTPUT.
   */
  void pinMode(uint8_t pin, uint8_t mode);

  /**
   * Writes a HIGH or a LOW value to a digital pin.
   *
   * @param {uint8_t} pin - Pin number whose value you wish to set.
   * @param {uint8_t} value - Pin value one of: HIGH, or LOW.
   */
  void digitalWrite(uint8_t pin, uint8_t value);

  /**
   * Reads the value from a specified digital pin, either HIGH or LOW.
   * **Note: When using INPUT_INVERTED on pinMode(), you will get the inverted
   * status.**
   *
   * @param {uint8_t} pin - Pin number whose value you wish to get.
   * @returns {uint8_t} The status of the pin either HIGH or LOW.
   */
  uint8_t digitalRead(uint8_t pin);

private:
  uint8_t _i2caddr; // I2C address of the device
  uint8_t _port; // Port configuration status on Configuration register
  uint8_t _invport; // Port inverted status on Polarity Inversion register
};