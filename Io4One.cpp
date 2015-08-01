/*
 * ----------------------------------------------------------------------------
 *            _____ _           _                   _
 *           | ____| | ___  ___| |_ _ __ ___  _ __ (_) ___
 *           |  _| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __|
 *           | |___| |  __/ (__| |_| | | (_) | | | | | (__
 *           |_____|_|\___|\___|\__|_|  \___/|_| |_|_|\___|
 *            ____                   _   ____
 *           / ___|_      _____  ___| |_|  _ \ ___  __ _ ___
 *           \___ \ \ /\ / / _ \/ _ \ __| |_) / _ \/ _` / __|
 *            ___) \ V  V /  __/  __/ |_|  __/  __/ (_| \__ \
 *           |____/ \_/\_/ \___|\___|\__|_|   \___|\__,_|___/
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 */
#include <Arduino.h>
#include <Wire.h>
#include "Io4One.h"
 
#define PCA9537_ADDRESS                 0x49

#define PCA9537_REG_INP                 0
#define PCA9537_REG_OUT                 1
#define PCA9537_REG_POL                 2
#define PCA9537_REG_CTRL                3  

uint8_t pinNum2bitNum[] = { 0x01, 0x02, 0x04, 0x08 };

/***************************************************************************
 *
 *  Writes 8-bits to the specified destination register
 *
 **************************************************************************/
static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value) 
{
  Wire.beginTransmission(i2cAddress);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

/***************************************************************************
 *
 * Reads 8-bits from the specified source register
 *
 **************************************************************************/
static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) 
{
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)1);
  return Wire.read();
}

/***************************************************************************
 *
 * Constructor for the Io4OneClass class, not much here yet 
 *
 **************************************************************************/
Io4OneClass::Io4OneClass(void) { }

/***************************************************************************
 *
 * Begin method. This method must be called before using this library
 * either directly if the class is initializing the Wire library or by
 * calling this library's function begin(sda, scl) in which case that
 * function will call this one.
 *
 **************************************************************************/
void Io4OneClass::begin(void)
{
  // Read out default values from the registers to the shadow variables.
  m_inp = readRegister(PCA9537_ADDRESS, PCA9537_REG_INP);
  m_out = readRegister(PCA9537_ADDRESS, PCA9537_REG_OUT);
  m_pol = readRegister(PCA9537_ADDRESS, PCA9537_REG_POL);
  m_ctrl = readRegister(PCA9537_ADDRESS, PCA9537_REG_CTRL);
}

#if defined(ARDUINO_ARCH_ESP8266)
/***************************************************************************
 *
 * Convenience method for ESP8266 systems such as the ESP210.
 *
 **************************************************************************/
void Io4OneClass::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  begin();
}
#endif

/***************************************************************************
 *
 * Sets the desired pin mode
 *
 **************************************************************************/
boolean Io4OneClass::pinMode(uint8_t pin, uint8_t mode)
{
  // Make sure the pin number is OK
  if (pin > 3) {
    return false;
  }

  // Calculate the new control register value
  if (mode == OUTPUT) {
    m_ctrl &= ~pinNum2bitNum[pin];
  } else if (mode == INPUT) {
    m_ctrl |= pinNum2bitNum[pin];
  } else {
    return false;
  }

  writeRegister(PCA9537_ADDRESS, PCA9537_REG_CTRL, m_ctrl);

  return true;
}


/***************************************************************************
 *
 * Sets the desired pin polarity. This can be used to invert inverse
 * hardware logic.
 *
 **************************************************************************/
boolean Io4OneClass::pinPolarity(uint8_t pin, uint8_t polarity)
{
  // Make sure pin number is OK
  if (pin > 3) {
    return false;
  }
  
  if (polarity == INVERTED) {
    m_pol |= pinNum2bitNum[pin];
  } else if (polarity == NORMAL) {
    m_pol &= ~pinNum2bitNum[pin];
  } else {
    return false;
  }

  writeRegister(PCA9537_ADDRESS, PCA9537_REG_POL, m_pol);

  return true;
}

/***************************************************************************
 *
 * Write digital value to pin
 *
 **************************************************************************/
boolean Io4OneClass::digitalWrite(uint8_t pin, boolean val)
{
  // Make sure pin number is OK
  if (pin > 3) {
    return false;
  }

  if (val == HIGH) {
    m_out |= pinNum2bitNum[pin];
  } else {
    m_out &= ~pinNum2bitNum[pin];
  }

  writeRegister(PCA9537_ADDRESS, PCA9537_REG_OUT, m_out);
}

/***************************************************************************
 *
 * Read digital value from pin.
 * Note, so far this function will fail silently if the pin parameter is
 * incorrectly specified.
 *
 **************************************************************************/
boolean Io4OneClass::digitalRead(uint8_t pin)
{
  return (readRegister(PCA9537_ADDRESS, PCA9537_REG_INP) & pinNum2bitNum[pin] != 0);
}

Io4OneClass Io4One;

