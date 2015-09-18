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

#include <Wire.h>
#include <Io4One.h>

#define SDA_PIN      2
#define SCL_PIN      14

void setup() {
  // Initialize the Io4One library
  Io4One.begin(SDA_PIN, SCL_PIN);
  
  // Initialization could also be done like this
  // Wire.begin(SDA_PIN, SCL_PIN);
  // Io4One.begin();
  // Which is usefull if you have more than one +One
  // module in the system and the I2C bus only needs to be
  // initialized once.

  // Set pin mode of pin 0 to output
  Io4One.pinMode(0, OUTPUT);
  Io4One.pinMode(1, OUTPUT);
  Io4One.pinMode(2, OUTPUT);
  Io4One.pinMode(3, OUTPUT);
  // Make it low
  Io4One.digitalWrite(0, HIGH);
  Io4One.digitalWrite(1, HIGH);
  Io4One.digitalWrite(2, HIGH);
  Io4One.digitalWrite(3, HIGH);
}

void loop() {
  static uint8_t pin = 0;
  
  // Simply toggle one output pin.
  Io4One.digitalWrite(pin, LOW);
  delay(500);
  
  Io4One.digitalWrite(pin, HIGH);
  delay(500);
  
  if (++pin == 4)
    pin = 0;
}

