/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_GFX.h>       //https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>   //https://github.com/adafruit/Adafruit_SSD1306
#include <Encoder.h>            //https://github.com/PaulStoffregen/Encoder
#include <TimerOne.h>           //https://github.com/PaulStoffregen/TimerOne
#include <DigitalIO.h>          //https://github.com/greiman/DigitalIO
#include <dell_psu.h>           //https://github.com/timothyjager/DellPSU
#include <EEWrap.h>             //https://github.com/Chris--A/EEWrap
#include <PID_v1.h>             //https://github.com/br3ttb/Arduino-PID-Library/
#include <NeoPixelBus.h>

#include "ADS1118.h"            //Header file for TI Signma Delta ADC
#include <MsTimer2.h>

