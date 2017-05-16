/* Copyright (c) 2017 timothyjager
 * JBC-Soldering-Controller
 * 
 * MIT License. See LICENSE file for details. 
 */
 
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_GFX.h>       //https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>   //https://github.com/adafruit/Adafruit_SSD1306
#include <Encoder.h>            //https://github.com/PaulStoffregen/Encoder
#include <TimerOne.h>           //https://github.com/PaulStoffregen/TimerOne
#include <DigitalIO.h>          //https://github.com/greiman/DigitalIO
#include <dell_psu.h>           //https://github.com/timothyjager/DellPSU

#include "ads1118.h"


//Pin Mapping
const int LPINA = 9;
const int LPINB = 10;
const int debug_pin_A = 7;
const int debug_pin_B = 6;
const int CS = 7; 
const int ENC_A = 2;  //TODO: determine which interrupt pins to use on the Leonardo
const int ENC_B = 3;  //TODO: determine which interrupt pins to use on the Leonardo
const int ENC_BUTTON = 5; //TODO: determine which  pins to use on the Leonardo


//Volatile Variables used by the interrupt handlers
volatile int16_t adc_value=0;          //ADC value read by ADS1118
volatile int16_t temperature_value=0;  //internal temp of ADS1118

//Global Objects
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);  //TODO: look into this reset pin. The LCD i'm using does not have a reset pin, just PWR,GND,SDA,SCL

DellPSU dell(1);   //specify the desired Arduino pin number


//----------------Setup-------------------------
void setup(void)
{
  //Start our debug serial port
  Serial.begin(115200);
  //while(!Serial);
  
  //Setup the OLED
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    // Clear the buffer.
    display.clearDisplay();

  //these are for debugging the Interrupts
  fastPinMode(debug_pin_A, OUTPUT);
  fastPinMode(debug_pin_B, OUTPUT);

    
  //Init the ADS1118 ADC
  //set chip select pin high as default
  fastPinMode(CS, OUTPUT);
  //set CS high as default
  fastDigitalWrite(CS, HIGH);
  //Start the SPI interface
  SPI.begin();
  //Setup the SPI parameters for the ADS1118
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  //start an initial reading of the internal temperature
  fastDigitalWrite(CS, LOW);
  SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
  fastDigitalWrite(CS, HIGH);
  delay(10);
  fastDigitalWrite(CS, LOW);
  temperature_value=SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
  fastDigitalWrite(CS, HIGH);
  delay(10);
 
  
  /*
   * The Timer1 interrupts fire at the rising and falling edges of the PWM pulse. 
   * A is used to control our main PWM power to the heater
   * B is used for sampling the ADC during the offtime of A
   * We only care about B for ADC sampling, but we need to know if the interrupt is the rising or falling edge. 
   * We want to sample the ADC right after the falling edge of B, and the internal temp after the rising edge of B 
   * Since the falling edge of B always occurs after either edge of A (since B always has a higher duty cycle), we can enable the B interrupt from A
   * and then just keep track of which edge we are. A bit is toggled each time the interrupt fires to remember if it's the rising or falling edge. 
   * This assumes we will never miss an interrupt. 
   * WE should probably add a handler in case the interrupt gets messed up and we get out of sync. 
  */
  //setup PWM and interrupts. These use the 16 bit timer1
  #define PWM_PERIOD_US 20000 //20000us = 20ms = 50Hz PWM frequency. We have to go slow enough to allow time for sampling the ADC during the PWM off time
  #define PWM_PERIOD_MS (PWM_PERIOD_US/1000)  
  Timer1.initialize(PWM_PERIOD_US);  
  Timer1.pwm(LPINB, 1023-61); //we need 1.2 ms to sample the ADC. assuming 860 SPS setting
  Timer1.pwm(LPINA, 2); //100% = 1023
  delay(PWM_PERIOD_MS); //make sure both PWM's have run at least one full period before enabling interrupts 

  //clear the A interrupt flag, so it doesn't fire right away, when we enable the A interrupt
  TIFR1 |= _BV(OCF1A);
  
  //enable comparator A interrupt vector. 
  //This will fire an interrupt on each edge of our main PWM. we only use this once to synchronise the B sampling interrupt. 
  TIMSK1 = _BV(OCIE1A);
}

void PulsePin(int pin)
{
   fastDigitalWrite(pin, HIGH);
   fastDigitalWrite(pin, LOW);
}




//This interrupt is set to fire approximately 1.2ms before the PWM output turn on. It is configured to allow just enough time to sample the ADC while the 
//power to the heater is turned off. This means that our max PWM has to be limited to ensure there is a long enough sample window.
//the interupt actually fires twice. once at the beginning of our interval and once at the end. we use this to our advantage by sampling the result of the 
//previous sample at each interrupt and then toggling the sample type back and forth between ADC and internal temp.
ISR(TIMER1_COMPB_vect)
{
static bool rising_edge=true;
  //Initiate the ADC reading and read back the internal temperature from last time
  if (rising_edge)
  {
   PulsePin(debug_pin_B);
   //read back the internal temp while simultaneously changing the config to start a oneshot read from the ADC)
   fastDigitalWrite(CS, LOW);
   temperature_value=SPI.transfer16(ADS1118_SINGLE_SHOT_ADC);
   //temperature_value=SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
   fastDigitalWrite(CS, HIGH);
   fastDigitalWrite(CS, LOW);
   
  }
  //else retreive the reading from the ADC
  else
  {
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B);
   //read back from the ADC while simultaneously changing the config to start a oneshot read of internal temp)
   fastDigitalWrite(CS, LOW);
   adc_value=SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
   fastDigitalWrite(CS, HIGH);
  }
  //every other interrupt will be a rising edge, we need to keep track of which is which
  rising_edge=!rising_edge;
}


//This should only run at the start of the program, then it disables itself
ISR(TIMER1_COMPA_vect)
{
  //clear the Timer 1 B interrupt flag so it doesn't fire right after we enable it. We only want to detect new interrupts, not old ones.
  TIFR1 |= _BV(OCF1B);
  //Enable only interrupt B, This also disables A since A has done it's one and only job of synchronizing the B interrupt.
  TIMSK1 = _BV(OCIE1B);
//4 pulses for debugging
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B); 
}





// The main program 
void loop(void)
{

 //block interrupts while retreiving the temperature values.
 noInterrupts();
 int16_t adc_copy=adc_value;
 int16_t temperature_copy=temperature_value;
 interrupts();
 
 //convert to degrees C
 temperature_copy=ADS1118_INT_TEMP_C(temperature_copy);
 
 
 //Serial.print(temperature_copy);
 //Serial.print(" ");
 //Serial.println(adc_copy);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);

  display.print(temperature_copy);
 display.print(" ");
 display.println(adc_copy);

display.display();

 delay(300); 
}


/*
   display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);

    
    if (dell.read_data()==true)
  {
    display.print(dell.watts());
    display.print("W "); 
    display.print(dell.millivolts());
    display.print("mv ");
    display.print(dell.milliamps());
    display.println("mA ");
    display.println(dell.response_string());
    //Serial.println(dell.milliamps());
    Serial.println(dell.response_string());
  }
  else
  {
    display.print("plug in adapter");
  }
  display.display(); 
  delay(1000); 
 
 */


 /*
  Serial.println(ADS1118_INT_TEMP_C(0x1000<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0FFF<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0C80<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0960<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0640<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0320<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0008<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0001<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x0000<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x3FF8<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x3CE0<<2));
  Serial.println(ADS1118_INT_TEMP_C(0x3B00<<2));
  delay(5000);
  */
