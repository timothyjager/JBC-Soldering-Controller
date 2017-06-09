/* Copyright (c) 2017 timothyjager
 * JBC-Soldering-Controller
 * 
 * MIT License. See LICENSE file for details. 
 */
 
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>       //https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_SSD1306.h>   //https://github.com/adafruit/Adafruit_SSD1306
#include <Encoder.h>            //https://github.com/PaulStoffregen/Encoder
#include <TimerOne.h>           //https://github.com/PaulStoffregen/TimerOne
#include <DigitalIO.h>          //https://github.com/greiman/DigitalIO
#include <dell_psu.h>           //https://github.com/timothyjager/DellPSU
#include <EEWrap.h>             //https://github.com/Chris--A/EEWrap
#include <PID_v1.h>             //https://github.com/br3ttb/Arduino-PID-Library/

#include "ads1118.h"


//Pin Mapping
const int ENC_A            = 0;
const int ENC_B            = 1;
const int I2C_SDA          = 2;
const int I2C_SCL          = 3;
const int SPARE_4          = 4;
const int DELL_PSU         = 5;  
const int debug_pin_B      = 6;
const int debug_pin_A      = 7;
const int CS               = 7;
const int SPARE_8          = 8;
const int LPINA            = 9;  //Heater PWM
const int LPINB            = 10;
const int SPI_MISO         = 14;
const int SPI_SCLK         = 15;
const int SPI_MOSI         = 16;
const int ENC_BUTTON       = 18; //Maybe this should be an interrupt. if it does, we need to move CS to a different pin
const int SPARE_19         = 19;
const int CURRENT_SENSE    = 20;
const int CRADLE_SENSOR    = 21;
const int OLED_RESET       = 4; //the library asks for a reset pin, but our OLED doesn't have one 

//EEProm parameter data (https://github.com/Chris--A/EEWrap)
//Use the xxx_e types rather than the standard types like uint8_t
struct NVOL{
  uint8_e a;  //TODO: add the actual non-volatile parameters to this struct.
  int16_e b;
  float_e c;
  uint8_e str[2][6];
};
//EEMEM tells the compiler that the object resides in the EEPROM
NVOL nvol EEMEM; 

//Hard-coded calibration points.  TODO: make these not hard-coded
#define NUM_CAL_POINTS 4
//uint16_t microvolts [NUM_CAL_POINTS] = {0,10,20,30,40,50};
uint16_t adc_reading [NUM_CAL_POINTS] = {283,584,919,1098}; 
uint16_t deg_c [NUM_CAL_POINTS] = {105,200,300,345};

//x = multiMap2(raw, adc_reading, deg_c, NUM_CAL_POINTS);

//Volatile Variables used by the interrupt handlers
volatile int16_t adc_value=0;          //ADC value read by ADS1118
volatile int16_t temperature_value=0;  //internal temp of ADS1118
volatile int16_t current_sense_raw;    //raw adc reading
double kP=2, kI=0, kD=0;

//Global Objects
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);  //TODO: look into this reset pin. The LCD i'm using does not have a reset pin, just PWR,GND,SDA,SCL
Encoder knob(ENC_A, ENC_B);
DellPSU dell(DELL_PSU);   //specify the desired Arduino pin number


//Serial comm /tuning unions/structs
union controller_packet_converter{
  struct {
      byte  start_of_packet; //always 0xBA. It was arbitrarily chosen.
      byte  automatic;
      float setpoint;
      float input;
      float output;
      float kP;
      float kI;
      float kD;
      float ITerm;
  } status;
  byte asBytes[sizeof(status)]; 
};

union host_packet_converter{
  struct {
      byte start_of_packet; //this should always be 0xAB. It was arbitrarily chosen
      byte automatic;       //0=manual Auto =1
      float setpoint;
      float input;
      float output;
      float kP;
      float kI;
      float kD;
  } param;
  byte asBytes[sizeof(param)];
};

host_packet_converter host_packet;
controller_packet_converter controller_packet;


volatile double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,kP,kI,kD, DIRECT); //TODO: map this properly


//----------------Setup-------------------------
void setup(void)
{
  //Start our debug serial port
  Serial.begin(115200);
  //while(!Serial);

  //Read the NVOL data as a test.
  Serial.println(nvol.a);

  //setup cradle detect
  fastPinMode(CRADLE_SENSOR, INPUT_PULLUP);

  //setup encoder button
  fastPinMode(ENC_BUTTON,INPUT_PULLUP);
  
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
  #define PWM_MAX_DUTY 1023 //the timer 1 libray scales the PWM duty cycle from 0 to 1023 where 0=0% and 1023=100%
  #define ADC_SAMPLE_WINDOW_US 1200 //1200us = 1.2ms //we need 1.2 ms to sample the ADC. assuming 860 SPS setting
  #define ADC_SAMPLE_WINDOW_PWM_DUTY 961//(((PWM_PERIOD_US-ADC_SAMPLE_WINDOW_US)*PWM_MAX_DUTY)/PWM_PERIOD_US)  // we set our PWM duty to as close to 100% as possible while still leaving enough time for the ADC sample.
  #define MAX_HEATER_PWM_DUTY ADC_SAMPLE_WINDOW_PWM_DUTY //our maximum allowable heater PWM duty is equal to the sampling window PWM duty.  
    
  Timer1.initialize(PWM_PERIOD_US);  
  Timer1.pwm(LPINB, ADC_SAMPLE_WINDOW_PWM_DUTY); 

  
  Timer1.pwm(LPINA, 60); //100% = 1023
  delay(PWM_PERIOD_MS); //make sure both PWM's have run at least one full period before enabling interrupts 

  //clear the A interrupt flag, so it doesn't fire right away, when we enable the A interrupt
  TIFR1 |= _BV(OCF1A);
  
  //enable comparator A interrupt vector. 
  //This will fire an interrupt on each edge of our main PWM. we only use this once to synchronise the B sampling interrupt. 
  TIMSK1 = _BV(OCIE1A);


  //enable the PID loop
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(20); //20ms  TODO: don't leave this hard-coded
  myPID.SetOutputLimits(0,900); //961max PWM, otherwise it will cut into the sample window TODO:dont leave hard coded
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
   //long time_start = millis();
   Input =adc_value; 
   myPID.Compute();
   Timer1.pwm(LPINA, Output); //100% = 1023 
   //Serial.println(millis()-time_start);
   //TODO: update the PWM duty
   fastDigitalWrite(CS, HIGH);
  }
  //every other interrupt will be a rising edge, we need to keep track of which is which
  rising_edge=!rising_edge;
}


//This should only run at the start of the program, then it disables itself
ISR(TIMER1_COMPA_vect)
{
  static bool one_shot = false;
  if (!one_shot)
  {
  //clear the Timer 1 B interrupt flag so it doesn't fire right after we enable it. We only want to detect new interrupts, not old ones.
  TIFR1 |= _BV(OCF1B);
  //Enable only interrupt B, This also disables A since A has done it's one and only job of synchronizing the B interrupt.
  TIMSK1 = _BV(OCIE1B);
  one_shot = true;
  }
//4 pulses for debugging
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B);
   current_sense_raw=analogRead(CURRENT_SENSE);
   PulsePin(debug_pin_B);
   PulsePin(debug_pin_B);

   // 
}





// The main program 
void loop(void)
{

  static long next_millis = millis()+1000; 

 int16_t enc = knob.read()>>2;
 if (enc<0 || fastDigitalRead(ENC_BUTTON)==false)
 {
    knob.write(0);
    enc=0;
 }


 //block interrupts while retreiving the temperature values.
 noInterrupts();
 int16_t adc_copy=adc_value;
 int16_t temperature_copy=temperature_value;
 int16_t current_sense_raw_copy = current_sense_raw;
 Setpoint = enc; //set the PID loop to our encooder knob value
 interrupts();
 
 //convert to degrees C
 temperature_copy=ADS1118_INT_TEMP_C(temperature_copy);

///////////////
   display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
 
//float tempfloat = (float)adc_copy * 0.1901 + 1.6192+(float)temperature_copy;

 display.print(temperature_copy);
 display.print(" ");
 display.print(adc_copy);
 if (dell.read_data()==true)
  {
 display.print(" ");
 display.print("PWR ");
  }
  else
  {
 display.print("     ");
  }
  if (fastDigitalRead(CRADLE_SENSOR)==false)
  {
 display.print(" ");
 display.print("CRDL ");
  }
  else
  {
 display.print("      ");
  }
display.setCursor(0,20);

display.print(enc);

display.print(" ");
display.print(current_sense_raw_copy);

 display.display();
 //Serial.print(millis());
 //Serial.print(" ");
 //Serial.print(adc_copy);
 //  Serial.print("ADC_SAMPLE_WINDOW_PWM_DUTY");
 // Serial.println(ADC_SAMPLE_WINDOW_PWM_DUTY);
 //Serial.print(" ");
 //Serial.println(tempfloat);
  //}
  //else
 // {
  //  display.print("plug in adapter");
  //}
  //display.display(); 

///////////////

static bool serial_active = false;

//send-receive with processing if it's time
  if(millis()>next_millis)
  {
    if (Serial.available())
    {
      if (SerialReceive())
      {
         serial_active=true;
      }
    }
    if (serial_active)
    {
      SendStatusPacket();
    }
    next_millis+=500;
  }
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


  // note: the _in array should have increasing values
//multimap2 allows for extrapolation if the input value is outside the bounds of the lookup array
int multiMap2(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  //if (val >= _in[size-1]) return _out[size-1];  //extrapolate the value using the last 2 xy pairs.
  //if (val >= _in[size-1]) return _out[size-1];  //extrapolate the value using the last 2 xy pairs.

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos] && pos<size-1) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

//this function sends all of our status values as a hex string. this reduces the CPU load on the MCU since it doesnt have to format float strings.
void SendStatusPacket()
{
  controller_packet.status.start_of_packet = 0xBA;
  controller_packet.status.setpoint = Setpoint;
  noInterrupts(); //make sure we disable interrupts while grabing these volatile values.
  controller_packet.status.input = Input;
  controller_packet.status.output = Output;
  controller_packet.status.ITerm = myPID.GetITerm();
  interrupts();
  controller_packet.status.kP = myPID.GetKp();
  controller_packet.status.kI = myPID.GetKi();
  controller_packet.status.kD = myPID.GetKd();
  controller_packet.status.automatic = (myPID.GetMode()==AUTOMATIC);
  int i;
  for (i=0;i<sizeof(controller_packet_converter);i++)
  {
     const char lookup[] = "0123456789abcdef";
     Serial.write(lookup[ controller_packet.asBytes[i] >> 4 ]);
     Serial.write(lookup[ controller_packet.asBytes[i] & 0x0f ]);
  }
  Serial.write('\n');
}

//Receive commands from the tuning PC app
bool SerialReceive()
{
  bool return_value = false;
  // read the bytes sent from tuning app
  int index=0;
  //if there are bytes in the serial buffer, then read them until we have received a full packet
  while(Serial.available()&&index<sizeof(host_packet_converter))
  {
   host_packet.asBytes[index]=Serial.read();
   index++;
  } 

  //when there are no more bytes to read, check if we read the correct number of bytes and make sure our first byte equals our hard-coded start of packet value
  if(index==sizeof(host_packet_converter) && host_packet.param.start_of_packet==0xAB)
  {
    //update PID settings
    Setpoint=host_packet.param.setpoint;
    //Input=host_packet.param.input;
    //only change the outpput if we are in manual mode
    if(host_packet.param.automatic==0)    
    {                                     
      Output=host_packet.param.output;    
      myPID.SetMode(MANUAL);              
    }
    else
    {
      myPID.SetMode(AUTOMATIC);              
    }
    //update the gains
    myPID.SetTunings(host_packet.param.kP, host_packet.param.kI, host_packet.param.kD);            // Set PID tunings  
  return_value=true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer
  return return_value;
}
