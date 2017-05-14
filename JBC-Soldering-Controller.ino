/*notes: on startup put into continuous internal temp mode.
 * 
 * 
 */


#include <TimerOne.h>
#include "DigitalIO.h"
#include <SPI.h>
#include "ads1118.h"

//Pin Mapping
const int LPINA = 9;
const int LPINB = 10;
const int debug_pin_A = 7;
const int debug_pin_B = 6;
const int CS = 7; 

//Volatile Variables used by the interrupt handlers
volatile int16_t adc_value=0;          //ADC value read by ADS1118
volatile int16_t temperature_value=0;  //internal temp of ADS1118

//Global Objects

//----------------Setup-------------------------
void setup(void)
{
  //Start our debug serial port
  Serial.begin(115200);
  while(!Serial);
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


//Converts the raw temperature value from the ADS1118 to an integer reading of degrees C.
//The incomming data arrives as a 16 bit value with the MSB first. 
//Since our actual data is 14 bit, we need to divide by 4 (or shift 2 bits to the right) to get the correct value
//Also per the datasheet page 18, we need to multiply by 0.03125 to convert this to Degrees F
//In order avoid floats, we will just divide by 32 by shifting right by 5 (plus 2 more to convert from 16 bit to 14 bit data for a total of 7 shifts)
int16_t ADS1118_INT_TEMP_C(int16_t internal_temp_raw)
{
  //per the datasheet, if the data returned is negative we need to take the 2's complement, then shift the data, then multiply by -0.03125
  return internal_temp_raw/128;
  /*if(internal_temp_raw < 0)
   {
    //internal_temp_raw *= -1 *(~(internal_temp_raw - 1) & 0x3FFF);  //
    internal_temp_raw *=-1; //2's complement (i.e. negate the value)
    internal_temp_raw>>=7;   //divide by 4 to get a 14 bit number, then divide by 32 to get degrees C. (same as shifting by 7)
    internal_temp_raw *=-1; //negate the number since it should be negative.   
   }
  else
  {
    internal_temp_raw>>=7;   //divide by 4 to get a 14 bit number, then divide by 32 to get degrees C. (same as shifting by 7)     
  }
  */
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
 
 //static int16_t last_temp=0;

 //if (last_temp!=temperature_copy)
 //{
 //Serial.print(adc_copy);
 //Serial.print("    ");
 
 Serial.print(temperature_copy);
 Serial.print(" ");
 Serial.println(adc_copy);
 //last_temp=temperature_copy;
 //}
 delay(300);
 
}
