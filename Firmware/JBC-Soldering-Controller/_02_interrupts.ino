/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Interrupts------------------------

//This interrupt is set to fire approximately 1.2ms before the PWM output turn on. It is configured to allow just enough time to sample the ADC while the
//power to the heater is turned off. This means that our max PWM has to be limited to ensure there is a long enough sample window.
//the interupt actually fires twice. once at the beginning of our interval and once at the end. we use this to our advantage by sampling the result of the
//previous sample at each interrupt and then toggling the sample type back and forth between ADC and internal temp.
ISR(TIMER1_COMPB_vect)
{
  static bool one_shot = false;
  //If we have never run this code before, then run it just once
  if (!one_shot)
  {
    MsTimer2::start();
    TIMSK1 = 0;                 //Disable interrupts on timer 1
    one_shot = true;
  }
}


//This should only run at the start of the program, then it disables itself
//It's used to start the B interupt at the right time
ISR(TIMER1_COMPA_vect)
{
  static bool one_shot = false;
  //If we have never run this code before, then run it just once
  if (!one_shot)
  {
    TIFR1 |= _BV(OCF1B);        //clear the Timer 1 B interrupt flag so it doesn't fire right after we enable it. We only want to detect new interrupts, not old ones.
    TIMSK1 = _BV(OCIE1B);       //Enable only interrupt B, This also disables A since A has done it's one and only job of synchronizing the B interrupt.
    one_shot = true;            //set oneshot to prevent this code from ever running again. It shouldn't try to since this interrupt should now be disable. 
    
  }
  //4 pulses for debugging with logi analyzer
  /*PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);*/
}

/*ISR(TIMER2_COMPA_vect)
{
  fastDigitalWrite(debug_pin_B, HIGH);
  fastDigitalWrite(debug_pin_B, LOW);
}*/

void flash() {
  /*static boolean output = HIGH;
  fastDigitalWrite(debug_pin_B, output);
  fastDigitalWrite(debug_pin_B, output);
  output = !output;
  */
  //static bool rising_edge = true;
  static int interrupt_count = 0;
  //Initiate the ADC reading and read back the internal temperature from last time
  //delay(1);
  //rising_edge = ~digitalRead(LPINA);
  if (interrupt_count == 0)
  {
    //PulsePin(debug_pin_B);
    //fastDigitalWrite(debug_pin_B, HIGH);
    //fastDigitalWrite(debug_pin_B, LOW);
    //read back the internal temp while simultaneously changing the config to start a oneshot read from the ADC)
    fastDigitalWrite(CS, LOW);
    status.adc_ic_temp_counts = SPI.transfer16(ADS1118_SINGLE_SHOT_ADC);
//    status.adc_ic_temp_counts = ADS.ADSread(1);
    //temperature_value=SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
    fastDigitalWrite(CS, HIGH);
    fastDigitalWrite(CS, LOW);
    interrupt_count++;
  }
  //else retreive the reading from the ADC
  else if(interrupt_count == 1)
  {
    //read back the tip temperature from the ADC while simultaneously changing the config to start a oneshot read of internal IC temperature (i.e. cold junction temp)
    fastDigitalWrite(CS, LOW);
    int16_t adc_raw = SPI.transfer16(ADS1118_SINGLE_SHOT_ADC_2);
    static int16_t adc_raw_last;
    //this is a temporary hack because sometimes the ADC returns an odd value.  TODO: investigate this. 
    if (abs(adc_raw-adc_raw_last)<1000)
    {
    status.adc_counts = adc_raw;
    }
    adc_raw_last = adc_raw;  
 
    double tip_temp_c = ADS.ADCcode2temp(status.adc_counts + ADS.localCompensation(status.adc_ic_temp_counts))/10.0;
    tip_temp_c = tip_temp_c*1.5384-17.198; //bodge to get temps right. The thermocouple is not quite a k type.
    
    //determine if PID loop should use the actual temperature or simulated temperature from the host.
    if (params.simulate_input == 1)
    {
      status.tip_temperature_c = params.simulated_input;
    }
    else
    {
      status.tip_temperature_c = tip_temp_c;
    }
    
    //Compute PID
    myPID.Compute();
    //Update PWM output
    if (status.tip_temperature_c > 1000) //If there is nothing connected...
    {
      Timer1.pwm(LPINA, 0); //100% = 1023 //...make sure that nothing is output. (this protects against the thermocouple disconnects)
    }
    else
    {
      Timer1.pwm(LPINA, status.pid_output); //100% = 1023
      if(disable_simultaneous_output)Timer1.pwm(LPINB, 0);//set other output to 0%
    }
    
    //Timer1.pwm(LPINB, status.pid_output); //100% = 1023
    //TODO: verify calling neopixel code within the interrupt doesn't affect the the heater control
    //updateLEDStatus();
    
    //Serial.println(millis()-time_start);
    fastDigitalWrite(CS, HIGH);
    interrupt_count++;
  }
  else
  {
    //read back the tip temperature from the ADC while simultaneously changing the config to start a oneshot read of internal IC temperature (i.e. cold junction temp)
    fastDigitalWrite(CS, LOW);
    int16_t adc_raw = SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
    static int16_t adc_raw_last;
    //this is a temporary hack because sometimes the ADC returns an odd value.  TODO: investigate this. 
    if (abs(adc_raw-adc_raw_last)<1000)
    {
    status.adc_counts = adc_raw;
    }
    adc_raw_last = adc_raw;  
 
    double tip_temp_c = ADS.ADCcode2temp(status.adc_counts + ADS.localCompensation(status.adc_ic_temp_counts))/10.0;
    tip_temp_c = tip_temp_c*1.5384-17.198; //bodge to get temps right. The thermocouple is not quite a k type.
    
    //determine if PID loop should use the actual temperature or simulated temperature from the host.
    if (params.simulate_input == 1)
    {
      status.tip_temperature_c2 = params.simulated_input;
    }
    else
    {
      status.tip_temperature_c2 = tip_temp_c;
    }
    
    //Compute PID
    myPID2.Compute();
    //Update PWM output
    if (status.tip_temperature_c2 > 1000) //If there is nothing connected...
    {
      Timer1.pwm(LPINB, 0); //100% = 1023 //...make sure that nothing is output. (this protects against the thermocouple disconnects)
    }
    else
    {
      Timer1.pwm(LPINB, status.pid_output2); //100% = 1023
      if(disable_simultaneous_output)Timer1.pwm(LPINA, 0);//set other output to 0%
    }
    
    
    fastDigitalWrite(CS, HIGH);
    interrupt_count = 0;
  }
}
