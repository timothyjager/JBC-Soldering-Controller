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
  static bool rising_edge = true;
  //Initiate the ADC reading and read back the internal temperature from last time
  if (rising_edge)
  {
    //for (int i=0;i<100;i++)
    //{
    //  fastDigitalWrite(CS, LOW);
    //}
    //read back the internal temp while simultaneously changing the config to start a oneshot read from the ADC)
    fastDigitalWrite(CS, LOW);
    status.adc_ic_temp_counts = SPI.transfer16(ADS1118_SINGLE_SHOT_ADC);
    //temperature_value=SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
    fastDigitalWrite(CS, HIGH);
    fastDigitalWrite(CS, LOW);
    //PulsePin(debug_pin_B);
    
  }
  //else retreive the reading from the ADC
  else
  {
    //2 pulses for debugging
    //PulsePin(debug_pin_B);
    //PulsePin(debug_pin_B);
    //read back the tip temperature from the ADC while simultaneously changing the config to start a oneshot read of internal IC temperature (i.e. cold junction temp)
    fastDigitalWrite(CS, LOW);
    int16_t adc_raw = SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
    //static int16_t adc_raw_last;
    //this is a temporary hack because sometimes the ADC returns an odd value.  TODO: investigate this. 
    //if (abs(adc_raw-adc_raw_last)<1000)
    //{
    status.adc_counts = adc_raw;
    //}
    //else
    //{
     //PulsePin(debug_pin_B); 
    //}
    //adc_raw_last = adc_raw;  
 
    double tip_temp_c = (0.2925 * (double)status.adc_counts) + 3.4536;

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
    Timer1.pwm(LPINA, status.pid_output); //100% = 1023
    //TODO: verify calling neopixel code within the interrupt doesn't affect the the heater control
    updateLEDStatus();
    
    //Serial.println(millis()-time_start);
    fastDigitalWrite(CS, HIGH);

    //PulsePin(debug_pin_B);
    //PulsePin(debug_pin_B);
  }
  //every other interrupt will be a rising edge, we need to keep track of which is which
  rising_edge = !rising_edge;
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
  //4 pulses for debugging with logic analyzer
  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);
}

