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
    PulsePin(debug_pin_B);
    //read back the internal temp while simultaneously changing the config to start a oneshot read from the ADC)
    fastDigitalWrite(CS, LOW);
    temperature_value = SPI.transfer16(ADS1118_SINGLE_SHOT_ADC);
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
    adc_value = SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
    //long time_start = millis();
    if (host_packet.param.simulate_input == 1)
    {
      Input = host_packet.param.input;
    }
    else
    {
      Input = adc_value;
    }
    myPID.Compute();
    Timer1.pwm(LPINA, Output); //100% = 1023
    //TODO: verify calling neopixel code within the interrupt doesn't affect the the heater control
    if (Setpoint == 0)
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 10)); // Blue
    }
    else
    {
      pixels.setPixelColor(0, pixels.Color(10, 0, 0)); // Red
    }
    pixels.show();
    //Serial.println(millis()-time_start);
    fastDigitalWrite(CS, HIGH);
  }
  //every other interrupt will be a rising edge, we need to keep track of which is which
  rising_edge = !rising_edge;
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

  PulsePin(debug_pin_B);
  PulsePin(debug_pin_B);

  //
}

