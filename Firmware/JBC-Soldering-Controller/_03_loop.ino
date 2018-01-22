/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Main Loop------------------------
void loop(void)
{
  static bool in_cradle;

  //read the encoder knob
  encoder_pos = knob.read() >> 1;  //divide by 2 to decrease sensitivity
  if (encoder_pos < 0 || fastDigitalRead(ENC_BUTTON) == false)
  {
    knob.write(0);
    encoder_pos = 0;
  }


  //current_sense_raw=analogRead(CURRENT_SENSE);  need to figure out how to read this only when the PWM is on.


  //TODO: Cradle logic:
  //TOOD: If off cradle and time < x seconds, then tip ON at MAIN setpoint. If Off cradle and time >= x seconds, then tip off (maybe someone left the handle on the table. yikes!).
  //TOOD: If On cradle and time < y seconds, then tip ON at IDLE setpoint.  If ON cradle and time >= y seconds, then tip off (someone hasnt used the tip in a long time so power down).
  //if (in_cradle)
  {
    //Setpoint = 0; //turn off the power while in the cradle. //temporary disable for testing PID tuning
    //pixels.setPixelColor(0, pixels.Color(0,0,10)); // Blue
  }
  //else
  {
    //Setpoint = enc; //set the PID loop to our encoder knob value
    //pixels.setPixelColor(0, pixels.Color(10,0,0)); // Red
  }
  //pixels.show();
  //enable interrupts
  ProcessSerialComm();
}




