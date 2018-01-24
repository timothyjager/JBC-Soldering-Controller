/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Main Loop------------------------
void loop(void)
{
  static bool in_cradle;
  bool update_display_now = false;

  //read the encoder knob
  static int16_t knob_pos_last = 0;

  status.encoder_pos = knob.read(); //>> 2;  //divide by 2 to decrease sensitivity

  //Check if knob has changed
  if (status.encoder_pos != knob_pos_last)
  {
    //Dont allow it to go negative.
    if (status.encoder_pos < 0)
    {
      knob.write(0);
      status.encoder_pos = 0;
    }
    params.setpoint = status.encoder_pos;
    if (myPID.GetMode() == AUTOMATIC)
    {
      status.pid_setpoint = params.setpoint;
    }

    update_display_now = true;  //refresh the screen more quickly while adjusting the knob
  }
  knob_pos_last = status.encoder_pos;

  //When button is pressed toggle power on/off
  if (fastDigitalRead(ENC_BUTTON) == false)
  {
    noInterrupts();
    if (params.pid_mode == AUTOMATIC)
    {
      params.pid_mode = MANUAL;
      myPID.SetMode(params.pid_mode);
      status.pid_output = 0;
      status.pid_setpoint = 0;
    }
    else
    {
      params.pid_mode = AUTOMATIC;
      myPID.SetMode(params.pid_mode);
      status.pid_setpoint = params.setpoint;
    }
    interrupts();
    delay(200);
    while (fastDigitalRead(ENC_BUTTON) == false)
      delay(200);
  }


  //When on cradle power on/off
  if (fastDigitalRead(CRADLE_SENSOR) == false)
  {
    noInterrupts();
    if (params.pid_mode == AUTOMATIC)
    {
      params.pid_mode = MANUAL;
      myPID.SetMode(params.pid_mode);
      status.pid_output = 0;
      status.pid_setpoint = 0;
    }
    interrupts();
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
  ProcessSerialComm();
  updateDisplay(update_display_now);
}




