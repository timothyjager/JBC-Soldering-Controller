/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Main Loop------------------------
void loop(void)
{
  static bool in_cradle;

  static long next_millis = millis() + 1000;

  int16_t enc = knob.read() >> 1;
  if (enc < 0 || fastDigitalRead(ENC_BUTTON) == false)
  {
    knob.write(0);
    enc = 0;
  }


  //current_sense_raw=analogRead(CURRENT_SENSE);  need to figure out how to read this only when the PWM is on.

  //block interrupts while retreiving the temperature values.
  noInterrupts();
  int16_t adc_copy = adc_value;
  int16_t temperature_copy = temperature_value;
  int16_t current_sense_raw_copy = current_sense_raw;
  if (in_cradle)
  {
    //Setpoint = 0; //turn off the power while in the cradle. //temporary disable for testing PID tuning
    //pixels.setPixelColor(0, pixels.Color(0,0,10)); // Blue
  }
  else
  {
    //Setpoint = enc; //set the PID loop to our encoder knob value
    //pixels.setPixelColor(0, pixels.Color(10,0,0)); // Red
  }
  //pixels.show();
  //enable interrupts
  interrupts();

  //convert to degrees C
  temperature_copy = ADS1118_INT_TEMP_C(temperature_copy);

  ///////////////
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  //float tempfloat = (float)adc_copy * 0.1901 + 1.6192+(float)temperature_copy;

  display.print(temperature_copy);
  display.print(" ");
  display.print(adc_copy);
  if (dell.read_data() == true)
  {
    display.print(" ");
    display.print("DELL PWR ");
  }
  else
  {
    display.print("     ");
  }
  if (fastDigitalRead(CRADLE_SENSOR) == false)
  {
    display.print(" ");
    display.print("CRDL ");
    in_cradle = true;
  }
  else
  {
    in_cradle = false;
    display.print("      ");
  }
  display.setCursor(0, 20);

  display.print(enc);

  display.print(" ");
  display.print(current_sense_raw_copy);

  display.display();

  ///////////////

  static bool serial_active = false;

  //send-receive with processing if it's time
  if (millis() > next_millis)
  {
    if (Serial.available())
    {
      if (SerialReceive())
      {
        serial_active = true;
      }
    }
    if (serial_active)
    {
      SendStatusPacket();
    }
    next_millis += 200;
  }
}



//this function sends all of our status values as a hex string. this reduces the CPU load on the MCU since it doesnt have to format float strings.
void SendStatusPacket()
{
  controller_packet.status.start_of_packet = 0xBA;
  controller_packet.status.setpoint = Setpoint;
  noInterrupts(); //make sure we disable interrupts while grabing these volatile values.
  controller_packet.status.input = Input;
  controller_packet.status.output = Output;
  //controller_packet.status.ITerm = myPID.GetITerm();
  interrupts();
  controller_packet.status.kP = myPID.GetKp();
  controller_packet.status.kI = myPID.GetKi();
  controller_packet.status.kD = myPID.GetKd();
  controller_packet.status.automatic = (myPID.GetMode() == AUTOMATIC);
  controller_packet.status.simulate_input = host_packet.param.simulate_input;
  int i;
  for (i = 0; i < sizeof(controller_packet_struct); i++)
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
  int index = 0;
  //if there are bytes in the serial buffer, then read them until we have received a full packet
  while (Serial.available() && index < sizeof(host_packet_struct))
  {
    host_packet.asBytes[index] = Serial.read();
    index++;
  }

  //when there are no more bytes to read, check if we read the correct number of bytes and make sure our first byte equals our hard-coded start of packet value
  if (index == sizeof(host_packet_struct) && host_packet.param.start_of_packet == 0xAB)
  {
    //update PID settings
    Setpoint = host_packet.param.setpoint;
    //Input=host_packet.param.input;
    //only change the output if we are in manual mode
    if (host_packet.param.automatic == 0)
    {
      Output = host_packet.param.output;
      myPID.SetMode(MANUAL);
    }
    else
    {
      myPID.SetMode(AUTOMATIC);
    }
    //update the gains
    myPID.SetTunings(host_packet.param.kP, host_packet.param.kI, host_packet.param.kD);            // Set PID tunings
    return_value = true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer
  return return_value;
}
