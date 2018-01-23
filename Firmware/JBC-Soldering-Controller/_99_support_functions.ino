/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Support Functions------------------------


//multimap2
//allows for extrapolation if the input value is outside the bounds of the lookup array
//This is a modified version of multimap: https://playground.arduino.cc/Main/MultiMap
//note: the _in array should have increasing values
int multiMap2(int val, int* _in, int* _out, uint8_t size)
{
  // take care the value is within range
  // val = constrain(val, _in[0], _in[size-1]);
  if (val <= _in[0]) return _out[0];
  //if (val >= _in[size-1]) return _out[size-1];  //extrapolate the value using the last 2 xy pairs.
  //if (val >= _in[size-1]) return _out[size-1];  //extrapolate the value using the last 2 xy pairs.

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while (val > _in[pos] && pos < size - 1) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos - 1]) * (_out[pos] - _out[pos - 1]) / (_in[pos] - _in[pos - 1]) + _out[pos - 1];
}



//try to read the wattage of the power supply
void Check_DELL_PSU(void)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  int PSU_VOLTAGE_RAW = analogRead(VIN_SENSE);

  if (dell.read_data() == true)
  {
    display.print(dell.watts());
    display.print("W ");
    display.print(dell.millivolts());
    display.print("mv ");
    display.print(dell.milliamps());
    display.println("mA ");
    display.println(dell.response_string());
    //Serial.println(dell.response_string());
  }
  else
  {
    display.print("supply not detected");
    display.setCursor(0, 20);
    // read the raw power supply voltage from the resistor divider:
    int PSU_VOLTAGE_RAW = analogRead(VIN_SENSE);
    long PSU_MILLIVOLTS = map(PSU_VOLTAGE_RAW, 0, 1023, 0.0, 34412); //Resistor divider (30k / 5.1k) scales 34.4V to 5V
    display.print(PSU_MILLIVOLTS);
    display.print("mV ");
  }
  display.display();
  delay(4000);
}

//This function quickly toggles a pin.
//Used for debugging with the logic analyzer
void PulsePin(int pin)
{
  fastDigitalWrite(pin, HIGH);
  fastDigitalWrite(pin, LOW);
}


//This function updates the LED status
//TODO: Implement better LED colors:  blue if the power is off and the tip is 'cold'.  Yellow if the power is off but the tip is still 'hot'. Red if the power is on. Or maybe transition smoothly to red as it heats up. 
void updateLEDStatus(void)
{
    if (status.pid_setpoint == 0)
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 10)); // Blue
    }
    else
    {
      pixels.setPixelColor(0, pixels.Color(10, 0, 0)); // Red
    }
    pixels.show();
}



