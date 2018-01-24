/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------OLED Display------------------------


//This updates the OLED display
void updateDisplay(bool update_now)
{
#define DISPLAY_UPDATE_PERIOD 250;
  static bool in_cradle;
  static long next_millis = millis() + DISPLAY_UPDATE_PERIOD;  //determine the next time this function should activate

  //Time to update the display?
  if (millis() > next_millis || update_now)
  {
    //block interrupts while retreiving the temperature values.
    noInterrupts();
    //int16_t adc_copy = status.adc_counts;
    int16_t tip_temperature_copy = status.tip_temperature_c;
    int16_t adc_ic_temp_counts_copy = status.adc_ic_temp_counts;
    interrupts();

    display.clearDisplay();
    display.setTextColor(WHITE);
    
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Set");
    display.setCursor(64, 0);
    display.print("Deg C");
    
    display.setTextSize(2);
    display.setCursor(0, 12);
    display.print((int16_t)params.setpoint);
    
    display.setCursor(64, 12);
    display.print(tip_temperature_copy);
     uint16_t pwm_bar = status.pid_output/7.5;  //scale max PWM value (961) down to 128 pixels. TODO: dont leave this hard coded. 
     display.fillRect(0, 29, pwm_bar, 3, WHITE);
    //display.print(" ");
    //display.print(ADS1118_INT_TEMP_C(adc_ic_temp_counts_copy));
    //display.setCursor(0, 20);

    //display.print(status.encoder_pos);

    
    display.display();

    next_millis += DISPLAY_UPDATE_PERIOD;  //set up our loop to run again in x ms
  }
}



/*
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
*/



