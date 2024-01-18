/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Setup-------------------------
void setup(void)
{
  //IO Pin Configuration
  fastPinMode(CRADLE_SENSOR, INPUT_PULLUP);    //setup cradle detect
  fastPinMode(ENC_BUTTON, INPUT_PULLUP);       //setup encoder button
  //fastPinMode(debug_pin_A, OUTPUT);            //for debugging the Interrupts
  //fastPinMode(debug_pin_B, OUTPUT);            //for debugging the Interrupts 
  fastPinMode(CS, OUTPUT);                     //SPI chip select pin

  //Serial Port
  Serial.begin(115200);

  //Neopixel
  pixels.Begin();                                  // This initializes the NeoPixel library.
  pixels.SetPixelColor(0, RgbwColor(0, 0, 10)); // Moderately bright green color.
  pixels.SetPixelColor(0, RgbwColor(1, 0, 10)); // Moderately bright green color.
  pixels.SetPixelColor(0, RgbwColor(2, 0, 10)); // Moderately bright green color.
  pixels.SetPixelColor(0, RgbwColor(3, 0, 10)); // Moderately bright green color.
  pixels.Show();

  //Read the NVOL data as a test.
  //Serial.println(nvol.a);

  //Setup the OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32). Module set to generate the OLED driver voltage internally
  display.setRotation(2);
  display.clearDisplay();                     // Clear the buffer.

  //Init for the ADS1118 ADC
//  ADS.begin(0);

  //Init the ADS1118 ADC
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
  status.adc_ic_temp_counts = SPI.transfer16(ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE);
  fastDigitalWrite(CS, HIGH);
  delay(10);


  //Interrupts
  /*
     The Timer1 interrupts fire at the rising and falling edges of the PWM pulse.
     A is used to control our main PWM power to the heater
     B is used for sampling the ADC during the offtime of A
     We only care about B for ADC sampling, but we need to know if the interrupt is the rising or falling edge.
     We want to sample the ADC right after the falling edge of B, and the internal temp after the rising edge of B
     Since the falling edge of B always occurs after either edge of A (since B always has a higher duty cycle), we can enable the B interrupt from A
     and then just keep track of which edge we are. A bit is toggled each time the interrupt fires to remember if it's the rising or falling edge.
     This assumes we will never miss an interrupt.
     WE should probably add a handler in case the interrupt gets messed up and we get out of sync.
  */
  //setup PWM and interrupts. These use the 16 bit timer1
#define PWM_PERIOD_US 50000                            //20000us = 20ms = 50Hz PWM frequency. We have to go slow enough to allow time for sampling the ADC during the PWM off time
#define PWM_PERIOD_MS (PWM_PERIOD_US/1000)             //20000/1000 = 20ms
#define PWM_MAX_DUTY 1023                              //the timer 1 libray scales the PWM duty cycle from 0 to 1023 where 0=0% and 1023=100%
#define ADC_SAMPLE_WINDOW_US 1600                      //1200us = 1.2ms //we need 1.2 ms to sample the ADC. assuming 860 SPS setting
#define ADC_SAMPLE_WINDOW_PWM_DUTY 941                 //(((PWM_PERIOD_US-ADC_SAMPLE_WINDOW_US)*PWM_MAX_DUTY)/PWM_PERIOD_US)  // we set our PWM duty to as close to 100% as possible while still leaving enough time for the ADC sample.
#define MAX_HEATER_PWM_DUTY ADC_SAMPLE_WINDOW_PWM_DUTY //our maximum allowable heater PWM duty is equal to the sampling window PWM duty.  

  Timer1.initialize(PWM_PERIOD_US);                    //Set timer1 to our main PWM period
  Timer1.pwm(LPINB, ADC_SAMPLE_WINDOW_PWM_DUTY);       //Set an interupt to define our sample window
  Timer1.pwm(LPINA, 60); //100% = 1023                 //Set a default PWM value for our output
  delay(PWM_PERIOD_MS);                                //make sure both PWM's have run at least one full period before enabling interrupts

  MsTimer2::set(50, flash); // 500ms period
  TIFR1 |= _BV(OCF1A);                                 //clear the A interrupt flag, so it doesn't fire right away, when we enable the A interrupt
  TIMSK1 = _BV(OCIE1A);                                //enable comparator A interrupt vector. This will fire an interrupt on each edge of our main PWM. we only use this once to synchronise the B sampling interrupt.
  //TIMSK2 |= _BV(OCIE2A);
  
  //MsTimer2::start();
  
  //Set up PID Control Loop
  myPID.SetMode(MANUAL);
  myPID.SetSampleTime(PWM_PERIOD_MS);                //Since we run out PID every interupt cylce, we set the sample time (ms) to our PWM timer period 
  myPID.SetOutputLimits(0, 650);     //961max PWM, otherwise it will cut into the sample window TODO:dont leave hard coded

  myPID2.SetMode(MANUAL);
  myPID2.SetSampleTime(PWM_PERIOD_MS);                //Since we run out PID every interupt cylce, we set the sample time (ms) to our PWM timer period 
  myPID2.SetOutputLimits(0, 650);

  //TODO: dont leave this hard-coded
  params.kP=30.0;
  params.kI=1;
  params.kD=0;
  myPID.SetTunings(params.kP, params.kI, params.kD);
  myPID2.SetTunings(params.kP, params.kI, params.kD);

  params.setpoint = 100.0;
  knob.write(100);
  status.encoder_pos = 100;
  
  //Detect DELL power supply
  //Check_DELL_PSU();
}



