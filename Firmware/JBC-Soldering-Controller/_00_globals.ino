/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Pin Mapping-------------------------
//Arduino Pro-Micro Pinout
//https://cdn.sparkfun.com/assets/9/c/3/c/4/523a1765757b7f5c6e8b4567.png
const int ENC_A            = 0;
const int ENC_B            = 1;
const int I2C_SDA          = 2;
const int I2C_SCL          = 3;
const int CS               = 4;
const int debug_pin_A      = 5;
const int DELL_PSU         = 6;
const int WS2812_DATA      = 7;
const int CRADLE_SENSOR    = 8;
const int LPINA            = 9;  //Heater PWM
const int VIN_SENSE        = 21;
const int CURRENT_SENSE    = 20;
const int debug_pin_B      = 19;
const int oled_reset       = 19; //The oled library requires a reset pin even though we doen thave one connected. use the debug pin to satisfy this requirement.
const int ENC_BUTTON       = 18;
const int SPI_SCLK         = 15;
const int SPI_MISO         = 14;
const int SPI_MOSI         = 16;
const int LPINB            = 10;  //Timer 1 Debug Pin
//-----------------------------------------------------


//----------------Function Prototypes------------------
bool SerialReceive(void);
void SendStatusPacket(void);
int multiMap2(int val, int* _in, int* _out, uint8_t size);
void Check_DELL_PSU(void);
//-----------------------------------------------------



//----------------Structure Definitions----------------
//EEProm parameter data (https://github.com/Chris--A/EEWrap)
//Use the xxx_e types rather than the standard types like uint8_t
struct NVOL {
  uint8_e a;  //TODO: add the actual non-volatile parameters to this struct.
  int16_e b;
  float_e c;
  uint8_e str[2][6];
};
NVOL nvol EEMEM;  //EEMEM tells the compiler that the object resides in the EEPROM

//Data structure sent from Arduino to host PC
union controller_packet_struct {
  struct {
    byte  start_of_packet; //always 0xBA. It was arbitrarily chosen.
    byte  automatic;
    byte  simulate_input;  //this allows us to override the actual input (temperature reading) using the tuning app
    float setpoint;
    float input;
    float output;
    float kP;
    float kI;
    float kD;
    float ITerm;
  } status;
  byte asBytes[sizeof(status)];
};


//Data structure sent from host PC
union host_packet_struct {
  struct {
    byte start_of_packet; //this should always be 0xAB. It was arbitrarily chosen
    byte automatic;       //0=manual Auto =1
    byte  simulate_input;  //this allows us to override the actual input (temperature reading) using the tuning app
    float setpoint;
    float input;
    float output;
    float kP;
    float kI;
    float kD;
  } param;
  byte asBytes[sizeof(param)];
};

//Instantiate these two structures one from the host to the controller and on
host_packet_struct host_packet;
controller_packet_struct controller_packet;
//-----------------------------------------------------



//Hard-coded calibration points.  TODO: make these not hard-coded
#define NUM_CAL_POINTS 4
//uint16_t microvolts [NUM_CAL_POINTS] = {0,10,20,30,40,50};
uint16_t adc_reading [NUM_CAL_POINTS] = {283, 584, 919, 1098};
uint16_t deg_c [NUM_CAL_POINTS] = {105, 200, 300, 345};

//x = multiMap2(raw, adc_reading, deg_c, NUM_CAL_POINTS);

//----------------Volatile Global Variables------------
volatile int16_t adc_value = 0;           //ADC value read by ADS1118
volatile int16_t temperature_value = 0;   //internal temp of ADS1118
volatile int16_t current_sense_raw;       //raw adc reading
volatile double Setpoint, Input, Output;  //PID input/output variables
double kP = 2, kI = 0, kD = 0;            //PID tuning values
//-----------------------------------------------------


//----------------Globals Objects----------------------
Adafruit_SSD1306 display(oled_reset);     //TODO: look into this reset pin. The LCD i'm using does not have a reset pin, just PWR,GND,SDA,SCL
Encoder knob(ENC_A, ENC_B);
DellPSU dell(DELL_PSU);                   //specify the desired Arduino pin number
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, WS2812_DATA, NEO_GRB + NEO_KHZ800);
PID myPID(&Input, &Output, &Setpoint, kP, kI, kD, P_ON_E, DIRECT); //TODO: map this properly to NVOL data storage
//-----------------------------------------------------









