/* Copyright (c) 2017 timothyjager
   JBC-Soldering-Controller
   MIT License. See LICENSE file for details.
*/

//----------------Pin Mapping-------------------------
//Arduino Pro-Micro Pinout
//https://cdn.sparkfun.com/assets/9/c/3/c/4/523a1765757b7f5c6e8b4567.png
const int ENC_A            = 7;
const int ENC_B            = 6;
const int I2C_SDA          = 2;
const int I2C_SCL          = 3;
const int CS               = 4;
//const int debug_pin_A      = 8;
//const int DELL_PSU         = 6; //not used!
const int WS2812_DATA      = 5;
const int CRADLE_SENSOR    = 8;
const int LPINA            = 9;  //Heater PWM
const int VIN_SENSE        = 21;
const int CURRENT_SENSE    = 20;
//const int debug_pin_B      = 8;
//const int oled_reset       = 19; //The oled library requires a reset pin even though we doen thave one connected. use the debug pin to satisfy this requirement.
const int ENC_BUTTON       = 18;
const int SPI_SCLK         = 15;
const int SPI_MISO         = 14;
const int SPI_MOSI         = 16;
const int LPINB            = 10;  //Timer 1 Debug Pin
//-----------------------------------------------------

//----------------Settings-----------------------------
// You should change these settings depending on your situation.
const bool disable_simultaneous_output = true;// If true, this will not allow both outputs to be on at the same time.

//-----------------------------------------------------

//----------------Function Prototypes------------------
bool SerialReceive(void);
void SendStatusPacket(void);
int  multiMap2(int val, int* _in, int* _out, uint8_t size);
void Check_DELL_PSU(void);
void PulsePin(int pin);
void updateLEDStatus(void);
void ProcessSerialComm(void);
void updateDisplay(bool update_now);
//-----------------------------------------------------

const uint16_t PixelCount = 4; // this example assumes 4 pixels, making it smaller will cause a failure
#define colorSaturation 128
#define coldTemp 80
bool cradle_present = false;
bool iron_active =  false;
int upper_set_limit = 450;

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

//System Parameters Data Structure
typedef struct {
  byte  pid_mode;          //PID mode - Automatic=1, Manual=0
  byte  simulate_input;     //this allows us to override the actual input (temperature reading) using the tuning app
  int16_t idle_temp_c;
  int16_t output_override;
  float setpoint;
  float kP;
  float kI;
  float kD;
  float simulated_input;
} system_parameters_struct;

//Status Variables Struct - hold global status values
typedef struct {
  byte gpio_port_b;                //Port b of GPIO
  byte gpio_port_c;                //Port C of GPIO
  byte gpio_port_d;                //Port d of GPIO
  byte gpio_port_e;                //Port e of GPIO
  int16_t encoder_pos;             //Enocder Position
  int16_t adapter_voltage_mv;      //Input power adapter voltage in millivolts
  int16_t adc_counts;              //ADC value read by ADS1118
  int16_t adc_ic_temp_counts;      //internal temp of ADS1118
  int16_t current_sense_mv;        //current sense in milliamps
  double pid_setpoint;             //setpoint of the PID loop
  double tip_temperature_c;        //input value of the PID loop
  double tip_temperature_c2;
  double pid_output;              //computed output value of the PID loop
  double pid_output2;
} status_struct;




//Data structure sent from Arduino to host PC
union controller_packet_struct {
  struct {
    volatile status_struct status;            //Enture status structure
    volatile system_parameters_struct params;
  } payload;
  byte asBytes[sizeof(payload)];
};


//Data structure sent from host PC
union host_packet_struct {
  struct {
    byte start_of_packet; //this should always be 0xAB. It was arbitrarily chosen
    system_parameters_struct params;
  } payload;
  byte asBytes[sizeof(payload)];
};
//-----------------------------------------------------



//----------------Standard Global Variables----------------
host_packet_struct host_packet;
controller_packet_struct controller_packet;
volatile status_struct status;
volatile system_parameters_struct params;

//Hard-coded calibration points.  TODO: make these not hard-coded
#define NUM_CAL_POINTS 4
uint16_t adc_reading [NUM_CAL_POINTS] = {283, 584, 919, 1098};
uint16_t deg_c [NUM_CAL_POINTS] = {105, 200, 300, 345};

//x = multiMap2(raw, adc_reading, deg_c, NUM_CAL_POINTS);

//double kP = 2, kI = 0, kD = 0;            //PID tuning values
//-----------------------------------------------------



//----------------Volatile Global Variables------------
//volatile int16_t adc_value = 0;           //ADC value read by ADS1118
//volatile int16_t temperature_value = 0;   //internal temp of ADS1118
//volatile int16_t current_sense_raw;       //raw adc reading
//volatile double Setpoint, Input, Output;  //PID input/output variables
//-----------------------------------------------------


//----------------Globals Objects----------------------
ADS1118 ADS;
Adafruit_SSD1306 display(-1);     //TODO: look into this reset pin. The LCD i'm using does not have a reset pin, just PWR,GND,SDA,SCL
Encoder knob(ENC_A, ENC_B);               //Setup the encoder object
//DellPSU dell(DELL_PSU);                   //This object reads data from a DELL power adapter using 1-wire protocol
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, WS2812_DATA, NEO_GRB + NEO_KHZ800);
NeoPixelBus<NeoRgbwFeature, Neo800KbpsMethod> pixels(PixelCount, WS2812_DATA);
PID myPID(&status.tip_temperature_c, &status.pid_output, &status.pid_setpoint, params.kP, params.kI, params.kD, P_ON_E, DIRECT); //TODO: map this properly to NVOL data storage
PID myPID2(&status.tip_temperature_c2, &status.pid_output2, &status.pid_setpoint, params.kP, params.kI, params.kD, P_ON_E, DIRECT); //TODO: map this properly to NVOL data storage
//-----------------------------------------------------









