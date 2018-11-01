/* Copyright (c) 2017 timothyjager
 * TI ADS1118 ADC configuration bits
 * MIT License. See LICENSE file for details. 
 */
#ifndef ads1118_h
#define ads1118_h


//ADS1118 Configuration 
//http://www.ti.com/lit/ds/symlink/ads1118.pdf
//Page 25-26

// |(1=Start Single-shot), (0=no effect)
// |
// | |Mux  (000 = P is AIN0 N is AIN1, 001 = P is AIN0 N is AIN3, 010 = P is AIN1 N is AIN3 , 011 = P is AIN2 N is AIN3
// | |     100 = P is AIN0 N is GND, 101 = P is AIN1 N is GND, 110 = P is AIN2 N is GND, 111 = P is AIN3 N is GND)
// | |    
// | |   |PGA Gain (000 = ±6.144V, 001 = ±4.096V, 010 = ±2.048V, 011 = ±1.024V, 100 = ±0.512V, 101 = ±0.256V)
// | |   |
// | |   |   |Mode (0 = Continuous conversion mode, 1 = Power-down and single-shot mode (default))
// | |   |   |
// | |   |   | |Data Rate (000 = 8SPS, 001 = 16SPS, 010 = 32SPS, 011 = 64SPS, 100 = 128SPS, 101 = 250SPS, 110 = 475SPS, 111 = 860SPS)
// | |   |   | |   
// | |   |   | |   |Sensor Mode (1=Internal Temp Mode, 0=ADC Mode)
// | |   |   | |   | 
// | |   |   | |   | |PullUp (0 = Pullup resistor disabled on DOUT/DRDY pin, 1 = Pullup resistor enabled on DOUT/DRDY pin (default))
// | |   |   | |   | |   
// | |   |   | |   | | |Update Config (01 = Valid data, update the Config register (default), 00,11,10 = do not update config register)
// | |   |   | |   | | |  
// | |   |   | |   | | |  |You must Always Write a 1 to this bit
// | |   |   | |   | | |  |
// 1 000 010 1 100 1 1 01 1  = 1000010110011011 = 0x859B (this sets up a single shot for the internal temperature sensor 128SPS)
// 1 100 101 1 111 0 1 01 1  = 1100101111101011 = 0xCBEB (this sets up a single shot for single ended AIN0, PGA 0.256V, 860SPS)
// 1 000 101 1 111 0 1 01 1  = 1000101111101011 = 0x8BEB (this sets up a single shot for differnetial AIN0 AIN1, PGA 0.256V, 860SPS)  
#define ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE 0x859B
#define ADS1118_SINGLE_SHOT_ADC 0x8BEB


//Converts the raw temperature value from the ADS1118 to an integer reading of degrees C.
//The incomming data arrives as a 16 bit value with the MSB first. 
//Since our actual data is 14 bit, we need to divide by 4 (or shift 2 bits to the right) to get the correct value
//Also per the datasheet page 18, we need to multiply by 0.03125 to convert this to Degrees F
//In order avoid floats, we will just divide by 32 which is the same as multiplying by 0.03125, but returns an integer
//overall we are dividing by 4 and then again by 32. i.e. dividing by 128
inline int16_t ADS1118_INT_TEMP_C(int16_t internal_temp_raw)
{
  return internal_temp_raw/128;
}


#endif
