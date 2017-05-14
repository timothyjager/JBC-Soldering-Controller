#ifndef ads1118_h
#define ads1118_h


//ADS1118 Configuration 
//http://www.ti.com/lit/ds/symlink/ads1118.pdf
//Page 25-26

// |(1=Start Single-shot), (0=no effect)
// |
// | |Mux
// | |    
// | |   |PGA Gain
// | |   |
// | |   |   |OneShot
// | |   |   |
// | |   |   | |Data Rate SPS
// | |   |   | |   
// | |   |   | |   |(1=Internal Temp Mode, 0=ADC Mode)
// | |   |   | |   | 
// | |   |   | |   | |PullUp Enabled
// | |   |   | |   | |   
// | |   |   | |   | | |Valid Data
// | |   |   | |   | | |  
// | |   |   | |   | | |  |Always Write a 1
// | |   |   | |   | | |  |
// 1 000 010 1 100 1 1 01 1  = 1000010110011011 = 0x859B (this sets up a single shot for the internal temperature sensor 128SPS)
// 1 100 101 1 111 0 1 01 1  = 1100101111101011 = 0xCBEB (this sets up a single shot for single ended AIN0, PGA 0.256V, 860SPS)
// 1 000 101 1 111 0 1 01 1  = 1000101111101011 = 0x8BEB (this sets up a single shot for differnetial AIN0 AIN1, PGA 0.256V, 860SPS)  
#define ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE 0x859B
#define ADS1118_SINGLE_SHOT_ADC 0x8BEB

#endif
