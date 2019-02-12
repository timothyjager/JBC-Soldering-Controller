/*
 * ADS1118.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: a0221660@ti.com
 *
 *      Copyright (c) 2014, Evan Wakefield
 *		All rights reserved.
 *
 *		Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *		1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 *		2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *		EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *		OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *		SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *		SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 *		OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *		HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *		(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *		EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "ADS1118.h"
//#include "Energia.h"
#include <SPI.h>


volatile bool ch_flag = 0;				// global flag

/**************************************************************
 * function: WriteSPI(unsigned int config, int mode)
 * introduction: write SPI to transmit the configuration parameter for ADS11118, and receive the convertion result.
 * parameters: config: configuration parameter of ADS11118's register, mode (0/1): internal temperature sensor, far-end temperature
 * return value: ADC result
 */
int ADS1118::writeSPI(unsigned int config, int mode){
	int msb;
	unsigned int temp;
	char dummy;

	SPI.setDataMode(1);
	temp = config;
	if (mode == 1)
		temp = config | 0x8000;

	msb = SPI.transfer(temp >> 8);
	msb = (msb << 8) | SPI.transfer(temp & 0xFF);

	dummy = SPI.transfer(temp >> 8);
	dummy = SPI.transfer(temp & 0xFF);

	return msb;
}

/******************************************************************************
 * function: begin ()
 * introduction: configure and start conversion. starts SPI.
 * parameters:
 * defaulted to mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor.
 * return value:
*******************************************************************************/

void ADS1118::begin(){
	unsigned int temp;

	SPI.begin();
	pinMode(8,OUTPUT);

	if(ch_flag)
	{
		temp = ADSCON_CH1 + ADS1118_TS;
	}
	else
	{
		temp = ADSCON_CH0 + ADS1118_TS;// temperature sensor mode.DR=8sps, PULLUP on DOUT
	}

	ADS_CS_LOW;

	writeSPI(temp, 1);

	ADS_CS_HIGH;
}

/******************************************************************************
 * function: begin (unsigned int mode)
 * introduction: configure and start conversion. starts SPI
 * parameters:
 * mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1, ADS1118 is set to convert the voltage of thermocouple.
 * return value:
*******************************************************************************/

void ADS1118::begin(unsigned int mode){
	unsigned int temp;

	SPI.begin();
	pinMode(8,OUTPUT);

	if(ch_flag)
	{
		if(mode == 1)
			temp = ADSCON_CH1;
		else
			temp = ADSCON_CH1 + ADS1118_TS;
	}
	else
	{
		if (mode==1)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			temp = ADSCON_CH0;
		else
			temp = ADSCON_CH0 + ADS1118_TS;// temperature sensor mode.DR=8sps, PULLUP on DOUT
	}

	ADS_CS_LOW;

	writeSPI(temp, 1);

	ADS_CS_HIGH;
}

/******************************************************************************
 * function: ADSread(unsigned int mode)
 * introduction: read the ADC result and tart a new conversion.
 * parameters:
 * mode = 0, ADS1118 is set to convert the voltage of integrated temperature sensor.
 * mode = 1, ADS1118 is set to convert the voltage of thermocouple.
 * return value:result of last conversion
 */

int ADS1118::ADSread(unsigned int mode){
	unsigned int temp;
	int result;

	/*if(ch_flag)
	{
		if (mode==1)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			temp = ADSCON_CH1;
		else
			temp = ADSCON_CH1 + ADS1118_TS;// temperature sensor mode.DR=8sps, PULLUP on DOUT
	}
	else
	{
		if (mode==1)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			temp = ADSCON_CH0;
		else
			temp = ADSCON_CH0 + ADS1118_TS;// temperature sensor mode.DR=8sps, PULLUP on DOUT
	}*/
	if (mode==1)		// Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
			temp = ADS1118_SINGLE_SHOT_ADC;
		else
			//temp = ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE;// temperature sensor mode.DR=8sps, PULLUP on DOUT
			temp = ADS1118_SINGLE_SHOT_ADC + ADS1118_TS;// temperature sensor mode.DR=8sps, PULLUP on DOUT
	ADS_CS_LOW;
	result = writeSPI(temp,1);
	ADS_CS_HIGH;

	return result;
}

/******************************************************************************
 * function: localCompensation(int local_code)
 * introduction:
 * this function transform internal temperature sensor code to compensation code, which is added to thermocouple code.
 * local_data is at the first 14bits of the 16bits data register.
 * So we let the result data to be divided by 4 to replace right shifting 2 bits
 * for internal temperature sensor, 32 LSBs is equal to 1 Celsius Degree.
 * We use local_code/4 to transform local data to n* 1/32 degree.
 * the local temperature is transformed to compensation code for thermocouple directly.
 *                                                   (Tin -T[n-1])
 * comp codes = Code[n-1] + (Code[n] - Code[n-1])* {---------------}
 *													(T[n] - T[n-1])
 * for example: 5-10 degree the equation is as below
 *
 * tmp = (0x001A*(local_temp - 5))/5 + 0x0019;
 *
 * 0x0019 is the 'Code[n-1]' for 5 Degrees; 	0x001A = (Code[n] - Code[n-1])
 * (local_temp - 5) is (Tin -T[n-1]);			denominator '5' is (T[n] - T[n-1])
 *
 * the compensation range of local temperature is 0-125.
 * parameters: local_code, internal sensor result
 * return value: compensation codes
 ******************************************************************************/

int ADS1118::localCompensation(int local_code){
	float tmp,local_temp;
	int comp;
	local_code = local_code / 4;
	local_temp = (float)local_code / 32;	//

	if (local_temp >=0 && local_temp <=5)		//0~5
	{
		tmp = (0x0019*local_temp)/5;
		comp = tmp;
	}
	else if (local_temp> 5 && local_temp <=10)	//5~10
	{
		tmp = (0x001A*(local_temp - 5))/5 + 0x0019 ;
		comp = tmp;
	}
	else if (local_temp> 10 && local_temp <=20)	//10~20
	{
		tmp = (0x0033*(local_temp - 10))/10 + 0x0033 ;
		comp = tmp;
	}
	else if (local_temp> 20 && local_temp <=30)	//20~30
	{
		tmp = (0x0034*(local_temp - 20))/10 + 0x0066 ;
		comp = tmp;
	}
	else if (local_temp> 30 && local_temp <=40)	//30~40
	{
		tmp = (0x0034*(local_temp - 30))/10 + 0x009A ;
		comp = tmp;
	}
	else if (local_temp> 40 && local_temp <=50)	//40~50
	{
		tmp = (0x0035*(local_temp - 40))/10 + 0x00CE;
		comp = tmp;
	}

	else if (local_temp> 50 && local_temp <=60)	//50~60
	{
		tmp = (0x0035*(local_temp - 50))/10 + 0x0103;
		comp = tmp;
	}
	else if (local_temp> 60 && local_temp <=80)	//60~80
	{
		tmp = (0x006A*(local_temp - 60))/20 + 0x0138;
		comp = tmp;
	}
	else if (local_temp> 80 && local_temp <=125)//80~125
	{
		tmp = (0x00EE*(local_temp - 80))/45 + 0x01A2;
		comp = tmp;
	}
	else
	{
		comp = 0;
	}
	return comp;
}

/******************************************************************************
 * function: ADCcode2temp(int code)
 * introduction:
 * this function is used to convert ADC result codes to temperature.
 * converted temperature range is 0 to 500 Celsius degree
 * Omega Engineering Inc. Type K thermocouple is used, seebeck coefficient is about 40uV/Degree from 0 to 1000 degree.
 * ADC input range is +/-256mV. 16bits. so 1 LSB = 7.8125uV. the coefficient of code to temperature is 1 degree = 40/7.8125 LSBs.
 * Because of nonlinearity of thermocouple. Different coefficients are used in different temperature ranges.
 * the voltage codes is transformed to temperature as below equation
 * 							      (Codes - Code[n-1])
 * T = T[n-1] + (T[n]-T[n-1]) * {---------------------}
 * 							     (Code[n] - Code[n-1])
 *
 * parameters: code
 * return value: far-end temperature
*******************************************************************************/

int ADS1118::ADCcode2temp(int code){
	float temp;
	int t;

	temp = (float)code;

	if (code > 0xFF6C && code <=0xFFB5)			//-30~-15
	{
		temp = (float)(15 * (temp - 0xFF6C)) / 0x0049 - 30.0f;
	}
	else if (code > 0xFFB5 && code <=0xFFFF)	//-15~0
	{
		temp = (float)(15 * (temp - 0xFFB5)) / 0x004B - 15.0f;
	}
	else if (code >=0 && code <=0x0019)			//0~5
	{
		temp = (float)(5 * (temp - 0)) / 0x0019;
	}
	else if (code >0x0019 && code <=0x0033)		//5~10
	{
		temp = (float)(5 * (temp - 0x0019)) / 0x001A + 5.0f;
	}
	else if (code >0x0033 && code <=0x0066)		//10~20
	{
		temp = (float)(10 * (temp - 0x0033)) / 0x0033 + 10.0f;
	}
	else if (code > 0x0066 && code <= 0x009A)	//20~30
	{
		temp = (float)(10 * (temp - 0x0066)) / 0x0034 + 20.0f;
	}
	else if (code > 0x009A && code <= 0x00CE)	//30~40
	{
		temp = (float)(10 * (temp - 0x009A)) / 0x0034 + 30.0f;
	}
	else if ( code > 0x00CE && code <= 0x0103)	//40~50
	{
		temp = (float)(10 * (temp - 0x00CE)) / 0x0035 + 40.0f;
	}
	else if ( code > 0x0103 && code <= 0x0138)	//50~60
	{
		temp = (float)(10 * (temp - 0x0103)) / 0x0035 + 50.0f;
	}
	else if (code > 0x0138 && code <=0x01A2)	//60~80
	{
		temp = (float)(20 * (temp - 0x0138)) / 0x006A + 60.0f;
	}
	else if (code > 0x01A2 && code <= 0x020C)	//80~100
	{
		temp = (float)((temp - 0x01A2) * 20)/ 0x06A + 80.0f;
	}
	else if (code > 0x020C && code <= 0x02DE)	//100~140
	{
		temp = (float)((temp - 0x020C) * 40)/ 0x0D2 + 100.0f;
	}
	else if (code > 0x02DE && code <= 0x03AC)	//140~180
	{
		temp = (float)((temp - 0x02DE) * 40)/ 0x00CE + 140.0f;
	}
	else if (code > 0x03AC && code <= 0x0478)	//180~220
	{
		temp = (float)((temp - 0x03AB) * 40) / 0x00CD + 180.0f;
	}
	else if (code > 0x0478 && code <= 0x0548)	//220~260
	{
		temp = (float)((temp - 0x0478) * 40) / 0x00D0 + 220.0f;
	}
	else if (code > 0x0548 && code <= 0x061B)	//260~300
	{
		temp = (float)((temp - 0x0548) * 40) / 0x00D3 + 260.0f;
	}
	else if (code > 0x061B && code <= 0x06F2)	//300~340
	{
		temp = (float)((temp - 0x061B) * 40) /  0x00D7 + 300.0f;
	}
	else if (code > 0x06F2 && code <= 0x07C7)	//340~400
	{
		temp =(float) ((temp - 0x06F2) *  40)  / 0x00D5 + 340.0f;
	}
	else if (code > 0x07C7 && code <= 0x089F)	//380~420
	{
		temp =(float) ((temp - 0x07C7) * 40)  / 0x00D8 + 380.0f;
	}

	else if (code > 0x089F && code <= 0x0978)	//420~460
	{
		temp = (float)((temp - 0x089F) * 40) / 0x00D9 + 420.0f;
	}
	else if (code > 0x0978 && code <=0x0A52)	//460~500
	{
		temp =(float)((temp - 0x0978) * 40) / 0x00DA + 460.0f;
	}
	else
	{
		temp = 0xA5A5;
	}

	t = (uint16_t)(10*temp);

	return t;
}

/**************************************************************
 * function: readCelcius()
 * introduction: returns the celcius temperature value as a double
 * return value: converted temperature value
 */

double ADS1118::readCelcius(){
	signed int local_data, far_data;
	signed int temp;
	double tempConv;

	local_data = ADSread(1);
	//Serial.println(local_data);
	delay(50);
	far_data = ADSread(0);
	delay(50);
	//Serial.println(far_data);
	temp = far_data + localCompensation(local_data);
	delay(50);
	//Serial.println(temp);
	delay(50);
	temp = ADCcode2temp(temp);
	tempConv = double(temp)/10;
	return tempConv;
}

/**************************************************************
 * function: readFarenheit()
 * introduction: returns the celcius temperature value as a double
 * return value: converted temperature value
 */

double ADS1118::readFarenheit(){
	signed int local_data, far_data;
	signed int temp;
	double tempConv;

	local_data = ADSread(1);
	//Serial.println(local_data);
	delay(50);
	far_data = ADSread(0);
	delay(50);
	//Serial.println(far_data);
	temp = far_data + localCompensation(local_data);
	delay(50);
	//Serial.println(temp);
	delay(50);
	temp = ADCcode2temp(temp);
	delay(50);
	temp = temp*9/5+320;
	delay(10);
	tempConv = double(temp)/10;
	return tempConv;
}

