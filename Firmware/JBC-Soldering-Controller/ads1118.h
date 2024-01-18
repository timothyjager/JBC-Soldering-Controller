/*
 * 		ADS1118.h
 *
*  		Created on: Nov 6, 2014
 *      Author: a0221660@ti.com
 *
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

#ifndef ADS1118_H_
#define ADS1118_H_

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile unsigned int  flag;		//global flag.

#define ADS1118_OS             (0x8000)         //
#define ADS1118_CH0            (0x0000)         //
#define ADS1118_CH1            (0x3000)         //
#define ADS1118_GAIN0          (0x0000)         //
#define ADS1118_GAIN1          (0x0200)         //
#define ADS1118_GAIN2          (0x0400)         //
#define ADS1118_GAIN4          (0x0600)         //
#define ADS1118_GAIN8          (0x0800)         //
#define ADS1118_GAIN16         (0x0A00)         //
#define ADS1118_PWRDOWN        (0x0100)         //

#define ADS1118_RATE8SPS       (0x0000)         //
#define ADS1118_RATE16SPS      (0x0020)         //
#define ADS1118_RATE32SPS      (0x0040)         //
#define ADS1118_RATE64SPS      (0x0060)         //
#define ADS1118_RATE128SPS     (0x0080)         //
#define ADS1118_RATE250SPS     (0x00A0)         //
#define ADS1118_RATE475SPS     (0x00C0)         //
#define ADS1118_RATE860SPS     (0x00E0)         //

#define ADS1118_TS			   (0x0010)         //
#define ADS1118_PULLUP     	   (0x0008)         //
#define ADS1118_NOP     	   (0x0002)         //
#define ADS1118_CNVRDY     	   (0x0001)         //
//Set the configuration to AIN0/AIN1, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH0		(0x8B8A)
//Set the configuration to AIN2/AIN3, FS=+/-0.256, SS, DR=128sps, PULLUP on DOUT
#define ADSCON_CH1		(0xBB8A)

#define ADS1118_SINGLE_SHOT_INTERNAL_TEMPERATURE 0x859B
#define ADS1118_SINGLE_SHOT_ADC 0x8BEB
#define ADS1118_SINGLE_SHOT_ADC_2 0xBBEB

#define ADS_CS_LOW      digitalWrite(8, LOW)
#define ADS_CS_HIGH     digitalWrite(8, HIGH)

class ADS1118
{
	public:
		void begin();
		void begin(unsigned int);
		int writeSPI(unsigned int, int);
		int readConfig(unsigned int, int);
		int ADSread(unsigned int);
		int ADCcode2temp(int);
		int localCompensation(int);
		double readCelcius();
		double readFarenheit();

	private:
};

#ifdef __cplusplus
}
#endif

#endif /* ADS1118_H_ */
