/*####################################
#            MCP3911 Library         #
#          Robert Ayrenschmalz       #
#               22/ 8/2014           #
####################################*/


#ifndef RAy_MCP3911_H
#define RAy_MCP3911_H

#include <Arduino.h>



#define MCP3911_ADDR  (0x00)

#define MOD_REG3911 (0x06) 

/* 8 bit register
bit 7-4 COMPn_CH1: Comparator Outputs from ADC Channel 1
bit 3-0 COMPn_CH0: Comparator Outputs from ADC Channel 0
*/

#define PHASE_REG3911 (0x07)

/* 16 bit register
bit 15-12 Unimplemented: Read as ‘0’
bit 11-0 CH0 relative to CH1 Phase Delay:
PHASE<11:0>: CH0 Relative to CH1 Phase Delay bits.
Delay = PHASE Register’s two’s complement code/DMCLK (Default PHASE = 0).
*/

#define GAIN_REG3911 (0x09)

/* 8 bit register
bit 7-6 BOOST<1:0>: Bias Current Selection
11 = Both channels have current x 2
10 = Both channels have current x 1(DEFAULT)
01 = Both channels have current x 0.66
00 = Both channels have current x 0.5
bit 5-3 PGA_CH1<2:0>: PGA Setting for Channel 1
111 = Reserved (Gain = 1)
110 = Reserved (Gain = 1)
101 = Gain is 32
100 = Gain is 16
011 = Gain is 8
010 = Gain is 4
001 = Gain is 2
000 = Gain is 1 (DEFAULT)
bit 2-0 PGA_CH0<2:0>: PGA Setting for Channel 0
111 = Reserved (Gain = 1)
110 = Reserved (Gain = 1)
101 = Gain is 32
100 = Gain is 16
011 = Gain is 8
010 = Gain is 4
001 = Gain is 2
000 = Gain is 1 (DEFAULT)
*/

#define STATUSCOM_REG3911 (0x0A)

/* 16 bit register
bit 15-14 MODOUT<1:0>: Modulator Output Setting for MDAT Pins
11 = Both CH0 and CH1 modulator outputs are present on MDAT1 and MDAT0 pins, both SINC filters
are off and no data ready pulse is present.
10 = CH1 ADC Modulator output present on MDAT1 pin, SINC filter on Channel 1 is off and data ready
pulse from Channel 1 is not present on DR pin.
01 = CH0 ADC Modulator output present on MDAT0 pin, SINC filter on Channel 0 is off and data ready
pulse from Channel 0 is not present on DR pin.
00 = No Modulator output is enabled, SINC filters are on and data ready pulses are present on DR pin
for both channels (DEFAULT)
bit 13 Unimplemented: Read as ‘0’.
bit 12 DR_HIZ: Data Ready Pin Inactive State Control
1 = The DR pin state is a logic high when data is NOT ready
0 = The DR pin state is high-impedance when data is NOT ready (DEFAULT)
bit 11-10 DRMODE<1:0>: Data Ready Pin (DR) mode configuration bits
11 = Both Data Ready pulses from CH0 and CH1 are output on DR pin.
10 = Data Ready pulses from CH1 ADC are output on DR pin. Data ready pulses from CH0 are not
present on the DR pin.
01 = Data Ready pulses from CH0 ADC are output on DR pin. Data ready pulses from CH1 are not
present on the DR pin.
00 = Data Ready pulses from the lagging ADC between the two are output on DR pin. The lagging
ADC depends on the PHASE register and on the OSR (DEFAULT).
bit 9-8 DRSTATUS<1:0>: Data Ready Status
11 = ADC Channel 1 and Channel 0 data not ready (DEFAULT)
10 = ADC Channel 1 data not ready, ADC Channel 0 data ready
01 = ADC Channel 0 data not ready, ADC Channel 1 data ready
00 = ADC Channel 1 and Channel 0 data ready
bit 7-6 READ<1:0>: Address Loop Setting
11 = Address counter incremented, cycle through entire register set
10 = Address counter loops on register types (DEFAULT)
01 = Address counter loops on register groups
00 = Address not incremented, continually read single register
bit 5 WRITE: Address Loop Setting for Write mode
1 = Address counter loops on entire register map (DEFAULT)
0 = Address not incremented, continually write same single register
bit 4-3 WIDTH<1:0> ADC Channel output data word width
11 = Both channels are in 24-bit mode(DEFAULT)
10 = Channel1 in 16-bit mode, Channel0 in 24-bit mode
01 = Channel1 in 16-bit mode, Channel0 in 24-bit mode
00 = Both channels are in 16-bit mode
bit 2 EN_OFFCAL Enables or disables the 24-bit digital offset calibration on both channels
1 = Enabled; this mode does not add any group delay
0 = Disabled (DEFAULT)
bit 1 EN_GAINCAL Enables or disables the 24-bit digital offset calibration on both channels
1 = Enabled; this mode adds a group delay on both channels of 24 DMCLK periods. All data ready
pulses are delayed by 24 clock periods compared to the mode with EN_GAINCAL = 0
0 = Disabled (DEFAULT)
bit 0 Unimplemented: Read as ‘0’
*/

#define CONFIG_REG3911 (0x0C)

/* 16 bit register
bit 15-14 PRE<1:0>: Analog Master Clock (AMCLK) Prescaler Value
11 = AMCLK = MCLK/8
10 = AMCLK = MCLK/4
01 = AMCLK = MCLK/2
00 = AMCLK = MCLK (DEFAULT)
bit 13-11 OSR<2:0>: Oversampling Ratio for Delta-Sigma A/D Conversion (ALL CHANNELS, fd/fS)
111 = 4096 (fd = 244 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
110 = 2048 (fd = 488 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
101 = 1024 (fd = 976 sps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
100 = 512 (fd = 1.953 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
011 = 256 (fd = 3.90625 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz) (DEFAULT)
010 = 128 (fd = 7.8125 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
001 = 64 (fd = 15.625 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
000 = 32 (fd = 31.25 ksps for MCLK = 4 MHz, fs = AMCLK = 1 MHz)
bit 10-9 DITHER<1:0>: Control for dithering circuit for idle tones cancellation and improved THD
11 = Dithering ON, both channels, Strength = Maximum(MCP3901 Equivalent) - (DEFAULT)
10 = Dithering ON, both channels, Strength = Medium
01 = Dithering ON, both channels, Strength = Minimum
00 = Dithering turned OFF
bit 8 AZ_FREQ: Auto-zero frequency setting
1 = Auto-zeroing algorithm running at higher speed
0 = Auto-zeroing algorithm running at lower speed (Default)
bit 7-6 RESET<1:0>: Reset mode setting for ADCs
11 = Both CH0 and CH1 ADC are in reset mode
10 = CH1 ADC in Reset mode
01 = CH0 ADC in Reset mode
00 = Neither ADC in Reset mode (default)
bit 5-4 SHUTDOWN<1:0>: Shutdown mode setting for ADCs
11 = Both CH0 and CH1 ADC in Shutdown
10 = CH1 ADC in Shutdown
01 = CH0 ADC in Shutdown
00 = Neither Channel in Shutdown (default)
bit 3 Not implemented: Read as ‘0’.
bit 2 VREFEXT Internal Voltage Reference Shutdown Control
1 = Internal Voltage Reference Disabled
0 = Internal Voltage Reference Enabled (Default)
bit 1 CLKEXT Internal Clock selection bits
1 = External clock drive by MCU on OSC1 pin (crystal oscillator disabled, no internal power
consumption) (Default)
0 = Crystal oscillator is enabled. A crystal must be placed between OSC1 and OSC2 pins.
bit 0 Not implemented: Read as ‘0’.
*/

#define OFFCAL_0_REG3911 (0x0E)

/*24bit register
bit 23-0 Digital Offset calibration value for the corresponding Channel CHn. This register simply is added to
the output code of the channel bit-by-bit. This register is 24-bit two's complement MSB first coding.
CHn Output Code = OFFCAL_CHn + ADC CHn Output Code. This register is a “Don't Care” if
EN_OFFCAL = 0 (Offset calibration disabled), but its value is not cleared by the EN_OFFCAL bit.
*/

#define GAINCAL_0_REG3911 (0x11)

/*24bit register
bit 23-0 Digital gain error calibration value for the corresponding Channel CHn. This register is 24-bit signed
MSB first coding with a range of -1x to +0.9999999x (from 0x80000 to 0x7FFFFF). The gain calibration
adds 1x to this register and multiplies it to the output code of the channel bit by bit, after the offset
calibration. Thus, the range of the gain calibration is from 0x to 1.9999999x (from 0x80000 to
0x7FFFFF). The LSB corresponds to a 2-23 increment in the multiplier.
CHn Output Code = (GAINCAL_CHn+1) x ADC CHn Output Code. This register is a “Don't Care” if
EN_GAINCAL = 0 (Offset calibration disabled), but its value is not cleared by the EN_GAINCAL bit.
*/


#define OFFCAL_1_REG3911 (0x14)

/*24bit register
bit 23-0 Digital Offset calibration value for the corresponding Channel CHn. This register simply is added to
the output code of the channel bit-by-bit. This register is 24-bit two's complement MSB first coding.
CHn Output Code = OFFCAL_CHn + ADC CHn Output Code. This register is a “Don't Care” if
EN_OFFCAL = 0 (Offset calibration disabled), but its value is not cleared by the EN_OFFCAL bit.
*/

#define GAINCAL_1_REG3911 (0x17)

/*24bit register
bit 23-0 Digital gain error calibration value for the corresponding Channel CHn. This register is 24-bit signed
MSB first coding with a range of -1x to +0.9999999x (from 0x80000 to 0x7FFFFF). The gain calibration
adds 1x to this register and multiplies it to the output code of the channel bit by bit, after the offset
calibration. Thus, the range of the gain calibration is from 0x to 1.9999999x (from 0x80000 to
0x7FFFFF). The LSB corresponds to a 2-23 increment in the multiplier.
CHn Output Code = (GAINCAL_CHn+1) x ADC CHn Output Code. This register is a “Don't Care” if
EN_GAINCAL = 0 (Offset calibration disabled), but its value is not cleared by the EN_GAINCAL bit.
*/


#define VREFCAL_REG3911 (0x1A)

/*8bit register
bit 7-0 Internal Voltage Temperature coefficient register value
*/


class RAy_MCP3911
 {

public:
	RAy_MCP3911(void);
  	void begin(byte CsPin);
	byte read8BitRegister(byte reg);
     void write8BitRegister(byte reg, byte data);
     word read16BitRegister(byte reg);
     void write16BitRegister(byte reg, word data);
     unsigned long read24BitRegister(byte reg);
     void write24BitRegister(byte reg, unsigned long data);
     double readAnalog(byte channel);
     double readAnalog(byte channel, byte resolution);
     double readAnalogAVG(byte channel,byte nos);
     double readAnalogAVG(byte channel, byte resolution,byte nos);
     void setGain(byte gain); 
     void setGain(byte channel, byte gain); 
     void setGain(byte channel, byte gain, float boost);
     byte getGain(byte channel);
     void setBoost(float boost);
     float getBoost();
     void setOSR(word OSR);
     word getOSR();

private:
};
#endif
