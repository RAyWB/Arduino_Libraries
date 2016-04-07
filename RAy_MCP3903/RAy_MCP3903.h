/*####################################
#            MCP3903 Library         #
#          Robert Ayrenschmalz       #
#               22/ 8/2014           #
####################################*/


#ifndef RAy_MCP3903_H
#define RAy_MCP3903_H

#include <Arduino.h>



#define MCP3903_ADDR  (0x40)

#define MOD_REG3903 (0x06)
/*
bit 23:20 COMPn_CH5: Comparator Outputs from ADC Channel 5
bit 19:16 COMPn_CH4: Comparator Outputs from ADC Channel 4
bit 15:12 COMPn_CH3: Comparator Outputs from ADC Channel 3
bit 11:8 COMPn_CH2: Comparator Outputs from ADC Channel 2
bit 7:4 COMPn_CH1: Comparator Outputs from ADC Channel 1
bit 3:0 COMPn_CH0: Comparator Outputs from ADC Channel 0

power on default 0B 00110011 00110011 00110011
*/

#define PHASE_REG3903 (0x07)

/*
bit 23:16 PHASECn: CH4 relative to CH5 phase delay
bit 15:8 PHASEBn: CH2 relative to CH3 phase delay
bit 7:0 PHASEAn: CH0 relative to CH1 phase delay

power on default 0B 00000000 00000000 00000000
*/

#define GAIN_REG3903 (0x08)
/*
bit 23:21 Gain  CH5
bit 20    Boost CH5
bit 19:17 Gain  CH4
bit 16    Boost CH4
bit 15:13 Gain  CH3
bit 12    Boost CH3
bit 11:9  Gain  CH2
bit 8     Boost CH2
bit 7:5   Gain  CH1
bit 4     Boost CH1
bit 3:1   Gain  CH0
bit 0     Boost CH0



bit PGA_CHn: PGA Setting for Channel n

111 = Reserved (Gain = 1)
110 = Reserved (Gain = 1)
101 = Gain is 32
100 = Gain is 16
011 = Gain is 8
010 = Gain is 4
001 = Gain is 2
000 = Gain is 1

bit BOOST_CHn Current Scaling for high speed operation for channel n
1 = Channel has current x 2
0 = Channel has normal current

power on default 0B 00000000 00000000 00000000
*/


#define STATUSCOM_REG3903 (0x09)
/*
bit 23:22 READ[1:0]: Address Loop Setting
11 = Address counter incremented, cycle through entire register map
10 = Address counter loops on register TYPES (DEFAULT)
01 = Address counter loops on register GROUPS
00 = Address not incremented, continually read single register
bit 21 WMODE: Write Mode Bit (internal use only)
1 = Static addressing Write Mode
0 = Incremental addressing Write Mode (DEFAULT)
bit 20:15 WIDTH_CHn ADC Channels output data word width control
1 = 24-bit mode for the corresponding channel
0 = 16-bit mode for the corresponding channel (default)
bit 14 DR_LTY: Data Ready Latency Control for DRA, DRB, and DRC pins
1 = True “No Latency” Conversion, data ready pulses after 3 DRCLK periods (DEFAULT)
0 = Unsettled Data is available after every DRCLK period
bit 13 DR_HIZ: Data Ready Pin Inactive State Control for DRA, DRB, and DRC pins
1 = The Default state is a logic high when data is NOT ready
0 = The Default state is high impedance when data is NOT ready (DEFAULT)
bit 12 DR_LINK Data Ready Link Control
1 = Data Ready Link turned ON, all channels linked and data ready pulses from the most lagging ADC
are present on each DRn pin
0 = Data Ready Link tunred OFF (DEFAULT)
bit 11:10 DRC_MODE[1:0]
11 = Both Data Ready pulses from CH4 and CH5 are output on DRC pin.
10 = Data Ready pulses from CH5 are output on DRC pin. Data Ready pulses R from CH4 are not present
on the pin.
01 = Data Ready pulses from CH4 are output on DRC pin. Data Ready pulses from CH5 are not present
on the pin.
00 = Data Ready pulses from the lagging ADC channel between the two are output on DRC pin. The
lagging ADC channel depends on the phase register and on the OSR. (DEFAULT)
bit 9:8 DRB_MODE[1:0]
11 = Both Data Ready pulses from CH2 and CH3 are output on DRB pin.
10 = Data Ready pulses from CH3 are output on DRB pin. Data Ready pulses from CH2 are not present
on the pin.
01 = Data Ready pulses from CH2 are output on DRB pin. Data Ready pulses from CH3 are not present
on the pin.
00 = Data Ready pulses from the lagging ADC channel between the two are output on DRB pin. The
lagging ADC channel depends on the phase register and on the OSR. (DEFAULT)
bit 7:6 DRA_MODE[1:0]
11 = Both Data Ready pulses from CH0 and CH1 are output on DRA pin.
10 = Data Ready pulses from CH1 are output on DRA pin. Data Ready pulses from CH0 are not present
on the pin.
01 = Data Ready pulses from CH0 are output on DRA pin. Data Ready pulses from CH1 are not present
on the pin.
00 = Data Ready pulses from the lagging ADC channel between the two are output on DRA pin. The
lagging ADC channel depends on the phase register and on the OSR. (DEFAULT)
bit 5:0 DRSTATUS_CHn: Data Ready Status
1 = Data Not Ready (default)
0 = Data Ready

power on default 0B 10000000 10000000 10000000
*/

#define CONFIG_REG3903 (0x0A)

/*
bit 23:18 RESET_CHn: Reset mode setting for ADCs
1 = Reset mode for the corresponding ADC channel ON
0 = Reset mode for the corresponding ADC chnnel OFF (default)
bit 17:12 SHUTDOWN_CHn: Shutdown mode setting for ADCs
1 = Shutdown mode for the corresponding ADC channel ON
0 = Shutdown mode for the corresponding ADC channel OFF(default)
bit 11:6 DITHER_CHn: Control for dithering circuit for idle tones cancellation
1 = Dithering circuit for the corresponding ADC channel ON (default)
0 = Dithering circuit for the corresponding ADC channel OFF
bit 5:4 OSR[1:0] Oversampling Ratio for Delta Sigma A/D Conversion (ALL CHANNELS, fd / fS)
11 = 256
10 = 128
01 = 64 (default)
00 = 32
bit 3:2 PRESCALE[1:0] Internal Master Clock (AMCLK) Prescaler Value
11 = AMCLK = MCLK/ 8
10 = AMCLK = MCLK/ 4
01 = AMCLK = MCLK/ 2
00 = AMCLK = MCLK (DEFAULT)
bit 1 EXTVREF Internal Voltage Reference Shutdown Control
1 = Internal Voltage Reference Disabled
0 = Internal Voltage Reference Enabled (default)
bit 0 EXTCLK Clock Mode
1 = CLOCK Mode (Internal Oscillator Disabled - Lower Power)
0 = XT Mode - A crystal must be placed between OSC1/OSC2 (default)
power on default 0B 00000000 00001111 11010000
*/



class RAy_MCP3903
 {

public:
	RAy_MCP3903(void);
  	void begin(byte CsPin);
	unsigned long read24BitRegister(byte reg);
     void write24BitRegister(byte reg, unsigned long data);
     double readAnalog(byte channel);
     double readAnalog(byte channel, byte resolution);
     double readAnalogAVG(byte channel,byte nos);
     double readAnalogAVG(byte channel, byte resolution,byte nos);
     void setGain(byte gain); 
     void setGain(byte channel, byte gain); 
     void setGain(byte channel, byte gain, byte boost);
     byte getGain(byte channel);
     void setBoost(byte boost);
     byte getBoost(byte channel);
     void setOSR(word OSR);
     word getOSR();

private:
};
#endif
