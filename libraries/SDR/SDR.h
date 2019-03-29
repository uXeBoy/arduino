
/*
  03/03/2019

  Menlopark Innovation LLC

  Software Defined Radio (SDR) created from FM RDS core.

*/

/*  Original code from:
    
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    
    See https://github.com/ChristopheJacquet/PiFmRds

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    Modification: by EMARD
    deleted everything except RDS bit generator
    converted to c++ for arduino, renamed functions
    
*/

#ifndef _SDR_h
#define _SDR_h

#include <stdint.h>

//
// SDR decodes from 0xFFFFFC00 - 0xFFFFFCFF which is a 256 byte
// range providing 64 32 bit registers that provide the
// primary interface to a Software Defined Radio subsystem
// consisting of multiple internal processing blocks.
//
// Note: The AM and FM synthesizer blocks share control signals
// for the frequency registers, PCM, and I+Q signals so only
// one at a time should be enabled.
//
// They have separate antennas to allow different external
// filters, and to minimize the logic paths for the radio
// frequency signal output from the FPGA.
//
// Synthesizer, filter, and modulator blocks will also be
// available as the project progresses to support using
// the SoC SDR block for radio communications.
//
// WARNING: Communication in the below bands must be according
// to your license and regulations of the country you use it.
// For example the long wave bands are still used for aircraft
// navigation/landing so operations there must be within the
// bands and power limits specified by your country and amateur
// radio license. You are also responsible for applying the proper
// external analog filtering to the direct digital synthesis
// signal which by its nature is very noisy and will generate
// harmful interference if not properly filtered.
//
// SDR Targets:
//
// 1) Amateur (HAM) radio communications on the high frequency (HF)
//    bands, otherwise known as "shortwave bands". Support SSB, CW,
//    AM, and narrow band FM (28Mhz and above).
//
// 2) Support ham radio digital modes on the shortwave bands, using
//    the SSB/CW modulation baselines.
//
// 3) Support WSPR automated ham radio beacon communications on the
//    long wave (488Khz) and short wave (HF) bands.
//
// 4) Support AM modulated CW tone in the LW amateur radio bands
//    similar to a long wave navigation beacon.
//
// 5) AM Broad Cast Band (BCB) transmission for local entertainment
//    purposes. (530Khz - 1.6Mhz)
//
// 6) FM broadcast band transmission with audio and Radio Data System
//    (RDS) digital information transmissions for enterainment purposes.
//
// 7) Support for higher frequency amateur radio bands (above 50Mhz)
//    depending on the capabilities of the FPGA.
//
// Goal is to start with transmit first, and eventually support receive
// with an outboard RF A/D converter with the receive front end
// filters, RF/LNA (Low Noise Amplifier), etc. This A/D converter
// could be narrow band using the audio spectrum similar to many PC
// based SDR's, or wide band depending on the bandwidth available
// in the FPGa and front end A/D. For example the LimeSDR LMS7002M
// provides programmable up to 62.5Mhz of bandwidth and interfaces
// to an Altera Cyclone IV FPGA.
//
// One of the main goals of this F32C based SDR SoC is to operate
// 100% standalone on the FPGA without any external ARM cores
// or PC. It can feed a PC, tablet, cellphone, or RaspberryPi as
// a "user interface", and to provide for less common SDR radio
// modes. But for common baseline commuications such as SSB, CW, AM,
// FM, and popular ham narrow band digitial modes its fully self
// contained with the soft core on the FPGA running the control
// loop, and the synthesis/filtering in FPGA RTL hardware blocks.
//
// The design can still take advantage of external bluetooth, WiFi,
// RasperryPi Zero/Compute module, or small ARM based microcontrollers
// for the "UI" aspects such as running a front panel, keyboard, mouse,
// etc. But all of the radio outside of the required analog filters
// and amplifiers are contained within the single FPGA.
//
// One main intention is to be able to "port" the F32C subsystem
// configuration with the SoC SDR to a software defined radio
// board such as the LimeSDR.
//
// Note at the current time the FPGA on the LimeSDR is rather
// small. Hopefully they will offer a larger FPGA upgrade option.
//
// If not, separating the radio into (2) FPGA's is acceptable, but this
// SoC SDR core will optimize for runnig the F32C CPU and standalone
// SDR radio on one FPGA, and the other FPGA will handle any Digital
// Down Conversion (DDC) and controls of the radio front end, and interface
// to the primary SDR SoC as a programmable 16 bit PCM I+Q stream. This
// model allows leveraging other SDR front ends such as SoftRock, etc.
// which mostly operate with a PC, but in this case would be standalone.
//

// SoC SDR block model:
//
//  RF output is from parallel AM and FM Direct Digital Synthesis modules.
//
//  Single Side Band (SSB) may be a separate module, or option on AM.
//
//  They are fed with a PCM modulation input which can come from multiple
//  sources.
//
//  PCM sources are:
//
//    1) CPU from memory mapped registers. This allows CPU processing
//       and generation of signals.
//
//    2) Sine wave synthesis modules. Useful for testing, calibration,
//       and provide signal sources for other modulation modes such as
//       a base tone for an AM based CW tone for a longwave beacon,
//       or a carrier wave for SSB synthesis.
//
//    3) Audio A/D converter on I2C or SPI bus for external sources
//       such as a communications microphone, music for broadcast, etc.
//
//    4) Filter/Modulator blocks. Filter blocks provide signal procesing and
//       DSP synthesis for generation of different modes such as AM, SSB,
//       tone modulated CW, etc.
//

#define SDR_FREQUENCY                  0xFFFFFC00 // Register 0

//
// FM RDS registers
//

#define FMRDS_DATA                     0xFFFFFC04 // Register 1
#define FMRDS_MESSAGE_LENGTH           0xFFFFFC08 // Register 2
#define FMRDS_CONTROL                  0xFFFFFC0C // Register 3

// Control bit definitions
#define FMRDS_CONTROL_FM_CW_ENABLE     0x00000001 // Enable FM carrier wave output
#define FMRDS_CONTROL_MODULATOR_ENABLE 0x00000002 // FM PCM input enable
#define FMRDS_CONTROL_RDS_DATA_ENABLE  0x00000004 // FM Radio Data System Enable

// AM generator definitions
#define SDR_CONTROL_AM_CW_ENABLE       0x00000008 // Enable AM carrier wave output
#define SDR_CONTROL_AM_PCM_ENABLE      0x00000010 // AM PCM input/modulator enable

//
// SDR registers
//
#define SDR_CONTROL0                   0xFFFFFC10 // Register 4
#define SDR_CONTROL1                   0xFFFFFC14
#define SDR_CONTROL2                   0xFFFFFC18
#define SDR_CONTROL3                   0xFFFFFC1C

//
// 16 bit PCM mono (left only) or stereo polar modulation
//
#define SDR_PCM_DATA                   0xFFFFFC20 // L (15 downto 0) R (31 downto 16)

// 16 bit PCM mono or stereo left channel I + Q modulation
#define SDR_PCM_IQ_RESERVED            0xFFFFFC24 // I (15 downto 0) Q (31 downto 16)

// 16 bit PCM stereo right channel I + Q modulation
#define SDR_PCM_IQ_R_RESERVED          0xFFFFFC28 // I (15 downto 0) Q (31 downto 16)

#define SDR_PCM_CS                     0xFFFFFC2C // Register 11 PCM source control/status register

#define FMRDS_CONTROL_PCM_FULL         0x00000001 // Bit 0 == 0 ready for next PCM polar sample
#define FMRDS_CONTROL_PCM_IQ_FULL      0x00000002 // Bit 1 == 0 ready for next PCM IQ sample
#define FMRDS_CONTROL_PCM_IQ_R_FULL    0x00000004 // Bit 2 == 0 ready for next PCM IQ right chan sample


//
// PCM signal synthesizer registers.
//
// Depending on configuration the I + Q registers may be used
// for a real time modulation signal.
//

// Control/Status register
#define SDR_PCM_SYNTH_CS                   0xFFFFFC30 // Register 12

#define SDR_PCM_SYNTH_ENABLE               0x00000001
#define SDR_PCM_SINE_ENABLE                0x00000002 // Generate sine tone
#define SDR_PCM_WRITE_PROTECT              0x00000004

//
// The modulation (PCM) synthesizer has block RAM tables for sine/cosine
// generation and FFT tables.
//
// The upper 16 bits is the address map for 64K 16 bit word
// locations, with the data in the lower 16 bits.
//
// The split of the address space is defined by the particular
// synthesizer module and mode enabled. For example multiple sine/cosine,
// and FFT tables may be present within ranges of the 64K address space.
//
#define SDR_PCM_SYNTH_RAM                  0xFFFFFC34 // Address (31 downto 16) Data (16 downto 0)
                                                      // Register 13

//
// Frequency constant for generated signal.
//
// Value depends on the synthesizer design.
//
// Changing this value in real time will frequency modulate
// the signal.
//
#define SDR_PCM_SYNTH_FREQ                 0xFFFFFC38 // Register 14

//
// Amplitude constant for generated signal.
//
// Value depends on the synthesizer design.
//
// Changing this value in real time will amplitude modulate
// the signal.
//
// 0x3C => 0011 xxxx
#define SDR_PCM_SYNTH_AMPLITUDE             0xFFFFFC3C // Register 15

//
// 16 registers currently implemented in sdr.vhd
//

//
// SDR software interface overview
//
// Definitive definition is in sdr.vhd
//
// Multiple carrier generators and modulators are available and
// can operate independently. The two currently implemented are AM
// and FM. Both are capable of CW (Continuous Wave) and
// have a frequency span up to 125Mhz.
//
// CW on/off morse keying, or AFSK with 100% modulation is provided
// by the CW_ENABLE bit. Note that this bit must be set to provide a
// carrier if AM or FM modulation is used.
//
// The FM carrier generator and modulator produces an FM carrier
// compatible with the FM broadcast band including generation of
// a stereo audio L-R subcarrier. Compile time options can be used
// to customize this for communications FM (narrow band).
//
// TODO: It is desireable to make these runtime options if they don't
// add extra delays to the already tight 250Mhz DDS
// (Direct Digital Synthesis) path. Narrow band FM for amateur (HAM)
// radio communications may be a separate parameterized instance.
//
// FM has the option of using RDS (Radio Data Service), which is
// a digital side band on its FM broadcast frequency modulation
// signal. To use this, carrier (CW enable) and modulation must
// be enabled.
//
// AM is under development, and will have the options of using
// either the PCM polar modulation signals, or PCM I + Q signals
// to allow synthesized signals such as SSB (Single Side Band),
// AFSK (Audio Frequency Shift Keying) and ham radio digital modes
// to be generated.
//
// Currently the PCM audio system is fed to both the AM and FM
// modulators, so they will share a signal source. In communications
// usage only one of the carrier generators/modulators will be
// operating at a time. This is to keep the complexity and resources
// consumed by the PCM audio system down.
//
// Plans:
//
// Plans are to provide for DMA for PCM audio output. The DMA
// would be able to source from either block ram, or SDRAM if
// its supported by a configuration.
//
// Plans are also to enable SDRAM to hold audio data, with the
// core SDR programs running out of block ram. By using DMA to access
// SDRA CPU stalls and delays are minimized.
//

//
// TODO: Update the code to use these proper constants once validated.
// IO_BASE + 0x400 => IO_BASE + 0x4FF
//
//#define IO_FMRDS 0x400
//
// This results in 0xFFFFFC00
//
// #define RDS_ADDRESS (IO_BASE + IO_FMRDS)
//

//
// Menlo:
//
// 0xFFFFFC00 - Hz register (32 bits)
// 0xFFFFFC04 - data register (combined 16 bits + 8 bits)
// 0xFFFFFC08 - message length register (32 bits)
//              Current RDS message buffer position on read.
// 0xFFFFFC0C - FM RDS control register.
//              Bit 0 - CW enable
//              Bit 1 - Modulator enable
//              Bit 2 - RDS Data enable
//
// It appears the data is now accessed with the message byte address
// in the upper 16 bit word of the message data register, with the
// byte in the lower byte. See msgbyte() function in this file for details.
//
// This means the message address register is not used.
//
// See the hardware decoder address 0xFFFF_FC00 for top_fmrds_bram_mips on
// DE10-Nano Menlo project. This appears to be inline with the actual
// implementation of FM RDS here.
//
// -- ---------------------------------------------------------------------------------
// -- | 3 3 2 2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1 0 0 | 0 0 0 0 | 0 0 0 0 |
// -- | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
// -- ---------------------------------------------------------------------------------
// -- | 1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 0 0   0 0 0 0   0 0 0 0 | 0xFFFF_FC00
// -- ---------------------------------------------------------------------------------
// -- | F         F         F         F       | F         C       | 0         0       |
// -- ---------------------------------------------------------------------------------
// -- |                            Frequency 31 - 0                                   |
// -- ---------------------------------------------------------------------------------
//
// -- ---------------------------------------------------------------------------------
// -- | 3 3 2 2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1 0 0 | 0 0 0 0 | 0 0 0 0 |
// -- | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
// -- ---------------------------------------------------------------------------------
// -- | 1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 0 0   0 0 0 0   0 1 0 0 | 0xFFFF_FC04
// -- ---------------------------------------------------------------------------------
// -- | F         F         F         F       | F         C       | 0         4       |
// -- ---------------------------------------------------------------------------------
// -- | Data Address 31 - 16                  | Unknown 15 - 8    | Data 7 - 0        |
// -- ---------------------------------------------------------------------------------
//
// -- ---------------------------------------------------------------------------------
// -- | 3 3 2 2 | 2 2 2 2 | 2 2 2 2 | 1 1 1 1 | 1 1 1 1 | 1 1 0 0 | 0 0 0 0 | 0 0 0 0 |
// -- | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 | 9 8 7 6 | 5 4 3 2 | 1 0 9 8 | 7 6 5 4 | 3 2 1 0 |
// -- ---------------------------------------------------------------------------------
// -- | 1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 1 1   1 1 0 0   0 0 0 0   1 0 0 0 | 0xFFFF_FC08
// -- ---------------------------------------------------------------------------------
// -- | F         F         F         F       | F         C       | 0         8       |
// -- ---------------------------------------------------------------------------------
// -- |                            Message Length 31 - 0                              |
// -- ---------------------------------------------------------------------------------
//

#define RDS_GROUP_LENGTH 4
#define RDS_BITS_PER_GROUP (RDS_GROUP_LENGTH * (RDS_BLOCK_SIZE+RDS_POLY_DEG))

/* The RDS error-detection code generator polynomial is
   x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + x^0
*/
#define RDS_POLY 0x1B9
#define RDS_POLY_DEG 10
#define RDS_MSB_BIT 0x8000
#define RDS_BLOCK_SIZE 16

#define RDS_RT_LENGTH 64
#define RDS_PS_LENGTH 8

class SDR {
  public:
    SDR();

    //
    // those function have immediate effect
    // pass them value and transmitter starts
    // sending it
    //
    // RDS message types described at:
    //
    // https://en.wikipedia.org/wiki/Radio_Data_System
    //

    void ps(char *ps);  // Program Service
    void rt(char *rt);  // Radio Text
    void ct(int16_t year, uint8_t mon, uint8_t mday, uint8_t hour, uint8_t min, int16_t gmtoff);
    void pi(uint16_t pi_code); // Program Identification Code (PI Code)
    void ta(uint8_t ta);       // Traffic Announcement

    void stereo(uint8_t stereo);

    inline void WriteControlRegister(uint32_t value)
    {
      volatile uint32_t *fmrds_control = (volatile uint32_t *)FMRDS_CONTROL;
      *fmrds_control = value;
    }

    inline void Hz(uint32_t f)
    {
      volatile uint32_t *fmrds_hz = (volatile uint32_t *)SDR_FREQUENCY;
      *fmrds_hz = f;
    }

    inline void msgbyte(uint16_t a, uint8_t b)
    {
      // Bits 7-0 of 32 bit register
      //volatile uint8_t *fmrds_msg_data = (volatile uint8_t *)FMRDS_DATA;

      // This is a 16 bit word address (upper half of FMRDS_DATA 32 bit register)
      //volatile uint16_t *fmrds_msg_addr = (volatile uint16_t *) FMRDS_ADDR;

      // This is data and length accessed in one 32 bit access
      volatile uint32_t *fmrds_msg_data_addr = (volatile uint32_t *)FMRDS_DATA;

      *fmrds_msg_data_addr = (a << 16) | b;

      //*fmrds_msg_addr = a;
      //*fmrds_msg_data = b;
    }

    inline void length(uint16_t len)
    {
      volatile uint32_t *fmrds_msg_len = (volatile uint32_t *)FMRDS_MESSAGE_LENGTH;
      *fmrds_msg_len = len-1;
    }

  private:

    //
    // those functions take value to class but
    // doesn't change transmitted data
    //
    void new_pi(uint16_t pi_code); // Program Identification Code (PI Code)
    void new_rt(char *rt);         // Radio Text
    void new_ps(char *ps);         // 
    void new_ta(uint8_t ta);       // Traffic Announcement

    // those functions convert values of this class to output binary
    void binary_buf_crc(uint8_t *buffer, uint16_t *blocks);
    void binary_ps_group(uint8_t *buffer, uint8_t group_number);
    void binary_rt_group(uint8_t *buffer, uint8_t group_number);
    void binary_ct_group(uint8_t *buffer);

    // copies output binary to hardware transmission buffer
    void send_ps();
    void send_rt();
    void send_ct();

    // calculates checksums for binary format  
    uint16_t crc(uint16_t block);

    // internal RDS message in cleartext
    uint16_t value_pi = 0xCAFE; // program ID
    uint8_t signal_ta = 0; // traffic announcement
    uint8_t signal_stereo = 0;
    uint8_t afs = 1;
    uint16_t af[7] = {1079, 0, 0, 0, 0, 0, 0}; // x0.1 MHz
    char string_ps[RDS_PS_LENGTH]; // short 8-char text shown as station name
    char string_rt[RDS_RT_LENGTH]; // long 64-char text

    /* time stuff */
    uint8_t tm_hour, tm_min;
    uint8_t tm_mon, tm_mday;
    int16_t tm_year; // year-1900
    int16_t tm_gmtoff; // local time to gmt offset in seconds

    const uint16_t offset_words[4] = {0x0FC, 0x198, 0x168, 0x1B4};
    // We don't handle offset word C' here for the sake of simplicity
};
#endif // _SDR_h
