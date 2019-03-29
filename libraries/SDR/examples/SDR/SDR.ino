
//
// TODO:
//
// 03/20/2019
//
//  Add real time scaled integer signal magnitude calculations.
//
//  TODO:
//   #Bring in Arduino morse program.
//   #Add serial console UI to allow commands, frequency change, morse/RDS messages.
//   #Add R/W test code for new SDR registers.
//   A 1Khz signal generation using delay(1) (actually 500hz with 1ms on/off)
//    - better: added RTL logic in SDR SOC to pace a 48Khz PCM sample rate
//   Add support for hardware PWM modulation
//   Connect FM PCM modulation.
//   Test sound sample.
//   AM modulation experiments
//   I+Q support
//   SSB modulation experiments
//   Port WSPR
//   Other ham digital modes.
//   Receiver support
//   Integrate with SoftRock, etc.
//
// Delta Sigma Modulation
// https://hackaday.io/project/162477-serial-port-sdr
// https://hackaday.io/project/162477-serial-port-sdr/log/156449-multiple-modulation-methods
//
// 03/15/2019
//
// Sine wave generation in real time using floating point math misses
// captures.
//
// 50Mhz 32 bit MIPS running from block ram, software floating point.
//
// 10 second duration takes actual 65 seconds (6X)
// 
// 48000 cycles out of 48000 cycles missed.
//
// Build a table and test that.
//
//  - Explore fixed point arithmetic for signal generation
//    - shift samples over by 10-14 bits, use 32 bit fixed point arithmetic
//      to generate signal from baseline and table to represent the sine/cosine.
//
//  - goal is to move real time signal generation to the FPGA as
//    an RTL blocks.
//
//  - Generate I+Q signals using above table driven scaled integer
//    techniques for moving to the FPGA.
// 
//  - Start writing the new signal synthesis modules in Verilog and
//    just use VHDL for the wrapper in the project.
//
//  - Need to create a SoC template in Verilog, but for now it will be
//    components instantiated from the sdr.vhd SoC.
//

/*
  03/03/2019

  Menlopark Innovation LLC

  Software Defined Radio (SDR) created from FM RDS core.

  03/05/2019
*/

#define AM_SUPPORT 1
#define SW_SUPPORT 1

// Serial port baud rate for Serial Monitor, UI
#define BAUD_RATE 115000

//
// AM frequency.
//
//#define AM_FREQUENCY 750000
#define AM_FREQUENCY 7125000

//
// 7.125Mhz Legal morse code band for all US hams, with proper filters.
//
// 7.185Mhz USB on MAX10 FPGA (legal for higher license's)
//
// MAX10/DE10-Lite may need an external reference clock for radio frequency
// accuracy.
//
#define SHORTWAVE_CW_FREQUENCY 7125000

//
// Note: My MAX10 FPGA generates a 0.5Mhz lower frequency,
// but sometimes will jump back.
//
#define FM_FREQUENCY 107900000

#define FM_RDS_PROGRAM_IDENTIFIER 0xCAFE

#define FM_RDS_PROGRAM_SERVICE "MenloSDR"

#define FM_RDS_SIGN_ON_RADIO_TEXT "FPGA Radio from DE10-Lite ..."

/*
This is a test program that dynamically updates RDS message.
AUTHOR=EMARD
LICENSE=GPL
*/

#include <SDR.h>

#include "MorseSync.h"

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;

//
// RDS is an operating mode of SDR for FM Radio Data System (RDS) display.
//

//
// Menlo SoC SDR module.
//
SDR sdr = SDR();

//
// Keep application variables in one struct.
//
#define MAX_INPUT_LINE 80

typedef struct _APPLICATION_VARS {

    // SDR library object
    SDR* sdr;

    /* Variable to store UART received character */
    int Ch;

    bool stringInput;
    int  inputBufferIndex;
    char inputBuffer[MAX_INPUT_LINE+1];

    bool verbose;
    bool fm_carrier_on;
    bool fm_modulation_on;

    bool am_carrier_on;
    bool am_modulation_on;
    bool morse_on;

    // FM RDS support
    uint16_t pi;      // Program Identification Code (national networks, etc.)
    char ps[9];       // Program Service
    char rt[65];      // Radio Text

    // Sine Synthesizer
    bool     synth_on;
    uint32_t synth_frequency;
    uint32_t synth_amplitude;

} *PAPPLICATION_VARS, APPLICATION_VARS;

APPLICATION_VARS G_app;

// Forward References
void fm_rds_setup(PAPPLICATION_VARS p);
void fm_rds_message(PAPPLICATION_VARS p, char*, char*);

void fm_rds_loop(PAPPLICATION_VARS p);

void sdr_registers_test();
void memory_test();

uint16_t*
generate_sinewave_table(
    double Samples,
    double Amplitude
    );

void free_sinewave_table(uint16_t* table);

void generate_test_sinewave();
void sdr_pcm_send_sine_wave_real_time();
void sdr_pcm_send_sine_wave_table();

void
sdr_pcm_send_sine_wave_table_scaled(
    int SamplesPerCycle,
    int PcmRate,
    double Frequency,
    double Amplitude,
    int    Time
    );

uint32_t synth_table_test(PAPPLICATION_VARS p);

#if AM_SUPPORT
void am_carrier_on(PAPPLICATION_VARS p);
void am_carrier_off(PAPPLICATION_VARS p);
#endif

#if SW_SUPPORT
void sw_am_test(PAPPLICATION_VARS p);
void sw_cw_setup(PAPPLICATION_VARS p);
void sw_cw_test(PAPPLICATION_VARS p);
#endif

void
ProcessUserInput(
    PAPPLICATION_VARS p
    );

class CwKeyer : public MorseKeyer {

public:
      CwKeyer() {
      }

      void Initialize(PAPPLICATION_VARS app) {
          m_app = app;
      }

      void virtual KeyDown(int interval) {
#if AM_SUPPORT
          am_carrier_on(m_app);
#endif
          delay(interval);
#if AM_SUPPORT
          am_carrier_off(m_app);
#endif
      }

private:
      PAPPLICATION_VARS m_app;
};

CwKeyer G_Keyer;

MorseSync G_Morse;

#if AM_SUPPORT
void am_setup(PAPPLICATION_VARS p, uint32_t Frequency)
{
    sdr.Hz(Frequency); // Hz carrier wave frequency
}

void
am_carrier_on(
    PAPPLICATION_VARS p
    )
{
    uint32_t cr = SDR_CONTROL_AM_CW_ENABLE;
    p->am_carrier_on = TRUE;
    sdr.WriteControlRegister(cr);
    digitalWrite(led, HIGH);
}

void
am_carrier_off(
    PAPPLICATION_VARS p
    )
{
    uint32_t cr = 0;
    sdr.WriteControlRegister(cr);
    p->am_carrier_on = FALSE;
    digitalWrite(led, LOW);
}

void
am_pure_carrier(
    PAPPLICATION_VARS p,
    int duration
    )
{
    am_carrier_on(p);
    delay(duration);
    am_carrier_off(p);
}

void
am_tone(
    PAPPLICATION_VARS p,
    int half_period,
    int count
    )
{
    uint32_t cr_on = SDR_CONTROL_AM_CW_ENABLE;
    uint32_t cr_off = 0;

    for (int i = 0; i < count; i++) {

        sdr.WriteControlRegister(cr_on);
        //digitalWrite(led, HIGH);

        delay(half_period);

        sdr.WriteControlRegister(cr_off);

        delay(half_period);

        //digitalWrite(led, LOW);
    }
}
#endif // AM_SUPPORT

#if SW_SUPPORT
void
sw_cw_setup(
    PAPPLICATION_VARS p,
    uint32_t Frequency
    )
{
    uint32_t cr = 0;

    sdr.WriteControlRegister(cr);

    // Set carrier frequency in Hertz
    sdr.Hz(Frequency);
}

void
sw_cw_test(PAPPLICATION_VARS p)
{
  int loopCount;
  int maxLoops = 10;

  if (p->verbose) Serial.println("Morse Test...");

  for (loopCount = 0; loopCount < maxLoops; loopCount++) {

      if (p->verbose) Serial.print("loop: ");
      if (p->verbose) Serial.println(loopCount);

      G_Morse.SendString("xxx");
      G_Morse.SendString("test");
      G_Morse.SendString("xxx");
  }

  if (p->verbose) Serial.println(" Done.");
}

void
sw_am_test(PAPPLICATION_VARS p)
{
  if (p->verbose) Serial.print("AM Tone Test...");

  // AM modulated tone, 500Hz, count for 10 seconds
  am_tone(p, 1, 10000);

  if (p->verbose) Serial.println(" Done.");
}
#endif

//
// FM RDS Setup
//

void fm_rds_on()
{
    uint32_t cr = 
        FMRDS_CONTROL_FM_CW_ENABLE |
        FMRDS_CONTROL_MODULATOR_ENABLE |
        FMRDS_CONTROL_RDS_DATA_ENABLE;       

    sdr.WriteControlRegister(cr);

    digitalWrite(led, HIGH);
}

void fm_carrier_on()
{
    uint32_t cr = FMRDS_CONTROL_FM_CW_ENABLE;
    sdr.WriteControlRegister(cr);
    digitalWrite(led, HIGH);
}

void fm_carrier_off()
{
    uint32_t cr = 0;
    sdr.WriteControlRegister(cr);
    digitalWrite(led, LOW);
}

void fm_pure_carrier(int duration)
{
    fm_carrier_on();
    delay(duration);
    fm_carrier_off();
}

void setup()
{
    PAPPLICATION_VARS p = &G_app;

    p->sdr = &sdr;

    p->verbose = TRUE;

    Serial.begin(BAUD_RATE);

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);     

    // Initialize morse
    G_Keyer.Initialize(p);
    G_Morse.Initialize(&G_Keyer);
}

void loop()
{
   ProcessUserInput(&G_app);
}

//
// Setup to transmit FM RDS data.
//
void
fm_rds_setup(PAPPLICATION_VARS p)
{
  unsigned int i;

  p->pi = FM_RDS_PROGRAM_IDENTIFIER;

  for(i = 0; i < sizeof(p->ps)-1; i++) {
    p->ps[i] = ' ';
  }

  for(i = 0; i < sizeof(p->rt)-1; i++) {
    p->rt[i] = ' ';
  }

  snprintf(p->ps, sizeof(p->ps), "%s", FM_RDS_PROGRAM_SERVICE);

  snprintf(p->rt, sizeof(p->rt), "%s", FM_RDS_SIGN_ON_RADIO_TEXT);

  /* Setup initial RDS text */
  sdr.pi(p->pi); // Program Identifier (network or station ID)
  sdr.stereo(0); // 0-Inform over RDS that we send Mono, 1-Stereo

  sdr.ta(0);  // 0-No, 1-Traffic Announcements
  sdr.ps(p->ps); // Program Service, 8-char text, displayed as station name
  sdr.rt(p->rt); // Radio Text, 64-char text, not every radio displays it

  sdr.Hz(FM_FREQUENCY); // Hz carrier wave frequency

  sdr.length(260); // bytes message length (260 default)

  // Set control register to enable carrier, modulation, FM RDS modulator.
  fm_rds_on();
}

void
fm_rds_message(
    PAPPLICATION_VARS p,
    char* programService,
    char* radioText
    )
{
  static uint8_t number;

  if (programService != NULL) {
      // Max 8 characters + NULL
      snprintf(p->ps, sizeof(p->ps), "%s", programService);

      sdr.ps(p->ps); // Program Service
  }

  if (radioText != NULL) {
      snprintf(p->rt, sizeof(p->rt), "%05d: %s", number % 100000, radioText);

      number++; // increment message number

      sdr.rt(p->rt); // Radio Text
  }

  //sdr.ct(2015,7,22,15,30,900); // Clock Time and Data

  // print actual status on serial
  Serial.print("0x");
  Serial.print(p->pi, HEX);
  Serial.print(" ");
  Serial.print(p->ps);
  Serial.print(" ");
  Serial.println(p->rt);

  return;
}

void fm_rds_loop(PAPPLICATION_VARS p)
{
  static char l_rt[65];      // Radio Text

  snprintf(l_rt, sizeof(l_rt), "FPGA Radio from DE10Lite");

  fm_rds_message(p, NULL, l_rt);
  
  delay(2000); // wait 2 seconds
}

//
// This tests the FM RDS registers.
//
void fm_rds_registers_display(PAPPLICATION_VARS p)
{
  volatile uint32_t *r;

  r = (volatile uint32_t*)SDR_FREQUENCY;
  Serial.print("Frequency: ");
  Serial.println(*r);

  r = (volatile uint32_t*)FMRDS_DATA;
  Serial.print("Data: ");
  Serial.println(*r);

  r = (volatile uint32_t*)FMRDS_MESSAGE_LENGTH;
  Serial.print("Message Length: ");
  Serial.println(*r);

  r = (volatile uint32_t*)FMRDS_CONTROL;
  Serial.print("Control: ");
  Serial.println(*r);
}

//
// UserInterface
// UI
//

void
ResetInputBuffer(PAPPLICATION_VARS p)
{
    p->stringInput = false;
    p->inputBufferIndex = 0;
    p->inputBuffer[p->inputBufferIndex] = 0;
}

//
// Processes input buffer state machine.
//
// 0 - No input string. Error or just <CR>
//
// 1 - n - Done with input. Count of characters input.
//
// -1 - Still processing.
//
int
ProcessInputBuffer(PAPPLICATION_VARS p, int c)
{
    //
    // This handles various states for inputing numbers, lines, etc.
    //
    if (p->stringInput) {

        // CR or line feed terminates input line
        if ((c == 0xa) || (c == 0xd)) {
            p->stringInput = false;

            // return current count, could be zero for just <CR> entered.
            return p->inputBufferIndex;
        }

        if (p->inputBufferIndex >= MAX_INPUT_LINE-1) {
            // overflow
            Serial.println("input buffer overflow");

            // Don't return incomplete commands, input data
            ResetInputBuffer(p);

            // No data return.
            return 0;
        }

        p->inputBuffer[p->inputBufferIndex] = c;
        p->inputBufferIndex++;

        // Keep string null terminated
        p->inputBuffer[p->inputBufferIndex] = 0;
    }

    // still inputing
    return -1;
}

//
// Return:
//
// 1 - got an input character. Hint to keep processing for more.
//
// 0 - No more input characters.
//
int
ProcessUserInputWorker(PAPPLICATION_VARS p)
{
    int ret;

    if (Serial.available()) {
        p->Ch = Serial.read();
    }
    else {
        p->Ch = -1;
        // No more input characters
        return 0;
    }

    if (p->stringInput) {
        ret = ProcessInputBuffer(p, p->Ch);
        if (ret == 0) {
            // nothing entered or overflow.

            // Cancel any states that set p->stringInput

            ResetInputBuffer(p);
            return 1;
        }
        else if (ret == (-1)) {
            // still processing input
            return 1;
        }
        else  {
            //
            // input done. ret is the count of characters input.
            // Handle any states that set p->stringInput
            //
            ResetInputBuffer(p);
            return 1;
        }

        return 1;
    }

    /* Set flags based on UART command */
    switch(p->Ch)
    {
        case 'a':
            // AM Modulation on
#if AM_SUPPORT
            am_setup(p, AM_FREQUENCY);
#endif
#if SW_SUPPORT
            sw_am_test(p);
#endif
            break;

        case 'A':
#if AM_SUPPORT
            // AM Modulation off
            am_carrier_off(p);
#endif
            break;

        case 'b':
            break;

        case 'c':
            // FM Carrier on
            p->fm_carrier_on = TRUE;
            fm_carrier_on();
            Serial.println("CW carrier ON");
            break;

        case 'C':
            // Carrier off
            fm_carrier_off();
            p->fm_carrier_on = FALSE;
            Serial.println("FM CW carrier OFF");
            break;

        case 'd':
            break;

        case 'e':
            break;

        case 'f':
            // Display frequency
            break;

        case 'F':
            // Set frequency
            break;

        case 'g':
            break;

        // 'h' is for help, handled below

        case 'i':
            break;

        case 'j':
            break;

        case 'k':
            break;

        case 'l':
            break;

        case 'm':
#if SW_SUPPORT
            // Morse Code  on
            sw_cw_setup(p, SHORTWAVE_CW_FREQUENCY);
            sw_cw_test(&G_app);
#endif
            break;

        case 'M':
            // Morse Code off
            break;

        case 'o':
            break;

        case 'p':
            break;

        case 'q':

            //sdr_pcm_send_sine_wave_table();

            // Table driven sine wave synthesis
            sdr_pcm_send_sine_wave_table_scaled(
                16,    // SamplesPerCycle
                48000, // PcmRate
                (double)1000,  // Frequency
                (double)32767, // Amplitude
                10             // Time in seconds
            );
            break;

        case 'Q':
            // Real time sine wave synthesis
            sdr_pcm_send_sine_wave_real_time();
            break;

        case 'r':
            // RDS on
            fm_rds_setup(p);

            fm_rds_registers_display(p);
            break;

        case 'R':
            // RDS off
            break;

        case 's':
            // FM Modulation on
            break;

        case 'S':
            // FM Modulation off
            break;

        case 't':
            // test sine wave generation/math
            generate_test_sinewave();
            break;

        case 'T':
            synth_table_test(p);
            break;

        case 'u':
            break;

        case 'v':
            // Verbose on
            p->verbose = TRUE;
            break;

        case 'V':
            // Verbose off
            p->verbose = TRUE;
            break;

        case 'w':
            break;

        case 'x':
            break;

        case 'y':
            break;

        case 'z':
            // Memory test
            Serial.print("Memory Test...");
            memory_test();
            Serial.println(" Done.");
            break;

        case 'Z':
            // Registers test
            Serial.print("Registers Test...");
            sdr_registers_test();
            Serial.println(" Done.");
            break;

        case '0':
        break;

        case '1':
        break;

        case '2':
        break;

        case '3':
        break;

        case '4':
        break;

        case '5':
        break;

        case '6':
        break;

        case '7':
        break;

        case '8':
        break;

        case '9':
        break;

	case 0:
	case -1:
	    /* No new data was received */
	    break;

	case 0xa:
	    // Ignore carriage return
	    break;

	case 0xd:
	    // Ignore linefeed
	    break;

	default:
	    Serial.print(F("Unrecognized Command: 0x"));
            Serial.println(p->Ch, HEX);

	    // FallThrough

	case '?':
	case 'h':
	case 'H':
	    // Help
	    Serial.println(F("Menlo SDR Help:\r\n"));
            Serial.println("a - AM modulation on, A AM modulation off");
            Serial.println("c - CW carrier on, C - CW Carrier off");
            Serial.println("f - Display frequency, F - Set frequency");
            Serial.println("m - Morse Code on, M - Morse Code off");
            Serial.println("q - Synthesize sine wave PCM modulation (table driven)");
            Serial.println("Q - Synthesize sine wave PCM modulation (real time)");
            Serial.println("r - FM RDS on, R - FM RDS off");
            Serial.println("s - FM modulation on, S - FM modulation off");
            Serial.println("t - Show test sine wave calculations");
            Serial.println("T - Test hardware sine wave table");
            Serial.println("v - Verbose on, V - Verbose off");
            Serial.println("z - Memory Test");
            Serial.println("Z - Registers test");
            Serial.println("?, h, H - Help menu");
	    break;    

    } // end switch

    return 1;
}

//
// Process user input till no more characters
// in hardware buffer.
//
void
ProcessUserInput(PAPPLICATION_VARS p)
{
    int ret;

    while(1) {
        ret = ProcessUserInputWorker(p);
        if (ret == 0) {
            return;
        }
    }
}

//
// ============= SYNTH SoC Module =============
//

uint32_t
synth_table_read_control()
{
  volatile uint32_t *r;
  uint32_t tmp;

  r = (uint32_t*)SDR_PCM_SYNTH_CS;

  tmp = *r;

  return tmp;
}

void
synth_table_write_control(uint32_t value)
{
  volatile uint32_t *r;

  r = (uint32_t*)SDR_PCM_SYNTH_CS;

  *r = value;

  return;
}

void
synth_table_write_frequency(uint32_t value)
{
  volatile uint32_t *r;

  r = (uint32_t*)SDR_PCM_SYNTH_FREQ;

  *r = value;

  return;
}

void
synth_table_write_amplitude(uint32_t value)
{
  volatile uint32_t *r;

  r = (uint32_t*)SDR_PCM_SYNTH_AMPLITUDE;

  *r = value;

  return;
}

void
synth_table_write_freq_amplitude(uint32_t frequency, uint32_t amplitude)
{
  volatile uint32_t *r;

  r = (uint32_t*)SDR_PCM_SYNTH_FREQ;
  *r = frequency;

  r = (uint32_t*)SDR_PCM_SYNTH_AMPLITUDE;
  *r = amplitude;

  return;
}

//
// Write entry to the synth table
//
void synth_table_write_entry(uint16_t index, uint16_t value)
{
  volatile uint32_t *r;
  uint32_t tmp;

  r = (uint32_t*)SDR_PCM_SYNTH_RAM;

  // Address (31 downto 16) Data (16 downto 0)
  tmp = value;;
  tmp = tmp | (index << 16);
  
  *r = tmp;
}

//
// Read the contents of the synthtable.
//
// Note: SDR_PCM_SYNTH_CS SDR_PCM_WRITE_PROTECT must be set to read
// existing contents without overwriting since the register
// is shared with address and data.
//
uint16_t
synth_table_read_entry(uint16_t index)
{
  volatile uint32_t *r;
  uint32_t tmp;

  r = (uint32_t*)SDR_PCM_SYNTH_RAM;

  // Address (31 downto 16) Data (16 downto 0)
  tmp = 0;
  tmp = tmp | (index << 16);
  
  // Write the address
  *r = tmp;

  // Now read it back with the data
  tmp = *r;

  return (uint16_t)tmp & 0x0000FFFF;
}

//
// Load the synth table
//
void
synth_load_table(
    uint16_t* table,
    uint16_t  tableSize
    )
{
    uint16_t index;

    for(index = 0; index < tableSize; index++) {
        synth_table_write_entry(index, table[index]);
    }

    return;
}

//
// Validate the contents of the synth table.
//
// Sets write protect.
//
int
synth_validate_table(
    uint16_t* table,
    uint16_t  tableSize
    )
{
    uint16_t index;
    uint16_t tmp;

    // Set write protect
    synth_table_write_control(SDR_PCM_WRITE_PROTECT);

    for(index = 0; index < tableSize; index++) {
        tmp = synth_table_read_entry(index);

        if (tmp != table[index]) {
            return index;
        }
    }

    return -1;
}

//
// Test the synthesizer table.
//
// test_synth
// synth_test
//
uint32_t
synth_table_test(PAPPLICATION_VARS p)
{
    uint16_t* table = NULL;
    uint16_t tableSize;
    int ret;

    double samples = 48.0;
    double amplitude = 32767.0;

    table = generate_sinewave_table(samples, amplitude);

    tableSize = (uint16_t)samples;

    synth_load_table(table, tableSize);

    ret = synth_validate_table(table, tableSize);
    if (ret != (-1)) {
        Serial.print("Synth Table validation failure index: ");
        Serial.println(ret);
    }
    else {
        Serial.println("Synth Table validation Success!");
    }

    free_sinewave_table(table);

    return 0;
}

//
// ============= SDR Registers Tests =============
//

//
// Look for aliases for the registers.
//
void register_alias_test()
{
  volatile uint32_t *r;

  r = (volatile uint32_t *)SDR_CONTROL0;
  *r = SDR_CONTROL0;
  if (*r != SDR_CONTROL0) {
    Serial.print("reg test failed SB SDR_CONTROL0 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL1;
  *r = SDR_CONTROL1;
  if (*r != SDR_CONTROL1) {
    Serial.print("reg test failed SB SDR_CONTROL1 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL2;
  *r = SDR_CONTROL2;
  if (*r != SDR_CONTROL2) {
    Serial.print("reg test failed SB SDR_CONTROL2 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL3;
  *r = SDR_CONTROL3;
  if (*r != SDR_CONTROL3) {
    Serial.print("reg test failed SB SDR_CONTROL3 is: ");
    Serial.println(*r, HEX);
  }

  //
  // Now look for aliases in the I/O range from
  // 0xFFFFF800 - 0xFFFFFFFF
  //
  // Note: The register range will show up, and any
  // undecoded bits will as well. Still a handy test.
  //

  r = (volatile uint32_t*)0xFFFFF800;

  // Going to get 32 bit wrap around
  while((r != 0) && (r < (uint32_t*)0xFFFFFFFC)) {

    if ((*r == SDR_CONTROL0) || (*r == SDR_CONTROL1) ||
        (*r == SDR_CONTROL2) || (*r == SDR_CONTROL3)) {
        Serial.print("Possible wrap around. Address: ");
        Serial.print((uint32_t)r, HEX);
        Serial.print(" Data: ");
        Serial.println(*r, HEX);
    }

    r++;
  }
}

//
// Test 32 bit register including byte and word read/write access.
//
void register_test32(void* addr)
{
  volatile uint32_t *r;
  volatile uint16_t *rw;
  volatile uint8_t  *rb;

  r = (uint32_t*)addr;

  *r = 0x03020100;

  if (*r != 0x03020100) {
    Serial.print("reg test failed SB 0x03020100 is: ");
    Serial.println(*r, HEX);
  }
 
  rw = (uint16_t*)addr;
  if (*rw != 0x0100) {
    Serial.print("reg test low word failed SB 0x0100 is: ");
    Serial.println(*rw, HEX);
  }

  rw = (uint16_t*)((int)addr+2);
  if (*rw != 0x0302) {
    Serial.print("reg test high word failed SB 0x0302 is: ");
    Serial.println(*rw, HEX);
  }

  rb = (uint8_t*)(addr);
  if (*rb != 0x00) {
    Serial.println("reg test byte 0 failed SB 0x00 is: ");
    Serial.println(*rb, HEX);
  }

  rb = (uint8_t*)((int)addr+1);
  if (*rb != 0x01) {
    Serial.println("reg test byte 1 failed SB 0x01 is: ");
    Serial.println(*rb, HEX);
  }

  rb = (uint8_t*)((int)addr+2);
  if (*rb != 0x02) {
    Serial.println("reg test byte 2 failed SB 0x02 is: ");
    Serial.println(*rb, HEX);
  }

  rb = (uint8_t*)((int)addr+3);
  if (*rb != 0x03) {
    Serial.println("reg test byte 3 failed SB 0x03 is: ");
    Serial.println(*rb, HEX);
  }

  //
  // Byte write
  //

  rb = (uint8_t*)(addr);
  *rb = 0xFF;
  if (*r != 0x030201FF) {
    Serial.println("reg test write byte 0 failed SB 0x030201FF is: ");
    Serial.println(*r, HEX);
  }

  rb = (uint8_t*)((int)addr+1);
  *rb = 0xFF;
  if (*r != 0x0302FFFF) {
    Serial.println("reg test write byte 1 failed SB 0x0302FFFF is: ");
    Serial.println(*r, HEX);
  }

  rb = (uint8_t*)((int)addr+2);
  *rb = 0xFF;
  if (*r != 0x03FFFFFF) {
    Serial.println("reg test write byte 2 failed SB 0x03FFFFFF is: ");
    Serial.println(*r, HEX);
  }

  rb = (uint8_t*)((int)addr+3);
  *rb = 0xFF;
  if (*r != 0xFFFFFFFF) {
    Serial.println("reg test write byte 4 failed SB 0xFFFFFFFF is: ");
    Serial.println(*r, HEX);
  }

  //
  // Word write
  //

  // write word
  rw = (uint16_t*)(addr);
  *rw = 0xAAAA;

  // read 32 bits
  if (*r != 0xFFFFAAAA) {
    Serial.println("reg test write word 0 failed SB 0xFFFFAAAA is: ");
    Serial.println(*r, HEX);
  }

  rw = (uint16_t*)((int)addr+2);
  *rw = 0x5555;
  if (*r != 0x5555AAAA) {
    Serial.println("reg test write word 1 failed SB 0x5555AAAA is: ");
    Serial.println(*r, HEX);
  }

  // Back to zero
  *r = 0x00000000;
  if (*r != 0x00000000) {
    Serial.println("reg test back to zero failed SB 0x00000000 is: ");
    Serial.println(*r, HEX);
  }
}

// Timeout in milliseconds
uint32_t
sdr_pcm_wait_for_ready(int timeout)
{
  volatile uint32_t *r_cs;
  uint32_t status;
  int index;

  r_cs = (volatile uint32_t *)SDR_PCM_CS;

  status = *r_cs;

  if ((status & FMRDS_CONTROL_PCM_FULL) == 0) {
      return status;
  }

  // Wait for PCM data ready (not full condition)
  for (index = 0; index < timeout; index++) {

      status = *r_cs;

      if ((status & FMRDS_CONTROL_PCM_FULL) == 0) {
          return status;
      }

      delay(1);
  }

  return status;
}

//
// Returns 0 if no wait.
//
// Returns value of wait loop if data has been written.
//
// Returns timeout value if timed out.
//
int
sdr_pcm_send_data(uint32_t data, int timeout)
{
  volatile uint32_t *r_cs;
  volatile uint32_t *r_pcm_data;
  uint32_t status;
  int index;

  r_cs = (volatile uint32_t *)SDR_PCM_CS;

  // R (31 downto 16) L (15 downto 0)
  r_pcm_data = (volatile uint32_t*)SDR_PCM_DATA;

  status = *r_cs;

  if ((status & FMRDS_CONTROL_PCM_FULL) == 0) {
      *r_pcm_data = data;
      return 0; // no wait
  }

  // Wait for PCM data ready (not full condition)
  for (index = 0; index < timeout; index++) {

      status = *r_cs;

      if ((status & FMRDS_CONTROL_PCM_FULL) == 0) {
          *r_pcm_data = data;
          return index;
      }
  }

  return timeout;
}

void sdr_pcm_test()
{
  uint32_t status;
  volatile uint32_t *r_pcm_data;

  // R (31 downto 16) L (15 downto 0)
  r_pcm_data = (volatile uint32_t*)SDR_PCM_DATA;

  status = sdr_pcm_wait_for_ready(1000);
  if (status & FMRDS_CONTROL_PCM_FULL) {
    // Timeout
  }

  *r_pcm_data = 0x00000000;

  return;
}

//
// This tests the SDR registers.
//
void sdr_registers_test()
{
  volatile uint32_t *r;

  //
  // Validate independence of addressing.
  //
  r = (volatile uint32_t *)SDR_CONTROL0;
  *r = 0x03020100;

  if (*r != 0x03020100) {
    Serial.println("reg test 0 failed SB 0x03020100 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL1;
  *r = 0x07060504;

  if (*r != 0x07060504) {
    Serial.println("reg test 1 failed SB 0x07060504 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL2;
  *r = 0x0B0A0908;

  if (*r != 0x0B0A0908) {
    Serial.println("reg test 2 failed SB 0x0B0A0908 is: ");
    Serial.println(*r, HEX);
  }

  r = (volatile uint32_t *)SDR_CONTROL3;
  *r = 0x0F0E0D0C;

  if (*r != 0x0F0E0D0C) {
    Serial.println("reg test 3 failed SB 0x0F0E0D0C is: ");
    Serial.println(*r, HEX);
  }

  //
  // Validate byte, word, and long read and writes.
  //
  register_test32((void*)SDR_CONTROL0);
  register_test32((void*)SDR_CONTROL1);
  register_test32((void*)SDR_CONTROL2);
  register_test32((void*)SDR_CONTROL3);

  //
  // Look for aliases in the I/O space.
  //
  register_alias_test();
}

void
memory_verify(void* addr, int length, uint8_t pattern)
{
    uint8_t* p = (uint8_t*)addr;

    for (int index = 0; index < length; index++) {
      if (*p != pattern) {
        Serial.print("Pattern mismatch Addr: ");
        Serial.print((uint32_t)p, HEX);
        Serial.print(" SB: ");
        Serial.print(pattern, HEX);
        Serial.print(" IS: ");
        Serial.println(*p, HEX);
        return;
      }

      p++;
    }
}

//
// Determine largest memory block that can be allocated.
//
void*
allocate_largest_block_possible(char pattern, int* plength)
{
    void* p;
    int length;

    length = 1024*1024;

    while(1) {

        p = malloc(length);
        if (p == NULL) {
            length = length / 2;
            continue;
        }

        Serial.print("Memory Test: Allocated: ");
        Serial.println(length, HEX);

        memset(p, pattern, length);

        *plength = length;

        return p;
    }

    // out of memory
    return NULL;
}

void
memory_test()
{
    void* p1;
    void* p2;
    void* p3;
    void* p4;
    int length1;
    int length2;
    int length3;
    int length4;

    p1 = allocate_largest_block_possible(0x55, &length1);
    p2 = allocate_largest_block_possible(0xAA, &length2);
    p3 = allocate_largest_block_possible(0x01, &length3);
    p4 = allocate_largest_block_possible(0xFE, &length4);

    if (p1 != NULL) {
      memory_verify(p1, length1, 0x55);
      free(p1);
    }

    if (p2 != NULL) {
      memory_verify(p2, length2, 0xAA);
      free(p2);
    }

    if (p3 != NULL) {
      memory_verify(p3, length3, 0x01);
      free(p3);
    }

    if (p4 != NULL) {
      memory_verify(p4, length4, 0xFE);
      free(p4);
    }

    return;
}

//
// Generate sinewave table for signed 16 bit PCM audio
// in allocated memory.
//
// Samples - Number of samples for the sine wave
//
// Amplitude - Maximum amplitude of the wave form.
//
//                The generated sine wave is from:
//
//                0          90        180       270         360
//                0 => +MaxAmplitude => 0 => -MaxAmplitude => 0
//
// Returns:
//  Pointer to: uint16_t[Samples]
//
uint16_t*
generate_sinewave_table(
    double Samples,
    double Amplitude
    )
{
    double rad;
    double rad_increment;
    double mag;
    int memorySize;
    int index;
    int16_t pcm_data_16;
    uint16_t* array;

    rad_increment = (2.0 * 3.14) / (double)(Samples);

    memorySize = Samples * sizeof(uint16_t);

    array = (uint16_t*)malloc(memorySize);
    if (array == NULL) {
        return NULL;
    }

    memset(array, 0, memorySize);

    //
    // This generates one complete sine wave cycle
    //
    rad = 0.0;
    for (index = 0; index < Samples; index++) {

        mag = sin(rad) * Amplitude;

        //
        // Downcast, caller must ensure the Amplitude fits with the
        // signed 16 range of the PCM audio sample.
        //
        pcm_data_16 = (int16_t)mag;
    
        array[index] = pcm_data_16;
    
        rad = rad + rad_increment;
    }

    return array;
}

void
free_sinewave_table(uint16_t* table)
{
    if (table != NULL) {
        free((void*)table);
    }
}

//
// Generates a test sine wave and outputs it for math verification.
//
// Used for designing sine tables that will go into the synthesis blocks.
//
void
generate_test_sinewave_worker(
    int SamplesPerCycle,
    double Amplitude
    )
{
    double rad;
    double deg;
    double mag;
    double rad_increment;
    double deg_increment;
    int index;
    int16_t pcm_data_16;

    // Samples for whole sine wave
    int sampleMax = 16;

    Serial.print("Generate SineWave: SamplesPerCycle: ");
    Serial.print(SamplesPerCycle);
    Serial.print(" Amplitude: ");
    Serial.println(Amplitude);

    //
    // PI day, 03/14/2019. Arduino sine function is in radians.
    //
    rad_increment = (2.0 * 3.14) / (double)(sampleMax);

    deg_increment = 360 / (double)(sampleMax);

    Serial.print("rad_increment: ");
    Serial.println(rad_increment);

    Serial.print("deg_increment: ");
    Serial.println(deg_increment);

    rad = 0.0;
    deg = 0.0;
    for (index = 0; index < sampleMax; index++) {

        mag = sin(rad) * Amplitude;

        Serial.print(deg);
        Serial.print(": ");
        Serial.print(mag);
        Serial.println("");

        // Downcast, hopefully the sign survives
        pcm_data_16 = (int16_t)mag;

        Serial.print("signed16: ");
        Serial.println(pcm_data_16);

        // Use a simple add in the main loop rather than multiply by ratio
        rad = rad + rad_increment;
        deg = deg + deg_increment;
    }

    return;
}

void
generate_test_sinewave()
{
    //
    // Test scaled integers for sine wave tables.
    //
    // The more the magnitude is shifted from the 1.0
    // base line, the less rounding occurs in the
    // dropping of the fractional part when converted into the
    // 16 bit integer entry in the table.
    //
    // A scaling by 8 (3 LSB's) provides reasonable accuracy
    // while scaling by 256 (8 LSB's) is better, but not likely
    // required for communications.
    //
    // Scaling by 16 is a sweet spot that supports 4 LSB's of
    // the fractional component.
    //
    // Note that this magnitude scale becomes a "DC offset", and
    // can be filtered out readily.
    //
    // The smallest value that provides the needed accuracy is
    // desirable as this sets the lower floor of the dynamic
    // range for the sine wave. Example: if 4 bits are used
    // for the fraction, then 12 bits are available for the modulation
    // range and represent 72.24 dB for modulation.
    //
    // The 16 bit signal itself has 96.33 dB of dynamic range.
    //
    // https://en.wikipedia.org/wiki/Audio_bit_depth
    //
    // Amplitude modulation of the sign wave now involves simple
    // integer multiplication. Sticking to powers of two, half powers
    // of two (shift and add) in an AM modulator can give a reasonable
    // range without consuming multiplier units.
    //
    // Different oversample values are tried to see the effect
    // on the fractional component, as oversampling would generate
    // smaller differences in the fractional component and be
    // subject to more distortion.
    //

    // Baseline with fractions
    generate_test_sinewave_worker(16, (double)1.0);

    // Full scale for 16 bit signed PCM
    generate_test_sinewave_worker(16, (double)32767.0);

    // Scaled by 256 for fixed integer math in RTL
    generate_test_sinewave_worker(16, (double)256.0);

    // Scaled by 16 for fixed integer math in RTL
    // Use 16X oversampling for the test.
    generate_test_sinewave_worker(32, (double)16.0);

    // Scaled by 8 for fixed integer math in RTL
    // Use 16X oversampling for the test.
    generate_test_sinewave_worker(32, (double)8.0);

    // Scaled by 256 for fixed integer math in RTL
    // Use 32X oversampling for the test.
    generate_test_sinewave_worker(64, (double)256.0);

    // Scaled by 8 for fixed integer math in RTL
    // Use 32X oversampling for the test.
    generate_test_sinewave_worker(64, (double)8.0);
}

//
// This uses a table driven approach to synthesizing a PCM
// audio modulation sine wave.
//
void
sdr_pcm_send_sine_wave_table()
{
    int missed = 0;
    int timeout = 10000;
    int16_t pcm_data_16;
    uint32_t pcm_data;
    int index;
    int ret;
    int loopIndex;
    uint16_t* table = NULL;

    double samples = 48.0;
    double amplitude = 32767.0;

    int sendLoops = 1000 * 10; // 10 seconds for 1000Hz sine wave

    table = generate_sinewave_table(samples, amplitude);

    Serial.println("Table Driven Send Sine Wave Start");

    for (loopIndex = 0; loopIndex < sendLoops; loopIndex++) {

        //
        // This generates one complete sine wave cycle
        //
	for (index = 0; index < samples; index++) {

	    pcm_data_16 = table[index];

            // Send same signal to L+R channels
            pcm_data = (pcm_data_16 << 16) | pcm_data_16;
    
    	    ret = sdr_pcm_send_data(pcm_data, timeout);
	    if ((ret == 0) && (loopIndex != 0)) {
		missed++;
	    }
	    else if (ret == timeout) {
	       // Not accepting data, error
	       Serial.println("Timeout sending Sine Wave");
               free_sinewave_table(table);
	       return;
	    }
	}
    }

    Serial.print("Table Driven Send Sine Wave Done, missed PCM samples: ");
    Serial.println(missed);

    free_sinewave_table(table);

    return;
}

//
// This uses a table driven approach to synthesizing a PCM
// audio modulation sine wave.
//
// It creates a sine wave output of a specified frequecy
// based on a sample rate, and per cycle sample count.
//
// SamplesPerCycle - Number of samples per cycle
//
// Frequency - Frequency in HZ for the generated sine wave
//
// PcmRate - PCM sample rate. Example: 48Khz, 8Khz.
//           The is the output rate the routine generates into.
//
// Amplitude - Full scale positive amplitude.
//             Example: 32767 for 16 bit signed PCM.
//             Negative half is -Amplitude (-32767).
//             An Amplitude of 1.0 (Unity) allows the table to be used
//             to generate sine waves of different amplitudes with simple
//             processing such as shift and add.
//
// Time - Time to generate the sine wave for in seconds.
//
void
sdr_pcm_send_sine_wave_table_scaled(
    int SamplesPerCycle,
    int PcmRate,
    double Frequency,
    double Amplitude,
    int    Time
    )
{
    int missed = 0;
    int timeout = 10000;
    int16_t pcm_data_16;
    uint32_t pcm_data;
    int ret;
    int sampleIndex;
    int timeIndex;
    int pcmIndex;
    int sampleScale;
    int scaleCounter;
    uint16_t* table = NULL;
    double samplesPerSecond;
    double samplesPerTableIndex;
    uint16_t range;

    //
    // The generated signal is a sign wave with SamplesPerCycle
    // samples. The sign wave is pre-computed into a table for
    // one complete cycle since the floating point math can take
    // a large number of CPU cycles. The calculated table has
    // SamplesPerCycle number of entries to represent one complete
    // 360 degree (2*PI radians) cycle.
    //
    // The complete sine wave represented by all the table values
    // is presented Frequency times per second. The total number of
    // PCM sample changes output is this frequency multiplied by the
    // samples in the table.
    // 
    // The PcmRate is typically higher than the samples per second
    // of the generated sine wave, and a factor is calculated for how
    // many PCM output cycles must tick before the next sample from
    // the array is sent. The array itself wraps around as required
    // restarting the sine wave at the end of each cycle.
    //

    // Samples per second for the signal
    samplesPerSecond = Frequency * (double)SamplesPerCycle;

    Serial.print("SamplesPerSecond: ");
    Serial.println(samplesPerSecond);

    // Number of PCM ouput cycles per signal sample
    samplesPerTableIndex = (double)PcmRate / samplesPerSecond;

    Serial.print("SamplesPerTableIndex: ");
    Serial.println(samplesPerTableIndex);

    //
    // The value of sampleScale determines the sine wave output
    // frequency and is the value that would be placed into the
    // frequency register of a hardware RTL block.
    //
    // Updating this value during operation will change the
    // output frequency using the configured ratio of per-calculated
    // table samples and PcmRate.
    //
    // A pre-computed table of values for sampleScale that is updated
    // by a modulation loop would allow for frequency modulation of
    // the sine wave signal while avoiding floating point calculations
    // in the inner loop.
    //

    // Integer truncate
    sampleScale = (int)samplesPerTableIndex;

    Serial.print("SamplesPerTableIndex: ");
    Serial.println(samplesPerTableIndex);

    //
    // Generate table with one sine wave cycle with the number of specified
    // samples at the specified peak amplitude.
    //
    // TODO: This takes the less compute approach of calculating a
    // fixed magnitude into the table. For hardware implementation
    // the magnitude needs to be calculated at PCM output time, or
    // the table re-computed for signal magnitude changes, such
    // as when producing amplitude modulation.
    //
    // table = generate_sinewave_table((double)SamplesPerCycle, Amplitude);

    // Scale by 4 bits (16)
    table = generate_sinewave_table((double)SamplesPerCycle, 16);

    range = (uint16_t)(Amplitude / 16.0);

    Serial.print("Table Driven Send Sine Wave Start Frequency: ");
    Serial.print(Frequency);
    Serial.print(" SamplesPerCycle: ");
    Serial.print(SamplesPerCycle);
    Serial.print(" Amplitude: ");
    Serial.println(Amplitude);
    
    Serial.print("sampleScale: ");
    Serial.println(sampleScale);

    Serial.print("range: ");
    Serial.println(range);

    scaleCounter = 0;
    sampleIndex = 0;

    // Seconds of signal generation
    for (timeIndex = 0; timeIndex < Time; timeIndex++) {

        // One second worth of PCM samples at PCM sample rate
        for (pcmIndex = 0; pcmIndex < PcmRate; pcmIndex++) {

            // Send next table sample at index
            pcm_data_16 = (table[sampleIndex] * range);

            // Send same signal to L+R channels
            pcm_data = (pcm_data_16 << 16) | pcm_data_16;
            ret = sdr_pcm_send_data(pcm_data, timeout);
	    if ((ret == 0) && (sampleIndex != 0)) {
		missed++;
	    }
	    else if (ret == timeout) {
	       // Not accepting data, error
	       Serial.println("Timeout sending Sine Wave");
               free_sinewave_table(table);
	       return;
	    }

            // One sine cycle at SamplesPerCycle rate
            scaleCounter++;
            if (scaleCounter >= sampleScale) {
                scaleCounter = 0;

                // Advance to the next table index for next send, modulo table samples.
                sampleIndex++;
                if (sampleIndex > SamplesPerCycle) {
                    sampleIndex = 0;
                }
            }
        }
    }

    Serial.print("Table Driven Send Sine Wave Done, missed PCM samples: ");
    Serial.println(missed);

    free_sinewave_table(table);

    return;
}

//
// This uses real time floating point calculatios to generate
// the sine wave.
//
// As such it places the highest burden on the CPU for real time
// synthesis of all the PCM samples of the waveform.
//
// Current test uses a 48Khz PCM sample rate for a mono signal.
//
// This is higher than ham radio communications need, but does not
// include the I + Q synthesis required for ham communication.
//
void
sdr_pcm_send_sine_wave_real_time()
{
    double rad;
    double mag;
    double rad_increment;
    int index;
    int16_t pcm_data_16;
    int ret;
    uint32_t pcm_data;
    int missed = 0;
    int timeout = 10000;
    int sendLoops = 1000 * 10; // 10 seconds for 1000Hz sine wave
    int loopIndex;

    // Samples for whole sine wave
    int sampleMax = 48;

    // Top of 16 bit PCM positive range
    double top = 32767.0;

    Serial.println("Send SineWave: ");

    //
    // PI day, 03/14/2019. Arduino sine function is in radians.
    //
    rad_increment = (2.0 * 3.14) / (double)(sampleMax);

    for (loopIndex = 0; loopIndex < sendLoops; loopIndex++) {

        //
        // This generates one complete sine wave cycle
        //
    	rad = 0.0;
	for (index = 0; index < sampleMax; index++) {

	    mag = sin(rad) * top;

	    // Downcast, hopefully the sign survives
	    pcm_data_16 = (int16_t)mag;

            // Send same signal to L+R channels
            pcm_data = (pcm_data_16 << 16) | pcm_data_16;
    
    	    ret = sdr_pcm_send_data(pcm_data, timeout);
	    if (ret == 0) {
		missed++;
	    }
	    else if (ret == timeout) {
	       // Not accepting data, error
	       Serial.println("Timeout sending Sine Wave");
	       return;
	    }

	    // Use a simple add in the main loop rather than multiply by ratio
	    rad = rad + rad_increment;
	}
    }

    Serial.print("Send Sine Wave Done, missed PCM samples: ");
    Serial.println(missed);

    return;
}

//
// Sine Wave with 48 samples
//
// 24 samples "+" side, 24 samples "-" side.
//
// The 16 bit number represents 16 bit signed PCM audio
// so numbers with bit 15 set are negative.
//
// Note that are in decimal here so range from 0 - 65535.
//
uint16_t Sinewave[] = {
    0,
    4276,
    8480,
    12539,
    16383,
    19947,
    23169,
    25995,
    28377,
    30272,
    31650,
    32486,
    32767,
    32486,
    31650,
    30272,
    28377,
    25995,
    23169,
    19947,
    16383,
    12539,
    8480,
    4276, // 24 entries
    0,
    61259,
    57056,
    52997,
    49153,
    45589,
    42366,
    39540,
    37159,
    35263,
    33885,
    33049,
    32768,
    33049,
    33885,
    35263,
    37159,
    39540,
    42366,
    45589,
    49152,
    52997,
    57056,
    61259  // 48 entries
    };

