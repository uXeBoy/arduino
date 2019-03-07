
/*
  03/03/2019

  Menlopark Innovation LLC

  Software Defined Radio (SDR) created from FM RDS core.

  03/05/2019
  TODO:
   #Bring in Arduino morse program.
   #Add serial console UI to allow commands, frequency change, morse/RDS messages.
   Add R/W test code for new SDR registers.
   A 1Khz signal generation using delay(1) (actually 500hz with 1ms on/off)
   Add support for hardware PWM modulation
   Connect FM PCM modulation.
   Test sound sample.
   AM modulation experiments
   SSB modulation experiments
   Port WSPR
   Other ham digital modes.
*/

#define BAUD_RATE 115000

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

/*
This is a test program that dynamically updates RDS message.
AUTHOR=EMARD
LICENSE=GPL
*/

#include <SDR.h>

#include "MorseSync.h"

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;

// RDS is an operating mode of SDR for FM Radio Data System (RDS) display.
RDS rds = RDS();

uint16_t pi = 0xCAFE;
char ps[9] = "TEST1234";
char rt[65] = "ABCDEFGH";

//
// Keep application variables in one struct.
//
typedef struct _APPLICATION_VARS {

    /* Variable to store UART received character */
    int Ch;

    bool verbose;
    bool carrier_on;
    bool fm_modulation_on;
    bool am_modulation_on;
    bool morse_on;

} *PAPPLICATION_VARS, APPLICATION_VARS;

APPLICATION_VARS G_app;

void carrier_on()
{
    uint32_t cr = FMRDS_CONTROL_CW_ENABLE;
    rds.WriteControlRegister(cr);
    digitalWrite(led, HIGH);
}

void carrier_off()
{
    uint32_t cr = 0;
    rds.WriteControlRegister(cr);
    digitalWrite(led, LOW);
}

void pure_carrier(int duration)
{
    carrier_on();
    delay(duration);
    carrier_off();
}

void am_tone(int half_period, int count)
{
    uint32_t cr_on = FMRDS_CONTROL_CW_ENABLE;
    uint32_t cr_off = 0;

    for (int i = 0; i < count; i++) {

        rds.WriteControlRegister(cr_on);
        //digitalWrite(led, HIGH);

        delay(half_period);

        rds.WriteControlRegister(cr_off);

        delay(half_period);

        //digitalWrite(led, LOW);
    }
}


class CwKeyer : public MorseKeyer {

public:
      CwKeyer() {
      }

      void Initialize(RDS* sdr) {
          m_sdr = sdr;
      }

      void virtual KeyDown(int interval) {
          carrier_on();
          delay(interval);
          carrier_off();
      }

private:
      RDS* m_sdr;
};

CwKeyer G_Keyer;

MorseSync G_Morse;

void fm_rds_setup(PAPPLICATION_VARS p);
void fm_rds_loop(PAPPLICATION_VARS p);

void sw_cw_setup(PAPPLICATION_VARS p);
void sw_cw_test(PAPPLICATION_VARS p);

void sw_am_test(PAPPLICATION_VARS p);

void sdr_registers_test();
void memory_test();

void ProcessUserInput(PAPPLICATION_VARS p);

void setup()
{
    PAPPLICATION_VARS p = &G_app;

    p->verbose = TRUE;

    Serial.begin(BAUD_RATE);

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);     

    // Initialize morse
    G_Morse.Initialize(&G_Keyer);

    // Select which setup for which radio operating mode to use
    // Currently done by UI

    // FM Radio Data System (RDS) setup.
    //fm_rds_setup(p);

    // Shortwave CW (Continuous Wave or morse code) setup.
    //sw_cw_setup(p);
}

void loop()
{
   ProcessUserInput(&G_app);
}

void sw_cw_setup(PAPPLICATION_VARS p)
{
  uint32_t frequency;

  frequency = SHORTWAVE_CW_FREQUENCY;

  // Set carrier frequency in Hertz
  rds.Hz(frequency);

  //carrier_on();
}

void
sw_cw_test(PAPPLICATION_VARS p)
{
  if (p->verbose) Serial.print("Morse Test...");
  G_Morse.SendString("xxx");
  G_Morse.SendString("test");
  G_Morse.SendString("xxx");
  if (p->verbose) Serial.println(" Done.");
}

void
sw_am_test(PAPPLICATION_VARS p)
{
  if (p->verbose) Serial.print("AM Tone Test...");
  // AM modulated tone, 500Hz, count for 10 seconds
  am_tone(1, 10000);
  if (p->verbose) Serial.println(" Done.");
}

//
// Setup to transmit FM RDS data.
//
void fm_rds_setup(PAPPLICATION_VARS p) {
  // int i;
  unsigned int i;

  for(i = 0; i < sizeof(rt)-1; i++) {
    rt[i] = '@'+i; // ascii map
  }

  /* Setup initial RDS text */
  rds.pi(pi); // station numeric ID
  rds.stereo(0); // 0-Inform over RDS that we send Mono, 1-Stereo
  rds.ta(0);  // 0-No, 1-Traffic Announcements
  rds.ps(ps); // 8-char text, displayed as station name
  rds.rt(rt); // 64-char text, not every radio displays it

  rds.Hz(FM_FREQUENCY); // Hz carrier wave frequency

  rds.length(260); // bytes message length (260 default)
}

//
// This tests the FM RDS registers.
//
void fm_rds_registers_display(PAPPLICATION_VARS p)
{
  volatile uint32_t *r;

  r = (volatile uint32_t*)FMRDS_FREQUENCY;
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

void fm_rds_loop(PAPPLICATION_VARS p)
{
  static uint8_t number;

  snprintf(ps, sizeof(ps), "TEST%04d", number % 10000);
  snprintf(rt, sizeof(rt), "%05d Zzz...", number % 100000);

  // send strings for transmission
  rds.ps(ps);
  rds.rt(rt);
  // rds.ct(2015,7,22,15,30,900);

  // print actual status on serial
  Serial.print("0x");
  Serial.print(pi, HEX);
  Serial.print(" ");
  Serial.print(ps);
  Serial.print(" ");
  Serial.println(rt);

  delay(2000); // wait 2 seconds
  number++; // increment number
}

int
ProcessUserInputWorker(PAPPLICATION_VARS p)
{
    if (Serial.available()) {
        p->Ch = Serial.read();
    }
    else {
        p->Ch = -1;
        return 0;
    }

    /* Set flags based on UART command */
    switch(p->Ch)
    {
        case 'a':
            // AM Modulation on
            sw_am_test(&G_app);
            break;

        case 'A':
            // AM Modulation off
            break;

        case 'c':
            // Carrier on
            p->carrier_on = TRUE;
            carrier_on();
            Serial.println("CW carrier ON");
            break;

        case 'C':
            // Carrier off
            carrier_off();
            p->carrier_on = FALSE;
            Serial.println("CW carrier FF");
            break;

        case 's':
            // FM Modulation on
            break;

        case 'S':
            // FM Modulation off
            break;

        case 'f':
            // Display frequency
            break;

        case 'F':
            // Set frequency
            break;

        case 'm':
            // Morse Code  on
            sw_cw_setup(p);
            sw_cw_test(&G_app);
            break;

        case 'M':
            // Morse Code off
            break;

        case 'r':
            // RDS on
            fm_rds_setup(p);
            fm_rds_registers_display(p);
            break;

        case 'R':
            // RDS off
            break;

        case 'Z':
            // Registers test
            Serial.print("Registers Test...");
            sdr_registers_test();
            Serial.println(" Done.");
            break;

        case 'v':
            // Verbose on
            p->verbose = TRUE;
            break;

        case 'V':
            // Verbose off
            p->verbose = TRUE;
            break;

        case 'z':
            // Memory test
            Serial.print("Memory Test...");
            memory_test();
            Serial.println(" Done.");
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
            Serial.println("c - CW carrier on, C - CW Carrier off");
            Serial.println("a - AM modulation test");
            Serial.println("s - FM modulation on, S - FM modulation off");
            Serial.println("r - FM RDS on, R - FM RDS off");
            Serial.println("f - Display frequency, F - Set frequency");
            Serial.println("m - Morse CW test");
            Serial.println("Z - Registers test");
            Serial.println("z - Memory Test");
            Serial.println("v - Verbose on, V - Verbose off");
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

