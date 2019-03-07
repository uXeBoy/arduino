
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
void sw_cw_loop(PAPPLICATION_VARS p);

void ProcessUserInput(PAPPLICATION_VARS p);

void setup()
{
    PAPPLICATION_VARS p = &G_app;

    Serial.begin(BAUD_RATE);

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);     

    // Initialize morse
    G_Morse.Initialize(&G_Keyer);

    // Select which setup for which radio operating mode to use

    // FM Radio Data System (RDS) setup.
    //fm_rds_setup(p);

    // Shortwave CW (Continuous Wave or morse code) setup.
    sw_cw_setup(p);
}

void loop()
{
   // FM Radio Data System (RDS) setup.
   //fm_rds_loop(&G_app);

   // Shortwave CW (Continuous Wave or morse code) setup.
   sw_cw_loop(&G_app);

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

void sw_cw_loop(PAPPLICATION_VARS p)
{
  static uint8_t loopCount;

  Serial.print("loopCount ");
  Serial.println(loopCount);

  //G_Morse.SendString("test");
  //G_Morse.SendString("What Hath God Wrought?");
  //G_Morse.SendString("the quick brown fox jumped over the lazy dog");
  //G_Morse.SendString("vvv");
  //G_Morse.SendString("xxx");
  //G_Morse.SendString("fpga dds sdr morse transmitter");
  //G_Morse.SendString("xxx");

  G_Morse.SendString("xxx");

  delay(2000);

  // AM modulated tone, 500Hz, count for 10 seconds
  am_tone(1, 10000);

  delay(2000);

  G_Morse.SendString("aaa");

  delay(2000);

  pure_carrier(10000);

  delay(2000);

  loopCount++;
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

void
ProcessUserInput(PAPPLICATION_VARS p)
{
    if (Serial.available()) {
        p->Ch = Serial.read();
    }
    else {
        p->Ch = -1;
        return;
    }

    /* Set flags based on UART command */
    switch(p->Ch)
    {
        case 'a':
            // AM Modulation on
            break;

        case 'A':
            // AM Modulation off
            break;

        case 'c':
            // Carrier on
            break;

        case 'C':
            // Carrier off
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
            break;

        case 'M':
            // Morse Code off
            break;

        case 'r':
            // RDS on
            break;

        case 'R':
            // RDS off
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
	    break;    

    } // end switch
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
    Serial.println("reg test failed");
  }
 
  rw = (uint16_t*)addr;
  if (*rw != 0x0100) {
    Serial.println("reg test low word failed");
  }

  rw = (uint16_t*)(addr+2);
  if (*rw != 0x0302) {
    Serial.println("reg test high word failed");
  }

  rb = (uint8_t*)(addr);
  if (*rb != 0x00) {
    Serial.println("reg test byte 0 failed");
  }

  rb = (uint8_t*)(addr+1);
  if (*rb != 0x01) {
    Serial.println("reg test byte 1 failed");
  }

  rb = (uint8_t*)(addr+2);
  if (*rb != 0x02) {
    Serial.println("reg test byte 2 failed");
  }

  rb = (uint8_t*)(addr+3);
  if (*rb != 0x03) {
    Serial.println("reg test byte 3 failed");
  }

  //
  // Byte write
  //

  rb = (uint8_t*)(addr);
  *rb = 0xFF;
  if (*r != 0x030201FF) {
    Serial.println("reg test write byte 0 failed");
  }

  rb = (uint8_t*)(addr+1);
  *rb = 0xFF;
  if (*r != 0x0302FFFF) {
    Serial.println("reg test write byte 1 failed");
  }

  rb = (uint8_t*)(addr+2);
  *rb = 0xFF;
  if (*r != 0x03FFFFFF) {
    Serial.println("reg test write byte 2 failed");
  }

  rb = (uint8_t*)(addr+3);
  *rb = 0xFF;
  if (*r != 0xFFFFFFFF) {
    Serial.println("reg test write byte 3 failed");
  }


  //
  // Word write
  //

  rw = (uint16_t*)(addr);
  *rw = 0xAAAA;
  if (*r != 0xFFFFAAAA) {
    Serial.println("reg test write word 0 failed");
  }

  rw = (uint16_t*)(addr+2);
  *rw = 0x5555;
  if (*r != 0xFFFFAAAA) {
    Serial.println("reg test write byte 1 failed");
  }

  // Back to zero
  *r = 0x00000000;
  if (*r != 0x00000000) {
    Serial.println("reg test back to zero failed");
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
    Serial.println("reg test 0 failed");
  }

  r = (volatile uint32_t *)SDR_CONTROL1;
  *r = 0x07060504;

  if (*r != 0x07060504) {
    Serial.println("reg test 1 failed");
  }

  r = (volatile uint32_t *)SDR_CONTROL2;
  *r = 0x0B0A0908;

  if (*r != 0x0B0A0908) {
    Serial.println("reg test 2 failed");
  }

  r = (volatile uint32_t *)SDR_CONTROL3;
  *r = 0x0F0E0D0C;

  if (*r != 0x0F0E0D0C) {
    Serial.println("reg test 3 failed");
  }

  //
  // Validate byte, word, and long read and writes.
  //
  register_test32(SDR_CONTROL0);
  register_test32(SDR_CONTROL1);
  register_test32(SDR_CONTROL2);
  register_test32(SDR_CONTROL3);
}

//
// Determine largest memory block that can be allocated.
//
void
memory_test()
{
    char* p;
    int length;

    length = 1024*1024;

    while(1) {

        p = (char*)malloc(length);
        if (p == NULL) {
            length = length / 2;
            continue;
        }

        Serial.print("Memory Test: Allocated");
        Serial.println(length, HEX);

        memset(p, 0, length);

        free(p);

        return;
    }
}
