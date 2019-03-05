
/*
  03/03/2019

  Menlopark Innovation LLC

  Software Defined Radio (SDR) created from FM RDS core.
*/

#define BAUD_RATE 115000

//
// 7.125Mhz Legal morse code band for all US hams, with proper filters.
//
// 7.185Mhz USB on MAX10 FPGA (legal for higher license's
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

// Pin 13 has an LED connected on most Arduino boards.
int led = 13;

// RDS is an operating mode of SDR for FM Radio Data System (RDS) display.
RDS rds = RDS();

uint16_t pi = 0xCAFE;
char ps[9] = "TEST1234";
char rt[65] = "ABCDEFGH";

void carrier_on() {
    uint32_t cr = FMRDS_CONTROL_CW_ENABLE;
    rds.WriteControlRegister(cr);
}

void carrier_off() {
    uint32_t cr = 0;
    rds.WriteControlRegister(cr);
}

void fm_rds_setup();
void fm_rds_loop();

void sw_cw_setup();
void sw_cw_loop();

void setup() {

    Serial.begin(BAUD_RATE);

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);     

    // Select which setup for which radio operating mode to use

    // FM Radio Data System (RDS) setup.
    //fm_rds_setup();

    // Shortwave CW (Continuous Wave or morse code) setup.
    sw_cw_setup();
}

void loop() {

   // FM Radio Data System (RDS) setup.
   //fm_rds_loop();

   // Shortwave CW (Continuous Wave or morse code) setup.
   sw_cw_loop();
}

void sw_cw_setup() {

  uint32_t frequency;

  frequency = SHORTWAVE_CW_FREQUENCY;

  // Set carrier frequency in Hertz
  rds.Hz(frequency);

  //carrier_on();
}

void sw_cw_loop() {
  static uint8_t loopCount;

  // Enable the output
  carrier_on();
  digitalWrite(led, HIGH);

  // print actual status on serial
  Serial.print("Carrier ON count : ");
  Serial.println(loopCount, HEX);

  delay(2000); // wait 2 seconds

  // Disable the output
  carrier_off();
  digitalWrite(led, LOW);

  Serial.print("Carrier OFF count : ");
  Serial.println(loopCount, HEX);

  delay(2000); // wait 2 seconds

  loopCount++;
}

//
// Setup to transmit FM RDS data.
//
void fm_rds_setup() {
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

void fm_rds_loop()
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
