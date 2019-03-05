
/*
  03/03/2019

  Menlopark Innovation LLC

  Software Defined Radio (SDR) created from FM RDS core.
*/

/*
This is a test program that dynamically updates RDS message.
AUTHOR=EMARD
LICENSE=GPL
*/

#include <SDR.h>

// RDS is an operating mode of SDR for FM Radio Data System (RDS) display.
RDS rds = RDS();

uint16_t pi = 0xCAFE;
char ps[9] = "TEST1234";
char rt[65] = "ABCDEFGH";

void setup() {
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

  // Shortwave experiment
  // Still send FM modulated RDS and see what it looks/hears like.
  // This works. 8.00Mhz signal with "noise" from RDS.
  // 03/04/2019
  rds.Hz(8000000); // Hz carrier wave frequency

  // Note: My MAX10 FPGA generates a 0.5Mhz lower frequency.
  //rds.Hz(107900000); // Hz carrier wave frequency

  //rds.Hz(87500000); // Hz carrier wave frequency

  rds.length(260); // bytes message length (260 default)
  Serial.begin(115200);
}

void loop()
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
