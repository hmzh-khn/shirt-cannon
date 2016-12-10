#include <stdio.h>
#include <stdlib.h>
#include "easyPIO.h"

#define AZI_DONE_PIN 16
#define POL_DONE_PIN 21
#define COMP_DONE_PIN 18
#define LOAD_PIN 23

void main(void) {
  printf("Content-Type: text/html;charset=s-ascii\n\n");
  printf("<META HTTP-EQUIV=\"Refresh\" CONTENT=\"3;url=/cannonCtrl.html\">\n");

  unsigned int x, y, f;
  char azimuthal, polar, force;

  char *data;
  data = getenv("QUERY_STRING");
  
  if(data == NULL) {
    printf("<P>Error! Error in passing data from form to script.");
  }
  else if(sscanf(data,"x=%ld&y=%ld&f=%ld",&x,&y,&f)!=3) {
    printf("<P>Error! Invalid data. Data must be numeric.");
  }
  else {
    pioInit();
    spiInit(244000, 0);

    // Load and done pins
    pinMode(LOAD_PIN, OUTPUT);
    pinMode(AZI_DONE_PIN, INPUT);
    pinMode(POL_DONE_PIN, INPUT);
    pinMode(COMP_DONE_PIN, OUTPUT);

    azimuthal = (char) x;
    polar = (char) y;
    force = (char) f;

    digitalWrite(LOAD_PIN, 0);
    digitalWrite(COMP_DONE_PIN, 1);

    int output = 0;
    output = (output << 8) + spiSendReceive(azimuthal);
    output = (output << 8) + spiSendReceive(polar);
    output = (output << 8) + spiSendReceive(force);

    // allow compressor to fill tank
    delayMillis(839*f);
    digitalWrite(LOAD_PIN, 1);
    digitalWrite(COMP_DONE_PIN, 0);
  }
}
