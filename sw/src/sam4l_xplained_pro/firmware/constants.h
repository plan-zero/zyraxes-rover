#ifndef CONSTANTS_H
#define CONSTANTS_H


#define iMAX 1.0             // Be careful adjusting this.  While the A4954 driver is rated for 2.0 Amp peak currents, it cannot handle these currents continuously.  Depending on how you operate the Mechaduino, you may be able to safely raise this value...please refer to the A4954 datasheet for more info
#define rSense 0.150
#define spr 200

extern volatile int uMAX;


#endif /*CONSTANTS_H*/