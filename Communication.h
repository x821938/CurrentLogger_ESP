#ifndef _COMMUNICATION_h
#define _COMMUNICATION_h

#include <WiFiUdp.h>



class Communication
{
 protected:
	 WiFiUDP Udp;
	 char incomingPacket[255];

	 void setTrigger();
	 void setConfig();

 public:
	 void setup();
	 void sendall( unsigned long timeStamp, uint16_t i2cReadings, float voltage, float current, float power, double mAh, double mWh );
	 void checkUdp();
};


#endif