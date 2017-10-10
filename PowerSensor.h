#ifndef _POWERSENSOR_h
#define _POWERSENSOR_h

#include <Arduino.h>
#include "External\INA219.h"



class PowerSensor
{
 protected:
	 INA219 monitor;

	 const float gains[4] = { 0.040, 0.080, 0.160, 0.320 }; // in mAh
	 const double sampletime[16] = { 84, 148, 276, 532, 0, 0, 0, 0, 0, 1060, 2130, 4260, 8510, 17020, 34050, 68100 }; // in microseconds

	 bool sampling = false;
	 unsigned long samplingStarted;

	 uint8_t busAdc;
	 uint8_t shuntAdc;

	 double mAh = 0;
	 double mWh = 0;

	 char triggered = false;
	 float triggerValue;
	 char triggerOn;
	 char triggerSource;

	 bool isTriggered( float voltage, float current, float power );

 public:
	 void setup();
	 void handle();
	 void configure( uint8_t range, uint8_t gain, uint8_t busAdc, uint8_t shuntAdc, float shuntResistor );
	 void configureTrigger( char triggerSource, char triggerOn, float triggerValue );
	 void startSampling();
	 void stopSampling();
};


#endif