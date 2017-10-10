/* INA219 Current Wifi Logger by Alex Skov Jensen
   Please adjust settings in setup.h before compiling */

#include "Setup.h"
#include "PowerSensor.h"
#include "Communication.h"
#include "WString.h"
#include <ESP8266WiFi.h>


Communication com;
PowerSensor powerSensor;



void setup()
{
	Serial.begin( 115200 );

	com.setup();
	powerSensor.setup();

	Serial.println( "Connect wifi" );
	WiFi.begin( WIFI_SSID, WIFI_PASSWD );

	while ( WiFi.status() != WL_CONNECTED ) { // Only go on if we are connected to wifi
		delay( 100 );
		Serial.print( "." );
	}
}


void loop() {
	com.checkUdp();
	powerSensor.handle();
}