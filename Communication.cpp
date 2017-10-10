#include "Communication.h"
#include "PowerSensor.h"
#include "Setup.h"
#include "External\StringSplitter.h"
#include "Logging.h"
#include <WiFiUdp.h>
#include <Arduino.h>


extern PowerSensor powerSensor;



/* Start listening for incoming UDP packet on port xxx */
void Communication::setup( ) {
	Udp.begin( LISTEN_PORT );
}



/* Sends an UDP packet to remote client in this format:
   "timeInMs:i2cReadings:Voltage:Current:Power:mAh:mWh" */
void Communication::sendall( unsigned long timeStamp, uint16_t i2cReadings, float voltage, float current, float power, double mAh, double mWh ) {
	static char timeTxt[12];
	static char i2cReadingsTxt[6];
	static char voltageTxt[10];
	static char currentTxt[10];
	static char powerTxt[10];
	static char mAhTxt[10];
	static char mWhTxt[10];

	// Convert all those provided numbers into strings
	sprintf( timeTxt, "%d", timeStamp );
	sprintf( i2cReadingsTxt, "%d", i2cReadings );
	dtostrf( voltage, 0, 3, voltageTxt );
	dtostrf( current, 0, 3, currentTxt );
	dtostrf( power, 0, 3, powerTxt );
	dtostrf( mAh, 0, 3, mAhTxt );
	dtostrf( mWh, 0, 3, mWhTxt );

	// And send it
	Udp.beginPacket( REMOTE_IP, REMOTE_PORT );
	Udp.write( timeTxt );
	Udp.write( ':' );
	Udp.write( i2cReadingsTxt );
	Udp.write( ':' );
	Udp.write( voltageTxt );
	Udp.write( ':' );
	Udp.write( currentTxt );
	Udp.write( ':' );
	Udp.write( powerTxt );
	Udp.write( ':' );
	Udp.write( mAhTxt );
	Udp.write( ':' );
	Udp.write( mWhTxt );
	Udp.endPacket();

	LOG_DEBUG( "COM", timeTxt << ":" << i2cReadingsTxt << ":" << voltageTxt << ":" << currentTxt << ":" << powerTxt << ":" << mAhTxt << ":" << mWhTxt );
}



/* Checks if there is an incoming UDP packet. If so the first character is checked:
   C=Configuration, A=Start sampling, O=Stop sampling, T=Setup trigger */
void Communication::checkUdp() {
	int packetSize = Udp.parsePacket();
	if ( packetSize ) {
		int len = Udp.read( incomingPacket, 255 );
		if ( len > 0 ) {
			LOG_DEBUG( "COM", "Received " << packetSize << " bytes from " << Udp.remoteIP().toString() << ", port " << Udp.remotePort() );
			LOG_INFO( "COM", "UDP packet contents: " << incomingPacket );
			incomingPacket[len] = 0; // Null terminate the incoming string
			switch ( incomingPacket[0] ) {
				case 'C':
					setConfig();
					break;
				case 'A':
					powerSensor.startSampling();
					break;
				case 'O':
					powerSensor.stopSampling();
					break;
				case 'T':
					setTrigger();
					break;
			}
		}
	}
}



/* Handle a trigger setup packet that has this format:
   "T:triggerSource:triggerOn:triggerValue"  */
void Communication::setTrigger() {
	int parameterCount;
	StringSplitter *splitter = new StringSplitter( incomingPacket, ':', 20 );
	parameterCount = splitter->getItemCount();
	LOG_DEBUG( "COM", "parameters=" << parameterCount );
	if ( parameterCount == 4 ) {
		char triggerSource = splitter->getItemAtIndex( 1 )[0];
		char triggerOn = splitter->getItemAtIndex( 2 )[0];
		float triggerValue = splitter->getItemAtIndex( 3 ).toFloat();
		LOG_DEBUG( "COM", "Trigger source=" << triggerSource << ". Trigger on " << triggerOn << ". Trigger Value " << triggerValue );

		powerSensor.configureTrigger( triggerSource, triggerOn, triggerValue );
	}
}



/* Handle a configuration packet that has this format:
   "C:range:gain:adcBus:adcShunt:rshunt" */
void Communication::setConfig() {
	int parameterCount;
	StringSplitter *splitter = new StringSplitter( incomingPacket, ':', 20 );
	parameterCount = splitter->getItemCount();
	LOG_DEBUG( "COM", "parameters=" << parameterCount );
	if ( parameterCount == 6 ) {
		uint8_t range = splitter->getItemAtIndex( 1 ).toInt();
		uint8_t gain = splitter->getItemAtIndex( 2 ).toInt();
		uint8_t adcBus = splitter->getItemAtIndex( 3 ).toInt();
		uint8_t adcShunt = splitter->getItemAtIndex( 4 ).toInt();
		float rshunt = splitter->getItemAtIndex( 5 ).toFloat();

		LOG_DEBUG( "COM", "range=" << range << " gain=" << gain << " adcbus=" << adcBus << " adcshunt=" << adcShunt <<" rshunt=" << rshunt );

		powerSensor.configure( range, gain, adcBus, adcShunt, rshunt );
	}
	delete( splitter );
}