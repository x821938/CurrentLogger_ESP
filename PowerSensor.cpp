#include "PowerSensor.h"
#include "External\INA219.h"
#include "Communication.h"
#include "Logging.h"


extern Communication com;



/* Starts up the INA219 sensor and sends a standard configuration:
   Range=16V, Shunt voltage sensitivity = 40mV, Bus ADC = 84us/9bit, Shunt ADC = 68.10ms 128samples, Shunt resistor = 0.1 Ohm 
   This should suit most situations, and this is what the windows client expects in the begining */
void PowerSensor::setup() {
	Wire.begin();

	monitor.begin();
	configure( 0, 0, 0, 15, 0.1 );
}



/* Should be called from main loop.
   Constantly polls the sensor and sends it to client pc if a trigger condition is met */
void PowerSensor::handle() {
	if ( sampling ) {
		uint16_t voltageRaw;
		bool newConversionReady;
		uint16_t readings = 0;

		do { // We keep asking the sensor until it gives us a new reading
			voltageRaw = monitor.busVoltageRaw();
			newConversionReady = bitRead( voltageRaw, 1 ); // Bit 1 in bus voltage setindicates that INA219 has new reading for us
			readings++; // How many times we ask sensor without getting a new value. Under 2 means that we are loosing values.
		} while ( !newConversionReady );

		// Get the rest of values from sensor
		float voltage = monitor.busVoltage( voltageRaw );
		float current = monitor.shuntCurrent() * 1000;
		float power = monitor.busPower() * 1000; // Reading the power clears the INA219 new reading flag.

		// Accumulated current in mAh and power in mWh
		mAh += ( current * sampletime[busAdc] + current * sampletime[shuntAdc] ) / ( 1000000UL *3600 );
		mWh += ( power * sampletime[busAdc] + power * sampletime[shuntAdc] ) / ( 1000000UL * 3600 );

		if ( isTriggered( voltage, current, power ) ) { // We only send measurements to client if our trigger condition was met
			unsigned long timeStamp = millis() - samplingStarted; // All times are relative to the time we triggered
			com.sendall( timeStamp, readings, voltage, current, power, mAh, mWh );
		}
	}
}



/* This method is given a reading from the INA sensor (voltage, current and power). 
   If we are not yet triggered, it first looks at which of the three we are triggering on.
   Now it is held against the triggervalue. If we conclude that we are triggered we return true otherwise false 
   Also if triggered it resets the mAh/mWh accumulations because we are starting a complete new measurement cycle */
bool PowerSensor::isTriggered( float voltage, float current, float power ) {
	if ( !triggered ) {
		switch ( triggerSource ) {
			case 'N': // Trigger on nothing, meaning that the user just want to manually start a maeasurent cycle.
				triggered = true;
				break;
			case 'C': // Trigger on Current
				if ( triggerOn == 'G' && current > triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because current > " << triggerValue << "mA");
					triggered = true;
				}
				if ( triggerOn == 'L' && current < triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because current < " << triggerValue << "mA" );
					triggered = true;
				}
				break;
			case 'V': // Trigger on Voltage
				if ( triggerOn == 'G' && voltage > triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because voltage > " << triggerValue << "V" );
					triggered = true;
				}
				if ( triggerOn == 'L' && voltage < triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because voltage < " << triggerValue << "V" );
					triggered = true;
				}
				break;
			case 'P': // Trigger on Power
				if ( triggerOn == 'G' && power > triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because power > " << triggerValue << "mW" );
					triggered = true;
				}
				if ( triggerOn == 'L' && power < triggerValue ) {
					LOG_NOTICE( "INA", "Triggered because power < " << triggerValue << "mW" );
					triggered = true;
				}
				break;
		}
		if ( triggered ) { // Start a new measurement cycle. Reset the accumulated mAh/mWh
			mAh = 0;
			mWh = 0;
			samplingStarted = millis(); // Save the time we were triggered
		}
	}
	return triggered;
}



/* Sets up what to trigger on. 
   triggerSource can be (N=Nothing, C=Current, V=Voltage, P=Power 
   triggerOn can be (L=Less than, G=Greater than)
   triggerValue is what the measurents should be compared against
   Tells the class that it should start watching the measurents from now on for trigger condition */
void PowerSensor::configureTrigger( char triggerSource, char triggerOn, float triggerValue ) {
	this->triggerSource = triggerSource;
	this->triggerOn = triggerOn;
	this->triggerValue = triggerValue;
	sampling = true;
	triggered = false;
}



/* Sends a configuration to the INA219 sensor */
void PowerSensor::configure( uint8_t range, uint8_t gain, uint8_t busAdc, uint8_t shuntAdc, float shuntResistor ) {
	this->shuntAdc = shuntAdc;
	this->busAdc = busAdc;

	monitor.configure( ( INA219::t_range ) range, ( INA219::t_gain ) gain, ( INA219::t_adc ) busAdc, ( INA219::t_adc ) shuntAdc );

	uint8_t vBusMax = (range == 0) ? 16 : 32;
	float vShuntMax = gains[gain];

	monitor.calibrate( shuntResistor, vShuntMax, vBusMax, vShuntMax/shuntResistor );
	stopSampling();
}



/* Sets up that a sampling should be started immediately */
void PowerSensor::startSampling() {
	triggerSource = 'N'; // None - manual trigger
	sampling = true;
	triggered = false;
}



/* Stop sending samples to client */
void PowerSensor::stopSampling() {
	sampling = false;
	triggered = false;
}