/*
 * Domeduino.ino 
 * Version 1.0 December, 2016
 * Copyright 2016 Enxine Dev S.C.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * The latest version of this library can always be found at
 *
 * https://github.com/Enxine/ViziblesArduinoAT
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <SoftwareSerial.h> 
#include <AltSoftSerial.h>
#include <ViziblesArduinoAT.h>
#include <EEPROM.h>
#include <PermanentRelay.h>
#include <PowerMeter78M6613.h>
#include <avr/pgmspace.h>

#define PIN_RELAY 8 

//Power Meter 78M6613 serial connection pins
#define PM78M6613_TX_PIN 6
#define PM78M6613_RX_PIN 7

//ESP8266 WiFi control pins
#define ESP8266_CH_PD_PIN 12
#define ESP8266_RESET_PIN 9
#define ESP8266_TX_PIN 3
#define ESP8266_RX_PIN 4

#define _DEBUG true
 
#define EEPROM_CONFIG_START 32 
 
#ifndef ESP8266_USE_ALTSOFTSERIAL
ViziblesArduinoAT cloud = ViziblesArduinoAT(ESP8266_TX_PIN, ESP8266_RX_PIN, ESP8266_RESET_PIN, ESP8266_CH_PD_PIN); 
#else
ViziblesArduinoAT cloud = ViziblesArduinoAT(ESP8266_RESET_PIN, ESP8266_CH_PD_PIN); 
#endif
PowerMeter78M6613 myPowerMeter = PowerMeter78M6613(PM78M6613_TX_PIN, PM78M6613_RX_PIN);
PermanentRelay Relay1 = PermanentRelay(PIN_RELAY, EEPROM_CONFIG_START);

unsigned long measure_interval = 10000; 
unsigned long previous_measurement = 0;
 
unsigned long previousMillis_report = 1;
unsigned long previousMillis_read = 0;
unsigned long interval_report = 600000;
unsigned long lastResetMillis = 0;
unsigned long lastAcceleration = 0;

long power = 0;
long voltage = 0;
long current = 0;
long apparent_power = 0;
long power_factor = 0;
long last_power = 0;
long last_voltage = 0;
long last_current = 0;
long last_apparent_power = 0;
long last_power_factor = 0;

//******************** 
// Relay stuff   
//********************                   

void switchOn (const char *) {
	Serial.println(F("SwitchOn"));
	Relay1.setSwitchON();
	previous_measurement = millis()-measure_interval;
	previousMillis_report = millis()-interval_report+10;;
}  
void switchOff (const char *) {
	Serial.println(F("SwitchOff"));
	Relay1.setSwitchOFF();
	previous_measurement = millis()-measure_interval;
	previousMillis_report = millis()-interval_report+10;;
}  


//******************** 
// Power meter stuff   
//********************                   

void measurePower() {
	cloud.stopSerialInterface();
	myPowerMeter.startSerialInterface();
		
	last_voltage = voltage;
	voltage = myPowerMeter.readVoltage();
	last_current = current;
	current = myPowerMeter.readCurrent();
	last_power = power;
	power = myPowerMeter.readPower();
	if(!((last_power + last_power*0.15 )> power && (last_power - last_power*0.15 ) < power)) {
		lastAcceleration = millis();
		measure_interval = 1000;
		previousMillis_report = millis()-interval_report-1;
		interval_report = 3000;
	}
	last_apparent_power = apparent_power;
	apparent_power = myPowerMeter.readApparentPower();
	last_power_factor = power_factor;
	power_factor = myPowerMeter.readPowerFactor();

	myPowerMeter.stopSerialInterface();
	cloud.startSerialInterface();
}

//****************
// Arduino stuff
//**************** 

const char vz_id[] PROGMEM = "\"id\",\"domeduino-";
const char vz_options[] PROGMEM = "\",\"type\",\"enx-dd-01\",\"keyID\",\"Gp2naLrsSpFE\",\"keySecret\",\"wGyFTwIHvYwGCBDJyA7j\"";
 
int ready = 0;
int connected = 0; 
int wifiOn = 0;
int macRead = 0;
char mac[25];
unsigned long connect_interval = 10000; 
unsigned long previous_connect = 0;
int exposed1 = 0;
int exposed2 = 0;
unsigned long expose_interval = 2000; 
unsigned long previous_expose = 0;
char first_update = 1;

void resetCallback(void) {
	Serial.println("ESP8266 Reset");
	ready = 1;
	connected = 0;
	exposed1 = 0;
	exposed2 = 0;
}

void errorCallback(void) {
	Serial.println("Error communicationg with the platform");
}
void connectCallback(void) {
	Serial.println("Connected to Vizibles");
	connected = 1;
	exposed1 = 0;
	exposed2 = 0;
}
void disconnectCallback(void) {
	Serial.println("Disconnected from Vizibles");
	connected = 0;
	exposed1 = 0;
	exposed2 = 0;
}

void setup() 
{  
	if (_DEBUG) Serial.begin(9600);
	
	cloud.setErrorCb(errorCallback);
	cloud.setConnectedCb(connectCallback);
	cloud.setDisconnectedCb(disconnectCallback);
	cloud.setResetCb(resetCallback);
	cloud.startSerialInterface();
}
 
void loop()
{
	cloud.process();
	if (!ready) {
		cloud.ESP8266Reset();
		delay(1000);
	}
	if (ready && !wifiOn) {
		cloud.connectWiFi(F("\"YOUR_WIFI_SSID_HERE\",\"YOUR_WIFI_PASSWORD_HERE\""));
		wifiOn=1;
	}
	if (ready && wifiOn && !macRead){
		macRead = 1;
		char *m = cloud.getMac(mac);
	}
  
	if(ready && macRead && !connected && (previous_connect>0?elapsedMillis(previous_connect)>connect_interval:1)) {
		char params[116];
		strcpy_P(params,vz_id);
		params[16] = mac[14];
		params[17] = mac[15];
		params[18] = mac[17];
		params[19] = mac[18];
		params[20] = mac[20];
		params[21] = mac[21];
		strcpy_P(&params[22], vz_options);
		cloud.connect(params);
		previous_connect = millis();
	}
  
	if(connected && !exposed1 && elapsedMillis(previous_expose)>expose_interval && elapsedMillis(previous_connect)>expose_interval) {
		exposed1 = cloud.expose("switchOn", switchOn);
		previous_expose = millis();
	}
  
	if(connected && exposed1 && !exposed2 && elapsedMillis(previous_expose)>expose_interval) {
		exposed2 = cloud.expose("switchOff", switchOff);
		previous_expose = millis();
	}
  
   
	if (exposed2 && elapsedMillis(lastAcceleration) > 240000) {
		measure_interval = 3333;
		interval_report = 60000;
	} else if (exposed2 && elapsedMillis(lastAcceleration) > 30000) {
		measure_interval = 3333;
		interval_report = 10000;
	}

	if (exposed2 && elapsedMillis(previous_measurement) > measure_interval) {
		measurePower(); 
		previous_measurement = millis();
	};  

	if (exposed2 && elapsedMillis(previousMillis_report) > interval_report) {
		if (_DEBUG) {Serial.println(F("Send power update to cloud"));};
		if(!first_update && elapsedMillis(lastAcceleration) < 200){
			cloud.update();
			cloud.print_P(F("\"power\",\""));
			cloud.print(last_power/(float)1000);
			cloud.print_P(F("\",\"apparent\",\""));
			cloud.print(last_apparent_power/(float)1000);
			cloud.print_P(F("\",\"factor\",\""));
			cloud.print((float)((float)last_power_factor/(float)1000));
			cloud.print_P(F("\",\"v\",\""));
			cloud.print(last_voltage/(float)1000);
			cloud.print_P(F("\",\"i\",\""));
			cloud.print(last_current/(float)1000);
			cloud.println_P(F("\""));
		}
		
		cloud.update();
		cloud.print_P(F("\"power\",\""));
		cloud.print(power/(float)1000);
		cloud.print_P(F("\",\"apparent\",\""));
		cloud.print(apparent_power/(float)1000);
		cloud.print_P(F("\",\"factor\",\""));
		cloud.print((float)((float)power_factor/(float)1000));
		cloud.print_P(F("\",\"v\",\""));
		cloud.print(voltage/(float)1000);
		cloud.print_P(F("\",\"i\",\""));
		cloud.print(current/(float)1000);
		cloud.print_P(F("\",\"state\",\""));
		cloud.print(Relay1.getSwitchState());
		cloud.println_P(F("\""));
		first_update=0;
		
		previousMillis_report = millis();
	}
}
