/*
* ViziblesArduinoAT.cpp 
* Version 1.0 Aug, 2016
* Copyright 2015 Pablo Rodiz Obaya
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

#include "ViziblesArduinoAT.h"
#include <Arduino.h>

unsigned long elapsedMillis(
		unsigned long m /*!< [in] Start time in millis. */) {
	unsigned long now = millis();
	if (now>=m) return (now-m);
	else return ((0xFFFFFFFF - m) + now);
}

/** 
 *	@brief Create ViziblesArduinoAT object.
 * 	
 *	Constructor for the ViziblesArduinoAT class.  
 */
#ifndef ESP8266_USE_ALTSOFTSERIAL
ViziblesArduinoAT::ViziblesArduinoAT(int serialTxPin, int serialRxPin, const int resetPin, const int CH_PDPin) {
#else
ViziblesArduinoAT::ViziblesArduinoAT(const int resetPin, const int CH_PDPin) {
#endif
#ifdef ESP8266_DEBUG
  //Serial.print(F("Setting up ESP8266 module"));
#endif
	serialStarted = false;
	errorCb = NULL;
	connectedCb = NULL;
	disconnectedCb = NULL;
	resetCb = NULL;
	ESP8266ResetPin = resetPin;
	ESP8266CH_PDPin = CH_PDPin;
	exposed = 0;
	for (int i = 0; i < MAX_EXPOSED_FUNCTIONS; i++) {
		functions[i].functionId = NULL;
		functions[i].handler = NULL;
	}
	//functionNames[MAX_EXPOSED_FUNCTIONS_NAME_BUFFER];
	functionNamesIndex = 0;
#ifndef ESP8266_USE_ALTSOFTSERIAL
	ESP8266Serial = new SoftwareSerial(serialTxPin, serialRxPin);
#else
	ESP8266Serial = new AltSoftSerial;
#endif
	pinMode(ESP8266CH_PDPin, OUTPUT);
	digitalWrite(ESP8266CH_PDPin, HIGH); 
	pinMode(ESP8266ResetPin, OUTPUT);
	digitalWrite(ESP8266ResetPin, HIGH);
	serialStarted = false;  

}

/** 
 *	@brief Start serial interface
 * 	
 *	Initialize serail communications with the ESP8266 module
 */
void ViziblesArduinoAT::startSerialInterface(
		unsigned long baudRate /*!< [in] (optional) Baud rate of the serial port in bauds per second. 9600 by default*/) {
  ESP8266Serial->begin(baudRate);
  //while (!ESP8266Serial) {
  //  ; // wait for serial port to connect. Needed for Leonardo only
  //}
  ESP8266Serial->write(CR);
  ESP8266Serial->write(LF);
  ESP8266Serial->flush();
  while (ESP8266Serial->available() > 0) {
    ESP8266Serial->read();
  }
  serialStarted = true;
}

/** 
 *	@brief Stop serial interface.
 * 	
 *	Stop serial communications with the ESP8266 module.
 */
void ViziblesArduinoAT::stopSerialInterface() {
  ESP8266Serial->flush();
  while (ESP8266Serial->available() > 0) {
    ESP8266Serial->read();
  }
  serialStarted = false;
  ESP8266Serial->end();
}

/** 
 *	@brief Reset ESP8266 module.
 * 	
 *	Performs hardware reset of the module and perform basic device configuration. 
 *	Or more complex application specific configuration if configuration callback is defined.
 */
void ViziblesArduinoAT::ESP8266Reset(void) {
	//Hard reset module
	digitalWrite(ESP8266CH_PDPin, HIGH); 
	digitalWrite(ESP8266ResetPin, LOW);
	delay(500);
	digitalWrite(ESP8266ResetPin, HIGH);
}

int ViziblesArduinoAT::parseCommands(
									 char *buf /*!< [in] (Optional) Buffer pointer for storing answer. */){
	int i = 0;
	int maxI = 0;
	char *buffer;
	if(buf==NULL){
			char b[200];
			buffer = b;
			maxI = 200;
	} else {
			buffer = buf;
			maxI = 256;
		}
	char h = '+';
	while (i<maxI && h!='\n') {
		if(ESP8266Serial->available()) {
			h = ESP8266Serial->read();
#ifdef VZ_AT_DEBUG
			Serial.print(h);
#endif			
			buffer[i++] = h;
		} else delay(10);	
	}
	buffer[i] = '\0';
	int thingDoLen = strlen_P(ATThingDo);
	if(!strncmp_P(buffer, ATThingDo, thingDoLen)){
		int messageLen = i;
		int fidLen = 0;
		for(int j = thingDoLen; j<messageLen; j++) {
			if(buffer[j] == '"') {
				fidLen = j-thingDoLen;
				break;
			} 
		}
		for (int j=0; j<exposed; j++) {
			if(!strncmp(&buffer[thingDoLen], functions[j].functionId, fidLen-1)) {
				if(functions[j].handler != NULL) functions[j].handler(buffer[thingDoLen]);
				break;
			}
		}
	}
	if(!strncmp_P(buffer, ATMac, strlen_P(ATMac))){
		return 1;
	}
	if(!strncmp_P(buffer, ATError, strlen_P(ATError))){
		if(errorCb!=NULL) errorCb();
		return 0;
	}				
	if(!strncmp_P(buffer, ATConnected, strlen_P(ATConnected))){
		if(connectedCb!=NULL) connectedCb();
		return 0;
	}				
	if(!strncmp_P(buffer, ATDisconnected, strlen_P(ATDisconnected))){
		if(disconnectedCb!=NULL) disconnectedCb();
		return 0;
	}
	if(!strncmp_P(buffer, ATReady, strlen_P(ATReady))){
		if(resetCb!=NULL) resetCb();
		return 0;
	}
	return 0;	
	
}

/** 
 *	@brief Process ESP8266 module output.
 * 	
 *	Reads incomming mssages from ESP8266 module and looks for commands. 
 */
void ViziblesArduinoAT::process(void){
    if (serialStarted && ESP8266Serial->available()) {
		while (ESP8266Serial->available()) {
			char c = ESP8266Serial->read();
#ifdef VZ_AT_DEBUG
			Serial.print(c);
#endif
			if(c=='+') {
				parseCommands();
			}
		}
	}
	
}

void ViziblesArduinoAT::print_P(const __FlashStringHelper *str) {
	PGM_P fp = reinterpret_cast<PGM_P>(str);
	for (uint8_t c; (c = pgm_read_byte(fp)); fp++) write(c);
}

void ViziblesArduinoAT::println_P(const __FlashStringHelper *str) {
	print_P(str);
	print_P((const __FlashStringHelper *)ATEndLine);
}

void ViziblesArduinoAT::print_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) write(c);
}

void ViziblesArduinoAT::println_P(PGM_P str) {
  print_P(str);
  print_P(ATEndLine);
}


size_t ViziblesArduinoAT::write(uint8_t t) {
#ifdef VZ_AT_DEBUG
	Serial.print((char)t);
#endif
	return ESP8266Serial->write(t);
}	

size_t ViziblesArduinoAT::write(uint8_t *b, int len){
#ifdef VZ_AT_DEBUG
	Serial.print((char *)*b);
#endif
	return ESP8266Serial->write(b, len);
}	


/** 
 *	@brief Waits for the answer of a command.
 * 	
 *	Reads incomming mssages from ESP8266 module and looks for command answers. 
 *
 *	@return 0 on error or 1 if everything was OK
 */

int ViziblesArduinoAT::waitForAnswer(
									char *buf /*!< [in] (Optional) Buffer pointer for storing answer. */){
	char buffer[30];
	int i = 0;
	unsigned long timeout = millis();
	while (i<30 && elapsedMillis(timeout)<5000) {
		if(ESP8266Serial->available()){
			buffer[i++] = ESP8266Serial->read();
#ifdef VZ_AT_DEBUG
			Serial.print(buffer[i-1]);
#endif
			buffer[i]='/0';
			//Serial.println(buffer);
			if(buffer[i-1]=='+') {
				if(buf!=NULL) {
					while(parseCommands(buf)==0){};
					return 1;
				}
				else parseCommands();
				i--;
				buffer[i]='/0';
			}
			if(i>2){
				if(!strncmp_P(&buffer[i-5], ATError, strlen_P(ATError))) return 0;
				else if(!strncmp_P(&buffer[i-2], ATOk, strlen_P(ATOk))) return 1;	
			}
		}	
	}
	return 0;
}





int ViziblesArduinoAT::update(const char *params) {
	if (serialStarted) {
		print_P(ATUpdate);
		if(params) {
			println(params);
			return waitForAnswer();
		} else return 0;		
	} else return 0;
}

int ViziblesArduinoAT::update(const __FlashStringHelper *params) {
	if (serialStarted) {
		print_P(ATUpdate);
		if(params) { 
			println_P(params);
			return waitForAnswer();
		} else return 0;		
	} else return 0;
}

int ViziblesArduinoAT::connect(const char *params) {
	if (serialStarted) {
		print_P(ATConnect);
		if(params) {
			print('=');
			println(params);
			return waitForAnswer();
		} else return 0;		
	} else return 0;
}

int ViziblesArduinoAT::connect(const __FlashStringHelper *params) {
	if (serialStarted) {
		print_P(ATConnect);
		if(params) {
			print('=');
			println_P(params);
			return waitForAnswer();
		} else return 0;		
	} else return 0;
}

int ViziblesArduinoAT::connectWiFi(const char *params) {
	if (serialStarted) {
		print_P(ATWifiConnect);
		println(params);
		return waitForAnswer();
	} else return 0;
}

int ViziblesArduinoAT::connectWiFi(const __FlashStringHelper *params) {
	if (serialStarted) {
		print_P(ATWifiConnect);
		println_P(params);
		return waitForAnswer();
	} else return 0;
}
/** 
 *	@brief Expose a function call.
 * 	
 *	This function enables calling an internal function of this thing from the
 *	Vizibles cloud or from another thing in the local network.
 * 
 *	@return 0 on error or 1 if everything was OK
 */
int ViziblesArduinoAT::expose(
		char *functionId, 		/*!< [in] Name that will reference to the exposed function.*/
		function_t *function	/*!< [in] Function pointer to call the functionality.*/){
	//Check if function ID is already known
	int found = 0;
	for (int i=0; i<exposed; i++) {
		if(!strcmp(functionId, functions[i].functionId)) {
			found = 1;
		}
	}
	//Add function if new
	if(!found) {
		if(exposed<MAX_EXPOSED_FUNCTIONS && (functionNamesIndex+strlen(functionId)+1<MAX_EXPOSED_FUNCTIONS_NAME_BUFFER)) {
			//Add function to exposed functions list
			exposed++;
			functions[exposed-1].functionId = &functionNames[functionNamesIndex];
			strcpy(&functionNames[functionNamesIndex], functionId);
			functionNamesIndex += strlen(functionId)+1;
			//functions[exposed-1].functionId = functionId;
			functions[exposed-1].handler = function;
			//Create parameters for sendFunctions
		} else return 0;
	}	
	
	print_P(ATExpose);
	print(functionId);
	println_P(F("\""));
	return waitForAnswer();
}

int ViziblesArduinoAT::setOptions(const char *params) {
	if (serialStarted) {
		print_P(ATSetOptions);
		if(params) {
			println(params);
			return waitForAnswer();
		} else return 0;	
	} else return 0;
}

int ViziblesArduinoAT::setOptions(const __FlashStringHelper *params) {
	if (serialStarted) {
		print_P(ATSetOptions);
		if(params) {
			println_P(params);
			return waitForAnswer();
		} else return 0;	
	} else return 0;
}

int ViziblesArduinoAT::endCommand(void) {
	if (serialStarted) {
		write(CR);
		write(LF);
		return waitForAnswer();
	} else return 0;
}

char *ViziblesArduinoAT::getMac(char *buf) {
	delay(100);
	print_P(F("AT+GETMAC"));
	write(CR);
	write(LF);
	if(waitForAnswer(buf)==1) return &buf[5];
	else return NULL;	
}
void ViziblesArduinoAT::setErrorCb(cloudCallback cb) {
	errorCb = cb;
}	

void ViziblesArduinoAT::setResetCb(cloudCallback cb){
	resetCb = cb;
}
	
void ViziblesArduinoAT::setConnectedCb(cloudCallback cb){
	connectedCb = cb;
}

void ViziblesArduinoAT::setDisconnectedCb(cloudCallback cb){
	disconnectedCb = cb;
}



