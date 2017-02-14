#ifndef _ViziblesATClient_h
#define _ViziblesATClient_h

//Uncomment the next line for debug output to serial port
//#define VZ_AT_DEBUG
//Uncomment the next line to use AltSoftSerial library instead of SoftwareSerial
//#define ESP8266_USE_ALTSOFTSERIAL

#ifndef ESP8266_USE_ALTSOFTSERIAL
#include <SoftwareSerial.h>
#else
#include <AltSoftSerial.h>
#endif

#define MAX_EXPOSED_FUNCTIONS 				5
#define MAX_EXPOSED_FUNCTIONS_NAME_BUFFER	127

#define CR 	13
#define LF	10

typedef void function_t (const char *parameters);	/*!< [in] Function callback type with option for handling parameters as char array.*/
typedef void (*cloudCallback)(void); 
	
typedef struct {
	char* 		functionId;
	function_t* handler;
} functions_t;

const char ATUpdate[] PROGMEM = { "AT+UPDATE=" };
const char ATSetOptions[] PROGMEM = { "AT+SETOPTIONS=" };
const char ATConnect[] PROGMEM = { "AT+CONNECT" };
const char ATWifiConnect[] PROGMEM = "AT+WIFICONNECT=";
const char ATExpose[] PROGMEM = { "AT+EXPOSE=\"" };
const char ATEndLine[] PROGMEM = { "\r\n" };
const char ATThingDo[] PROGMEM = { "THINGDO=\"" };
const char ATError[] PROGMEM = { "ERROR" };
const char ATOk[] PROGMEM = { "OK" };
const char ATConnected[] PROGMEM = { "CONNECTED" };
const char ATDisconnected[] PROGMEM = { "DISCONNECTED" };
const char ATReady[] PROGMEM = { "VZ-READY>"};
const char ATMac[] PROGMEM = { "+MAC="};

unsigned long elapsedMillis(unsigned long m);

class ViziblesArduinoAT: public Print {

  public:
#ifndef ESP8266_USE_ALTSOFTSERIAL
	ViziblesArduinoAT(int serialTxPin, int serialRxPin, int resetPin, int CH_PDPin);
#else
	ViziblesArduinoAT(int resetPin, int CH_PDPin);
#endif
    void startSerialInterface() { startSerialInterface(9600); };
    void startSerialInterface(unsigned long baudRate);
    void stopSerialInterface();
	void  ESP8266Reset(void);

	
	void print_P(const __FlashStringHelper *str);
	void println_P(const __FlashStringHelper *str);
	void print_P(PGM_P str); 
	void println_P(PGM_P str);
	//To comply with print interface
	virtual size_t write(uint8_t);
	virtual size_t write(uint8_t *, int len);
	
	void process(void);
	int update(const char *params = NULL);
	int update(const __FlashStringHelper *params);
	int connect(const char *params = NULL);
	int connect(const __FlashStringHelper *params);
	int connectWiFi(const char *params);
	int connectWiFi(const __FlashStringHelper *params);
	int expose(char *functionId, function_t *function);
	int setOptions(const char *params);
	int setOptions(const __FlashStringHelper *params);
	char *getMac(char *buf);
	int endCommand(void);
	
	void setErrorCb(cloudCallback cb);
	void setResetCb(cloudCallback cb);
	void setConnectedCb(cloudCallback cb);
	void setDisconnectedCb(cloudCallback cb);
	
  private:
#ifndef ESP8266_USE_ALTSOFTSERIAL
	SoftwareSerial *ESP8266Serial;
#else
	AltSoftSerial *ESP8266Serial;
#endif	
	bool serialStarted; //!< Serial interface started flag
	int ESP8266ResetPin; //!< Number of the Arduino pin to which ESP8266 Reset is connected
	int ESP8266CH_PDPin; //!< Number of the Arduino pin to which ESP8266 CH_PD is connected
	
	cloudCallback errorCb;
	cloudCallback connectedCb;
	cloudCallback disconnectedCb;
	cloudCallback resetCb;
	
	functions_t functions[MAX_EXPOSED_FUNCTIONS]; 					
	unsigned char exposed; /*!< Number of functions available for remote calls.*/	
	char functionNames[MAX_EXPOSED_FUNCTIONS_NAME_BUFFER];
	int functionNamesIndex;

	int waitForAnswer(char *buf=NULL);	
	int parseCommands(char *buf=NULL);

};

#endif /*_ViziblesArduinoAT_h*/
