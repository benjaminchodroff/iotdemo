#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <Energia.h>
#include <aJSON.h>
#include <ChainableLED.h>
#include <DHT.h>
#include <IPAddress.h>
#include <math.h>
#include <pins_energia.h>
#include <PubNub.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <TimerSerial.h>
#include <TM1637.h>
#include <utility/wl_definitions.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <Countdown.h>
#include <MQTTClient.h>
#include <MQTTConnect.h>
#include <MQTTPacket.h>
#include <stdio.h>
#include <WifiIPStack.h>
#include <WString.h>

void tachCount();
void setup();
void checkTemperatureHumidity();
void checkPushedButton(int DEVICE,volatile boolean* PUSHED_STATE);
void webClient();
void checkRotaryAngle();
void updateLCD();
void publishPubNub();
void checkTach();
void updateSpeed();
boolean endsWith(char* inString, char* compString);
void printWifiStatus();
void buttonPushedIgnition();
void buttonPushedBrakes();
void buttonPushedHeadlights();
void playTone(int _tone, int duration);
void playNote(char note, int duration);
void loop();

bool debug = false;
bool usePubNub = false;
bool useMQTT = true;
char version[] = "2.1.3";
char boardtype[] = "MSP430F5529";
/*
char ssid[] = "iscape2.4ghz";
char password[] = "J7bJpvPJ8rJzM";
*/
char ssid[] = "chodroff";
char password[] = "dummydummy";

#define MQTT_MAX_PACKET_SIZE 200
#define SIZE 200
int mqttPort = 1883;
char topic[] = "iot-2/evt/status/fmt/json";
char subTopic[] = "iot-2/cmd/+/fmt/json";
char organization[] = "5dj1dm";
char typeId[] = "launchpad";
char deviceId[] = "78a5040fad88";

// Authentication method. Should be use-toke-auth
// When using authenticated mode
char authMethod[] = "use-token-auth";
// The auth-token from the information above
char authToken[] = "Fr1+(n*(JPegi)Ktci";
// This string will be created in setup
char clientId[48];
// This string will be created in setup
char registeredUrl[SIZE];

// The function to call when a message arrives
void callback(char* topic, byte* payload, unsigned int length);
WifiIPStack ipstack;
MQTT::Client<WifiIPStack, Countdown, MQTT_MAX_PACKET_SIZE> client(ipstack);
void messageArrived(MQTT::MessageData& md);

const unsigned long tachCheckDelay = 1000; // How often to check the tach
int fanspeed = 255; // Start the fan at the slowest speed
const unsigned long rotaryDelay = 50; // How often to check the rotary sensor
const unsigned long debounceTime = 250;

#define TACH 39 //fan tachometer
#define FANPWM 40 //fan pulse width modulation control
#define RELAY_PIN 23
#define IGNITION_BUTTON 38
#define BRAKES_BUTTON 37
#define HEADLIGHTS_BUTTON 35
#define HEADLIGHTS_LED 26
#define BRAKES_LED 4
#define DHTPIN A2
#define DHTTYPE DHT22 //DHT 22 (AM2302) type 
#define NUM_LEDS 1 //How many RGB LEDs 
#define LEDCLK 27 //RGB LED clock on J9
#define LEDDI 28  //RGB LED data
#define CLK 9 // LCD 4 Digit Display clock on J10
#define DIO 10  // LCD 4 digital Display data
#define PEDAL 24 // Rotary switch on J6

ChainableLED leds(LEDCLK, LEDDI, NUM_LEDS); //Initialize the RGB Led
DHT dht(DHTPIN, DHTTYPE); //Initialize the temperature and humidity device
float humidity = 0;
float temperature = 0;

unsigned long rpm = 0; // Conversion of tach impulses to rpm
volatile unsigned long half_revolutions = 0; //How many impulses have been seen
unsigned long lastTach = 0; // Last time the tach was checked

volatile boolean IGNITION_PUSHED = false; // external button pushed
volatile boolean BRAKES_PUSHED = false; // brakes button pushed
volatile boolean HEADLIGHTS_PUSHED = false; // headlights button pushed

uint8_t mac[6];  // the MAC address of your Wifi shield
char macStr[18]; // char representation of mac address

const static char pubkey[] = "pub-c-38ef9127-ee77-4f40-ae48-d508ce44518e";
const static char subkey[] = "sub-c-76ce338e-865f-11e4-a400-02ee2ddab7fe";
const static char channel[] = "my_channel";
unsigned long startWifi = 0; // Start time for wifi
unsigned long endWifi = 0; // Last time working with wifi

unsigned long lastPushedIgnition = 0; // Last time the button was pushed (for debouncing)
unsigned long lastPushedBrakes = 0; // Last time brakes was pushed
unsigned long lastPushedHeadlights = 0; // Last time headlights was pushed
unsigned long lastRotaryTime = 0; // Last time the rotary was checked

// Number of digital channels
#define NUM_DCHANNELS 3
const static uint8_t digital_pins[] = { HEADLIGHTS_LED, BRAKES_LED, RELAY_PIN };

const unsigned long pubnubmsgdelay = 1000; //How often to publish to pubnub in ms
unsigned long lastpubnubmsgtime = 0; // Last time published to pubnub
const unsigned long lcddelay = 500; // How often to update the LCD in ms
unsigned long lastlcdtime = 0; // Last time lcd was updated

TM1637 tm1637(CLK, DIO); //Initialize the 4 digit LCD
int analog_value = 0; //rotary value
int8_t bits[4] = { 0 }; //The LCD display values

WiFiServer server(80);

void tachCount() {
	half_revolutions++;
}

void setup() {
	detachInterrupt (WDTHOLD);
	detachInterrupt ((uint8_t)WDTPW);
	Serial.begin(115200);
	Serial.println("\nStarting CloudOne IOT Demo");
	Serial.print("Version: ");
	Serial.println(version);
	Serial.print("Initializing board: ");
	Serial.println(boardtype);
	leds.init();
	leds.begin();
	// Set the LED to blue to indicate boot
	leds.setColorHSB(0, 0.5, 1.0, 0.01);
	dht.begin();
	tm1637.init();
	tm1637.set(BRIGHT_TYPICAL);
	pinMode(RELAY_PIN, OUTPUT);
	pinMode(BRAKES_LED, OUTPUT);
	pinMode(HEADLIGHTS_LED, OUTPUT);
	pinMode(IGNITION_BUTTON, INPUT_PULLUP);
	pinMode(BRAKES_BUTTON, INPUT_PULLUP);
	pinMode(HEADLIGHTS_BUTTON, INPUT_PULLUP);
//	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(FANPWM, OUTPUT);
	pinMode(TACH, INPUT_PULLUP);
	attachInterrupt(TACH, tachCount, FALLING);
	attachInterrupt(IGNITION_BUTTON, buttonPushedIgnition, RISING);
	attachInterrupt(BRAKES_BUTTON, buttonPushedBrakes, RISING);
	attachInterrupt(HEADLIGHTS_BUTTON, buttonPushedHeadlights, RISING);
	digitalWrite(BRAKES_LED, LOW);
	digitalWrite(HEADLIGHTS_LED, LOW);
	digitalWrite(RELAY_PIN, LOW);
	analogWrite(FANPWM, fanspeed);

	WiFi.macAddress(mac);
	snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X",
	         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	Serial.print("MAC Address: ");
	Serial.println(macStr);
	Serial.print("Attempting to connect to Network named: ");
	Serial.println(ssid);
	if (strcmp(password,"")==0){
		WiFi.begin(ssid);
	} else {
		WiFi.begin(ssid, password);
	}
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(300);
	}

	Serial.println("\nYou're connected to the network");
	Serial.println("Waiting for an ip address");

	while (WiFi.localIP() == INADDR_NONE) {
		Serial.print(".");
		delay(300);
	}

	Serial.println("\nIP Address obtained");
	printWifiStatus();

	Serial.println("Starting webserver on port 80");
	server.begin();
	Serial.println("Webserver started!");

	if(usePubNub){
		PubNub.begin(pubkey, subkey);
		Serial.println("PubNub set up");
	}

	if(useMQTT){
	// Create the mqtt url and the client id
		sprintf(clientId, "d:%s:%s:%s", organization, typeId, deviceId);
		sprintf(registeredUrl,"%s.messaging.internetofthings.ibmcloud.com",organization);
	}
	Serial.println("Initialization Complete!");
}

aJsonObject *createMessage() {
	aJsonObject *msg = aJson.createObject();
	aJsonObject *sender = aJson.createObject();
	aJson.addStringToObject(sender, "name", "cloudone");
	aJson.addItemToObject(msg, "sender", sender);

	int digitalValues[NUM_DCHANNELS];

	digitalValues[0] = digitalRead(HEADLIGHTS_LED);
	digitalValues[1] = digitalRead(BRAKES_LED);
	digitalValues[2] = digitalRead(RELAY_PIN);

	int rpmValues[3];
	rpmValues[0] = rpm;
	rpmValues[1] = fanspeed;
	rpmValues[2] = analog_value;

	double dhtValues[2];
	dhtValues[0] = double(humidity);
	dhtValues[1] = double(temperature);

	const char *board[3];
	board[0] = version;
	board[1] = macStr;
	board[2] = boardtype;

	aJsonObject *digital = aJson.createIntArray(digitalValues, NUM_DCHANNELS);
	aJsonObject *dhtJson = aJson.createFloatArray(dhtValues, 2);
	aJsonObject *rpmJson = aJson.createIntArray(rpmValues, 3);
	aJsonObject *boardJson = aJson.createStringArray(board, 3);

	aJson.addItemToObject(msg, "digital", digital);
	aJson.addItemToObject(msg, "dht", dhtJson);
	aJson.addItemToObject(msg, "fan", rpmJson);
	aJson.addItemToObject(msg, "board", boardJson);

	return msg;
}

void checkTemperatureHumidity() {
	humidity = dht.readHumidity();
	temperature = dht.readTemperature();
	if (isnan(temperature) || isnan(humidity)) {
		if(debug) Serial.println("Failed to read from DHT");
	}
}

void checkPushedButton(int DEVICE,volatile boolean* PUSHED_STATE) {
	if (*PUSHED_STATE) {
		*PUSHED_STATE = false; // Reset external button trigger
		boolean state = digitalRead(DEVICE);
		digitalWrite(DEVICE, !state);
	}
}

void webClient() {
	int i = 0;
	WiFiClient client = server.available();   // listen for incoming clients
	if (client) {                             // if you get a client,
		if(debug) Serial.println("new client");     // print a message out the serial port
		char buffer[150] = { 0 };         // make a buffer to hold incoming data
		while (client.connected()) {        // loop while the client's connected
			if (client.available()) { // if there's bytes to read from the client,
				char c = client.read();             // read a byte, then
				Serial.write(c);              // print it out the serial monitor
				if (c == '\n') {           // if the byte is a newline character

					// if the current line is blank, you got two newline characters in a row.
					// that's the end of the client HTTP request, so send a response:
					if (strlen(buffer) == 0) {
						// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
						// and a content-type so the client knows what's coming, then a blank line:
						client.println("HTTP/1.1 200 OK");
						client.println("Content-type:text/html");
						client.println();

						// the content of the HTTP response follows the header:
						client.println(
								"<html><head><title>CloudOne IOT</title></head><body align=center>");
						client.println(
								"<h1 align=center><font color=\"red\">CloudOne IOT Demo</font></h1>");
						client.print(
								"RED LED (brakes) <button onclick=\"location.href='/RH'\">HIGH</button>");
						client.println(
								" <button onclick=\"location.href='/RL'\">LOW</button><br>");
						client.print(
								"GREEN LED (headlights) <button onclick=\"location.href='/GH'\">HIGH</button>");
						client.print(
								" <button onclick=\"location.href='/GL'\">LOW</button><br>");
						client.print(
								"FLASH LEDS <button onclick=\"location.href='/FL'\">FLASH</button><br>");
						client.print(
								"RELAY (ignition) <button onclick=\"location.href='/PH'\">HIGH</button>");
						client.print(
								" <button onclick=\"location.href='/PL'\">LOW</button><br>");
						// The HTTP response ends with another blank line:
						client.println();
						// break out of the while loop:
						break;
					} else {     // if you got a newline, then clear the buffer:
						memset(buffer, 0, 150);
						i = 0;
					}
				} else if (c != '\r') { // if you got anything else but a carriage return character,
					buffer[i++] = c;     // add it to the end of the currentLine
				}

				// Check to see if the client request was "GET /H" or "GET /L":
				if (endsWith(buffer, "GET /RH")) {
					digitalWrite(BRAKES_LED, HIGH);      // GET /H turns the LED on
				}
				if (endsWith(buffer, "GET /RL")) {
					digitalWrite(BRAKES_LED, LOW);      // GET /L turns the LED off
				}
				if (endsWith(buffer, "GET /GH")) {
					digitalWrite(HEADLIGHTS_LED, HIGH);
				}
				if (endsWith(buffer, "GET /GL")) {
					digitalWrite(HEADLIGHTS_LED, LOW);
				}
				if (endsWith(buffer, "GET /FL")) {
					digitalWrite(BRAKES_LED, HIGH);
					delay(100);
					digitalWrite(HEADLIGHTS_LED, HIGH);
					digitalWrite(BRAKES_LED, LOW);
					delay(100);
					digitalWrite(HEADLIGHTS_LED, LOW);
				}
				if (endsWith(buffer, "GET /PH")) {
					digitalWrite(RELAY_PIN, HIGH);
				}
				if (endsWith(buffer, "GET /PL")) {
					digitalWrite(RELAY_PIN, LOW);
				}
			}
		}
		// close the connection:
		client.stop();
		if(debug) Serial.println("client disconnected");
	}
}

void checkRotaryAngle() {
	if ((millis() - lastRotaryTime) > rotaryDelay) {
		analog_value = analogRead(PEDAL);
		lastRotaryTime = millis();
	}
}

void updateLCD() {
	if ((millis() - lastlcdtime) > lcddelay || lastlcdtime == 0) {
		lastlcdtime = millis();
		memset(bits, 0, 4);
		int lcddisplay = rpm;
		if (lcddisplay > 9999) {
			lcddisplay = 9999;
		} else if (lcddisplay < 0) {
			lcddisplay = 0;
		}
		for (int i = 3; i >= 0; i--) {
			bits[i] = lcddisplay % 10;
			lcddisplay = lcddisplay / 10;
			tm1637.display(i, bits[i]);
		}
	}
}

void publishPubNub() {
	if ((millis() - lastpubnubmsgtime) > pubnubmsgdelay
			|| lastpubnubmsgtime == 0) {
		lastpubnubmsgtime = millis();
		aJsonObject *msg = createMessage();
		char *msgStr = aJson.print(msg);
		aJson.deleteItem(msg);
		if(debug) Serial.println(msgStr);
		WiFiClient *pubclient;
		pubclient = PubNub.publish(channel, msgStr, 1);
		free(msgStr);
		//Commenting out error handling since it happens too frequently
		if (!pubclient) {
			Serial.println("publishing error\n");
			leds.setColorHSB(0, 0.5, 1.0, 0.01);
			return;
		}
		pubclient->stop();
	}
}

void publishMQTT() {
	int rc = -1;
	if (!client.isConnected()) {
		Serial.print("Connecting to: ");
		Serial.print(registeredUrl);
		Serial.print(" with client id: ");
		Serial.println(clientId);
		while (rc != 0) {
		  rc = ipstack.connect(registeredUrl, mqttPort);
		}

		MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
		options.MQTTVersion = 3;
		options.clientID.cstring = clientId;
		options.username.cstring = authMethod;
		options.password.cstring = authToken;
		options.keepAliveInterval = 10;
		rc = -1;
		while ((rc = client.connect(options)) != 0)
		  ;
		Serial.println("Connected\n");

		Serial.print("Subscribing to topic: ");
		Serial.println(subTopic);
		// Unsubscribe the topic, if it had subscribed it before.
		client.unsubscribe(subTopic);
		// Try to subscribe for commands
		if ((rc = client.subscribe(subTopic, MQTT::QOS0, messageArrived)) != 0) {
				Serial.print("Subscribe failed with return code : ");
				Serial.println(rc);
		} else {
			  Serial.println("Subscribe success\n");
		}
	}

	aJsonObject *msg = createMessage();
	char *msgStr = aJson.print(msg);
	aJson.deleteItem(msg);
	if(debug) Serial.println(msgStr);

	MQTT::Message message;
	message.qos = MQTT::QOS0;
	message.retained = false;
	message.payload = msgStr;
	message.payloadlen = strlen(msgStr);
	rc = client.publish(topic, message);

	if (rc != 0) {
		Serial.print("Message publish failed with return code : ");
		Serial.println(rc);
	}
	free(msgStr);
	// Wait for one second before publishing again
	// This will also service any incomming messages
	//client.yield(5000);
}

void checkTach() {
	long time = millis();
	// Monitoring cannot occur while sending and receiving wifi, exclude this time
	long lostTime = endWifi - startWifi;

	// If we do not have more than tachCheckDelay, wait another cycle
	if (time - lastTach - lostTime >= tachCheckDelay) {
		// stop Monitoring
		detachInterrupt(TACH);
		// Determine the frequency of time spent monitoring per monitoring period
		double freq = (time - lastTach - lostTime) / tachCheckDelay;

		// If for some reason either the frequency or half_revolutions is less than 0
		// We should just set half_revolutions to 0 to avoid a division by 0 or odd results
		if (freq > 0 && half_revolutions > 0) {
			//Convert the counted half revolutions to the frequency period of half revolution/1 second
			half_revolutions = (half_revolutions / freq)
					/ (tachCheckDelay / 1000);
		} else {
			half_revolutions = 0;
		}
		rpm = (half_revolutions * 30);
		lastTach = millis();
		attachInterrupt(TACH, tachCount, FALLING);
		//Reset the count
		half_revolutions = 0;
	}
}

void updateSpeed() {
	int accel = (analog_value-1550) / 9.6;
	if (accel > 255) {
		accel = 255;
	} else if (accel<1){
		accel = 0;
	}
	if (fanspeed != accel) {
		fanspeed = accel;
		analogWrite(FANPWM, fanspeed);
	}
}

//
//a way to check if one array ends with another array
//
boolean endsWith(char* inString, char* compString) {
	int compLength = strlen(compString);
	int strLength = strlen(inString);

	//compare the last "compLength" values of the inString
	int i;
	for (i = 0; i < compLength; i++) {
		char a = inString[(strLength - 1) - i];
		char b = compString[(compLength - 1) - i];
		if (a != b) {
			return false;
		}
	}
	return true;
}

void printWifiStatus() {
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");

	Serial.print("To see this page in action, open a browser to http://");
	Serial.println(ip);
}

void buttonPushedIgnition() {
	if (millis() - lastPushedIgnition >= debounceTime) {
		IGNITION_PUSHED = true;
		lastPushedIgnition = millis();
	}
}

void buttonPushedBrakes() {
	if (millis() - lastPushedBrakes >= debounceTime) {
		BRAKES_PUSHED = true;
		lastPushedBrakes = millis();
	}
}

void buttonPushedHeadlights() {
	if (millis() - lastPushedHeadlights >= debounceTime) {
		HEADLIGHTS_PUSHED = true;
		lastPushedHeadlights = millis();
	}
}

void loop() {
	//Set the RGB to green to indicate loop start
	leds.setColorHSB(0, 0.25, 1, 0.01);
	checkTemperatureHumidity();
	checkPushedButton(RELAY_PIN,&IGNITION_PUSHED);
	checkPushedButton(BRAKES_LED,&BRAKES_PUSHED);
	checkPushedButton(HEADLIGHTS_LED,&HEADLIGHTS_PUSHED);
	checkRotaryAngle();
	updateSpeed();
	checkTach();
	updateLCD();
	// Set the RGB to red to indicating publishing a message
	leds.setColorHSB(0, 1 ,1, 0.01);
	detachInterrupt(TACH);
	startWifi = millis();
	if(debug) webClient();
	if(usePubNub){
		publishPubNub();
	}
	if(useMQTT){
		publishMQTT();
	}
	endWifi = millis();
	attachInterrupt(TACH, tachCount, FALLING);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message has arrived");

  char * msg = (char *)malloc(length * sizeof(char));
  int count = 0;
  for(count = 0 ; count < length ; count++) {
    msg[count] = payload[count];
  }
  msg[count] = '\0';
  Serial.println(msg);

  if(length > 0) {
    //digitalWrite(ledPin, HIGH);
    //delay(1000);
    //digitalWrite(ledPin, LOW);
  }

  free(msg);
}

void messageArrived(MQTT::MessageData& md) {
  Serial.print("Message Received\t");
    MQTT::Message &message = md.message;
    int topicLen = strlen(md.topicName.lenstring.data) + 1;
//    char* topic = new char[topicLen];
    char * topic = (char *)malloc(topicLen * sizeof(char));
    topic = md.topicName.lenstring.data;
    topic[topicLen] = '\0';

    int payloadLen = message.payloadlen + 1;
//    char* payload = new char[payloadLen];
    char * payload = (char*)message.payload;
    payload[payloadLen] = '\0';

    String topicStr = topic;
    String payloadStr = payload;

    //Command topic: iot-2/cmd/blink/fmt/json

    if(strstr(topic, "/cmd/blink") != NULL) {
      Serial.print("Command IS Supported : ");
      Serial.print(payload);
      Serial.println("\t.....");

      //pinMode(ledPin, OUTPUT);

      //Blink twice
      for(int i = 0 ; i < 2 ; i++ ) {
        //digitalWrite(ledPin, HIGH);
        //delay(250);
        //digitalWrite(ledPin, LOW);
        //delay(250);
      }
    } else {
      Serial.println("Command Not Supported:");
    }
}

