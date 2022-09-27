/*
	LoRa Simple Gateway/Node Exemple

	This code uses InvertIQ function to create a simple Gateway/Node logic.

	Gateway - Sends messages with enableInvertIQ()
					- Receives messages with disableInvertIQ()

	Node		- Sends messages with disableInvertIQ()
					- Receives messages with enableInvertIQ()

	With this arrangement a Gateway never receive messages from another Gateway
	and a Node never receive message from another Node.
	Only Gateway to Node and vice versa.

	This code receives messages and sends a message every second.

	InvertIQ function basically invert the LoRa I and Q signals.

	See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
	for more on InvertIQ register 0x33.

	created 05 August 2018
	by Luiz H. Cassettari
*/

#include <JeeLib.h>			//JeeLib: Used for Sleepy class
#include <SPI.h>			// include libraries
#include <LoRa.h>

// #define DEBUG

#define SERIAL_BAUD	 				57600
#define NODE_ID			 			30			// NodeId of this LoRa Node
#define MAX_PACKET_SIZE				10

#define MSG_ID_NODE_STARTUP		1 	 	// Node startup notification
#define MSG_ID_STILL_ALIVE		2  		// Node still alive
#define MSG_ID_CMND_REQUEST		3  		// Node wakeup/cmnd request
#define MSG_ID_COUNTED_PULSES	4  		// Switch change detected

#define SEND_MSG_EVERY	22				// Watchdog is a timerTick on a avg 8,2 sec timebase
										// SEND_MSG_EVERY=7	-> +- 1min
										// SEND_MSG_EVERY=14 -> +- 2min
										// SEND_MSG_EVERY=22 -> +- 3min -> used
										// SEND_MSG_EVERY=26 -> +- 4min
										// SEND_MSG_EVERY=33 -> +- 5min

#define SEND_MEASURE_VCC_EVERY	90		// Measure battery voltage every N messages
										// MEASURE_EVERY=90 -> +- 4 hour


#ifdef DEBUG
volatile word sendMsgTimer = SEND_MSG_EVERY - 1;
#else
volatile word sendMsgTimer = SEND_MSG_EVERY - 2;
#endif
volatile unsigned char sendMsgVccLevelTimer = SEND_MEASURE_VCC_EVERY;

//Message max 30 bytes
struct Payload {
	byte nodeId;
	byte msgId;
	byte voltageVcc;				//getVcc 1.0V=0, 1.8V=40, 3,0V=100, 3.3V=115, 5.0V=200, 6.0V=250
	unsigned int nrOfPulses;		//nrOfPulses counted since last send message
	unsigned int nrOfPulsesCheck;	//To check if the last message was lost, counter does not reset like nrOfPulses value
} txPayload;

const long loRaFrequency = 866E6;			// LoRa loRaFrequency

const int loRaCsPin = 15;					// LoRa radio chip select
const int loRaResetPin = 14;			 	// LoRa radio reset
const int loRaIrqPin = 2;					// change for your board; must be a hardware interrupt pin

const int pulseIrqPin = 3;

//Number of pulses, used to measure energy.
volatile unsigned int	nrOfPulses = 0; //Unsigned int=32bit=max 4.294.967.295
volatile unsigned char pulseSendCounter = 0;
volatile unsigned char oldPulseSendCounter = 0;
volatile unsigned char deltaPulse = 0;
volatile bool sendMsg = false;
volatile bool timerTick = false;


ISR (WDT_vect)
{
	// WDIE & WDIF is cleared in hardware upon entering this ISR
	// wdt_disable();
	Sleepy::watchdogEvent();
	if(!timerTick) { timerTick = true; }
}

//External interrupt 1
void pulseCount () { 
	nrOfPulses++; 
	pulseSendCounter++; 
}

void LoRa_rxMode(){
	LoRa.enableInvertIQ();								// active invert I and Q signals
	LoRa.receive();										// set receive mode
}

void LoRa_txMode(){
	LoRa.idle();											// set standby mode
	LoRa.disableInvertIQ();								// normal mode
}

void LoRa_sendMessage(Payload payload, byte payloadLen) {
	LoRa_txMode();											// set tx mode
	LoRa.beginPacket();								 	// start packet
	LoRa.write((byte*) &payload, payloadLen); 	// add payload
	LoRa.endPacket(true);								// finish packet and send it
}

void onReceive(int packetSize) {
	byte rxPayload [MAX_PACKET_SIZE];

	byte i = 0, rxByte;

	while (LoRa.available()) {
		rxByte = (byte)LoRa.read();
		if (i < MAX_PACKET_SIZE) {
			rxPayload[i] = rxByte;
			i++;
		}
	}

	// Only accept messages with our NodeId
	if (rxPayload[0] == NODE_ID) {
#ifdef DEBUG
		Serial.print("Rx packet OK "); // Start received message
		for (char i = 0; i < packetSize; i++) {
				Serial.print(rxPayload[i], DEC);
				Serial.print(' ');
		}
#endif
	}
}

void onTxDone() {
	// Serial.println("TxDone");
	LoRa_rxMode();
}

static byte vccLevelRead()
{
  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Let mux settle a little to get a more stable A/D conversion
  
  // Start a conversion
  ADCSRA |= _BV(ADSC);
  
  // Wait for it to complete
  while (bit_is_set(ADCSRA, ADSC));
  
  // convert ADC readings to fit in one byte, i.e. 20 mV steps:
  // 1.0V = 0, 1.8V = 40, 3.0V = 100, 3.3V = 115, 5.0V = 200, 6.0V = 250
  return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
#ifdef DEBUG
	Serial.begin(SERIAL_BAUD);									 // initialize serial
	while (!Serial);

	Serial.println();
	Serial.print("[LORA-NODE.");
	Serial.print(NODE_ID);
	Serial.println("]");
#endif

	LoRa.setPins(loRaCsPin, loRaResetPin, loRaIrqPin);

	if (!LoRa.begin(loRaFrequency)) {
#ifdef DEBUG
		Serial.println("LoRa init failed. Check your connections.");
#endif
		while (true);											 // if failed, do nothing
	}

	//LoRa.setTxPower(20);
	LoRa.enableCrc();
	LoRa.onReceive(onReceive);
	LoRa.onTxDone(onTxDone);
	LoRa_rxMode();

	txPayload.voltageVcc = 0;

	// Switch input external interrupt
	pinMode(pulseIrqPin, INPUT_PULLUP); //TODO _PULLUP weghalen
	attachInterrupt(digitalPinToInterrupt(pulseIrqPin), pulseCount, FALLING);
	// Send Node startup msg
	txPayload.nodeId = NODE_ID;
	txPayload.msgId = MSG_ID_NODE_STARTUP;

	LoRa_sendMessage(txPayload, 2); // send a message
	delay(40); // [ms] Give RFM95W time to send the message
	LoRa.sleep(); // Put RFM95W in sleep mode
#ifdef DEBUG
	delay(100); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG
	 Sleepy::watchdogInterrupts(1);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
}

void loop() {
	// Enter power down state with ADC and BOD module disabled. Wake up when wake up pin is low
	// Serial.println("Sleep for 8s....");
	// delay(100);

	// Waked up! From timer or movement (external interrupt)
	 if (timerTick) 
	 { // There has ben a Watchdog interrupt for time measurement
		timerTick = false;

#ifdef DEBUG
		Serial.println("timerTick");
#endif //DEBUG

		//Normal situation, power test not active
		deltaPulse = oldPulseSendCounter >> 2; //Divide pulses (power) by 4, which is a 25% change
		if (deltaPulse == 0) deltaPulse = 1; //Limit deltaPulse by 1, this is a minimum of aprox. 44W
		
		if ((pulseSendCounter < oldPulseSendCounter - deltaPulse) || (pulseSendCounter > oldPulseSendCounter + deltaPulse)) {
			//Power is changed more than -25% or +25%
			sendMsgTimer = SEND_MSG_EVERY;
		} else {
			sendMsgTimer++;
		}
		oldPulseSendCounter = pulseSendCounter;
		pulseSendCounter = 0;

		if (sendMsgTimer >= SEND_MSG_EVERY) {
			sendMsgTimer = 0;
			
			sendMsgVccLevelTimer++;
			if (sendMsgVccLevelTimer >= SEND_MEASURE_VCC_EVERY) {
				sendMsgVccLevelTimer = 0;
				txPayload.voltageVcc = vccLevelRead();
			}
			sendMsg = true;
		}
#ifdef DEBUG
		delay(100); // [ms] Give time to print the debug messages before sleep
#endif //DEBUG
	}

	if (sendMsg) 
	{
		sendMsg = false;
		
		txPayload.nodeId = NODE_ID;
		txPayload.msgId = MSG_ID_COUNTED_PULSES;
		txPayload.nrOfPulses = nrOfPulses;
		txPayload.nrOfPulsesCheck += nrOfPulses;
		nrOfPulses = 0; // Reset the counter for the next measurement
		LoRa_sendMessage(txPayload, sizeof txPayload); // send the message
		delay(40); // [ms] Give RFM95W time to send the message
		LoRa.sleep(); // Put RFM95W in sleep mode
	}
	Sleepy::watchdogInterrupts(9);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
	Sleepy::powerDown();
}
