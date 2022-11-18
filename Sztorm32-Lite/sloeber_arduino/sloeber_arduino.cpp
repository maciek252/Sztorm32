/*
 Blink

 Turns an LED on for one second, then off for one second, repeatedly.

 Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
 it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
 the correct LED pin independent of which board is used.
 If you want to know what pin the on-board LED is connected to on your Arduino
 model, check the Technical Specs of your board at:
 https://www.arduino.cc/en/Main/Products

 modified 8 May 2014
 by Scott Fitzgerald
 modified 2 Sep 2016
 by Arturo Guadalupi
 modified 8 Sep 2016
 by Colby Newman

 This example code is in the public domain.

 https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
 */
#include "uCRC16XModemLib.h"

#include "Crc16.h"
//Crc 16 library (XModem)
Crc16 crc2;

// diody dzialaja bez power on!
// 19 - power on! // chyba nie...
// 27 - RED LED FRONT
// 32 led blue
// 12 tez led blue?
// 14 - no led?
// 23  - zielona z boku!
// 34, 35 - no led
// 21 lub 32 - blue z przodu, mruga!
// 33 lub 25 - czerwona z boku!
#define LED_BUILTIN 33orien

#define LED_BUILTIN2 25

//orientacja:
// yaw: USBSIDE: 0/-0
// ROUNDSIDE -90
// POWER SWITCHSIDE -180/80
// TWO BUTTONS SIDE: 90

// pitch: forwards: 180
// up: 90
// down -90
// back: 0

// roll:
// horizontal: 0
// motor up: 45 // motor down: -45


//0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 , 0xBC , 0x02 , 0x07 , 0xBB
//0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 ,0x58 ,0x02 ,0x71 ,0x67
//A5 5A 00 11 05 FF 2D 00 00 00 7C 98
//const unsigned char command[] = {0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 ,0x58 ,0x02 ,0x71 ,0x67};
const uint8_t commandShort[] =
		{ 0x00, 0x11, 0x05, 0xff, 0x44, 0x00, 0x00, 0x00 };
// 0xff 0x2d, 0x00, 0,0 - yaw ff- bardzo malo, w lewo
// 0xff 0x00, 0x00, 0,0 - yaw : - bez ruchu
// 0xff 0xbb, 0x00, 0,0 - yaw : - ~20 stopni
// 0xff 0xff, 0x00, 0,0 - yaw : - na początku mało a potem się rozpędza, 90 stopni robi na 4 razy
// 500 ms - w praktyce ruch ciagly!
// 0xff 0xaa, 0x00, 0,0 - yaw : - na początku mało a potem się rozpędza, 90 stopni robi na 4 razy
// 0xff,0x44 - powoli sie kreci
// 500 ms - tez ruch ciagly, wyraznie wolniej

//const uint8_t command[] = {0xA5, 0x5A, 0x00 , 0x11 , 0x05 , 0xff ,0x2d ,0x00, 0x00,0x00 ,0x7c ,0x98};
const uint8_t commandStartTm[] = { 0x00, 0x10, 0x05, 0xff, 0x00, 0x00, 0x00,
		0x00 };
//001005ff00000000
const uint8_t commandHeader[] = { 0xA5, 0x5A };
#define RXD2 16
#define TXD2 17

int sendCounter = 0;

long sendTime = 0;
int receiveCounter = 0;
int receiveEffectiveCounter = 0;
int receiveLength = 0;
int messageFirstByte = 0;
int messageSecondByte = 0;
uint8_t pitchFirstByte = 0;
uint8_t pitchSecondByte = 0;
uint8_t rollFirstByte = 0;
uint8_t rollSecondByte = 0;
uint8_t yawFirstByte = 0;
uint8_t yawSecondByte = 0;

HardwareSerial gps_serial(2);
uCRC16XModemLib crc;


class GimbalState {
public:
	double yaw;
	double roll;
	double pitch;

};


// the setup function runs once when you press reset or power the board
void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	gps_serial.begin(115200, SERIAL_8N1, RXD2, TXD2);
	pinMode(32, OUTPUT);
	digitalWrite(32, HIGH);   // turn the LED on (HIGH is the voltage level)

	//pinMode(14, OUTPUT);
	//digitalWrite(14, HIGH);   // turn the LED on (HIGH is the voltage level)

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LED_BUILTIN2, OUTPUT);
	// 19 - switch the power on
	pinMode(19, OUTPUT);
	digitalWrite(19, HIGH);   // turn the LED on (HIGH is the voltage level)

	pinMode(18, OUTPUT);
	digitalWrite(18, HIGH);   // turn the LED on (HIGH is the voltage level)
	Serial.begin(115200);

	sendCounter = 0;
}

double feyiuAngleTo360(double angle) {
	if (angle > 0)
		return angle;

	return 360 + angle;
}

void setYaw(double desiredAngle) {

}

int decodeTwoBytesInt(int8_t firstByte, int8_t secondByte) {
	int result = (int) ((int8_t) firstByte); // this should sign extend the number
	result <<= 8;
	int second = (int) ((int8_t) secondByte);
	if (result > 0)
		result += second;
	else
		result -= second;

	return result;
}


//GimbalState gimbalState
bool getGimbalState(GimbalState &g) {

	if (gps_serial.available()) {
		//inform that Arduino heard you saying something
		//Serial.print("Arduino heard you say: ");
		int serIn;
		//keep reading and printing from serial untill there are bytes in the serial buffer
		while (gps_serial.available() > 0) {
			serIn = gps_serial.read();  //read Serial
			//Serial.println(serIn, HEX);
			if (serIn == 0xA5) {
				Serial.print("poczatek ramki!");
				receiveCounter = 1;
				continue;
			} else if (serIn == 0x5A and receiveCounter == 1) {
				Serial.print("frame second byte!");
				receiveCounter = 2;
				continue;
			} else if (receiveCounter == 2) {
				receiveCounter++;
				messageFirstByte = (int) serIn;
				continue;
			} else if (receiveCounter == 3) {
				receiveCounter++;
				messageSecondByte = (int) serIn;
				continue;
			} else if (receiveCounter == 4) {
				receiveLength = (int) serIn;
				Serial.print(receiveLength);
				receiveCounter++;
				continue;
			}

			if (receiveCounter == 6)
				pitchSecondByte = (uint8_t) serIn;
			if (receiveCounter == 7)
				pitchFirstByte = (uint8_t) serIn;
			if (receiveCounter == 8)
				rollSecondByte = (uint8_t) serIn;
			if (receiveCounter == 9)
				rollFirstByte = (uint8_t) serIn;
			if (receiveCounter == 10)
				yawSecondByte = (uint8_t) serIn;
			if (receiveCounter == 11)
				yawFirstByte = (uint8_t) serIn;

			if (receiveLength > 0) {
				receiveLength--;
				receiveCounter++;

				if (receiveLength == 0) {

					if (messageFirstByte == 0x03 && messageSecondByte == 0x10) {
						//int pozPitch = pitchFirstByte << 8 + yawSecondByte;
						//int pozPitch = pitchFirstByte << 8 ;
						int pozPitch = (int) ((int8_t) pitchFirstByte); // this should sign extend the number
						pozPitch <<= 8;
						pozPitch += (int) ((int8_t) pitchSecondByte);

						int pozRoll = (int) ((int8_t) rollFirstByte); // this should sign extend the number
						pozRoll <<= 8;
						pozRoll += (int) ((int8_t) rollSecondByte);

						int pozYaw = (int) ((int8_t) yawFirstByte); // this should sign extend the number
						pozYaw <<= 8;
						//pozYaw += (int) ((int8_t) yawSecondByte);

						int posPitch = decodeTwoBytesInt(pitchFirstByte,
								pitchSecondByte);
						int posRoll = decodeTwoBytesInt(rollFirstByte,
								rollSecondByte);
						int posYaw = decodeTwoBytesInt(yawFirstByte,
								yawSecondByte);
//						Serial.print("pozycja!\n");
//						Serial.print(pozPitch);
//						Serial.print(" ");
//						Serial.print(pozRoll);
//						Serial.print(" ");
//						Serial.print(pozYaw);
//						Serial.print(" ");
//						Serial.print(yawFirstByte);
//						Serial.print(" ");
//						Serial.print(yawSecondByte);
						Serial.print("\npos!\n");
						Serial.print(posPitch);
						Serial.print(" ");
						Serial.print(posRoll);
						Serial.print(" ");
						Serial.print(posYaw);

						g.yaw = ((double)posYaw )/ 100.0;
						g.roll = ((double)posRoll) / 100.0;
						g.pitch = ((double)posPitch) / 100.0;
						Serial.print("message parsed (without the CRC)\n");
						return true;
					}

				}
			}

		}
	}
	return false;
}

void sendCommand(const uint8_t *command, int length) {

	//Serial.println(sizeof(command));
	//Serial.println(command[1], HEX);
	crc2.clearCrc();
	for (int j = 0; j < length; j++) {
		crc2.updateCrc(command[j]);
	}
	unsigned short value = crc2.getCrc();
	//Serial.print("crc = 0x");
	//Serial.println(value, HEX);

	gps_serial.write((uint8_t*) commandHeader, sizeof(commandHeader));
	gps_serial.write((uint8_t*) command, length);
	byte *b;
	b = (byte*) &value;
	gps_serial.write(b[0]);
	gps_serial.write(b[1]);

}


// the loop function runs over and over again forever
void loop() {

	byte bajty[30];


	GimbalState g;
	//bool wynik = true;
	bool wynik = getGimbalState(g);
	if (wynik) {
		Serial.print("\nposi!\n");
		Serial.print(g.pitch);
		Serial.print(" ");
		Serial.print(g.roll);
		Serial.print(" ");
		Serial.print(g.yaw);
		Serial.print("\n");

	}

//	digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
//	digitalWrite(LED_BUILTIN2, HIGH); // turn the LED on (HIGH is the voltage level)
//	delay(1000);                       // wait for a second
//	digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
//	digitalWrite(LED_BUILTIN2, LOW); // turn the LED off by making the voltage LOW
//	delay(1000);                       // wait for a second

	if (sendCounter < 40) {
		sendCommand(commandStartTm, 8);
		sendCounter++;
	} else
		sendCounter = 0;

	if (millis() - sendTime > 500) {
		sendTime = millis();
		//if (sendCounter < 400) {

		//sendCounter++;
		Serial.print(
				"-------------------------------------------------------SEND-COMMAND-----------------------!\n");
		sendCommand(commandShort, 8);
	}

	const char *p_buf = "abcd";
	const unsigned int *p_int = reinterpret_cast<const unsigned int*>(p_buf);
}

