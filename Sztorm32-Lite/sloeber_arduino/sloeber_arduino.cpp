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

namespace {

const int TIME_BETWEEN_STEPS = 5000;
const int TIME_BETWEEN_MOVES = 100;

}

//Crc 16 library (XModem)
Crc16 crc2;

// diody dzialaja bez power on!
// 19 - power on! // chyba nie...
// 27 - RED LED FRONT
// 32 led blue CONFIRMED!
// 12 tez led blue?
// 14 - no led?
// 23  - zielona z boku!
// 34, 35 - no led
// 21 lub 32 - blue z przodu, mruga!
// 33 lub 25 - czerwona z boku!
#define LED_BUILTIN 27
#define LED_BUILTIN2 14
#define PIN_MAIN_BUTTON 26
#define PIN_POWER_OFF 19

enum Parameters {
	ReversePanningControlBool = 0x0A,
	ReverseTiltingControlBool = 0x0B,
	TiltingSpeed = 0x0C,
	PanningSpeed = 0x0D,
	MotorStrengthTiltAxis = 0x07,
	MotorStrengthPanAxis = 0x09,
	MotorStrengthRollAxis = 0x08,
	Scene = 0x12,
	TimelapsePanningSpeed = 0x14,
	TimelapseTiltingSpeed = 0x15,
	HorizontalCalibration = 0x16
};

bool setTiltingSpeed = false;
bool setPanningSpeed = false;

// speed:
// tilting 3/4, panning 50%:
//  A5 5A 02 0D 03 0D 32 00
//  0c 4b 00
// czyli drugi bajt zawsze 00, pierwszy 00-64?
// reverse tilting control (bool)
//A5 5A 02 0D 03 0B 00 00
//

// motor strengh:
// pan axis max, tilt: 1/3, roll: 50%
// A5 5A 02 0D 03 07 DB D3
// A5 5A 02 0D 03 08 2A B2
// A5 5A 02 0D 03 09 20 4E
// pan i roll max, tilt 15% (default setting):
// A5 5A 02 0D 03 07 48 F4 (tilt)
// A5 5A 02 0D 03 08 20 4E
// A5 5A 02 0D 03 09 20 4E
// pan i tilt 40%, roll 80%:
// A5 5A 02 0D 03 07 8E B8 (tilt)
// A5 5A 02 0D 03 09 D0 B6 (pan)
// A5 5A 02 0D 03 08 AA 31
// pan i roll 50%, tilt 20%:
// 07 D4 E3
// 09 D8 10 // 9 to roll?
// 08 33 0C
// pan i roll 30%, tilt 50%:
// 07 33 0C (tilt)
// 09 14 C7
// 08 A0 C5
// pan: 75%, roll 80%, tilt: 30%
// 09 E2 24 (pan)
// 08 1e 3c (roll)
// 07 24 c2 (tilt)

// 15 48 F4
// 20 D4 E3
// 40 xx B8
// 30 xx C7
// 50 33 0C
// 75 e2 24
// 80 aa 31, 1e 3c
// 100 xx 4e - 100? 4e - 78?
// srodek to zero!!! w leys w prawo plusy!

//

// max 20 4e

// horizontal calibration from the main page: (it is not permanent compared to the one from the specialized tab?)
// A5 5A 00 10 - 05 F7 00 00 00 00
// A5 5A 00 10 - 05 FB 00 00 00 00
//(lewo-prawo)

// PAN SETTING (main display):
// A5 5A 02 40 02 04 00 (left button)
// A5 5A 02 40 02 04 01 (central button)
// A5 5A 02 40 02 04 03 right button

// horizontal adjustment range - local setting, influences the calibration value
// timelapse:
//A5 5A 02 0D 03 15 4F 00 87 84

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

// certain, proved: -2 almost flat, -8, 8 - boundaries
// 9 too much
// asymmetrical, maybe there is some trim?

// YAW, ROLL, PITCH
const int cameraMovesVector[6][3] = { { 1040, 9, 1010 }, { 1039, -9, 1100 }, {
		1001, -2, 1190 }, { 0, -7, 0 }, { 0, 0, 0 } };
//const int cameraMovesVector[6][3] = { { 0, 8, 0 }, { 0, -8, 0 }, { 0, -2, 0 }, {
//		0, -7, 0 }, { 0, 0, 0 } };
//const int cameraMovesVector[6][3] = { { 0, 10, -50 }, { -30, -10, 50 }, { 30,
//	10, -50 }, { 0, 0, 50 } };
const int numOfSteps = 3;
int currentStep = -1;

bool gimbReset = false;
bool calculatedCompensation = false;

int cameraMovesCounter = 0;

//const uint8_t command[] = {0xA5, 0x5A, 0x00 , 0x11 , 0x05 , 0xff ,0x2d ,0x00, 0x00,0x00 ,0x7c ,0x98};
const uint8_t commandStartTm[] = { 0x00, 0x10, 0x05, 0xff, 0x00, 0x00, 0x00,
		0x00 };
//001005ff00000000
const uint8_t commandHeader[] = { 0xA5, 0x5A };
#define RXD2 16
#define TXD2 17
const int YAW_IDX = 0;
const int ROLL_IDX = 1;
const int PITCH_IDX = 2;

int sendCounter = 0;
bool tempRoll = false;
int rollDirection;

double desired[3];
bool isMoving[3] = { false, false, false };
double pos360[3];
double speed[3] = { 0, 0, 0 };
double compensation[3] = { 0, 0, 0 };

long sendTime = 0;
long moveTime = 0;

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

struct GimbalFrame {
public:
	byte frameCode;
	byte frameSubcode;
	byte length;
	byte contents[10];

};

class GimbalState {
public:
	double yaw;
	double roll;
	double pitch;

};

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

void switchFrontLED(bool on) {

	if (on) {

		//32 - blue confirmed
		digitalWrite(LED_BUILTIN2, HIGH); // turn the LED on (HIGH is the voltage level)
		digitalWrite(LED_BUILTIN, LOW); // turn the LED on (HIGH is the voltage level)
	} else {
		digitalWrite(LED_BUILTIN2, LOW); // turn the LED on (HIGH is the voltage level)
		digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
	}
}

double feyiuAngleTo360(double angle) {
	if (angle > 0)
		return angle;

	return 360 + angle;
}

char getMoveStrength(int distance) {
	return 0xff;

	if (distance > 15)
		return 0xf2;
	else if (distance > 8)
		return 0xc2;
	else if (distance > 4)
		return 0x90;

	return 0x40;
}

uint8_t* prepareMoveCommand(uint8_t *commandZero, int yaw, int pitch) {

	char yawStrength = getMoveStrength(abs(yaw));

	if (yaw > 0)
		commandZero[4] = yawStrength;
	else if (yaw < 0)
		commandZero[5] = yawStrength;
	else if (yaw == 0) {
		commandZero[4] = 0xff;
		commandZero[5] = 0xff;
	}

	char pitchStrength = getMoveStrength(abs(pitch));

	// this moves upwards
	if (pitch > 0)
		commandZero[6] = pitchStrength;
	else if (pitch < 0)
		commandZero[7] = pitchStrength;
	else if (pitch == 0) {
		commandZero[6] = 0xff;
		commandZero[7] = 0xff;
	}
	return commandZero;

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

bool getGimbalResponse(uint8_t messageTypeId, uint8_t &firstField,
		uint8_t &secondField, uint8_t &thirdField) {

	bool readMessage = false;
	uint8_t zeroField;

	if (gps_serial.available()) {
		//inform that Arduino heard you saying something
		//Serial.print("Arduino heard you say: ");
		int serIn;
		int payloadLength = 0;
		//keep reading and printing from serial untill there are bytes in the serial buffer
		while (gps_serial.available() > 0) {

			serIn = gps_serial.read();  //read Serial
			//Serial.println(serIn, HEX);
			//Serial.println("\n");
			if (serIn == 0xA5) {
				//Serial.print("poczatek ramki!");
				receiveCounter = 1;

				continue;
			} else if (serIn == 0x5A and receiveCounter == 1) {
				//Serial.print("frame second byte!");
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
				payloadLength = receiveLength;
				//Serial.print(receiveLength);
				receiveCounter++;
				continue;
			}

			if (receiveCounter == 5) {

				zeroField = (uint8_t) serIn;
			}
			if (receiveCounter == 6) {

				firstField = (uint8_t) serIn;
			}
			if (receiveCounter == 7) {
				secondField = (uint8_t) serIn;
			}
			if (receiveCounter == 8) {
				thirdField = (uint8_t) serIn;
				readMessage = true;
			}

			if (receiveLength > 0) {
				receiveLength--;
				receiveCounter++;

				if (receiveLength == 0) {
					if (messageFirstByte == 0x03 && messageSecondByte == 0x0E) {

						Serial.print("ANSWER2223!!!!!");
						Serial.print(zeroField, HEX);
						Serial.print(" ");
						Serial.print(firstField, HEX);
						Serial.print(" ");
						Serial.print(secondField, HEX);
						Serial.print(" ");
						Serial.print(thirdField, HEX);
						Serial.print("\n");

					}
				}
			}

		}
	}
	if (readMessage && (zeroField == messageTypeId)) {
		Serial.print("ANSWER2223OK!!!!!");
		Serial.print(zeroField, HEX);
		Serial.print(" ");
		Serial.print(messageTypeId, HEX);
		Serial.print("\n");
		return true;

	}
	return false;
}

//GimbalState gimbalState
bool getGimbalState(GimbalState &g) {

	uint8_t firstExtra, secondExtra, zeroField, firstField, secondField,
			thirdField;

	if (gps_serial.available()) {
		//inform that Arduino heard you saying something
		//Serial.print("Arduino heard you say: ");
		int serIn;
		int payloadLength = 0;
		//keep reading and printing from serial untill there are bytes in the serial buffer
		while (gps_serial.available() > 0) {

			serIn = gps_serial.read();  //read Serial
			//Serial.println(serIn, HEX);
			//Serial.println("\n");
			if (serIn == 0xA5) {
				//Serial.print("poczatek ramki!");
				receiveCounter = 1;

				continue;
			} else if (serIn == 0x5A and receiveCounter == 1) {
				//Serial.print("frame second byte!");
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
				payloadLength = receiveLength;
				//Serial.print(receiveLength);
				receiveCounter++;
				continue;
			}

			if (receiveCounter == 5) {

				zeroField = serIn;
			}
			if (receiveCounter == 6) {
				pitchSecondByte = (uint8_t) serIn;
				firstField = serIn;
			}
			if (receiveCounter == 7) {
				pitchFirstByte = (uint8_t) serIn;
				secondField = serIn;
			}
			if (receiveCounter == 8) {
				rollSecondByte = (uint8_t) serIn;
				thirdField = serIn;
			}
			if (receiveCounter == 9)
				rollFirstByte = (uint8_t) serIn;
			if (receiveCounter == 10)
				yawSecondByte = (uint8_t) serIn;
			if (receiveCounter == 11)
				yawFirstByte = (uint8_t) serIn;
			if (receiveCounter == 12)
				firstExtra = (uint8_t) serIn;
			if (receiveCounter == 13)
				secondExtra = (uint8_t) serIn;

			if (receiveLength > 0) {
				receiveLength--;
				receiveCounter++;

				if (receiveLength == 0) {
					////A5 5A 02 0D 03 0C 00 00 // 03 - index?

					// po 5A tylko 03 10 08?
					// dwa ostatnie bajty to odpowiedz?
					// scene EF i pozniej 3e, 3f, 40 - scene?
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

						g.yaw = ((double) posYaw) / 100.0;
						g.roll = ((double) posRoll) / 100.0;
						g.pitch = ((double) posPitch) / 100.0;
						//Serial.print("message parsed (without the CRC)\n");

						//Serial.print("EXTRA EXTRA!");
						Serial.print(firstExtra, HEX);
						Serial.print(" ");
						Serial.print(secondExtra, HEX);
						Serial.print("\n");

						return true;
					} else if (messageFirstByte == 0x01
							&& messageSecondByte == 0x00
							&& payloadLength == 4) {
						Serial.print("ANSWER!!!!!");

					} else if (messageFirstByte == 0x03
							&& messageSecondByte == 0x0E) {

						Serial.print("ANSWER2222!!!!!");
						Serial.print(zeroField, HEX);
						Serial.print(" ");
						Serial.print(firstField, HEX);
						Serial.print(" ");
						Serial.print(secondField, HEX);
						Serial.print(" ");
						Serial.print(thirdField, HEX);
						Serial.print("\n");
						while (1) {
							//	Serial.print("ANSWER2222!!!!!");
						}
						// wlacz tm?
						// przychodzi i bez tm, heartbeat?
//						Serial.print("03 0E CO TO CO TO? len=0 len=");
//						Serial.print(payloadLength, HEX);
//						Serial.print("\n");
					} else {
						// 03 0e e
						// 03 20 0 też bywa
						// 01 00 00 // gdy gimbal się nie rusza chyba?
						// 01 01 00
						// 01 0e 00

						// pan-tilt
						// A5 5A 02 40 02 04 00  - zadanie
						//    5A 02 40 02 04 01
						//    5A 02 40 02 04 03
						// A5 5A 01 00 04 8C 13 CE 01 -to chyba to?
						// a odpowiedz 03 40 0?
						Serial.print("INNE INNE!");
						Serial.print(messageFirstByte, HEX);
						Serial.print("\n");
						Serial.print(messageSecondByte, HEX);
						Serial.print(" ");
						Serial.print(firstField, HEX);
						Serial.print(" ");
						Serial.print(secondField, HEX);
						Serial.print(" ");
						Serial.print(thirdField, HEX);
						Serial.print("\n");
						Serial.print(payloadLength, HEX);
						Serial.print("\n");

					}

				}
			}

		}
	}
	return false;
}

double normalize360(double a) {
	double result = a;
	if (a > 360.0)
		result -= 360.0;
	if (a < 0)
		result = 360.0 + result;
	return result;
}

double getCwOrCcw(double desired, double current, int variant) {

	double ccw = normalize360(desired - current);
	double cw = normalize360(current - desired);

	Serial.print("ccw=");
	Serial.print(ccw);
	Serial.print("cw=");
	Serial.print(cw);
	Serial.print("\n");

	if (variant == PITCH_IDX) {
		if (cw < ccw) {
			return ccw;
		}
		return -cw;
	}

	if (cw > ccw) {
		return ccw;
	}
	return -cw;
}

void recenterGimbal() {

	// works!
	//02 40 02 05 02
	uint8_t commandRecenter[] = { 0x02, 0x40, 0x02, 0x05, 0x02 };

	//sendCommand(commandShort, 8);
	sendCommand(commandRecenter, 5);
}

void resetGimbal() {
	//02 0F 00
	uint8_t commandReset[] = { 0x02, 0x0f, 0x00 };

	//sendCommand(commandShort, 8);
	sendCommand(commandReset, 3);
}

void printReport() {
	Serial.print("CURR360 Y=");
	Serial.print(pos360[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(pos360[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(pos360[PITCH_IDX]);
	Serial.print("  DES Y=");
	Serial.print(desired[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(desired[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(desired[PITCH_IDX]);
	Serial.print("  SPEED Y=");
	Serial.print(speed[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(speed[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(speed[PITCH_IDX]);

	Serial.print("  IS_MOV Y=");
	Serial.print(isMoving[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(isMoving[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(isMoving[PITCH_IDX]);
	Serial.print("currSt=");
	Serial.print(currentStep);
	Serial.print("\n");
}

void setRoll(int value) {
	// CZYLI 02 0D 03 - to set? Czy zależy od indeksu?
	//02 0D 03 16 - 16 to index
	// BC 02
	// dziala! 2003 - w lewo delikatnie
	// bc02 - nic 9001 - b. delikatnie w prawo!
	// A001- nic
	// 845c - mocno w prawo i wariuje!
	//841c - tez wariuje
	// druga wazna i ma byc malo!
	// 8403 - nic zauwazalnego
// 2003 0 mocniej w lewo
// 2c03 - nic
	// 2c01 - prawie plasko
	// 2f01 - nic
	// 2004
	// 2005 - mocniej niz 2003? -
	// 2007 - zdecydowanie mocniej niz 2003!! koniec polki prawie cm!
	// 20fe - malo ale w prawo //z grubsza plasko
	// 20ee tez mocno w prawo
// 20fa - mocno w prawo!

	// 20xx
	// fa, ee - mocno w prawo i prawie plasko
	// 03, fa - pol-lewo i malo prawo
	// 03, ee- pol-lewo(03) i bardz o mocno w prawo (ee)
	// 07, ee -mocno w obie strony, ee mocniej

	if (value > 16 || value < -16) {
		return;
	}

	uint8_t commandRoll[] = { 0x02, 0x0d, 0x03, 0x16, 0x22, 0x0d };
	if (value > 0)
		commandRoll[5] = (char) value;
	else if (value < 0)
		commandRoll[5] = (char) (256 + value);

	// fd - praktycznie plasko
	// fa - lekko w prawo
	// ed - max w prawo
	// 08 prawie max w lewo
	// 09 jeszcze mocniej w lewo

//	if(tempRoll){
//		commandRoll[5] = 0xee;
//	}
	// ee to minusy? ff to zero, i potem w dol
	// 22xx, 0d, ee - glebokie, 22 nic nie zmienia, chyba less significant byte?

	tempRoll = !tempRoll;

	//commandRoll[3] = 0x16; // to chyba indeks tej operacji
	//commandRoll[4] = 0x20;
	//commandRoll[5] = 0x03;

	// 9020 tez nic, nie wiem czy poprzednie dobre? 0302 nie dziala!
	// zle indeksy byly, jeszcze raZ!
	// 0302 w zasadzie nic moze trzeba save?
	//sendCommand(commandShort, 8);
	Serial.print("SETTING ROLL=");
	Serial.print(commandRoll[5]);
	Serial.print("=\n");
	sendCommand(commandRoll, 6);
	printReport();
	// this is starttm
	//uint8_t commandSave[] = { 0x00, 0x10, 0x05, 0xff, 0x00, 0x00, 0x00, 0x00 };
	//sendCommand(commandSave, 8);
}

void doMovingPitchAndYaw() {

	int yawMove = 0;
	int pitchMove = 0;

	if (isMoving[YAW_IDX]) {

		if (abs(desired[YAW_IDX] - pos360[YAW_IDX]) > 8) {

			Serial.print(
					"===========================YAW MOVE!!==========================!\n");
			int yawDirection = getCwOrCcw(desired[YAW_IDX], pos360[YAW_IDX],
					YAW_IDX);

			yawMove = abs(desired[YAW_IDX] - pos360[YAW_IDX]);
			if (yawDirection < 0) {
				yawMove *= -1;
			}
		} else if (speed[YAW_IDX] < 1) {
			Serial.print(
					"===========================REACHED YAW TARGET!!==========================!\n");
			isMoving[YAW_IDX] = false;
			moveTime = millis();
		}
	}
	if (isMoving[PITCH_IDX]) {
		if (abs(desired[PITCH_IDX] - pos360[PITCH_IDX]) > 8) {
			Serial.print(
					"===========================PITCH MOVE!!==========================!\n");
			pitchMove = abs(desired[PITCH_IDX] - pos360[PITCH_IDX]);
			int pitchDirection = getCwOrCcw(desired[PITCH_IDX],
					pos360[PITCH_IDX], PITCH_IDX);

			if (pitchDirection < 0)
				pitchMove *= -1;

		} else if (speed[PITCH_IDX] < 1) {
			Serial.print(
					"===========================REACHED PITCh tARGET!!==========================!\n");

			isMoving[PITCH_IDX] = false;
			moveTime = millis();
		}
	}
	if (yawMove != 0 || pitchMove != 0) {

		//if (sendCounter < 400) {

		uint8_t commandZero[] =
				{ 0x00, 0x11, 0x05, 0xff, 0x00, 0x00, 0x00, 0x00 };

		uint8_t *commandMove = prepareMoveCommand(commandZero, yawMove,
				pitchMove);

		//sendCommand(commandShort, 8);
		sendCommand(commandMove, 8);
		Serial.print("-----SEND SEND-COMMAND-----------------------!");
		Serial.print("yawMove=");
		Serial.print(yawMove);
		Serial.print("pitchMove=");
		Serial.print(pitchMove);
		Serial.print("\n");

	}
//sendCounter++;
}

void doMovingRoll() {

	int rollMove = 0;

//	if (isMoving[ROLL_IDX]) {
//		if (abs(desired[ROLL_IDX] - pos360[ROLL_IDX]) > 10) {
//
//			rollMove = 256;
////			int pitchDirection = getCwOrCcw(desired[PITCH_IDX],
////					pos360[PITCH_IDX]);
//			if (rollDirection > 0) {
//				rollMove = -256;
//			}
//		} else {
//			Serial.print(
//					"===========================REACHED ROLL TARGET!!==========================!\n");
//			isMoving[ROLL_IDX] = false;
//			moveTime = millis();
//		}
//	}

// if(rollMove != 0)
	if (isMoving[ROLL_IDX]) {

		setRoll(desired[ROLL_IDX]);

		//delay(2000);
		isMoving[ROLL_IDX] = false;
		moveTime = millis();
		Serial.print("-----SEND ROLL-MOVE-COMMAND-----------------------!");
		Serial.print("\n");

	}

}

void doMoving() {

	doMovingPitchAndYaw();
	doMovingRoll();

	if (!isMoving[YAW_IDX] && !isMoving[ROLL_IDX] && !isMoving[PITCH_IDX]) {
		Serial.print(
				"\n-REACHED_ALL TARGETS!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@-!");
		moveTime = millis();
		switchFrontLED(false);
	}

}

void calculateCompensation() {

	compensation[YAW_IDX] = pos360[YAW_IDX];
	compensation[ROLL_IDX] = pos360[ROLL_IDX];
	compensation[PITCH_IDX] = pos360[PITCH_IDX];
	Serial.println("COMPENSATIONS:");
	Serial.print("YAW=");
	Serial.print(compensation[YAW_IDX]);
	Serial.print(" ROLL=");
	Serial.print(compensation[ROLL_IDX]);
	Serial.print(" PITCH=");
	Serial.print(compensation[PITCH_IDX]);
	Serial.print("\n");
}

bool getFrame(GimbalFrame &frame) {

	int frameCounter = 0;
	if (gps_serial.available()) {
		//inform that Arduino heard you saying something
		//Serial.print("Arduino heard you say: ");

		//keep reading and printing from serial untill there are bytes in the serial buffer
		while (gps_serial.available() > 0) {
			int serIn = gps_serial.read();  //read Serial
			Serial.print("przyszlo = ");
			Serial.print(serIn, HEX);
			Serial.print("\n");
			//Serial.println(serIn, HEX);
			if (serIn == 0xA5) {
				Serial.print("poczatek ramki!");
				frameCounter = 1;
				continue;
			} else if (serIn == 0x5A and frameCounter == 1) {
				//Serial.print("frame second byte!");
				frameCounter = 2;
				continue;
			} else if (frameCounter == 2) {
				frameCounter++;
				frame.frameCode = serIn;
				continue;
			} else if (frameCounter == 3) {
				frameCounter++;
				frame.frameSubcode = serIn;
				continue;
			} else if (frameCounter == 4) {
				frame.length = serIn;
				//Serial.print(receiveLength);
				frameCounter++;
				continue;
			}

			frame.contents[frameCounter - 5] = serIn;
			frameCounter++;

			if (frameCounter == frame.length + 5)
				return true;
		}
	}
}

void readSpeeds() {

	// TAK oppowiedz to jest 03 0d 00, bo tyle razy!

	//0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 , 0xBC , 0x02 , // save????

	// motor strength - 3 arg:
	// 02 0d 03 07 20 4e

	// odpytanie?
	// A5 5A 02 0E 01 12 // ?
	// chyba tak
	// 07, 08, 09 - motor strengths?
	// 12 scene smooth etc
	// 0d - joystick? 0c, 0a, 0b - te same indeksy // c,d - suwaki, a,b - przelaczniki
	//A5 5A 02 40 02 04 00
	//       uint8_t commandRecenter[] = { 0x02, 0x40, 0x02, 0x05, 0x02 };
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x40, 0x02, 0x04, 0x01 };
	//A5 5A 02 0D 03 - 12 01 00 - no i OK, skoro odpowiada!

	// scene :12
	//A5 5A 02 0E 01 12
	// horizonatal calibration
	// przekreślone zero w prawo:
	// A5 5A - 02 0D 03 16 58 02
	// w lewo:
	// A5 5A 02 0D 03 16 0C FE

	const uint8_t commandAskSpeeds[] = { 0x02, 0x0d, 0x03, 0x0A, 0x01, 0x00 };
	sendCommand(commandAskSpeeds, 6);

//	GimbalFrame frame;
//	bool wynik = getFrame(frame);
//	Serial.print("wynik = ");
//	Serial.print(wynik);
//	Serial.print("\n");
//	Serial.print(frame.contents[0], HEX);
//	Serial.print("\n");
//	Serial.print(frame.contents[1], HEX);
//	Serial.print("\n");
//	Serial.print(frame.contents[2], HEX);
//	Serial.print("\n");
//	while (1) {
//
//	}

	//A5 5A 02 0D    .Z......  .....Z..
	//03 0D 00 00 B7 73
	//A5 5A 02 0D 03 0C 00 00 // 03 - index?

	// save
	// A5 5A 02 0D 03 0B 01 00 26 F 0b - index, 01 00 - value, save
}

void readParameter(Parameters parameter) {
	uint8_t commandAskSpeeds[] = { 0x02, 0x0e, 0x01, 0x09 }; // na to nie odpowiada nic
	commandAskSpeeds[3] = (uint8_t) parameter;
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x40, 0x02, 0x04, 0x02}; //- no answer to this
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x0d, 0x03, 0x0B, 0x01, 0x00 }; //- answers  - confirmation?
	sendCommand(commandAskSpeeds, 4);

}

void setParameter(Parameters parameter, uint8_t valueFirst,
		uint8_t valueSecond) {
	//A5 5A 02 0D 03 0C 64 00
	uint8_t commandAskSpeeds[] = { 0x02, 0x0d, 0x03, 0x0c, 0x64, 0x00 }; // na to nie odpowiada nic
	commandAskSpeeds[3] = (uint8_t) parameter;
	commandAskSpeeds[4] = (uint8_t) valueFirst;
	commandAskSpeeds[5] = (uint8_t) valueSecond;
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x40, 0x02, 0x04, 0x02}; //- no answer to this
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x0d, 0x03, 0x0B, 0x01, 0x00 }; //- answers  - confirmation?
	sendCommand(commandAskSpeeds, 6);

}

void readSpeeds2() {

	// nowy dump:

	// kabelki białe - potrzebne do odpytania przez telefon o tab, np timelapse:
	// timelapse - te dwa (panning speed i tilting speed?
	// raz wczyta, pozniej pamieta - to chyba pytanie bo tylko jeden bajt?
	// A5 5A 02 0E 01 09 - tak odpytuje?
	//  A5 5A 02 0E 01 08
	// po restarcie gimba wczytuje od nowa
	// a teraz z zielonego po podłączeniu białych i wejściu na tę kartę:

	// to jest odpowiedz chyba o timelapse
	// A5 5A 03 0E 04 15 00 57 02 AF C1
	// A5 5A 03 0E 04 14 00 57 02 1B B7

	// a teraz to samo dla MOTORS:
	// biale:
	// A5 5A 02 0E 01 09
	// A5 5A 02 0E 01 08
	// // A5 5A 02 0E  01 07
	//tak!!!  i do FTDI232 podłączamy szary kabelek i nic,
	// otwieramy karte na tel. i dotykamy szarego bialym i wtedy rusza telemetria
	// (a więc zapewne biały wychodzi z ESP32 a szary idzie do STM32?)
	// zielony - z STM32? na poczatek pisze WG2X

	// motor strength, zielony:
	// pan i tilt polowa, roll axis max
	// A5 5A 03 0E 04 07 00 20 03
	// A5 5A 03 0E 04 08 00 20 4E  // ok taką odpowiedź dostalem!!!! tylko chyba bez pierwszego pola?
	// ANSWER2222!!!!!0 20 4E cos zle z odczytem?

	// A5 5A 03 0E 04 09 00 BE 01
	// ZGADZA SIE!!! 20 20 i potem BE

	// TAK oppowiedz to jest 03 0d 00, bo tyle razy!

	//0xA5, 0x5A, 0x02 , 0x0D , 0x03 , 0x16 , 0xBC , 0x02 , // save????

	// motor strength - 3 arg:
	// 02 0d 03 07 20 4e

	// odpytanie?
	// A5 5A 02 0E 01 12 // ?
	// chyba tak
	// 07, 08, 09 - motor strengths?
	// 12 scene smooth etc
	// 0d - joystick? 0c, 0a, 0b - te same indeksy // c,d - suwaki, a,b - przelaczniki
	//A5 5A 02 40 02 04 00
	//       uint8_t commandRecenter[] = { 0x02, 0x40, 0x02, 0x05, 0x02 };
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x40, 0x02, 0x04, 0x01 };
	//A5 5A 02 0D 03 - 12 01 00
	//A5 5A 03 40 01 04
	// 01 00    ....F..o  >D.c.Z..
	// 04 8C 13 CE - to odpowiedz?

	// ---------- teraz save
	//zielony STM32->ESP32
	//bialy - ESP32->
	// szary ->STM32

	// TILTING SPEED - od lewa do prawa
	// ustawienie wartosci: (a moze save?) // a moze odpytanie wartosci?
	// A5 5A 02 0E 01 0B
	// A5 5A 02 0E 01 0D
	// A5 5A 02 0E 01 0A
	// czy to wystarczy? wartosci jest chyba wiecej?
	// a to chyba ust. wartosci:
	// A5 5A 02 0D 03 0A 00 00
	// chyba tilting speed to 0C! // zakres 00-64? hex?
	//A5 5A 02 0D 03 0C 64 00

	// sprawdzone - to jest odpytanie?
	// chyba sa tylko dwa: odpytanie, i zapisz wartosc, jedna po drugiej
	// ruszanie suwakami nic nie zmienia, tylko lokalnie na tel.
	// A5 5A 02 0D 03 0C 64 00 (3arg) - zapisz wartosc!

	//const uint8_t commandAskSpeeds[] = { 0x03, 0x40, 0x01, 0x04}; // na to nie odpowiada nic
	const uint8_t commandAskSpeeds[] = { 0x02, 0x0e, 0x01, 0x0C }; // na to nie odpowiada nic
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x40, 0x02, 0x04, 0x02}; //- no answer to this
	//const uint8_t commandAskSpeeds[] = { 0x02, 0x0d, 0x03, 0x0B, 0x01, 0x00 }; //- answers  - confirmation?
	sendCommand(commandAskSpeeds, 4);

//	GimbalFrame frame;
//	bool wynik = getFrame(frame);
//	Serial.print("wynik = ");
//	Serial.print(wynik);
//	Serial.print("\n");
//	Serial.print(frame.contents[0], HEX);
//	Serial.print("\n");
//	Serial.print(frame.contents[1], HEX);
//	Serial.print("\n");
//	Serial.print(frame.contents[2], HEX);
//	Serial.print("\n");
//	while (1) {
//
//	}

	//A5 5A 02 0D    .Z......  .....Z..
	//03 0D 00 00 B7 73
	//A5 5A 02 0D 03 0C 00 00 // 03 - index?

	// save
	// A5 5A 02 0D 03 0B 01 00 26 F 0b - index, 01 00 - value, save
}

// the setup function runs once when you press reset or power the board
void setup() {

	pinMode(PIN_MAIN_BUTTON, INPUT);

	// initialize digital pin LED_BUILTIN as an output.
	gps_serial.begin(115200, SERIAL_8N1, RXD2, TXD2);
	pinMode(32, OUTPUT);
	digitalWrite(32, HIGH);   // turn the LED on (HIGH is the voltage level)

	//pinMode(14, OUTPUT);
	//digitalWrite(14 HIGH);   // turn the LED on (HIGH is the voltage level)

	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(LED_BUILTIN2, OUTPUT);
	// 19 - switch the power on
	pinMode(19, OUTPUT);
	digitalWrite(19, HIGH);   // turn the LED on (HIGH is the voltage level)

	pinMode(18, OUTPUT);
	digitalWrite(18, HIGH);   // turn the LED on (HIGH is the voltage level)
	Serial.begin(115200);

	sendCounter = 0;

	while (false) {
		// 19 or so - power off?
		// 23 green small?
		// 27 red front
		// 7, 8 - crash!
		// 14 green front?
		for (int i = 15; i < 20; i++) {
			if (i == 7)
				continue;
			if (i == 8)
				continue;
			pinMode(i, OUTPUT);
			Serial.print(i);
			Serial.print("\n");
			//digitalWrite(i, HIGH); // turn the LED on (HIGH is the voltage level)
			delay(1000);
			digitalWrite(i, LOW); // turn the LED on (HIGH is the voltage level)
			delay(1000);
		}
	}
	// 26 - JEDEN BUTTON (lewy)

	// 16 i 18 i 19 - main power button
	// 16 tylko gdy jest wlaczone USB, poza tym ciagle 1
	// 18 - wylaczenie?
	// 1: 17, 9, 10, 16, 17, 11, 21
	// analogInput: 12,13,14,15 25, 27, 32, 33, 34
	// 33 gdy podłączone USB to 0
	// z odlaczonym USB zatrzymuje sie na 18? TAK!,
	// 27 i 14 - nie to diody, 16 i 17 - serial, 32 led niebieska, 25 - led czerwona z boku
	// 15:00 25:77, 27: 286, 12: 283, 13: 113, 14: 247, 15: 133
	// 15:12 25: 77, 27: 286, 12: 276, 13: 115, 14:252, 32: 400, 33: 60, 34: 1453
	// 16:49 12: 279, 13: 119, 14: 252, 15: 144, 32: 390, 33: 66, 34: 1350
	// 17:51  12: 272 13: 117 14 - za duzy rozrzut, 15: 133 32: 400, 33: 66, 34: 1322,
	// 19:07 34: 1254
	// 19:16: 34: 1251
	// 19:45 34: 1231
	// 19:47 34: 1261 (po podłączeniu USB)
	// 20:03: 34: 1314
	// 23:38: 34: 1510

	while (false) {
		for (int i = 9; i < 39; i++) {
			if (i == 18)
				continue;
			long timer = millis();
			pinMode(i, INPUT);
			long current = millis();
			do {
				delay(100);
				Serial.print(i);
				Serial.print(" ");
				//Serial.print(digitalRead(i));
				Serial.print(analogRead(i));
				Serial.print("\n");
				current = millis();
			} while (timer + 3000 > current);
		}
	}

	delay(900);
	//readSpeeds();
}

// the loop function runs over and over again forever
void loop() {

	int pinMainButton = digitalRead(PIN_MAIN_BUTTON);
	Serial.print(pinMainButton);
	if (pinMainButton == 0) {
		Serial.print("POWER OFF!");

		for (int i = 15; i < 20; i++) {
			pinMode(i, OUTPUT);
			Serial.print(i);
			Serial.print("\n");
			//digitalWrite(i, HIGH); // turn the LED on (HIGH is the voltage level)
			delay(1000);
			digitalWrite(i, LOW);// turn the LED on (HIGH is the voltage level)
			delay(1000);
		}

	}

	if (!gimbReset) {

		gimbReset = true;

		recenterGimbal();
		delay(5000);
		//resetGimbal();
		//delay(1000);

	}

//	INNE
//

	if (!setPanningSpeed) {
		while (1) {
			readParameter(Parameters::PanningSpeed);
			delay(100);
			uint8_t firstField, secondField, thirdField;
			bool wynik = getGimbalResponse(Parameters::PanningSpeed, firstField,
					secondField, thirdField);
			if (wynik) {
				const uint8_t PANNING_SPEED_FIRST = 0x32;
				const uint8_t PANNING_SPEED_second = 0x00;
				if (secondField != PANNING_SPEED_FIRST
						&& thirdField != PANNING_SPEED_second) {
					Serial.print("PARAMETETR PANNINGSPEED ma Zla wartosc!");
					Serial.print(secondField, HEX);
					Serial.print(" ");
					Serial.print(thirdField, HEX);
					Serial.print("\n");
					setParameter(Parameters::PanningSpeed, PANNING_SPEED_FIRST,
							PANNING_SPEED_second);
				} else {
					Serial.print("PARAMETETR PANNINGSPEED ma dobra wartosc!");
					setPanningSpeed = true;
					break;
				}
			}
		}
	}

	if (!setTiltingSpeed) {
		while (1) {
			readParameter(Parameters::TiltingSpeed);
			delay(100);
			uint8_t firstField, secondField, thirdField;
			bool wynik = getGimbalResponse(Parameters::TiltingSpeed, firstField,
					secondField, thirdField);
			if (wynik) {
				const uint8_t TILTING_SPEED_FIRST = 0x32;
				const uint8_t TILTING_SPEED_second = 0x00;
				if (secondField != TILTING_SPEED_FIRST
						&& thirdField != TILTING_SPEED_second) {
					Serial.print("PARAMETETR TILTINGSPEED ma Zla wartosc!");
					Serial.print(secondField, HEX);
					Serial.print(" ");
					Serial.print(thirdField, HEX);
					Serial.print("\n");
					setParameter(Parameters::TiltingSpeed, TILTING_SPEED_FIRST,
							TILTING_SPEED_second);
				} else {
					Serial.print("PARAMETETR TILTINGSPEED ma dobra wartosc!");
					setTiltingSpeed = true;
					break;
				}
			}
		}
	}
	if (sendCounter < 5) {
		sendCommand(commandStartTm, 8);

		sendCounter++;
		delay(100);
		Serial.print("@@@@@@@@@@@@@@@@@@@@@!!START TM=\n");
	}

	bool wynik = false;
	GimbalState g;

	//for(int i = 0; i < 100; i++){

	wynik = getGimbalState(g);

//recenterGimbal();
//resetGimbal();
//	setRoll(3);
//	delay(1000);
//	return;

//delay(100);
	if (wynik) {

		double temp;

		temp = feyiuAngleTo360(
				(int) ((int) ((g.yaw)) - compensation[YAW_IDX]) % 360);
		;
		speed[YAW_IDX] = abs(temp - pos360[YAW_IDX]);
		pos360[YAW_IDX] = temp;

		temp = feyiuAngleTo360(
				(int) ((int) ((g.roll)) - compensation[ROLL_IDX]) % 360);
		;
		speed[ROLL_IDX] = abs(temp - pos360[ROLL_IDX]);
		pos360[ROLL_IDX] = temp;

		//feyiuAngleTo360(g.roll);//  - compensation[ROLL_IDX];
		temp = feyiuAngleTo360(
				(int) ((int) ((g.pitch)) - compensation[PITCH_IDX]) % 360);
		;
		speed[PITCH_IDX] = abs(temp - pos360[PITCH_IDX]);
		pos360[PITCH_IDX] = temp;
		//feyiuAngleTo360(g.pitch);//  - compensation[PITCH_IDX];
		//Serial.print("g.yaw=");
		//Serial.print(g.yaw);
		//Serial.print("\n");
		if (!calculatedCompensation) {
			calculatedCompensation = true;
			calculateCompensation();
			;
		}

		// not moving - so set the target and get going
		if (!isMoving[YAW_IDX] && !isMoving[PITCH_IDX]
				&& (millis() - moveTime > TIME_BETWEEN_STEPS)) {

			Serial.println("NEW_STEP---------------------------------\n");

			isMoving[YAW_IDX] = true;
			isMoving[PITCH_IDX] = true;
			isMoving[ROLL_IDX] = true;

			currentStep++;
			if (currentStep == numOfSteps) {
				currentStep = 0;
			}

			if (cameraMovesVector[currentStep][YAW_IDX] == -1
					&& cameraMovesVector[currentStep][ROLL_IDX] == -1
					&& cameraMovesVector[currentStep][PITCH_IDX] == -1) {
				recenterGimbal();
				delay(4000);
				return;
			}

			desired[ROLL_IDX] = cameraMovesVector[currentStep][ROLL_IDX];

			if (abs(cameraMovesVector[currentStep][YAW_IDX]) < 1000) {
				desired[YAW_IDX] = normalize360(
						pos360[YAW_IDX]
								+ cameraMovesVector[currentStep][YAW_IDX]);
			} else {
				desired[YAW_IDX] = cameraMovesVector[currentStep][YAW_IDX]
						% 1000;
			}

			if (abs(cameraMovesVector[currentStep][PITCH_IDX]) < 1000) {
				desired[PITCH_IDX] = normalize360(
						pos360[PITCH_IDX]
								+ cameraMovesVector[currentStep][PITCH_IDX]);
			} else {
				desired[PITCH_IDX] = cameraMovesVector[currentStep][PITCH_IDX]
						% 1000;
			}

//			pitchDirection = getCwOrCcw(desired[PITCH_IDX], pos360[PITCH_IDX],
//					PITCH_IDX);

//			yawDirection = getCwOrCcw(desired[YAW_IDX], pos360[YAW_IDX],
//					YAW_IDX);

//			rollDirection = getCwOrCcw(desired[ROLL_IDX], pos360[ROLL_IDX],
//					ROLL_IDX);

			Serial.print(
					"SETTING TARGETS!!!========================================");

			printReport();
			switchFrontLED(true);
		} else if ((millis() - sendTime > TIME_BETWEEN_MOVES)
				&& (isMoving[YAW_IDX] || isMoving[ROLL_IDX]
						|| isMoving[PITCH_IDX])) {
			sendTime = millis();
			Serial.print("DO MOVING=======");

			doMoving();
			printReport();
		}
	}

//	const char *p_buf = "abcd";
//	const unsigned int *p_int = reinterpret_cast<const unsigned int*>(p_buf);
}

