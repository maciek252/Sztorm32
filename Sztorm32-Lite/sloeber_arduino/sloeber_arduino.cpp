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
#define LED_BUILTIN 33
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

// YAW, ROLL, PITCH
const int cameraMovesVector[6][3] = { { 0, 0, -50 }, { -30, 0, 50 }, { 30, 0,
		-50 }, { 0, 0, 50 } };
const int numOfSteps = 4;
int currentStep = -1;

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

int pitchDirection;
int yawDirection;

double desired[3];
bool isMoving[3] = { false, false, false };
double pos360[3];

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

	delay(900);
}

double feyiuAngleTo360(double angle) {
	if (angle > 0)
		return angle;

	return 360 + angle;
}

uint8_t* prepareMoveCommand(uint8_t *commandZero, int yaw, int pitch) {

	if (yaw > 0)
		commandZero[4] = 0xff;
	else if (yaw < 0)
		commandZero[5] = 0xff;

	// this moves upwards
	if (pitch > 0)
		commandZero[6] = 0xff;
	else if (pitch < 0)
		commandZero[7] = 0xff;
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
				//Serial.print(receiveLength);
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

						g.yaw = ((double) posYaw) / 100.0;
						g.roll = ((double) posRoll) / 100.0;
						g.pitch = ((double) posPitch) / 100.0;
						//Serial.print("message parsed (without the CRC)\n");
						return true;
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

void setRoll() {
	//02 0D 03 16 BC 02
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
	// 20fe - malo ale w prawo
// 20fa - mocno w prawo!

	uint8_t commandRoll[] = { 0x02, 0x0d, 0x03, 0x16, 0x20, 0xfa };
	//commandRoll[3] = 0x16; // to chyba indeks tej operacji
	//commandRoll[4] = 0x20;
	//commandRoll[5] = 0x03;



	// 9020 tez nic, nie wiem czy poprzednie dobre? 0302 nie dziala!
	// zle indeksy byly, jeszcze raZ!
	// 0302 w zasadzie nic moze trzeba save?
	//sendCommand(commandShort, 8);
	Serial.print("SETTING ROLL=\n");
	sendCommand(commandRoll, 6);


	// this is starttm
	//uint8_t commandSave[] = { 0x00, 0x10, 0x05, 0xff, 0x00, 0x00, 0x00, 0x00 };
	//sendCommand(commandSave, 8);
}

void doMovingPitchAndYaw() {

	int yawMove = 0;
	int pitchMove = 0;

	if (isMoving[YAW_IDX]) {
		if (abs(desired[YAW_IDX] - pos360[YAW_IDX]) > 10) {

			Serial.print(
					"===========================YAW MOVE!!==========================!\n");
			yawMove = 256;
			//int yawDirection = getCwOrCcw(desired[YAW_IDX], pos360[YAW_IDX]);
			if (yawDirection > 0) {
				yawMove = -256;
			}

		} else {
			Serial.print(
					"===========================REACHED YAW TARGET!!==========================!\n");
			isMoving[YAW_IDX] = false;
			moveTime = millis();

		}
	}
	if (isMoving[PITCH_IDX]) {
		if (abs(desired[PITCH_IDX] - pos360[PITCH_IDX]) > 10) {

			pitchMove = 256;
//			int pitchDirection = getCwOrCcw(desired[PITCH_IDX],
//					pos360[PITCH_IDX]);
			if (pitchDirection > 0) {
				pitchMove = -256;
			}
		} else {
			Serial.print(
					"===========================REACHED PITCH TARGET!!==========================!\n");
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

}

void printReport() {
	Serial.print("CURRENT 360 Y=");
	Serial.print(pos360[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(pos360[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(pos360[PITCH_IDX]);
	Serial.print("  DESIRED Y=");
	Serial.print(desired[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(desired[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(desired[PITCH_IDX]);
	Serial.print("  IS_MOVING Y=");
	Serial.print(isMoving[YAW_IDX]);
	Serial.print(" R=");
	Serial.print(isMoving[ROLL_IDX]);
	Serial.print(" P=");
	Serial.print(isMoving[PITCH_IDX]);
	Serial.print("currStep=");
	Serial.print(currentStep);
	Serial.print("\n");
}

void doMoving() {

	doMovingPitchAndYaw();
	//doMovingRoll();

}

// the loop function runs over and over again forever
void loop() {

	byte bajty[30];

	if (sendCounter < 50) {
		sendCommand(commandStartTm, 8);
		sendCounter++;
		delay(100);
		Serial.print("@@@@@@@@@@@@@@@@@@@@@!!START TM=\n");
	} else {
//	sendCounter = 0;
	}

	GimbalState g;

	bool wynik = getGimbalState(g);


	//recenterGimbal();
	//resetGimbal();
	setRoll();
	delay(1000);
	return;

//delay(100);
	if (wynik) {

		pos360[YAW_IDX] = feyiuAngleTo360(g.yaw);
		pos360[ROLL_IDX] = feyiuAngleTo360(g.roll);
		pos360[PITCH_IDX] = feyiuAngleTo360(g.pitch);

		// not moving - so set the target and get going
		if (!isMoving[YAW_IDX] && !isMoving[PITCH_IDX]
				&& (millis() - moveTime > 5000)) {



			moveTime = millis();
			isMoving[YAW_IDX] = true;
			isMoving[PITCH_IDX] = true;

			currentStep++;
			if (currentStep == numOfSteps) {
				currentStep = 0;
			}
			desired[YAW_IDX] = normalize360(
					pos360[YAW_IDX] + cameraMovesVector[currentStep][YAW_IDX]);
			desired[PITCH_IDX] = normalize360(
					pos360[PITCH_IDX]
							+ cameraMovesVector[currentStep][PITCH_IDX]);

			pitchDirection = getCwOrCcw(desired[PITCH_IDX], pos360[PITCH_IDX],
					PITCH_IDX);

			yawDirection = getCwOrCcw(desired[YAW_IDX], pos360[YAW_IDX],
					YAW_IDX);

			Serial.print(
					"SETTING TARGETS!!!========================================");

			printReport();

		}

		if ((millis() - sendTime > 500)) {
			sendTime = millis();
			Serial.print("DO MOVING==================================");

			doMoving();
			printReport();
		}
	}

//	const char *p_buf = "abcd";
//	const unsigned int *p_int = reinterpret_cast<const unsigned int*>(p_buf);
}

