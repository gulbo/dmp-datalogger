// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#define DEBUG 1

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <SD.h>
#include "Wire.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL
#define OUTPUT_TIME
#define OUTPUT_TO_SD

#define INTERRUPT_PIN 4  // use pin 2 on Arduino Uno & most board
#define MOSI_PIN 23
#define MISO_PIN 19
#define SCK_PIN 18
#define CS_PIN 5
#define SD_LED_PIN 15
#define MPU_LED_PIN 13
#define BUTTON_PIN 12

#ifdef OUTPUT_TO_SD
	#define OUT myFile
#else
	#define OUT Serial
#endif

const char* ssid     = "GULPO";
const char* password = "f117f117bagonghi";

// class default I2C address is 0x68
MPU6050 mpu;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// sd card
SPIClass mySPI(VSPI);
File myFile;
String fileName = "datalog";

inline void led_blink() {blinkState = !blinkState;digitalWrite(BUILTIN_LED, blinkState);}
inline void led_on(int8_t pin) {blinkState = 1; digitalWrite(pin, blinkState);}
inline void led_off(int8_t pin) {blinkState = 0; digitalWrite(pin, blinkState);}

void wait_button() {
	led_on(BUILTIN_LED);
	while (!digitalRead(BUTTON_PIN));
	led_off(BUILTIN_LED);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dataReady_isr() {
   mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	// configure LEDs and BUTTONs
	pinMode(BUILTIN_LED, OUTPUT);
	pinMode(SD_LED_PIN, OUTPUT);
	pinMode(MPU_LED_PIN, OUTPUT);
	led_off(SD_LED_PIN);
	led_off(MPU_LED_PIN);
	pinMode(BUTTON_PIN,INPUT_PULLDOWN);

	// initialize serial communication
	Serial.begin(115200);
	while (!Serial); // wait for Leonardo enumeration, others continue immediately
	
	Serial.println("Press button to start!");
	wait_button();
	Serial.println("Start!");

	// Connect to Wi-Fi network with SSID and password
	Serial.print("Connecting to ");
	Serial.print(ssid);
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) { 
		led_blink();
		delay(300); //wifi blinks every 300ms
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected.");

	// Initialize a NTPClient to get time
	led_on(BUILTIN_LED); //led on while updating time
	Serial.print("Initializing time client...");
	delay(1000);
	timeClient.begin();
	// GMT+1=3600  1/1/19=1546300800 SummerTime=3600
	timeClient.setTimeOffset(3600-1546300800+3600);
	timeClient.setUpdateInterval(INT_MAX); //never update! update only in setup
	//update timer
	while (!timeClient.forceUpdate()){
		led_blink();
		Serial.print("."); //NTPclient blinks every second
	}
	Serial.println();
	Serial.println("Time updated!");
	led_off(BUILTIN_LED);

	//turn off wifi
	WiFi.disconnect();
	WiFi.mode(WIFI_OFF);
	Serial.println("Wifi turned off.");
	delay(1000);

	// SD card
	#ifdef OUTPUT_TO_SD
		led_on(SD_LED_PIN);
		mySPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
		Serial.print("Initializing SD card...");
		while (!SD.begin(CS_PIN, mySPI)) {
			Serial.println(".");
			delay(400); 
		}
		Serial.println("initialization done.");
		uint8_t cardType = SD.cardType();
		if(cardType == CARD_NONE){
			Serial.println("No SD card attached");
			while(1);
		}
		fileName = "/" + timeClient.getFormattedDate() + "_" + fileName;
		String incremental_fileName = fileName;
		while (SD.exists(incremental_fileName + ".txt")){
			Serial.println(incremental_fileName + " exists!");
			static int8_t num = 1;
			incremental_fileName = fileName + num;
			num++;
		}
		fileName = incremental_fileName;
		Serial.println(String("New file: ") + fileName);
		myFile = SD.open(fileName + ".txt", FILE_APPEND);	
	#endif


	OUT.print("***************** ");
	OUT.print(timeClient.getFormattedDate() + " - ");
	OUT.print(timeClient.getFormattedTime());
	OUT.println(" *****************");
	#ifdef OUTPUT_TO_SD
		myFile.close();
		delay(500); //signal SD checked keeping led on for 500ms
	#endif
	// turn on the SD led to signal all good
	led_off(SD_LED_PIN);

	// MPU
	led_on(MPU_LED_PIN);
	Serial.print("Initializing MPU...");
	while(!mpu.testConnection()){
		Serial.print(".");
		delay(300);
	}
	Serial.println();
	mpu.initialize();
	Serial.println("MPU ready.");
	pinMode(INTERRUPT_PIN, INPUT);
	// signal MPU checked keepeing the led on for 500ms
	delay(500);
	led_off(MPU_LED_PIN);

	// wait for ready
	Serial.println(F("\nPress the button to begin!"));
	wait_button();

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true );

		//set Low-Pass filter
		Serial.print("Digital low-pass filter: ");
		Serial.println(mpu.getDLPFMode());
		mpu.setRate(7); // 1khz / (1 + 11) = 83Hz but EMPIRICALLY, 38HZ!!!
						 // 1khz / (1 + 7) = 125hz but EMPIRICALLY 63HZ!!!

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dataReady_isr, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("Setup: DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) {
	  Serial.println("DMP not ready!");
	  while(1);
	}

   // wait for MPU interrupt or extra packet(s) available
   while (!mpuInterrupt && fifoCount < packetSize) {
	   if (mpuInterrupt && fifoCount < packetSize) {
		 // try to get out of the infinite loop 
		 fifoCount = mpu.getFIFOCount();
	   }  
	   // other program behavior stuff here
	   // .
	   // .
	   // .
	   // if you are really paranoid you can frequently test in between other
	   // stuff to see if mpuInterrupt is true, and if so, "break;" from the
	   // while() loop to immediately process the MPU data
	   // .
	   // .
	   // .
   }

   // reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		Serial.println(F("FIFO overflow!"));

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		#ifdef OUTPUT_TO_SD
			myFile = SD.open(fileName + ".txt", FILE_APPEND);
		#endif

		#ifdef OUTPUT_TIME
		  OUT.print(timeClient.getFormattedTime());
		  OUT.print("s - ");
		#endif
		
		#ifdef OUTPUT_READABLE_QUATERNION
			// display quaternion values in easy matrix form: w x y z
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			Serial.print("quat\t");
			Serial.print(q.w);
			Serial.print("\t");
			Serial.print(q.x);
			Serial.print("\t");
			Serial.print(q.y);
			Serial.print("\t");
			Serial.println(q.z);
		#endif

		#ifdef OUTPUT_READABLE_EULER
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetEuler(euler, &q);
			Serial.print("euler\t");
			Serial.print(euler[0] * 180/M_PI);
			Serial.print("\t");
			Serial.print(euler[1] * 180/M_PI);
			Serial.print("\t");
			Serial.println(euler[2] * 180/M_PI);
		#endif

		#ifdef OUTPUT_READABLE_YAWPITCHROLL
			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			Serial.print("ypr\t");
			Serial.print(ypr[0] * 180/M_PI);
			Serial.print("\t");
			Serial.print(ypr[1] * 180/M_PI);
			Serial.print("\t");
			Serial.println(ypr[2] * 180/M_PI);
		#endif

		#ifdef OUTPUT_READABLE_REALACCEL
			// display real acceleration, adjusted to remove gravity
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			Serial.print("areal\t");
			Serial.print(aaReal.x);
			Serial.print("\t");
			Serial.print(aaReal.y);
			Serial.print("\t");
			Serial.println(aaReal.z);
		#endif

		#ifdef OUTPUT_READABLE_WORLDACCEL
			// display initial world-frame acceleration, adjusted to remove gravity
			// and rotated based on known orientation from quaternion
			// static int64_t loops = 0;
			// static uint64_t start = millis();
			// loops++;
			// OUT.print(loops*1.0/(millis()-start)*1000);
			// OUT.print("Hz - ");
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			OUT.print(aaWorld.x);
			OUT.print(" ");
			OUT.print(aaWorld.y);
			OUT.print(" ");
			OUT.println(aaWorld.z);
		#endif

		#ifdef OUTPUT_TO_SD
			myFile.close();
		#endif

		//in case of button pressed, suspend
		if(digitalRead(BUTTON_PIN)){
			led_on(BUILTIN_LED);
			delay(1000);
			wait_button();
			delay(200);
		}
		// blink LED to indicate activity
		led_blink();
	}
}