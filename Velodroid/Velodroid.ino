//This is the one from github!
#include <Adafruit_NeoPixel.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(13, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(13, 10, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(3, 5, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4 = Adafruit_NeoPixel(3, 6, NEO_GRB + NEO_KHZ800);

//Global Joystick Variables
int Chkturn; //1023=Left, 521=neutral,0=Right
int ChkUD;//1023=UP, 521=neutral, 0=Down
int Chksel;//489=unpressed, 0 pressed.
bool brakebool = false;
bool leftturnsignal = false;
bool rightturnsignal = false;
bool hazards = false;
bool cancel = false;

// Pin Varibles for Joystick
const int turn = A1; //Y
const int UD = A0; //X
const int sel = A2; //Sel

//Accelerometer
//MMA8452Q accel;
float offset = 0;
bool firstTime = true;
float sigValuePos = 0;
float sigValueNeg = 0;
float avgaccel = 0.00;

//Bluetooth
byte turnSig;
byte whichTurnSig;
byte howClose;
byte data;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_REALACCEL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//ISR
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





void setup() {
  //Initialize the LED strips
  Serial.begin(9600);
  Serial.println("Hello there!");
  strip1.begin();
  strip1.show();
  strip2.begin();
  strip2.show();
  strip3.begin();
  strip3.show();
  strip4.begin();
  strip4.show();
  Serial.println("I did LED stuff");


  //Initialize the Joystick pins
  pinMode(turn, INPUT);
  pinMode(UD, INPUT);
  pinMode(sel, INPUT);
Serial.println("I did joystick stuff");
  //Initialize Accelerometer
  //accel.init();
Serial.println("I made it through setup");


Serial.println("starting i2c styff");
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}

void loop() {
    if (Serial.available() > 0)     // Send data only when you receive data:
    {
      Serial.println("you");
      data = Serial.read();        //Read the incoming data & store into data
      turnSig = data & B00000001;
      Serial.println(turnSig);
      whichTurnSig = data & B00000010;
      Serial.println(whichTurnSig);
      howClose = data & B00110000;
      Serial.println(howClose);
      clear3();
      clear4();
      if(turnSig == 0){leftturnsignal = false; rightturnsignal = true;}
      if(turnSig == 1 && whichTurnSig ==0){strip3.setPixelColor(2,0,0,255);strip3.show();}
      if(turnSig == 1 && whichTurnSig ==2){strip3.setPixelColor(0,0,0,255);strip3.show();}
      if(howClose == 16){strip4.setPixelColor(2,0,255,0);strip4.show();}
      if(howClose == 32){strip4.setPixelColor(2,0,255,0);strip4.setPixelColor(1,0,255,0);strip4.show();}
      if(howClose == 48){strip4.setPixelColor(2,0,255,0);strip4.setPixelColor(1,0,255,0);strip4.setPixelColor(0,0,255,0);strip4.show();}
      
    }

    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

         OUTPUT_READABLE_REALACCEL
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
//        
//
//        // blink LED to indicate activity
//        blinkState = !blinkState;
//        digitalWrite(LED_PIN, blinkState);
    }
  
//  if(firstTime) {
//    Serial.println("here comes the offset!");
//    Serial.println(accel.cx, 3);
//    sigValuePos = accel.cx + 0.1;
//        sigValueNeg = accel.cx - 0.1;
//
//    Serial.println(sigValueNeg);
//    Serial.println("finished setup");
//    firstTime = false;
//  }
  
  Chkturn = analogRead(turn); //1023=Left, 521=neutral,0=Right
  ChkUD = analogRead(UD);//1023=UP, 521=neutral, 0=Down
  Chksel = analogRead(sel);//489=unpressed, 0 pressed.


//  if (accel.available()) {
//    checkaccel();
//    Serial.print("sigValueNeg is:  ");
//    Serial.print(sigValueNeg);
//    if( avgaccel < sigValueNeg) {
//      brakebool = true;
//      brake();
//    }
//    else{clear1();clear2();brakebool=false;}
//  }
  //if (turnSig ==0){ leftturnsignal = false; rightturnsignal = false;} 
  if (Chkturn < 200 || leftturnsignal == true || (whichTurnSig == 0 && turnSig == 1)) {
      leftturnsignal = true;
      rightturnsignal = false;
      hazards=false;
      leftturn();
  }

  if (Chkturn > 900 || rightturnsignal == true || (whichTurnSig == 2 && turnSig == 1)) {
      rightturnsignal=true;
      leftturnsignal=false;
      hazards=false;
      rightturn();
  }

  else if (ChkUD > 900 || hazards == true) {
      hazards = true;
      rightturnsignal=false;
      leftturnsignal=false;
      hazard();
  }

  if (Chksel < 100){
    delay(100);
    Chksel = analogRead(sel);//489=unpressed, 0 pressed;
    if (Chksel < 100){
      cancelall();
      Serial.println("I cancelled all");
      clear1();
      clear2();
      clear3();
      clear4();
  }
  }
}











void clear1() {
  for (int k = 0; k < 13; k++) {
    strip1.setPixelColor(k, 0, 0, 0);
    strip1.show();
  }
}

void clear2() {
  for (int k = 0; k < 13; k++) {
    strip2.setPixelColor(k, 0, 0, 0);
    strip2.show();
  }
}

void clear3() {
  for (int k = 0; k < 3; k++) {
    strip3.setPixelColor(k, 0, 0, 0);
    strip3.show();
  }
}

void clear4() {
  for (int k = 0; k < 3; k++) {
    strip4.setPixelColor(k, 0, 0, 0);
    strip4.show();
  }
}

void leftturn() {
  if (brakebool) {
    brake();
  }
  strip3.setPixelColor(2,0,0,255);
  strip3.show;
  for (int k = 0; k < 13; k++) {
    strip1.setPixelColor(k, 255, 190, 0);
    strip1.show();
    delay(66);
  }
  clear1();
  if (brakebool) {
    brake();
  }
}


void rightturn() {
  if (brakebool) {
    brake();
  }
  strip3.setPixelColor(0,0,0,255);
  strip3.show();
  for (int k = 0; k < 13; k++) {
    strip2.setPixelColor(k, 255, 190, 0);
    strip2.show();
    delay(66);
  }
  clear2();
  if (brakebool) {
    brake();
  }
}

void hazard() {
  //This will eventually be controlled by left right up down inputs from the joystick
  for (int k = 0; k < 13; k++) {
    strip1.setPixelColor(k, 255, 190, 0);
    strip2.setPixelColor(k, 255, 190, 0);
  }
  strip1.show();
  strip2.show();
  delay(400);
  clear1();
  clear2();
  delay(400);
}


void brake() {
  for (int k = 0; k < 13; k++) {
    strip1.setPixelColor(k, 255, 0, 0);
    strip2.setPixelColor(k, 255, 0, 0);
  }
  strip1.show();
  strip2.show();
}

//void checkaccel(){
//  accel.read();
//  float time1 = accel.cx;
//  delay(50);
//  accel.read();
//  float time2 = accel.cx;
//  delay(50);
//  accel.read();
//  float time3 = accel.cx;
//  delay(50);
//  accel.read();
//  float time4 = accel.cx;
//  delay(50);
//  accel.read();
//  float time5 = accel.cx;
//  delay(50);
//  accel.read();
//  float time6 = accel.cx;
//  avgaccel = (time1+time2+time3+time4+time5+time6)/6;
//  Serial.println(avgaccel);
//}

void cancelall(){
  rightturnsignal = false;
  leftturnsignal = false;
  //brakebool = false;
  hazards = false;
}




