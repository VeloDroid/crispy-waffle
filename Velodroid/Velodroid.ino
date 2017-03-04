//THis is the one from github!
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(13, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(13, 10, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
strip1.begin();
strip1.show();
strip2.begin();
strip2.show();
}

void loop(){
  // put your main code here, to run repeatedly:
// for (int k=0; k<60; k++){ 
//    strip1.setPixelColor(k, 255, 0, 255);
//    strip1.show();
//  }
//  clear1();
//  delay(10000);


rightturn();

}

void clear1(){
  for (int k=0; k<13; k++){
    strip1.setPixelColor(k,0,0,0);
    strip1.show();
  }
}

void clear2(){
  for (int k=0; k<13; k++){
    strip2.setPixelColor(k,0,0,0);
    strip2.show();
  }
}

void leftturn(){
  for (int k = 0; k<13; k++){
    strip1.setPixelColor(k,255,191,0);
    strip1.show();
    delay(66);
  }
  clear1();
}


void rightturn(){
  for (int k = 0; k<13; k++){
    strip2.setPixelColor(k,255,191,0);
    strip2.show();
    delay(66);
  }
  clear2();
}

void hazard(){
  //This will eventually be controlled by left right up down inputs from the joystick
  //
  for (int i=0; i<10; i++) {
    for (int k=0; k<13; k++){
      strip1.setPixelColor(k,255,191,0);
      strip2.setPixelColor(k,255,191,0);
    }
    strip1.show();
    strip2.show();
    delay(400);
    clear1();
    clear2();
  }
}

void brake(){
  for (int k=0; k<13; k++){
    strip1.setPixelColor(k,255,0,0);
    strip2.setPixelColor(k,255,0,0);
  }
  strip1.show();
  strip2.show();
  delay(5000);
}

