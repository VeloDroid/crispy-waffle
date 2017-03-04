//This is the one from github!
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(13, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(13, 10, NEO_GRB + NEO_KHZ800);

//Global Joystick Variables
int Chkturn; //1023=Left, 521=neutral,0=Right
int ChkUD;//1023=UP, 521=neutral, 0=Down
int Chksel;//489=unpressed, 0 pressed.
bool brakebool = false;

// Pin Varibles for Joystick
const int turn = A1; //Y
const int UD = A0; //X
const int sel = A2; //Sel

void setup() {
  //Initialize the LED strips
    strip1.begin();
    strip1.show();
    strip2.begin();
    strip2.show();

  //Initialize the Joystick pins
    pinMode(turn,INPUT);
    pinMode(UD,INPUT);
    pinMode(sel,INPUT);
}

void loop(){
Chkturn = analogRead(turn); //1023=Left, 521=neutral,0=Right
ChkUD = analogRead(UD);//1023=UP, 521=neutral, 0=Down
Chksel = analogRead(sel);//489=unpressed, 0 pressed.

  if (Chkturn > 900){
    while(Chksel > 300){
      leftturn();
      Chksel = analogRead(sel);//489=unpressed, 0 pressed.
    }  
  }
  
  else if (Chkturn <200){
    while(Chksel > 300){
      rightturn(); 
      Chksel = analogRead(sel);//489=unpressed, 0 pressed.
    }
  }

  else if (ChkUD > 900){
    while(Chksel > 300){
      hazard();
      Chksel = analogRead(sel);//489=unpressed, 0 pressed.
    }
  }
//    else if (ChkUD < 300){
//      brakebool=!brakebool;
//    if (brakebool){
//      brake();
//    }
//    //else{clear1();clear2();}
//  }
  
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
  if (brakebool){
    brake();
  }
  for (int k = 0; k<13; k++){
    strip1.setPixelColor(k,80,40,0);
    strip1.show();
    delay(66);
  }
  clear1();
  if (brakebool){
    brake();
  }
}


void rightturn(){
  if (brakebool){
    brake();
  }
  for (int k = 0; k<13; k++){
    strip2.setPixelColor(k,80,41,0);
    strip2.show();
    delay(66);
  }
  clear2();
  if (brakebool){
    brake();
  }
}

void hazard(){
  //This will eventually be controlled by left right up down inputs from the joystick
    for (int k=0; k<13; k++){
      strip1.setPixelColor(k,80,40,0);
      strip2.setPixelColor(k,80,40,0);
    }
    strip1.show();
    strip2.show();
    delay(400);
    clear1();
    clear2();
    delay(400);
  }


void brake(){
    for (int k=0; k<13; k++){
      strip1.setPixelColor(k,100,0,0);
      strip2.setPixelColor(k,100,0,0);
    }
    strip1.show();
    strip2.show();
}

