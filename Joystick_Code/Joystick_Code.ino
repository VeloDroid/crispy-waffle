
// Pin Varibles for Joystick

const int turn = A1; //Y
const int UD = A0; //X
const int sel = A2; //Sel

void setup() {
// put your setup code here, to run once:
pinMode(turn,INPUT);
pinMode(UD,INPUT);
pinMode(sel,INPUT);
Serial.begin(9600);

}
void loop() {

int Chkturn = analogRead(turn); //1023=Left, 521=neutral,0=Right
int ChkUD = analogRead(UD);//1023=UP, 521=neutral, 0=Down
int Chksel = analogRead(sel);//489=unpressed, 0 pressed.

Serial.println(Chkturn);
Serial.println(ChkUD);
Serial.println(Chksel);
Serial.println();
delay(500);
}
