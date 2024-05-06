#include "Wire.h"
#include <LiquidCrystal.h>
#define SLAVE_ADDRESS 0x08

// For LCD code to work
const int rs = 13, en = 12, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
void HappyEyes();
void inverted_HappyEyes();
void BlinkHappy();
void inverted_BlinkHappy();
void MadEyes();
void inverted_MadEyes();
void BlinkMad();
void inverted_BlinkMad();
void Sleep();
void inverted_Sleep();
byte data_to_echo = 0;
void (*FuncAr[])() = {inverted_HappyEyes, inverted_BlinkHappy, inverted_MadEyes, inverted_BlinkMad, inverted_Sleep};

// For motor code
// Define the speed at which you want to move
int maxspeed = 100;

// Define the pins used for the motor controller
int lf_motor[] = {50,52,7};
int rf_motor[] = {48,46,6};
int lb_motor[] = {51,53,5};
int rb_motor[] = {49,47,4};
int tail[] = {44,45,3};

int straight, side, rot;
int spd[] = {100,70,50};
int pastcommand = 6;  // Default stop

// Define function type
typedef void (*FunctionPointer)();

// Motor control functions
void movestraight();
void movesideways();
void rotatebot();
void spinmotor(int speed, int pin1, int pin2, int pwmpin);

// Utilities
int pwm_conv(int x);
int findNonZero(int vec[]);
bool areVectorsIdentical(int vec1[], int vec2[], int size);

void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  for (int i = 0; i < 3; i++) {
  pinMode(lf_motor[i], OUTPUT);
 }
  for (int i = 0; i < 3; i++) {
  pinMode(rf_motor[i], OUTPUT);
 }
 for (int i = 0; i < 3; i++) {
  pinMode(lb_motor[i], OUTPUT);
 }
  for (int i = 0; i < 3; i++) {
  pinMode(rb_motor[i], OUTPUT);
 }
 for (int i = 0; i < 3; i++) {
  pinMode(tail[i], OUTPUT);
 }
 pinMode(A0, INPUT);
 pinMode(A1, INPUT);
 pinMode(A2, INPUT);
 
 pinMode(A3, INPUT); // tail command


 Serial.begin(9600);
 straight =0;
 movestraight();

}

void loop() {

//  LCD
  FuncAr[data_to_echo]();



//  Tail
  int spin_tail = digitalRead(A3);
  if(spin_tail != 0){
    spinmotor(-50, tail[0],tail[1],tail[2]);
  }
  else {
    spinmotor(0,tail[0],tail[1],tail[2]);}
    

//  Movement

  int pin_input[] = {digitalRead(A0),digitalRead(A1), digitalRead(A2)};
  int sum = 0;
  int mult[] = {1,3,5};
  for(int i = 0; i<3; i++) {
    sum = sum + pin_input[i] * mult[i];
  }

  int c_vals[] = {1,3,5,4,8,6,0}; // {for,bac,right,left,CW,CCW,Stop}
  FunctionPointer functions[] = {movestraight,movestraight,movesideways,movesideways, rotatebot, rotatebot,movestraight};
  
  for(int i = 0; i<7; i++) {
    if(sum == c_vals[i]) {
        if (i%2 !=0){
          straight = -1*spd[0];
          side = -1*spd[1];
          rot = -1*spd[2];
        } else if(i == 6) {
          straight = 0;
          side = 0;
          rot = 0;
        }
        else{
          straight = spd[0];
          side = spd[1];
          rot = spd[2];
        }
        if (i != pastcommand) {
          Serial.println(i);
          functions[i]();
        }
        pastcommand = i;
        break;
    }
  }

}


void receiveData(int bytecount){
  for (int i = 0; i < bytecount; i++) {
    data_to_echo = Wire.read();
  }
}


void Write(){
  lcd.setCursor(5, 0);
  lcd.write((byte)0);
  lcd.write((byte)1);
  lcd.setCursor(9, 0);
  lcd.write((byte)4);
  lcd.write((byte)5);
  lcd.setCursor(5, 1);
  lcd.write((byte)2);
  lcd.write((byte)3);
  lcd.setCursor(9, 1);
  lcd.write((byte)6);
  lcd.write((byte)7);
}

void HappyEyes(){
  byte HappyL1[8] = {0b00000,0b00011,0b01100,0b00000,0b00011,0b00110,0b01100,0b01000};
  byte HappyL2[8] = {0b11100,0b00000,0b00000,0b00000,0b11000,0b01100,0b00110,0b00010};
  byte HappyL3[8] = {0b01000,0b01000,0b01000,0b01001,0b01011,0b01111,0b00111,0b00011};
  byte HappyL4[8] = {0b00010,0b01010,0b00010,0b10010,0b11010,0b11110,0b11100,0b11000};
  byte HappyR1[8] = {0b00111,0b00000,0b00000,0b00000,0b00011,0b00110,0b01100,0b01000};
  byte HappyR2[8] = {0b00000,0b11000,0b00110,0b00000,0b11000,0b01100,0b00110,0b00010};
  byte HappyR3[8] = {0b01000,0b01000,0b01000,0b01001,0b01011,0b01111,0b00111,0b00011};
  byte HappyR4[8] = {0b00010,0b01010,0b00010,0b10010,0b11010,0b11110,0b11100,0b11000};
  byte* Happy[] = {HappyL1,HappyL2,HappyL3,HappyL4,HappyR1,HappyR2,HappyR3,HappyR4};
  
  for(int i=0; i<8; i++) {
    lcd.createChar(i, Happy[i]);
  }
  Write(); 
}

void inverted_HappyEyes(){
  byte HappyL1[8] = {0b00011, 0b00111, 0b01111, 0b01011, 0b01001, 0b01000, 0b01010, 0b01000}; // Reversed bits in HappyR4
  byte HappyL2[8] = {0b11000, 0b11100, 0b11110, 0b11010, 0b10010, 0b00010, 0b00010, 0b00010}; // Reversed bits in HappyR3
  byte HappyL3[8] = {0b01000, 0b01100, 0b00110, 0b00011, 0b00000, 0b01100, 0b00011, 0b00000}; // Reversed bits in HappyR2
  byte HappyL4[8] = {0b00010, 0b00110, 0b01100, 0b11000, 0b00000, 0b00000, 0b00000, 0b11100}; // Reversed bits in HappyR1
  byte HappyR1[8] = {0b00011, 0b00111, 0b01111, 0b01011, 0b01001, 0b01000, 0b01010, 0b01000}; // Reversed bits in HappyL4
  byte HappyR2[8] = {0b11000, 0b11100, 0b11110, 0b11010, 0b10010, 0b00010, 0b00010, 0b00010}; // Reversed bits in HappyL3
  byte HappyR3[8] = {0b01000, 0b01100, 0b00110, 0b00011, 0b00000, 0b00000, 0b00000, 0b00111}; // Reversed bits in HappyL2
  byte HappyR4[8] = {0b00010, 0b00110, 0b01100, 0b11000, 0b00000, 0b00110, 0b11000, 0b00000}; // Reversed bits in HappyL1
  byte* Happy[] = {HappyL1, HappyL2, HappyL3, HappyL4, HappyR1, HappyR2, HappyR3, HappyR4};
  
  for(int i=0; i<8; i++) {
    lcd.createChar(i, Happy[i]);
  }
  Write(); 
  }
  
void MadEyes(){
  byte MadL1[8] = { 0b00111, 0b00000, 0b00000, 0b00000, 0b00011, 0b00110, 0b01100, 0b01000 };
  byte MadL2[8] = { 0b10000, 0b11100, 0b00110, 0b00000, 0b11000, 0b01100, 0b00110, 0b00010 };
  byte MadL3[8] = { 0b01000, 0b01000, 0b01000, 0b01001, 0b01011, 0b01111, 0b00111, 0b00011 };
  byte MadL4[8] = { 0b00010, 0b01010, 0b00010, 0b10010, 0b11010, 0b11110, 0b11100, 0b11000 };
  byte MadR1[8] = { 0b00001, 0b00111, 0b01100, 0b00000, 0b00011, 0b00110, 0b01100, 0b01000 };
  byte MadR2[8] = { 0b11100, 0b00000, 0b00000, 0b00000, 0b11000, 0b01100, 0b00110, 0b00010 };
  byte MadR3[8] = { 0b01000, 0b01000, 0b01000, 0b01001, 0b01011, 0b01111, 0b00111, 0b00011 };
  byte MadR4[8] = { 0b00010, 0b01010, 0b00010, 0b10010, 0b11010, 0b11110, 0b11100, 0b11000 };
  byte* Mad[] = {MadL1,MadL2,MadL3,MadL4,MadR1,MadR2,MadR3,MadR4};

  for(int i=0; i<8; i++) {
    lcd.createChar(i, Mad[i]);
  }
  Write(); 
}
void inverted_MadEyes() {
  byte MadL1[8] = {0b00011, 0b00111, 0b01111, 0b01011, 0b01001, 0b01000, 0b01010, 0b01000}; // Reversed bits in HappyR4
  byte MadL2[8] = {0b11000, 0b11100, 0b11110, 0b11010, 0b10010, 0b00010, 0b00010, 0b00010}; // Reversed bits in HappyR3
  byte MadL3[8] = {0b01000, 0b01100, 0b00110, 0b00011, 0b00000, 0b00000, 0b00000, 0b00111}; // Reversed bits in HappyR2
  byte MadL4[8] = {0b00010, 0b00110, 0b01100, 0b11000, 0b00000, 0b00110, 0b11100, 0b10000}; // Reversed bits in HappyR1
  byte MadR1[8] = {0b00011, 0b00111, 0b01111, 0b01011, 0b01001, 0b01000, 0b01010, 0b01000}; // Reversed bits in HappyL4
  byte MadR2[8] = {0b11000, 0b11100, 0b11110, 0b11010, 0b10010, 0b00010, 0b00010, 0b00010}; // Reversed bits in HappyL3
  byte MadR3[8] = {0b01000, 0b01100, 0b00110, 0b00011, 0b00000, 0b01100, 0b00111, 0b00001}; // Reversed bits in HappyL2
  byte MadR4[8] = {0b00010, 0b00110, 0b01100, 0b11000, 0b00000, 0b00000, 0b00000, 0b11100}; // Reversed bits in HappyL1
  byte* Mad[] = {MadL1, MadL2, MadL3, MadL4, MadR1, MadR2, MadR3, MadR4};

  // Create characters on LCD
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, Mad[i]);
  }

  // Display on LCD
  Write();
}

void BlinkHappy() {
  byte blink_left1[8] = { 0b00000, 0b00011, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_left2[8] = { 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right1[8] = { 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right2[8] = { 0b00000, 0b11000, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink3[8] = { 0b00111, 0b01000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink4[8] = { 0b11100, 0b00010, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte* Blink[] = {blink_left1,blink_left2,blink3,blink4,blink_right1,blink_right2,blink3,blink4};

  for(int i=0; i<8; i++) {
    lcd.createChar(i, Blink[i]);
  }
  Write();
  delay(250);
  data_to_echo=0;
}
void inverted_BlinkHappy() {
  byte blink_left1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b11000, 0b00000};
  byte blink_left2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00111};
  byte blink_right1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11100};
  byte blink_right2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b00011, 0b00000};
  byte blink3[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00010, 0b11100};
  byte blink4[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01000, 0b00111};
  byte* Blink[] = {blink4,blink3,blink_right2, blink_right1, blink4, blink3, blink_left2, blink_left1};

  // Create characters on LCD
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, Blink[i]);
  }

  // Display on LCD
  Write();
  delay(250);
  data_to_echo = 0;
}

void BlinkMad() {
  byte blink_left1[8] = { 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_left2[8] = { 0b10000, 0b11100, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right1[8] = { 0b00001, 0b00111, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right2[8] = { 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink3[8] = { 0b00111, 0b01000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink4[8] = { 0b11100, 0b00010, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte* Blink[] = {blink_left1,blink_left2,blink3,blink4,blink_right1,blink_right2,blink3,blink4};

  for(int i=0; i<8; i++) {
    lcd.createChar(i, Blink[i]);
  }
  Write();
  delay(250);
  MadEyes();
  data_to_echo = 2;
}
void inverted_BlinkMad() {
  byte blink_left1[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11100 };
  byte blink_left2[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b00111, 0b00001 };
  byte blink_right1[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b11100, 0b10000 };
  byte blink_right2[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00111 };
  byte blink3[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00010, 0b11100 };
  byte blink4[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01000, 0b00111 };
  byte* Blink[] = { blink4, blink3, blink_right2, blink_right1, blink4, blink3, blink_left2, blink_left1};

  // Create characters on LCD
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, Blink[i]);
  }

  // Display on LCD
  Write();
  delay(250);
  inverted_MadEyes(); // Rotate MadEyes by 180 degrees
  data_to_echo = 2;
}

void Sleep() {
  byte blink_left1[8] = { 0b00000, 0b00011, 0b01100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_left2[8] = { 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right1[8] = { 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink_right2[8] = { 0b00000, 0b11000, 0b00110, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink3[8] = { 0b00111, 0b01000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte blink4[8] = { 0b11100, 0b00010, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
  byte* Blink[] = {blink_left1,blink_left2,blink3,blink4,blink_right1,blink_right2,blink3,blink4};

  for(int i=0; i<8; i++) {
    lcd.createChar(i, Blink[i]);
  }
  Write();
}
void inverted_Sleep() {
  byte blink_left1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b11000, 0b00000};
  byte blink_left2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00111};
  byte blink_right1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11100};
  byte blink_right2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b00011, 0b00000};
  byte blink3[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00010, 0b11100};
  byte blink4[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01000, 0b00111};
  byte* Blink[] = {blink4,blink3,blink_right2, blink_right1, blink4, blink3, blink_left2, blink_left1};

  // Create characters on LCD
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, Blink[i]);
  }

  // Display on LCD
  Write();
}

int findNonZero(int vec[3]) {
  for (int i = 0; i < 3; i++) {
    if (vec[i] > 0) {
      return i; // Return the index of the non-zero element
    }
  }
  return 0; // Return -1 if no non-zero element found
}

// Function to check if two vectors are not identical
bool areVectorsIdentical(int vec1[], int vec2[], int size) {
  for (int i = 0; i < size; i++) {
    if (vec1[i] != vec2[i]) {
      return false; // If any element is different, vectors are not identical
    }
  }
  return true; // Vectors are identical if all elements are the same
}

void spinmotor(int speed,int pin1, int pin2, int pwmpin){
  if (speed < 0){
    digitalWrite(pin2,1);
    digitalWrite(pin1,0);
    analogWrite(pwmpin,abs(speed));
  }
  else {
    digitalWrite(pin2,0);
    digitalWrite(pin1,1);
    analogWrite(pwmpin,abs(speed));
  }
}

void movestraight(){
  int speed = straight;
  Serial.print("straight ");
  Serial.println(speed);
  spinmotor(-speed, lf_motor[0], lf_motor[1],lf_motor[2]);
  spinmotor(speed, rf_motor[0], rf_motor[1],rf_motor[2]);
  spinmotor(speed, lb_motor[0], lb_motor[1],lb_motor[2]);
  spinmotor(speed, rb_motor[0], rb_motor[1],rb_motor[2]);
}

//Right Default
void movesideways(){
  int speed = side;
  Serial.print("side ");
  Serial.println(speed);
  spinmotor(-speed, lf_motor[0], lf_motor[1],lf_motor[2]);
  spinmotor(-speed, rf_motor[0], rf_motor[1],rf_motor[2]);
  spinmotor(-speed, lb_motor[0], lb_motor[1],lb_motor[2]);
  spinmotor(speed, rb_motor[0], rb_motor[1],rb_motor[2]);
}

//Right/CW Default
void rotatebot(){
  int speed = rot;
  Serial.print("rotate ");
  Serial.println(speed);
  spinmotor(-speed, lf_motor[0], lf_motor[1],lf_motor[2]);
  spinmotor(-speed, rf_motor[0], rf_motor[1],rf_motor[2]);
  spinmotor(speed, lb_motor[0], lb_motor[1],lb_motor[2]);
  spinmotor(-speed, rb_motor[0], rb_motor[1],rb_motor[2]);
}
