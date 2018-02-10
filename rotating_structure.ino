#include <Stepper.h>
#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3
const int SPR = 200;
char x,y,z;
int i=10,j=20,k=30 ;
int go=0;

SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);
Stepper myStepper(SPR, 8, 9, 10, 11);

void setup() {
 pinMode(rxPin, INPUT);
 pinMode(txPin, OUTPUT);
 xbee.begin(9600);
 // Serial.begin(9600);
 //Serial.println("Starting XBee Comunication");
}

void loop() {
  myStepper.setSpeed(20);
  x=(xbee.read()); 
   switch(x){
      case 'a':
        i = 0;
        break;
      case 'b':
        i = 1;
        break;
      case 'c':
        i = 2;
        break;
      default:
        i = 9;
   } 

  y=(xbee.read());
  switch(x){
      case 'w':
        j = 1;
        break;
      case 'x':
        j = 2;
        break;
      case 'y':
        j = 3;
        break;
      case 'z':
        j = 4;
        break;
      default:
        j = 9;
  }  
  k=i+j; 
  switch(k){
    case 1:
      go = SPR/4;
      break;
    case 2:
      go = SPR/2;
      break;
    case 3:
      go = SPR*0.75;
      break;
    case 4:
      go = SPR*0;
      break;
    case 5:
      go = SPR/4;
      break;
    case 6:
      go = SPR/2;
      break;
    default:
      go = SPR*0;
  }
  myStepper.step(go);   
  delay(500);
}
