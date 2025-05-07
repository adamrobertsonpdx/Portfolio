#include <Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

   //use this for the quarter inch!!!

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
//uint8_t button=0;
uint8_t butt_pin=10;
//uint8_t ser_led=0;//serial led 
//uint8_t tcs_led=0;//sensor led
Servo serv;
//scl is pin A5 and SDA is pin A4

typedef struct{
  uint16_t colorleft;
  uint16_t colorright;
  uint16_t colorave;
  uint16_t luxleft;
  uint16_t luxright;
  uint16_t luxave;
  uint16_t rleft;
  uint16_t rright;
  uint16_t rave;
  uint16_t gleft;
  uint16_t gright;
  uint16_t gave;
  uint16_t bleft;
  uint16_t bright;
  uint16_t bave;
  uint16_t cleft;
  uint16_t cright;
  uint16_t cave;
}nylon_t;
typedef struct{
  uint16_t colorleft;
  uint16_t colorright;
  uint16_t colorave;
  uint16_t luxleft;
  uint16_t luxright;
  uint16_t luxave;
  uint16_t rleft;
  uint16_t rright;
  uint16_t rave;
  uint16_t gleft;
  uint16_t gright;
  uint16_t gave;
  uint16_t bleft;
  uint16_t bright;
  uint16_t bave;
  uint16_t cleft;
  uint16_t cright;
  uint16_t cave;
}brass_t;
typedef struct{
  uint16_t colorleft;
  uint16_t colorright;
  uint16_t colorave;
  uint16_t luxleft;
  uint16_t luxright;
  uint16_t luxave;
  uint16_t rleft;
  uint16_t rright;
  uint16_t rave;
  uint16_t gleft;
  uint16_t gright;
  uint16_t gave;
  uint16_t bleft;
  uint16_t bright;
  uint16_t bave;
  uint16_t cleft;
  uint16_t cright;
  uint16_t cave;
}chrome_t;
typedef struct{
  uint16_t colorleft;
  uint16_t colorright;
  uint16_t colorave;
  uint16_t luxleft;
  uint16_t luxright;
  uint16_t luxave;
  uint16_t rleft;
  uint16_t rright;
  uint16_t rave;
  uint16_t gleft;
  uint16_t gright;
  uint16_t gave;
  uint16_t bleft;
  uint16_t bright;
  uint16_t bave;
  uint16_t cleft;
  uint16_t cright;
  uint16_t cave;
}black_t;
nylon_t nylon;
chrome_t chrome;
brass_t brass;
black_t black;
uint8_t enA=11;//pwm pin
uint8_t in1=12;
uint8_t in2=13;
Servo big;
Servo small;
uint8_t top=125;
uint8_t bott=100;
uint8_t start=100;
uint8_t per=1;
uint8_t dc=50;
uint8_t step=1;
uint8_t speed=100;
uint8_t left=0;
uint8_t middle=0;
uint8_t right=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.println("Serial initialized");
  pinMode(butt_pin,INPUT); //button pin
  //pinMode(ser_led,OUTPUT);
  //pinMode(tcs_led,OUTPUT);
  serv.attach(5);
  attachInterrupt(digitalPinToInterrupt(butt_pin),butChange,CHANGE);
  serv.write(11);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    //while (1);
  }
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  delay(100);
}
void loop() {
  // put your main code here, to run repeatedly:
  smallSer(true);
  screw();
  uint8_t col=getCol();
  screw();
  if((col!=0)&&(col!=4)){
    screw();
    bigSer(col);
  }
}
uint8_t getCol(){
  uint16_t arrTemp[3];
  uint16_t arrLux[3];
  uint16_t arrR[3];
  uint16_t arrG[3];
  uint16_t arrB[3];
  uint16_t arrC[3];
  uint8_t a=0;
  uint16_t r, g, b, c, colorTemp, lux;
  Serial.println("\nTaking readings\n");
  screw();
  while(a<3){
    uint16_t r, g, b, c, colorTemp, lux;
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);
    arrTemp[a]=colorTemp;
    arrLux[a]=lux;
    arrR[a]=r;
    arrG[a]=g;
    arrB[a]=b;
    arrC[a]=c;
    a++;
    screw();
  }
  colorTemp=0;lux=0;r=0;g=0;b=0;c=0;
  for(uint8_t d=0;d<3;d++){
    colorTemp+=arrTemp[d];
    lux+=arrLux[d];
    r+=arrR[d];
    g=arrG[d];
    b=arrB[d];
    c=arrC[d];
  }
  screw();
  colorTemp=colorTemp/4;lux=lux/4;r=r/4;g=g/4;b=b/4;c=c/4;
  if((lux<black.luxright)|(r<black.rright)|(g<black.gright)|(c<black.cright)|(b<black.bright)){
    return 4;//4 says its black
  }else if((nylon.colorleft<colorTemp)&&(colorTemp<nylon.colorright)){
    return 1;//1 is nylon
  }else if((brass.colorleft<colorTemp)&&(colorTemp<brass.colorright)){
    return 2;//2 is brass
  }else if((chrome.colorleft<colorTemp)&&(colorTemp<chrome.colorright)){
    return 3;//3 is chrome
  }else{
    //will have to figure something out based off the other values
  }
  return 4;
}
void screw(){
  cont();
}
void bigSer(uint8_t col){
  switch(col){
    case 1:
      big.write(left);
      screw();
      delay(200);
      break;
    case 2:
      big.write(middle);
      screw();
      delay(200);
      break;
    case 3:
      big.write(right);
      screw();
      delay(200);
      break;
    case 5:
      big.write(180);//this is for when the sorting ends
      break;
  }
  smallSer(false);
  big.write(middle);
  screw();
  delay(200);//do I need this?
}
void smallSer(bool in){
  if(in){
    small.write(5);
  }else{
    small.write(25);
  }
  screw();
  delay(50);//again do I really need?
}

void begin(){
  //handles everything upon start
  //could "start" in the middle of a cycle, so everything needs to reset
  //start screw, set servos to middle and in
  //reset values?
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  analogWrite(enA,start);
  smallSer(true);
  bigSer(1);
}
void butChange(){
  if(digitalRead(butt_pin==HIGH)){
    begin();
  }else{
    stop();
  }
}
void stop(){
  //needs to stop everything and set servos to correct positions
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA,0);
  bigSer(5);
  smallSer(false);
}
void cont(){
  analogWrite(enA,speed);
  delay(dc);
}
void pulse(){
  analogWrite(enA,top);
  delay(per);
  analogWrite(enA,bott);
  delay(dc-per);
}
void grad(){
  for(uint8_t a=bott;a<=top;a+=step){
    if(a<((top-bott)/2)+bott){
      analogWrite(enA,a);
      delay(249);
    }else{
      analogWrite(enA,a);
      delay(1);
    } 
  }
   /*
  for (int a = bott; a <= top; speed += step) {
    analogWrite(enA, a);
    delay(dc);
  }*/
}