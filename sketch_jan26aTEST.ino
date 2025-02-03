#include <Servo.h>
#include <Wire.h>
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//GY-521 stuff (accelerometer, gyro, temp)
//addy for the module, will be 0X69(nice) id ADD is HIGH and plugged in
const int MPU_ADDR = 0x68;
//accelerometer xyz
int16_t ax,ay,az;
//gyro xyz
int16_t gx,gy,gz;
//temp
int16_t temp;

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//head servo
Servo myServo;
#define HEAD 3
//max 180
int leftAngle = 135;
//90 should be the center
int centerAngle = 90;
//min 0
int rightAngle = 45;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//Ultra Sonic sensor
int Echo = 10;  
int Trig = 9; 

#define COLDIST 10
int leftDist,rightDist = 0;
#define turnTime 250

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//wheel vars
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

int carSpeed = 250;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void adjustSpeed(int newSpeed) {
  carSpeed = newSpeed;
}

void forward(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

}
void backward(){
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stop(){
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);

}

int distanceTest() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}

void checkRange(){
  //Serial.println("checking left");
  myServo.write(leftAngle);
  delay(300);
  leftDist = distanceTest();
  delay(300);
  //Serial.println("checking right");
  myServo.write(rightAngle);
  delay(300);
  rightDist = distanceTest();
  delay(300);
  myServo.write(centerAngle);
}

void read_gy(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  ax = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  ay = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  az = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temp= Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gx = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gy = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gz = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(ax));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(ay));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(az));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temp/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gx));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gy));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gz));
  Serial.println();
}


void setup() {
  // wheel stuff:
  pinMode(HEAD, OUTPUT);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  analogWrite(ENA,carSpeed);
  analogWrite(ENB,carSpeed);
  stop();
  //head stuff
  myServo.attach(3);
  myServo.write(centerAngle);
  Serial.begin(9600);
  //gy-521
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() {
  //forward();
  read_gy();
  delay(200);
  //Serial.println();
  // put your main code here, to run repeatedly
  /*if (distanceTest() < COLDIST){
    stop();
    checkRange();
    //Serial.println("done checking");
    if (leftDist >= rightDist){
      left();
      delay(turnTime);
    }
    else if(rightDist > leftDist){
      right();
      delay(turnTime);
    }
  }*/
}