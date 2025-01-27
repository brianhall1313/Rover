int HEAD = 3;

int ENA = 5;
int ENB = 6;
int IN1 = 7;
int IN2 = 8;
int IN3 = 9;
int IN4 = 11;


void setup() {
  // put your setup code here, to run once:
  pinMode(HEAD, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void left(){
  digitalWrite(IN1,HIGH);
  
}
void right(){

}
void forward(){

}
void backward(){

}