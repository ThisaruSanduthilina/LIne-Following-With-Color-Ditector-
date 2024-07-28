#define leftSensor 17
#define midLeftSensor 18
#define midSensor 19
#define midRightSensor 20
#define rightSensor 21

#define RMF 4
#define RMB 5
#define RMS 9
#define LMB 3
#define LMF 2
#define LMS 10

//LED BULBS 
#define L1 15
#define L2 16


// LED pins connected to Arduino
int redLed = 14; 
int greenLed = 44; 
int blueLed = 42;
// Variables 
int red = 0; 
int green = 0; 
int blue = 0; 

// Sensor states
bool ls, mls, ms, mrs, rs;

// PID Constants
float Kp = 1.5;  // Adjust these values based on your system and tuning
float Ki = 0.1;
float Kd = 0.01;

const int s0 = 22; 
const int s1 = 24; 
const int s2 = 26; 
const int s3 = 28; 
const int out = 30;  

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int initial_motor_speed = 90; // Initial motor speed

void setup() {

    //color sensor 


  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  pinMode(out, INPUT); 
  pinMode(redLed, OUTPUT); 
  pinMode(greenLed, OUTPUT); 
  pinMode(blueLed, OUTPUT); 
  digitalWrite(s0, HIGH); 
  digitalWrite(s1, HIGH); 

  pinMode(leftSensor, INPUT);
  pinMode(midLeftSensor, INPUT);
  pinMode(midSensor, INPUT);
  pinMode(midRightSensor, INPUT);
  pinMode(rightSensor, INPUT);

  pinMode(RMF, OUTPUT);
  pinMode(RMB, OUTPUT);
  pinMode(RMS, OUTPUT);
  pinMode(LMB, OUTPUT);
  pinMode(LMF, OUTPUT);
  pinMode(LMS, OUTPUT);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);

  
  Serial.begin(9600);
}

void loop() {
  read_sensor_values();
  calculate_pid();
  motor_control();

    Serial.print(ls);
  Serial.print(" | ");
  Serial.print(mls);
  Serial.print(" | ");
  Serial.print(ms);
  Serial.print(" | ");
  Serial.print(mrs);
  Serial.print(" | ");
  Serial.println(rs);
}

void read_sensor_values() {
  ls = digitalRead(leftSensor);
  mls = digitalRead(midLeftSensor);
  ms = digitalRead(midSensor);
  mrs = digitalRead(midRightSensor);
  rs = digitalRead(rightSensor);

  // Implement sensor reading logic if needed
}

void calculate_pid() {
  P = error;
  I = constrain(I + previous_I, -50, 50);  // Limit the integral term to prevent wind-up
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control() {
  // Calculate individual motor speeds based on PID control
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // Ensure motor speeds are within allowable range
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  // Set motor speeds
  analogWrite(RMS, right_motor_speed);
  analogWrite(LMS, left_motor_speed);

  // Implement logic to determine motor actions based on sensor states
  if ((ls == 1) && (mls == 1) && (ms == 0) && (mrs == 1) && (rs == 1)) {
    forward();
  } else if ((ls == 1) && (mls == 1) && (ms == 1) && (mrs == 0) && (rs == 1)) {
    right();
  } else if ((ls == 1) && (mls == 1) && (ms == 1) && (mrs == 1) && (rs == 0)) {
    right();
  } else if ((ls == 1) && (mls == 1) && (ms == 1) && (mrs == 0) && (rs == 0)) {
    right();
  } else if ((ls == 1) && (mls == 1) && (ms == 0) && (mrs == 0) && (rs == 1)) {
    right();
  } else if ((ls == 1) && (mls == 0) && (ms == 1) && (mrs == 1) && (rs == 1)) {
    left();
  } else if ((ls == 0) && (mls == 1) && (ms == 1) && (mrs == 1) && (rs == 1)) {
    left();
  } else if ((ls == 0) && (mls == 0) && (ms == 1) && (mrs == 1) && (rs == 1)) {
    left();
  } else if ((ls == 1) && (mls == 0) && (ms == 0) && (mrs == 1) && (rs == 1)) {
    left();
  } else if ((ls == 0) && (mls == 0) && (ms == 0) && (mrs == 1) && (rs == 1)) {
    left();
  } else if ((ls == 0) && (mls == 0) && (ms == 0) && (mrs == 0) && (rs == 1)) {
    left();
  } else if ((ls == 0) && (mls == 0) && (ms == 0) && (mrs == 1) && (rs == 0)) {
    left();
  } else if ((ls == 1) && (mls == 1) && (ms == 0) && (mrs == 0) && (rs == 0)) {
    right();
  } else if ((ls == 1) && (mls == 0) && (ms == 0) && (mrs == 0) && (rs == 0)) {
    right();
  } else if ((ls == 0) && (mls == 1) && (ms == 0) && (mrs == 0) && (rs == 0)) {
    right();
  } else if ((ls == 1) && (mls == 1) && (ms == 1) && (mrs == 1) && (rs == 1)) {
    forward();
    threeSixty();
  } else if ((ls == 0) && (mls == 0) && (ms == 0) && (mrs == 0) && (rs == 0)) {
    
    color();

    
  if (red < blue && red < green && red < 20)
  { 
   Serial.println(" - (Red Color)"); 
   digitalWrite(redLed, HIGH); // Turn RED LED ON
   digitalWrite(greenLed, LOW); 
   digitalWrite(blueLed, LOW); 
  } 

  else if (blue < red && blue < green)  
  { 
   Serial.println(" - (Blue Color)"); 
   digitalWrite(redLed, LOW); 
   digitalWrite(greenLed, LOW); 
   digitalWrite(blueLed, HIGH); // Turn BLUE LED ON 
  } 

  else if (green < red && green < blue) 
  { 
   Serial.println(" - (Green Color)"); 
   digitalWrite(redLed, LOW); 
   digitalWrite(greenLed, HIGH); // Turn GREEN LED ON
   digitalWrite(blueLed, LOW); 
  } 
  else{
  Serial.println(); 
  }
  delay(300);  
  digitalWrite(redLed, LOW); 
  digitalWrite(greenLed, LOW); 
  digitalWrite(blueLed, LOW); 
  
   delay(150);
   if((ls == 0) && (mls == 0) && (ms == 0) && (mrs == 0) && (rs == 0)){
        stop();
   
  }
  }
}

void forward() {
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
}

void stop() {
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);

  digitalWrite(L1,HIGH);
  digitalWrite(L2,HIGH);
}

void right() {
  digitalWrite(RMF, LOW);
  digitalWrite(RMB, LOW);
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB, LOW);
     rightSignal();
}

void left() {
  digitalWrite(RMF, HIGH);
  digitalWrite(RMB, LOW);
  digitalWrite(LMF, LOW);
  digitalWrite(LMB, LOW);
     leftSignal();
}

void threeSixty() {
  digitalWrite(RMB, HIGH);
  digitalWrite(RMF, LOW);
  digitalWrite(LMF, HIGH);
  digitalWrite(LMB,LOW);

  digitalWrite(L1,HIGH);
  digitalWrite(L2,HIGH);
}


void color() 
{   
  digitalWrite(s2, LOW); 
  digitalWrite(s3, LOW); 
  //count OUT, pRed, RED 
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); 
  digitalWrite(s3, HIGH); 
  //count OUT, pBLUE, BLUE 
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); 
  digitalWrite(s2, HIGH); 
  //count OUT, pGreen, GREEN 
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); 
}


void leftSignal(){
  digitalWrite(L1,LOW);
  digitalWrite(L2,HIGH);
  delay(200);
  digitalWrite(L2,LOW);
 
}

void rightSignal(){
  digitalWrite(L2,LOW);
  digitalWrite(L1,HIGH);
  delay(200);
  digitalWrite(L1,LOW);

}
