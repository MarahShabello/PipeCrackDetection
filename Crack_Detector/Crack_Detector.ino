// Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.

// Motor 1
int MOTOR_1_PIN_1 = 2;
int MOTOR_1_PIN_2 = 3;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2
int MOTOR_2_PIN_1 = 4;
int MOTOR_2_PIN_2 = 5;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed

unsigned long int value = 0; // stores the incoming hex value
byte seq = 0; //stores the current number of executed sequenceso
byte seq_Array[50];// array to store the movement sequence in terms of integers(1 for FWD, 2 for LEFT and so on..)

//counter for counting the number of times program pass through a movement function(fwd, lft etc.)
int fwd_Counter = -1;
int lft_Counter = -1;
int rgt_Counter = -1;
int bwd_Counter = -1;
int stp_Counter = -1;

//global "current time" variables for different movement functions(fwd, lft etc.)
unsigned long int current_Time0 = 0;// for FWD movement
unsigned long int current_Time1 = 0;// for LEFT movement
unsigned long int current_Time2 = 0;// for RIGHT movement
unsigned long int current_Time3 = 0;// for BWD movement
unsigned long int current_Time4 = 0;// for STOP

//total time spend by the pgm in executing the movement(fwd, lft etc.) for a particular movement counter
unsigned long int total_Fwd_Time[10];
unsigned long int total_Lft_Time[10];
unsigned long int total_Rgt_Time[10];
unsigned long int total_Bwd_Time[10];
unsigned long int total_Stp_Time[10];

bool flag = false;

#define RX 0
#define TX 1

int motorSpeed = 80;

#include <SoftwareSerial.h>
#include "string.h"
SoftwareSerial SoftSerial(RX, TX);

#include <SharpIR.h>

// Define model and input pin:
#define IRPin A0
#define model 1080

SharpIR mySensor = SharpIR(IRPin, model);

#include <Servo.h>
#define trigPin 7
#define echoPin 8
Servo servo;

void setup() {  // Setup runs once per reset
  // initialize serial communication @ 9600 baud:
  Serial.begin(9600);
  SoftSerial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo.attach(6);
  servo.write(90);

  //Define L298N Dual H-Bridge Motor Controller Pins
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(speedPinA, OUTPUT);

  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(speedPinB, OUTPUT);

  flag = true;

  //  goForward();
  //  delay(7000);
  //  goLeft(LOW, HIGH);
  //  delay(3000);
  //  goRight(LOW, HIGH);
  //  delay(3000);
  //  goBackward();
  //  delay(7000);
  //  goStop();
  //  delay(1000);

  //  goForward();
  //  delay(7000);
  //  goLeft(LOW, HIGH);
  //  delay(3000);
  //  goRight(LOW, HIGH);
  //  delay(3000);
  //  goBackward();
  //  delay(7000);
  //  goStop();
  //  delay(7000);

  flag = false;
}

void loop() {
  //  go_In_Seq();

  String cmd;
  SoftSerial.write("Hello Boss");
  cmd = SoftSerial.readString() ;
  cmd.trim();
  Serial.print("cmd: ");
  Serial.println(cmd);
  cmd = "auto";

  if (cmd == "Forward") {
    goForward();
  }
  else if (cmd == "Right") {
    goRight(LOW, HIGH);
  }
  else if (cmd == "Backward") {
    goBackward();
  }
  else if (cmd == "Left") {
    goLeft(LOW, HIGH);
  }
  else if (cmd == "Stop") {
    goStop();
  }
  else if (cmd == "pathSaving") {
    flag = true;
  }
  else if (cmd == "increase") {
    increaseSpeed();
  }
  else if (cmd == "decrease") {
    decreaseSpeed();
  }
  else if (cmd == "path") {
    go_In_Seq();
  }
  else if (cmd == "auto") {
    Serial.println("Auto Mode");
    move_autoPath();
  }
  delay(500);

  Serial.println("LED");
  //  go_Back_Seq();
}


void goForward() {
  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM

  if (flag) {
    Serial.println("Forward2");
    current_Time0 = millis();
    int i = seq_Array[(seq - 1)];
    switch (i) {
      case 2:
        // total time elaspsed since Left button is pressed including rest time
        total_Lft_Time[lft_Counter + 1] = (current_Time0 - current_Time1);
        lft_Counter++;
        break;

      case 3:
        // total time elaspsed since Right button is pressed including rest time
        total_Rgt_Time[rgt_Counter + 1] = (current_Time0 - current_Time2);
        rgt_Counter++;
        break;

      case 4:
        total_Bwd_Time[bwd_Counter + 1] = (current_Time0 - current_Time3);
        bwd_Counter++;
        break;

      case 5:
        total_Stp_Time[stp_Counter + 1] = (current_Time0 - current_Time4);
        stp_Counter++;
        break;
    }

    seq_Array[seq] = 1;
    Serial.print("Value = ");
    Serial.println(seq_Array[seq]);
    seq++;
  }

  digitalWrite(MOTOR_1_PIN_1, HIGH);
  digitalWrite(MOTOR_1_PIN_2, LOW);

  digitalWrite(MOTOR_2_PIN_1, HIGH);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void goRight(bool pin1, bool pin2) {
  if (flag) {
    Serial.println("Right2");
    current_Time2 = millis();
    int i = seq_Array[(seq - 1)];
    switch (i) {
      case 1:
        total_Fwd_Time[fwd_Counter + 1] = (current_Time2 - current_Time0);
        fwd_Counter++;
        break;

      case 2:
        total_Lft_Time[lft_Counter + 1] = (current_Time2 - current_Time1);
        lft_Counter++;
        break;

      case 4:
        total_Bwd_Time[bwd_Counter + 1] = (current_Time2 - current_Time3);
        bwd_Counter++;
        break;

      case 5:
        total_Stp_Time[stp_Counter + 1] = (current_Time2 - current_Time4);
        stp_Counter++;
        break;
    }

    seq_Array[seq] = 3;
    Serial.print("Value = ");
    Serial.println(seq_Array[seq]);
    seq++;
  }

  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM

  digitalWrite(MOTOR_1_PIN_1, pin2);
  digitalWrite(MOTOR_1_PIN_2, pin1);

  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void goLeft(bool pin1, bool pin2) {
  if (flag) {
    Serial.println("Left2");
    current_Time1 = millis();
    int i = seq_Array[(seq - 1)];
    switch (i) {
      case 1:
        total_Fwd_Time[fwd_Counter + 1] = (current_Time1 - current_Time0);
        Serial.print("fwd_TIME = ");
        Serial.print(current_Time1 - current_Time0);
        Serial.print(", in index: ");
        Serial.println(fwd_Counter + 1);
        fwd_Counter++;
        break;

      case 3:
        total_Rgt_Time[rgt_Counter + 1] = (current_Time1 - current_Time2);
        rgt_Counter++;
        break;

      case 4:
        total_Bwd_Time[bwd_Counter + 1] = (current_Time1 - current_Time3);
        bwd_Counter++;
        break;

      case 5:
        total_Stp_Time[stp_Counter + 1] = (current_Time1 - current_Time4);
        stp_Counter++;
        break;
    }

    seq_Array[seq] = 2;
    Serial.print("Value = ");
    Serial.println(seq_Array[seq]);
    seq++;
  }
  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM

  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, LOW);

  digitalWrite(MOTOR_2_PIN_1, pin2);
  digitalWrite(MOTOR_2_PIN_2, pin1);
}

void goBackward() {
  if (flag) {
    Serial.println("Back2");
    current_Time3 = millis();
    int i = seq_Array[(seq - 1)];
    switch (i) {
      case 1:
        total_Fwd_Time[fwd_Counter + 1] = (current_Time3 - current_Time0);
        fwd_Counter++;
        break;

      case 2:
        total_Lft_Time[lft_Counter + 1] = (current_Time3 - current_Time1);
        lft_Counter++;
        break;

      case 3:
        total_Rgt_Time[rgt_Counter + 1] = (current_Time3 - current_Time2);
        rgt_Counter++;
        break;

      case 5:
        total_Stp_Time[stp_Counter + 1] = (current_Time3 - current_Time4);
        stp_Counter++;
        break;
    }

    seq_Array[seq] = 4;
    Serial.print("Value = ");
    Serial.println(seq_Array[seq]);
    seq++;
  }
  analogWrite(speedPinA, motorSpeed);//Sets speed variable via PWM
  analogWrite(speedPinB, motorSpeed);//Sets speed variable via PWM

  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, HIGH);

  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, HIGH);
}

void goStop() {
  if (flag) {
    Serial.println("Stop2");
    current_Time4 = millis();
    int i = seq_Array[(seq - 1)];
    switch (i) {
      case 1:
        total_Fwd_Time[fwd_Counter + 1] = (current_Time4 - current_Time0);
        fwd_Counter++;
        break;

      case 2:
        total_Lft_Time[lft_Counter + 1] = (current_Time4 - current_Time1);
        lft_Counter++;
        break;

      case 3:
        total_Rgt_Time[rgt_Counter + 1] = (current_Time4 - current_Time2);
        rgt_Counter++;
        break;

      case 4:
        total_Bwd_Time[bwd_Counter + 1] = (current_Time4 - current_Time3);
        bwd_Counter++;
        break;
    }

    seq_Array[seq] = 5;
    Serial.print("Value = ");
    Serial.println(seq_Array[seq]);
    seq++;
  }
  analogWrite(speedPinA, 0);//Sets speed variable via PWM
  analogWrite(speedPinB, 0);//Sets speed variable via PWM

  digitalWrite(MOTOR_1_PIN_1, LOW);
  digitalWrite(MOTOR_1_PIN_2, LOW);

  digitalWrite(MOTOR_2_PIN_1, LOW);
  digitalWrite(MOTOR_2_PIN_2, LOW);
}

void go_In_Seq(void) {
  value = 0;
  for (int i = 0; i < (seq + 1); i++) {
    int value1 = 0;
    value1 = seq_Array[i];
    Serial.print(i);
    Serial.print(": ");
    Serial.print(seq_Array[i]);
    Serial.print(", with value = ");
    Serial.println(value1);
    switch (value1) {
      case 1:
        static int j = 0;
        go_Forward_Seq(j);
        j++;
        break;
      case 2:
        static int k = 0;
        go_Left_Seq(k);
        k++;
        break;
      case 3:
        static int l = 0;
        go_Right_Seq(l);
        l++;
        break;
      case 4:
        static int m = 0;
        go_Backward_Seq(m);
        m++;
        break;
      case 5:
        static int n = 0;
        go_Stop_Seq(n);
        n++;
        break;
      default:
        j = 0; k = 0; l = 0; m = 0; n = 0;
    }
  }
  //  flag = false;
}

void go_Forward_Seq(int j) {
  //go in forward direction sequence
  //movement_Inst_Fwd();
  goForward();
  delay(total_Fwd_Time[j]);
}

void go_Left_Seq(int k) {
  //go in Left direction sequence
  //movement_Inst_Lft();
  goLeft(LOW, HIGH);
  delay(total_Lft_Time[k]);
}

void go_Right_Seq(int l) {
  //go in right direction sequence
  //movement_Inst_Rgt();
  goRight(LOW, HIGH);
  delay(total_Rgt_Time[l]);
}

void go_Backward_Seq(int m) {
  //go in backward direction sequence
  //movement_Inst_Bwd();
  goBackward();
  delay(total_Bwd_Time[m]);
}

void go_Stop_Seq(int n) {
  //go in Stop sequence
  //movement_Inst_Stp();
  goStop();
  delay(total_Stp_Time[n]);
}

void go_Back_Seq(void) {
  for (int i = seq; i >= 0; i--) {
    int value1 = 0;
    value1 = seq_Array[i];
    Serial.print("Back => ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(seq_Array[i]);
    switch (value1) {
      case 1:
        static int m = fwd_Counter;
        Serial.print("bwd_Counter = ");
        Serial.println(fwd_Counter);
        go_Backward_Seq(m);
        m--;
        break;
      case 2:
        static int k = lft_Counter;
        Serial.print("lft_Counter = ");
        Serial.println(lft_Counter);
        go_Right_Seq(k);
        //        goRight(HIGH, LOW);
        //        delay(total_Rgt_Time[k]);
        k--;
        break;
      case 3:
        static int l = rgt_Counter;
        Serial.print("rgt_Counter = ");
        Serial.println(rgt_Counter);
        go_Left_Seq(l);
        //        goLeft(HIGH, LOW);
        //        delay(total_Lft_Time[l]);
        l++;
        break;
      case 4:
        static int j = bwd_Counter;
        Serial.print("bwd_Counter = ");
        Serial.println(bwd_Counter);
        go_Forward_Seq(j);
        j--;
        break;
      case 5:
        static int n = stp_Counter;
        Serial.print("stp_Counter = ");
        Serial.println(stp_Counter);
        go_Stop_Seq(n);
        n--;
        break;
      default:
        j = fwd_Counter; k = lft_Counter; l = rgt_Counter; m = bwd_Counter; n = stp_Counter;
    }
  }
}

void increaseSpeed () {
  motorSpeed += 10;
}

void decreaseSpeed () {
  motorSpeed -= 10;
}

void move_autoPath () {
  long duration, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  if (distance < 10) {
    Serial.print(distance);
    Serial.println(" cm");
    servo.write(0); //right
    // servo.write(180); //left
  }

  else {
    Serial.println("The distance is greater than 10cm");
    // Serial.println("Forward");
    servo.write(90);
  }
  // int distance_cm = mySensor.getDistance() / 2;

  // // Print the measured distance to the serial monitor:
  // Serial.print("Mean distance: ");
  // Serial.print(distance_cm);
  // Serial.println(" cm");

  // if (distance_cm < 5) {
  //   goStop();


  //   // Rotate the sensor to the right
  //   distance_cm = mySensor.getDistance() / 2;

  //   // Print the measured distance to the serial monitor:
  //   Serial.print("Mean distance inside {distance_cm < 5}: ");
  //   Serial.print(distance_cm);
  //   Serial.println(" cm");
  //   if (distance_cm > 5) {
  //     goRight(LOW, HIGH);
  //   }
  //   else {
  //     // Rotate the sensor to the left
  //     distance_cm = mySensor.getDistance() / 2;

  //     // Print the measured distance to the serial monitor:
  //     Serial.print("Mean distance...: ");
  //     Serial.print(distance_cm);
  //     Serial.println(" cm");
  //     if (distance_cm > 5) {
  //       goLeft(LOW, HIGH);
  //     }
  //   }
  // }
  // else {
  //   goForward();
  // }

  //   if (distance_cm < 10) {
  // Serial.print(distance_cm);
  // Serial.println(" cm");
  // servo.write(0); //right
  // // servo.write(180); //left
  // Serial.println("Distance is less than 10");
  // }

  // else {
  // Serial.println("The distance is more than 10cm");
  // // Serial.println("Forward");
  // servo.write(90);
  // }

  delay(100);
}
