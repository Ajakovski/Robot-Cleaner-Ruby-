#define BLYNK_TEMPLATE_ID "YOUR TEMPLATE ID"
#define BLYNK_TEMPLATE_NAME "YOUR TEMPLATE NAME"
#define BLYNK_AUTH_TOKEN "YOUR BLYNK AUTH TOKEN"

#define BLYNK_PRINT Serial
#include <Servo.h> 
#include <SoftwareSerial.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "YOUR WIFI";
char pass[] = "YOUR WIFI PASSWORD";

Servo smallServo;
Servo mainServo;

#define EspSerial Serial1
#define ESP8266_BAUD 38400
ESP8266 wifi(&EspSerial);

#define pwm1 28
#define pwm2 29
#define IN1  22
#define IN2  23
#define IN3  24
#define IN4  25
#define VACUUM_PIN 50
#define TRIG_PIN 3
#define ECHO_PIN 2
#define LED_PIN 51
#define CLIFF_THRESHOLD_CM 20 //Limit for safety
#define CLIFF_LED_VPIN V7

BlynkTimer timer;
int mainHandID = -1;
int smallHandID = -1;
int cliffActionTimerID = -1;
int mPos = 75;
bool mUp = false;
int sPos = 90;
bool sUp = false;
bool cliffDetected = false;
float lastPrintedDistance = -1;
int lastPrintedMainHand = -1;
int lastPrintedSmallHand = -1;
int distanceFilter=0;

void Stop();
void forward();
void backward();
void left();
void right();


void checkPath() {
  timer.setInterval(100L, updateCliffSensor);  // Check every 100ms
}

void handleCliffDetectedStop() {
  Stop();  // Stop after reversing
  digitalWrite(LED_PIN, LOW);  // Turn off physical LED
  Blynk.virtualWrite(CLIFF_LED_VPIN, 0);  // Turn off Blynk LED

  cliffDetected = false;        // Allow new detections
}

void updateCliffSensor() {
  // Send the trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the pulse duration
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // Timeout after 30ms

  if (duration == 0) return; // No echo received

  float distance = duration * 0.034 / 2.0;
  
  if (abs(distance - lastPrintedDistance) > 2.0) { //distance is printed but filteredDistance has power of control
    Serial.print("Distance measured: ");
    Serial.print(distance);
    Serial.println(" cm");
    lastPrintedDistance = distance;
  }

  if (distance > CLIFF_THRESHOLD_CM) { //Replace distance with filteredDistance
    if (!cliffDetected /*&& distanceFilter>100*/) {
      //distanceFilter=0;
      cliffDetected = true;
      Stop(); //ne e testirano ama pozhelno za po dobra funkcionalnost
      delay(500); //DA SE TESTIRA SAMO STOP FUNKCIJA ZA SPRECUVANJE NA TRAGEDIJA OD SZTKM 2 God
      //backward();
      digitalWrite(LED_PIN, HIGH);  // Turn ON LED
      Blynk.virtualWrite(CLIFF_LED_VPIN, 1);  // Turn ON Blynk LED
      if (cliffActionTimerID != -1) timer.deleteTimer(cliffActionTimerID);
      cliffActionTimerID = timer.setTimeout(500L, handleCliffDetectedStop);
    }
  } else {
    //  distanceFilter++;
    if (cliffDetected) {
      cliffDetected = false;
      digitalWrite(LED_PIN, LOW); //Turn off LED
      Blynk.virtualWrite(CLIFF_LED_VPIN, 0);  // Turn OFF Blynk LED
    }
  }
}

void Stop() {
  Serial.println("Stop");
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

void forward() {
  Serial.println("Moving forward");
  analogWrite(pwm1, 140);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(pwm2, 140);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  Serial.println("Moving backward");
  analogWrite(pwm1, 140);
  analogWrite(pwm2, 140);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH); 
  digitalWrite(IN4, HIGH); 
}

void left() {
  Serial.println("Turning left");
  analogWrite(pwm1, 250);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); 
  analogWrite(pwm2, 170);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
}

void right() {
  Serial.println("Turning right");
  analogWrite(pwm1, 170);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); 
  analogWrite(pwm2, 250);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
}

void mainHandMove() {
  if (mPos % 10 == 0 && mPos != lastPrintedMainHand) {
      Serial.print("Main hand moving: ");
      Serial.println(mPos);
      lastPrintedMainHand = mPos;
    }
  if (mUp && mPos <= 115) {
    mainServo.write(mPos);
    mPos++;
  } else if (!mUp && mPos >= 75) {
    mainServo.write(mPos);
    mPos--;
  }
  if (mPos > 115 || mPos < 75) {
    Serial.println("Main hand movement complete");
    timer.disable(mainHandID);
  }
}
void smallHandMove() {
  if (sPos % 10 == 0 && sPos != lastPrintedSmallHand) {
      Serial.print("Small hand moving: ");
      Serial.println(sPos);
      lastPrintedSmallHand = sPos;
    }
  if (sUp && sPos >= 55) {
    smallServo.write(sPos);
    sPos--;
  } else if (!sUp && sPos <= 90) {
    smallServo.write(sPos);
    sPos++;
  }
  if (sPos < 55 || sPos > 90) {
    Serial.println("Small hand movement complete");
    timer.disable(smallHandID);
  }
}

void vacuum(int state) {
  Serial.print("Vacuum: "); // DEBUG
  Serial.println(state == 1 ? "ON" : "OFF");
  digitalWrite(VACUUM_PIN, state);
}

BLYNK_WRITE(V0) {
  if (param.asInt()==1) forward(); else Stop();
}

BLYNK_WRITE(V1) {
  if (param.asInt()==1) backward(); else Stop();
}

BLYNK_WRITE(V2) {
  if (param.asInt()==1) left(); else Stop();
}

BLYNK_WRITE(V3) {
  if (param.asInt()==1) right(); else Stop();
}

BLYNK_WRITE(V4) {
  sUp = (param.asInt() == 1);
  Serial.print("Small hand "); Serial.println(sUp ? "closing" : "opening");
  smallHandID = timer.setInterval(50, smallHandMove);
}

BLYNK_WRITE(V5) {
  mUp = !(param.asInt() == 1);
  Serial.print("Main hand "); Serial.println(mUp ? "lifting" : "lowering");
  mainHandID = timer.setInterval(50, mainHandMove);
}

BLYNK_WRITE(V6) {
  vacuum(param.asInt());
}

void setup() {
  Serial.begin(115200);
  EspSerial.begin(ESP8266_BAUD);
  delay(2000);

  Blynk.config(wifi, auth);
  if (Blynk.connectWiFi(ssid, pass)) {
    Blynk.connect();
    Serial.println("Blynk connected!");
  } else {
    Serial.println("WiFi/Blynk failed to connect.");
  }

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(VACUUM_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  mainServo.attach(26);
  smallServo.attach(27);
  checkPath();
}

void loop() {
  if (!Blynk.connected()) {
    Serial.println("Blynk disconnected, attempting to reconnect...");
    Blynk.connect();
  }

  Blynk.run();
  timer.run();
}
