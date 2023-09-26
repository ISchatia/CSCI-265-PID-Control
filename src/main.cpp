#include <Arduino.h>
#include <Rangefinder.h>
#include <Chassis.h>

#define STATE_BEGIN 1
#define STATE_SENSE 2
#define STATE_THINK 3
#define STATE_ACT 4

#define ECHO_PIN 11
#define TRIGGER_PIN 4

#define BAUD_RATE 115200

#define DEBUG true

#define CM_TO_IN 0.393701

#define K_P 13.0
#define K_I 0.01
#define K_D 0.01

#define WHEEL_DIAMETER 7.0
#define ENCODER_COUNTS_PER_REV 1440
#define DIST_BETWEEN_WHEELS 14.9

int state = STATE_BEGIN;

float val = 0.0;
float ref = 6.0;

float eT = 0.0;
float eT_deltaT = 0.0;

float meas = 0.0;
float meas_deltaT = 0.0;

float ddt_eT = 0.0;
float i_eT = 0.0;

float actSignal = 0.0;

Rangefinder rangefinder(ECHO_PIN, TRIGGER_PIN);
Chassis chassis(WHEEL_DIAMETER, ENCODER_COUNTS_PER_REV, DIST_BETWEEN_WHEELS);

void setup() {
  // put your setup code here, to run once:

  Serial.begin(BAUD_RATE);
  // while(!Serial) {
  //   ;
  // }

  rangefinder.init();
  chassis.init();
  chassis.setMotorPIDcoeffs(5, 0.5);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state) {

    case STATE_BEGIN:
      // Serial.println("STATE_BEGIN");
      
      // might not be needed
      meas = meas_deltaT;
      eT = ref - meas_deltaT;

      state = STATE_SENSE;
      break;

    case STATE_SENSE:
      // Serial.println("STATE_SENSE");
      
      meas = meas_deltaT;
      eT = eT_deltaT;

      // measure
      meas_deltaT = rangefinder.getDistance();
      meas_deltaT = meas_deltaT * CM_TO_IN;
      Serial.print("meas_deltaT= ");
      Serial.print(meas_deltaT);

      // compute next error
      eT_deltaT = ref - meas_deltaT;

      // compute derivative and integral
      ddt_eT = eT_deltaT - eT;
      i_eT = i_eT + eT_deltaT;

      state = STATE_THINK;
      break;

    case STATE_THINK:
      // Serial.println("STATE_THINK");

      // eT = ref - meas_deltaT; // unsure if this part is needed.
      Serial.print(":     eTd=");
      Serial.print(eT_deltaT);

      Serial.print(":     IeT=");
      Serial.print(i_eT);

      Serial.print(":     ddteT=");
      Serial.print(ddt_eT);

      actSignal = (K_P * eT_deltaT) + (K_I * i_eT) + (K_D * ddt_eT);
      Serial.print(":     actSignal=");
      Serial.print(-actSignal);

      state = STATE_ACT;
      break;

    case STATE_ACT:
      // Serial.println("STATE_ACT");

      //chassis.setWheelSpeeds(5, 5); 
      //chassis.driveFor(5,10);
      float drive = actSignal*-1;
      float direction = 2*drive/abs(drive);
      chassis.driveFor(direction,drive,false);
      bool check = chassis.checkMotionComplete();
      Serial.print(": chassis :");
      Serial.println(check);
      state = STATE_SENSE;
      break;

  }

}