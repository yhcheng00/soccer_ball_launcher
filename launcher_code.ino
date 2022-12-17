#include <ESP32Encoder.h>
#include "SparkFunLSM6DSO.h"
#include "Wire.h"

// right motor
#define RPWM 16
#define REN1 34
#define REN2 39

// left motor
#define LPWM 21
#define LEN1 36
#define LEN2 4

// yaw motor
#define YIN1 15
#define YIN2 33
#define YEN1 26
#define YEN2 25

// linear actuator
#define LIN1 32
#define LIN2 14

enum states {
  WAIT,
  SETUP,
  SPINUP,
  LAUNCH,
};

states state;

// encoder definitions
ESP32Encoder motor_r;
ESP32Encoder motor_l;
ESP32Encoder motor_y;
LSM6DSO myIMU;

// Timer variables
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * timer1 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
volatile int r_count = 0;
volatile int l_count = 0;
volatile int y_count = 0;
volatile bool del_t = false;
volatile bool del_m = false;

// control variables
int pr_count; int prev_er;
int pl_count; int prev_el;
float kp = 2.0; float kd = 1.0;

// PWM variables
const int freq = 1000;
const int right_pwm = 1;
const int left_pwm = 2;
const int resolution = 16;
const int MAX_PWM = 255;

// Target Variables
int target_yaw;
int target_pitch;
int target_rpm;

bool pdone = false;
bool ydone = false;
bool sdone = false;

const float tpd = 28 * 188 * 2 * 12 / 360;

float pit;
float pit_avg;

void IRAM_ATTR onTime0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  y_count = motor_y.getCount();
  del_m = true;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  r_count = motor_r.getCount();
  motor_r.clearCount();
  l_count = motor_l.getCount();
  motor_l.clearCount();
  del_t = true;
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Input yaw/pitch (in degrees) and velocity(in MPH) in format XXX,XX,XX");
  Serial.println("Max velocity: 40 mph");
  Serial.println("Max pitch: 35 degrees");
  Serial.println("For example: -05,15,30 or +07,12,25");
  Serial.println("-----------------------------------------------------");
  ESP32Encoder::useInternalWeakPullResistors = UP;
  
  // IO config
  pinMode(LIN1, OUTPUT);
  pinMode(LIN2, OUTPUT);

  pinMode(YIN1, OUTPUT);
  pinMode(YIN2, OUTPUT);

  // Encoder configuration
  motor_r.attachFullQuad(REN1, REN2);
  motor_r.setCount(0);

  motor_l.attachFullQuad(LEN1, LEN2);
  motor_l.setCount(0);

  motor_y.attachFullQuad(YEN1, YEN2);
  motor_y.setCount(0);

  // PWM configuration
  ledcSetup(right_pwm, freq, resolution);
  ledcAttachPin(RPWM, right_pwm);

  ledcSetup(left_pwm, freq, resolution);
  ledcAttachPin(LPWM, left_pwm);

  // Accelerometer Setup
  Wire.begin(23,22);
  delay(10);
  myIMU.begin();
  myIMU.initialize(BASIC_SETTINGS);

  // Timer configuration
  timer1 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer1, &onTime1, true);
  timerAlarmWrite(timer1, 250000, true);
  timerAlarmEnable(timer1);
  timerStop(timer1);

  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &onTime0, true);
  timerAlarmWrite(timer0, 1000, true);
  timerAlarmEnable(timer0);
  timerStop(timer0);
}

void loop() {
  switch(state) {
    case WAIT:
      if (Serial.available() > 8) {
        char message[10];
        for (int i = 0; i < 9; i++) {
          char rec = Serial.read();
          Serial.println(rec);
          message[i] = rec;
        }
        message[9] = '\0';

        //Yaw Extract
        char yaw_buf[3];
        memcpy(yaw_buf,message+1,2);
        yaw_buf[2] = '\0';
        if(message[0] == '+') {
          target_yaw = atoi(yaw_buf);
        } else {
          target_yaw = -1 * atoi(yaw_buf);
        }
        Serial.println(target_yaw);

        //Pitch Extract
        char pitch_buf[3];
        memcpy(pitch_buf,message+4,2);
        pitch_buf[2] = '\0';
        target_pitch = atoi(pitch_buf);
        if (target_pitch > 35) {
          target_pitch = 35;
        }
        Serial.println(target_pitch);


        //RPM Extract
        char rpm_buf[3];
        memcpy(rpm_buf,message+7,4);
        rpm_buf[2] = '\0';
        target_rpm = atoi(rpm_buf);

        if (target_rpm > 40) {
          target_rpm = 40;
        }
        Serial.println(target_rpm);

        float w = target_rpm / 2.237;
        w = w / (0.11 + 0.127);
        w = ((2.0 * 0.0088 + 0.0263) * w) / (2.0 * 0.0088);


        target_rpm = w * 9.5493;
        Serial.println(target_rpm);

        // transition state
        pdone = false;
        ydone = false;
        sdone = false;
        pit = 0;
        pit_avg = 0;
        start_setup();

        state = SETUP;
      }
      break;

    case SETUP:
      while(!pdone) {
        pitch(target_pitch);
      }

      while(!ydone) {
        yaw(target_yaw);
      }
      if (sdone) {
        state = WAIT;
      } else {
        Serial.println("Spinning up for launch");
        timerStop(timer0);
        start_control();
        state = SPINUP;
      }
      break;

    case SPINUP:
      launch(target_rpm);

      if (sdone) {
        Serial.println("Ready for launch!");
        state = LAUNCH;
      }
      break;

    case LAUNCH:
    launch(target_rpm);
      if(Serial.available() >= 1) {
        Serial.println("Stopping motors");

        pdone = false;
        ydone = false;
        target_yaw = 0;
        target_pitch = 0;
        pit = 0;
        pit_avg = 0;

        clear_serial();
        motor_stop();
        timerStop(timer1);
        start_setup();
        sdone = true;

        state = SETUP;
      } else if (!sdone) {
        Serial.println("Launch detected, spinning back up");
        state = SPINUP;
      }
      break;
  }
}

void clear_serial() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}

void start_control() {
  prev_el = 0;
  prev_er = 0;

  timerRestart(timer1);
  timerStart(timer1);
}

void start_setup() {
  timerRestart(timer0);
  timerStart(timer0);
}

void motor_stop() {
  ledcWrite(right_pwm, 0);
  ledcWrite(left_pwm, 0);
}

// CCW -> positive (low, high)
// CW - > negative
void yaw(float g) {
  if (del_m) {
      portENTER_CRITICAL_ISR(&timerMux0);
      del_m = false;
      portEXIT_CRITICAL_ISR(&timerMux0);

      float ya = float(y_count) / tpd;

      if (abs(g - ya) < .1) {
        ydone = true;
        digitalWrite(YIN1, LOW);
        digitalWrite(YIN2, LOW);
      } else if (g > ya) {
        digitalWrite(YIN1, LOW);
        digitalWrite(YIN2, HIGH);
      } else {
        digitalWrite(YIN1, HIGH);
        digitalWrite(YIN2, LOW);       
      }

      //plot_angle_data(ya, g);

  }
}

// true -> down
// false -> up
void pitch(float g) {
  if (del_m) {
    portENTER_CRITICAL_ISR(&timerMux0);
    del_m = false;
    portEXIT_CRITICAL_ISR(&timerMux0);

    float ax = myIMU.readFloatAccelX();
    float ay = myIMU.readFloatAccelY();
    float az = myIMU.readFloatAccelZ();

    float t_pit = asin(ax / sqrt(ax*ax + ay*ay + az*az));
    pit = pit + t_pit * 180.0 / 3.14;
    pit_avg = pit_avg + 1;

    if(pit_avg > 10) {
      pit = pit / pit_avg;
      if(abs(g - pit) < .2) {
        pdone = true;
        digitalWrite(LIN1, LOW);
        digitalWrite(LIN2, LOW);
      } else if (g < pit) {
        digitalWrite(LIN1, HIGH);
        digitalWrite(LIN2, LOW);
      } else {
        digitalWrite(LIN1, LOW);
        digitalWrite(LIN2, HIGH);  
      }
      //plot_angle_data(pit, g);
      pit = 0;
      pit_avg = 0;
    }
  }
}

void launch(float g) {
  if (del_t) {
    portENTER_CRITICAL(&timerMux1);
    del_t = false;
    portEXIT_CRITICAL(&timerMux1);

    float vel1 = pid_r(g);
    float vel2 = pid_l(g);

    if ((abs(g - vel1) < 50) && (abs(g - vel2) < 50)) {
      sdone = true;
    } else if (sdone && ((abs(g - vel1) > 250) && (abs(g - vel2) > 250))) {
      sdone = false;
    }

    //plot_control_data(vel1, vel2, g);
  }
}

float pid_r(float g) {
  float vel = float(r_count) / 28 * 240;
  float e = g - vel;
  int PWM = kp * e + kd * (e - prev_er);

  if (PWM > MAX_PWM) {
    PWM = MAX_PWM;
  } else if (PWM < 0) {
    PWM = 0;
  }


  ledcWrite(right_pwm, PWM);
  prev_er = e;

  return vel;
}

float pid_l(float g) {
  float vel = float(l_count) / 28 * 240;
  float e = g - vel;
  int PWM = kp * e + kd * (e - prev_el);

  if (PWM > MAX_PWM) {
    PWM = MAX_PWM;
  } else if (PWM < 0) {
    PWM = 0;
  }


  ledcWrite(left_pwm, PWM);
  prev_el = e;

  return vel;
}

void plot_control_data(float c, float g, int p) {
  Serial.print("Speed_R:");
  Serial.print(c);
  Serial.print(",");
  Serial.print("Speed_L:");
  Serial.print(g);
  Serial.print(",");
  Serial.print("Desired:");
  Serial.println(p);
}

void plot_angle_data(float curr, float goal) {
  Serial.print("Current_angle:");
  Serial.print(curr);
  Serial.print(",");
  Serial.print("Goal_angle:");
  Serial.println(goal);
}

