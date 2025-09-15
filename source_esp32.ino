/*
  Quadruped robot arduino sketch.
  3/10/2020 by Alexandros Petkos
  Updates available at https://github.com/maestrakos/warp

  This kinematics sketch is placed under CC-BY.

  This file is part of warp_kinematics.
*/

#include "datatypes.h"

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// #include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps20.h>
// MPU6050 mpu;

#include <Bluepad32.h>

/*
  ==============================
  IMPORTANT PARAMETERS
  ==============================
*/
const float frequency = 440.0; // ## Hz

/// Kinematics Parameters
const datatypes::Transform body_transform = {
  {0, 0, 0},  // ## {mm, mm, mm}
  {0, 0, 0},  // ## {deg, deg, deg}
  {300, 40, 180} // ## {mm, mm, mm}
};

const datatypes::Vector p_joint_origin[] = {
  { -50, 0, 0},
  { +50, 0, 0},
  { +50, 0, 0},
  { -50, 0, 0}
};
const float bone_length = 105; // ## mm

const datatypes::Vector step_extent = {40, 40, 26}; 
float vrt_offset = -16.50;
float hrz_offset = -6.00;

float base_offset[] = { 0, -1, 0, -2};
const float precision = 0.01;

void setup() {
  Serial.begin(115200);

  init_hardware();
  init_input();
}

//: gait states
datatypes::Vector2D _direction = {0, 0};
float turn = 0;
float height = 0;

int state = 0; 
float _period = 1.0;

datatypes::Rotator _sRotation;

unsigned long duration;
int sample_sum, sample_num = 10,
                sample_index;
float freq;

void loop() {
  duration = millis();

  handle_hardware();
  handle_kinematics(_direction, turn, height, _period);

  handle_input();

  if (Serial.available())
    handle_serial();
}

/*
  ==============================
  CONTROLLER INPUT
  ==============================
*/

ControllerPtr myController;

float vo, ho;
void init_input() {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  vo = vrt_offset;
  ho = hrz_offset;
}

bool _tb = false;
float stick_min = 6.f;
float lx, ly, rx, ry;

void onConnectedController(ControllerPtr ctl) {
  Serial.println("Controller connected!");
  myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected!");
  if (myController == ctl)
    myController = nullptr;
}

void handle_input() {
  BP32.update();

  if (myController && myController->isConnected()) {
    // Joystick input
    lx = inter(lx, myController->axisX() / 4.f, 0.5f);
    ly = inter(ly, myController->axisY() / 4.f, 0.5f);
    rx = inter(rx, myController->axisRX() / 4.f, 0.5f);
    ry = inter(ry, myController->axisRY() / 4.f, 0.5f);

    // Movement
    if (abs(lx) > stick_min) {
      float x0 = lx - stick_min * sign(lx);
      if (state == 1) {
        _direction.y = 0;
      } else if (state != 4) {
        _direction.y = x0 / 2;
      }
    } else {
      _direction.y = 0;
    }

    if (abs(ly) > stick_min) {
      float y0 = ly - stick_min * sign(ly);
      if (state == 1) {
        _direction.x = y0 / 10.f;
        if (y0 > 0)
          vrt_offset = inter(vrt_offset, vo - 6.f, 2.f);
        else
          vrt_offset = inter(vrt_offset, vo + 3.f, 2.f);
      } else if (state != 4) {
        _direction.x = y0 / 2;
        vrt_offset = vo;
      }
    } else {
      _direction.x = 0;
      vrt_offset = vo;
    }

    if (abs(rx) > stick_min) {
      float x1 = rx - stick_min * sign(rx);
      if (state == 1)
        turn = x1 / 16.f;
      else if (state != 4)
        turn = x1;
    } else {
      turn = 0;
    }

    if (abs(ry) > stick_min) {
      float y1 = ry - stick_min * sign(ry);
      height = y1;
    } else {
      height = 0;
    }

    // Button B
    if (myController->b()) { 
      if (_tb == true) {
        _tb = false;
        state++;
        if (state > 4) state = 0;

            // In ra mode hiện tại
          Serial.print("Switched to mode: ");
          Serial.println(state);
      }
    } else {
      _tb = true;
    }
  } else {
    // No controller
    _direction = {0, 0};
    turn = 0;
    height = 0;
    vrt_offset = vo;
  }
}

/*
  ==============================
  SERIAL INPUT
  ==============================
*/

#define _mode 1
void handle_serial() {
  int i = 0; float buff[3] = {0, 0, 0};
  String s_buff = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 13 || c == 32 || c == '\n') {
      buff[i] = s_buff.toFloat();
      s_buff = "";
      i++;
    } else
      s_buff += c;
  }

  if (_mode == 0)
    commands_exe(buff[0], buff[1], buff[2]);
  else if (_mode == 1)
    if (state == 4) {
      _direction = {buff[0], buff[1]};
      turn = buff[2];
    }
}

/*
  ==============================
  HELPERS
  ==============================
*/

float inter(float in, float en, float pl) {
  if (in < en - pl) {
    return ((in * 1000.f) + (pl * 1000.f)) / 1000.0;
  } else if (in > en + pl) {
    return ((in * 1000.f) - (pl * 1000.f)) / 1000.0;
  } else return en;
}

#define properties 0
void commands_exe(float val1, float val2, float val3) {
  if (properties == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" joint ");
    Serial.print(joint);
    Serial.print(" set to ");
    Serial.print(servo);
    Serial.print(".\n");

    set_servo(leg, joint, servo);
  }
  else if (properties == 1) {
    int leg = val1;
    int empty = val2;
    int ammount = val3;
    Serial.print("- leg ");
    Serial.print(leg);
    Serial.print(" null ");
    Serial.print(empty);
    Serial.print(" set to ");
    Serial.print(ammount);
    Serial.print(".\n");

    base_offset[leg] = ammount;
  }
}
