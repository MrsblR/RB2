#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace webots;
using namespace std;

#define TIME_STEP 64

int main(int argc, char **argv) {

  Supervisor robot;

  // Motores
  Motor *leftMotor  = robot.getMotor("left wheel motor");
  Motor *rightMotor = robot.getMotor("right wheel motor");

  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);

  // Sensores
  vector<DistanceSensor*> ps(8);
  for (int i = 0; i < 8; i++) {
    ps[i] = robot.getDistanceSensor("ps" + to_string(i));
    ps[i]->enable(TIME_STEP);
  }

  // Supervisor
  Node *selfNode   = robot.getSelf();
  Node *targetNode = robot.getFromDef("TARGET");

  Field *selfTr   = selfNode->getField("translation");
  Field *targetTr = targetNode->getField("translation");

  while (robot.step(TIME_STEP) != -1) {

    // Sensado
    double left  = ps[5]->getValue() + ps[6]->getValue();
    double front = ps[0]->getValue() + ps[7]->getValue();
    double right = ps[1]->getValue() + ps[2]->getValue();

    int s_left  = left  > 110;
    int s_front = front > 120;
    int s_right = right > 110;

    // Posiciones
    const double *r = selfTr->getSFVec3f();
    const double *t = targetTr->getSFVec3f();

    double dx = t[0] - r[0];
    double dz = t[2] - r[2];
    double dist = sqrt(dx*dx + dz*dz);

    // Acción aleatoria (placeholder)
    int action = rand() % 3;

    // Actuar
    if (action == 0) {
      leftMotor->setVelocity(3.9);
      rightMotor->setVelocity(3.9);
    } else if (action == 1) {
      leftMotor->setVelocity(-2.6);
      rightMotor->setVelocity(2.6);
    } else {
      leftMotor->setVelocity(2.6);
      rightMotor->setVelocity(-2.6);
    }
  }

  return 0;
}
