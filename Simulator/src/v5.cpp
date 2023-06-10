#include "v5.h"

using namespace vex;

encoder::encoder(Vector2d offset, Vector2d rotation, double wheelRadiusIn) {
	offsetFromCenter = offset;
	rotationRealitiveToFrontOfRobot = rotation.getUnitVector();
	wheelRadius = wheelRadiusIn;
}

void encoder::setPosition(double p, vex::rotationUnits rotUnits) {
	revolutions = p;
}

double encoder::position(vex::rotationUnits rotUnits) {
	return revolutions;
}

void encoder::updateEnc(simulator::RobotBase* robot, double deltaTime) {
	double arc = (robot->getRealitiveVel() * deltaTime).dot(rotationRealitiveToFrontOfRobot) + (offsetFromCenter.cross(rotationRealitiveToFrontOfRobot)) * robot->getAngularVel() * deltaTime;
	revolutions += arc / (M_2PI*wheelRadius);
}

void motor::updatePhysics(simulator::RobotBase* robot, double deltaTime) {
	double delta = setPower - percentPower;
	if (abs(delta / deltaTime) > MAX_PCT_ACCEL) {
		delta = sign(delta) * MAX_PCT_ACCEL * deltaTime;
	}
	percentPower += delta;
	percentPower = fclamp(percentPower, -1, 1);
};

void motor::setVelocity(double value, vex::velocityUnits units) {
	setPower = fclamp(value / 100, -1, 1);
};

double motor::velocity(vex::velocityUnits units) {
	return percentPower * 100;
};

double motor::getAngularSpeed() {
	return percentPower* MAX_ANGULAR_SPEED;
};

inertial::inertial() {

};

void inertial::update(simulator::RobotBase* robot, double deltaTime) {
	CWAngle -= radToDeg(robot->getAngularVel() * deltaTime);
};

bool inertial::installed() {
	return isInstalled;
}

double inertial::angle() {
	return CWAngle;
}

void inertial::setRotation(double in, vex::rotationUnits type) {
	CWAngle = in;
}

using namespace simulator;

RobotBase::RobotBase(Point2d startPos, double startHeadInDeg) {
	center = startPos;
	heading = normalizeAngle(degToRad(startHeadInDeg));
};

RobotBase::~RobotBase() {
	//encoders.clear();
	//motors.clear();
};

Point2d RobotBase::getCenter() {
	return center;
}

void RobotBase::setHeading(double head, bool inDeg) {
	heading = normalizeAngle(inDeg ? degToRad(head) : head);
};

double RobotBase::getHeading() {
	return heading;
};

Vector2d RobotBase::getFacingVector() {
	return Vector2d(1, heading, false);
};

Vector2d RobotBase::getAbsVel() {
	return vel;
}

Vector2d RobotBase::getRealitiveVel() {
	return vel.getRotatedVector(M_PI_2 - getHeading(), false);
}

void RobotBase::setAbsVel(Vector2d newVel) {
	vel = newVel;
}

void RobotBase::setRealitiveVel(Vector2d newVel) {
	vel = newVel.getRotatedVector(getHeading() - M_PI_2, false);
}

void RobotBase::setAngularVel(double newAngVel) {
	angularVel = newAngVel;
}

double RobotBase::getAngularVel() {
	return angularVel;
}

void RobotBase::add(vex::motor* motorIn) {
	motors.push_back(motorIn);
}

void RobotBase::add(vex::encoder* encoderIn) {
	encoders.push_back(encoderIn);
}

void RobotBase::add(vex::inertial* inertialSensorIn) {
	inertialSensor = inertialSensorIn;
}

void RobotBase::updatePhysics(double deltaTime) {
	for (unsigned int i = 0; i < motors.size(); i++) {
		motors[i]->updatePhysics(this, deltaTime);
	}

	center = vel * deltaTime + center;
	heading += angularVel * deltaTime;
	heading = heading;

	if (inertialSensor != NULL) {
		inertialSensor->update(this, deltaTime);
	}
	for (unsigned int i = 0; i < encoders.size(); i++) {
		encoders[i]->updateEnc(this, deltaTime);
	}
	for (unsigned int i = 0; i < motors.size(); i++) {
		motors[i]->updateEnc(this, deltaTime); //TODO Needs to take into account slipage
	}
};

void RobotBase::update(int msecs) {
	for (int z = 0; z < msecs; z++) {
		for (int i = 0; i < cyclesPerMsec; i++) {
			updatePhysics(timeStep);
		}
	}
};

TankRobot::TankRobot(Point2d startPos, double startHeadInDeg, vex::motor* left, vex::motor* right, double widthIn) :
	RobotBase(startPos, startHeadInDeg) {
	add(left);
	add(right);
	leftMotor = left;
	rightMotor = right;
	width = widthIn;
};

void TankRobot::updatePhysics(double deltaTime) {
	vel = Vector2d(0.5 * (leftMotor->getAngularSpeed() + rightMotor->getAngularSpeed()) * wheelRadius, heading, false);
	angularVel = (rightMotor->getAngularSpeed() - leftMotor->getAngularSpeed()) * wheelRadius / width;
	RobotBase::updatePhysics(deltaTime);
};