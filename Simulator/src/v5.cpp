#include "v5.h"

using namespace vex;

encoder::encoder(Vector2d offset, Vector2d rotation, double wheelRadiusIn) {
	offsetFromCenter = offset;
	rotationRealitiveToFrontOfRobot = rotation.getUnitVector();
	wheelRadius = wheelRadiusIn;
}

void encoder::setPosition(double p, vex::rotationUnits rotUnits) {
	if (rotUnits == vex::rotationUnits::rev) {
		revolutions = p;
	}
	else if (rotUnits == vex::rotationUnits::deg) {
		revolutions = p / 360.0;
	}
}

double encoder::position(vex::rotationUnits rotUnits) {
	if (rotUnits == vex::rotationUnits::rev) {
		return revolutions;
	}
	else if (rotUnits == vex::rotationUnits::deg) {
		return revolutions * 360.0;
	}
}

void encoder::updateEnc(simulator::RobotBase* robot, double deltaTime) {
	double arc = (robot->getRealitiveVel() * deltaTime).dot(rotationRealitiveToFrontOfRobot) + (offsetFromCenter.cross(rotationRealitiveToFrontOfRobot)) * robot->getAngularVel() * deltaTime;
	revolutions += arc / (M_2PI*wheelRadius);
}

motor::motor(Vector2d offset, Vector2d rotation, double wheelRadiusIn) : encoder(offset, rotation, wheelRadiusIn) {};

void motor::updatePhysics(simulator::RobotBase* robot, double deltaTime) {
	double delta = setPower - percentPower;
	if (abs(delta / deltaTime) > simulator::MAX_PCT_ACCEL) {
		delta = sign(delta) * simulator::MAX_PCT_ACCEL * deltaTime;
	}
	percentPower += delta;
	percentPower = fclamp(percentPower, -1, 1);
};

void motor::setVelocity(double value, vex::velocityUnits units) {
	if (units == vex::velocityUnits::pct) {
		setPower = fclamp(value / 100, -1, 1);
	}
	else if (units == vex::velocityUnits::dps) {
		setPower = fclamp(degToRad(value) / MAX_ANGULAR_SPEED, -1, 1);
	}
	else if (units == vex::velocityUnits::rpm) {
		setPower = fclamp(degToRad(value * 6) / MAX_ANGULAR_SPEED, -1, 1);
	}
};

void motor::setVelocity(double value, vex::percentUnits units) {
	setVelocity(value, vex::velocityUnits::pct);
};

double motor::velocity(vex::velocityUnits units) {
	if (units == vex::velocityUnits::pct) {
		return percentPower * 100;
	}
	else if (units == vex::velocityUnits::dps) {
		return radToDeg(setPower* MAX_ANGULAR_SPEED);
	}
	else if (units == vex::velocityUnits::rpm) {
		return radToDeg(setPower * MAX_ANGULAR_SPEED) / 6.0;
	}
};

double motor::velocity(vex::percentUnits units) {
	return velocity(vex::velocityUnits::pct);
};

double motor::getAngularSpeed() {
	return percentPower* MAX_ANGULAR_SPEED;
};

motor_group::motor_group() {};

motor_group::motor_group(motor& A) {
	motors.push_back(&A);
};

void motor_group::setVelocity(double value, vex::velocityUnits units) {
	for (motor* m : motors) {
		m->setVelocity(value, units);
	}
};

void motor_group::setVelocity(double value, vex::percentUnits units) {
	for (motor* m : motors) {
		m->setVelocity(value, units);
	}
};

double motor_group::velocity(vex::velocityUnits units) {
	double avg = 0;
	for (motor* m : motors) {
		avg += m->velocity(units);
	}
	return avg / motors.size();
};

double motor_group::velocity(vex::percentUnits units) {
	double avg = 0;
	for (motor* m : motors) {
		avg += m->velocity(units);
	}
	return avg / motors.size();
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
	if (type == vex::rotationUnits::deg){
		CWAngle = in;
	}
	else if (type == vex::rotationUnits::rev) {
		CWAngle = in * 360;
	}
}

using namespace simulator;

RobotBase::RobotBase(Point2d startPos, double startHeadInDeg, double widthInchesIn, double heightInchesIn) {
	center = startPos;
	heading = normalizeAngle(degToRad(startHeadInDeg));
	widthInches = widthInchesIn;
	heightInches = heightInchesIn;
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

Vector2d RobotBase::size() {
	return Vector2d(widthInches, heightInches);
};

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

	//TODO Apply external phycics here

	center = vel * deltaTime + center;
	heading += angularVel * deltaTime;
	heading = heading;

	//End apply external phycics

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

TankRobot::TankRobot(Point2d startPos, double startHeadInDeg, vex::motor* left, vex::motor* right, double wheelWidthIn, double widthInchesIn, double heightInchesIn) :
	RobotBase(startPos, startHeadInDeg, widthInchesIn, heightInchesIn) {
	add(left);
	add(right);
	leftMotor = left;
	rightMotor = right;
	wheelWidth = wheelWidthIn;
};

void TankRobot::updatePhysics(double deltaTime) {
	vel = Vector2d(0.5 * (leftMotor->getAngularSpeed() + rightMotor->getAngularSpeed()) * wheelRadius, heading, false);
	angularVel = (rightMotor->getAngularSpeed() - leftMotor->getAngularSpeed()) * wheelRadius / wheelWidth;
	RobotBase::updatePhysics(deltaTime);
};