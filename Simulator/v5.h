#ifndef ROBOT_V5_H
#define ROBOT_V5_H
#include "robotmath.h"
#include <vector>
#include <iostream>
const double wheelRadius = 3; //Inches

//TODO GPS; Returns postion +- noise
////Can be used to test kalman and various filter implementation

namespace vex {
	const enum rotationUnits {
		deg,
		rev
	};
	const enum velocityUnits {
		pct,
	};
	class encoder;
	class motor;
	class inertial;
};

namespace simulator {
	const int cyclesPerMsec = 10;
	const double timeStep = 0.001 / cyclesPerMsec;

	class RobotBase {
	protected:
		Point2d center;
		double heading;

		Vector2d vel = Vector2d(0, 0); //Meters per second in abs frame
		double angularVel = 0;

		double widthInches;
		double heightInches;


		std::vector<vex::encoder*> encoders;
		std::vector<vex::motor*> motors;
		vex::inertial* inertialSensor = NULL;

		virtual void updatePhysics(double deltaTime); //Change Pos, Change facing vector
	public:
		RobotBase(Point2d startPos, double startHeadInDeg, double width, double height);
		~RobotBase();

		Point2d getCenter();
		void setHeading(double head, bool inDeg = true);
		double getHeading();
		Vector2d getFacingVector();
		Vector2d getAbsVel();
		Vector2d getRealitiveVel();
		Vector2d size();
		void setAbsVel(Vector2d newVel);
		void setRealitiveVel(Vector2d newVel);
		void setAngularVel(double newAngVel);
		double getAngularVel();

		void add(vex::motor* motorIn);
		void add(vex::encoder* encoderIn);
		void add(vex::inertial* inertialSensorIn);

		void update(int msecs); //Call update psyics msec times
	};

	class TankRobot : public RobotBase {
	protected:
		vex::motor* leftMotor;
		vex::motor* rightMotor;
		double wheelWidth;

		void updatePhysics(double deltaTime) override; //Change Pos, Change facing vector
	public:
		TankRobot(Point2d startPos, double startHeadInDeg, vex::motor* left, vex::motor* right, double wheelWidthIn, double widthInchesIn, double heightInchesIn);
	};
}

//Phycics and vex sim
namespace vex {
	class encoder {
	protected:
		Vector2d offsetFromCenter;
		Vector2d rotationRealitiveToFrontOfRobot; //<0, 1> would be facing same direction
		double revolutions = 0;
		friend simulator::RobotBase;
		double wheelRadius;

		void updateEnc(simulator::RobotBase* robot, double deltaTime);

	public:
		encoder(Vector2d offset, Vector2d rotation, double wheelRadiusIn);

		void setPosition(double p, vex::rotationUnits rotUnits = vex::rotationUnits::rev);
		double position(vex::rotationUnits rotUnits = vex::rotationUnits::rev);
	};

	class motor : public encoder {
	protected:
		const double MAX_PCT_ACCEL = 200.00; //Pct per second
		const double MAX_ANGULAR_SPEED = 40 / wheelRadius;
		friend simulator::RobotBase;
		friend simulator::TankRobot;

		double percentPower = 0; //TODO Clamp between -1 and 1
		double setPower = 0; //Clamp between -1 and 1

		void updatePhysics(simulator::RobotBase* robot, double deltaTime);
		double getAngularSpeed();
	public:
		motor(Vector2d offset, Vector2d rotation, double wheelRadiusIn) : encoder(offset, rotation, wheelRadiusIn) {};
		//TODO Encoder wheel function when building speed needs to take into account slipage and slideing friction if vel is higher than roll speed

		void setVelocity(double value, vex::velocityUnits units = vex::velocityUnits::pct);
		double velocity(vex::velocityUnits units = vex::velocityUnits::pct);
	};

	class inertial {
	private:
		bool isInstalled = true;
		double CWAngle = 0;
		friend simulator::RobotBase;
		void update(simulator::RobotBase* robot, double deltaTime);

	public:
		inertial();

		bool installed();
		double angle();
		void setRotation(double in, vex::rotationUnits type = vex::rotationUnits::deg);
	};
}
#endif