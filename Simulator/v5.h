#ifndef ROBOT_V5_H
#define ROBOT_V5_H
#include "robotmath.h"
#include <vector>
#include <iostream>
#include <stdarg.h>

namespace simulator {
	const int cyclesPerMsec = 10;
	const double timeStep = 0.001 / cyclesPerMsec;
	const double wheelRadius = 3; //Inches
	const double MAX_SPEED = 40; //Inches per second
	const double MAX_PCT_ACCEL = 200.00; //Pct per second
}

namespace vex {
	const enum class directionType {
		fwd,
		rev,
		undefined
	};
	const enum class rotationUnits {
		deg,
		rev
	};
	const enum class velocityUnits {
		pct,
		dps,
		rpm
	};
	const enum class percentUnits {
		pct
	};
	class encoder;
	class motor;
	class inertial;
}

namespace simulator {

	class RobotBase {
	protected:
		Point2d center;
		double heading;

		Vector2d vel = Vector2d(0, 0); //Speed in abs frame
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

		void setPosition(double p, vex::rotationUnits rotUnits);
		double position(vex::rotationUnits rotUnits);
	};

	class motor : public encoder {
	protected:
		const double MAX_ANGULAR_SPEED = simulator::MAX_SPEED / wheelRadius;
		friend simulator::RobotBase;
		friend simulator::TankRobot;

		double percentPower = 0; //Clamp between -1 and 1
		double setPower = 0; //Clamp between -1 and 1

		void updatePhysics(simulator::RobotBase* robot, double deltaTime);
		double getAngularSpeed();
	public:
		motor(Vector2d offset, Vector2d rotation, double wheelRadiusIn);

		void setVelocity(double value, vex::velocityUnits units);
		void setVelocity(double value, vex::percentUnits units);
		double velocity(vex::velocityUnits units);
		double velocity(vex::percentUnits units);

		//UNIMPLEMENTED
		void spin(vex::directionType) {};
		void stop() {};
	};

	class motor_group {
	private:
		std::vector<motor*> motors;

	public:
		motor_group();
		motor_group(motor& A);
		void setVelocity(double value, vex::velocityUnits units);
		void setVelocity(double value, vex::percentUnits units);
		double velocity(vex::velocityUnits units);
		double velocity(vex::percentUnits units);

		//UNIMPLEMENTED
		void spin(vex::directionType) {};
		void stop() {};
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
		void setRotation(double in, vex::rotationUnits type);
	};
}

#endif