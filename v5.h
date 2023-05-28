#ifndef ROBOT_V5_H
#define ROBOT_V5_H
#include "AUBIE-VEX-Core/robotmath.h"
#include <vector>
#include <iostream>

namespace vex {
	namespace rotationUnits {};
	class encoder;
	class motor;
	class inertial;
};

namespace simulator {
	const int cyclesPerMsec = 10;
	const double timeStep = 0.001 / cyclesPerMsec;

	const double MU_STATIC = 0.8;
	const double MU_KINETIC = 0.7;
	const double g = 9.81; //M/s^2

	class RobotBase {
	protected:
		double mass = 10; //Kg

		Point2d center;
		double heading;

		Vector2d vel = Vector2d(0, 0); //Meters per second in abs frame
		double angularVel = 0;

		std::vector<vex::encoder*> encoders;
		std::vector<vex::motor*> motors;
		vex::inertial* inertialSensor = NULL;

		virtual void updatePhysics(double deltaTime); //Change Pos, Change facing vector
	public:
		RobotBase(Point2d startPos, double startHeadInDeg);
		~RobotBase();

		Point2d getCenter();
		void setHeading(double head, bool inDeg = true);
		double getHeading();
		Vector2d getFacingVector();
		Vector2d getAbsVel();
		Vector2d getRealitiveVel();
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
	public:
		TankRobot(Point2d startPos, double startHeadInDeg, vex::motor* left, vex::motor* right);
	};
}

//Phycics and vex sim
namespace vex {
	namespace rotationUnits {
		const int deg = 0;
		const int rev = 1;
	}

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

		void setPosition(double p, int rotUnits = vex::rotationUnits::rev);
		double position(int rotUnits = vex::rotationUnits::rev);
	};

	class motor : public encoder {
	protected:
		const double MAX_POWER = 12.75; //Watts
		const double STALL_TORQUE = 2.1; //Nm
		const double MAX_PCT_ACCEL = 200; //Pct per second
		const double K_TORQUE_RootPower = STALL_TORQUE / sqrtf(MAX_POWER);
		const double TORQUE_LIMIT = 2;
		const double SPEED_LIMIT_PERCENTAGE = 100;

		double percentPower = 0; //TODO Clamp between -1 and 1

		double angularVelcoity; //PercentPower*MaxRPM in angular rotation

	public:
		motor(Vector2d offset, Vector2d rotation, double wheelRadiusIn) : encoder(offset, rotation, wheelRadiusIn) {};
		//TODO Encoder wheel function when building speed needs to take into account slipage and slideing friction if vel is higher than roll speed

		void applyAcceleration(simulator::RobotBase* robot, double deltaTime);
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
		void setRotation(double in, int type = vex::rotationUnits::deg);
	};
}
#endif