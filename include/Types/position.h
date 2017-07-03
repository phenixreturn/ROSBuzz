#pragma once
namespace rosbuzz{
	class Position;
}
#include <math.h>
#define EARTH_RADIUS (double) 6371000.0
#define DEG2RAD(DEG) ((DEG)*((M_PI)/(180.0)))

namespace rosbuzz {
class Position {
public:
	// Class for Geodic coordinate system
	class Gps {
	public:
		Gps():latitude_(0.0), longitude_(0.0), altitude_(0.0) {}
		Gps(double latitude, double longitude, double altitude) :
				latitude_(latitude), longitude_(longitude), altitude_(altitude) {}
		inline double GetLatitude();
		inline double GetLongitude();
		inline double GetAltitude();
		void SetLatitude(double latitude);
		void SetLongitude(double longitude);
		void SetAltitude(double altitude);
	private:
		double latitude_;
		double longitude_;
		double altitude_;
	};
	//class for NED coordiante system
	class Ned {
	public:
		Ned():ned_x_(0), ned_y_(0), ned_z_(0) {}
		Ned(double ned_x, double ned_y, double ned_z) :
				ned_x_(ned_x), ned_y_(ned_y), ned_z_(ned_z) {
		}
		Ned(Gps gps_current, Gps gps_origin);
		Gps ToGps();
		inline double GetNedX();
		inline double GetNedY();
		inline double GetNedZ();
		void SetNedX(double ned_x);
		void SetNedY(double ned_y);
		void SetNedZ(double ned_z);
	private:
		double ned_x_;
		double ned_y_;
		double ned_z_;
	};
	//class for cartesian coordiante system
	class Cartesian {
	public:
		Cartesian() :
				cartesian_x_(0), cartesian_y_(0), cartesian_z_(0) {
		}
		Cartesian(double cartesian_x, double cartesian_y, double cartesian_z) :
				cartesian_x_(cartesian_x), cartesian_y_(cartesian_y), cartesian_z_(
						cartesian_z) {
		}
		Cartesian(Gps gps);
		Gps ToGps();
		inline double GetCartesianX();
		inline double GetCartesianY();
		inline double GetCartesianZ();
		void SetCartesianX(double cartesian_x);
		void SetCartesianY(double cartesian_y);
		void SetCartesianZ(double cartesian_z);
	private:
		double cartesian_x_;
		double cartesian_y_;
		double cartesian_z_;
	};
	//class for polar coordiante system
	class Polar {
	public:
		Polar() :
				polar_r_(0), polar_theta_(0), polar_phi_(0) {
		}
		Polar(double polar_r, double polar_theta, double polar_phi) :
				polar_r_(polar_r), polar_theta_(polar_theta), polar_phi_(
						polar_phi) {
		}
		Polar(Gps gps);
		Gps ToGps();
		inline double GetPolarR();
		inline double GetPolarTheta();
		inline double GetPolarPhi();
		void SetPolarR(double polar_r);
		void SetPolarTheta(double polar_theta);
		void SetPolarPhi(double polar_phi);
	private:
		double polar_r_;
		double polar_theta_;
		double polar_phi_;

	};

};

}
