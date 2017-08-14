#include "Types/position.h"

namespace rosbuzz {
// Methods of Gps class
inline double Position::Gps::GetLatitude() {
	return this->latitude_;
}
inline double Position::Gps::GetLongitude() {
	return this->longitude_;
}
inline double Position::Gps::GetAltitude() {
	return this->altitude_;
}
void Position::Gps::SetLatitude(double latitude) {
	this->latitude_ = latitude;
}
void Position::Gps::SetLongitude(double longitude) {
	this->longitude_ = longitude;
}
void Position::Gps::SetAltitude(double altitude) {
	this->altitude_ = altitude;
}
// Overloaded constructor of class Ned from Gps
Position::Ned::Ned(Gps gps_current, Gps gps_origin) {
	double d_long = gps_current.GetLongitude() - gps_origin.GetLongitude();
	double d_lat = gps_current.GetLatitude() - gps_origin.GetLatitude();
	double d_alt = gps_current.GetAltitude() - gps_origin.GetAltitude();
	this->ned_x_ = DEG2RAD(d_lat) * EARTH_RADIUS;
	this->ned_y_ = DEG2RAD(d_long) * EARTH_RADIUS
			* cos(DEG2RAD(gps_current.GetLatitude()));
	this->ned_z_ = DEG2RAD(d_alt) * EARTH_RADIUS
			* sin(DEG2RAD(gps_current.GetLongitude()));
}
// Methods of class Ned
inline double Position::Ned::GetNedX() {
	return this->ned_x_;
}
inline double Position::Ned::GetNedY() {
	return this->ned_y_;
}
inline double Position::Ned::GetNedZ() {
	return this->ned_z_;
}
void Position::Ned::SetNedX(double ned_x) {
	this->ned_z_ = ned_x;
}
void Position::Ned::SetNedY(double ned_y) {
	this->ned_y_ = ned_y;
}
void Position::Ned::SetNedZ(double ned_z) {
	this->ned_z_ = ned_z;
}
// Overloaded constructor of class Cartesian from Gps
Position::Cartesian::Cartesian(Gps gps) {
	//TODO Construction of cartesian class  
}
//Methods of class Cartesian
inline double Position::Cartesian::GetCartesianX() {
	return this->cartesian_x_;
}
inline double Position::Cartesian::GetCartesianY() {
	return this->cartesian_y_;
}
inline double Position::Cartesian::GetCartesianZ() {
	return this->cartesian_z_;
}
void Position::Cartesian::SetCartesianX(double cartesian_x) {
	this->cartesian_x_ = cartesian_x;
}
void Position::Cartesian::SetCartesianY(double cartesian_y) {
	this->cartesian_y_ = cartesian_y;
}
void Position::Cartesian::SetCartesianZ(double cartesian_z) {
	this->cartesian_z_ = cartesian_z;
}
// Overloaded constructor of class polar from gps
Position::Polar::Polar(Gps gps) {
	//TODO Constructor of Polar class
}
// Methods of class Polar
inline double Position::Polar::GetPolarR() {
	return this->polar_r_;
}
inline double Position::Polar::GetPolarTheta() {
	return this->polar_theta_;
}
inline double Position::Polar::GetPolarPhi() {
	return this->polar_phi_;
}
void Position::Polar::SetPolarR(double polar_r) {
	this->polar_r_ = polar_r;
}
void Position::Polar::SetPolarTheta(double polar_theta) {
	this->polar_theta_ = polar_theta;
}
void Position::Polar::SetPolarPhi(double polar_phi) {
	this->polar_phi_ = polar_phi;
}
}
