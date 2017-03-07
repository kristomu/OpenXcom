/*
 * Copyright 2010-2016 OpenXcom Developers.
 *
 * This file is part of OpenXcom.
 *
 * OpenXcom is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OpenXcom is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenXcom.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "MovingTarget.h"
#include "../fmath.h"
#include "SerializationHelper.h"
#include "../Engine/Options.h"

namespace OpenXcom
{

/**
 * Initializes a moving target with blank coordinates.
 */
MovingTarget::MovingTarget() : Target(), _dest(0), _speedLon(0.0), _speedLat(0.0), _speedRadian(0.0), _meetPointLon(0.0), _meetPointLat(0.0), _speed(0)
{
}

/**
 * Make sure to cleanup the target's destination followers.
 */
MovingTarget::~MovingTarget()
{
	if (_dest != 0 && !_dest->getFollowers()->empty())
	{
		for (std::vector<Target*>::iterator i = _dest->getFollowers()->begin(); i != _dest->getFollowers()->end(); ++i)
		{
			if ((*i) == this)
			{
				_dest->getFollowers()->erase(i);
				break;
			}
		}
	}
}

/**
 * Loads the moving target from a YAML file.
 * @param node YAML node.
 */
void MovingTarget::load(const YAML::Node &node)
{
	Target::load(node);
	_speedLon = node["speedLon"].as<double>(_speedLon);
	_speedLat = node["speedLat"].as<double>(_speedLat);
	_speedRadian = node["speedRadian"].as<double>(_speedRadian);
	_speed = node["speed"].as<int>(_speed);
}

/**
 * Saves the moving target to a YAML file.
 * @return YAML node.
 */
YAML::Node MovingTarget::save() const
{
	YAML::Node node = Target::save();
	if (_dest != 0)
	{
		node["dest"] = _dest->saveId();
	}
	node["speedLon"] = serializeDouble(_speedLon);
	node["speedLat"] = serializeDouble(_speedLat);
	node["speedRadian"] = serializeDouble(_speedRadian);
	node["speed"] = _speed;
	return node;
}

/**
 * Returns the destination the moving target is heading to.
 * @return Pointer to destination.
 */
Target *MovingTarget::getDestination() const
{
	return _dest;
}

/**
 * Changes the destination the moving target is heading to.
 * @param dest Pointer to destination.
 */
void MovingTarget::setDestination(Target *dest)
{
	// Remove moving target from old destination's followers
	if (_dest != 0)
	{
		for (std::vector<Target*>::iterator i = _dest->getFollowers()->begin(); i != _dest->getFollowers()->end(); ++i)
		{
			if ((*i) == this)
			{
				_dest->getFollowers()->erase(i);
				break;
			}
		}
	}
	_dest = dest;
	// Add moving target to new destination's followers
	if (_dest != 0)
	{
		_dest->getFollowers()->push_back(this);
	}
	calculateSpeed();
}

/**
 * Returns the speed of the moving target.
 * @return Speed in knots.
 */
int MovingTarget::getSpeed() const
{
	return _speed;
}

/**
 * Returns the radial speed of the moving target.
 * @return Speed in 1 / 5 sec.
 */
double MovingTarget::getSpeedRadian() const
{
	return _speedRadian;
}

/**
 * Changes the speed of the moving target
 * and converts it from standard knots (nautical miles per hour)
 * into radians per 5 in-game seconds.
 * @param speed Speed in knots.
 */
void MovingTarget::setSpeed(int speed)
{
	_speed = speed;
	// Each nautical mile is 1/60th of a degree.
	// Each hour contains 720 5-seconds.
	_speedRadian = _speed * (1 / 60.0) * (M_PI / 180) / 720.0;
	calculateSpeed();
}

/// Calculates delta longitude and latitude to get to a meet point.
void MovingTarget::calcdLonLat(double & dLon, double & dLat)
{
	dLon = sin(_meetPointLon - _lon) * cos(_meetPointLat);
	dLat = cos(_lat) * sin(_meetPointLat) - sin(_lat) * cos(_meetPointLat) * 
		cos(_meetPointLon - _lon);
}

/**
 * Calculates the speed vector based on the
 * great circle distance to destination and
 * current raw speed.
 */
void MovingTarget::calculateSpeed()
{
	calculateMeetPoint();
	if (_dest != 0)
	{
		double dLon, dLat, length;
		calcdLonLat(dLon, dLat);
		length = sqrt(dLon * dLon + dLat * dLat);
		_speedLat = dLat / length * _speedRadian;
		_speedLon = dLon / length * _speedRadian / cos(_lat + _speedLat);

		// Check for invalid speeds when a division by zero occurs due to near-zero values
		if (!(_speedLon == _speedLon) || !(_speedLat == _speedLat))
		{
			_speedLon = 0;
			_speedLat = 0;
		}
	}
	else
	{
		_speedLon = 0;
		_speedLat = 0;
	}
}

/**
 * Checks if the moving target has reached its destination.
 * @return True if it has, False otherwise.
 */
bool MovingTarget::reachedDestination() const
{
	if (_dest == 0)
	{
		return false;
	}
	return ( AreSame(_dest->getLongitude(), _lon) && AreSame(_dest->getLatitude(), _lat) );
}

/**
 * Executes a movement cycle for the moving target.
 */
void MovingTarget::move()
{
	calculateSpeed();
	if (_dest != 0)
	{
		if (getDistance(_dest) > _speedRadian)
		{
			setLongitude(_lon + _speedLon);
			setLatitude(_lat + _speedLat);
		}
		else
		{
			setLongitude(_dest->getLongitude());
			setLatitude(_dest->getLatitude());
		}
	}
}

void MovingTarget::cross(double x1, double y1, double z1, double x2, 
	double y2, double z2, double & xout, double & yout, double & zout) {

	xout = y1 * z2 - z1 * y2;
	yout = z1 * x2 - x1 * z2;
	zout = x1 * y2 - y1 * x2;
}

void MovingTarget::getGreatCirclePoint(double t, double uX, double uY, 
	double uZ, double wX, double wY, double wZ, double & trX, 
	double & trY, double & trZ) {

	trX = uX * cos(t) + wX * sin(t);
	trY = uY * cos(t) + wY * sin(t);
	trZ = uZ * cos(t) + wZ * sin(t);
}

double MovingTarget::evaluateInterceptProximity(double t, double ourX, 
	double ourY, double ourZ, double uX, double uY, double uZ, double wX, 
	double wY, double wZ, double ufoSpeedRadian, double ourSpeedRadian) 
{
	double trX, trY, trZ;
	getGreatCirclePoint(t, uX, uY, uZ, wX, wY, wZ, trX, trY, trZ);

	double dist = acosl(ourX*trX + ourY*trY + ourZ*trZ);
	double reachable = t * ourSpeedRadian/ufoSpeedRadian;
	return fabs(dist - reachable);
}

/// Find an intercept point betweeen tMin and tMax assuming there's only one
/// closest point in between them. Uses golden section search.
std::pair<double, double> MovingTarget::findMinimum(double tMin, double tMax, 
	double ourX, double ourY, double ourZ, double uX, double uY, double uZ, 
	double wX, double wY, double wZ, double ufoSpeedRadian, 
	double ourSpeedRadian)
{
	double gr = (sqrt(5) + 1)/2.0, tolerance = 1e-7;

	double c = tMin, d = tMax, a = tMin, b = tMax;
	double f_c = INFINITY, f_d = INFINITY;

	while (fabs(c-d) > tolerance)
	{
		c = b - (b-a)/gr;
		d = a + (b-a) / gr;

		if (std::isinf(f_c))
			f_c = evaluateInterceptProximity(c, ourX, ourY, ourZ, uX, uY, uZ,
				wX, wY, wZ, ufoSpeedRadian, ourSpeedRadian);
		if (std::isinf(f_d))
			f_d = evaluateInterceptProximity(d, ourX, ourY, ourZ, uX, uY, uZ,
				wX, wY, wZ, ufoSpeedRadian, ourSpeedRadian);

		if (f_c < f_d)
		{
			b = d;
			f_d = f_c;
			f_c = INFINITY;
		}
		else
		{
			a = c;
			f_c = f_d;
			f_d = INFINITY;
		}
	}
	// Return minimum point and its approximate value.
	return std::pair<double, double>((b+a)/2.0, std::min(f_c, f_d));
}

/**
 * Calculate meeting point with the target.
 */
void MovingTarget::calculateMeetPoint()
{
	// Initialize
	if (_dest != 0)
	{
		_meetPointLat = _dest->getLatitude();
		_meetPointLon = _dest->getLongitude();
	}
	else
	{
		_meetPointLat = _lat;
		_meetPointLon = _lon;
	}

	if (!_dest || !Options::meetingPoint) return;

	MovingTarget *u = dynamic_cast<MovingTarget*>(_dest);
	if (!u || !u->getDestination()) return;

	// Get the speed ratio, and abort if the UFO is stationary.
	if (AreSame(u->getSpeedRadian(), 0.0)) return;

	double uLon = u->getLongitude(), uLat = u->getLatitude(),
		   destLon = u->getDestination()->getLongitude(), 
		   destLat = u->getDestination()->getLatitude();

	// Cartesian coordinates for the UFO and its destination
	double uX = cos(uLon) * cos(uLat);
	double uY = sin(uLon) * cos(uLat), uZ = sin(uLat);
	double destX = cos(destLon) * cos(destLat);
	double destY = sin(destLon) * cos(destLat), destZ = sin(destLat);

	// Find an intercept point where the distance between the interceptor (us)
	// and the UFO is minimal. It may still be positive if it's a very fast 
	// UFO and we're hoping for it to change course. We'll find two such 
	// minimal points: one that potentially involves crossing half the globe,
	// and one that doesn't.

	// First get a normal vector to u in the plane of the great circle.
	double uvX, uvY, uvZ, wX, wY, wZ, trX, trY, trZ;
	
	// w = (u x v) x u, then w is the normal vector
	cross(uX, uY, uZ, destX, destY, destZ, uvX, uvY, uvZ);
	cross(uvX, uvY, uvZ, uX, uY, uZ, wX, wY, wZ);

	double r = sqrt(wX*wX+wY*wY+wZ*wZ);
	wX /= r;
	wY /= r;
	wZ /= r;

	// Now the parametric for the great circle is R(t) = u cos t + w sin t
	// where t is on 0..2*pi. https://math.stackexchange.com/questions/383711/

	double ourX = cosl(_lat) * cosl(_lon);
	double ourY = cosl(_lat) * sinl(_lon);
	double ourZ = sinl(_lat);

	// this is the value of t for which we can reach half the world and
	// beyond wich we are at risk of going the wrong way around.
	double halfwayPoint = std::min(_speedRadian/u->getSpeedRadian(), 2*M_PI);
	double fullPoint = std::min(2*halfwayPoint, 2*M_PI);

	// Find the angle t where we're as close as possible to the UFO.
	std::pair<double, double> tNear = findMinimum(0, halfwayPoint, ourX, ourY, 
		ourZ, uX, uY, uZ, wX, wY, wZ, u->getSpeedRadian(), _speedRadian);

	std::pair<double, double> tFar = findMinimum(halfwayPoint, fullPoint, ourX, 
		ourY, ourZ, uX, uY,	uZ, wX, wY, wZ, u->getSpeedRadian(), _speedRadian);

	// Check whether near or far eventually gets us closest to the intercept 
	// point. If it's near, we're done; just return that point. If it's far,
	// we need to determine which way around the globe is the right one.

	if (tNear.second <= tFar.second) {
		getGreatCirclePoint(tNear.first, uX, uY, uZ, wX, wY, wZ, 
			trX, trY, trZ);
		_meetPointLon =  atan2(trY, trX); 
		_meetPointLat = asin(trZ);
		return;
	}

	// For Far, determine the bearing to the UFO after it has gone half the
	// distance to the intercept point, and pick the same hemisphere. If 
	// we're chasing a relatively close UFO, it will tell us to keep chasing;
	// if we're going after a near-antipodal UFO that will soon be traveling 
	// towards us, it'll tell us to meet it on the other side.
	// That means to go to the Far point but along the hemisphere where the
	// UFO is in half that time. This is an approximation to minimizing the
	// average distance between the UFO and the interceptor along the 
	// intercept path, which makes sense since the UFO might change its
	// trajectory at some later point and we then want to be close to it.

	double aX, aY, aZ;
	//long double bX, bY, bZ;

	getGreatCirclePoint(tFar.first/4.0, uX, uY, uZ, wX, wY, wZ, 
		aX, aY, aZ);
	_meetPointLon = atan2(aY, aX);
	_meetPointLat = asin(aZ);
	double halfwaydLon, halfwaydLat;
	calcdLonLat(halfwaydLon, halfwaydLat);

	getGreatCirclePoint(tFar.first, uX, uY, uZ, wX, wY, wZ, 
		trX, trY, trZ);
	_meetPointLon =  atan2(trY, trX); 
	_meetPointLat = asin(trZ);
	double dLon, dLat;
	calcdLonLat(dLon, dLat);

	/*double cX, cY, cZ;
	//cross(ourX, ourY, ourZ, destX, destY, destZ, cX, cY, cZ); // <-- STABLE!
	cross(ourX, ourY, ourZ, trX, trY, trZ, cX, cY, cZ); // <-- unstable
	//cross(trX, trY, trZ, ourX, ourY, ourZ, cX, cY, cZ);
	bX = cX;
	bY = cY;
	bZ = cZ;
	//cross(cX, cY, cZ, ourX, ourY, ourZ, bX, bY, bZ);
	r = sqrtl(bX*bX+bY*bY+bZ*bZ);
	bX /= r;
	bY /= r;
	bZ /= r;

	//double cX, cY, cZ;
	long double lenOffPlane = aX * bX + aY * bY + aZ * bZ;
	aX -= lenOffPlane * bX;
	aY -= lenOffPlane * bY;
	aZ -= lenOffPlane * bZ;
	
	_meetPointLon = atan2(aY, aX);
	_meetPointLat = asin(aZ);*/
	//_lon = atan2(aY, aX);
	//_lat = asin(aZ);

	
	// If the delta lat/lon according to the halfway is opposite the
	// delta lat/long according to the final intercept point, we should 
	// adjust the latter to go along the same hemisphere as the former.
	if (halfwaydLon * dLon < 0 || halfwaydLat * dLat < 0)
	{
		// Shorten our destination point to be halfway along the great circle
		// given by our position and the destination point.
		trX = (trX + uX) / 2.0;
		trY = (trY + uY) / 2.0;
		trZ = (trZ + uZ) / 2.0;
		_meetPointLon = atan2(trY, trX); 
		_meetPointLat = asin(trZ);
	}
}

/**
 * Returns the latitude of the meeting point.
 * @return Angle in rad.
 */
double MovingTarget::getMeetLatitude() const
{
	return _meetPointLat;
}

/**
 * Returns the longitude of the meeting point.
 * @return Angle in rad.
 */
double MovingTarget::getMeetLongitude() const
{
	return _meetPointLon;
}

}