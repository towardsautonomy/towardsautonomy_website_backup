/**
 * @file gps_lib.h
 * @author Shubham Shrivastava
 * @date 03 Oct 2017
 * @brief This file contains the GPS Library API definitions
 *
 */

#ifndef __GPS_LIB_H__
#define __GPS_LIB_H__

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**
 * @brief Constant Definitions
 */
#define earthRad 6378137		/**< Radius of Earth in Meters */
#define PI 3.1415926535898		/**< PI in Radians */

/**
 * @brief Relative Position Structure
 */
typedef struct REL_POSITION {
	double relX;				/**< Relative X in meters; X -> +ve forward ahead */
	double relY;				/**< Relative Y in meters; Y -> +ve towards the right */
	double relDist;				/**< Relative absolute distance in meters */
} REL_POSITION;

/**
 * @brief 2D GeoLocation Structure
 */
typedef struct GEO_LOCATION {
	double latitude;			/**< Latitude in degrees */
	double longitude;			/**< Longitude in degrees */
} GEO_LOCATION;

/** 
 * @brief This function converts degrees to radians
 * 
 * @param deg Degrees to be converted into Radians
 *
 * @return Converted value in Radians
 */
double deg_to_rad(double deg);

/** 
 * @brief This function converts radians to degrees
 * 
 * @param rad Radians to be converted into Degrees
 *
 * @return Converted value in Degrees
 */
double rad_to_deg(double rad);

/** 
 * @brief This function computes the absolute distance between two position in meters
 * 
 * @param lat1,long1 Latitude and Longitude of the First point in degrees 
 * @param lat2,long2 Latitude and Longitude of the Second point in degrees 
 *
 * @return Distance between two points in meters
 */
double calcGpsDistance(double lat1, double long1, double lat2, double long2);

/** 
 * @brief This function computes the Latitude and Longitude of a point at certain distance and angle from the Reference point.
 * 
 * The Reference point is an absolute Latitude and Longitude in degrees. The relative point can be specified as a certain absolute distence in meters and at a certain angle (0 to 360 degrees from North; North being 0 degrees)
 *
 * @param refLat,refLong Latitude and Longitude of the Reference point in degrees
 * @param azimuth,dist Azimuth and Distance of the point for which Latitude and Longitude needs to be computed 
 *
 * @return Latitude and Longitude contained in a GEO_LOCATION structure.
 */
GEO_LOCATION convertDistToLatLong(double refLat, double refLong, double azimuth, double dist);

/** 
 * @brief This function computes the relative X and Y in meters given the absolute position, heading of first point and the absolute position of second point
 * 
 * X is in meters with forward being positive; Y is in meters with right side being positive. 
 *
 * @param refLat,refLong,refHeading Latitude, Longitude, and Heading of the Reference point in degrees. Heading between 0 to 360 degrees clockwise; True North being 0 degrees.
 * @param latitude,longitude Latitude and Longitude of the point for which relative X and Y needs to be computed.
 *
 * @return Relative X and Y contained in a REL_POSITION structure.
 */
REL_POSITION convertLatLongToXY(double refLat, double refLong, double refHeading, double latitude, double longitude);

/** 
 * @brief This function computes the Latitude and Longitude of a point given the absolute position, heading of first point and the relative X and Y of the second point
 * 
 * X is in meters with forward being positive; Y is in meters with right side being positive. 
 *
 * @param refLat,refLong,refHeading Latitude, Longitude, and Heading of the Reference point in degrees. Heading between 0 to 360 degrees clockwise; True North being 0 degrees.
 * @param relX,relY Relative X and Y in meters of the point for which Latitude and Longitude needs to be computed.
 *
 * @return Latitude and Longitude contained in a GEO_LOCATION structure.
 */
GEO_LOCATION convertXYtoLatLong(double refLat, double refLong, double refHeading, double relX, double relY);

/** 
 * @brief This function computes the azimuth between two points given the absolute position, heading of first point and the absolute position of second point
 *
 * @param refLat,refLong,refHeading Latitude, Longitude, and Heading of the Reference point in degrees. Heading between 0 to 360 degrees clockwise; True North being 0 degrees.
 * @param latitude,longitude Latitude and Longitude of the point to which azimuth needs to be computed.
 *
 * @return Azimuth in degrees; -180 to 180 degrees
 */
double calcAzimuth(double refLat, double refLong, double refHeading, double latitude, double longitude);

/** 
 * @brief This function computes the central angle between three absolute positions
 *
 * @param lat1,long1 Latitude and Longitude of the First point in degrees.
 * @param latCenter,longCenter Latitude and Longitude of the Second point (Point corresponding to the center of three points) in degrees.
 * @param lat2,long2 Latitude and Longitude of the Third point in degrees.
 *
 * @return Central Angle in degrees; -180 to 180 degrees
 */
double calcCentralAngle(double lat1, double long1, double latCenter, double longCenter, double lat2, double long2);

/** 
 * @brief This function converts any angle into a range of -180 to 180 degrees.
 *
 * Example: If the input passed was 270; it will return -90 degrees.
 *
 * @param degAngle Angle in degrees
 *
 * @return Angle in the range -180 to 180 degrees
 */
double rangeM180ToP180(double degAngle);

/** 
 * @brief This function converts any angle into a range of 0 to 360 degrees.
 *
 * Example: If the input passed was -90; it will return 270 degrees.
 *
 * @param degAngle Angle in degrees
 *
 * @return Angle in the range 0 to 360 degrees
 */
double range0To360(double degAngle);

#ifdef __cplusplus
	}
#endif

#endif  /* ndef __GPS_LIB_H__ */
