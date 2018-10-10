/**
 * This file implements the GPS Library functions
 */
 
#include "gps_lib.h"

/*
 * This function converts Radians to Degrees
 */
double deg_to_rad(double deg) {
	return deg*PI/180;
}

/*
 * This function converts Degrees to Radians
 */
double rad_to_deg(double rad) {
	return rad*180/PI;
}

/*
 * Calculates distance between two GPS points
 */
double calcGpsDistance(double lat1, double long1, double lat2, double long2) {
	double xh, xr, yh, yr, zh, zr, dist;
	lat1    = deg_to_rad(lat1);
	long1   = deg_to_rad(long1);
	lat2    = deg_to_rad(lat2);
	long2   = deg_to_rad(long2);
	xh      = earthRad*cos(lat1)*cos(long1);
	yh      = earthRad*cos(lat1)*sin(long1);
	zh      = earthRad*sin(lat1);
	xr      = earthRad*cos(lat2)*cos(long2);
	yr      = earthRad*cos(lat2)*sin(long2);
	zr      = earthRad*sin(lat2);
	double arg = 1 - ((pow((xh-xr),2) + pow((yh-yr),2) + pow((zh-zr),2)))/(2*pow(earthRad,2));
	if(arg > 1) {
        arg = 1;
	}
	else if(arg < -1) {
        arg = -1;
	}
	dist    = earthRad*acos(arg);
	return dist;
}

/*
 * This function converts Relative distance to Latitude and Longitude
 */
GEO_LOCATION convertDistToLatLong(double refLat, double refLong, double azimuth, double dist) {
	GEO_LOCATION remotePos;
	double partLat, partLong;
	refLat	= deg_to_rad(refLat);
	refLong	= deg_to_rad(refLong);
	azimuth	= deg_to_rad(azimuth);
	double arg = cos(PI/2-refLat)*cos(dist/earthRad)+sin(PI/2-refLat)*sin(dist/earthRad)*cos(azimuth);
	if(arg > 1) {
        arg = 1;
	}
	else if(arg < -1) {
        arg = -1;
	}
	partLat = acos(arg);
	partLong = sin(sin(dist/earthRad)*sin(azimuth)/sin(partLat));
	remotePos.latitude = rad_to_deg(PI/2 - partLat);
	remotePos.longitude = rad_to_deg(refLong + partLong);
	return remotePos;
}

/*
 * This function converts Latitude and Longitude to a Relative X and Y
 */
REL_POSITION convertLatLongToXY(double refLat, double refLong, double refHeading, double latitude, double longitude) {
	double azimuth, den, num, arg;
	REL_POSITION relPos;

	/* Calculate distance between (refLat,refLong) and (latitude,longitude) */
	relPos.relDist = calcGpsDistance(refLat, refLong, latitude, longitude);

    refLat 		= deg_to_rad(refLat);
	latitude 	= deg_to_rad(latitude);
	refLong		= deg_to_rad(refLong);
	longitude	= deg_to_rad(longitude);
	refHeading	= deg_to_rad(refHeading);

	/* Calculate azimuth between (refLat,refLong) and (latitude,longitude) */
	den = sqrt(1 - (pow(((sin(refLat)*sin(latitude)) + (cos(refLat)*cos(latitude)*cos(longitude - refLong))),2)));
	if(den == 0) { //Avoid devide by zero condition
        den = 0.001;
	}
	num = sin(longitude - refLong)*cos(latitude);
	arg = num/den;
	if(arg > 1) {
        arg = 1;
	}
	else if(arg < -1) {
        arg = -1;
	}
	if((asin(arg) < 0) & ((latitude - refLat) >=0 )) {
		azimuth = asin(arg) + 2*PI;
	}
	else if((asin(arg) >= 0) & ((latitude - refLat) >=0 )) {
		azimuth = asin(arg);
	}
	else {
		azimuth = PI - asin(arg);
	}

	/* Calculate relative X and Y between (refLat,refLong) and (latitude,longitude) */
	if((azimuth - refHeading) < 0) {
		relPos.relX = relPos.relDist*cos(2*PI + azimuth - refHeading);
		relPos.relY = relPos.relDist*sin(2*PI + azimuth - refHeading);
	}
	else {
		relPos.relX = relPos.relDist*cos(azimuth - refHeading);
		relPos.relY = relPos.relDist*sin(azimuth - refHeading);
	}
	return relPos;
}

/*
 * This function converts relative X and Y to Latitude and Longitude
 */
GEO_LOCATION convertXYtoLatLong(double refLat, double refLong, double refHeading, double relX, double relY) {
	GEO_LOCATION remotePos;
	double flattening, eccentricity, radOfEarthMeridian, radOfEarthPrimeVertical, E, N;
	refLat = deg_to_rad(refLat);
	refLong = deg_to_rad(refLong);
	refHeading = deg_to_rad(refHeading);
	flattening = 0.003353;
	eccentricity = pow((flattening*(2 - flattening)), 0.5);
	radOfEarthMeridian = earthRad*(1 - pow(eccentricity, 2))/pow((1 - pow(eccentricity, 2)*pow(sin(refLat),2)),(3/2));
	radOfEarthPrimeVertical = earthRad/(pow(1 - pow(eccentricity, 2)*pow(sin(refLat),2),(1/2)));
	E = cos(refHeading)*relY + sin(refHeading)*relX;
	N = cos(refHeading)*relX - sin(refHeading)*relY;
	remotePos.latitude = rad_to_deg((1/radOfEarthMeridian)*N + refLat);
	remotePos.longitude = rad_to_deg((1/(radOfEarthPrimeVertical*cos(refLat)))*E + refLong);
	
	/* Alternate way of computing XY to LatLong */
	/*
	double azimuth, dist;
	if(relX == 0) {
		relX = 0.001;
	}
	azimuth = fmod((refHeading + rad_to_deg(atan2(relY, relX))), 360);
	dist = sqrt(pow(relX, 2) + pow(relY, 2));
	remotePos = convertDistToLatLong(refLat, refLong, azimuth, dist);
	*/
	
	return remotePos;
}

/*
 * This function computes the azimuth between two GPS points
 */
double calcAzimuth(double refLat, double refLong, double refHeading, double latitude, double longitude) {
	double azimuth;
	REL_POSITION relPos = convertLatLongToXY(refLat, refLong, refHeading, latitude, longitude);
	if(relPos.relX == 0) {
		relPos.relX = 0.001;
	}
	azimuth = rangeM180ToP180(rad_to_deg(atan2(relPos.relY, relPos.relX)));
	return azimuth;
}

/*
 * This function calculates the central angle between pos1 (lat1, long1), pos2 (latCenter, longCenter), and pos3 (lat2, long2)
 */
double calcCentralAngle(double lat1, double long1, double latCenter, double longCenter, double lat2, double long2) {
    double veh1_veh1PPC_Dist, veh1PPC_veh2_Dist, veh1_veh2_Dist, centralAngle;
    veh1_veh1PPC_Dist   = calcGpsDistance(lat1, long1, latCenter, longCenter);
    veh1PPC_veh2_Dist   = calcGpsDistance(latCenter, longCenter, lat2, long2);
    veh1_veh2_Dist      = calcGpsDistance(lat1, long1, lat2, long2);
	double arg = (pow(veh1_veh1PPC_Dist,2) + pow(veh1PPC_veh2_Dist,2) - pow(veh1_veh2_Dist,2))/(2*veh1_veh1PPC_Dist*veh1PPC_veh2_Dist);
	if(arg > 1) {
        arg = 1;
	}
	else if(arg < -1) {
        arg = -1;
	}
    centralAngle = rangeM180ToP180(rad_to_deg(acos(arg)));
    return centralAngle;
}

/*
 * This function maps angles (range -x to +x degrees) to -180 to 180 degrees
 */
double rangeM180ToP180(double degAngle) {
    double maxAngle = 360;
    degAngle = degAngle/maxAngle;
    if(degAngle > 0) {
        degAngle = (degAngle - floor(degAngle))*360;
    }
    else {
        degAngle = (degAngle - ceil(degAngle))*360;
    }
    if(degAngle > 180) {
        return (degAngle - maxAngle);
    }
    else if(degAngle <= -180) {
        return (degAngle + maxAngle);
    }
    else {
        return degAngle;
    }
}

/* 
 * This function maps angles (range -x to +x degrees) to 0 to 360 degrees
 */
double range0To360(double degAngle) {
	double maxAngle = 360;
	degAngle = degAngle / maxAngle;
	if (degAngle > 0) {
		degAngle = (degAngle - floor(degAngle)) * 360;
	}
	else {
		degAngle = (degAngle - ceil(degAngle)) * 360;
	}

	if (degAngle < 0) {
		degAngle += maxAngle;
	}

	return degAngle;
}