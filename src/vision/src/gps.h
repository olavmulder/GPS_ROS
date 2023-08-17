#ifndef GPS
#define GPS

#include <cmath>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265359


double gpsLatToMeter(double v1, double v2)
{
        double r = 6371000; //straal aarde in meter
        //#print(v1, v2)
        return (2 * PI * r * ((v2 - v1) / 360));
}
double gpsLongToMeter(double angle, double v1, double v2)
{
        double r = 6371000; //#straal earth in meters
        //#print(angle)
        //#print(math.cos(math.radians(angle)))
	double rad_angle = angle * (180/PI);
        double a = (cos(rad_angle)) * r; //#Hight rechthoekige triangle, needed to calculate radius
        //#print(a)
        //#a = math.sqrt((r * r) - (x * x)) #radius circle at given coordinates
        double meters = 2 * PI * a * ((v2 - v1) / 360); 
        return meters;
}

double gpsdif(double lon1, double lat1, double lon2, double lat2)
{
	double r = 6371000;
	double lat1_rad = lat1 * (PI/180);
	double lat2_rad = lat2 * (PI/180);
	double lat_dif = (lat2-lat1) * (PI/180);
	double lon_dif = (lon2-lon1) * (PI/180);

	double a = sin(lat_dif/2) * sin(lat_dif/2) + cos(lat1_rad) * cos(lat2_rad) * sin(lon_dif/2) * sin(lon_dif/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double meters = r*c;
	return meters;	
}
#endif