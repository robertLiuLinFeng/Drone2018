#include "wgs_conversions/wgs_conversions.h"




void main()
{
	WgsConversions wgs_test;
	double enu[3];
	double xyz[3];
	const double lla[3] = { 37.2654 + 0.01, 137.1254 + 0.01, 375 + 10};
	const double ref_lla[3] = { 37.2654, 137.1254, 375 };
	bool flag = wgs_test.lla2enu(enu, lla, ref_lla);
	flag = wgs_test.lla2xyz( xyz, lla);
	if (true == flag)
	{
		std::cout << "\n\tSuccessfully converted from lla back to ecef" << std::endl;
		std::cout << "\t\tX: " << xyz[0] << "\tY: " << xyz[1] << "\tZ: " << xyz[2] << std::endl;
		
		std::cout << "\n\tSuccessfully converted from lla to enu" << std::endl;
		std::cout << "\t\tE: " << enu[0] << "\tN: " << enu[1] << "\tU: " << enu[2] << std::endl;
	}
	
}
