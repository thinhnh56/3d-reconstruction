  #include "../utils.h"
  
	const double PI = 3.14159265;

	double calculate_angleA(double a, double b, double c){
		double temp = (b*b + c*c - a*a)/(2*b*c);
		return acos(temp)*180.0/PI;
  
	}
	
	double calculate_angleA(PointRGB A, PointRGB B, PointRGB C){
		double a = sqrt((B.x - C.x)*(B.x - C.x) + (B.y - C.y)*(B.y - C.y) +(B.z - C.z)*(B.z - C.z));
		double b = sqrt((A.x - C.x)*(A.x - C.x) + (A.y - C.y)*(A.y - C.y) +(A.z - C.z)*(A.z - C.z));
		double c = sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y) +(A.z - B.z)*(A.z - B.z));
		
		return calculate_angleA(a, b, c);
  
	}