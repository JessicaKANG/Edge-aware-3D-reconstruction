#ifndef OPENEDGE_SLOPE_MANAGER_HPP
#define OPENEDGE_SLOPE_MANAGER_HPP

#include <math.h>
#define PI 3.14159265

/* header for distance function */
double distance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


/* header for slope function */
double slope(double x1, double y1, double x2, double y2)
{
    return (y2 - y1) / (x2 - x1);
}


/* header for angle function */
double angle(double x1, double y1, double x2, double y2)
{
    double a;
//     if (x1 == x2){
//        a = PI / 2;
//     }
//     else{
//       if (y1 > y2){
// 	a = atan((y1 - y2) / (x1 - x2));
//       }
//       else{
// 	a = atan((y2 - y1) / (x2 - x1));
//       }
//     }
    a = atan2(y2-y1, x2-x1);
    if(a<0) a = a+2*PI; // range a to 0~2PI
 
    return a;
}

#endif //OPENEDGE_SLOPE_MANAGER_HPP