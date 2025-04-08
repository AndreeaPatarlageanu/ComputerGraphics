
#include <cmath> 
#include "Sphere.h"

double sqr( double x) {
	return x * x;
}

Sphere::Sphere( const Vector& C, double R, const Vector& color, bool mirror, bool transparent ) : C(C), R(R), color(color), mirror(mirror), transparent(transparent) {}

bool Sphere::intersect( const Ray& r, Vector &P, Vector &N ) const{
    double delta = sqr(dot(r.u, r.origin - C )) - ( r.origin - C).norm2() + sqr(R);
    if ( delta < 0 ) return false;
    double x = dot(r.u, C - r.origin);
    double t1 = x - sqrt(delta);
    double t2 = x + sqrt(delta);
    if ( t2 < 0 ) return false;
    double T;
    if ( t1 > 0 )
        T = t1;
    else
        T = t2;
    
    P = r.origin + T * r.u;
    N = ( P - C ) / ( P - C ).norm(); //.normalize();
    //N.normalize();
    // if (dot(r.u, N) > 0) {
    // 	N = -1 * N;
    // }
    return true;
}
	