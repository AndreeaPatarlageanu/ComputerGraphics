
#include <cmath> 
#include "Sphere.h"
#include "Auxiliary.h"

Sphere::Sphere(const Vector& C, double R, const Vector& color, bool mirror, bool transparent, const Vector& velocity) 
    : Geometry(color, mirror, transparent), C(C), R(R), velocity(velocity) {}

bool Sphere::intersect( const Ray& r, Vector &P, Vector &N ) const{
    Vector center_t = C + velocity * r.t; //added this for motion blur
    double delta = sqr(dot(r.u, r.origin - center_t )) - ( r.origin - center_t).norm2() + sqr(R);
    if ( delta < 0 ) return false;
    double x = dot(r.u, center_t - r.origin);
    double t1 = x - sqrt(delta);
    double t2 = x + sqrt(delta);
    if ( t2 < 0 ) return false;
    double T;
    if ( t1 > 0 )
        T = t1;
    else
        T = t2;
    
    P = r.origin + T * r.u;
    N = ( P - center_t ) / ( P - center_t ).norm(); //.normalize();
    //N.normalize();
    // if (dot(r.u, N) > 0) {
    // 	N = -1 * N;
    // }
    return true;
}
	