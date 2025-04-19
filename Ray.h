#ifndef RAY_H
#define RAY_H

#include "Vector.h"

class Ray {
public:
    Ray(const Vector& O, const Vector& u, double t = 0) : origin(O), u(u), t(t) {}  //added t for motion blur
    Vector origin;
    Vector u;
    double t;
};

#endif // RAY_H
