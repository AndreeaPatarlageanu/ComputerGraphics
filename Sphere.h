#ifndef SPHERE_H

#include "Vector.h"
#include "Ray.h"

class Sphere {
    public:
        Sphere( const Vector& C, double R, const Vector& color, bool mirror, bool transparent );
        Vector C, color;
        double R;
        bool mirror, transparent;
    
        bool intersect( const Ray& r, Vector &P, Vector &N ) const;

};

#endif