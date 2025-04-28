#ifndef SPHERE_H

#include "Vector.h"
#include "Ray.h"
#include "Geometry.h"

class Sphere : public Geometry{
    public:
        Sphere( const Vector& C, double R, const Vector& color, bool mirror, bool transparent, const Vector& velocity = Vector( ) );
        Vector C, velocity;
        double R;
        //bool mirror, transparent;
    
        bool intersect( const Ray& r, Vector &P, Vector &N ) const override;

};

#endif