// Geometry.h
#pragma once
#include "Vector.h"
#include "Ray.h"

class Geometry {
public:
    Vector color;
    bool mirror, transparent;

    Geometry( const Vector& color, bool mirror, bool transparent );

    virtual bool intersect ( const Ray& r, Vector &P, Vector &N ) const = 0; 
};

//add an intersect function
