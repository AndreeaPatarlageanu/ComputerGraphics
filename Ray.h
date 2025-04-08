#ifndef RAY_H
#define RAY_H
#include "Vector.h"

class Ray {
	public:
		Ray( const Vector& O, const Vector& u) : origin(O), u(u) {};
		Vector origin;
		Vector u;
	};

#endif