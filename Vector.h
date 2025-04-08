#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>

class Vector {
    public:
        explicit Vector(double x = 0, double y = 0, double z = 0) {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }
        double norm2() const {
            return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
        }
        double norm() const {
            return sqrt(norm2());
        }
        void normalize() {
            double n = norm();
            data[0] /= n;
            data[1] /= n;
            data[2] /= n;
        }
        double operator[](int i) const { return data[i]; };
        double operator[](int i) { return data[i]; };
        double data[3];
    
        int maximum() {
            if ( data[0] >= data[1] && data[0] >= data[2] ) {
                return 0;
            }
            if ( data[1] >= data[0] && data[1] >= data[2] ) {
                return 1;
            }
            return 2;
        }
    };

    Vector operator+(const Vector& a, const Vector& b);

    Vector operator-(const Vector& a, const Vector& b);

    Vector operator*(const double a, const Vector& b);

    Vector operator*(const Vector& a, const double b);

    Vector operator/(const Vector& a, const double b);

    double dot(const Vector& a, const Vector& b);

    Vector cross(const Vector& a, const Vector& b);

    Vector operator*( const Vector& a, const Vector& b );

#endif