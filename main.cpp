#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

#define PI 3.14159265359

double sqr( double x) {
	return x * x;
}

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

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
	return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
	return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator/(const Vector& a, const double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

class Ray {
	public:
		Ray( const Vector& O, const Vector& u) : origin(O), u(u) {};
		Vector origin;
		Vector u;
	};

class Sphere {
public:
	Sphere( const Vector& C, double R) : C(C), R(R) {};
	bool intersect( const Ray& r, Vector &P, Vector &N ){
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
		N = ( P - C ); //.normalize();
		N.normalize();
		return true;
	}
	Vector C;
	double R;
};

// class Scene{   ASTA NU E TERMINATA
// 	public:
// 	Scene(){};
// 	void add(const Sphere &s) { objects.push_back(s) };
// 	bool intersect( const Ray& r, Vector &P, Vector &N, double& t, double& object_id ){
// 		vool result = false;
// 		for ( int i = 0; i< objects.size(); i++ ){
// 			if ( objects[i]. intersect(r, localP, localN, localt)) {
			// 	result = true;
			// 	if( localt < t){
			// 		t = localt;
			// 		P = localP;
			// 		N = localN;
			//		object_id = ....
			// 	}
			// }
// 		}
// 	}
// 	std::vector<Sphere> objects;
// };

int main() {
	int W = 512;
	int H = 512;
	Vector camera_origin(0, 0, 55);
	double fov = 60 * PI / 180.;
	Sphere S(Vector (0, 0, 0), 10);
	Vector albedo(1, 1, 1);
	double I = 1E7;
	//double I = 1.0;
	Vector light_pos( -10, 20, 40 );

	std::vector<unsigned char> image(W * H * 3, 0);
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			double d = -W / (2. * tan(fov / 2 ) );
			Vector r_dir(j - W / 2 + 0.5, H / 2 - i + 0.5 , d);
			r_dir.normalize();
			Ray r( camera_origin, r_dir );
			Vector P, N;
			if ( S.intersect(r, P, N) ){				
				Vector lightDir = light_pos - P;
				double d2 = lightDir.norm2();
				//std::cout<<"Haha"; 
				lightDir.normalize(); 
				//have to take care of the shadowing as well
				//epsilon 1e-4, P + epsilon * N
				Vector launch_point = P + 1e-4 * N;
				//from the lecture notes " launching a ray from point P towards the light source S"
				Ray shadow( launch_point, lightDir);

				Vector randomvar1, randomvar2;
				bool visibility_term = S.intersect( shadow, randomvar1, randomvar2 );
				if ( visibility_term == true )  //we do not apply the color
					continue;

				Vector color = I / (4 * PI * d2) * albedo / PI * std::max(0.0, dot(N, lightDir));

				image[(i * W + j) * 3 + 0] = std::min(255., color[0]);
				image[(i * W + j) * 3 + 1] = std::min(255., color[1]);
				image[(i * W + j) * 3 + 2] = std::min(255., color[2]);
			}
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}

/*
scene.add(Sphere(Vector(0, -1000, 0), 900, Vector()))
*/