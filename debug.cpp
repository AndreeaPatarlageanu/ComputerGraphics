#include <vector>
#include <limits>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include<random>

#define PI 3.14159265359

std::default_random_engine generator;
std::uniform_real_distribution<double> uniform(0.0,1.0);

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
	Sphere( const Vector& C, double R, const Vector& color, bool mirror, bool transparent ) : C(C), R(R), color(color), mirror(mirror), transparent(transparent) {};
	Vector C, color;
    double R;
	bool mirror, transparent;

	bool intersect( const Ray& r, Vector &P, Vector &N ) const{
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
	// Vector C;
	// double R;
};

class Scene{
	//int Shadow_effect(const Scene& scene, const Vector& P, Vector light_pos);
	public:
	std::vector<Sphere> objects;
	void add(const Sphere &s) { objects.push_back(s); }
	bool intersect( const Ray& r, Vector &P, Vector &N, int& object_id, Vector& color ) {
		bool result = false;
		//for this part I received a bit of help from a friend
		double t_min = std::numeric_limits<double>::max();

		for ( int i = 0; i < objects.size(); i++ ){
			Vector localP, localN;
			if ( objects[i]. intersect(r, localP, localN )) {
				double t = ( localP - r.origin ).norm();
				//result = true;
				if( t < t_min ){
					t_min = t;
					P = localP;
					N = localN;
					object_id = i;
					result = true;
					color = objects[i].color;
				}
			}
		}
		return result;
	}

	Vector getColour(const Ray& ray, int ray_depth, Vector light_pos, double I) {
		Vector P, N, colour;
		int id;
		int object_id;

		if ( ray_depth <= 0) return Vector(0, 0, 0);
	
		if(intersect(ray, P, N, object_id, colour)) {
			Sphere sph = objects[object_id];
			//in the case it is transparent we need to have a separate case
			if (sph.transparent == true) {
				double n1 = 1.0, n2 = 1.5;
				double n = n1 / n2;
				Vector normal = N;
				if( dot(ray.u, normal ) > 0 ) {
					n = 1.0 / n;
					normal = (-1) * normal;
				} 

				Vector reflectedDir = ray.u - 2 * dot( ray.u, normal ) * normal;
				Ray reflectedRay( P, reflectedDir );
				if( 1 - n * n * ( 1 - dot(ray.u, normal) * dot( ray.u, normal) ) < 0 ){
					return getColour( reflectedRay, ray_depth - 1, light_pos, I);
				}

				double k0 = ( n1 - n2 ) / ( n1 + n2);
				k0 = k0 * k0;
				double R = k0 + (1 - k0) * std::pow( 1 - abs(dot(ray.u, normal)), 5);
				double random = uniform(generator);
				if(random < R )
					return getColour(reflectedRay, ray_depth - 1, light_pos, I);
				
					Vector refractedTangential = n * (ray.u - dot(normal, ray.u) * normal);
					Vector refractedNormal = - std::sqrt(1 - pow(n, 2) * (1 - dot(ray.u, normal) * dot(ray.u, normal))) * normal;
					Vector refractedDir = refractedTangential + refractedNormal;
					Ray refracted_ray(P, refractedDir);
					return getColour(refracted_ray, ray_depth - 1, light_pos, I);

			}
			else if ( sph.mirror == false ) {

				if( Shadow_effect( P, light_pos ) == 1 ) return Vector(0., 0., 0. );
				else{
					double distance = ( light_pos - P ).norm2();
					//long formula from the lecture notes:
					double attenuation = I / ( 4 * PI * distance );
					Vector material = colour / PI;
					double product = dot( N, (light_pos - P ) / (light_pos - P).norm() );
					double solid_angle = std::max( 0.0, product );

					return attenuation * material* solid_angle;
				}
			}
			
			else {
				//reflection direction formula
				Vector formula_transparent = ray.u - 2 * N * dot( ray.u, N );
				formula_transparent.normalize();

				Vector newP =  P + 1e-4 * N;

				//recursive call as the TA said
				Ray new_ray = Ray( newP, formula_transparent );
				ray_depth--;
				return getColour( new_ray, ray_depth, light_pos, I );
			}
		}
	
		return Vector(0, 0, 0);
	}

	int Shadow_effect( const Vector& P, Vector light_pos ){
		//formulas from the lecture notes
		double light_distance = ( light_pos - P ).norm();
		Vector direction = light_pos - P; //.normalize();
		direction.normalize();
		Vector precision = P + 1e-4 * direction;
		Ray shadow = Ray( precision, direction );
		int object_id;
		Vector localN, localP, colour;
		
		if ( intersect( shadow, localP, localN, object_id, colour ) ){
			double prime = ( localP - P ).norm();
			if ( light_distance > prime ) //we need shadow
				return 1;
		}
		return -1;  //no shadow
	}
};


int main() {
	int W = 512;
	int H = 512;
	Scene scene;

	//ball in the middle
	//just for the sake of example, we take one ball to be the mirror and one to be solid
    //scene.add( Sphere( Vector( -12, -10, 6 ), 10, Vector( 0.102, 0.255, 0.541 ), true, false ) );
	//scene.add( Sphere( Vector( 20, -10, 6 ), 10, Vector( 0.941, 0.627, 0.949 ), true, false ) ); //added 2 balls to see the difference in shadow and lter for mirror
	scene.add( Sphere( Vector( 0, -10, 6 ), 10, Vector( 0.89, 0.902, 0.278  ), false, false ) );

    //4 spheres each of them being a wall
	scene.add( Sphere( Vector( -100000, 0, 0 ), 99965, Vector( 0.779, 0.378, 0.752 ), false, false)); //left
	scene.add( Sphere( Vector( 0, 100000, 0 ), 99965, Vector( 0.125, 0.332, 0.777 ), false, false)); //top
	scene.add( Sphere( Vector( 100000, 0, 0 ), 99965, Vector( 0.223, 0.677, 0.020 ), false, false)); //right
	scene.add( Sphere( Vector(0, -100010, 0), 99990, Vector( 0.300, 0.400, 0.700 ), false, false)); //down
	scene.add( Sphere( Vector( 0, 0, -100000 ), 99970, Vector( 0.273, 0.494, 0.453 ), false, false)); //back
	scene.add( Sphere( Vector( 0, 0, 100050 ), 99970, Vector( 0.780, 0.094, 0.298 ), false, false)); //behind

	Vector camera_origin(0, 0, 72); 

	double fov = 60 * PI / 180.;
	//Sphere S(Vector (0, 0, 0), 10, Vector(1, 1, 1));
	Vector albedo(1, 1, 1);
	double I = 170000;
	//double I = 1.0;
	Vector light_pos( -10, 20, 40 );

	std::vector<unsigned char> image(W * H * 3, 0);
	auto start_time = std::chrono::high_resolution_clock::now();  //this was written using AI

	#pragma omp parallel for schedule(static)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			double d = -W / (2. * tan(fov / 2 ) );
			Vector r_dir(j - W / 2 + 0.5, H / 2 - i + 0.5 , d);
			r_dir.normalize();
			Ray r( camera_origin, r_dir );
			Vector P, N;
			Vector albedo;
			int object_id;

			Vector color = scene.getColour( r, 5, light_pos, I );
			//Vector color_sh = I / (4 * PI * d2) * albedo / PI * std::max(0.0, dot(N, lightDir));

			// 	Vector color = scene.getColour( )
			double color1 = std::pow( color[0], 1 / 2.2 ) * 255;
			double color2 = std::pow( color[1], 1 / 2.2 ) * 255;
			double color3 = std::pow( color[2], 1 / 2.2 ) * 255;


			image[(i * W + j) * 3 + 0] = std::min(255., color1 );
			image[(i * W + j) * 3 + 1] = std::min(255., color2 );
			image[(i * W + j) * 3 + 2] = std::min(255., color3 );
			//}
		}
	}
	auto end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Rendering time (in parallel): " << elapsed.count() << " seconds" << std::endl;  //these last 3 lines were also written using AI

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}