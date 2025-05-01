#include <vector>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

#define M_PI 3.14159265359

#include <random>
#include "Vector.h"
#include "Ray.h"
#include "Sphere.h"
#include "Auxiliary.h"
#include "mesh.h"

static std::default_random_engine engine(10);  //random seed = 10
static std::uniform_real_distribution<double> uniform(0, 1);

const double EPSILON = 1e-4;

void boxMuller( double stdev, double &x, double &y ){
	double r1 = uniform( engine );
	double r2 = uniform( engine );
	x = sqrt( -2 * log(r1)) * cos( 2 * M_PI * r2 ) * stdev;
	y = sqrt( -2 * log(r1)) * sin( 2 * M_PI * r2 ) * stdev;
}

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

Vector operator*( const Vector& a, const Vector& b ) {
    return Vector( a[0] * b[0], a[1] * b[1], a[2] * b[2] );
}

class Scene{
	//int Shadow_effect(const Scene& scene, const Vector& P, Vector light_pos);
	public:
	//std::vector<Sphere> objects;
	//void add(const Sphere &s) { objects.push_back(s); }
	std::vector<Geometry*> objects;
    void add(Geometry* obj) { objects.push_back(obj); }

	bool intersect( const Ray& r, Vector &P, Vector &N, int& object_id, Vector& color ) {
		bool result = false;
		//for this part I received a bit of help from a friend
		double t_min = std::numeric_limits<double>::max();

		for ( int i = 0; i < objects.size(); i++ ){
			Vector localP, localN;
			if ( objects[i]->intersect(r, localP, localN )) {
				double t = ( localP - r.origin ).norm();
				//result = true;
				if( t < t_min ){
					t_min = t;
					P = localP;
					N = localN;
					object_id = i;
					result = true;
					color = objects[i]->color;
				}
			}
		}
		return result;
	}

	Vector cosineFunction( const Vector& N ) {
		//Vector N;
		double r1 = uniform( engine );
		double r2 = uniform( engine );

		double x = cos( 2 * M_PI * r1 ) * sqrt( 1 - r2 );
		double y = sin( 2 * M_PI * r1 ) * sqrt( 1 - r2 );
		double z = sqrt( r2 );

		//done during lecture:
		Vector T1, T2;
		if ( abs( N[0] ) <= abs( N[1] ) && abs( N[0] ) <= abs( N[2] ) )
			T1 = Vector (0, -N[2], N[1] );
		else if ( abs( N[1] ) <= abs( N[0] ) && abs( N[1] ) <= abs( N[2] ) ) 
			T1 = Vector( -N[2], 0, N[0]);
		else
			T1 = Vector ( -N[1], N[0], 0 );
		
		T1.normalize();
		//T2 = dot( N, T1 );
		T2 = cross( N, T1 );
		return x * T1 + y * T2 + z * N;
	}

	Vector getColour(const Ray& ray, int ray_depth, Vector light_pos, double I) {
		Vector P, N, colour;
		int id;
		int object_id;

		if ( ray_depth <= 0) return Vector(0, 0, 0);
	
		if(intersect(ray, P, N, object_id, colour)) {
			//Sphere sph = objects[object_id];
			Geometry* obj = objects[object_id];
			//in the case it is transparent we need to have a separate case
			if (obj->transparent == true) {
				Vector rayDirection = ray.u;
				rayDirection.normalize();

				double n1 = 1.0, n2 = 1.5, n; //according to the lectur enotes
				bool inside;
				if ( dot(rayDirection, N ) < 0 )
					inside = true;
				else
					inside = false;

				Vector normal;
				if ( inside == true ){
					normal = N;
					n = n1 / n2;
				}
				else{
					normal = (-1) * N;
					n = n2 / n1;
				}
				
				Vector reflectionDirection = rayDirection - 2 * dot( rayDirection, normal ) * normal;
				reflectionDirection.normalize();

				double cos, r0, fresnel;
				cos = (-1) * dot( rayDirection, normal );
				r0 = pow( ( n1 - n2 ) / ( n1 + n2 ), 2 );
				fresnel =  r0 + ( 1 - r0 ) * pow( 1 - cos, 5 );

				double decision;
				decision = 1 - n * n * ( 1 - cos * cos );

				if (decision < 0 ){
					Ray reflection( P + EPSILON * reflectionDirection, reflectionDirection );
					return getColour( reflection, ray_depth - 1, light_pos, I );
				}
				else{
					double random;
					random = uniform( engine );
					
					if( random < fresnel ) {  //then we have reflection
						Ray reflection( P + EPSILON * reflectionDirection, reflectionDirection );
						return getColour( reflection, ray_depth - 1, light_pos, I );
					} 
					else {  //refraction part
						Vector refractedTg, refractedN, refractionDirection;

						refractedTg = n * ( rayDirection - dot( rayDirection, normal ) * normal );
						refractedN = (-1) * sqrt( 1 - refractedTg.norm2() ) * normal;
						refractionDirection = refractedTg + refractedN;
						refractionDirection.normalize();
						
						Ray refraction( P + EPSILON * refractionDirection, refractionDirection );
						return getColour( refraction, ray_depth - 1, light_pos, I );
					}
				}
			}
			else if ( obj->mirror == false ) {  //we need to add both direct and indirect lighting
				
				Vector direct(0, 0, 0 ), indirect(0, 0, 0);
				if ( Shadow_effect( P, light_pos ) != 1 ) {
					double distance = ( light_pos - P ).norm2();
					//long formula from the lecture notes:
					double attenuation = I / ( 4 * M_PI * distance );
					Vector material = colour / M_PI;
					double product = dot( N, (light_pos - P ) / (light_pos - P).norm() );
					double solid_angle = std::max( 0.0, product );

					direct = attenuation * material* solid_angle;
				}

				if( ray_depth > 1 ) {
					Vector directionRandom = cosineFunction( N );  //random direction
					Ray ray_indirect( P + 1e-4 * N, directionRandom );
					Vector colorIndirect = getColour( ray_indirect, ray_depth - 1, light_pos, I );

					indirect = colour * colorIndirect;
				}

				return direct + indirect;
			}
			
			else {
				//reflection direction formula
				N.normalize();
				Vector debug = ray.u;
				debug.normalize();
				Vector formula_transparent = debug - 2 * N * dot( debug, N );
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
    // scene.add( new Sphere( Vector( -20, -10, 6 ), 10, Vector( 0.102, 0.255, 0.541 ), true, false, Vector( 0, 0, 0 ) ) );
	// scene.add( new Sphere( Vector( 20, -10, 6 ), 10, Vector( 0.941, 0.627, 0.949 ), true, false, Vector( 0, 0, 0 ) ) ); //added 2 balls to see the difference in shadow and lter for mirror
	// scene.add( new Sphere( Vector( 0, -10, 6 ), 10, Vector( 0.89, 0.902, 0.278  ), false, false, Vector( 0, 0, 0 ) ) );
	//scene.add( Sphere( Vector( 0, -10, 6 ), 10, Vector( 0.89, 0.902, 0.278 ), false, false, Vector( 0, 15, 0 ) ) );


    //4 spheres each of them being a wall
	scene.add( new Sphere( Vector( -100000, 0, 0 ), 99965, Vector( 0.779, 0.378, 0.752 ), false, false)); //left
	scene.add( new Sphere( Vector( 0, 100000, 0 ), 99965, Vector( 0.125, 0.332, 0.777 ), false, false)); //top
	scene.add( new Sphere( Vector( 100000, 0, 0 ), 99965, Vector( 0.223, 0.677, 0.020 ), false, false)); //right
	scene.add( new Sphere( Vector(0, -100010, 0), 99990, Vector( 0.300, 0.400, 0.700 ), false, false)); //down
	scene.add( new Sphere( Vector( 0, 0, -100000 ), 99970, Vector( 0.273, 0.494, 0.453 ), false, false)); //back
	scene.add( new Sphere( Vector( 0, 0, 100050 ), 99970, Vector( 0.780, 0.094, 0.298 ), false, false)); //behind

	TriangleMesh* mesh = new TriangleMesh( Vector( 1.0, 1.0, 1.0 ), false, false );
    mesh->readOBJ("cat/cat.obj");
	mesh->scaleTranslate( 0.5, Vector( 0, -40, 40 ) ); 
	mesh->BVHStart();
    scene.add( mesh );	
	// TriangleMesh* testTriangle = new TriangleMesh(Vector(1.0, 0.0, 0.0), false, false);
	// testTriangle->vertices.push_back(Vector(-10, 0, 20));
	// testTriangle->vertices.push_back(Vector(10, 0, 20));
	// testTriangle->vertices.push_back(Vector(0, 15, 20));
	// TriangleIndices tri(0, 1, 2);
	// testTriangle->indices.push_back(tri);
	// scene.add(testTriangle);
	
	Vector camera_origin(0, 0, 72); 

	double fov = 60 * M_PI / 180.;
	//Sphere S(Vector (0, 0, 0), 10, Vector(1, 1, 1));
	Vector albedo(1, 1, 1);
	double I = 70000;
	//double I = 1.0;
	Vector light_pos( -10, 20, 40 );

	std::vector<unsigned char> image(W * H * 3, 0);

	int NB_PATHS = 30;

	auto start_time = std::chrono::high_resolution_clock::now();  //this was written using AI

	#pragma omp parallel for schedule( dynamic, 1)
	for ( int i = 0; i < H; i++ ) {
    	for ( int j = 0; j < W; j++ ) {  //page 32 in the lecture notes
        	Vector pixelColor( 0., 0., 0. );

        	for ( int k = 0; k < NB_PATHS; k++ ) {
            	double d = -W / ( 2. * tan( fov / 2 ) );  //Computing the direction of rays page 15
            	double X = uniform( engine ), Y = uniform( engine );
            
            	Vector rand_dir( j - W / 2 + X, H / 2 - i + Y, d );
            	rand_dir.normalize();

            	Ray ray(camera_origin, rand_dir);
				ray.t = k / ( double ) NB_PATHS;  //randomly select the time parameter
            	pixelColor = pixelColor + scene.getColour( ray, 5, light_pos, I);
        	}
        
        	double color1 = std::pow( pixelColor[0] / NB_PATHS, 1 / 2.2) * 255;
        	double color2 = std::pow( pixelColor[1] / NB_PATHS, 1 / 2.2) * 255;
        	double color3 = std::pow( pixelColor[2] / NB_PATHS, 1 / 2.2) * 255;

        	image[(i * W + j) * 3 + 0] = std::min(255., color1);
        	image[(i * W + j) * 3 + 1] = std::min(255., color2);
        	image[(i * W + j) * 3 + 2] = std::min(255., color3);
    	}
	}

	auto end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration
    std::chrono::duration<double> elapsed = end_time - start_time;
    std::cout << "Rendering time (in parallel): " << elapsed.count() << " seconds" << std::endl;  //these last 3 lines were also written using AI

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}