#include <vector>
#include <limits>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include "TriangleMesh.h"

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



//--------------------------------------------------------------------------------------------------------------------------------------------------------

class Intersection {
    public:
        bool isIntersection;
        double t;
        Vector P, N, albedo;
        bool isMirror;
        Intersection (bool isIntersection ) : isIntersection( isIntersection ) {} ;
};

class Object {
    public:
       // Object( const Vector& albedo = Vector(1., 1., 1. ), bool isMirro = false, bool isTransparent = false ) : albedo(albedo), isMirror(isMirror), isTransparent(isTransparent) {}
        //virtual bool intersect( const Ray& r, Vector &P, Vector &N, double &t ) const = 0;

        Vector albedo;
        bool isMirror;
        //bool isTransparent;
        virtual bool intersect ( const Ray& r ) = 0;
        virtual Intersection intersection ( const Ray& r ) = 0;
};

class TriangleIndices {
    public:
        TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
        };
        int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
        int uvi, uvj, uvk;  // indices within the uv coordinates array
        int ni, nj, nk;  // indices within the normals array
        int group;       // face group
};

class BoundingBox{
    public:
        BoundingBox( const Vector& m = Vector( 0, 0, 0 ), const Vector& M = Vector(0, 0, 0 ) ) : m(m), M(M) {}
        
        bool intersect( const Ray& r ) const{  //from the board
            double tx1 = ( m[0] - r.O[0] ) / r.u[0];
            double tx2 = ( M[0] - r.O[0] ) / r.u[0];
            double txMin = std::min( tx1, tx2 );
            double txMax = tx1 + tx2 - txMin;

            double ty1 = ( m[1] - r.O[1] ) / r.u[1];
            double ty2 = ( M[1] - r.O[1] ) / r.u[1];
            double tyMin = std::min( ty1, ty2 );
            double tyMax = ty1 + ty2 - tyMin;

            double tz1 = ( m[2] - r.O[2] ) / r.u[2];
            double tz2 = ( M[2] - r.O[2] ) / r.u[2];
            double tzMin = std::min( tz1, tz2 );
            double tzMax = tz1 + tz2 - tzMin;

            double t1 = std::min( txMax, std::min( tyMax, tzMax ) );
            double t2 = std::max( txMin, std::min(tyMin, tzMin ) );

            if( t2 > t1 )
                return false;
            if( t1 < 0)
                return false;
            return true;
        }

        Vector m, M;
};

class TriangleMesh : public Object {
public:
    ~TriangleMesh() {}
    TriangleMesh() {};

    //MORE FUNCTIONS TO ADD HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    Vector albedo;
    bool isMirror;
    
    void Object( const Vector & albedo = Vector ( 1., 1., 1. ) ){
        isMirror = false;
    };

    bool intersect ( const Ray& r ) {  //from the board
        
        bool intersection = false;

        int i;
        for ( i = 0; i < indices.size(); i++ ) {
            const Vector& A = vertices[ indices[i].vtxi ];
            const Vector& B = vertices[ indices[i].vtxj ];
            const Vector& C = vertices[ indices[i].vtxk ];

            const Vector e1 = B - A;
            const Vector e2 = C - A;
            const Vector N = cross( e1, e2 );

            double invUN = 1. / dot( r.u, N );
            Vector OAcrossU = cross( A - r.O, r.u );

            double beta = dot( e2, OAcrossU ) * invUN;
            double gamma = -dot( e1, OAcrossU ) * invUN;
            double alpha = 1 - beta - gamma;

            double t_local = dot( A - r.O, N ) * invUN;

            if( t_local < 0 )
                continue;
            if( beta < 0 || beta > 1 )
                continue;
            if( gamma < 0 || gamma > 1 )
                continue;
            if( alpha < 0 || alpha > 1 )
                continue;

            intersection = true; 
            break;
            
        }

        return intersection;

    }

    //---------------------------------------------------------------------------------------------------------------

    void readOBJ(const char* obj) {
 
        char matfile[255];
        char grp[255];
 
        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;
 
            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());
 
            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }
 
            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;
 
                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));
 
                    vertices.push_back(vec);
                    vertexcolors.push_back(col);
 
                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;
 
                char* consumedline = line + 1;
                int offset;
 
                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }
 
                consumedline = consumedline + offset;
 
                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;                             
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
 
            }
 
        }
        fclose(f);
    }

}


//-------------------------------------------------------------------------------------------------------------------------------------------------------
class Sphere : public Object {
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
	std::vector<Object*> objects;
	void add(Object &s) { objects.push_back(s); }
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
				Vector i = ray.u, normal_vec = N;
				double n1, n2; //these represent the refr indices and we have to give different values for them depending on the situation
				double angle_i = dot( i, N );
				bool condition;  //supposes dot(i, N) < 0 in the lectur enotes, we change the mediums
				if ( angle_i < 0 )
					condition = true;
				else
					condition = false;
				
				//now we treat the cases if it enters in the medium or not
				if ( condition == true ) //from air to glass
					n1 = 1.0, n2 = 1.5;   //n1 = air, n2 = glass, according to lecture notes
				else {  //glass to air
					n1 = 1.5, n2 = 1.0;
					normal_vec = ( -1.0 ) * N;
					angle_i = ( -1 ) * angle_i;
				}
				
				double computation;
				computation = 1.0 - ( n1 / n2 ) * ( n1 / n2 ) * ( 1.0 - angle_i * angle_i );
				
				if ( computation < 0) {
					//slide 37
					Vector direction_of_reflection = i - 2 * dot( i, normal_vec ) * normal_vec;
					direction_of_reflection.normalize();
					Ray newRay( P + 1e-4 * direction_of_reflection, direction_of_reflection );
					
					return getColour( newRay, ray_depth - 1, light_pos, I );
				} 
				else {
					double t_N = -sqrt( computation );//slide 39
					Vector computation2 = i - dot( i, normal_vec ) * normal_vec;
					Vector t_T = ( n1 / n2) * computation2;
					Vector direction_of_refraction = t_N * normal_vec + t_T; //* computation2
					direction_of_refraction.normalize();
					Ray newRay( P + 1e-4 * direction_of_refraction, direction_of_refraction );
					
					return getColour( newRay, ray_depth - 1, light_pos, I );
				}
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
    scene.add( Sphere( Vector( -20, -10, 6 ), 10, Vector( 0.102, 0.255, 0.541 ), true, false ) );
	scene.add( Sphere( Vector( 20, -10, 6 ), 10, Vector( 0.941, 0.627, 0.949 ), false, false ) ); //added 2 balls to see the difference in shadow and lter for mirror
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
	double I = 80000;
	//double I = 1.0;
	Vector light_pos( -10, 20, 40 );

	std::vector<unsigned char> image(W * H * 3, 0);
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
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}