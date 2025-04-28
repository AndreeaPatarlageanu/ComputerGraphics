#include "mesh.h"

TriangleMesh::TriangleMesh( const Vector& color, bool mirror, bool transparent ) 
    : Geometry( color, mirror, transparent ) {
}

void TriangleMesh::scaleTranslate( double scaleNr, Vector translateNr ) {
    for( Vector& v : vertices ) {
        v = v + translateNr;
        v = v * scaleNr;
    }
}; 

//i implementg the intersect

bool TriangleMesh::intersect( const Ray& r, Vector &P, Vector &N ) const {
    //go through all the triangles of the mesh
    //for each triangle check if the ray is inside
    //if it is, update P and N if this is the closest intersection
    //the same as in momo s function, except that for him P = r.origin + t * r.direction
    Vector origin = r.origin;
    Vector u = r.u;
    double alpha, beta, gamma, denominator, t2;
    Vector A, B, C, N2;
    double t = std::numeric_limits<double>::max();

    //t = std::numeric_limits<double>::max();
    std::vector<int> reduced_indices;
    for( size_t i = 0; i < indices.size(); i++ ){
        reduced_indices.emplace_back(i);
    }
    for( int i : reduced_indices ) {
        auto triangle = indices[i]; //from the lecture notes
        A = vertices[ triangle.vtxi ];
        B = vertices[ triangle.vtxj ];
        C = vertices[ triangle.vtxk ];

        N2 = cross( B - A, C - A );
        denominator = dot( u, N2 );
        beta = dot( C - A, cross( A - origin, u ) ) / denominator;
        gamma = (-1) * dot( B - A, cross( A - origin, u ) ) / denominator;
        alpha = 1 - beta - gamma;
        t2 = dot( A - origin, N2 ) / denominator;

        if( t2 > 0 && t > t2 && alpha >= 0 && beta >= 0 && gamma >= 0 ) {
            t = t2;
            N2.normalize();
            N = N2;
        }
    }
    bool result = ( t < std::numeric_limits<double>::max() );
    if ( result == true )
        P = origin + u * t;
    return result;
};