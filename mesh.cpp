#include "mesh.h"

//BoundingBox function to be inserted here

double intersectPlane( const Ray& r, const Vector& N, const Vector& A ){
    return dot( A - r.origin, N ) / dot( r.u, N );
}

bool BoundingBox::intersect( const Ray& r ) const{
    double tMinX, tMinY, tMinZ, tMaxX, tMaxY, tMaxZ;
    //here i have to compute the min
    tMinX = intersectPlane( r, Vector(1, 0, 0), MinB );
    tMinY = intersectPlane( r, Vector(0, 1, 0), MinB );
    tMinZ = intersectPlane( r, Vector(0, 0, 1), MinB );
    //max
    tMaxX = intersectPlane( r, Vector(1, 0, 0), MaxB );
    tMaxY = intersectPlane( r, Vector(0, 1, 0), MaxB );
    tMaxZ = intersectPlane( r, Vector(0, 0, 1), MaxB );

    double aux, tMin, tMax;
    if( tMinX > tMaxX ) {
        aux = tMinX;
        tMinX = tMaxX;
        tMaxX = aux;
    }
    if( tMinY > tMaxY ) {
        aux = tMinY;
        tMinY = tMaxY;
        tMaxY = aux;
    }
    if( tMinZ > tMaxZ ) {
        aux = tMinZ;
        tMinZ = tMaxZ;
        tMaxZ = aux;
    }

    tMin = std::max( tMinX, std::max( tMinY, tMinZ ) );
    tMax = std::min( tMaxX, std::min( tMaxY, tMaxZ ) );
    bool result;
    if( tMin < tMax && tMax > 0 ) result = true;
    else result = false;
    return result;
}

//insert the BVH here!!!!!!!
//page 46 from the lecture notes:
BVHNode::BVHNode( const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int start, int end ) {
    leftRecursive = rightRecursive = NULL;
    starting_triangle = start;
    ending_triangle = end;
    bbox = BoundingBox( vertices, indices, start, end );

    Vector diag = bbox.MaxB + bbox.MinB;  //twice the center point so we divide by 2, as in the lecture notes
    Vector middle_diag = diag * 0.5;  //like this there are less operations => faster

    int longest_axis = ( diag[0] >= diag[1] ) ? 
                   ( ( diag[0] >= diag[2]) ? 0 : 2 ) :
                   ( ( diag[1] >= diag[2]) ? 1 : 2 );
    int pivot_index = start;

    int i;
    for( i = start; i < end; i++ ) {
        TriangleIndices tr_ind = indices[i];
        Vector barycenter = ( vertices[tr_ind.vtxi] + vertices[tr_ind.vtxj] + vertices[tr_ind.vtxk] ) / 3.0;

        if( barycenter[longest_axis] < middle_diag[longest_axis] ){
            std::swap( indices[i], indices[pivot_index] );
            pivot_index++;
        }
    }
    if( pivot_index <= start || pivot_index >= end - 1 || end - start < 5 ) return;

    leftRecursive = new BVHNode( vertices, indices, start, pivot_index );
    rightRecursive = new BVHNode( vertices, indices, pivot_index, end );
}

void BVHNode::intersect( const Ray& r, std::vector<int>& indices2 ){
    if( !bbox.intersect( r ) ) return;

    if( leftRecursive == NULL && rightRecursive == NULL ) {
        size_t i;
        for( i = starting_triangle; i < ending_triangle; i++ ) {
            indices2.emplace_back( i );
        }
        return;
    }
    leftRecursive->intersect( r, indices2 );
    rightRecursive->intersect( r, indices2 );
}

TriangleMesh::TriangleMesh( const Vector& color, bool mirror, bool transparent ) : Geometry( color, mirror, transparent ), BVHRoot( NULL ) {}

void TriangleMesh::scaleTranslate( double scaleNr, Vector translateNr ) {
    for( Vector& v : vertices ) {
        v = v + translateNr;
        v = v * scaleNr;
    }
    bbox = BoundingBox(vertices);
}; 

//i implementg the intersect

bool TriangleMesh::intersect( const Ray& r, Vector &P, Vector &N ) const {
    //go through all the triangles of the mesh
    //for each triangle check if the ray is inside
    //if it is, update P and N if this is the closest intersection

    // if ( !bbox.intersect( r ) ) {
    //     return false;
    // }
    Vector origin = r.origin;
    Vector u = r.u;
    double alpha, beta, gamma, denominator, t2;
    Vector A, B, C, N2;
    double t = std::numeric_limits<double>::max();

    //t = std::numeric_limits<double>::max();
    std::vector<int> indices2;

    if( BVHcase == true )
        BVHRoot->intersect( r, indices2 );

    else{
        for( size_t i = 0; i < indices.size(); i++ ){
            indices2.emplace_back(i);
        }
    }

    
    for( int i : indices2 ) {
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
            //PHONG INTERPOLATION TO BE ADDED HERE!!!
            if( Phong == true ){
                N = alpha * normals[ triangle.ni ] + beta * normals[ triangle.nj ] + gamma * normals[ triangle.nk ];
                N.normalize();
            }
            else{
                N2.normalize();
                N = N2;
            }
            
        }
    }
    bool result = ( t < std::numeric_limits<double>::max() );
    if ( result == true )
        P = origin + u * t;
    return result;
};