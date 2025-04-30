#include "Vector.h"
#include "Geometry.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <float.h>
#include <iostream>
#include <limits>
#include <stdio.h>
#include <string>
#include <vector>


class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {};
    
    int vtxi, vtxj, vtxk;
    int uvi, uvj, uvk;
    int ni, nj, nk;
    int group;
};

class BoundingBox{
public:
    Vector MinB, MaxB;
    static constexpr double MAX_CONST = std::numeric_limits<double>::max();
    BoundingBox() : MinB( MAX_CONST, MAX_CONST, MAX_CONST ),
                    MaxB( -MAX_CONST, -MAX_CONST, -MAX_CONST ) {}

    BoundingBox( const std::vector<Vector> &vertices ) : BoundingBox() { newVertices( vertices ); }
    BoundingBox( const std::vector<Vector> &vertices, const std::vector<TriangleIndices>& indices, int first, int last ) : BoundingBox() {
        int i;
        for( i = first; i < last; i++ ) {
            TriangleIndices triangle = indices[i];
            addVertex( vertices[ triangle.vtxi ] );
            addVertex( vertices[ triangle.vtxj ] );
            addVertex( vertices[ triangle.vtxk ] );
        }
    }

    void addVertex( const Vector& vertex ) {
        MinB.data[0] = std::min( MinB[0], vertex.data[0] );
        MinB.data[1] = std::min( MinB[1], vertex.data[1] );
        MinB.data[2] = std::min( MinB[2], vertex.data[2] );

        MaxB.data[0] = std::max( MaxB[0], vertex.data[0] );
        MaxB.data[1] = std::max( MaxB[1], vertex.data[1] );
        MaxB.data[2] = std::max( MaxB[2], vertex.data[2] );
    }

    void newVertices( const std::vector<Vector>& vertices ) {
        int i;
        for( i = 0; i < vertices.size(); i++ )
            addVertex( vertices[i] );
    }

    bool intersect( const Ray& r ) const;

};

//BVH!!!!!!!!!!
class BVHNode{
    BoundingBox bbox;
    BVHNode *leftRecursive, *rightRecursive;
    int starting_triangle, ending_triangle;

public: 
    BVHNode(): leftRecursive( NULL ), rightRecursive( NULL ) {};
    ~BVHNode() {
        if( leftRecursive != NULL ) delete leftRecursive;
        if( rightRecursive != NULL ) delete rightRecursive;
    }

    BVHNode( const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int start, int end );

    void intersect( const Ray& r, std::vector<int>& indices2 );
};


//end bvh


class TriangleMesh : public Geometry {
public:
        std::vector<TriangleIndices> indices;
        std::vector<Vector> vertices;
        std::vector<Vector> normals;
        std::vector<Vector> uvs;
        std::vector<Vector> vertexcolors;
        BoundingBox bbox;

        BVHNode *BVHRoot;
        bool BVHcase = true, Phong = true;

        ~TriangleMesh() {
            if( BVHRoot != NULL ) delete BVHRoot;
        }
        TriangleMesh( const Vector& color, bool mirror, bool transparent );  //this line to be commented for the BVH
        //TriangleMesh() : BVHRoot( NULL ){};
        
        void scaleTranslate( double scaleNr, Vector translateNr ); 

        bool intersect( const Ray& r, Vector &P, Vector &N ) const override;

        //BVH ones:
        void BVHStart() {
            BVHRoot = new BVHNode( vertices, indices, 0, indices.size() );
        };

        
        void readOBJ(const char* obj) {
     
            char matfile[255];
            char grp[255];
     
            FILE* f;
            f = fopen(obj, "r");
            //bbox = BoundingBox(vertices);
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
                        sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2] );
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
            //bbox = BoundingBox( vertices );
        }
        
};