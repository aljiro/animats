#ifndef MESHES_H
#define MESHES_H

#include <vector>
#include <armadillo>
#include <cstdlib>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include "util.h"
#include "primitives.h"
#include "geometry.h"

using namespace arma;
using namespace std;
using namespace morph;


typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel>    Reconstruction;
typedef Kernel::Point_3 CGALPoint;
typedef Reconstruction::Facet_const_iterator                   Facet_iterator;
typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother< Kernel > Smoother;
typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher< Kernel >    Mesher;

class MeshProvider{
private:
	

public:
		
	//void addFace4( int p1, int p2, int p3, int p4 );
	void addFace3( GeometricObject*, int p1, int p2, int p3 );
	void addEdgeUnique( GeometricObject*, Edge& e );
	void collectEdges( GeometricObject* );
	/*
		Generates the triangulation of the polygon given by the polyhedron described by the given points
	*/
	virtual void populate( GeometricObject* go ) = 0;

};

class CloneMeshProvider: public MeshProvider{
private:
	GeometricObject *original;
public:
	CloneMeshProvider(GeometricObject *original);
	void populate( GeometricObject* go );
};


class PlaneMeshProvider : public MeshProvider{
private:	

public:

	PlaneMeshProvider( );
	void populate( GeometricObject* go );
	void fixNormal(GeometricObject*);

};

class RegularSphereMeshProvider : public MeshProvider{
private:
	int N;
	int sc;
	double r;
public:
	RegularSphereMeshProvider( double r, int N, int sc );
	CGALPoint getPoint( double eta, double phi, double r );
	void generateMesh( GeometricObject*, vector<CGALPoint>& points, int sc );
	// Interfaces
	void populate( GeometricObject* go );
	// Generating a grid of points unifomily distributed on the sphere
	vector<CGALPoint> generatePointCloud( int N );
};

class ObjMeshProvider : public MeshProvider{
private:
	const char* path;
public:
	ObjMeshProvider( const char* path );
	void populate( GeometricObject* go );
};

class ObjMeshProccessChain{
private:
	ObjMeshProccessChain * next;
	virtual bool doProcess( FILE* f, char *s, GeometricObject* go ) = 0;
public:
	ObjMeshProccessChain( ObjMeshProccessChain *next );
	void process( FILE *f, char *s, GeometricObject* go );
};

class VertexChainLink : public ObjMeshProccessChain{
private:
	bool doProcess( FILE *f, char *s, GeometricObject* go );
public:
	VertexChainLink(ObjMeshProccessChain *next);
};

class TextureChainLink : public ObjMeshProccessChain{
private:
	bool doProcess( FILE *f, char *s, GeometricObject* go );
public:
	TextureChainLink(ObjMeshProccessChain *next);
};

class FaceChainLink : public ObjMeshProccessChain{
private:
	vector<vec> normals;
	bool doProcess( FILE *f, char *s, GeometricObject* go );
public:
	FaceChainLink(ObjMeshProccessChain *next);
};

#endif