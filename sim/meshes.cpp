#include "../include/meshes.h"

using namespace arma;
using namespace std;
using namespace Util;

using arma::vec;
using arma::mat;


void MeshProvider::addFace3( GeometricObject* go, int p1, int p2, int p3 ){
	cout << "Adding face: "<< p1 << ", " << p2 << ", " << p3<< ": " << go->points.size()<<endl;
	go->addFace( p1, p2, p3 );	
}

void MeshProvider::addEdgeUnique( GeometricObject* go, Edge& e ){
	bool in_vector = false;

	for( int i = 0; i < go->edges.size(); i++ ){
		Edge edge = go->edges[i];
		if( e.v0 == edge.v0 && e.v1 == edge.v1 )
			in_vector = true;
	}

	if( !in_vector )
		go->edges.push_back( e );
}

void MeshProvider::collectEdges( GeometricObject* go ){
	for( int i = 0; i < go->faces.size(); i++ ){		

		for( int j = 0; j < go->faces[i]->edges.size(); j++ )
			this->addEdgeUnique(go, go->faces[i]->edges[j]);
	} 
}


// CloneMeshProvider
CloneMeshProvider::CloneMeshProvider( GeometricObject* original ){
	this->original = original;	
}

void CloneMeshProvider::populate( GeometricObject* go ){
	for( PointMass *p : original->points ){
		go->addPoint( p->originalPosition );
	}

	for( Face *f : original->faces ){
		go->addFace( f->indexes[0], f->indexes[1], f->indexes[2]);
	}
}

// PlaneSkin
PlaneMeshProvider::PlaneMeshProvider(){

}

void PlaneMeshProvider::populate( GeometricObject* go ){

	vec v1 = {1, 1, 0};
	vec v2 = {1, -1, 0};
	vec v3 = {-1, -1, 0};
	vec v4 = {-1, 1, 0};
	
	go->addPoint( v1 );
	go->addPoint( v2 );
	go->addPoint( v3 );
	go->addPoint( v4 );

	this->addFace3( go, 0, 1, 2 );
	this->addFace3( go, 0, 2, 3 );
	this->collectEdges(go);
	this->fixNormal(go);

}

void PlaneMeshProvider::fixNormal(GeometricObject* go){
	vec u = {0, 0, 1}; // THIS IS GOIN TO FAIL FOR NON-HORIZONTAL

	for(Face *f : go->faces){
		f->normal = u;
		f->recompute = false;
	}
}


//RegularSphereSkin
RegularSphereMeshProvider::RegularSphereMeshProvider( double r, int N, int sc ){
	this->r = r;
	this->N = N;
	this->sc = sc;
}

CGALPoint RegularSphereMeshProvider::getPoint( double eta, double phi, double r ){
	double x = r*sin( eta )*cos( phi );
	double y = r*sin( eta )*sin( phi );
	double z = r*cos( eta );
	vec np = { {x}, {y}, {z} };
	CGALPoint p( x, y, z );

	return p;
}

void RegularSphereMeshProvider::generateMesh( GeometricObject* go, vector<CGALPoint>& points, int sc ){
	Smoother smoother( 5, 10 );
	Reconstruction reconstruct( points.begin(), points.end() );
	reconstruct.increase_scale( sc, smoother );
	Mesher mesher( smoother.squared_radius(),
             false, // Do not separate shells
             true // Force manifold output
           );                 
	reconstruct.reconstruct_surface( mesher );	
	vector<int> indexes;
	vector<vector<int>> lfaces;
	// Translating into the internal data structures
	for(Facet_iterator it = reconstruct.facets_begin(); it != reconstruct.facets_end(); ++it ){		
		int inds[3];

		for( int j = 0; j < 3; j++ ){
			int idx = find(indexes.begin(), indexes.end(), (*it)[j]) - indexes.begin();

			if( idx == indexes.size() ){
				
				CGALPoint currp = points[(*it)[j]];
				vec pos = {currp.x(), currp.y(), currp.z()};
				
				go->addPoint( pos );
				indexes.push_back((*it)[j]);
				idx = indexes.size()-1;
			}

			inds[j] = idx;
		}

		vector<int> fp = {inds[0], inds[1], inds[2]};
		lfaces.push_back(fp);
	}

	for( vector<int> inds : lfaces )		
		this->addFace3( go, inds[0], inds[1], inds[2]);

	this->collectEdges( go );
}

void RegularSphereMeshProvider::populate( GeometricObject* go ){
	
	vector<CGALPoint> points = generatePointCloud( N );
	generateMesh( go, points, sc );
}


// Generating a grid of points unifomily distributed on the sphere
vector<CGALPoint> RegularSphereMeshProvider::generatePointCloud( int N ){
	vector<CGALPoint> points;
	int N_count = 0;
	double a = 4*M_PI*r*r/N;
	double d = sqrt(a);
	int M_eta = round(M_PI/d), M_phi;
	double d_eta = M_PI/M_eta;
	double d_phi = a/d_eta;
	double eta, phi;

	for( int m = 0; m < M_eta; m++ ){
		eta = M_PI*( m + 0.5 )/M_eta;
		M_phi = round(2*M_PI*sin(eta)/d_phi);

		for( int n = 0; n < M_phi; n++ ){
			phi = 2*M_PI*n/M_phi;
			CGALPoint p = getPoint( eta, phi, r );
			points.push_back(p);
			N_count++;
		}
	}
	
	Debug::log(std::to_string(N_count) + string(" points generated in the point cloud."));
	return points;
}

// ObjMeshProvider
ObjMeshProvider::ObjMeshProvider( const char* path ){
	this->path = path;
}


void ObjMeshProvider::populate( GeometricObject* go ){
	FILE *f = fopen( this->path, "r" );
	ObjMeshProccessChain *chain = new VertexChainLink( 
								  new TextureChainLink(
								  new FaceChainLink(NULL))); 

	if( f == NULL ){
		cerr << "No mesh file present." << endl;
		exit(0);
		return;
	}

	char lineheader[128];

	while( fscanf( f, "%s", lineheader ) != EOF ){
		chain->process( f, lineheader, go );
	}

	fclose( f );
}

// ObjMeshProccessChain
ObjMeshProccessChain::ObjMeshProccessChain( ObjMeshProccessChain *next ){
	this->next = next;
}

void ObjMeshProccessChain::process( FILE *f, char *s, GeometricObject* go ){
	this->doProcess( f, s, go );

	if( this->next != NULL )
		this->next->process( f, s, go );
}

// VertexChainLink
VertexChainLink::VertexChainLink(ObjMeshProccessChain *next):ObjMeshProccessChain( next ){}

bool VertexChainLink::doProcess( FILE *f, char *s, GeometricObject* go ){
	if( strcmp( s, "v" ) == 0 ){

		float x, y, z;
		fscanf(f, "%f %f %f\n", &x, &y, &z );
		vec pos = { x, y, z };
		cout << "Adding point: "<<printvec(pos)<<endl;
		go->addPoint( pos );
		return true;
	}

	return false;
}

// TextureChainLink
TextureChainLink::TextureChainLink(ObjMeshProccessChain *next):ObjMeshProccessChain( next ){}

bool TextureChainLink::doProcess( FILE *f, char *s, GeometricObject* go ){
	// do nothing 
	if( strcmp( s, "vt" ) == 0 ){	
		cout << "Not processing textures so far" << endl;
	}

	return false;
}

// FaceChainLink
FaceChainLink::FaceChainLink(ObjMeshProccessChain *next):ObjMeshProccessChain( next ){}

bool FaceChainLink::doProcess( FILE *f, char *s, GeometricObject* go ){
	if( strcmp( s, "vn" ) == 0 ){	
		float x, y, z;
		fscanf(f, "%f %f %f\n", &x, &y, &z );
		vec pos = { x, y, z };
		cout << "Adding normal: "<<printvec(pos)<<endl;
		this->normals.push_back( pos );
		return true;
	}

	if( strcmp( s, "f" ) == 0 ){
		int vi, vj, vk, ti, tj, tk, ni, nj, nk;	
		// int matches = fscanf(f, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vi, &ti, &ni, &vj, &tj, &nj, &vk, &tk, &nk );
		int matches = fscanf(f, "%d//%d %d//%d %d//%d\n", &vi, &ni, &vj, &nj, &vk, &nk );
	    // if (matches != 9){
	    //     cerr<<"Error in the format of the mesh file"<<endl;
	    //     return false;
	    // }

	    vi--;
	    vj--;
	    vk--;
	    cout << "Adding face: "<< vi << ", "<< vj << ", "<< vk <<endl;
	    go->addFace( vi, vj, vk, normals[nk] );

	    return true;
	}

	return false;
}

