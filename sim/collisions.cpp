#include "../include/collisions.h"

using namespace std;
using namespace arma;

CollisionManager::CollisionManager():ht(5000,0.2){}

void CollisionManager::findCollisions( int step ){
	// Prune the contact regions
	contacts.prune();
	// Hash the points      
	firstPass( step );
	// Scan the faces    
	secondPass( step );
	// Tightening the collision regions
	contacts.tightenRegions();
}

void CollisionManager::registerObject(GeometricObject *go){

	vector<int> objIdxs;
	Debug::log(string("Registering objects for collision detection"));

	// Put all the pointers to points in the corresponding vector
	for( int i = 0; i < go->points.size(); i++ ){
		
		objIdxs.push_back( this->points.size() );

		CPoint cp;
		cp.point = go->points[i];
		cp.go = go;
		cp.originalIdx = i;
		this->points.push_back( cp );
	}

	cout << this->points.size() <<  " points registered. "<<endl;
	Debug::log(string("Registering faces"));
	
	for( int i = 0; i < go->faces.size(); i++ ){
		CFace cf;
		cf.face = go->faces[i];
		cf.go = go;
		cf.aabb = new Box();
		cf.originalIdx = i;
		this->faces.push_back( cf );
	}

	cout << this->faces.size() << " faces registered " << endl;
	this->indexes[go] = objIdxs;
	this->objects[go] = this->objects.size();
}

void CollisionManager::firstPass( int step ){
	for( int i = 0; i < this->points.size(); i++ ){
		// Add hash
		CPoint cp = this->points[i];
		PointMass *p = cp.point;
		vector<int> idxs = indexes[cp.go]; 
		this->ht.hashIn( cp.point->position, i, step );
	}

	// Compute the bounding boxes
	for( CFace cf : this->faces ){
		Box::compute( cf.face->points, cf.aabb );
	}
}

void CollisionManager::evaluateContacts( CFace cf, int step ){
	unsigned int h;
	// Discretizing the AABB	
	this->ht.discretizeBox(cf.aabb);	

	// Hashing the cells
	for( int kx = (int)cf.aabb->min(0); kx <= cf.aabb->max(0); kx++ ){
		for( int ky = (int)cf.aabb->min(1); ky <= cf.aabb->max(1); ky++ ){
			for( int kz = (int)cf.aabb->min(2); kz <= cf.aabb->max(2); kz++ )
			{
				vec p = {kx, ky, kz};				
				h = ht.getHashDiscrete( p );
				CHashItem chi = ht.getItem( h );

				if( chi.timestamp == step ){					
					this->handleCollisions( cf, chi, step );
				}
			}
		}
	}
}

void CollisionManager::secondPass( int step ){
	for( int i = 0; i < this->faces.size(); i++ ){		
		CFace cf = this->faces[i];
		this->evaluateContacts( cf, step );
	}

}

void CollisionManaer::storeCollision( CFace& cf, CPoint& cp ){
	int i, j;

	PointMass *p = cp.point;
	Edge e( p->pre, p );
	vec pd = {ht.discretize(p->position(0)), ht.discretize(p->position(1)), ht.discretize(p->position(2))};
	
	if( (cf.face->isPenetrated( e ).result || 
		 cf.face->penetrates(p)) && 
		 cf.face->isInside(cf.face->getFaceProjection(e)) ){
	// Point p penetrates the face cf.face
		i = this->objects[ cp.go ];
		j = this->objects[ cf.go ];
		this->addCollisionResponse( cf.face, cp.point );
		this->contactRegions.get( i, j ).add( cp );
	}
}

void CollisionManager::handleCollisions( CFace cf, CHashItem chi, int step ){
	
	for( list<int>::iterator it = chi.items.begin(); it != chi.items.end(); ++it ){

		CPoint cp = this->points[*it];
		PointMass *p = cp.point;		
		
		if( cp.go == cf.go ) continue;
		if( p->pre == NULL ) continue;

		this->storeCollision( cp, cf );
	}
}

void CollisionManager::correctVelocities(){
	this->contacts.correctVelocities();
}


// Add corresponding velocity components to all points
// in order to preserve the momentum
void CollisionManager::compensateVelocities( GeometricObject *go, vec vc ){
		
	vector<int> idxs = indexes[go];

	for( int i = 0; i < idxs.size(); i++ ){
		CPoint cp = this->points[idxs[i]];
		//Debug::log(string("Compensating point: ") + printvec(cp.point->collisionVelComp));
		cp.point->vc += vc;	
	}
}


vec CollisionManager::addCollisionResponse( CFace& cf, CPoint& cp ){
	vec v = zeros<vec>(3);
	double j;
	PointMass *p = cp.point;
	Face *f = cf.face;
 	vec v1 = p->velocity;	
	vec n = f->normal;
	 // if( dot(v1, f->normal) > 0)
	 // 	n = -n;
	double Cr =  1;
	
	vec v2 = (f->points[0]->velocity + f->points[1]->velocity + f->points[2]->velocity)/3;

	if( dot(v1 - v2, n) > 0 )
		return v;

	double m1 = p->mass;
	double m2 = f->points[0]->mass;

	j = -(1.0 + Cr)*dot(v1 - v2, n)/(1/m1 + 1/m2);
	v = j*n/m1;
	co.point->vc = v;
}

vec CollisionManager::getRigidBodyResponse( GeometricObject* o1, GeometricObject* o2, vec penetration ){
// Physically based collision response
	vec v;
	double j;

	double Cr = 1;
	vec v1 = o1->points[0]->velocity;
	vec v2 = o2->points[0]->velocity;
	vec n = penetration;

	if( !(n(0) == 0 && n(1) == 0 && n(2) == 0) )
		n /= norm(n);

	double m1 = o1->getMass();
	double m2 = o2->getMass();
	cout << "Components of the response:"<<endl;
	cout << "v1: (" << o1->toString()<<")"<<printvec(v1)<<endl;
	cout << "v2 (" << o2->toString()<<"): "<<printvec(v2)<<endl;
	cout << "n: "<<printvec(n)<<endl;
	j = -(1 + Cr)*dot(v1 - v2, n)/(1/m1 + 1/m2);
	cout << "j: "<<j<<endl;
	v = j*n/m1;
	cout << "response: "<< printvec(v)<<endl;

	return v;
}

//Box
Box::Box(){

}

Box::Box( vec min, vec max ){
	
    this->min = min;
    this->max = max;
}

bool Box::collide( Box& b ){
    bool ex = this->max(0) > b.min(0) && this->min(0) < this->max(0);
    bool ey = this->max(1) > b.min(1) && this->min(1) < this->max(1);
    bool ez = this->max(2) > b.min(2) && this->min(2) < this->max(2);

    return ex && ey && ez;
}

// CHashTable
CHashTable::CHashTable( int n, double l ):hashes(n){
	this->n = n;
	this->l = l;
	this->hashes.reserve(n);
}

int CHashTable::discretize( double a ){
	int d = floor((a)/this->l);
	return d;
}

unsigned int CHashTable::getHash( vec point ){
	
	int x = discretize(point(0));
	int y = discretize(point(1));
	int z = discretize(point(2));
	vec p = {x, y, z};
	return this->getHashDiscrete(p);
	
}

unsigned int CHashTable::getHashDiscrete( vec p ){
	unsigned long h;
	unsigned long x = p(0);
	unsigned long y = p(1);
	unsigned long z = p(2);

	unsigned long t1 = (x*this->p1);
	unsigned long t2 = (y*this->p2) ;
	unsigned long t3 = (z*this->p3);

	h = (t1 ^ t2 ^ t3)%((unsigned long)this->n);

	return (unsigned int)h;
}

void CHashTable::hashIn( vec point, int index, int step ){
	unsigned int h = getHash( point );

	if( this->hashes[h].timestamp != step ){
		this->hashes[h].items.clear();
		this->hashes[h].timestamp = step;
	}

	this->hashes[h].items.insert( this->hashes[h].items.end(), index );
}

void CHashTable::discretizeBox( Box *b ){
	//cout << "Discretizing box - min: " << printvec(b->min) << ", max: "<<printvec(b->max)<<endl;
	b->min(0) = discretize(b->min(0));
	b->min(1) = discretize(b->min(1));
	b->min(2) = discretize(b->min(2));

	b->max(0) = discretize(b->max(0));
	b->max(1) = discretize(b->max(1));
	b->max(2) = discretize(b->max(2));
}

CHashItem CHashTable::getItem( unsigned int h ){
	return hashes[h];
}

