class ContactRegion{
private:
	vector<CPoint&> points;
	Mat P; // projector
	vec t; // translation
	bool fixed; 

	void compute();
public:
	void add( CPoint& p );
	void prune();
	void tighten();
};

class ContactList{
public:
	ContactList();
	ContactRegion& get( int i, int j );
	void tightenRegions();
	void correctVelocities();

};