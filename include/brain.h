#ifndef BRAIN_H
#define BRAIN_H


class Brain{
public:
	virtual void step( int step ) = 0;
};

class DefaultBrain:public Brain{
public:
	DefaultBrain();
	void step( int step );
};

#endif