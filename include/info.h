#include "animats.h"
#include "morph/display.h"
#include <X11/Xlib.h>
#include <X11/X.h>

using namespace morph;

class AnimatInfoWindow{
private:
	Window infoPanel;
	morph::Gdisplay *gdisp;
	int width;
	int height;
	int animat;
	int screen;
public:

	AnimatInfoWindow( morph::Gdisplay *disp, int rwidth, int rheight, bool docked );

	void draw( Animat &a );	

	vec map( Animat& a, vec p );
};