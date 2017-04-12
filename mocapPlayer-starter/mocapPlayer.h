/*
player.h

All the user interface functions .

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012
*/

#ifndef _PLAYER_H
#define _PLAYER_H

#include <FL/Fl_Gl_Window.H>

class Player_Gl_Window : public Fl_Gl_Window 
{
public:
  inline Player_Gl_Window(int x, int y, int w, int h, const char *l=0) : 
  Fl_Gl_Window(x, y, w, h, l) {};

  /* This is an overloading of a Fl_Gl_Window call.  It is called
  whenever a window needs refreshing. */
  void draw(); 

  /* This is an overloading of a Window call.  It is 
  called whenever a event happens inside the space 
  taken up by the Anim_Gl_Window. */		
  int handle(int event); 
};


typedef struct _MouseT {
  int button;
  int x;
  int y;
} MouseT;


typedef struct _CameraT {
  double zoom;
  double tw;
  double el;
  double az;
  double tx;
  double ty;
  double tz;
  double atx;
  double aty;
  double atz;
} CameraT;


void GraphicsInit();
void display();

#endif
