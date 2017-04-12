/*

Revision 1 - Steve Lin, Jan. 14, 2002
Revision 2 - Alla and Kiran, Jan 18, 2002
Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012

*/
#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>				
#include <fstream>
#include <cassert>
#include <cmath>

#include <FL/gl.h>
#include <FL/glut.H>  // GLUT for use with FLTK
#include <FL/Fl_File_Chooser.H> // file chooser for load/save
#include <FL/fl_ask.H>

#include "mocapPlayer.h"   		      
#include "interface.h"  // UI framework built by FLTK (using fluid)

#include "pic.h"   
#include <time.h>
#include "transform.h"  // utility functions for vector and matrix transformation  
#include "displaySkeleton.h"   
#include "performanceCounter.h"

enum SwitchStatus {OFF, ON};

DisplaySkeleton displayer;		

Skeleton *pSkeleton = NULL;	// Skeleton info as read from ASF file
Motion *pMotion = NULL;     // Motion information as read from AMC file

Fl_Window *form = NULL; // Global form 
MouseT mouse;    // Keeping track of mouse input 
CameraT camera;  // Structure about camera setting 

SwitchStatus playButton = OFF;
SwitchStatus rewindButton = OFF; 
SwitchStatus repeatButton = OFF;
SwitchStatus minusOneButton = OFF;
SwitchStatus plusOneButton = OFF;
SwitchStatus groundPlane = ON;
SwitchStatus previousPlayButtonStatus = playButton;

GLfloat groundPlaneLightHeight = 100.0;
GLint displayListGround;
int lastSkeleton = -1;
int lastMotion = -1;

char lastMotionFilename[FILENAME_MAX];

enum SaveScreenToFileMode
{
  SAVE_DISABLED, SAVE_ONCE, SAVE_CONTINUOUS
};

int saveScreenToFileOnceCount = 0;
char saveScreenToFileOnceFilename[FILENAME_MAX];
int saveScreenToFileContinuousCount = 0;
char saveScreenToFileContinuousFilename[FILENAME_MAX];
SaveScreenToFileMode saveScreenToFile = SAVE_DISABLED;

SwitchStatus useFog = OFF;
double fogStart = 4.0;
double fogEnd = 12.0;
double fogDensity = 0.1;

SwitchStatus renderWorldAxes = ON;

const double standardFPS = 120.0;
double expectedFPS = standardFPS;
// maximum number of frames among all the motions loaded so far
int maxFrames = 0;  
// Current frame and frame increment
int currentFrameIndex = 0;
double currentFrameIndexDoublePrecision = 0.0;
double framesIncrementDoublePrecision = 1.0;

PerformanceCounter performanceCounter;
PerformanceCounter saveFileTimeCounter;
double saveFileTimeCost = -1.0; // if value is negative, it means the data is invalid

void CreateScreenFilename(SaveScreenToFileMode saveToFileMode, int fileCount, char * filename)
{
  switch (saveToFileMode)
  {
  case SAVE_ONCE:
    strcpy(filename, "scr");
    break;
  case SAVE_CONTINUOUS:
    strcpy(filename, "pic");
  	break;
  default:
    printf("Error in CreateScreenFilename: saveToFileMode is not recognized.\n");
    exit(0);
    break;
  }
  char s[20];
  sprintf(s, "%05d.ppm", fileCount);
  strcat(filename, s);
}

static void RenderWorldAxes() 
{
  glBegin(GL_LINES);

  /* draw x axis in red, y axis in green, z axis in blue */
  glColor3f(1.0f, 0.2f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(1.0f, 0.0f, 0.0f);

  glColor3f(0.2f, 1.0f, 0.2f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 1.0f, 0.0f);
  glColor3f(0.2f, 0.2f, 1.0f);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 1.0f);

  glEnd();
}

void RenderGroundPlane(double groundPlaneSize, double groundPlaneHeight, double rPlane, double gPlane, double bPlane, double ambientFskeleton, double diffuseFskeleton, double specularFskeleton, double shininess)
{
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(2.0,2.0);

  float planeAmbient[4] = { (float)(ambientFskeleton * rPlane), (float)(ambientFskeleton * gPlane), (float)(ambientFskeleton * bPlane), 1.0f};
  float planeDiffuse[4] = { (float)(diffuseFskeleton * rPlane), (float)(diffuseFskeleton * gPlane), (float)(diffuseFskeleton * bPlane), 1.0f};
  float planeSpecular[4] = { (float)(specularFskeleton * rPlane), (float)(specularFskeleton * gPlane), (float)(specularFskeleton * bPlane), 1.0f};
  float planeShininess = (float)shininess;
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, planeAmbient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, planeDiffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, planeSpecular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, planeShininess);
  glNormal3f(0,1,0);
  const int planeResolution = 200;
  double planeIncrement = groundPlaneSize / planeResolution;
  for(int i=0; i<planeResolution; i++)
    for(int j=0; j<planeResolution; j++)
    {
        clock_t timeValue=clock();
        GLfloat coef1 = (sin(timeValue) / 2) + 0.5;
        GLfloat coef2 = (cos(timeValue) / 2) + 0.5;
        GLfloat coef3 = (sin(timeValue)+cos(timeValue) / 4) + 0.5;
      float planeAmbientAct[4] = { (float)(ambientFskeleton * rPlane), (float)(ambientFskeleton * gPlane), (float)(ambientFskeleton * bPlane), 1.0f};
      float factor = (((i+j) % 2) == 0) ? 0.5f : 1.0f;
//      planeAmbientAct[0] *= factor;
//      planeAmbientAct[1] *= factor;
//      planeAmbientAct[2] *= factor;
//      planeAmbientAct[3] *= factor;
        planeAmbientAct[0] *= coef1*factor;
        planeAmbientAct[1] *= coef2*factor;
        planeAmbientAct[2] *= coef3*factor;
        planeAmbientAct[3] *= (coef1+coef2+coef3)/3.0*factor;
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, planeAmbientAct);
      glBegin(GL_TRIANGLE_STRIP);
      glVertex3f((float)(-groundPlaneSize/2 + i * planeIncrement), (float)groundPlaneHeight, (float)(-groundPlaneSize/2 + j * planeIncrement));
      glVertex3f((float)(-groundPlaneSize/2 + (i+1) * planeIncrement), (float)groundPlaneHeight, (float)(-groundPlaneSize/2 + j * planeIncrement));
      glVertex3f((float)(-groundPlaneSize/2 + i * planeIncrement), (float)groundPlaneHeight, (float)(-groundPlaneSize/2 + (j+1) * planeIncrement));
      glVertex3f((float)(-groundPlaneSize/2 + (i+1) * planeIncrement), (float)groundPlaneHeight, (float)(-groundPlaneSize/2 + (j+1) * planeIncrement));

      glEnd();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void cameraView(void)
{
  glTranslated(camera.tx, camera.ty, camera.tz);
  glTranslated(camera.atx, camera.aty, camera.atz);

  glRotated(-camera.tw, 0.0, 1.0, 0.0);
  glRotated(-camera.el, 1.0, 0.0, 0.0);
  glRotated(camera.az, 0.0, 1.0, 0.0); 
  
  glTranslated(-camera.atx, -camera.aty, -camera.atz);
  glScaled(camera.zoom, camera.zoom, camera.zoom);
}

/*
* Redisplay() is called by Player_Gl_Window::draw().
*
* The display is double buffered, and FLTK swap buffers when
* Player_Gl_Window::draw() returns.  The GL context associated with this
* instance of Player_Gl_Window is set to be the current context by FLTK
* when it calls draw().
*/
void Redisplay() 
{
  /* clear image buffer to black */
  glClearColor(1.0, 1.0, 1.0, 0);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); /* clear image, zbuf */

  glPushMatrix();  /* save current transform matrix */

  cameraView();

  glLineWidth(2.0);  /* we'll draw background with thick lines */

  if (renderWorldAxes == ON)
  {
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_FOG);
    RenderWorldAxes();  /* draw a triad in the origin of the world coordinate */
  }

  if (groundPlane == ON)
  { 
    if (useFog == ON)
    {
      glEnable(GL_FOG);
      GLfloat fogColor[4] = {1.0, 1.0, 1.0, 1.0};
      glFogfv(GL_FOG_COLOR, fogColor);
      glFogf(GL_FOG_START, (float)fogStart);
      glFogf(GL_FOG_END, (float)fogEnd);
      glFogf(GL_FOG_DENSITY, (float)fogDensity);
      glFogi(GL_FOG_MODE, GL_LINEAR);
    }

    // draw_ground();
    glEnable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glCallList(displayListGround);

    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glLineWidth(1.0);
    glColor3f(0.1f, 0.1f, 0.1f);
    double ground[4] = {0,1,0,0};
    double light[4] = {0,groundPlaneLightHeight,0,1};
    displayer.RenderShadow(ground, light);
  }

  // render the skeletons
  if (displayer.GetNumSkeletons()) 
  {
    glEnable(GL_LIGHTING);
    glDisable(GL_FOG);
    displayer.Render(DisplaySkeleton::BONES_AND_LOCAL_FRAMES);
  }

  glPopMatrix(); // restore current transformation matrix
}

void renderWorldAxes_callback(Fl_Light_Button *obj, long val) 
{
  renderWorldAxes  = (SwitchStatus)worldAxes_button->value();
  glwindow->redraw();
}

void renderGroundPlane_callback(Fl_Light_Button *obj, long val) 
{
  groundPlane  = (SwitchStatus)groundPlane_button->value();
  glwindow->redraw();
}

void useFog_callback(Fl_Light_Button *obj, long val) 
{
  useFog  = (SwitchStatus)fog_button->value();
  glwindow->redraw();
}

void resetScene_callback(Fl_Button *button, void *)
{
  rewindButton = ON;
  playButton = OFF;
  repeatButton = OFF;
  lastSkeleton = -1;
  lastMotion = -1;
  displayer.Reset();
  maxFrames = 0;
  glwindow->redraw();
  framesIncrementDoublePrecision = 1.0;
  currentFrameIndex = 0;
  currentFrameIndexDoublePrecision = 0.0;
}

void saveScreenshot(int windowWidth, int windowHeight, char * filename);

void saveScreenToFile_callback(Fl_Button *button, void *)
{
  if (button == screenShot_button)
  {
    CreateScreenFilename(SAVE_ONCE, saveScreenToFileOnceCount, saveScreenToFileOnceFilename);
    saveScreenshot(640, 480, saveScreenToFileOnceFilename);
    printf("%s is saved to disk.\n", saveScreenToFileOnceFilename);
    saveScreenToFileOnceCount++;
    saveScreenToFile = SAVE_DISABLED;
  }
}

void resetPostureAccordingFrameSlider(void)
{
  currentFrameIndex = (int)frame_slider->value() - 1;
  currentFrameIndexDoublePrecision = currentFrameIndex;

  // display
  for (int skeletonIndex = 0; skeletonIndex < displayer.GetNumSkeletons(); skeletonIndex++)
  {
    int postureID;
    if (currentFrameIndex >= displayer.GetSkeletonMotion(skeletonIndex)->GetNumFrames())
      postureID = displayer.GetSkeletonMotion(skeletonIndex)->GetNumFrames() - 1;
    else
      postureID = currentFrameIndex;
    // Set skeleton to the first posture
    Posture * currentPosture = displayer.GetSkeletonMotion(skeletonIndex)->GetPosture(postureID);
    displayer.GetSkeleton(skeletonIndex)->setPosture(*currentPosture);
  }
}

void UpdateMaxFrameNumber(void)
{
  maxFrames = 0;
  for(int skeletonIndex = 0; skeletonIndex < displayer.GetNumSkeletons(); skeletonIndex++)
  {
    int currentFrames = displayer.GetSkeletonMotion(skeletonIndex)->GetNumFrames();
    if (currentFrames > maxFrames)
      maxFrames = currentFrames;
  }
}

void load_callback(Fl_Button *button, void *) 
{
  if(button == loadSkeleton_button)
    if (lastSkeleton <= lastMotion)  // cannot load new skeleton until motion is assigned to the current skeleton
    {
      char * filename = fl_file_chooser("Select filename","*.ASF","");
      if(filename != NULL)
      {
        // Read skeleton from asf file
        pSkeleton = new Skeleton(filename, MOCAP_SCALE);
        lastSkeleton++;
        // Set the rotations for all bones in their local coordinate system to 0
        // Set root position to (0, 0, 0)
        pSkeleton->setBasePosture();
        displayer.LoadSkeleton(pSkeleton);
        glwindow->redraw();
      }
    }

  if(button == loadMotion_button)
  {
    if ((lastSkeleton >= 0) && (lastSkeleton >= lastMotion))
    {
      char * filename = fl_file_chooser("Select filename","*.AMC","");
      if(filename != NULL)
      {
        // Read motion (.amc) file and create a motion
        pMotion = new Motion(filename, MOCAP_SCALE, pSkeleton);

        // backup the filename
        strcpy(lastMotionFilename, filename);

        // set sampled motion for display
        displayer.LoadMotion(pMotion);      
        if (lastSkeleton > lastMotion)         
          lastMotion++;
        
        UpdateMaxFrameNumber();
        resetPostureAccordingFrameSlider();
        frame_slider->value(currentFrameIndex);
        frame_slider->maximum((double)maxFrames);
        frame_slider->redraw();
        glwindow->redraw();
        Fl::flush();
      }
    } // if (lastSkeleton > lastMotion)
  }
  glwindow->redraw();
}

void reload_callback(Fl_Button *button, void *) 
{
  if (!displayer.GetNumSkeletons())
    return;

  // Read motion (.amc) file and create a motion
  pMotion = new Motion(lastMotionFilename, MOCAP_SCALE, pSkeleton);

  // Set sampled motion for display
  displayer.LoadMotion(pMotion);
  
  resetPostureAccordingFrameSlider();
  UpdateMaxFrameNumber();
  frame_slider->maximum((double)maxFrames);
  frame_slider->value(currentFrameIndex);
  frame_slider->redraw();
  Fl::flush();
  glwindow->redraw();
}

void play_callback(Fl_Button * button, void *)
{
  if (button == play_button)     { minusOneButton = OFF; plusOneButton = OFF; rewindButton = OFF; playButton = ON;  rewindButton = OFF; }
  if (button == minusOne_button) { minusOneButton = ON;  plusOneButton = OFF; rewindButton = OFF; playButton = OFF; repeatButton = OFF; }
  if (button == plusOne_button)  { minusOneButton = OFF; plusOneButton = ON;  rewindButton = OFF; playButton = OFF; repeatButton = OFF; }
  if (button == pause_button)    { minusOneButton = OFF; plusOneButton = OFF; rewindButton = OFF; playButton = OFF; repeatButton = OFF; } 
  if (button == repeat_button)   { minusOneButton = OFF; plusOneButton = OFF; rewindButton = OFF; playButton = ON;  repeatButton = ON;  }
  if (button == rewind_button)   { minusOneButton = OFF; plusOneButton = OFF; rewindButton = ON;  playButton = OFF; repeatButton = OFF; }
  
  if ((previousPlayButtonStatus == OFF) && (playButton == ON))
    framesIncrementDoublePrecision = 1.0;  // Just start playing the animation, no time has been measured

  if (button == pause_button)
     if (saveScreenToFile == SAVE_CONTINUOUS)
       saveScreenToFile = SAVE_DISABLED;
}

void record_callback(Fl_Light_Button * button, void * )
{
  if ((SwitchStatus)(record_button->value()) == OFF)
    saveScreenToFile = SAVE_DISABLED;
  else
    saveScreenToFile = SAVE_CONTINUOUS;
  glwindow->redraw();
}

/*
Set all skeletons to a specified frame (frameIndex). If frameIndex is larger than the number of frames of the motion, 
set the skeleton to the last frame of the motion.
*/
void SetSkeletonsToSpecifiedFrame(int frameIndex)
{
  if (frameIndex < 0)
  {
    printf("Error in SetSkeletonsToSpecifiedFrame: frameIndex %d is illegal.\n", frameIndex);
    exit(0);
  }
  for (int skeletonIndex = 0; skeletonIndex < displayer.GetNumSkeletons(); skeletonIndex++)
    if (displayer.GetSkeletonMotion(skeletonIndex) != NULL)
    {
      int postureID;
      if (frameIndex >= displayer.GetSkeletonMotion(skeletonIndex)->GetNumFrames())
        postureID = displayer.GetSkeletonMotion(skeletonIndex)->GetNumFrames() - 1;
      else 
        postureID = frameIndex;
      displayer.GetSkeleton(skeletonIndex)->setPosture(* (displayer.GetSkeletonMotion(skeletonIndex)->GetPosture(postureID)));
    }
}

// Write a screen-shot, in the PPM format, to the specified filename, in PPM format
void saveScreenshot(int windowWidth, int windowHeight, char * filename)
{
  if (filename == NULL)
    return;

  // Allocate a picture buffer 
  Pic * in = pic_alloc(windowWidth, windowHeight, 3, NULL);

  printf("File to save to: %s\n", filename);

  for (int i=windowHeight-1; i>=0; i--) 
  {
    glReadPixels(0, windowHeight-i-1, windowWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
      &in->pix[i*in->nx*in->bpp]);
  }

  if (ppm_write(filename, in))
    printf("File saved Successfully\n");
  else
    printf("Error in Saving\n");

  pic_free(in);
}
void idle(void*)
{
  if (previousPlayButtonStatus == ON)  
  {
    // it means we should measure the interval between two frames
    // if it is too tiny, we should slow down the motion
    performanceCounter.StopCounter();
    double actualTimeCostOneFrame = performanceCounter.GetElapsedTime(); // in seconds

    // time spent on saving the screen in previous time-step should be excluded
    if (saveFileTimeCost > 0.0)   
      actualTimeCostOneFrame -= saveFileTimeCost;

    framesIncrementDoublePrecision = actualTimeCostOneFrame * expectedFPS;
  }
  // start counter at the beginning of the new round
  if (playButton == ON)
    performanceCounter.StartCounter();

  if(rewindButton == ON)
  {
    currentFrameIndex = 0;
    currentFrameIndexDoublePrecision = 0.0;
    for (int i = 0; i < displayer.GetNumSkeletons(); i++)
    {
      if (displayer.GetSkeletonMotion(i) != NULL)
      {
        Posture * initSkeleton = displayer.GetSkeletonMotion(i)->GetPosture(0);
        displayer.GetSkeleton(i)->setPosture(*initSkeleton);
      }
    }
    rewindButton = OFF;
  }

  // Initialization
  saveFileTimeCost = -1.0;

  if(playButton == ON) 
  {
    if (saveScreenToFile == SAVE_CONTINUOUS)
    {
      saveFileTimeCounter.StartCounter();
      CreateScreenFilename(SAVE_CONTINUOUS, saveScreenToFileContinuousCount, saveScreenToFileContinuousFilename);
      saveScreenshot(640, 480, saveScreenToFileContinuousFilename);
      printf("%s is saved to disk.\n", saveScreenToFileContinuousFilename);
      saveScreenToFileContinuousCount++;
      saveFileTimeCounter.StopCounter();
      saveFileTimeCost = saveFileTimeCounter.GetElapsedTime();
    }

    if (saveScreenToFile == SAVE_CONTINUOUS)
    {
      currentFrameIndexDoublePrecision += 1.0;
    }
    else
    {
      currentFrameIndexDoublePrecision += framesIncrementDoublePrecision;
    }

    currentFrameIndex = (int)currentFrameIndexDoublePrecision;

    if(currentFrameIndex >= maxFrames)
    {
      if (repeatButton == ON)
      {
        currentFrameIndex = 0;
        currentFrameIndexDoublePrecision = 0.0;
      }
      else  // repeat button is OFF
      {
        currentFrameIndex = maxFrames - 1;
        currentFrameIndexDoublePrecision = currentFrameIndex;
        playButton = OFF;  // important, especially in "recording" mode
      }
    }

    if (currentFrameIndex < 0)
    {
      currentFrameIndex = 0;
      currentFrameIndexDoublePrecision = 0.0;
    }

    SetSkeletonsToSpecifiedFrame(currentFrameIndex);

    frame_slider->value((double) currentFrameIndex + 1);
  }  // if(playButton == ON)

  if (minusOneButton == ON)
    if (displayer.GetNumSkeletons() != 0)
    {
      currentFrameIndex--;
      if (currentFrameIndex < 0)
        currentFrameIndex = 0;
      frame_slider->value((double) currentFrameIndex + 1);

      SetSkeletonsToSpecifiedFrame(currentFrameIndex);    
      if (saveScreenToFile == SAVE_CONTINUOUS)
      {
        CreateScreenFilename(SAVE_CONTINUOUS, saveScreenToFileContinuousCount, saveScreenToFileContinuousFilename);
        saveScreenshot(640, 480, saveScreenToFileContinuousFilename);
        printf("%s is saved to disk.\n", saveScreenToFileContinuousFilename);
        saveScreenToFileContinuousCount++;
      }
      minusOneButton = OFF;
    }

  if (plusOneButton == ON)
  {
    if (displayer.GetNumSkeletons() != 0)
    {
      currentFrameIndex++;
      if (currentFrameIndex >= maxFrames)
        currentFrameIndex = maxFrames - 1;
      frame_slider->value((double) currentFrameIndex + 1);

      SetSkeletonsToSpecifiedFrame(currentFrameIndex);
      if (saveScreenToFile == SAVE_CONTINUOUS)
      {
        CreateScreenFilename(SAVE_CONTINUOUS, saveScreenToFileContinuousCount, saveScreenToFileContinuousFilename);
        saveScreenshot(640, 480, saveScreenToFileContinuousFilename);
        printf("%s is saved to disk.\n", saveScreenToFileContinuousFilename);
        saveScreenToFileContinuousCount++;
      }
      plusOneButton = OFF;
    }
  }

  frame_slider->value((double)(currentFrameIndex + 1));

  previousPlayButtonStatus = playButton; // Super important updating

  glwindow->redraw();
}

void fslider_callback(Fl_Value_Slider *slider, long val)
{
  currentFrameIndex = (int) frame_slider->value() - 1;
  currentFrameIndexDoublePrecision = currentFrameIndex;
  minusOneButton = OFF; 
  plusOneButton = OFF; 
  rewindButton = OFF; 
  playButton = OFF; 
  repeatButton = OFF;
  SetSkeletonsToSpecifiedFrame(currentFrameIndex);
  Fl::flush();
}

void playSpeed_callback(Fl_Value_Input *obj, void *)
{
  //framesIncrementDoublePrecision = speedUp->value();
  double speedRatio = speedUp->value();
  expectedFPS = standardFPS * speedRatio;
  glwindow->redraw();
}

void spotJoint_callback(Fl_Value_Input *obj, void *)
{
  displayer.SetDisplayedSpotJoint((int) joint_idx->value());
  glwindow->redraw();
}

void skeletonID_callback(Fl_Value_Input *obj, void*)
{
  int subnum;
  subnum = (int)sub_input->value();

  if (subnum < 0) 
  {
    sub_input->value(0);
    subnum = 0;
  }
  else
    if (subnum >= displayer.GetNumSkeletons())
    {
      sub_input->value(displayer.GetNumSkeletons() - 1);
      subnum = displayer.GetNumSkeletons() - 1;
    }
    
  // Change values of other inputs to match sub-number
  double translation[3];
  displayer.GetSkeleton(subnum)->GetTranslation(translation);
  double rotationAngle[3];
  displayer.GetSkeleton(subnum)->GetRotationAngle(rotationAngle);
  tx_input->value(translation[0]);
  ty_input->value(translation[1]);
  tz_input->value(translation[2]);
  rx_input->value(rotationAngle[0]);
  ry_input->value(rotationAngle[1]);
  rz_input->value(rotationAngle[2]);
 
  glwindow->redraw();
}

void tx_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();
  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetTranslationX(tx_input->value());
  glwindow->redraw();
}

void ty_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();

  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetTranslationY(ty_input->value());

  glwindow->redraw();
}

void tz_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();

  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetTranslationZ(tz_input->value());

  glwindow->redraw();
}

void rx_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();
  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetRotationAngleX(rx_input->value());
  glwindow->redraw();
}

void ry_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();
  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetRotationAngleY(ry_input->value());
  glwindow->redraw();
}

void rz_callback(Fl_Value_Input *obj, void*)
{
  int subnum = 0;
  subnum = (int)sub_input->value();
  if (subnum < displayer.GetNumSkeletons() && subnum >= 0)
    displayer.GetSkeleton(subnum)->SetRotationAngleZ(rz_input->value());
  glwindow->redraw();
}

void aboutPlayer_callback(Fl_Button * button, void *)
{
  fl_message_title("About ASF/AMC Motion Capture Player");
  fl_message("ASF/AMC Motion Capture Player.\n\nVersion 1.0: Steve Lin, Alla Safonova, Kiran Bhat\nCarnegie Mellon University, 2002\n\nVersion 2.0: Yili Zhao, Jernej Barbic\nUniversity of Southern California, 2012\n");
}

void GraphicsInit() 
{
  int red_bits, green_bits, blue_bits;
  struct {GLint x, y, width, height;} viewport;
  glEnable(GL_DEPTH_TEST);	/* turn on z-buffer */

  glGetIntegerv(GL_RED_BITS, &red_bits);
  glGetIntegerv(GL_GREEN_BITS, &green_bits);
  glGetIntegerv(GL_BLUE_BITS, &blue_bits);
  glGetIntegerv(GL_VIEWPORT, &viewport.x);
  printf("OpenGL window has %d bits red, %d green, %d blue; viewport is %dx%d\n",
    red_bits, green_bits, blue_bits, viewport.width, viewport.height);

  /* setup perspective camera with OpenGL */
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0,(double) viewport.width/viewport.height, 0.01, 200.0); 

  /* from here on we're setting modeling transformations */
  glMatrixMode(GL_MODELVIEW);

  //Move away from center
  glTranslatef(0.0, 0.0, -5.0);

  camera.zoom = 1;

  camera.tw = 0;
  camera.el = -15;
  camera.az = -25;

  camera.atx = 0;
  camera.aty = 0;
  camera.atz = 0;

  // two white lights
  GLfloat light_Ka[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat light_Kd[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat light_Ks[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat light0_pos[] = { 0.0, groundPlaneLightHeight, 0.0, 0.0 };
  // GLfloat light0_pos[] = { -50.0, 50.0, 30.0, 0.0 };
  GLfloat light1_pos[] = {  1.0, -1.0, 0.0, 0.0 };

  // lights
  glLightfv(GL_LIGHT0, GL_AMBIENT,  light_Ka);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_Kd);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_Ks);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);

  glLightfv(GL_LIGHT1, GL_AMBIENT,  light_Ka);
  glLightfv(GL_LIGHT1, GL_DIFFUSE,  light_Kd);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light_Ks);
  glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);

  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_NORMALIZE);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, true);

  // screen buffer
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
  glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);

  //&groundPlaneHeight, &groundPlaneLightHeight, &groundPlaneSize, &groundPlaneR, &groundPlaneG, &groundPlaneB, &groundPlaneAmbient, &groundPlaneDiffuse, &groundPlaneSpecular, &groundPlaneShininess;
  // 0.0,180.0,150.0,r0.81,g0.81,b0.55,a0.1,d0.4,s0.1,sh120.0
//    clock_t timeValue=clock();
//    GLfloat coef1 = (sin(timeValue) / 2) + 0.5;
//    GLfloat coef2 = (cos(timeValue) / 2) + 0.5;
//    GLfloat coef3 = (sin(timeValue)+cos(timeValue) / 4) + 0.5;
  double groundPlaneHeight = 0.0;
  double groundPlaneSize = 200.0;
  double groundPlaneR = 0.81;
  double groundPlaneG = 0.81;
  double groundPlaneB = 0.55;
//    double groundPlaneR = coef1;
//    double groundPlaneG = coef2;
//    double groundPlaneB = coef3;
  double groundPlaneAmbient = 0.1;
  double groundPlaneDiffuse = 0.9;
  double groundPlaneSpecular = 0.1;
  double groundPlaneShininess = 120.0;
  displayListGround = glGenLists(1);
  glNewList(displayListGround, GL_COMPILE);
  RenderGroundPlane(groundPlaneSize, groundPlaneHeight, groundPlaneR, groundPlaneG, groundPlaneB, groundPlaneAmbient, groundPlaneDiffuse, groundPlaneSpecular, groundPlaneShininess);
  glEndList();
}

/*
* Define the methods for glwindow, a subset of Fl_Gl_Window. 
* Handle keyboard and mouse events.  Don't make any OpenGL calls here;
* the GL Context is not set!  Make the calls in Redisplay() and call
* the redraw() method to cause FLTK to set up the context and call draw().
* See the FLTK documentation under "Using OpenGL in FLTK" for additional
* tricks and tips.
*/
int Player_Gl_Window::handle(int event) 
{
  int handled = 1;
  static int prev_x, prev_y;
  int delta_x=0, delta_y=0; 
  //float ev_x, ev_y;

  switch(event) {
  case FL_RELEASE:
    mouse.x = (Fl::event_x());
    mouse.y = (Fl::event_y());
    mouse.button = 0;
    break;
  case FL_PUSH:
    mouse.x = (Fl::event_x());
    mouse.y = (Fl::event_y());
    mouse.button = (Fl::event_button());
    break;
  case FL_DRAG:
    mouse.x = (Fl::event_x());
    mouse.y = (Fl::event_y());
    delta_x=mouse.x-prev_x;
    delta_y=mouse.y-prev_y; 

    if(mouse.button == 3)
    { 
      if(abs(delta_x) > abs(delta_y))
        camera.az += (GLdouble) (delta_x);          
      else
        camera.el -= (GLdouble) (delta_y);
    }
    else if(mouse.button == 2)
    {
      if(abs(delta_y) > abs(delta_x))
      {
        glScalef(float(1+delta_y/100.0), float(1+delta_y/100.0), float(1+delta_y/100.0));	
        //     camera.zoom -= (GLdouble) delta_y/100.0;
        //     if(camera.zoom < 0.) camera.zoom = 0;
      }
    }
    else 
      if(mouse.button == 1)
      {
        camera.tx += (GLdouble) cos( camera.az / 180.0 * M_PI) * delta_x / 10.0;
        camera.tz += (GLdouble) sin( camera.az / 180.0 * M_PI) * delta_x / 10.0;
        camera.ty -= (GLdouble) delta_y / 10.0; //FLTK's origin is at the left_top corner

        camera.atx = -camera.tx;
        camera.aty = -camera.ty;
        camera.atz = -camera.tz;
      }
    break;
  case FL_KEYBOARD:
    switch (Fl::event_key()) 
    {
    case 'q':
    case 'Q':
    case 65307:
      exit(0);
    }
    break;
  default:
    // pass other events to the base class...
    handled= Fl_Gl_Window::handle(event);
  }

  prev_x=mouse.x;
  prev_y=mouse.y;
  glwindow->redraw();

  return (handled);  // Returning one acknowledges that we handled this event
}

//Pre-written Draw Function
void Player_Gl_Window::draw() 
{
  // Upon setup of the window (or when Fl_Gl_Window->invalidate is called), 
  // the set of functions inside the if block are executed.
  if (!valid()) 
  {
    GraphicsInit();   
}

  // Redisplay the screen then put the proper buffer on the screen.
  Redisplay();
}

int main(int argc, char **argv) 
{
  // Initialize form, sliders and buttons
  form = make_window();

  performanceCounter.StartCounter();  // init
  saveFileTimeCounter.StartCounter(); // init

  groundPlane_button->value(groundPlane);
  fog_button->value(useFog);
  worldAxes_button->value(renderWorldAxes);
  frame_slider->value(1);
  if (saveScreenToFile == SAVE_CONTINUOUS)
    record_button->value(1);  // ON
  else
    record_button->value(0);  // OFF

  // just do some timing, no special purpose
  // because the first data is always not trustable according to experience
  performanceCounter.StopCounter(); 
  performanceCounter.GetElapsedTime();
  saveFileTimeCounter.StopCounter();
  saveFileTimeCounter.GetElapsedTime();
  performanceCounter.StartCounter();
  // show form, and do initial draw of model
  form->show();
  glwindow->show(); // glwindow is initialized when the form is built
  performanceCounter.StopCounter();

  if (argc > 2)
  {
    char *filename;

    filename = argv[1];
    if(filename != NULL)
    {
      //Read skeleton from asf file
      pSkeleton = new Skeleton(filename, MOCAP_SCALE);

      //Set the rotations for all bones in their local coordinate system to 0
      //Set root position to (0, 0, 0)
      pSkeleton->setBasePosture();
      displayer.LoadSkeleton(pSkeleton);
      lastSkeleton++;
    }

    if (displayer.GetNumSkeletons())
    {
      filename = argv[2];
      if(filename != NULL)
      {
        //Read motion (.amc) file and create a motion
        pMotion = new Motion(filename, MOCAP_SCALE,pSkeleton);

        //set sampled motion for display
        displayer.LoadMotion(pMotion);               
        
        lastMotion++;

        //Tell skeleton to perform the first pose ( first posture )
        pSkeleton->setPosture(*(displayer.GetSkeletonMotion(0)->GetPosture(0)));          

        // Set skeleton to perform the first pose ( first posture )         
        int currentFrames = displayer.GetSkeletonMotion(0)->GetNumFrames();
        if (currentFrames > maxFrames)
        {
          maxFrames = currentFrames;
          frame_slider->maximum((double)maxFrames);

        }
        frame_slider->maximum((double)maxFrames);

        currentFrameIndex=0;
      } // if(filename != NULL)
    }
    else
      printf("Load a skeleton first.\n");
    framesIncrementDoublePrecision = 1.0;            // Current frame and frame increment
    playButton = ON;
    repeatButton = OFF;
    groundPlane = ON; 
    glwindow->redraw();
  }  // if (argc > 2)
  Fl::add_idle(idle);
  return Fl::run();
}

