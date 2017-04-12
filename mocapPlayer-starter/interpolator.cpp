#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include <fstream>
#include <time.h>
using namespace std;

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
//    ofstream graph1("graph1_le.txt");
//    ofstream graph3("graph3_le.txt");
      ofstream times("LinearEuler_time.txt");
    clock_t  start=clock();
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    clock_t end=clock();
    times<<end-start<<endl;

//    for(int i=600;i<=800;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph1<<g1->bone_rotation[2].p[0]<<endl;
//
//    }
//    for(int i=200;i<=500;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph3<<g1->bone_rotation[0].p[2]<<endl;
//    }
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // students should implement this
    for(int i=0;i<3;i++)angles[i]/=(180/M_PI);
    R[0]=cos(angles[1])*cos(angles[2]);
    R[1]=sin(angles[0])*sin(angles[1])*cos(angles[2])-cos(angles[0])*sin(angles[2]);
    R[2]=sin(angles[0])*sin(angles[2])+cos(angles[0])*sin(angles[1])*cos(angles[2]);
    R[3]=cos(angles[1])*sin(angles[2]);
    R[4]=cos(angles[0])*cos(angles[2])+sin(angles[0])*sin(angles[1])*sin(angles[2]);
    R[5]=cos(angles[0])*sin(angles[1])*sin(angles[2])-sin(angles[0])*cos(angles[2]);
    R[6]=-sin(angles[1]);
    R[7]=sin(angles[0])*cos(angles[1]);
    R[8]=cos(angles[0])*cos(angles[1]);
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
//    ofstream graph1_("graph1_be.txt");
//    ofstream graph1("graph1_input.txt");
//    ofstream graph3_("graph3_be.txt");
//    ofstream graph3("graph3_input.txt");
    ofstream times("BezierEuler_time.txt");
    clock_t start=clock();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0;
    int previousKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        int thirdKeyframe = startKeyframe + 2 * N + 2;

        Posture * previousPosture = pInputMotion->GetPosture(previousKeyframe);
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        Posture * thirdPosture;
        if(thirdKeyframe<inputLength)thirdPosture = pInputMotion->GetPosture(thirdKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);

           // interpolate root position
            vector an_root,bn_root,middle_root,an_hat1_root,an_hat2_root;
            vector previousRoot=previousPosture->root_pos;
            vector startRoot=startPosture->root_pos;
            vector endRoot=endPosture->root_pos;
            vector thirdRoot;
            if(thirdKeyframe<inputLength)thirdRoot=thirdPosture->root_pos;
            if(startKeyframe==0)
            {
                middle_root=endRoot*2-thirdRoot;
                an_root=startRoot*(1.0-1.0/3)+middle_root*(1.0/3);
            }
            else
            {
                middle_root=startRoot*2-previousRoot;
                an_hat1_root=middle_root*0.5+endRoot*0.5;
                an_root=startRoot*(1.0-1.0/3)+an_hat1_root*(1.0/3);
            }
            if(thirdKeyframe>inputLength)
            {
                middle_root=startRoot*2-previousRoot;
                bn_root=endRoot*(1.0-1.0/3)+middle_root*(1.0/3);
            }
            else
            {
                middle_root=endRoot*2.0-startRoot;
                an_hat2_root=middle_root*0.5+thirdRoot*0.5;
                bn_root=endRoot*(1+1.0/3)-an_hat2_root*(1.0/3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t,startRoot,an_root,bn_root,endRoot);
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {

                vector angles0,angles1,angles2,angles3,angles;
                vector middle,an_hat1,an_hat2,an,bn_,an_;
                angles0=(previousPosture->bone_rotation)[bone];
                angles1=(startPosture->bone_rotation)[bone];
                angles2=(endPosture->bone_rotation)[bone];

                if(thirdKeyframe<inputLength)
                {
                    angles3 = (thirdPosture->bone_rotation)[bone];
                }
                //calculate an,bn_
                if(startKeyframe==0)
                {
                    middle=angles2*2.0-angles3;
                    an=angles1*(1.0-1.0/3)+middle*(1.0/3);
                }
                else
                {
                    middle = angles1*2.0-angles0;
                    an_hat1=middle*0.5+angles2*0.5;
                    an=angles1*(1.0-1.0/3)+an_hat1*(1.0/3);
                }
                if(thirdKeyframe>inputLength)
                {
                    middle = angles1*2.0-angles0;
                    bn_=angles2*(1.0-1.0/3)+middle*(1.0/3);
                }
                else
                {
                    middle=angles2*2.0-angles1;
                    an_hat2=middle*0.5+angles3*0.5;
                    bn_=angles2*(1.0+1.0/3)-an_hat2*(1.0/3);
                }
                angles=DeCasteljauEuler(t,angles1,an,bn_,angles2);

                interpolatedPosture.bone_rotation[bone]=angles;
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
            previousKeyframe = startKeyframe;
            startKeyframe = endKeyframe;
        }
        for(int frame=startKeyframe+1; frame<inputLength; frame++)
         pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
        clock_t end=clock();
        times<<end-start<<endl;
//    for(int i=600;i<=800;i++)
//    {
//        Posture* g1;
//        Posture* g2;
//        g1=pInputMotion->GetPosture(i);
//        g2=pOutputMotion->GetPosture(i);
//        graph1<<g1->bone_rotation[2].p[0]<<endl;
//        graph1_<<g2->bone_rotation[2].p[0]<<endl;
//    }
//    for(int i=200;i<=500;i++)
//    {
//        Posture* g1;
//        Posture* g2;
//        g1=pInputMotion->GetPosture(i);
//        g2=pOutputMotion->GetPosture(i);
//        graph3<<g1->bone_rotation[0].p[2]<<endl;
//        graph3_<<g2->bone_rotation[0].p[2]<<endl;
//    }
}


void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
//    ofstream graph1("graph1_lq.txt");
//    ofstream graph3("graph3_lq.txt");
    ofstream times("LinearQuaternion_time.txt");
    clock_t start=clock();
    int inputLength=pInputMotion->GetNumFrames();
    int startKeyframe = 0;
    while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Quaternion<double> bone_rotate1;
        Quaternion<double> bone_rotate2;
        Quaternion<double> bone_con;
        double angles1[3];
        double angles2[3];
        double angles[3];
        angles1[0]=((startPosture->bone_rotation)[bone]).p[0];
        angles1[1]=((startPosture->bone_rotation)[bone]).p[1];
        angles1[2]=((startPosture->bone_rotation)[bone]).p[2];
        Euler2Quaternion(angles1, bone_rotate1);
        
        angles2[0]=((endPosture->bone_rotation)[bone]).p[0];
        angles2[1]=((endPosture->bone_rotation)[bone]).p[1];
        angles2[2]=((endPosture->bone_rotation)[bone]).p[2];

        Euler2Quaternion(angles2, bone_rotate2);
          bone_con=Slerp(t,bone_rotate1,bone_rotate2);
          bone_con.Normalize();
        Quaternion2Euler(bone_con,angles);

        interpolatedPosture.bone_rotation[bone].p[0]=angles[0];
        interpolatedPosture.bone_rotation[bone].p[1]=angles[1];
        interpolatedPosture.bone_rotation[bone].p[2]=angles[2];

      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));

//    for(int i=600;i<=800;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph1<<g1->bone_rotation[2].p[0]<<endl;
//
//    }
//    for(int i=200;i<=500;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph3<<g1->bone_rotation[0].p[2]<<endl;
//    }
    clock_t end=clock();
    times<<end-start<<endl;
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
//    ofstream graph1("graph1_bq.txt");
//    ofstream graph3("graph3_bq.txt");
    ofstream times("BezierQuaternion.txt");
    clock_t start=clock();
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    int startKeyframe = 0;
    int previousKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        int thirdKeyframe = startKeyframe + 2 * N + 2;

        Posture * previousPosture = pInputMotion->GetPosture(previousKeyframe);
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        Posture * thirdPosture;
        if(thirdKeyframe<inputLength)thirdPosture = pInputMotion->GetPosture(thirdKeyframe);

        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);

        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);


            // interpolate root position
            vector an_root,bn_root,middle_root,an_hat1_root,an_hat2_root;
            vector previousRoot=previousPosture->root_pos;
            vector startRoot=startPosture->root_pos;
            vector endRoot=endPosture->root_pos;
            vector thirdRoot;
            if(thirdKeyframe<inputLength)thirdRoot=thirdPosture->root_pos;
            if(startKeyframe==0)
            {
                middle_root=endRoot*2-thirdRoot;
                an_root=startRoot*(1.0-1.0/3)+middle_root*(1.0/3);
            }
            else
            {
                middle_root=startRoot*2-previousRoot;
                an_hat1_root=middle_root*0.5+endRoot*0.5;
                an_root=startRoot*(1.0-1.0/3)+an_hat1_root*(1.0/3);
            }
            if(thirdKeyframe>inputLength)
            {
                middle_root=startRoot*2-previousRoot;
                bn_root=endRoot*(1.0-1.0/3)+middle_root*(1.0/3);
            }
            else
            {
                middle_root=endRoot*2.0-startRoot;
                an_hat2_root=middle_root*0.5+thirdRoot*0.5;
                bn_root=endRoot*(1+1.0/3)-an_hat2_root*(1.0/3);
            }
            interpolatedPosture.root_pos = DeCasteljauEuler(t,startRoot,an_root,bn_root,endRoot);

            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                Quaternion<double> bone_rotate1;
                Quaternion<double> bone_rotate2;
                Quaternion<double> bone_rotate0;
                Quaternion<double> bone_rotate3;
                Quaternion<double> bone_con;
                Quaternion<double> an,bn_;
                double angles1[3];
                double angles2[3];
                double angles0[3];
                double angles3[3];
                double angles[3];

                angles0[0]=((previousPosture->bone_rotation)[bone]).p[0];
                angles0[1]=((previousPosture->bone_rotation)[bone]).p[1];
                angles0[2]=((previousPosture->bone_rotation)[bone]).p[2];
                Euler2Quaternion(angles0, bone_rotate0);

                angles1[0]=((startPosture->bone_rotation)[bone]).p[0];
                angles1[1]=((startPosture->bone_rotation)[bone]).p[1];
                angles1[2]=((startPosture->bone_rotation)[bone]).p[2];
                Euler2Quaternion(angles1, bone_rotate1);

                angles2[0]=((endPosture->bone_rotation)[bone]).p[0];
                angles2[1]=((endPosture->bone_rotation)[bone]).p[1];
                angles2[2]=((endPosture->bone_rotation)[bone]).p[2];
                Euler2Quaternion(angles2, bone_rotate2);

                if(thirdKeyframe<inputLength)
                {
                    angles3[0] = ((thirdPosture->bone_rotation)[bone]).p[0];
                    angles3[1] = ((thirdPosture->bone_rotation)[bone]).p[1];
                    angles3[2] = ((thirdPosture->bone_rotation)[bone]).p[2];
                    Euler2Quaternion(angles3, bone_rotate3);
                }

                //calculate an,bn_
                Quaternion<double> middle;
                Quaternion<double> an_hat1, an_hat2,an_;
                if(startKeyframe==0)
                {
                    middle=Slerp(2.0,bone_rotate3,bone_rotate2);
                    an=Slerp(1.0/3,bone_rotate1,middle);
                }
                else
                {
                    middle=Slerp(2.0,bone_rotate0,bone_rotate1);
                    an_hat1=Slerp(0.5,middle,bone_rotate2);
                    an=Slerp(1.0/3,bone_rotate1,an_hat1);
                }
                if(thirdKeyframe>inputLength)
                {
                    middle = Slerp(2.0,bone_rotate0,bone_rotate1);
                    bn_=Slerp(1.0/3,bone_rotate2,middle);
                }
                else
                {
                    middle=Slerp(2.0,bone_rotate1,bone_rotate2);
                    an_hat2=Slerp(0.5,middle,bone_rotate3);
                    an_=Slerp(1.0/3,bone_rotate2,middle);
                    bn_=Slerp(-1.0/3,bone_rotate2,an_hat2);
                }
                bone_con=DeCasteljauQuaternion(t,bone_rotate1,an,bn_,bone_rotate2);
                Quaternion2Euler(bone_con,angles);
                interpolatedPosture.bone_rotation[bone].p[0]=angles[0];
                interpolatedPosture.bone_rotation[bone].p[1]=angles[1];
                interpolatedPosture.bone_rotation[bone].p[2]=angles[2];
            }

            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        previousKeyframe = startKeyframe;
        startKeyframe = endKeyframe;
    }

    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    clock_t end=clock();
    times<<end-start<<endl;
//    for(int i=600;i<=800;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph1<<g1->bone_rotation[2].p[0]<<endl;
//
//    }
//    for(int i=200;i<=500;i++)
//    {
//        Posture* g1;
//        g1=pOutputMotion->GetPosture(i);
//        graph3<<g1->bone_rotation[0].p[2]<<endl;
//    }
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
    double R[9];
    Euler2Rotation(angles,R);
    q=q.Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
    double R[9];
    q.Quaternion2Matrix(R);
    Rotation2Euler(R,angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result,mid;
  double dot,s,x,y,z;
  const double threshold=0.9995;
  double theta,theta_0;
  qStart.Normalize();
  qEnd_.Normalize();
  dot=qStart.Gets()*qEnd_.Gets()+qStart.Getx()*qEnd_.Getx()+qStart.Gety()*qEnd_.Gety()+qStart.Getz()*qEnd_.Getz();
  if(dot>threshold)
  {
        s=qStart.Gets()*(1-t)+qEnd_.Gets()*t;
        x=qStart.Getx()*(1-t)+qEnd_.Getx()*t;
        y=qStart.Gety()*(1-t)+qEnd_.Gety()*t;
        z=qStart.Getz()*(1-t)+qEnd_.Getz()*t;
        result.Set(s,x,y,z);
        result.Normalize();
        return result;
  }
  if(dot<0.0f)
  {
    qEnd_.Set(-qEnd_.Gets(),-qEnd_.Getx(),-qEnd_.Gety(),-qEnd_.Getz());
    dot=-dot;
  }
  if(dot>1.0)dot=1.0;
  theta_0=acos(dot);
  theta=theta_0*t;
  mid.Set(qEnd_.Gets()-qStart.Gets()*dot,qEnd_.Getx()-qStart.Getx()*dot,qEnd_.Gety()-qStart.Gety()*dot,qEnd_.Getz()-qStart.Getz()*dot);
  mid.Normalize();
  result.Set(qStart.Gets()*cos(theta)+mid.Gets()*sin(theta),
             qStart.Getx()*cos(theta)+mid.Getx()*sin(theta),
             qStart.Gety()*cos(theta)+mid.Gety()*sin(theta),
             qStart.Getz()*cos(theta)+mid.Getz()*sin(theta));
  result.Normalize();
  return result;

}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
   Quaternion<double> part1;
    double pq;
    pq=p.Gets()*q.Gets()+p.Getx()*q.Getx()+p.Gety()*q.Gety()+p.Getz()+q.Getz();
    pq*=2;
    part1.Set(q.Gets()*pq,q.Getx()*pq,q.Gety()*pq,q.Getz()*pq);
    result=part1.operator-(p);
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
    vector Q0,Q1,Q2,R0,R1;
    for(int i=0;i<3;i++)
    {
        Q0[i]=p0[i]*(1-t)+p1[i]*t;
        Q1[i]=p1[i]*(1-t)+p2[i]*t;
        Q2[i]=p2[i]*(1-t)+p3[i]*t;
        R0[i]=Q0[i]*(1-t)+Q1[i]*t;
        R1[i]=Q1[i]*(1-t)+Q2[i]*t;
        result[i]=R0[i]*(1-t)+R1[i]*t;
    }
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
    Quaternion<double> Q0,Q1,Q2,R0,R1;
    Q0=Slerp(t,p0,p1);
    Q1=Slerp(t,p1,p2);
    Q2=Slerp(t,p2,p3);
    R0=Slerp(t,Q0,Q1);
    R1=Slerp(t,Q1,Q2);
    result=Slerp(t,R0,R1);
    result.Normalize();
  return result;
}

