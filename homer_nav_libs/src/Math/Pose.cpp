/*******************************************************************************
 *  Pose.cpp
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  $Id: Pose.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include <cmath>

#include <homer_nav_libs/Math/Pose.h>
#include <homer_nav_libs/Math/Transformation2D.h>

using namespace std;


#define THIS Pose

THIS::THIS(float x, float y, float theta) : Point2D(x, y) {
  m_Theta = theta;
}

THIS::THIS() {
  m_Theta = 0.0;
}

THIS::~THIS() {
}

float THIS::theta() const {
  return m_Theta;
}

void THIS::setTheta(float theta) {
  m_Theta = theta;
}

Pose THIS::operator+ ( const Transformation2D& transformation ) const {
  float x, y, theta;
  x = m_X + transformation.x();
  y = m_Y + transformation.y();
  theta = m_Theta + transformation.theta();
  while (theta >= M_PI) theta -= 2*M_PI;
  while (theta < -M_PI) theta += 2*M_PI;

	return Pose(x, y, theta);
}

Pose THIS::operator- ( const Transformation2D& transformation ) const {
  float x, y, theta;
  x = m_X - transformation.x();
  y = m_Y - transformation.y();
  theta = m_Theta - transformation.theta();
  while (theta >= M_PI) theta -= 2*M_PI;
  while (theta < -M_PI) theta += 2*M_PI;

	return Pose(x, y, theta);
}

Transformation2D THIS::operator- ( const Pose& pose ) const {
  float x, y, theta;
  x = m_X - pose.x();
  y = m_Y - pose.y();

  float s1, s2;
  if (m_Theta > pose.theta()) {
    s1 = -( 2 * M_PI - m_Theta + pose.theta());
    s2 = m_Theta - pose.theta();
  } else {
    s1 = 2 * M_PI - pose.theta() + m_Theta;
    s2 = -(pose.theta() - m_Theta);
  }
  if (fabs(s1) > fabs(s2)) {
    theta = s2;
  } else {
    theta = s1;
  }
  while (theta >= M_PI) theta -= 2*M_PI;
  while (theta < -M_PI) theta += 2*M_PI;

	return Transformation2D(x, y, theta);
}

Pose THIS::interpolate(const Pose& referencePose, float t) const {

  float newX = m_X + t * (referencePose.x() - m_X);
  float newY = m_Y + t * (referencePose.y() - m_Y);

  // Calculate mean angle by adding the vem_Thetaors in unit circle
  float x1 = cosf(m_Theta);
  float y1 = sinf(m_Theta);
  float x2 = cosf(referencePose.theta());
  float y2 = sinf(referencePose.theta());
  float newTheta = atan2 (y1*(1-t)+y2*t, x1*(1-t)+x2*t);

  return Pose(newX, newY, newTheta);
}

//THIS::THIS( ExtendedInStream& extStrm )
//{
//  char version;
//  extStrm >> version;
//  extStrm >> m_X;
//  extStrm >> m_Y;
//  extStrm >> m_Theta;
//}

//void THIS::storer( ExtendedOutStream& extStrm ) const
//{
//  char version=10;
//  extStrm << version;
//  extStrm << m_X;
//  extStrm << m_Y;
//  extStrm << m_Theta;
//}

