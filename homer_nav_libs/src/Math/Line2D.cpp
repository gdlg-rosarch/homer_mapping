/*******************************************************************************
 *  Line2D.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: SM; DevelTest: Date; Reviewer: Initials; Review: Date; State: NOK
 *
 *  Additional information:
 *  $Id: Line2D.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include <iostream>
#include <sstream>

#include <homer_nav_libs/Math/Line2D.h>
#include <homer_nav_libs/Math/vec2.h>

#define THIS Line2D

float THIS::gradient() const
{
  float gradient = 10000000.0;
  if ( m_Vec[0] != 0.0 )
  {
    gradient = m_Vec[1]/m_Vec[0];
  }
  return gradient;
}

std::vector< Point2D > THIS::vertices ( unsigned substeps )
{
  unsigned steps = substeps+2;
  std::vector<Point2D> myVertices ( steps );
  for ( unsigned i=0; i<steps; i++ )
  {
    float t= float ( i ) / float ( steps-1 );
    myVertices[i] = m_Start + t*m_Vec;
  }
  return myVertices;
}

Point2D THIS::getClosestPoint ( Point2D point ) const
{
  float t = ( point-m_Start ) * m_Vec;
  t /= m_Vec * m_Vec;
  if ( t > 1.0 )
  {
    t = 1.0;
  }
  else if ( t < 0.0 )
  {
    t = 0.0;
  }
  Point2D pointOnLine = m_Start + ( t * m_Vec );
  return pointOnLine;
}

Point2D THIS::getIntersectionPoint ( Line2D line ) const
{
  Point2D intersecPoint;
  double det1 = m_Vec.x() * ( -line.vec().y() ) - ( -line.vec().x() ) * m_Vec.y();
  // lines are not parallel
  if ( det1 != 0 )
  {
    CVec2 startToStart = line.start() -m_Start;
    // calculate intersection
    double lambda = ( startToStart.x() * ( -line.vec().y() ) - ( -line.vec().x() ) * startToStart.y() ) / det1;
    intersecPoint = m_Start + lambda* m_Vec;
  }

  return intersecPoint;
}

float THIS::getIntersectionPointParameter ( Line2D line ) const
{
  double lambda = 0.0;
  double det1 = m_Vec.x() * ( -line.vec().y() ) - ( -line.vec().x() ) * m_Vec.y();
  // lines are not parallel
  if ( det1 != 0 )
  {
    CVec2 startToStart = line.start() -m_Start;
    // calculate intersection
    lambda = ( startToStart.x() * ( -line.vec().y() ) - ( -line.vec().x() ) * startToStart.y() ) / det1;
  }

  return lambda;
}

std::string THIS::toString() const
{
  std::ostringstream str;
//   str << "Startpoint: " << m_Start.x() << " " << m_Start.y() << " Endpoint: " << end().x() << " " << end().y() <<
//   " Vector: " << m_Vec.x() << " " << m_Vec.y() << " ";
  str << m_Start.x() << " " << m_Start.y() << std::endl << end().x() << " " << end().y() << std::endl;
  return str.str();
}

