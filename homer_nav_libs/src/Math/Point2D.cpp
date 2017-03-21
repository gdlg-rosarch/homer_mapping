/*******************************************************************************
 *  Point2D.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include <homer_nav_libs/Math/Point2D.h>

#define THIS Point2D

float THIS::getPolarAngle() const
{
  float angle = atan(m_Y / m_X);
  if (m_X < 0)
  {
    angle = -(M_PI - angle);
  }
  while (angle >= M_PI)
  {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
  return angle;
}

#undef THIS
