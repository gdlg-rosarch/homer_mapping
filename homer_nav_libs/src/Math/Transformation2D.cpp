/*******************************************************************************
 *  Transformation2D.cpp
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Transformation2D.cpp 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include <homer_nav_libs/Math/Transformation2D.h>

#include <homer_nav_libs/Math/Line2D.h>
#include <homer_nav_libs/Math/Point2D.h>
#include <homer_nav_libs/Math/mat2.h>  // TODO das sieht nach baselib aus ggf. durch baselib ersetzen
#include <homer_nav_libs/Math/vec2.h>  // TODO das sieht nach baselib aus ggf. durch baselib ersetzen
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#define THIS Transformation2D
#define BASE CVec2

THIS::Transformation2D() : BASE()
{
  m_Theta = 0.0;
}

THIS::Transformation2D(double x, double y, double theta) : BASE(x, y)
{
  m_Theta = theta;
}

THIS::Transformation2D(const CVec2& vec, double theta) : BASE(vec)
{
  m_Theta = theta;
}

THIS::~Transformation2D()
{
}

void THIS::set(double x, double y, double theta)
{
  m_X = x;
  m_Y = y;
  m_Theta = theta;
}

double THIS::theta() const
{
  return m_Theta;
}

Transformation2D THIS::operator+(Transformation2D t) const
{
  double theta = m_Theta + t.theta();
  // TODO comment only for scan matching test
  //   while ( theta >= M_PI ) theta -= 2*M_PI;
  //   while ( theta < -M_PI ) theta += 2*M_PI;
  return Transformation2D(m_X + t.x(), m_Y + t.y(), theta);
}

Transformation2D& THIS::operator+=(Transformation2D t)
{
  m_X += t.x();
  m_Y += t.y();
  m_Theta += t.theta();
  // TODO comment only for scan matching test
  //   while ( m_Theta >= M_PI ) m_Theta -= 2*M_PI;
  //   while ( m_Theta < -M_PI ) m_Theta += 2*M_PI;
  return (*this);
}

Transformation2D THIS::operator-(Transformation2D t) const
{
  float s1, s2, theta;
  if (m_Theta > t.theta())
  {
    s1 = -(2 * M_PI - m_Theta + t.theta());
    s2 = m_Theta - t.theta();
  }
  else
  {
    s1 = 2 * M_PI - t.theta() + m_Theta;
    s2 = -(t.theta() - m_Theta);
  }
  if (fabs(s1) > fabs(s2))
  {
    theta = s2;
  }
  else
  {
    theta = s1;
  }
  while (theta >= M_PI)
    theta -= 2 * M_PI;
  while (theta < -M_PI)
    theta += 2 * M_PI;
  //   double theta = m_Theta - t.theta();
  //   while ( theta >= M_PI ) theta -= 2*M_PI;
  //   while ( theta < -M_PI ) theta += 2*M_PI;
  return Transformation2D(m_X - t.x(), m_Y - t.y(), theta);
}

Transformation2D& THIS::operator-=(Transformation2D t)
{
  m_X -= t.x();
  m_Y -= t.y();

  float s1, s2, theta;
  if (m_Theta > t.theta())
  {
    s1 = -(2 * M_PI - m_Theta + t.theta());
    s2 = m_Theta - t.theta();
  }
  else
  {
    s1 = 2 * M_PI - t.theta() + m_Theta;
    s2 = -(t.theta() - m_Theta);
  }
  if (fabs(s1) > fabs(s2))
  {
    theta = s2;
  }
  else
  {
    theta = s1;
  }
  while (theta >= M_PI)
    theta -= 2 * M_PI;
  while (theta < -M_PI)
    theta += 2 * M_PI;
  m_Theta = theta;

  return (*this);

  //   m_X -= t.x();
  //   m_Y -= t.y();
  //   m_Theta -= t.theta();
  //   while ( m_Theta >= M_PI ) m_Theta -= 2*M_PI;
  //   while ( m_Theta < -M_PI ) m_Theta += 2*M_PI;
  //   return ( *this );
}

Transformation2D THIS::operator*(float factor) const
{
  double theta = m_Theta * factor;
  while (theta >= M_PI)
    theta -= 2 * M_PI;
  while (theta < -M_PI)
    theta += 2 * M_PI;
  return Transformation2D(m_X * factor, m_Y * factor, theta);
}

Transformation2D& THIS::operator*=(float factor)
{
  m_X *= factor;
  m_Y *= factor;
  m_Theta *= factor;
  while (m_Theta >= M_PI)
    m_Theta -= 2 * M_PI;
  while (m_Theta < -M_PI)
    m_Theta += 2 * M_PI;
  return (*this);
}

Transformation2D THIS::operator/(float factor) const
{
  double theta = m_Theta / factor;
  return Transformation2D(m_X / factor, m_Y / factor, theta);
}

Transformation2D& THIS::operator/=(float factor)
{
  m_X /= factor;
  m_Y /= factor;
  m_Theta /= factor;
  return (*this);
}

bool THIS::operator==(Transformation2D t) const
{
  if (t.x() == m_X && t.y() == m_Y && t.theta() == m_Theta)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool THIS::operator!=(Transformation2D t) const
{
  return !((*this) == t);
}

bool THIS::operator<=(Transformation2D t) const
{
  return (this->magnitude() <= t.magnitude()) && (m_Theta <= t.theta());
}

bool THIS::operator>=(Transformation2D t) const
{
  return (this->magnitude() >= t.magnitude()) && (m_Theta >= t.theta());
}

bool THIS::operator<(Transformation2D t) const
{
  return (m_X < t.x()) || (m_Y < t.y()) ||
         ((m_Theta < t.theta()) && (*this <= t));
}

bool THIS::operator>(Transformation2D t) const
{
  return (m_X > t.x()) || (m_Y > t.y()) ||
         ((m_Theta > t.theta()) && (*this >= t));
}

Transformation2D THIS::abs() const
{
  return Transformation2D(std::abs(m_X), std::abs(m_Y), std::abs(m_Theta));
}

Transformation2D THIS::inverse() const
{
  return (*this) * (-1.0);
}

Point2D THIS::transform(const Point2D& point) const
{
  CMat2 rotMat = CMat2(m_Theta);
  CVec2 transVec = CVec2(m_X, m_Y);
  Point2D transformedPoint = rotMat * (point);
  transformedPoint += transVec;
  return transformedPoint;
}

std::vector<Point2D> THIS::transform(const std::vector<Point2D>& points) const
{
  CMat2 rotMat = CMat2(m_Theta);
  CVec2 transVec = CVec2(m_X, m_Y);
  std::vector<Point2D> transformedPoints;
  std::vector<Point2D>::const_iterator iter = points.begin();
  while (iter != points.end())
  {
    Point2D currPoint = rotMat * (*iter);
    currPoint += transVec;
    transformedPoints.push_back(currPoint);
    iter++;
  }
  return transformedPoints;
}

// // Reihenfolge rotation/translation vertauscht !!!
// Point2D THIS::transform ( Point2D point ) const
// {
//   CMat2 rotMat = CMat2 ( m_Theta );
//   CVec2 transVec = CVec2 ( m_X, m_Y );
//   Point2D transformedPoint = point+transVec;
//   transformedPoint = rotMat * point;
//   return transformedPoint;
// }
//
// // Reihenfolge rotation/translation vertauscht !!!
// std::vector<Point2D> THIS::transform ( std::vector<Point2D> points ) const
// {
//   CMat2 rotMat = CMat2 ( m_Theta );
//   CVec2 transVec = CVec2 ( m_X, m_Y );
//   std::vector<Point2D> transformedPoints;
//   std::vector<Point2D>::const_iterator iter = points.begin();
//   while ( iter != points.end() )
//   {
//     Point2D currPoint = ( *iter )+ transVec;
//     currPoint = rotMat * currPoint;
//     transformedPoints.push_back ( currPoint );
//     iter++;
//   }
//   return transformedPoints;
// }

Line2D THIS::transform(const Line2D& line) const
{
  CMat2 rotMat = CMat2(m_Theta);
  CVec2 transVec = CVec2(m_X, m_Y);
  Line2D transformedLine =
      Line2D(rotMat * line.start() + transVec, rotMat * line.end() + transVec);
  return transformedLine;
}

std::vector<Line2D> THIS::transform(const std::vector<Line2D>& lines) const
{
  // CMat2 rotMat = CMat2 ( m_Theta );
  // CVec2 transVec = CVec2 ( m_X, m_Y );
  std::vector<Line2D> transformedLines;
  std::vector<Line2D>::const_iterator iter = lines.begin();
  while (iter != lines.end())
  {
    transformedLines.push_back(transform(*iter));
    iter++;
  }
  return transformedLines;
}

std::string THIS::toString() const
{
  std::ostringstream str;
  str << "deltaX: " << m_X << ", deltaY: " << m_Y
      << ", deltaTheta: " << m_Theta;
  return str.str();
}

#undef THIS
#undef BASE
