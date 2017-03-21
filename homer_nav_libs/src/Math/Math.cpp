/*******************************************************************************
 *  Math.cpp
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#include <homer_nav_libs/Math/Math.h>
#include <limits.h>
#include <math.h>

#include <homer_nav_libs/Math/vec2.h>

#define THIS Math

THIS::THIS()
{
}

THIS::~THIS()
{
}

float THIS::meanAngle(const std::vector<float>& angles)
{
  // calculate vectors from angles
  CVec2 vectorSum(0, 0);
  for (unsigned i = 0; i < angles.size(); i++)
  {
    vectorSum = vectorSum + CVec2(cos(angles[i]), sin(angles[i]));
  }
  // return vectorSum.getAngle( CVec2(1,0) );
  if (vectorSum.magnitude() == 0)
  {
    return 0;
  }
  return atan2(vectorSum.y(), vectorSum.x());
}

float THIS::meanAngleWeighted(const std::vector<WeightedValue>& weightedAngles)
{
  // calculate vectors from angles
  CVec2 vectorSum(0, 0);
  for (unsigned i = 0; i < weightedAngles.size(); i++)
  {
    vectorSum = vectorSum +
                weightedAngles[i].weight * CVec2(cos(weightedAngles[i].value),
                                                 sin(weightedAngles[i].value));
  }
  // return vectorSum.getAngle( CVec2(1,0) );
  if (vectorSum.magnitude() == 0)
  {
    return 0;
  }
  return atan2(vectorSum.y(), vectorSum.x());
}

float THIS::angleVariance(float meanAngle, const std::vector<float>& angles)
{
  float quadSum = 0;
  for (unsigned i = 0; i < angles.size(); i++)
  {
    float turnAngle = minTurnAngle(angles[i], meanAngle);
    quadSum += turnAngle * turnAngle;
  }
  return quadSum / float(angles.size());
}

float THIS::minTurnAngle(float angle1, float angle2)
{
  /*  CVec2 vector1( cos( angle1 ), sin ( angle1 ) );
    CVec2 vector2( cos( angle2 ), sin ( angle2 ) );
    return vector1.getAngle( vector2 );
    */
  angle1 *= 180.0 / M_PI;
  angle2 *= 180.0 / M_PI;
  // if(angle1 < 0) angle1 += M_PI * 2;
  // if(angle2 < 0) angle2 += M_PI * 2;
  int diff = angle2 - angle1;
  diff = (diff + 180) % 360 - 180;

  // float sign=1;
  // if ( diff < 0 ) { sign=-1; }
  // minimal turn angle:
  // if the absolute difference is above 180°, calculate the difference in other
  // direction
  // if ( fabs(diff) > M_PI ) {
  //  diff = 2*M_PI - fabs(diff);
  //  diff *= sign;
  //}

  float ret = static_cast<double>(diff) * M_PI / 180.0;
  return ret;
}

Point2D THIS::center(std::vector<Point2D>& points)
{
  double numPoints = double(points.size());
  double sumX = 0, sumY = 0;
  for (unsigned i = 0; i < points.size(); i++)
  {
    sumX += points[i].x();
    sumY += points[i].y();
  }
  return Point2D(sumX / numPoints, sumY / numPoints);
}

double THIS::randomGauss(float variance)
{
  if (variance < 0)
  {
    variance = -variance;
  }
  double x1, x2, w, y1;
  do
  {
    x1 = 2.0 * random01() - 1.0;
    x2 = 2.0 * random01() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w >= 1.0);

  w = sqrt((-2.0 * log(w)) / w);
  y1 = x1 * w;
  // now y1 is uniformly distributed
  return sqrt(variance) * y1;
}

double THIS::random01(unsigned long init)
{
  static unsigned long n;
  if (init > 0)
  {
    n = init;
  }
  n = 1664525 * n + 1013904223;
  // create double from unsigned long
  return (double)(n / 2) / (double)LONG_MAX;
}

#undef THIS
