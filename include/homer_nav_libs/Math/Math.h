/*******************************************************************************
 *  Math.h
 *
 *  (C) 2007 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: $
 *******************************************************************************/

#ifndef Math_H
#define Math_H

#include <vector>
#include <homer_nav_libs/Math/Point2D.h>

/**
 * @class  Math
 * @brief  Generic math and statistics functions
 * @author David Gossow (RX)
 */
class Math
{
  public:

    struct WeightedValue
    {
      float value;
      float weight;
    };

    static const double Pi = 3.14159265358979323846;

    /** @return mean value */
    template<class ContainerT>
    static double mean ( const ContainerT& values );

    /** @return variance of given values */
    template<class ContainerT>
    static double variance ( const ContainerT& values );

    /** @return mean angle of given values
     *  @note   there are always two possible choices for the mean angle. This function returns the one with the smallest deviation
     *  @note   Works for angles in [-Pi..Pi], negative angles are treated
     */
    static float meanAngle ( const std::vector<float>& angles );

    static float meanAngleWeighted ( const std::vector< WeightedValue >& weightedAngles );

    /** @return variance for given mean */
    static float angleVariance ( float meanAngle, const std::vector<float>& angles );

    /** @return minimal angle needed to turn from angle 1 to angle 2 [-Pi..Pi] */
    static float minTurnAngle ( float angle1, float angle2 );

    static Point2D center ( std::vector<Point2D>& points );

    static float deg2Rad ( float deg ) { return deg / 180.0*Pi; }

    static float rad2Deg ( float rad ) { return rad / Pi*180.0; }

    static double randomGauss ( float variance = 1.0 );

    static double random01 ( unsigned long init = 0 );

    /** @return ratio between one dimension seen under old viewangle and dimension under new viewangle*/
    static double angleToPercent ( double newAngle, double oldAngle ) { return tan ( ( Pi / 180.0 ) * newAngle / 2 ) / tan ( ( Pi / 180.0 ) * oldAngle / 2 ); };

    /** @return angle under which the ratio between dimension seen under old viewangle and new viewangle equals percent*/
    static double percentToAngle ( double percent, double angle ) { return 2* atan ( tan ( ( Pi / 180.0 ) * angle / 2 ) * percent ) * ( 180 / Pi ); };

    /** @return horizontal view angle corresponding to diagonal view angle and aspect ratio (e.g. 4.0/3.0)*/
    static double horizontalViewAngle ( double diagonalAngle, double aspectRatio ) { return verticalViewAngle ( diagonalAngle, 1.0 / aspectRatio ); };

    /** @return vertical view angle corresponding to diagonal view angle and aspect ratio (e.g. 4.0/3.0)*/
    static double verticalViewAngle ( double diagonalAngle, double aspectRatio )
    {
      return percentToAngle ( 1.0 / sqrt ( pow ( aspectRatio, 2 ) + 1.0 ), diagonalAngle );
    };

    template<class ValueT>
    static inline ValueT min ( ValueT a, ValueT b ) { return a < b ? a : b; }

    template<class ValueT>
    static inline ValueT max ( ValueT a, ValueT b ) { return a > b ? a : b; }

  private:

    /** @brief The constructor */
    Math();

    /** @brief The destructor */
    ~Math();

};

template<class ContainerT>
double Math::mean ( const ContainerT& values )
{
  typename ContainerT::const_iterator it;
  it = values.begin();
  double sum = 0;
  while ( it != values.end() )
  {
    sum += *it;
    it++;
  }
  return sum / double ( values.size() );
}


template<class ContainerT>
double Math::variance ( const ContainerT& values )
{
  double mean = mean ( values );
  typename ContainerT::const_iterator it;
  it = values.begin();
  double sum = 0;
  while ( it != values.end() )
  {
    double diff = *it - mean;
    sum += diff * diff;
    it++;
  }
  return sum / double ( values.size() );
}


#endif
