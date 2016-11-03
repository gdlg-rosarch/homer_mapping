/*******************************************************************************
 *  Line2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Information on Code Review state:
 *  Author: SM; DevelTest: Date; Reviewer: Initials; Review: Date; State: NOK
 *
 *  Additional information:
 *  $Id: Line2D.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef LINE2D_H
#define LINE2D_H

#include <homer_nav_libs/Math/vec2.h>
#include <homer_nav_libs/Math/mat2.h>
#include <homer_nav_libs/Math/Point2D.h>
#include <vector>

/**
 * @class Line2D
 *
 * @author Susanne Maur
 *
 */
class Line2D
{

  public:

    /**
     * Creates a new line.
     * @param start Start point of the line.
     * @param end End point of the line.
     */
    inline Line2D ( Point2D start, Point2D end )
    {
      m_Start = start;
      m_Vec = end-m_Start;
    }


    /**
      * Creates a new line.
      * @param start Start point of the line.
      * @param vec The vector from start to end point of the line.
      */
    inline Line2D ( Point2D start,  CVec2 vec )
    {
      m_Start = start;
      m_Vec = vec;
    }

    /**
     * Destructor does nothing.
     */
    inline ~Line2D() {}

    /**
     * Sets the start point of the line to a new value.
     * @param start Start point of the line.
     */
    inline void setStart ( const Point2D start )
    {
      Point2D end = m_Start + m_Vec;
      m_Start = start;
      m_Vec = end-m_Start;
    }

    /**
     * Sets the end point of the line to a new value.
     * @param end End point of the line.
     */

    inline void setEnd ( const Point2D end )
    {
      m_Vec = end - m_Start;
    }

    /**
     * Returns the start point of the line.
     * @return Start point of the line.
     */
    inline Point2D start() const
    {
      return m_Start;
    }

    /**
     * Returns the end point of the line.
     * @return End point of the line.
     */
    inline Point2D end() const
    {
      return m_Start + m_Vec;
    }

    /**
     * Returns a vector from the start to the end of the line.
     * @return Vector to the end point of the line.
     */
    inline CVec2 vec() const
    {
      return m_Vec;
    }

    inline bool operator== ( const Line2D& line ) const
    {
      return ( m_Start == line.start() && end() == line.end() );
    }

    /**
     * Returns the gradient of the line.
     * @return Gradient of the line.
     */
    float gradient() const ;

    /**
    * Returns the length of the line.
    * @return Length of the line.
     */
    inline float length() const {
      return m_Vec.magnitude();
    }

    /**
     * Returns the minimum  euclidean distance of the given point to the line.
     * @param point Point of which the distance to the line will be calculated.
     * @return Distance of point to line.
     */
    inline float distance ( Point2D point ) const
    {
      Point2D pointOnLine = getRootPoint ( point );
      return ( point - pointOnLine ).magnitude();
    }

    /**
     * Rotates the line round the origin.
     * @param angle The angle of rotation in radiants.
     */
    inline void rotate ( float angle )
    {
      CMat2 rotMat = CMat2 ( angle );
      m_Start = rotMat * m_Start;
      m_Vec = rotMat * m_Vec;
    }

    /**
     * Returns the point of the line with the minimal distance to a given point.
     * This algorithm may return a point which lies on the extension of the line and not on itself.
     * See also: getClosestPoint.
     * @param point Point to which the distance is calculated.
     * @return Root point of the line.
     */
    inline Point2D getRootPoint ( Point2D point ) const
    {
      float t = ( point-m_Start ) * m_Vec;
      t /= m_Vec * m_Vec;
      Point2D pointOnLine = m_Start + ( t * m_Vec );
      return pointOnLine;
    }

    /**
     * Returns the normal of the line.
     * @return Normal of the line.
     */
    inline CVec2 getNormal() const {
      return m_Vec.getNormal()/m_Vec.magnitude();
    }

    /**
    * Returns the point of the line with the minimal distance to a given point.
    * This algorithm returns always a point which lies on the line.
    * Therefor it is not always the root point.
    * See also: getRootPoint.
    * @param point Point to which the distance is calculated.
    * @return Root point of the line.
     */
    Point2D getClosestPoint ( Point2D point ) const;

    /**
    * Returns the intersection point of this line with a second line.
    * The intersection point is element of this line.
    * @param line The line with which the intersection is calculated.
    * @return Intersection point.
     */
    Point2D getIntersectionPoint ( Line2D line ) const;

    /**
     * Returns the parameter t which identifies the intersection point of this line with a second line.
     * The intersection point is element of this line.
     * @param line The line with which the intersection is calculated.
     * @return Parameter t which identifies the intersection point on the line.
     */
    float getIntersectionPointParameter ( Line2D line ) const;

    /**
     * @return Vertices, e.g. for use in a VectorObject
     * @param  substeps number of linear interpolation steps between start and end
     */
    std::vector< Point2D > vertices ( unsigned substeps=0 );

    /**
     * @overwrite
     */
    std::string toString() const;

  private:

    /**
     * Start point of the line.
     */
    Point2D m_Start;

    /**
     * Vector from the start to the end of the line.
     */
    CVec2 m_Vec;
};

#endif
