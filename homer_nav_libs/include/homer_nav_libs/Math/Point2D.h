/*******************************************************************************
 *  Point2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Point2D.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#include <iostream>
#include <sstream>

#include <homer_nav_libs/Math/vec2.h>
#include <float.h>

#ifndef POINT2D_H
#define POINT2D_H

/**
 * @class Point2D
 *
 * @author Susanne Maur
 *
 */
class Point2D
{

  public:

    /**
     * Creates a new point in 2D with x- and y-coordinat set to zero.
     */
    inline Point2D()
    {
      m_X = 0.0;
      m_Y = 0.0;
    }

    /**
    * Creates a new point in 2D.
    * @param x x-coordinate of the point.
    * @param y y-coordinate of the point.
     */
    inline Point2D ( double x, double y )
    {
      m_X = x;
      m_Y = y;
    }



    /**
     * Copy construcor
     */
    inline Point2D (const Point2D& p){
      m_X = p.x();
      m_Y = p.y();
    }
    
    /**
    * Creates a new point in 2D.
    * @param v Vector form origin to the point.
     */
    inline Point2D ( const CVec2& v )
    {
      m_X = v[0];
      m_Y = v[1];
    }

    /**
    * Destructor, does nothing.
     */
    inline ~Point2D()
    {
    }

    /**
    * Returns the x-coordinate of the point.
    * @return the x-coordinate of the point.
     */
    inline double x() const
    {
      return m_X;
    }

    /**
    * Returns the y-coordinate of the point.
    * @return the y-coordinate of the point.
     */
    inline double y() const
    {
      return m_Y;
    }

    /**
    * Sets the x- and y-coordinate of the point to new values.
    * @param x the new value of the x coordinate.
    * @param y the new value of the x coordinate.
     */
    inline void set ( double x, double y )
    {
      m_X = x;
      m_Y = y;
    }

    /**
    * Sets the x-coordinate of the point to a new value.
    * @param x the new value of the x coordinate.
     */
    inline void setX ( double x )
    {
      m_X = x;
    }

    /**
    * Sets the y-coordinate of the point to a new value.
    * @param y the new value of the x coordinate.
     */
    inline void setY ( double y )
    {
      m_Y = y;
    }

    /**
     * Overloaded operators.
     */

    inline Point2D& operator= ( const Point2D& p) {
      m_X = p.x();
      m_Y = p.y(); 
      return *this;
    }

    inline Point2D operator+ ( const CVec2& v ) const
    {
      return Point2D ( m_X + v[0], m_Y + v[1] );
    }

    inline Point2D operator+ ( const Point2D& p ) const
    {
      return Point2D ( m_X + p.x(), m_Y + p.y() );
    }

    inline CVec2 operator- ( const Point2D& p ) const
    {
      return CVec2 ( m_X - p.x(), m_Y - p.y() );
    }

    inline Point2D operator- ( const CVec2& v ) const
    {
      return Point2D ( m_X - v[0], m_Y - v[1] );
    }

    inline Point2D operator* ( double scalar ) const
    {
      return Point2D ( m_X * scalar, m_Y * scalar );
    }

    inline Point2D operator/ ( double scalar ) const
    {
      return Point2D ( m_X / scalar, m_Y / scalar );
    }

    inline Point2D& operator+= ( const CVec2& v )
    {
      m_X += v[0];
      m_Y += v[1];
      return ( *this );
    }

    inline Point2D& operator-= ( const CVec2& v )
    {
      m_X -= v[0];
      m_Y -= v[1];
      return ( *this );
    }

    inline Point2D& operator*= ( double scalar )
    {
      m_X *= scalar;
      m_Y *= scalar;
      return ( *this );
    }

    inline Point2D& operator/= ( double scalar )
    {
      m_X /= scalar;
      m_Y /= scalar;
      return ( *this );
    }

    inline double operator [] ( unsigned int i ) const
    {
      return ( ( double* ) this ) [i];
    }

    inline double& operator [] ( unsigned int i )
    {
      return ( ( double* ) this ) [i];
    }

    inline bool operator== ( const Point2D& point ) const
    {
      return ( fabs(m_X - point.x()) < 0.001 && fabs(m_Y - point.y()) < 0.001 );
    }

    inline bool operator!= ( const Point2D& point ) const
    {
      return !((*this)== point);
    }

    /**
    * Returns the distance to a given point.
    * @param point The point to calculate the distance to.
    * @return the distance between point the two points.
     */
    inline double distance ( const Point2D& point ) const
    {
      return sqrt ( ( m_X-point.x() ) * ( m_X-point.x() ) + ( m_Y-point.y() ) * ( m_Y-point.y() ) );
    }

    /**
    * Returns the distance to origin.
    * @return the distance between point the two points.
     */
    inline double distance ( ) const
    {
      return sqrt ( m_X * m_X + m_Y * m_Y );
    }

    /**
    * Checks whether two points are equal.
    * @param p The point to check equality.
    * @return true if points are equal, false otherwise.
     */
    inline bool equal ( const Point2D& point ) const
    {
      if ( ( *this - point ).magnitude() < 0.0001 )
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /**
    * Returns the vector which represents the point in 2D.
    * @return vector which represents the point in 2D.
     */
    inline CVec2 toVector() const
    {
      return CVec2 ( m_X, m_Y );
    }

        /**
     * Returns the angle of the corresponding polar coordinates.
     * @return polar angle.
         */
    float getPolarAngle () const;

    /**
    * Rotate by angle (in radiants) around center.
    * @param center Center of rotation
    * @param angle Angle in radiants
     */
    inline void rotate ( const Point2D& center, float angle )
    {
      double x0=m_X-center.m_X;
      double y0=m_Y-center.m_Y;
      double xRot =  x0*cos ( angle ) - y0*sin ( angle );
      double yRot =  x0*sin ( angle ) + y0*cos ( angle );
      m_X = xRot+center.m_X;
      m_Y = yRot+center.m_Y;
    }

    /**
     * Rotate by angle (in radiants) around (0,0).
     * @param angle Angle in radiants
     */
    inline void rotate ( float angle )
    {
      double xRot =  m_X*cos ( angle ) - m_Y*sin ( angle );
      double yRot =  m_X*sin ( angle ) + m_Y*cos ( angle );
      m_X = xRot;
      m_Y = yRot;
    }

    /**
    * Returns the string representation of the point.
    * @return string representation of the point.
     */
    inline std::string toString() const
    {
      std::ostringstream str;
      str << m_X << " " << m_Y;
      return str.str();
    }

    /** @return "invalid" Point (used as end marker in vector drawings) **/
    static Point2D invalidPoint() { return Point2D( DBL_MAX, DBL_MAX  ); }

    bool isValid() { return ( ( m_X != DBL_MAX ) || ( m_Y != DBL_MAX ) ); }

  protected:

    double m_X;
    double m_Y;
};

#endif
