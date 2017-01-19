/*******************************************************************************
 *  vec2.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus, Susanne Maur
 *******************************************************************************/

#include <iostream>
#include <sstream>

#ifndef VEC2_H
#define VEC2_H

#include <math.h>

class CVec2
{
  public:

    inline CVec2()
    {
      m_X = 0; m_Y = 0;
    }

    inline CVec2 ( double x, double y )
    {
      m_X=x; m_Y=y;
    }

    inline CVec2 ( const CVec2& vec )
    {
      m_X=vec.x(); m_Y=vec.y();
    }

    inline CVec2 operator+ ( const CVec2& vVector ) const
    {
      return CVec2 ( vVector[0] + m_X, vVector[1] + m_Y );
    };

    inline CVec2 operator- ( const CVec2& vVector ) const
    {
      return CVec2 ( m_X - vVector[0], m_Y - vVector[1] );
    };
    
    inline CVec2 operator- ( ) const
    {
        return CVec2 ( - m_X, - m_Y );
    };

    inline CVec2 operator* ( double num ) const
    {
      return CVec2 ( m_X * num, m_Y * num );
    };

    inline double operator* ( const CVec2& vVector ) const
    {
      return m_X*vVector[0]+m_Y*vVector[1];
    }

    inline CVec2 operator/ ( double num ) const
    {
      return CVec2 ( m_X / num, m_Y / num );
    }

    inline void set ( double fx, double fy )
    {
      m_X=fx; m_Y=fy;
    }

    inline double x() const
    {
      return m_X;
    }

    inline double y() const
    {
      return m_Y;
    }

    inline double magnitude() const
    {
      double sumOfSquares = m_X*m_X + m_Y*m_Y;
      return sqrt ( sumOfSquares );
    }

    inline double operator [] ( unsigned int i ) const
    {
      return ( ( double* ) this ) [i];
    }

    inline double& operator [] ( unsigned int i )
    {
      return ( ( double* ) this ) [i];
    }

    inline CVec2& operator/= ( double num )
    {
      double inv=1.0f/num;
      m_X*=inv;
      m_Y*=inv;
      return ( *this );
    }

    inline CVec2& operator*= ( double num )
    {
      m_X*=num;
      m_Y*=num;
      return ( *this );
    }

    inline CVec2& normalize()
    {
      return ( *this/=magnitude() );
    }

    inline CVec2& makePerp()
    {
      double xn=m_X;
      m_X=-m_Y;
      m_Y=xn;
      return *this;
    }

    inline CVec2 getNormal() const
    {
      return CVec2 ( m_Y, -m_X ); //?
    }

    inline CVec2 getNormalized() const
    {
      return ( *this ) /magnitude();
    }

    inline double sqr() const
    {
      return ( *this ) * ( *this );
    }

    inline double dot ( const CVec2& vec ) const
    {
      return ( m_X*vec[0] ) + ( m_Y*vec[1] );
    }

    inline double getAngle ( const CVec2& vec ) const
    {
      return acos ( dot ( vec ) / ( magnitude() *vec.magnitude() ) );
    }

    /// @param angle Rotation angle in radiants
    inline CVec2 rotate ( float angle ) const
    {
      double xRot =  m_X*cos ( angle ) - m_Y*sin ( angle );
      double yRot =  m_X*sin ( angle ) + m_Y*cos ( angle );
      return CVec2 ( xRot, yRot );
    }

    inline bool equal ( CVec2 vec ) const
    {
      return ( m_X==vec.x() && m_Y==vec.y() );
    }

    /**
     * Returns the string representation of the vector.
     * @return string representation of the point.
     */
    inline std::string toString() const
    {
      std::ostringstream str;
      str << m_X << " " << m_Y;
      return str.str();
    }

  protected:
    double m_X, m_Y;
};

inline CVec2 operator* ( double f, const CVec2& v )
{
  return v*f;
}

inline CVec2 normalize ( const CVec2& v )
{
  return v/v.magnitude();
}

#endif
