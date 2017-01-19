/*******************************************************************************
 *  Pixel.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Pixel.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef PIXEL_H
#define PIXEL_H

#include <homer_nav_libs/Math/Point2D.h>
#include <vector>

/** @class Pixel
  * @brief Stores discrete pixel coordinates
  * @author Stephan Wirth, David Gossow (RX)
  */
class Pixel {

  public:

    inline Pixel( int x = 0, int y = 0) { m_X = x; m_Y = y; };
    inline ~Pixel() {};

    inline int x() const { return m_X; }
    inline int y() const { return m_Y; }

    inline void setX( int x ) { m_X = x; }
    inline void setY( int y ) { m_Y = y; }

    inline bool operator ==( Pixel& rhs ) { return ( m_X == rhs.m_X ) && ( m_Y == rhs.m_Y ); }
    inline bool operator !=( Pixel& rhs ) { return ( m_X != rhs.m_X ) || ( m_Y != rhs.m_Y ); }
    inline Pixel operator *( float rhs ) { return Pixel( m_X * rhs, m_Y * rhs ); }
    inline Pixel operator /( float rhs ) { return Pixel( m_X / rhs, m_Y / rhs ); }

    inline Point2D toPoint2D() { return Point2D( m_X, m_Y ); }
    
    inline std::vector<Point2D> vertices()
    {
      std::vector<Point2D> result(5);
      result[0]=Point2D( m_X-0.5, m_Y-0.5 );
      result[1]=Point2D( m_X+0.5, m_Y-0.5 );
      result[2]=Point2D( m_X+0.5, m_Y+0.5 );
      result[3]=Point2D( m_X-0.5, m_Y+0.5 );
      result[4]=result[0];
      return result;
    }

  private:

    int m_X;
    int m_Y;

};


#endif
