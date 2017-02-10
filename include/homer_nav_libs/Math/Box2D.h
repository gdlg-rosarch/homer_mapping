/*******************************************************************************
 *  Box2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 *  Additional information:
 *  $Id: Box2D.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef Box2D_H
#define Box2D_H

#include <homer_nav_libs/Math/Point2D.h>
#include <vector>

/**
 * @class  Box2D
 * @author David Gossow (RX)
 * @brief  Represents a box given by the upper-left and lower-right corner
 */
template<class T=float>
class Box2D
{

  public:

    /** @brief Creates a box given by top-left (minX,minY) and lower-right (maxX,maxY) coordinates */
    Box2D(T minX=0, T minY=0, T maxX=0, T maxY=0);

    /** @brief The destructor */
    ~Box2D() {};

    inline void setMinX(T value) { m_MinX=value; }
    inline void setMaxX(T value) { m_MaxX=value; }
    inline void setMinY(T value) { m_MinY=value; }
    inline void setMaxY(T value) { m_MaxY=value; }

    inline T minX() const { return m_MinX; }
    inline T maxX() const { return m_MaxX; }
    inline T minY() const { return m_MinY; }
    inline T maxY() const { return m_MaxY; }

    inline T width() const { return m_MaxX-m_MinX; }
    inline T height() const { return m_MaxY-m_MinY; }

    std::vector< Point2D > vertices();

    /** @brief Clip the box to fit into clipArea */
    void clip( Box2D<T> clipArea );

    /** @return true if the given point is inside the box */
    bool contains( T x, T y );

    /** @brief enlarge the box by 'size' units in all directions */
    void expand( T size );

    /** @brief shrink the box by 'size' units in all directions */
    void shrink( T size );

    /** @brief expand the box so that it contains the given point */
    void enclose( Point2D point );
    void enclose( T x, T y );

    template<class OtherT>
    void enclose( Box2D<OtherT> box );

    Point2D centerPoint()
    {
        Point2D center;
        center.setX(m_MinX + ( (m_MaxX - m_MinX) / 2 ) );
        center.setY(m_MinY + ( (m_MaxY - m_MinY) / 2 ) );
        return center;
    }

    /** @brief area covered by the box */
    T area();

    Box2D<T>& operator/= ( T div ) { m_MinX/=div; m_MinY/=div; m_MaxX/=div; m_MaxY/=div; return *this; }
    Box2D<T>& operator*= ( T div ) { m_MinX*=div; m_MinY*=div; m_MaxX*=div; m_MaxY*=div; return *this; }

  private:

    T m_MinX;
    T m_MaxX;
    T m_MinY;
    T m_MaxY;

};

template<class T>
Box2D<T>::Box2D(T minX, T minY, T maxX, T maxY)
{
  m_MinX=minX;
  m_MinY=minY;
  m_MaxX=maxX;
  m_MaxY=maxY;
}

template<class T>
void Box2D<T>::clip( Box2D<T> clipArea )
{
  if (m_MinX < clipArea.minX()) { m_MinX=clipArea.minX(); }
  if (m_MinY < clipArea.minY()) { m_MinY=clipArea.minY(); }
  if (m_MaxX > clipArea.maxX()) { m_MaxX=clipArea.maxX(); }
  if (m_MaxY > clipArea.maxY()) { m_MaxY=clipArea.maxY(); }
}

template<class T>
bool Box2D<T>::contains( T x, T y )
{
  return ( (x>=m_MinX) && (x<=m_MaxX) && (y>=m_MinY) && (y<=m_MaxY) );
}

template<class T>
    void Box2D<T>::expand( T size )
{
  m_MinX-=size;
  m_MaxX+=size;
  m_MinY-=size;
  m_MaxY+=size;
}

template<class T>
void Box2D<T>::shrink( T size )
{
  m_MinX+=size;
  m_MaxX-=size;
  m_MinY+=size;
  m_MaxY-=size;
}

template<class T>
T Box2D<T>::area()
{
  T width = m_MaxX - m_MinX;
  T height = m_MaxY - m_MinY;

  T capacity = width * height;

  return (T) capacity;
}

template<class T>
void Box2D<T>::enclose( Point2D point )
{
  if ( m_MinX > point.x() ) { m_MinX=point.x(); }
  if ( m_MinY > point.y() ) { m_MinY=point.y(); }
  if ( m_MaxX < point.x() ) { m_MaxX=point.x(); }
  if ( m_MaxY < point.y() ) { m_MaxY=point.y(); }
}

template<class T>
void Box2D<T>::enclose( T x, T y )
{
  if ( m_MinX > x ) { m_MinX=x; }
  if ( m_MinY > y ) { m_MinY=y; }
  if ( m_MaxX < x ) { m_MaxX=x; }
  if ( m_MaxY < y ) { m_MaxY=y; }
}

template<class T>
template<class OtherT>
void Box2D<T>::enclose( Box2D<OtherT> box )
{
  enclose( box.minX(), box.minY() );
  enclose( box.maxX(), box.maxY() );
}


template<class T>
std::vector< Point2D > Box2D<T>::vertices()
{
  std::vector<Point2D> myVertices(5);
  myVertices[0]=Point2D( m_MinX-0.5, m_MinY-0.5 );
  myVertices[1]=Point2D( m_MinX-0.5, m_MaxY+0.5 );
  myVertices[2]=Point2D( m_MaxX+0.5, m_MaxY+0.5 );
  myVertices[3]=Point2D( m_MaxX+0.5, m_MinY-0.5 );
  myVertices[4]=myVertices[0];
  return myVertices;
}


#endif
