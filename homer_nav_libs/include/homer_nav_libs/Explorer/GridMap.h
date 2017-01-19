#ifndef   GridMap_H
#define   GridMap_H

#include <float.h>
#include <iostream>
#include <sstream>

#include "ros/ros.h"

#include <Eigen/Geometry>

#define GRIDMAP_SAFE_ACCESS

/**
 * @class  GridMap
 * @author Malte Knauf, Stephan Wirth, David Gossow (RX)
 * @brief  GridMap data structure. Implemeted as template class. The template type
 *         defines the data type of each map cell.
 */

template<class DataT>
class GridMap
{

	public:

		/// Initialize empty map
		GridMap();

		/**
		 * @param width Width of the map.
		 * @param height Height of the map.
		 * @param data Pointer to map data, must be of size width*height.
		 * @param copyData if true, the map data will be copied
		 *                 if false, GridMap takes ownership of the pointer
         * @param cellSize physical size of each map cell [m]
		 * @param centerX,centerY center of the map in world coordinates
		 */
		GridMap ( int width, int height, DataT* data = 0, bool copyData = true, float cellSize = 1, float centerX = 0, float centerY = 0 );

		/// Copy data from given region
        GridMap ( int width, int height, DataT* data, Eigen::AlignedBox2i extractRegion );

		/// Copy data from given map
		GridMap<DataT> ( const GridMap<DataT>& other ) { m_Data=0; *this = other; }

		/// Copy data from given map
		GridMap<DataT>& operator= ( const GridMap<DataT>& other );

        ~GridMap();

		/// Convert map coordinates to world coordinates
		void mapToWorld ( int mapX, int mapY, float& worldX, float& worldY );

		/// Convert world coordinates to map coordinates
		void worldToMap ( float worldX, float worldY, int& mapX, int& mapY );

		/// @brief set value at given position
		inline void setValue ( int x, int y, DataT val );

		/// @brief replace content with given value
		void fill ( DataT val );
		
		/// @brief Draw a filled polygon into the map (world coords)
        void drawPolygon ( std::vector<Eigen::Vector2d> vertices, DataT value );

		/// @brief Draw a filled circle into the map (world coords)
        void drawCircle( Eigen::Vector2d center, float radius, DataT value );

		/// @return Value at the given position.
		inline DataT getValue ( int x, int y ) const;

		/// @return Pointer to given pixel
        inline DataT* getDirectAccess ( int x, int y );

		/// @return width in grid cells
		int width() const { return m_Width; }

		/// @return height in grid cells
		int height() const { return m_Height; }

  /// @return center of the map in world coordinates
  Eigen::Vector2d center() const {return Eigen::Vector2d(m_CenterX,m_CenterY);}

		/// @return side length of one cell in mm
		float cellSize() { return m_CellSize; }

	private:

		void drawLine ( DataT *data, int startX, int startY, int endX, int endY, DataT value );
		void fillPolygon ( DataT* data, int x, int y, char value );

		int m_Width;
		int m_Height;
		int m_DataSize;
		DataT* m_Data;
		float m_CellSize;
		float m_CenterX;
		float m_CenterY;
};


template<class DataT>
GridMap<DataT>::GridMap()
{
	m_Width = 0;
	m_Height = 0;
	m_DataSize = 0;
	m_Data = 0;
	m_CellSize = 0;
	m_CenterX = 0;
	m_CenterY = 0;
}

template<class DataT>
GridMap<DataT>::GridMap ( int width, int height, DataT* data, bool copyData, float cellSize, float centerX, float centerY )
{
	m_Width = width;
	m_Height = height;
	m_CellSize = cellSize;
	m_DataSize = width * height;
	m_CenterX = centerX;
	m_CenterY = centerY;
	m_Data = 0;

	if ( data )
	{
		if ( copyData )
		{
			m_Data = new DataT[m_DataSize];

			for ( int i = 0; i < m_DataSize; i++ )
			{
				m_Data[i] = data[i];
			}
		}
		else
		{
			m_Data = data;
		}
	}
	else
	{
		m_Data = new DataT[m_DataSize];

		for ( int i = 0; i < m_DataSize; i++ )
		{
			m_Data[i] = 0;
		}
	}
}

template<class DataT>
GridMap<DataT>::GridMap (int width, int height, DataT* data, Eigen::AlignedBox2i extractRegion )
{
    m_Width = extractRegion.sizes().x();
    m_Height = extractRegion.sizes().y();
	m_DataSize = m_Width * m_Height;
	m_Data = new DataT[m_DataSize];
	m_CellSize = 1;
	m_CenterX = 0;
	m_CenterY = 0;

    for ( int y = extractRegion.min().y(); y <= extractRegion.max().y(); y++ )
	{
        int yOffset = m_Width * y;

        for ( int x = extractRegion.min().x(); x <= extractRegion.max().x(); x++ )
		{
            int i = x + yOffset;
			m_Data[i] = data[i];
		}
	}
}


template<class DataT>
inline DataT* GridMap<DataT>::getDirectAccess ( int x, int y )
{
#ifdef GRIDMAP_SAFE_ACCESS
	if ( x >= 0 && x < m_Width && y >= 0 && y < m_Height )
	{
        return &m_Data[y * m_Width + x];
	}
	else
	{
		throw;
	}
#else
    return &m_Data[y * m_Width + x];
#endif
}


template<class DataT>
GridMap<DataT>& GridMap<DataT>::operator= ( const GridMap<DataT>& other )
{
	delete[] m_Data;
	m_Width = other.m_Width;
	m_Height = other.m_Height;
	m_DataSize = other.m_DataSize;
	m_Data = new DataT[m_DataSize];
	memcpy ( m_Data, other.m_Data, sizeof ( DataT ) *m_DataSize );
	m_CellSize = other.m_CellSize;
	m_CenterX = other.m_CenterX;
	m_CenterY = other.m_CenterY;
  return *this;
}
/* TODO
template<class DataT>
GridMap<DataT>::GridMap ( ExtendedInStream& strm )
{
	short version;
	strm >> version;
	strm >> m_Width;
	strm >> m_Height;
	strm >> m_CellSize;
	strm >> m_CenterX;
	strm >> m_CenterY;
	m_DataSize = m_Width * m_Height;
	m_Data = new DataT[m_DataSize];
	strm.get ( m_Data, m_DataSize );
}
*/
template<class DataT>
GridMap<DataT>::~GridMap()
{
	if ( m_Data )
	{
		delete m_Data;
		m_Data = 0;
	}
}
/*
template<class DataT>
void GridMap<DataT>::storer ( ExtendedOutStream& strm ) const
{
	strm << short ( 12 );
	strm << m_Width;
	strm << m_Height;
	strm << m_CellSize;
	strm << m_CenterX;
	strm << m_CenterY;
	strm.put ( m_Data, m_DataSize );
}
*/

template<class DataT>
void GridMap<DataT>::mapToWorld ( int mapX, int mapY, float& worldX, float& worldY )
{
	worldX = m_CenterX + m_CellSize * ( mapX - m_Width / 2 );
	worldY = m_CenterY + m_CellSize * ( mapY - m_Height / 2 );
}

template<class DataT>
void GridMap<DataT>::worldToMap ( float worldX, float worldY, int& mapX, int& mapY )
{
	mapX = float ( m_Width ) / 2.0  - ( ( worldY - m_CenterY ) / m_CellSize + 0.5 );
	mapY = float ( m_Height ) / 2.0 - ( ( worldX - m_CenterX ) / m_CellSize + 0.5 );

	if ( mapX < 0 || mapX >= m_Width || mapY < 0 || mapY >= m_Height )
	{
                //ROS_WARN_STREAM ( "Index out of bounds: " << mapX << "," << mapY ); //TODO

		if ( mapX < 0 )
		{
			mapX = 0;
		}

		if ( mapX >= m_Width )
		{
			mapX = m_Width - 1;
		}

		if ( mapY < 0 )
		{
			mapY = 0;
		}

		if ( mapY >= m_Height )
		{
			mapY = m_Height - 1;
		}
	}
}


template<class DataT>
inline void GridMap<DataT>::setValue ( int x, int y, DataT val )
{
#ifdef GRIDMAP_SAFE_ACCESS
	if ( x >= 0 && x < m_Width && y >= 0 && y < m_Height )
	{
        m_Data[y * m_Width + x] = val;
	}
	else
	{
		throw;
	}
#else
    m_Data[y * m_Width + x] = val;
#endif
}

template<class DataT>
inline DataT GridMap<DataT>::getValue ( int x, int y ) const
{
#ifdef GRIDMAP_SAFE_ACCESS
	if ( x >= 0 && x < m_Width && y >= 0 && y < m_Height )
	{
        return  m_Data[y * m_Width + x];
	}
	else
	{
                ROS_ERROR_STREAM( "Accessing map pixels " << x << "," << y << ": out of bounds (0,0," << m_Width-1 << "," << m_Height-1 << ")" ); //TODO
		throw;
	}
#else
    return  m_Data[y * m_Width + x];
#endif
}

template<class DataT>
void GridMap<DataT>::fill ( DataT val )
{
	for ( int i = 0; i < m_DataSize; i++ )
	{
		m_Data[i] = val;
	}
}

/* TODO do we need image representation?
template<class DataT>
puma2::ColorImageRGB8* GridMap<DataT>::getImage ( DataT specialValue, DataT clipRangeLow, DataT clipRangeHigh )
{
	puma2::ColorImageRGB8* image = new puma2::ColorImageRGB8 ( m_Width, m_Height );
	double maxVal = 0.0001;
	double minVal = 0.0;

	for ( int i = 0; i < m_DataSize; i++ )
    {
		if ( ( m_Data[i] < minVal ) && ( m_Data[i] != specialValue ) )
		{
			minVal = m_Data[i];
		}

		if ( ( m_Data[i] > maxVal ) && ( m_Data[i] != specialValue ) )
		{
			maxVal = m_Data[i];
		}
	}

	std::ostringstream stream;

	stream << " Min: " << minVal << "Max: " << maxVal;
	stream << " ClipMin: " << double ( clipRangeLow ) << " ClipMax: " << double ( clipRangeHigh );
        ROS_DEBUG_STREAM ( stream.str() ); //TODO: was TRACE_SYSTEMINFO

	if ( maxVal > clipRangeHigh )
	{
		maxVal = clipRangeHigh;
	}

	if ( minVal < clipRangeLow )
	{
		minVal = clipRangeLow;
	}

	double range = maxVal - minVal;

	puma2::ColorImageRGB8::PixelType* imageData;
	imageData = image->unsafeRowPointerArray() [0];

	for ( int i = 0; i < m_DataSize; i++ )
	{
		DataT currentValue = m_Data[i];

		if ( currentValue == specialValue )
		{
			imageData[i][0] = 40;
			imageData[i][1] = 220;
			imageData[i][2] = 120;
			continue;
		}

		if ( currentValue > clipRangeHigh )
		{
			imageData[i][0] = 200;
			imageData[i][1] = 200;
			imageData[i][2] = 128;
			continue;
		}

		if ( currentValue < clipRangeLow )
		{
			imageData[i][0] = 40;
			imageData[i][1] = 40;
			imageData[i][2] = 180;
			continue;
		}

		double valueDouble  = ( ( double ) ( currentValue - minVal ) ) / range;

		unsigned char value = ( unsigned char ) ( valueDouble * 255 );

		imageData[i][0] = value;
		imageData[i][1] = value;
		imageData[i][2] = value;
	}

	return image;
}
*/



template<class DataT>
void GridMap<DataT>::drawCircle(Eigen::Vector2d center, float radius, DataT value )
{
	int centerMapX,centerMapY;
	worldToMap( center.x(), center.y(), centerMapX, centerMapY );
	
	int radiusCells = radius / m_CellSize;
	int radiusCells2 = radiusCells*radiusCells;
	
    Eigen::AlignedBox2i bBox( Eigen::Vector2i(centerMapX - radiusCells, centerMapY - radiusCells), Eigen::Vector2i(centerMapX + radiusCells, centerMapY + radiusCells) );
    Eigen::AlignedBox2i bBoxGrid( Eigen::Vector2i(0,0), Eigen::Vector2i(m_Width-1,m_Height-1) );
    bBox.clamp( bBoxGrid );
	
    for ( int y = bBox.min().y(); y <= bBox.max().y(); y++ )
	{
        for ( int x = bBox.min().x(); x <= bBox.max().x(); x++ )
		{
			int xC = x-centerMapX;
            int yC = y-centerMapY;
			if ( xC*xC+yC*yC <= radiusCells2 )
			{
				setValue( x, y, value );
			}
		}
	}
}


template<class DataT>
void GridMap<DataT>::drawPolygon (std::vector<Eigen::Vector2d> vertices, DataT value )
{
  if ( vertices.size() == 0 )
  {
    ROS_INFO( "No vertices given!" );
    return;
  }
	//make temp. map
	DataT* data = new DataT[ m_DataSize ];
	for ( int i = 0; i < m_DataSize; i++ )
	{
		data[i] = 0;
	}
	
  //draw the lines surrounding the polygon
  for ( unsigned int i = 0; i < vertices.size(); i++ )
  {
		int i2 = ( i+1 ) % vertices.size();
		int startX,startY,endX,endY;
		worldToMap( vertices[i].x(), vertices[i].y(), startX, startY );
		worldToMap( vertices[i2].x(), vertices[i2].y(), endX, endY );
    drawLine ( data, startX, startY, endX, endY, 1 );
  }
  //claculate a point in the middle of the polygon
  float midX = 0;
  float midY = 0;
  for ( unsigned int i = 0; i < vertices.size(); i++ )
  {
    midX += vertices[i].x();
    midY += vertices[i].y();
  }
  midX /= vertices.size();
  midY /= vertices.size();
	int midMapX,midMapY;
	worldToMap( midX, midY, midMapX, midMapY );
  //fill polygon
  fillPolygon ( data, midMapX, midMapY, 1 );
	
	//copy polygon to map
	for ( int i = 0; i < m_DataSize; i++ )
	{
		if ( data[i] != 0 )
		{
			m_Data[i] = value;
		}
	}
	
	delete[] data;
}

template<class DataT>
void GridMap<DataT>::fillPolygon ( DataT* data, int x, int y, char value )
{
  int index = x + m_Width * y;
  if ( value != data[index] )
  {
    data[index] = value;
    fillPolygon ( data, x + 1, y, value );
    fillPolygon ( data, x - 1, y, value );
    fillPolygon ( data, x, y + 1, value );
    fillPolygon ( data, x, y - 1, value );
  }
}


template<class DataT>
void GridMap<DataT>::drawLine ( DataT *data, int startX, int startY, int endX, int endY, DataT value )
{
  //bresenham algorithm
  int x, y, t, dist, xerr, yerr, dx, dy, incx, incy;
  // compute distances
  dx = endX - startX;
  dy = endY - startY;

  // compute increment
  if ( dx < 0 )
  {
    incx = -1;
    dx = -dx;
  }
  else
  {
    incx = dx ? 1 : 0;
  }

  if ( dy < 0 )
  {
    incy = -1;
    dy = -dy;
  }
  else
  {
    incy = dy ? 1 : 0;
  }

  // which distance is greater?
  dist = ( dx > dy ) ? dx : dy;
  // initializing
  x = startX;
  y = startY;
  xerr = dx;
  yerr = dy;

  // compute cells
  for ( t = 0; t < dist; t++ )
  {
    data[x + m_Width * y] = value;
		
    xerr += dx;
    yerr += dy;
    if ( xerr > dist )
    {
      xerr -= dist;
      x += incx;
    }
    if ( yerr > dist )
    {
      yerr -= dist;
      y += incy;
    }
  }
}



#endif

#ifdef GRIDMAP_SAFE_ACCESS
#undef GRIDMAP_SAFE_ACCESS
#endif
