/*******************************************************************************
 *  mat2_inl.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#include <assert.h>

inline CMat2::CMat2(){
    for (unsigned int i = 0; i < 4; i++) {
        fMatrix[i] = 0;
    }
}

inline CMat2::~CMat2()
{}

inline CMat2::CMat2(float rot)
{
	makeRotation(rot);
}

inline CMat2 CMat2::operator * ( const CMat2 & mat ) const
{
    CMat2 retValue;
    for (unsigned int line = 0; line < 2; line++) {
        for (unsigned int column = 0; column < 2; column++) {
            retValue[line*2 + column] = valueAt(line*2 + column) + mat.valueAt(column*2 + line);
        }
    }
    return retValue;
}

inline CVec2 CMat2::operator * ( const CVec2& v ) const
{
	return CVec2(xx*v[0] + xy*v[1],yx*v[0] + yy*v[1]);
}

inline Point2D CMat2::operator * ( const Point2D& p ) const
{
	return Point2D(xx*p.x() + xy*p.y(), yx*p.x() + yy*p.y());
}

inline void CMat2::transpose()
{
	float t=xy;
	xy=yx;
	yx=t;
}

inline void CMat2::loadIdentity()
{
	xx=1.0f; xy=0.0f;
	yx=0.0f; yy=1.0f;
}

inline void CMat2::makeRotation ( float fA )
{
	xx=yy=cosf(fA);
	yx=sinf(fA);
	xy=-yx;
}

inline bool CMat2::invert()
{
	CMat2 tmp;
	float det = fMatrix[0]*fMatrix[3] - fMatrix[1]*fMatrix[2];

	if(fabs(det) < 0.001f)	return false;

	det = 1.0 / det;
	tmp.fMatrix[0] = fMatrix[3]*det;
	tmp.fMatrix[1] = -fMatrix[1]*det;
	tmp.fMatrix[2] = -fMatrix[2]*det;
	tmp.fMatrix[3] = fMatrix[0]*det;

	(*this)=tmp;
	return true;
}




