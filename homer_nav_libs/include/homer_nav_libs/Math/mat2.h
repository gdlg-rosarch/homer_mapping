/*******************************************************************************
 *  mat2.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * Author: Frank Neuhaus
 *******************************************************************************/

#ifndef MAT2_H
#define MAT2_H

#include <math.h>
#include <homer_nav_libs/Math/Point2D.h>
#include <homer_nav_libs/Math/vec2.h>

class CMat2
{
	public:
		CMat2();
		CMat2(float rot);
		~CMat2();

		CMat2 operator *(const CMat2 &mat) const;

		CVec2 operator *(const CVec2& v) const;

		Point2D operator *(const Point2D& p) const;
        
        float& operator [] (unsigned int position) {
            return fMatrix[position];
        }
        
        CMat2 operator +(const CMat2 rhs) const {
            CMat2 newMatrix;
            for (unsigned int i = 0; i < 4; i++) {
                newMatrix[i] = valueAt(i) + rhs.valueAt(i);
            }
            return newMatrix;
        }
        
        CMat2 operator -(const CMat2 rhs) const {
            CMat2 newMatrix;
            for (unsigned int i = 0; i < 4; i++) {
                newMatrix[i] = valueAt(i) - rhs.valueAt(i);
            }
            return newMatrix;
        }
        
        float valueAt(unsigned int position) const{
            return fMatrix[position];
        }
        
		union
		{
			float fMatrix[4];
			float m[2][2];
			struct
			{
				float xx, xy;
				float yx, yy;
			};
		};

		void transpose();
		void loadIdentity();

		void makeRotation(float fA);

		bool invert();
};



#include "mat2_inl.h"

#endif
