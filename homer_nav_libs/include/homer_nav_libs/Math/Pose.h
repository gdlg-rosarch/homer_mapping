/*******************************************************************************
 *  Pose.h
 *
 *  (C) 2006 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Pose.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef POSE_H
#define POSE_H

#include <homer_nav_libs/Math/Point2D.h>

class Transformation2D;

/**
 * @class Pose
 *
 * @author Stephan Wirth, Susanne Maur (RX), David Gossow (RX)
 * @brief Class to describe and hold a pose of the robot (x, y)-Position + Orientation
 *        in world-coordinates
 */
class Pose : public Point2D {

public:

    /**
     * Constructor which initializes the members with the given values.
     * @param x x-position
     * @param y y-position
     * @param theta orientation in radiants
     */
    Pose(float x, float y, float theta);

    /**
     * Default constructor, initializes members to 0.
     */
    Pose();

    /**
    * The destructor is empty.
    */
    ~Pose();

    float theta() const;

    void setTheta(float theta);

    Pose operator+ ( const Transformation2D& transformation ) const;
    Pose operator- ( const Transformation2D& transformation ) const;
    Transformation2D operator- ( const Pose& pose ) const;

    /**
     * Interpolates between two poses and returns a pose which correlates with
     * current pose +  t * (reference pose - current pose)
     * @param referencePose The second pose to interpolate between.
     * @param t The factor of interpolation.
     * @return Interpolated pose
     */
    Pose interpolate(const Pose& referencePose, float t) const;

//    Pose( ExtendedInStream& extStrm );

//    void storer( ExtendedOutStream& extStrm ) const;

protected:

    float m_Theta;
};

#endif
