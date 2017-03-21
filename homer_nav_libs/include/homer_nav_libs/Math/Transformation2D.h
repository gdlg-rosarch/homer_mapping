/*******************************************************************************
 *  Transformation2D.h
 *
 *  (C) 2008 AG Aktives Sehen <agas@uni-koblenz.de>
 *           Universitaet Koblenz-Landau
 *
 * $Id: Transformation2D.h 44313 2011-04-06 22:46:28Z agas $
 *******************************************************************************/

#ifndef TRANSFORMATION2D_H
#define TRANSFORMATION2D_H

#include <homer_nav_libs/Math/Line2D.h>
#include <homer_nav_libs/Math/Point2D.h>
#include <cmath>
#include <vector>

/**
 * @class Transformation2D
 *
 * @author Susanne Maur
 *
 * @brief Class to describe a transformation of poses in 2D.
 * This inplies a translation in x and y direction each and a rotation.
 */
class Transformation2D : public CVec2
{
public:
  /**
   * Constructor that initializes the members.
   * @param x translation in x direction in m
   * @param y translation in y direction in m
   * @param theta rotation in radiants
   */
  Transformation2D(double x, double y, double theta);

  /**
   * Constructor that initializes the members.
   * @param vec a vector which represents the translation in x and y direction
   * @param theta rotation in radiants
   */
  Transformation2D(const CVec2& vec, double theta);

  /**
   * Default constructor sets all members to 0.0.
   */
  Transformation2D();

  /**
   * Default destructor.
   */
  ~Transformation2D();

  /**
   * Sets the values of transformation.
   * @param x translation in x direction in mm
   * @param y translation in y direction in mm
   * @param theta rotation in radiants
   */
  void set(double x, double y, double theta);

  /**
   * Returns the rotation in radiants.
   * @return rotation in radiants
   */
  double theta() const;

  /**
   * Adds two transformations.
   */
  Transformation2D operator+(Transformation2D t) const;
  Transformation2D& operator+=(Transformation2D t);

  /**
   * Subtracts two transformations.
   */
  Transformation2D operator-(Transformation2D t) const;
  Transformation2D& operator-=(Transformation2D t);

  /**
   * Scales a transformation by a factor
   */
  Transformation2D operator*(float factor) const;
  Transformation2D& operator*=(float factor);

  /**
   * Scales a transformation by a factor
   */
  Transformation2D operator/(float factor) const;
  Transformation2D& operator/=(float factor);

  /**
   * Test equality of transformations.
   */
  bool operator==(Transformation2D t) const;
  bool operator!=(Transformation2D t) const;

  /**
   * Compare transformations.
   * (attention: algebraic signs are taken into account, if necessary use
   * fabs())
   */
  bool operator<=(Transformation2D t) const;
  bool operator>=(Transformation2D t) const;
  bool operator<(Transformation2D t) const;
  bool operator>(Transformation2D t) const;

  /**
   * Applies abs() on every attribute.
   */
  Transformation2D abs() const;

  /**
   * Inverts the transformation, scales every attribute with -1.
   */
  Transformation2D inverse() const;

  /**
   * Transformes points by first rotation, then translating.
   */
  Point2D transform(const Point2D& point) const;
  std::vector<Point2D> transform(const std::vector<Point2D>& points) const;

  /**
   * Transformes lines by first rotation, then translating.
   */
  Line2D transform(const Line2D& line) const;
  std::vector<Line2D> transform(const std::vector<Line2D>& lines) const;

  /**
   * Returns the string representation of the transformation.
   * @return string representation of the transformation.
   */
  std::string toString() const;

private:
  double m_Theta;
};

#endif
