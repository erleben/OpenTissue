#ifndef OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_BASE_INTERPOLATOR_H
#define OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_BASE_INTERPOLATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>


namespace OpenTissue
{

  namespace interpolation
  {

    /**
    * Interpolator Base Class.
    * All interpolation and extrapolation algorithms must
    * be derived from this base class.
    *
    * This inteface defines the basic methods which all
    * (1-dimensional) interpolation and extrapolation algorithms
    *  must support.
    */
    template<typename child_type_,typename real_type_>
    class BaseInterpolator
    {
    public:

      typedef child_type_   child_type;
      typedef real_type_    real_type;

    public:
      /**
      * Set Tableau Method.
      * This method should be used to initialize the tableau
      * that should be used to compute the interpolated (or
      * extrapolated) value.
      *
      * We have
      *
      * y0 = f(x0),y1 = f(x1),...yn = f(xn),
      *
      * @param x   The array of parameter values.
      * @param y   The array of corresponding function values. Must
      *            have same length as the x-array.
      * @param cnt The number of value pairs.
      */
      void set_tableau(real_type  * x,real_type * y,int cnt)
      {
        child_type & self = static_cast<child_type &>(*this);
        self.set_tableau(x,y,cnt);
      }

      /**
      * Get Interpolation Value.
      * This method should compute the interpolated or extrapolated
      * value at the specified point.
      *
      * @param x    The point at which the interpolated or extrapolated
      *             value should be found at.
      *
      * @return     The interpolated (or extrapolated) value at
      *             the point x.
      */
      real_type get_value(real_type const & x)
      {
        child_type & self = static_cast<child_type &>(*this);
        return self.get_value(x);
      }

      /**
      * Error Estimate Method.
      * This method should return the error estimate of the
      * last interpolated value computation, which have
      * taken place.
      *
      * @return   The value of the error estimate.
      */
      real_type get_error_estimate()
      {
        child_type & self = static_cast<child_type &>(*this);
        return self.get_error_estimate();
      }

    };

  } // namespace interpolation

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_INTERPOLATION_INTERPOLATION_BASE_INTERPOLATOR_H
#endif
