#ifndef OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
#define OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> // used for boost::numeric_cast


namespace OpenTissue
{

  namespace math
  {

    /**
    * Trillinear Interpolation.
    * Interpolation of values in a cubic grid.
    */
    template <typename T,typename T2>
    inline T  trillinear(
      T const & d000
      , T const & d001
      , T const & d010
      , T const & d011
      , T const & d100
      , T const & d101
      , T const & d110
      , T const & d111
      , T2 const & s
      , T2 const & t
      , T2 const & u
      ) 
    {
      //T x00 = ( detail::one<T>() - s ) * d000 + s * d001 ;
      //T x01 = ( detail::one<T>() - s ) * d010 + s * d011 ;
      //T x10 = ( detail::one<T>() - s ) * d100 + s * d101 ;
      //T x11 = ( detail::one<T>() - s ) * d110 + s * d111 ;
      //T y0  = ( detail::one<T>() - t ) * x00  + t * x01  ;
      //T y1  = ( detail::one<T>() - t ) * x10  + t * x11  ;        
      T x00 = boost::numeric_cast<T>( (d001 - d000)*s + d000 );
      T x01 = boost::numeric_cast<T>( (d011 - d010)*s + d010 );
      T x10 = boost::numeric_cast<T>( (d101 - d100)*s + d100 );
      T x11 = boost::numeric_cast<T>( (d111 - d110)*s + d110 );
      T y0  = boost::numeric_cast<T>( ( x01 -  x00)*t +  x00 );
      T y1  = boost::numeric_cast<T>( ( x11 -  x10)*t +  x10 );
      return boost::numeric_cast<T>( (y1-y0)*u + y0 );
    }

  } // namespace math

} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_MATH_TRILLINEAR_H
#endif
