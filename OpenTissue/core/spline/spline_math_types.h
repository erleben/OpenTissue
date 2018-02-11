#ifndef OPENTISSUE_CORE_SPLINE_SPLINE_MATH_TYPES_H
#define OPENTISSUE_CORE_SPLINE_SPLINE_MATH_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>

#include <boost/numeric/ublas/matrix.hpp>  
#include <boost/numeric/ublas/vector.hpp>  
  
namespace OpenTissue
{
  namespace spline
  {

    /**
     * Spline Default Math Types.
     * This class is provided as a utility class making it easier for users to get started.
     *
     * The spline library is transparent to the math types. That means that one is
     * not requred to use OpenTissue math types together with the spline library
     * but can use user-customized math types instead.
     *
     * The class assumes that the end-user wants to use the OpenTissue usual
     * math types. If one has a third-party matrix-vector library that one would
     * rather use then one should create ones own math types to be used with the
     * spline library.
     *
     * We have decided to use a math type binder to reduce the uglyness in
     * written template arguments for the functions and classes in the
     * spline library. Thus the main functionality of the type binder is
     * to serve as a collection of types.
     */
    template< typename T, typename I >
    class MathTypes 
      : public OpenTissue::math::BasicMathTypes<T, I>
    {
    protected:

      typedef          OpenTissue::math::BasicMathTypes<T, I>  base_math_types;

    public:

      typedef          boost::numeric::ublas::matrix<T>        matrix_type;
      typedef          boost::numeric::ublas::vector<T>        vector_type;

    };

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_SPLINE_MATH_TYPES_H
#endif
