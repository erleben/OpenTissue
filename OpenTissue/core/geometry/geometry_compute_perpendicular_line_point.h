#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PERPENDICULAR_LINE_POINT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PERPENDICULAR_LINE_POINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>


namespace OpenTissue
{
  namespace geometry
  {

// 2007-07-24 kenny: Where is the unit-test for this new piece of code?
//   2007-07-24 micky: Unit-test, are you serious? Ain't this a bit overkill?
//     2007-07-24 kenny: Yes I am very serious... Too many times we have got into trouble by not making sure small simple pieces of code are not consistent and working (last time were when we re-factored vector3 class)
//     2007-09-18 kenny: he he, just to add some salt to the wound, do you recall what happended with the ``clamp'' unit-test:-)
//       2007-09-25 micky: I'll add the unit test ASAP.
    /**
    * Compute Perpendicular Line Point in nD.
    *
    * @param p0   First point on line.
    * @param p1   Second point on line.
    * @param x    Requesting point to find the perpendicular point on the line.
    * @param t    The line parameter t in the form of p0+t(p1-p0).
    *
    */
// 2007-07-23 kenny: why not drop second type argument and simply use typename vector3_type::value_type instead?
//   2007-07-23 micky: can't really see how t will then be usable for fx. vector3<int>.
//     2007-07-24 kenny: Right now you can actually have vector_type = vector3<double> and real_type = int. If the code is re-written to be 
//                       template<typename vector_type>
//                       void compute_perpendicular_line_point( vector_type const & p0
//                                                                   , vector_type const & p1
//                                                                   , vector_type const & x
//                                                                   , typename vector_type::value_type & t
//                                                                   )
//                       then you are forcing the caller to use the same precision as whatever vector type he is calling. At least you will know that all the types inside your code are the same.
// 
//       2007-07-24 micky: My last comment still stands, and frankly it's quite realistic. Besides if the caller uses int as a real type he's a victim of his own incompetence.
//         2007-07-24 kenny: So why not make sure the compiler tells the caller that he is doing something stupid?
//           2007-09-25 micky: Should we try to wrap up this one? I world like both template args to stay. They both are used implicitly from the caller.
//             2007-07-26 kenny: Fine, but maybe you should add a note about this ``implicitly'' handling of types, either as source comments or as doxygen comments? Maybe this way it will be easy for others to track down future problems of implicit type conversions?
    template<typename vector_type, typename real_type>
    void compute_perpendicular_line_point( vector_type const & p0
                                                , vector_type const & p1
                                                , vector_type const & x
                                                , real_type & t
                                                )
    {
      vector_type const v = p1-p0;
      real_type const vl2 = inner_prod(v, v);
      assert(vl2 > math::ValueTraits<real_type>::zero() || !"p0 and p1 does not define a line as they are coinciding!");
      vector_type const d = p0-x;
      t = -inner_prod(d, v)/vl2;
    }

  }  // namespace geometry

}  // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PERPENDICULAR_LINE_POINT_H
#endif
