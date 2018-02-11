#ifndef OPENTISSUE_CORE_FUNCTION_SIGNED_DISTANCE_FUNCTION_H
#define OPENTISSUE_CORE_FUNCTION_SIGNED_DISTANCE_FUNCTION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/function/function_implicit_function.h>


namespace OpenTissue
{

  namespace function
  {

    // 2007-06-24 kenny: doxygen class documentation maybe with example of usage?
    /**
     * Signed Distance Function interface
     *
     */
    template<typename math_types >
    class SignedDistanceFunction : public ImplicitFunction< math_types >
    {
    public:

      virtual ~SignedDistanceFunction() {}

    public:

      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;

    public:

      // 2007-06-24 kenny: I have mixed feelings about our naming... should the names reflect whether computations are done inside a function call or not? For instance compute_signed_distance will tell caller that this call might be expensive????

      /**
       * The unit normal of the implicit function evaluated in x.
       */
      virtual vector3_type  normal(vector3_type const & x) const = 0;

      /**
       * The signed distance from x to the surface,
       * e.g. -normal*signed_distance is the shortest direction to the surface from any x.
       */
      virtual real_type     signed_distance(vector3_type const & x) const = 0;

    };

  }  // namespace function

}  // namespace OpenTissue

//OPENTISSUE_CORE_FUNCTION_SIGNED_DISTANCE_FUNCTION_H
#endif
