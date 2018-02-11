#ifndef OPENTISSUE_CORE_FUNCTION_IMPLICIT_FUNCTION_H
#define OPENTISSUE_CORE_FUNCTION_IMPLICIT_FUNCTION_H
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

  namespace function
  {
    /**
     * Implicit Function interface
     *
     * If you have an implicit function (surface) and want to be consistent with other IFs in OT,
     * then derive from this interface and implement evaluate() and gradient().
     * Note: This interface is often employed as an integrated part of 
     *       the signed distance function interface, @see SignedDistanceFunction
     * 
     * @param math_types  The (almost) mandatory math types
     */
    template<typename math_types >
    class ImplicitFunction
    {
    public:

      virtual ~ImplicitFunction() {}

    public:

      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;

    public:

      /**
       * Evaluate the implicit function that defines the surface.
       * All implicit functions must be designed in such way that:
       * evaluate(x) < 0  -->  x is inside the surface (or inside the volume),
       * evaluate(x) = 0  -->  x is on the surface, and
       * evaluate(x) > 0  -->  x is outside the surface.
       */
      virtual real_type     evaluate(vector3_type const & x) const = 0;

      /**
       * The gradient of the implicit function evaluated in x.
       * The gradient will always point from low to high values wrt evaluate(x).
       */
      virtual vector3_type  gradient(vector3_type const & x) const = 0;

    };

  }  // namespace function

}  // namespace OpenTissue

//OPENTISSUE_CORE_FUNCTION_IMPLICIT_FUNCTION_H
#endif
