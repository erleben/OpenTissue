#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_IDENTITY_PRECONDITIONER_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_IDENTITY_PRECONDITIONER_H
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
  namespace math
  {
    namespace big
    {

        /**
        * An identity preconditioner.
        * This preconditioner does not do anything.
        */
        class IdentityPreconditioner
        {
        public:

          template<typename matrix_type, typename vector_type>
          void operator()(
              matrix_type const & P
            , vector_type & e
            , vector_type const & r
            ) const 
          { 
            e = r; 
          }
        };

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_IDENTITY_PRECONDITIONER_H
#endif
