#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_VELOCITY_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_VELOCITY_VECTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Extract Generalized Velocity Vector.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param u       Upon return this vector holds the extracted
    *                generalized velocity vector.
    */
    template<typename indirect_body_iterator,typename vector_type>
    void get_velocity_vector(indirect_body_iterator begin, indirect_body_iterator end, vector_type & u)
    {
      typedef typename indirect_body_iterator::value_type  body_type;
      typedef typename body_type::math_policy              math_policy;
      typedef typename body_type::vector3_type             vector3_type;
      typedef typename vector_type::size_type              size_type;

      vector3_type V,W;
      size_type n = std::distance(begin,end);
      
      math_policy::resize( u, 6*n);      

      typename vector_type::iterator uval = u.begin();
      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"get_velocity_vector(): body was not active");

        body->get_velocity(V);
        body->get_spin(W);

        assert(is_number(V(0)) || !"get_velocity_vector(): non number encountered");
        assert(is_number(V(1)) || !"get_velocity_vector(): non number encountered");
        assert(is_number(V(2)) || !"get_velocity_vector(): non number encountered");
        assert(is_number(W(0)) || !"get_velocity_vector(): non number encountered");
        assert(is_number(W(1)) || !"get_velocity_vector(): non number encountered");
        assert(is_number(W(2)) || !"get_velocity_vector(): non number encountered");

        *uval++ = V(0);
        *uval++ = V(1);
        *uval++ = V(2);
        *uval++ = W(0);
        *uval++ = W(1);
        *uval++ = W(2);
      }
    }

    template<typename group_type,typename vector_type>
    void get_velocity_vector(group_type const & group, vector_type & u)
    {
      get_velocity_vector(group.body_begin(),group.body_end(),u);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_VELOCITY_VECTOR_H
#endif
