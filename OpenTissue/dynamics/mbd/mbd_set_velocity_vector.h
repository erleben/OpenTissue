#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
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
    * Set velocities and spins of a sequence of bodies.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param u       This vector holds the extracted generalized velocity vector.
    */
    template<typename indirect_body_iterator,typename vector_type>
    void set_velocity_vector(indirect_body_iterator begin, indirect_body_iterator end, vector_type const & u)
    {
      typedef typename indirect_body_iterator::value_type  body_type;
      typedef typename body_type::vector3_type    vector3_type;
      typedef typename vector_type::size_type     size_type;

      vector3_type V,W;
      size_type n = std::distance(begin,end);
      
      assert(u.size() == 6*n || !"set_velocity_vector(): u has incorrect dimension");

      typename vector_type::const_iterator uval = u.begin();
      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"set_velocity_vector(): body was not active");
        V(0) = *uval++;
        V(1) = *uval++;
        V(2) = *uval++;
        W(0) = *uval++;
        W(1) = *uval++;
        W(2) = *uval++;
        assert(is_number(V(0)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(V(1)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(V(2)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(0)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(1)) || !"set_velocity_vector(): non number encountered");
        assert(is_number(W(2)) || !"set_velocity_vector(): non number encountered");
        if(!body->is_scripted())
        {
          body->set_velocity(V);
          body->set_spin(W);
        }
      }
    }

    template<typename group_type,typename vector_type>
    void set_velocity_vector(group_type const & group, vector_type & u)
    {
      set_velocity_vector(group.body_begin(),group.body_end(),u);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_VELOCITY_VECTOR_H
#endif
