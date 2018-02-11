#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_POSITION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_POSITION_VECTOR_H
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
    * Set positions and orientations of a sequence of bodies.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param s       This vector holds the extracted generalized position vector.
    */
    template<typename indirect_body_iterator,typename vector_type>
    void set_position_vector(indirect_body_iterator begin, indirect_body_iterator end, vector_type const & s)
    {
      typedef typename indirect_body_iterator::value_type     body_type;
      typedef typename body_type::vector3_type       vector3_type;
      typedef typename body_type::quaternion_type    quaternion_type;
      typedef typename vector_type::size_type        size_type;
      vector3_type r;
      quaternion_type q;
      size_type n = std::distance(begin,end);      

      assert(s.size() == 7*n || !"set_position_vector(): u has incorrect dimension");

      typename vector_type::const_iterator sval = s.begin();
      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"set_position_vector(): body was not active");
        r(0) = *sval++;
        r(1) = *sval++;
        r(2) = *sval++;
        q.s() = *sval++;
        q.v()(0) = *sval++;
        q.v()(1) = *sval++;
        q.v()(2) = *sval++;
        assert(is_number(r(0)) || !"set_position_vector(): non number encountered");
        assert(is_number(r(1)) || !"set_position_vector(): non number encountered");
        assert(is_number(r(2)) || !"set_position_vector(): non number encountered");
        assert(is_number(q.s()) || !"set_position_vector(): non number encountered");
        assert(is_number(q.v()(0)) || !"set_position_vector(): non number encountered");
        assert(is_number(q.v()(1)) || !"set_position_vector(): non number encountered");
        assert(is_number(q.v()(2)) || !"set_position_vector(): non number encountered");
        if(!body->is_scripted())
        {
          body->set_position(r);
          body->set_orientation(q);
        }
      }
    }

    template<typename group_type,typename vector_type>
    void set_position_vector(group_type const & group, vector_type & s)
    {
      set_position_vector(group.body_begin(),group.body_end(),s);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_POSITION_VECTOR_H
#endif
