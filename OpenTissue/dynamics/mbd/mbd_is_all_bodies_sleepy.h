#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
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
  namespace mbd
  {

    template<typename indirect_body_iterator>
    bool is_all_bodies_sleepy(indirect_body_iterator begin, indirect_body_iterator end)
    {
      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"is_all_bodies_sleepy(): body was not active");
        if(!body->is_sleepy())
          return false;

      }
      return true;
    }

    template<typename group_type>
    bool is_all_bodies_sleepy(group_type const & group)
    {
      return is_all_bodies_sleepy(group.body_begin(),group.body_end());
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_IS_ALL_BODIES_SLEEPY_H
#endif
