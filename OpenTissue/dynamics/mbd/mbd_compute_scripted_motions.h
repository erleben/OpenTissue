#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
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

    /**
    * Compute Scripted Motion.
    * This method will update the scripted motion of all scripted bodies
    * in the body sequence. That is the state of each scripted body is updated by
    * invoking the run method on the attached scripted motion.
    *
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param time       The time at which the scripted motion should be evaluated.
    */
    template<typename indirect_body_iterator,typename real_type>
    void compute_scripted_motions(indirect_body_iterator begin, indirect_body_iterator end, real_type const & time)
    {
      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"get_position_vector(): body was not active");
        if(body->is_scripted())
          body->compute_scripted_motion(time);
      }
    }

    template<typename group_type,typename real_type>
    void compute_scripted_motions(group_type const & group, real_type const & time)
    {
      compute_scripted_motions(group.body_begin(),group.body_end(),time);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_SCRIPTED_MOTIONS_H
#endif
