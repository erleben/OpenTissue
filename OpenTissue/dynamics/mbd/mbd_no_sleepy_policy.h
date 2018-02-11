#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * No Sleepy Policy.
    * This is an implementation of a default sleepy
    * policy
    */
    template< typename mbd_types>
    struct NoSleepyPolicy
    {
      typedef typename mbd_types::group_type             group_type;
      typedef typename group_type::indirect_body_iterator           indirect_body_iterator;
      
      class node_traits{};
      class edge_traits{};
      class constraint_traits{};
      
      void evaluate(indirect_body_iterator /*begin*/, indirect_body_iterator /*end*/){}

      void clear(){}
    };

  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_NO_SLEEPY_POLICY_H
#endif
