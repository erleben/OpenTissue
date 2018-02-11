#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_EXPLICIT_FIXED_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_EXPLICIT_FIXED_STEP_SIMULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_simulator_interface.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * A Explicit Fixed Time Step Simulator.
    * This is a default simple fixed time step simulator. It can be made
    * to work with any kind of collision detection engine and stepping
    * algorithm through use of policies.
    *
    * Also works with First Order Physics.
    */
    template< typename mbd_types >
    class ExplicitFixedStepSimulator 
      : public SimulatorInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::real_type         real_type;    
      typedef typename mbd_types::group_type                     group_type;
      typedef typename mbd_types::group_ptr_container            group_ptr_container;

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{};

    protected:

      group_ptr_container        m_groups;

    public:

      ExplicitFixedStepSimulator(){}

      virtual ~ExplicitFixedStepSimulator(){}

    public:

      void run(real_type const & time_step)
      {
        mbd::compute_scripted_motions(*(this->get_configuration()->get_all_body_group()),this->time());

        this->get_collision_detection()->run( m_groups );

        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);

          this->get_sleepy()->evaluate(group->body_begin(),group->body_end());

          if(!mbd::is_all_bodies_sleepy(*group))
            this->get_stepper()->run(*group,time_step);
        }
        update_time(time_step);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_EXPLICIT_FIXED_STEP_SIMULATOR_H
#endif
