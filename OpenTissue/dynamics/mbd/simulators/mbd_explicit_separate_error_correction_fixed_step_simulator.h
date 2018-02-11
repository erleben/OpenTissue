#ifndef OPENTISSUE_DYNAMICS_MBD_SIMULATORS_MBD_EXPLICIT_SEPARATE_ERROR_CORRECTION_FIXED_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_SIMULATORS_MBD_EXPLICIT_SEPARATE_ERROR_CORRECTION_FIXED_STEP_SIMULATOR_H
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
    * Explicit Separate Error Correction Fixed Time Step Simulator.
    * Performs an explicit fixed time step followed by
    * a separate error correction step.
    *
    * Thus two collision detection queries are performed for each time-step.
    */
    template< typename mbd_types >
    class ExplicitSeparateErrorCorrectionFixedStepSimulator 
      : public SimulatorInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::real_type        real_type;
      typedef typename mbd_types::math_policy::vector_type      vector_type;
      typedef typename mbd_types::group_type                    group_type;
      typedef typename mbd_types::group_ptr_container           group_ptr_container;

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{};

    protected:

      group_ptr_container        m_groups;               ///< Temporary Storage, used to hold results from the collision detection engine.

    public:

      ExplicitSeparateErrorCorrectionFixedStepSimulator(){}

      virtual ~ExplicitSeparateErrorCorrectionFixedStepSimulator(){}

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
        //--- Anti rippling...!
        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_stepper()->error_correction(*group);
        }
        update_time(time_step);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_SIMULATORS_MBD_EXPLICIT_SEPARATE_ERROR_CORRECTION_FIXED_STEP_SIMULATOR_H
#endif
