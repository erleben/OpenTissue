#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEMI_IMPLICIT_FIXED_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEMI_IMPLICIT_FIXED_STEP_SIMULATOR_H
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
    * A Semi-Implicit Fixed Time Step Simulator.
    *
    * Do not work with First Order Physics!!!
    */
    template< typename mbd_types  >
    class SemiImplicitFixedStepSimulator  
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

      vector_type                m_s;                    ///< Generalized position vector of all bodies.
      vector_type                m_ss;                   ///< Generalized position vector of all bodies.
      vector_type                m_u;                    ///< Generalized velocity vector of all bodies.
      vector_type                m_uu;                   ///< Generalized velocity vector of all bodies.
      vector_type                m_F;                    ///< Generalized force vector of all bodies.
      group_type *               m_all;                  ///< body_type group, used to handle the state of all bodies at once
      group_ptr_container        m_groups;               ///< Temporary Storage, used to hold results from the collision
                                                         ///< detection engine.
    public:

      SemiImplicitFixedStepSimulator(){}

      virtual ~SemiImplicitFixedStepSimulator(){}

    public:

      void run(real_type const & time_step)
      {
        m_all = this->get_configuration()->get_all_body_group();

        mbd::compute_scripted_motions(*m_all,this->time());
        mbd::get_position_vector(*m_all, m_s);
        mbd::get_velocity_vector(*m_all, m_u);

        //--- position update
        mbd::compute_position_update(*m_all,m_s,m_u,time_step,m_ss);
        mbd::set_position_vector(*m_all,m_ss);

        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_sleepy()->evaluate(group->body_begin(),group->body_end());
          if(!mbd::is_all_bodies_sleepy(*group))
            this->get_stepper()->run(*group,time_step);
        }
        mbd::get_velocity_vector(*m_all, m_u);
        mbd::compute_position_update(*m_all,m_s,m_u,time_step,m_ss);
        mbd::set_position_vector(*m_all,m_ss);
        mbd::compute_scripted_motions(*m_all,this->time() + time_step);

        update_time(time_step);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_SEMI_IMPLICIT_FIXED_STEP_SIMULATOR_H
#endif
