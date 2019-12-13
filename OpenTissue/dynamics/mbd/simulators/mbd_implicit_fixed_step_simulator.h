#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_IMPLICIT_FIXED_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_IMPLICIT_FIXED_STEP_SIMULATOR_H
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
    * A Implicit Fixed Time Step Simulator.
    *
    * Do not work with First Order Physics!!!
    */
    template< typename mbd_types >
    class ImplicitFixedStepSimulator 
      : public SimulatorInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy                   math_policy;
      typedef typename mbd_types::math_policy::real_type        real_type;
      typedef typename mbd_types::math_policy::vector_type      vector_type;
      typedef typename mbd_types::math_policy::matrix_type      matrix_type;
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
      vector_type                m_f_ext;                ///< Generalized force vector of all bodies.
      matrix_type                m_invM;                 ///< Inverse body massed stored in flat compressed format.
      group_type *               m_all;                  ///< body_type group, used to handle the state of all bodies at once
      group_ptr_container        m_groups;               ///< Temporary Storage, used to hold results from the collision
                                                         ///< detection engine.

    public:

      ImplicitFixedStepSimulator(){}

      virtual ~ImplicitFixedStepSimulator(){}

    public:

      void run(real_type const & time_step)
      {
        m_all = this->get_configuration()->get_all_body_group();
        
        mbd::compute_scripted_motions(*m_all,this->time());
        mbd::get_position_vector(*m_all, m_s);
        mbd::get_velocity_vector(*m_all, m_u);
        mbd::get_external_force_vector(*m_all,m_f_ext,true);
        mbd::get_inverse_mass_matrix(*m_all,m_invM);

        //--- velocity update, compute effect of gravity: h*inv(M)f_ext
        //m_u += ublas::prod(m_invM, m_f_ext)*time_step;
        math_policy::prod_add(m_invM, m_f_ext, m_u, time_step);
        
        //--- fake position update
        mbd::compute_position_update(*m_all,m_s,m_u,time_step,m_ss);
        mbd::set_position_vector(*m_all,m_ss);
        mbd::compute_scripted_motions(*m_all,this->time() + time_step);

        this->get_collision_detection()->run( m_groups );
        for(typename group_ptr_container::iterator tmp=m_groups.begin();tmp!=m_groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_sleepy()->evaluate(group->body_begin(),group->body_end());
          this->get_stepper()->run(*group,time_step);
        }

        //--- get constrained velocities
        mbd::get_velocity_vector(*m_all, m_u);
        //--- perform true position update
        mbd::compute_position_update(*m_all,m_s,m_u,time_step,m_ss);
        mbd::set_position_vector(*m_all,m_ss);
        SimulatorInterface<mbd_types>::update_time(time_step);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_IMPLICIT_FIXED_STEP_SIMULATOR_H
#endif
