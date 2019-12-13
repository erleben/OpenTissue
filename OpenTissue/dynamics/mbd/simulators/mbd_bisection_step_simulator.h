#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_BISECTION_STEP_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_BISECTION_STEP_SIMULATOR_H
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
    * A Bisection Time-based Simulator.
    * This simulator tries to find the time of impact using bisection, it
    * then ask the stepper to resolve collisions at that time, before it
    * tries to simulate forward again.
    *
    * This kind of simulator was designed with acceleration based formulations in mind.
    *
    * This kind of simulator requires the stepper method to have three
    * methods: A run method, a resolve_collisions method,
    * and a error_correction method.
    *
    * 09-09-2004 KE: This simulator have not been tested yet....
    *
    */
    template< typename mbd_types >
    class BisectionStepSimulator  
      : public SimulatorInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::index_type       size_type;
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
      vector_type                m_u;                    ///< Generalized velocity vector of all bodies.
      group_type *               m_all;                  ///< body_type group, used to handle the state of all bodies at once, see
                                                         ///< record_state() and rewind_state() methods
      group_ptr_container        m_groups;               ///< Temporary Storage, used to hold results from the collision detection engine.

    public:

      BisectionStepSimulator()
        : m_all(0)
      {}

      virtual ~BisectionStepSimulator() {}

    public:

      void run(real_type const & time_step)
      {
        //--- Make sure that collision detection engine do not perform unnecessary work
        this->get_collision_detection()->set_short_circuiting(true);

        size_type max_iteration = 100;        //--- Upper limit on the number of times a backtrack is allowed
        size_type iteration     = 0;          //--- Backtrack iteration counter
        real_type tst_underflow = 1e-3;       //--- Underflow threshold test value (or smallest allowable step-size)
        real_type final         = time_step;  //--- The final simulation time we are trying to reach.
        real_type cur           = 0;          //--- The current simulation time we are at.
        real_type next          = time_step;  //--- The simulation time that we hope to be able to simulate forward to.

        mbd::compute_scripted_motions(*m_all,this->time());
        
        this->get_collision_detection()->run(m_groups);

        resolve_collisions(m_groups);        
        do
        {
          record_state();
          run(m_groups,next-cur);
          bool penetration = this->get_collision_detection()->run(m_groups);
          if(penetration)
          {
            if(iteration>max_iteration)
            {
              next = final;
              resolve_collisions(m_groups);
              error_correction(m_groups);
            }
            else
            {
              rewind_state();
              real_type tmp = next;
              next = cur + ((next - cur)/2.0f);
              if(std::fabs(tmp-next)<tst_underflow)
              {
                cur = next;
                next = final;
                resolve_collisions(m_groups);
              }
              ++iteration;
            }
          }
          else
          {
            cur  = next;
            next = final;
            resolve_collisions(m_groups);
          }
        }
        while(cur<final);
        SimulatorInterface<mbd_types>::update_time(time_step);
      }

    protected:

      void error_correction(group_ptr_container & groups)
      {
        for(typename group_ptr_container::iterator tmp=groups.begin();tmp!=groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_stepper()->error_correction(*group);
        }
      }

      void resolve_collisions(group_ptr_container & groups)
      {
        for(typename group_ptr_container::iterator tmp=groups.begin();tmp!=groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_stepper()->resolve_collisions(*group);
        }
      }

      void run(group_ptr_container & groups,real_type const & time_step)
      {
        m_all = this->get_configuration()->get_all_body_group();
        mbd::compute_scripted_motions(*m_all, this->time()+time_step);

        for(typename group_ptr_container::iterator tmp=groups.begin();tmp!=groups.end();++tmp)
        {
          group_type * group = (*tmp);
          this->get_sleepy()->evaluate(group->body_begin(),group->body_end());
          if(!mbd::is_all_bodies_sleepy(*group))
            this->get_stepper()->run(*group,time_step);
        }
      }

      void record_state()
      {
        m_all = this->get_configuration()->get_all_body_group();
        mbd::get_position_vector(*m_all, m_s);
        mbd::get_velocity_vector(*m_all, m_u);
      }

      void rewind_state()
      {
        if(m_all)
        {
          mbd::set_position_vector(*m_all,m_s);
          mbd::set_velocity_vector(*m_all,m_u);
        }
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_SIMULATORS_MBD_BISECTION_STEP_SIMULATOR_H
#endif
