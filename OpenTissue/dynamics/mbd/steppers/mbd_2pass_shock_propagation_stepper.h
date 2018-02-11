#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_2PASS_SHOCK_PROPAGATION_STEPPER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_2PASS_SHOCK_PROPAGATION_STEPPER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_stepper_interface.h>

#include <OpenTissue/dynamics/mbd/steppers/mbd_dynamics_stepper.h>
#include <OpenTissue/dynamics/mbd/steppers/mbd_first_order_stepper.h>
#include <OpenTissue/dynamics/mbd/mbd_stack_propagation.h>
#include <OpenTissue/dynamics/mbd/mbd_is_all_bodies_sleepy.h>
#include <OpenTissue/dynamics/mbd/collision_laws/mbd_frictional_newton_collision_law_policy.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * A 2 Pass Shock Propagation Constraint Based Stepper Method.
    *
    * This method makes two runs over all stack layers, one
    * going top-to-bottom and another going from bottom-to-top.
    *
    * During first pass only impulse is transfered, in the second
    * pass bottom-most objects are artificially fixated resulting
    * in correction of velocities before doing the actual position
    * update.
    */
    template< typename mbd_types, typename solver_type  >
    class TwoPassShockPropagationStepper 
      : public StepperInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::real_type             real_type;
      typedef typename mbd_types::math_policy::value_traits          value_traits;
      typedef typename mbd_types::math_policy::vector3_type          vector3_type;
      typedef typename mbd_types::math_policy::vector_type           vector_type;
      typedef typename mbd_types::math_policy::idx_vector_type       idx_vector_type;
      typedef typename mbd_types::group_type                         group_type;

      typedef StackPropagation<mbd_types>                      propagation_algorithm;
      typedef DynamicsStepper<mbd_types,solver_type>           dynamics_algorithm;
      typedef FirstOrderStepper<mbd_types,solver_type>         error_correction_algorithm;

      typedef typename collision_laws::FrictionalNewtonCollisionLawPolicy      collision_law_policy;
      typedef IterateOnceCollisionResolver<mbd_types, collision_law_policy > collision_resolver_algorithm;

    public:

      class node_traits 
        : public dynamics_algorithm::node_traits
        , public error_correction_algorithm::node_traits
        , public propagation_algorithm::node_traits
      {};

      class edge_traits
        : public dynamics_algorithm::edge_traits
        , public error_correction_algorithm::edge_traits
        , public propagation_algorithm::edge_traits
      {};

      class constraint_traits
        : public dynamics_algorithm::constraint_traits
        , public error_correction_algorithm::constraint_traits
        , public propagation_algorithm::constraint_traits
      {};

    protected:

      struct ErrorFunctor
      {
        error_correction_algorithm m_correction;

        ErrorFunctor()
        {
          m_correction.warm_starting()       = false;
          m_correction.use_external_forces() = false;
          m_correction.use_erp()             = false;
          m_correction.get_solver()->set_max_iterations(5);
        }

        void operator()(group_type & layer)
        {
          if(mbd::is_all_bodies_sleepy(layer))
            return;
          m_correction.error_correction(layer);
        }
      };

      struct DynamicsFunctor
      {
        dynamics_algorithm  m_dynamics;
        real_type          m_h;

        DynamicsFunctor()
        {
          m_dynamics.warm_starting()          = false;
          m_dynamics.use_stabilization()      = false;
          m_dynamics.use_friction()           = true;
          m_dynamics.use_bounce()             = true;
          m_dynamics.get_solver()->set_max_iterations(5);
        }

        void operator()(group_type & layer)
        {
          if(mbd::is_all_bodies_sleepy(layer))
            return;
          m_dynamics.run(layer,m_h);
        }
      };

      struct StepperFunctor
      {
        dynamics_algorithm          m_dynamics; 
        error_correction_algorithm m_correction;
        real_type                  m_h;         

        StepperFunctor()
        {
          m_correction.warm_starting()       = false;
          m_correction.use_external_forces() = false;
          m_correction.use_erp()             = false;
          m_correction.get_solver()->set_max_iterations(5);

          m_dynamics.warm_starting()          = false;
          m_dynamics.use_stabilization()      = false;
          m_dynamics.use_friction()           = true;
          m_dynamics.use_bounce()             = true;
          m_dynamics.get_solver()->set_max_iterations(5);
        }

        void operator()(group_type & layer)
        {
          if(mbd::is_all_bodies_sleepy(layer))
            return;
          m_correction.error_correction(layer);
          m_dynamics.run(layer,m_h);
        }
      };

    protected:

      propagation_algorithm        m_propagation;      ///< Stack Propagation Algorithm to be used.
      StepperFunctor               m_stepper_functor;  ///< Stepper function to be used together with propagation algorithm.
      collision_resolver_algorithm m_resolver;         ///< Collision resolver.
      DynamicsFunctor              m_dynamics_functor; ///< Pure Dynamics function to be used together with propagation algorithm.
      ErrorFunctor                 m_error_functor;    ///< Pure Error function to be used together with propagation algorithm.

    public:

      TwoPassShockPropagationStepper(){}
      virtual ~TwoPassShockPropagationStepper(){}

    public:

      void run(group_type & group,real_type const & time_step)
      {
        m_dynamics_functor.m_h = value_traits::zero();
        m_propagation.run(  
            group
          , m_dynamics_functor 
          , typename propagation_algorithm::downward_tag() 
          );

        m_dynamics_functor.m_h = time_step;
        m_propagation.rerun(  
            group
          , m_dynamics_functor 
          , typename propagation_algorithm::fixate_tag()
          , typename propagation_algorithm::upward_tag() 
          );
      }

      void error_correction(group_type & group)
      {
        m_propagation.rerun(  
            group
          , m_error_functor 
          , typename propagation_algorithm::fixate_tag()
          , typename propagation_algorithm::upward_tag() 
          );
      }

      void resolve_collisions(group_type & group)
      {
        m_dynamics_functor.m_dynamics.resolve_collisions(group);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_2PASS_SHOCK_PROPAGATION_STEPPER_H
#endif
