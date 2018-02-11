#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_CONSTRAINT_BASED_SHOCK_PROPAGATION_STEPPER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_CONSTRAINT_BASED_SHOCK_PROPAGATION_STEPPER_H
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

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * This stepper combines a velocity based complementarity formulation with stack propagation.
    */
    template< typename mbd_types, typename solver_type  >
    class ConstraintBasedShockPropagationStepper 
      : public StepperInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::real_type           real_type;
      typedef typename mbd_types::math_policy::value_traits        value_traits;
      typedef typename mbd_types::group_type                       group_type;
      typedef StackPropagation<mbd_types>                     propagation_algorithm;
      typedef DynamicsStepper<mbd_types,solver_type>          dynamics_algorithm;
      typedef FirstOrderStepper<mbd_types,solver_type>        correction_algorithm;

    public:

      class node_traits 
        : public dynamics_algorithm::node_traits
        , public correction_algorithm::node_traits
        , public propagation_algorithm::node_traits
      {};

      class edge_traits
        : public dynamics_algorithm::edge_traits
        , public correction_algorithm::edge_traits
        , public propagation_algorithm::edge_traits
      {};

      class constraint_traits
        : public dynamics_algorithm::constraint_traits
        , public correction_algorithm::constraint_traits
        , public propagation_algorithm::constraint_traits
      {};

    protected:

      struct StepperFunctor
      {
        dynamics_algorithm     m_dynamics;
        correction_algorithm   m_correction;
        real_type              m_h;         

        StepperFunctor()
        {
          m_dynamics.warm_starting()          = false;
          m_dynamics.use_stabilization()      = false;
          m_dynamics.use_friction()           = true;
          m_dynamics.use_bounce()             = true;
          m_dynamics.get_solver()->set_max_iterations(10);

          m_correction.warm_starting()       = false;
          m_correction.use_external_forces() = false;
          m_correction.use_erp()             = false;
          m_correction.get_solver()->set_max_iterations(10);
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

      propagation_algorithm m_propagation;
      StepperFunctor        m_stepper_functor;
      real_type             m_fraction;

    public:

      ConstraintBasedShockPropagationStepper()
        : m_fraction(value_traits::zero())
      {}

      virtual ~ConstraintBasedShockPropagationStepper(){};

    public:

      /**
      * Set Fraction.
      *
      * @param fraction     The new fraction value (the weight of dynamics vs. propagation algorithm).
      */
      void set_fraction(real_type const & fraction)
      {
        if(m_fraction< value_traits::zero())
          throw std::invalid_argument("ConstraintBasedShockPropagationStepper::set_fraction(): value was negtaive");

        if(m_fraction> value_traits::one())
          throw std::invalid_argument("ConstraintBasedShockPropagationStepper::set_fraction(): value larger then one");

        m_fraction = fraction;
      }

    public:


      void run(group_type & group,real_type const & time_step)
      {
        if(time_step<=value_traits::zero())
          throw std::invalid_argument( "ConstraintBasedShockPropagationStepper::run() time step was non-positive" );
        if(m_fraction< value_traits::zero())
          throw std::invalid_argument("ConstraintBasedShockPropagationStepper::run() m_fraction was non-positive");

        m_stepper_functor.m_dynamics.run(group,m_fraction*time_step);

        m_stepper_functor.m_h = (value_traits::one()-m_fraction)*time_step;

        m_propagation.run(
          group
          , m_stepper_functor
          , typename propagation_algorithm::fixate_tag()
          , typename propagation_algorithm::upward_tag()
          );

        //std::cout << " |layers| = " << m_propagation.size() << std::endl;
      }

      void resolve_collisions(group_type & group)
      {
        m_stepper_functor.m_dynamics.resolve_collisions(group);
      }

      void error_correction(group_type & group)
      {
        m_stepper_functor.m_correction.error_correction(group);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_CONSTRAINT_BASED_SHOCK_PROPAGATION_STEPPER_H
#endif
