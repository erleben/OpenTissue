#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_PROJECTION_STEPPER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_PROJECTION_STEPPER_H
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

namespace OpenTissue
{

  namespace mbd
  {
    /**
    * A velocity-based complementatiry formulation using projection
    * to handle constraint errors.
    */
    template< typename mbd_types, typename solver_type  >
    class DynamicsProjectionStepper
      : public StepperInterface<mbd_types>
    {
    protected:

      typedef typename mbd_types::math_policy::real_type           real_type;
      typedef typename mbd_types::math_policy::vector3_type        vector3_type;
      typedef typename mbd_types::math_policy::vector_type         vector_type;
      typedef typename mbd_types::math_policy::idx_vector_type     idx_vector_type;
      typedef typename mbd_types::group_type                       group_type;
      typedef typename mbd_types::math_policy::index_type          size_type;

      typedef DynamicsStepper<mbd_types,solver_type>         dynamics_algorithm;
      typedef FirstOrderStepper<mbd_types,solver_type>       error_correction_algorithm;

    protected:

      dynamics_algorithm          m_dynamics;      
      error_correction_algorithm m_correction;     

    public:

      class node_traits 
        : public dynamics_algorithm::node_traits
        , public error_correction_algorithm::node_traits
      {};

      class edge_traits
        : public dynamics_algorithm::edge_traits
        , public error_correction_algorithm::edge_traits
      {};

      class constraint_traits
        : public dynamics_algorithm::constraint_traits
        , public error_correction_algorithm::constraint_traits
      {};

    public:

      DynamicsProjectionStepper()
      {
        // Setup the dynamics stepper
        m_dynamics.warm_starting()          = false;
        m_dynamics.use_stabilization()      = false;
        m_dynamics.use_friction()           = true;
        m_dynamics.use_bounce()             = true;
        m_dynamics.get_solver()->set_max_iterations(10);

        // Setup the error correction stepper
        m_correction.warm_starting()       = false;
        m_correction.use_external_forces() = false;
        m_correction.use_erp()             = false;
        m_correction.get_solver()->set_max_iterations(10);
      }

      virtual ~DynamicsProjectionStepper(){}

    public:

      void run(group_type & group,real_type const & time_step)
      {
        m_dynamics.run(group,time_step);
        m_correction.error_correction(group);
      }

      void error_correction(group_type & group)
      { 
        m_correction.error_correction(group);
      }

      void resolve_collisions(group_type & group)
      {
        m_dynamics.resolve_collisions(group);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_PROJECTION_STEPPER_H
#endif
