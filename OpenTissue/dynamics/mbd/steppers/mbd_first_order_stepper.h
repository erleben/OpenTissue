#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_FIRST_ORDER_STEPPER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_FIRST_ORDER_STEPPER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_stepper_interface.h>

#include <OpenTissue/dynamics/mbd/mbd_get_ncp_formulation.h>
#include <OpenTissue/dynamics/mbd/mbd_get_cached_solution_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_set_cached_solution_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_external_force_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_position_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_set_position_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_position_update.h>

#include <OpenTissue/utility/utility_timer.h>

namespace OpenTissue
{
  namespace mbd
  {
    template< typename mbd_types, typename solver_type  >
    class FirstOrderStepper 
      : public StepperInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy          math_policy;
      typedef typename math_policy::value_traits         value_traits;
      typedef typename math_policy::index_type           size_type;
      typedef typename math_policy::real_type            real_type;
      typedef typename math_policy::vector_type          vector_type;
      typedef typename math_policy::matrix_type          matrix_type;
      typedef typename math_policy::idx_vector_type      idx_vector_type;
      typedef typename mbd_types::group_type           group_type;

    protected:

      solver_type            m_solver;
      vector_type            m_f_ext; 
      vector_type            m_s;     
      matrix_type            m_invM;
      matrix_type            m_J; 
      idx_vector_type        m_pi;  
      vector_type            m_mu;  
      vector_type            m_b;   
      vector_type            m_rhs; 
      vector_type            m_gamma; 
      vector_type            m_lo;    
      vector_type            m_hi;    
      vector_type            m_x;
      vector_type            m_tmp;

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{};

    protected:

      bool            m_warm_starting;
      bool            m_use_external_forces;
      bool            m_use_erp;              ///< If set to true all constraints and contacts use their own erp-parameter value. If set to value all constraints and contacts use a value of value_traits::one() as their erp-value.

      real_type       m_query_time;     ///< The time (in seconds) it took to query and retrieve all the basic information needed to assemble the NCP matrices and vectors.
      real_type       m_assembly_time;  ///< The time (in seconds) it took to assemble the matrices and vectors of the NCP formulation.
      real_type       m_solver_time;    ///< The time (in seconds) it took to solve the NCP formulation.
      real_type       m_update_time;    ///< The time (in seconds) it took to udpate the body states, that is the position update.
      real_type       m_total_time;     ///< The total time of last invokation of the run-method given in seconds.

    public:

      bool & warm_starting()       {    return m_warm_starting;       }
      bool & use_external_forces() {    return m_use_external_forces;  }
      bool & use_erp()             {    return m_use_erp;  }

      bool const & warm_starting()       const {    return m_warm_starting;       }
      bool const & use_external_forces() const {    return m_use_external_forces;  }
      bool const & use_erp()             const {    return m_use_erp;  }

      real_type const & query_time()    const { return m_query_time;    }
      real_type const & assembly_time() const { return m_assembly_time; }
      real_type const & solver_time()   const { return m_solver_time;   }
      real_type const & update_time()   const { return m_update_time;   }
      real_type const & total_time()    const { return m_total_time;    }

      solver_type const * get_solver() const { return &m_solver; }
      solver_type       * get_solver()       { return &m_solver; }

    public:

      FirstOrderStepper()
        : m_warm_starting(false)
        , m_use_external_forces(true)
        , m_use_erp(false)
      {}

      virtual ~FirstOrderStepper(){}

    public:

      void run(group_type & group,real_type const & time_step)
      {
        OpenTissue::utility::Timer<real_type> watch1,watch2;

        watch1.start();
        watch2.start();

        assert( time_step>value_traits::zero() || !"FirstOrderStepper::run(): time step must be positive");

        real_type fps = value_traits::one()/time_step;

        bool const use_stabilization   = true;
        bool const use_friction        = false;
        bool const use_bounce          = false;

        mbd::get_ncp_formulation(
          group
          , fps
          , m_J
          , m_invM
          , m_lo
          , m_hi
          , m_pi
          , m_mu
          , m_gamma
          , m_rhs
          , use_stabilization
          , use_friction
          , use_bounce
          , this->use_erp()
          );

        watch1.stop();
        m_query_time = watch1();
        watch1.start();

        size_type m;
        size_type n;
        math_policy::get_dimensions(m_J,m,n);

        if(m>0)
        {

          //--- Compute Right Hand Side: b = - error + J*inv(M)*F_ext*h
          math_policy::resize(m_b,m);

          if(this->use_external_forces())
          {
            mbd::get_external_force_vector(group, m_f_ext, false);
            //m_f_ext *= time_step;
            //ublas::noalias(m_b) = ublas::prod(m_J , vector_type( ublas::prod(m_invM,m_f_ext) ) ) - m_rhs;
            math_policy::prod(m_f_ext, time_step);
            math_policy::prod(m_invM, m_f_ext, m_tmp);
            math_policy::prod_minus( m_J, m_tmp, m_rhs, m_b);
          }
          else
          {
            //m_b =  - m_rhs;
            math_policy::assign_minus(m_rhs, m_b);
          }

          watch1.stop();
          m_assembly_time = watch1();
          watch1.start();

          math_policy::resize(m_x,m);

          if(this->warm_starting())
            mbd::get_cached_solution_vector(group,m,m_x);

          m_solver.run( m_J, m_invM, m_gamma, m_b, m_lo, m_hi, m_pi, m_mu, m_x );

          if(this->warm_starting())
            mbd::set_cached_solution_vector(group,m,m_x);

          watch1.stop();
          m_solver_time = watch1();
          watch1.start();

        }

        vector_type P;
        math_policy::resize(P,n);

        if(this->use_external_forces())
        {
          //--- Add constraint forces, P =inv(M)(J^T lambda + F_ext*h)
          //ublas::noalias(P) = ublas::prod( m_invM, vector_type( ublas::prod( ublas::trans(m_J) , m_x) )   +   m_f_ext );
          if(m>0)
          {
            math_policy::prod_trans(m_J, m_x, m_f_ext, m_tmp);
            math_policy::prod( m_invM, m_tmp, P);
          }
          else
            math_policy::prod( m_invM, m_f_ext, P);

        }
        else
        {
          //noalias(P) = prod( m_invM, vector_type( prod( trans(m_J) , m_x) ) );
          if(m>0)
          {
            math_policy::prod_trans(m_J, m_x, m_tmp);
            math_policy::prod( m_invM, m_tmp, P);
          }
        }

        mbd::get_position_vector(group, m_s);
        mbd::compute_position_update(group, m_s, P, time_step, m_s);
        mbd::set_position_vector(group, m_s);

        watch1.stop();
        watch2.stop();
        m_update_time = watch1();
        m_total_time = watch2();
      }

      void error_correction(group_type & group)
      { 
        bool tmp1 = this->warm_starting();
        bool tmp2 = this->use_external_forces();
        bool tmp3 = this->use_erp();

        this->warm_starting()       = false;
        this->use_external_forces() = false;
        this->use_erp()             = false;

        run(group,value_traits::one());

        this->use_erp()             = tmp3;
        this->use_external_forces() = tmp2;
        this->warm_starting()       = tmp1;
      }

      void resolve_collisions(group_type & /*group*/)
      {
        assert(false || !"FirstOrderStepper::resolve_collisions(): collision resolving is not defined for this type of stepper");
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_FIRST_ORDER_STEPPER_H
#endif
