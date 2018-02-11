#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_STEPPER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_STEPPER_H
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
#include <OpenTissue/dynamics/mbd/mbd_get_velocity_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_set_velocity_vector.h>

#include <OpenTissue/utility/utility_timer.h>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types, typename solver_type >
    class DynamicsStepper 
      : public StepperInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy             math_policy;
      typedef typename math_policy::value_traits            value_traits;
      typedef typename math_policy::index_type              size_type;
      typedef typename math_policy::real_type               real_type;
      typedef typename math_policy::vector_type             vector_type;
      typedef typename math_policy::matrix_type             matrix_type;
      typedef typename math_policy::idx_vector_type         idx_vector_type;
      typedef typename mbd_types::group_type              group_type;

    protected:

      solver_type          m_solver;
      vector_type          m_f_ext; 
      vector_type          m_s;     
      matrix_type          m_invM;
      matrix_type          m_J; 
      idx_vector_type      m_pi;  
      vector_type          m_mu;  
      vector_type          m_b;   
      vector_type          m_rhs; 
      vector_type          m_gamma; 
      vector_type          m_lo;    
      vector_type          m_hi;    
      vector_type          m_x;     
      vector_type          m_u;   
      vector_type          m_tmp;   

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{};

    protected:

      bool            m_warm_starting;  
      bool            m_use_stabilization;
      bool            m_use_friction;
      bool            m_use_bounce;

      real_type       m_query_time;     ///< The time (in seconds) it took to query and retrieve all the basic information needed to assemble the NCP matrices and vectors.
      real_type       m_assembly_time;  ///< The time (in seconds) it took to assemble the matrices and vectors of the NCP formulation.
      real_type       m_solver_time;    ///< The time (in seconds) it took to solve the NCP formulation.
      real_type       m_update_time;    ///< The time (in seconds) it took to udpate the body states. Ie. velocity and position update.
      real_type       m_total_time;     ///< The total time of last invokation of the run-method given in seconds.

    public:

      bool & warm_starting()      {    return m_warm_starting;       }
      bool & use_stabilization()  {    return m_use_stabilization;   }
      bool & use_friction()       {    return m_use_friction;        }
      bool & use_bounce()         {    return m_use_bounce;          }

      bool const & warm_starting()      const {    return m_warm_starting;       }
      bool const & use_stabilization()  const {    return m_use_stabilization;   }
      bool const & use_friction()       const {    return m_use_friction;        }
      bool const & use_bounce()         const {    return m_use_bounce;          }

      real_type const & query_time()    const { return m_query_time;    }
      real_type const & assembly_time() const { return m_assembly_time; }
      real_type const & solver_time()   const { return m_solver_time;   }
      real_type const & update_time()   const { return m_update_time;   }
      real_type const & total_time()    const { return m_total_time;    }

      solver_type const * get_solver() const { return &m_solver; }
      solver_type       * get_solver()       { return &m_solver; }

    public:

      DynamicsStepper()
        : m_warm_starting(false)
        , m_use_stabilization(false)
        , m_use_friction(true)
        , m_use_bounce(true)
      {}

      virtual ~DynamicsStepper(){}

    public:

      void run(group_type & group, real_type const & time_step)
      {
        OpenTissue::utility::Timer<real_type> watch1,watch2;

        watch1.start();
        watch2.start();

        if( time_step < value_traits::zero() )
          throw std::invalid_argument("DynamicsStepper::run(): time step must be non-negative");

        real_type fps = (time_step>value_traits::zero()) ? (value_traits::one()/time_step) : value_traits::zero();

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
          , this->use_stabilization()  
          , this->use_friction()       
          , this->use_bounce()         
          , this->use_stabilization()  // The use_erp parameter only makes sense for this type of stepper if stabilization is used!
          );

        watch1.stop();
        m_query_time = watch1();
        watch1.start();

        mbd::get_velocity_vector(group, m_u);

        mbd::get_external_force_vector(group, m_f_ext, true);
        // m_f_ext *= time_step;
        math_policy::prod(m_f_ext,time_step);

        size_type m;
        size_type n;
        math_policy::get_dimensions(m_J,m,n);

        if(m>0)
        {

          //--- Compute Right Hand Side:  b = J(u + h*inv(M)f_ext) - error
          math_policy::resize(m_b,m);
          if(time_step>value_traits::zero())
          {
            // ublas::noalias(m_b) = ublas::prod( m_J, m_u + vector_type( ublas::prod( m_invM , m_f_ext ) ) ) - m_rhs;
            math_policy::prod(m_invM , m_f_ext, m_u, m_tmp );
            math_policy::prod_minus(m_J, m_tmp, m_rhs, m_b);
          }
          else
          {
            //ublas::noalias(m_b) = ublas::prod(m_J, m_u) - m_rhs;
            math_policy::prod_minus(m_J,m_u,m_rhs,m_b);
          }
          math_policy::prod(m_gamma,fps);

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

        //--- Velocity Update:  u = u + Minv (Jtrans * lambda + h*f_ext);
        if(time_step>value_traits::zero())
        {
          //ublas::noalias(m_u) += ublas::prod( m_invM, vector_type( ublas::prod( ublas::trans(m_J) , m_x ) ) + m_f_ext );
          if(m>0)
          {
            math_policy::prod_trans(m_J, m_x, m_f_ext, m_tmp);
            math_policy::prod_add( m_invM, m_tmp, m_u);
          }
          else
            math_policy::prod_add( m_invM, m_f_ext, m_u);
        }
        else
        {
          //ublas::noalias(m_u) += ublas::prod( m_invM, vector_type( ublas::prod( ublas::trans(m_J) , m_x ) ) );
          if(m>0)
          {
            math_policy::prod_trans(m_J, m_x, m_tmp);
            math_policy::prod_add( m_invM, m_tmp, m_u);
          }
        }
        mbd::set_velocity_vector(group,m_u);

        //--- Position Update
        if(time_step>value_traits::zero())
        {
          mbd::get_position_vector(group, m_s);
          mbd::compute_position_update(group,m_s,m_u,time_step,m_s);
          mbd::set_position_vector(group,m_s);

        }

        watch1.stop();
        watch2.stop();
        m_update_time = watch1();
        m_total_time = watch2();
      }

      void error_correction(group_type & /*group*/)
      {
        throw std::logic_error("DynamicsStepper(): error correction is not defined for this type of stepper");
      }

      void resolve_collisions(group_type & group)
      {
        bool tmp1 = this->use_stabilization();
        this->use_stabilization() = false;

        run(group,value_traits::zero());

        this->use_stabilization() = tmp1;
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_STEPPERS_MBD_DYNAMICS_STEPPER_H
#endif
