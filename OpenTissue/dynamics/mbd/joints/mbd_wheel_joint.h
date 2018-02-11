#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_WHEEL_JOINT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_WHEEL_JOINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_joint_interface.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_angular_joint_limit.h>
#include <OpenTissue/dynamics/mbd/motors/mbd_angular_joint_motor.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {

    /**
    * A Wheel joint_type.
    * body_type A is the Car object and body_type B is the wheel object.
    *
    *
    * For fast spinning wheels it is recommend to use:
    *
    *  if(m_wheel.get_wheel_body())
    *  {
    *    m_wheel.get_wheel_body()->set_finite_rotation_axis(m_wheel.get_motor_axis_world());
    *  }
    *
    * This should be done before any position updates.
    *
    */
    template< typename mbd_types >
    class WheelJoint
      : public JointInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                       math_policy;
      typedef typename mbd_types::math_policy::index_type           size_type;
      typedef typename mbd_types::math_policy::real_type            real_type;
      typedef typename mbd_types::math_policy::vector_range         vector_range;
      typedef typename mbd_types::math_policy::vector3_type         vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type       matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type      quaternion_type;
      typedef typename mbd_types::math_policy::idx_vector_range     idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range         matrix_range;
      typedef typename mbd_types::body_type                         body_type;
      typedef typename mbd_types::math_policy::vector_type          vector_type;
      typedef typename mbd_types::math_policy::value_traits         value_traits;

      typedef AngularJointLimit<mbd_types>    angular_limit_type;
      typedef AngularJointMotor<mbd_types>    angular_motor_type;

    protected:

      matrix3x3_type   m_R;               ///< Rotation matrix from WCS to locla joint frame.
      matrix3x3_type   m_J_A;             ///< Angular part of jacobian of body A.
      matrix3x3_type   m_J_B;             ///< Angular part of jacobian of body B.

      vector3_type     m_anc_A_wcs;       ///< The anchor point wrt. to body A in WCS.
      vector3_type     m_anc_B_wcs;       ///< The anchor point wrt. to body B in WCS.

      vector3_type     m_s;               ///< Suspension axis in WCS.
      vector3_type     m_t1;              ///< Orthognal axis on suspension axis in WCS.
      vector3_type     m_t2;              ///< Orthognal axis on suspension and t1 axes in WCS.
      real_type        m_erp_susp;        ///< Error reduction parameter along suspension axis.
      real_type        m_cfm_susp;        ///< Constraint force mixing parameter along suspension axis.

      real_type        m_cos_theta0;
      real_type        m_sin_theta0;
      real_type        m_cos_theta;
      real_type        m_sin_theta;

      vector3_type     m_v1;               ///< X-axis, used when measuring steering angle.
      vector3_type     m_v2;               ///< Y-axis, used when measuring steering angle.
      vector3_type     m_u;                ///< Constraint axis.
      size_type        m_rows;             ///< Number of jacobian rows.
      vector_type      m_gamma;            ///< Local vector of constraint force mixing terms.
      vector_type      m_solution;         ///< Local solution vector, ie. vector of lagrange multipliers.

      angular_limit_type  *  m_limit;         ///< Steering angle limits
      angular_motor_type  *  m_motor1;        ///< Steering axis motor
      angular_motor_type  *  m_motor2;        ///< Wheel motor

      size_type           m_limit_offset;
      size_type           m_motor_offset1;
      size_type           m_motor_offset2;

    public:

      /**
      * Computes and returns the steering angle.
      *
      * That is (in world coordinates)
      *
      *    1) Initially project the s_motor axis onto a plane orthogonal to s_steering axis
      *
      *               v1 =   s_motor - (s_steering*s_motor)*s_steering
      *               v1 =   v1/||v1||
      *
      *        Let this vector denote an x-axis in the projected plane.
      *
      *    2) Define the y-axis in the projected plane as
      *
      *               v2 = s_steering X v1
      *
      *       Observe that the s_steering joint axis is the z-axis in this
      *       local coordinate frame.
      *
      *    3) During simulation project the current motor axis onto the ``steering axis plane''
      *
      *               s' =   s_motor - (s_steering*s_motor)*s_steering
      *
      *       Now
      *
      *              x =  s'*v1
      *              y =  s'*v2
      *
      *              theta = -atan2(y,x)
      *
      *      Notice that s'*v1 = s_motor*v1, since the projection of s_motor onto
      *      the axes v1 and v2 will have the same coordinates as s'.
      */
      real_type get_steering_angle() const
      {
        using std::atan2;

        quaternion_type Q;
        get_car_body()->get_orientation(Q);
        vector3_type s_motor = conj(Q).rotate( get_motor_axis_world() );

        real_type x = m_v1*s_motor;
        real_type y = m_v2*s_motor;
        return atan2( y , x );
      }

      /**
      * Get Steering Rate (How fast we are turning the steering wheel around the steering axis).
      *
      * @return   The rate.
      */
      real_type get_steering_rate() const
      {
        vector3_type s_wcs = get_steering_axis_world();
        this->m_socketA->get_body()->get_spin(this->w_A);
        this->m_socketB->get_body()->get_spin(this->w_B);
        return s_wcs * (this->w_A - this->w_B);
      }

      /**
      * Get Wheel Rate (How fast are the wheel spinning).
      *
      * @return   The rate.
      */
      real_type get_wheel_rate() const
      {
        vector3_type s_wcs = get_motor_axis_world();
        this->m_socketA->get_body()->get_spin(this->w_A);
        this->m_socketB->get_body()->get_spin(this->w_B);
        return s_wcs * (this->w_A - this->w_B);
      }

      vector3_type get_steering_axis_world() const 
      {
        return this->m_socketA->get_joint_axis_world(); 
      }

      vector3_type get_motor_axis_world() const 
      {
        return this->m_socketB->get_joint_axis_world(); 
      }

      body_type * get_wheel_body() const
      {
        if(this->m_socketB)
          return this->m_socketB->get_body();
        return 0;
      }

      body_type * get_car_body() const
      {
        if(this->m_socketA)
          return this->m_socketA->get_body();
        return 0;
      }

      /**
      * Set Suspension.
      * Only works if constraint force mixing is supported (newton damping).
      *
      * Suspension is modelled by making the steering axis ``soft''
      *
      * @param syspension.
      */
      void set_suspension(real_type const & gamma,real_type const & erp)
      {
        assert(gamma>=value_traits::zero() || !"WheelJoint::set_suspension(): gamma less than 0");
        assert(gamma<=value_traits::one() || !"WheelJoint::set_suspension(): gamma greater than 1");
        assert(erp>=value_traits::zero() || !"WheelJoint::set_suspension(): erp less than 0");
        assert(erp<=value_traits::one() || !"WheelJoint::set_suspension(): ero greater than 1");
        m_cfm_susp = gamma;
        m_erp_susp = erp;
      }

      real_type get_suspension_cfm() const 
      { 
        return m_cfm_susp; 
      }

      real_type get_suspension_erp() const   
      {
        return m_erp_susp;   
      }

      void set_steering_limit(angular_limit_type const & limit) 
      { 
        m_limit = const_cast<angular_limit_type*>(&limit); 
      }
      
      void set_steering_motor(angular_motor_type const & motor) 
      { 
        m_motor1 = const_cast<angular_motor_type*>(&motor); 
      }

      void set_wheel_motor(angular_motor_type const & motor) 
      { 
        m_motor2 = const_cast<angular_motor_type*>(&motor); 
      }

    public:

      WheelJoint()
        : m_erp_susp( value_traits::zero() )
        , m_cfm_susp( value_traits::zero() )
        , m_rows(0)
        , m_limit(0)
        , m_motor1(0)
        , m_motor2(0)
      {}

      virtual ~WheelJoint() {}

    public:

      void evaluate()
      {
        using std::sqrt;

        m_rows = 0;

        if(!(this->m_socketA && this->m_socketB))//--- joint_type must be disconnected nothing to evaluate
          return;

        m_rows = 4;

        quaternion_type Q;

        this->m_socketA->get_body()->get_orientation(Q);
        vector3_type r_a = Q.rotate(  this->m_socketA->get_anchor_local());

        this->m_socketB->get_body()->get_orientation(Q);
        vector3_type r_b = Q.rotate(  this->m_socketB->get_anchor_local());

        //---
        //---  Ordinary in WCS coordinate frame we have
        //---
        //---   v_a + w_a X r_a - (v_b + w_b X r_b)  = 0
        //---   v_a - star(r_a) w_a - v_b + star(r_b) w_b  = 0
        //---
        //---  In local joint frame (JF) given by the basis (s,t1,t2) we have
        //---
        //---   R v_a - R star(r_a) w_a - R v_b + R star(r_b) w_b  = 0
        //---
        //---  where
        //---
        //---           |  s^T   |
        //---     R =   |  t1^T  |     : WCS -> JF
        //---           |  t2^T  |
        //---
        //--- Now
        //---
        //---  J_ang_a =  - R star(r_a)  =    star(r_a) R
        //---
        //--- from which we see
        //---
        //---
        //---                 |  (r_a X s)^T   |
        //---     J_ang_a =   |  (r_a X t1)^T  |
        //---                 |  (r_a X t2)^T  |
        //---
        //--- Similar we find (notice the minus sign):
        //---
        //---                  |  (r_b X s)^T   |
        //---     J_ang_b =  - |  (r_b X t1)^T  |
        //---                  |  (r_b X t2)^T  |
        //---
        //--- The linear parts are simply J_lin_a = R and  J_lin_b = - R
        //---
        //---
        //---  The error term is usually computed as
        //---
        //---      fps*erp (  r_anc_b_wcs  -   r_anc_a_wcs )
        //---
        //--- In the rotated frame this is
        //---
        //---      fps*erp R (  r_anc_b_wcs  -   r_anc_a_wcs )
        //---
        //--- Furthermore suspension modelling usually means that the erp-factor along
        //--- the steering axis is differnt than the one along the other axes. Which
        //--- means:
        //---
        //---              | fps*erp_susp  s*(  r_anc_b_wcs  -   r_anc_a_wcs ) |
        //---  b_error =   | fps*erp_norm t1*(  r_anc_b_wcs  -   r_anc_a_wcs ) |
        //---              | fps*erp_norm t2*(  r_anc_b_wcs  -   r_anc_a_wcs ) |
        //---
        m_s  = get_steering_axis_world();
        m_t1 = normalize( orthogonal(m_s) );
        m_t2 = cross(m_s , m_t1);
        m_R(0,0) =  m_s(0);     m_R(0,1) =  m_s(1);   m_R(0,2) =  m_s(2);
        m_R(1,0) = m_t1(0);     m_R(1,1) = m_t1(1);   m_R(1,2) = m_t1(2);
        m_R(2,0) = m_t2(0);     m_R(2,1) = m_t2(1);   m_R(2,2) = m_t2(2);
        m_J_A  =  - m_R*star(r_a);
        m_J_B  =    m_R*star(r_b);

        m_anc_A_wcs = this->m_socketA->get_anchor_world();
        m_anc_B_wcs = this->m_socketB->get_anchor_world();

        m_u = cross( m_s , get_motor_axis_world());  //--- Constrained axis
        m_sin_theta = sqrt(m_u*m_u);
        m_cos_theta = m_s * get_motor_axis_world();


        m_limit_offset  = 0;
        m_motor_offset1 = 0;
        m_motor_offset2 = 0;
        if(m_limit)
        {
          m_limit->evaluate(get_steering_axis_world(),get_steering_angle());
          if(m_limit->is_active())
          {
            m_limit_offset = m_rows;
            ++m_rows;
          }
        }
        if(m_motor1)
        {
          m_motor1->evaluate(get_steering_axis_world());
          if(m_motor1->is_active())
          {
            m_motor_offset1 = m_rows;
            ++m_rows;
          }
        }
        if(m_motor2)
        {
          m_motor2->evaluate(get_motor_axis_world());
          if(m_motor2->is_active())
          {
            m_motor_offset2 = m_rows;
            ++m_rows;
          }
        }
      }

      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"WheelJoint::get_linear_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"WheelJoint::get_linear_jacobian_A(): incorrect dimensions");
        J(0,0) = m_R(0,0); J(0,1) = m_R(0,1); J(0,2) = m_R(0,2);
        J(1,0) = m_R(1,0); J(1,1) = m_R(1,1); J(1,2) = m_R(1,2);
        J(2,0) = m_R(2,0); J(2,1) = m_R(2,1); J(2,2) = m_R(2,2);
        J(3,0) = value_traits::zero();
        J(3,1) = value_traits::zero();
        J(3,2) = value_traits::zero();
        if(m_limit_offset)
          m_limit->get_linear_jacobian_A(J,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_linear_jacobian_A(J,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_linear_jacobian_A(J,m_motor_offset2);
      }

      void get_linear_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"WheelJoint::get_linear_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"WheelJoint::get_linear_jacobian_B(): incorrect dimensions");
        J(0,0) = -m_R(0,0); J(0,1) = -m_R(0,1); J(0,2) = -m_R(0,2);
        J(1,0) = -m_R(1,0); J(1,1) = -m_R(1,1); J(1,2) = -m_R(1,2);
        J(2,0) = -m_R(2,0); J(2,1) = -m_R(2,1); J(2,2) = -m_R(2,2);
        J(3,0) =  value_traits::zero();
        J(3,1) =  value_traits::zero();
        J(3,2) =  value_traits::zero();
        if(m_limit_offset)
          m_limit->get_linear_jacobian_B(J,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_linear_jacobian_B(J,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_linear_jacobian_B(J,m_motor_offset2);
      }

      void get_angular_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"WheelJoint::get_angular_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"WheelJoint::get_angular_jacobian_A(): incorrect dimensions");
        J(0,0) = m_J_A(0,0); J(0,1) = m_J_A(0,1); J(0,2) = m_J_A(0,2);
        J(1,0) = m_J_A(1,0); J(1,1) = m_J_A(1,1); J(1,2) = m_J_A(1,2);
        J(2,0) = m_J_A(2,0); J(2,1) = m_J_A(2,1); J(2,2) = m_J_A(2,2);
        J(3,0) = m_u(0);             J(3,1) = m_u(1);             J(3,2) = m_u(2);
        if(m_limit_offset)
          m_limit->get_angular_jacobian_A(J,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_angular_jacobian_A(J,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_angular_jacobian_A(J,m_motor_offset2);
      }

      void get_angular_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"WheelJoint::get_angular_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"WheelJoint::get_angular_jacobian_B(): incorrect dimensions");
        J(0,0) = m_J_B(0,0); J(0,1) = m_J_B(0,1); J(0,2) = m_J_B(0,2);
        J(1,0) = m_J_B(1,0); J(1,1) = m_J_B(1,1); J(1,2) = m_J_B(1,2);
        J(2,0) = m_J_B(2,0); J(2,1) = m_J_B(2,1); J(2,2) = m_J_B(2,2);
        J(3,0) = -m_u(0);           J(3,1) = -m_u(1);           J(3,2) = -m_u(2);
        if(m_limit_offset)
          m_limit->get_angular_jacobian_B(J,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_angular_jacobian_B(J,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_angular_jacobian_B(J,m_motor_offset2);
      }

      void get_stabilization_term(vector_range & b_error)const
      {
        assert(b_error.size()==m_rows || !"WheelJoint::get_stabilization_term(): incorrect dimensions");

        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();
        real_type k_cor_susp = k_cor;
        if(m_erp_susp)
          k_cor_susp = m_erp_susp*this->get_frames_per_second();

        vector3_type diff = m_anc_B_wcs - m_anc_A_wcs;

        b_error(0) = k_cor_susp * (m_s *diff);
        b_error(1) = k_cor      * (m_t1*diff);
        b_error(2) = k_cor      * (m_t2*diff);
        //(theta0-theta) can be computed using the following small-angle-difference
        //  approximation:
        //    theta0-theta ~= tan(theta0-theta)
        //                  = sin(theta0-theta)/cos(theta0-theta)
        //                  = (c*s0 - s*c0) / (c*c0 + s*s0)
        //                  = c*s0 - s*c0         assuming c*c0 + s*s0 ~= 1
        //  where c = cos(theta), s = sin(theta)
        //        c0 = cos(theta0), s0 = sin(theta0)
        //
        b_error(3) = k_cor*(m_cos_theta0*m_sin_theta - m_sin_theta0*m_cos_theta);

        if(m_limit_offset)
          m_limit->get_stabilization_term(b_error,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_stabilization_term(b_error,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_stabilization_term(b_error,m_motor_offset2);
      }

      void get_low_limits(vector_range & lo)const
      {
        assert(lo.size()==m_rows || !"WheelJoint::get_low_limits(): incorrect dimensions");
        lo(0) = OpenTissue::math::detail::lowest<real_type>();
        lo(1) = OpenTissue::math::detail::lowest<real_type>();
        lo(2) = OpenTissue::math::detail::lowest<real_type>();
        lo(3) = OpenTissue::math::detail::lowest<real_type>();
        if(m_limit_offset)
          m_limit->get_low_limits(lo,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_low_limits(lo,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_low_limits(lo,m_motor_offset2);
      }

      void get_high_limits(vector_range & hi)const
      {
        assert(hi.size()==m_rows || !"WheelJoint::get_high_limits(): incorrect dimensions");
        hi(0) = OpenTissue::math::detail::highest<real_type>();
        hi(1) = OpenTissue::math::detail::highest<real_type>();
        hi(2) = OpenTissue::math::detail::highest<real_type>();
        hi(3) = OpenTissue::math::detail::highest<real_type>();
        if(m_limit_offset)
          m_limit->get_high_limits(hi,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_high_limits(hi,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_high_limits(hi,m_motor_offset2);
      }

      void get_dependency_indices(idx_vector_range & dep)const
      {
        assert(dep.size()==m_rows || !"WheelJoint::get_dependency_indices(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          dep(i) = OpenTissue::math::detail::highest<size_type>();
      }

      void get_dependency_factors(vector_range & factors)const
      {
        assert(factors.size()==m_rows || !"WheelJoint::get_dependency_factors(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          factors(i) = value_traits::zero();
      }

      void set_regularization(vector_range const & gamma)
      {
        assert(gamma.size()==m_rows || !"WheelJoint::set_regularization(): incorrect dimensions");
        math_policy::resize( m_gamma, 4);
        for(size_type i=0;i<4;++i)
        {
          assert(gamma(i)<=value_traits::one()  || !"WheelJoint::set_regularization(): gamma greater than 1");
          assert(gamma(i)>=value_traits::zero() || !"WheelJoint::set_regularization(): gamma less than 0");
          m_gamma(i) = gamma(i);
        }
        if(m_limit_offset)
          m_limit->set_regularization(gamma,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->set_regularization(gamma,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->set_regularization(gamma,m_motor_offset2);
      }

      void get_regularization(vector_range & gamma)const
      {
        assert(gamma.size()==m_rows || !"WheelJoint::get_regularization(): incorrect dimensions");
        if(m_gamma.size()==0)
        {
          for(size_type i=0;i<4;++i)
            gamma(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<4;++i)
            gamma(i) = m_gamma(i);
        }
        if(m_cfm_susp)
          gamma(0) = m_cfm_susp;
        if(m_limit_offset)
          m_limit->get_regularization(gamma,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_regularization(gamma,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_regularization(gamma,m_motor_offset2);
      }

      void set_solution(vector_range const & solution)
      {
        assert(solution.size()==m_rows || !"WheelJoint::set_solution(): incorrect dimensions");
        math_policy::resize( m_solution, 4);
        for(size_type i=0;i<4;++i)
          m_solution(i) = solution(i);
        if(m_limit_offset)
          m_limit->set_solution(solution,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->set_solution(solution,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->set_solution(solution,m_motor_offset2);
      }

      void get_solution(vector_range & solution)const
      {
        assert(solution.size()==m_rows || !"WheelJoint::get_solution(): incorrect dimensions");
        if(m_solution.size()==0)
        {
          for(size_type i=0;i<4;++i)
            solution(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<4;++i)
            solution(i) = m_solution(i);
        }
        if(m_limit_offset)
          m_limit->get_solution(solution,m_limit_offset);
        if(m_motor_offset1)
          m_motor1->get_solution(solution,m_motor_offset1);
        if(m_motor_offset2)
          m_motor2->get_solution(solution,m_motor_offset2);
      }

      void calibration()
      {
        using std::sqrt;

        vector3_type s_steering = get_steering_axis_world();
        vector3_type s_motor    = get_motor_axis_world();

        //--- verify that joint was set up correctly.
        assert(s_steering != s_motor || !"WheelJoint::calibration(): steering and motor axis were the same");

        m_u = cross(s_steering , s_motor);
        m_sin_theta0 = sqrt(m_u*m_u);
        m_cos_theta0 = s_steering * s_motor;

        m_v1 =  normalize( s_motor - (s_steering * s_motor) * s_steering  );
        m_v2 =  cross(s_steering, m_v1);

        quaternion_type Q;
        get_car_body()->get_orientation(Q);
        m_v1 = conj(Q).rotate( m_v1 );
        m_v2 = conj(Q).rotate( m_v2 );
      }
    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_WHEEL_JOINT_H
#endif
