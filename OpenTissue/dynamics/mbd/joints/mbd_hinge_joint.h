#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_HINGE_JOINT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_HINGE_JOINT_H
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
    template< typename mbd_types >
    class HingeJoint 
      : public JointInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                       math_policy;
      typedef typename mbd_types::math_policy::index_type           size_type;
      typedef typename mbd_types::math_policy::real_type            real_type;
      typedef typename mbd_types::math_policy::vector3_type         vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type       matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type      quaternion_type;
      typedef typename mbd_types::math_policy::vector_range         vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range     idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range         matrix_range;
      typedef typename mbd_types::math_policy::vector_type          vector_type;
      typedef typename mbd_types::math_policy::value_traits         value_traits;

      typedef AngularJointLimit<mbd_types>    angular_limit_type;
      typedef AngularJointMotor<mbd_types>    angular_motor_type;


    protected:

      vector3_type           m_s_A_wcs;         ///< The joint axe wrt. to body A in WCS.
      vector3_type           m_s_B_wcs;         ///< The joint axe wrt. to body B in WCS.
      vector3_type           m_t1;              ///< A vector orthogonal to the joint axe (in WCS).
      vector3_type           m_t2;              ///< Another vector orthogonal to the joint axe and t1 (in WCS).
      vector3_type           m_u;               ///< Error correction rotation axe. Ie. if joint is misaligned this vector is used to determine an rotation axe which can be used to align the joint along.
      matrix3x3_type         m_star_anc_A;      ///< The cross product operator of the anchor point wrt. to body A rotated (but not translated) into WCS.
      matrix3x3_type         m_star_anc_B;      ///< The cross product operator of the anchor point wrt. to body B rotated (but not translated) into WCS.
      vector3_type           m_anc_A_wcs;       ///< The anchor point wrt. to body A in WCS.
      vector3_type           m_anc_B_wcs;       ///< The anchor point wrt. to body B in WCS.
      quaternion_type        m_Q_initial;       ///< The initial relative orientation between the two bodies (wrt. WCS) i.e. the rotation that aligns body A with body B.
      quaternion_type        m_Q_initial_conj;  ///< The conjugate of Q_initial.
      angular_limit_type  *  m_limit;
      angular_motor_type  *  m_motor;

      size_type           m_rows;
      size_type           m_motor_offset;
      size_type           m_limit_offset;

      vector_type            m_gamma;           ///< Local vector of constraint force mixing terms.
      vector_type            m_solution;        ///< Local solution vector, ie. vector of lagrange multipliers.

    public:

      /**
      * Get Rotation Angle.
      *
      *
      *  This angle tells how much body B have rotated around the joint
      *  axis (postive value means CCW, negative means CW) from its
      *  initial placement.
      *
      *
      * Think of it as if you have fixated body A so only body B is allowd to move.
      *
      *
      * Let BF_B' indicate the initial orientation of body B's body frame
      * and BF_B the current orientation, now BF_A should be thought of as
      * being immoveable ie. constant.
      *
      *   Q_initial : BF_A -> BF_B'
      *   Q_cur     : BF_A -> BF_B
      *
      * Now we must have the relation
      *
      *   Q_rel Q_initial = Q_cur
      *
      * From which we deduce
      *
      *   Q_rel = Q_cur conj(Q_initial)
      *
      * And we see that
      *
      *   Q_rel : BF_B' -> BF_B
      *
      * That is how much the body frame of body B have rotated (measured
      * with respect to the fixed body frame A).
      *
      * @return     The angle in radians, which the joint have rotated around
      *             its joints axe away from its initial position (at calibration
      *             time).
      */
      real_type get_hinge_angle() const
      {
        assert(this->m_socketA || !"HingeJoint::get_hinge_angle(): socket A was null");
        assert(this->m_socketB || !"HingeJoint::get_hinge_angle(): socket B was null");

        //--- Given the hinge axis and the initial relative orientation between the bodies
        //--- (Q_initial), return the relative rotation angle.
        //---
        //--- Where the initial relative orientation corresponds to an angle of zero.
        //---
        //--- This will not return the correct angle if the bodies rotate along any axis
        //--- other than the given hinge axis.
        quaternion_type Q_cur,Q_rel;

        //--- Compute current relative orientation between bodies
        quaternion_type Q_A,Q_B;
        this->m_socketA->get_body()->get_orientation(Q_A);
        this->m_socketB->get_body()->get_orientation(Q_B);

        Q_cur = conj(Q_B) % Q_A;

        //--- Now ``subtract'' the initial relative orientation
        //--- what is left is the change of joint position. The
        //--- formula is as follows:
        //---
        //---                   Q_rel * Q_initial = Q_cur
        //--- Q_rel * Q_initial * conj(Q_initial) = Q_cur conj(Q_initial)
        //---                               Q_rel = Q_cur conj(Q_initial)
        Q_rel = Q_cur % m_Q_initial_conj;

        return get_angle(Q_rel,this->m_socketB->get_joint_axis_local());
      }

      /**
      * Get Angle Rate.
      * This method computes the (signed) speed by which the joint
      * angle changes its radial position around the joint axe.
      *
      * @return   The angle rate.
      */
      real_type get_hinge_angle_rate() const
      {
        assert(this->m_socketA || !"HingeJoint::get_hinge_angle_rate(): socket A was null");
        assert(this->m_socketB || !"HingeJoint::get_hinge_angle_rate(): socket B was null");

        vector3_type s_wcs = this->m_socketA->get_joint_axis_world();

        vector3_type w_A,w_B;
        this->m_socketA->get_body()->get_spin(w_A);
        this->m_socketB->get_body()->get_spin(w_B);
        return s_wcs * (w_A - w_B);
      }

      vector3_type get_hinge_axis_world() const 
      {  
        return this->m_socketA->get_joint_axis_world(); 
      }

      void set_limit(angular_limit_type const & limit) 
      { 
        m_limit = const_cast<angular_limit_type*>(&limit); 
      }
      
      void set_motor(angular_motor_type const & motor) 
      {
        m_motor = const_cast<angular_motor_type*>(&motor); 
      }

    public:

      HingeJoint()
        : m_limit(0)
        , m_motor(0)
        , m_rows(0)
      {}

      virtual ~HingeJoint() {}

    public:

      void evaluate()
      {
        m_rows = 0;
        if(!(this->m_socketA && this->m_socketB))
          return;

        m_s_A_wcs = this->m_socketA->get_joint_axis_world();
        m_s_B_wcs = this->m_socketB->get_joint_axis_world();

        quaternion_type Q;
        this->m_socketA->get_body()->get_orientation(Q);
        m_star_anc_A = star(    Q.rotate(  this->m_socketA->get_anchor_local()) );
        this->m_socketB->get_body()->get_orientation(Q);
        m_star_anc_B = star( Q.rotate(  this->m_socketB->get_anchor_local()) );

        m_anc_A_wcs = this->m_socketA->get_anchor_world();
        m_anc_B_wcs = this->m_socketB->get_anchor_world();

        m_t1 = this->m_socketA->get_axis1_world();
        m_t2 = this->m_socketA->get_axis2_world();
        //--- Compute ``correction'' rotation axe used to set up b_error
        m_u = cross(m_s_A_wcs , m_s_B_wcs);

        m_rows = 5;
        m_limit_offset = 0;
        m_motor_offset = 0;
        if(m_limit)
        {
          m_limit->evaluate(get_hinge_axis_world(),get_hinge_angle());
          if(m_limit->is_active())
          {
            m_limit_offset = m_rows;
            ++m_rows;
          }
        }
        if(m_motor)
        {
          m_motor->evaluate(get_hinge_axis_world());
          if(m_motor->is_active())
          {
            m_motor_offset = m_rows;
            ++m_rows;
          }
        }

      }

      size_type get_number_of_jacobian_rows() const 
      { 
        return m_rows; 
      }

      void get_linear_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"HingeJoint::get_linear_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"HingeJoint::get_linear_jacobian_A(): incorrect dimensions");
        J(0,0) = value_traits::one();  J(0,1) = value_traits::zero(); J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero(); J(1,1) = value_traits::one();  J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero(); J(2,1) = value_traits::zero(); J(2,2) = value_traits::one();
        J(3,0) = value_traits::zero(); J(3,1) = value_traits::zero(); J(3,2) = value_traits::zero();
        J(4,0) = value_traits::zero(); J(4,1) = value_traits::zero(); J(4,2) = value_traits::zero();
        if(m_limit_offset)
          m_limit->get_linear_jacobian_A(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_linear_jacobian_A(J,m_motor_offset);
      }

      void get_linear_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"HingeJoint::get_linear_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"HingeJoint::get_linear_jacobian_B(): incorrect dimensions");
        J(0,0) = -value_traits::one();  J(0,1) =  value_traits::zero(); J(0,2) =  value_traits::zero();
        J(1,0) =  value_traits::zero(); J(1,1) = -value_traits::one();  J(1,2) =  value_traits::zero();
        J(2,0) =  value_traits::zero(); J(2,1) =  value_traits::zero(); J(2,2) = -value_traits::one();
        J(3,0) =  value_traits::zero(); J(3,1) =  value_traits::zero(); J(3,2) =  value_traits::zero();
        J(4,0) =  value_traits::zero(); J(4,1) =  value_traits::zero(); J(4,2) =  value_traits::zero();
        if(m_limit_offset)
          m_limit->get_linear_jacobian_B(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_linear_jacobian_B(J,m_motor_offset);
      }

      void get_angular_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"HingeJoint::get_angular_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"HingeJoint::get_angular_jacobian_A(): incorrect dimensions");
        J(0,0) = -m_star_anc_A(0,0); J(0,1) = -m_star_anc_A(0,1); J(0,2) = -m_star_anc_A(0,2);
        J(1,0) = -m_star_anc_A(1,0); J(1,1) = -m_star_anc_A(1,1); J(1,2) = -m_star_anc_A(1,2);
        J(2,0) = -m_star_anc_A(2,0); J(2,1) = -m_star_anc_A(2,1); J(2,2) = -m_star_anc_A(2,2);
        J(3,0) = m_t1(0);             J(3,1) = m_t1(1);           J(3,2) = m_t1(2);
        J(4,0) = m_t2(0);             J(4,1) = m_t2(1);           J(4,2) = m_t2(2);
        if(m_limit_offset)
          m_limit->get_angular_jacobian_A(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_angular_jacobian_A(J,m_motor_offset);
      }

      void get_angular_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"HingeJoint::get_angular_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"HingeJoint::get_angular_jacobian_B(): incorrect dimensions");
        J(0,0) = m_star_anc_B(0,0); J(0,1) = m_star_anc_B(0,1); J(0,2) = m_star_anc_B(0,2);
        J(1,0) = m_star_anc_B(1,0); J(1,1) = m_star_anc_B(1,1); J(1,2) = m_star_anc_B(1,2);
        J(2,0) = m_star_anc_B(2,0); J(2,1) = m_star_anc_B(2,1); J(2,2) = m_star_anc_B(2,2);
        J(3,0) = -m_t1(0);           J(3,1) = -m_t1(1);         J(3,2) = -m_t1(2);
        J(4,0) = -m_t2(0);           J(4,1) = -m_t2(1);         J(4,2) = -m_t2(2);

        if(m_limit_offset)
          m_limit->get_angular_jacobian_B(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_angular_jacobian_B(J,m_motor_offset);

      }

      void get_stabilization_term(vector_range & b_error)const
      {
        assert(b_error.size()==m_rows || !"HingeJoint::get_stabilization_term(): incorrect dimensions");
        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();

        b_error(0) = k_cor*(m_anc_B_wcs(0) - m_anc_A_wcs(0));
        b_error(1) = k_cor*(m_anc_B_wcs(1) - m_anc_A_wcs(1));
        b_error(2) = k_cor*(m_anc_B_wcs(2) - m_anc_A_wcs(2));
        b_error(3) = k_cor*(m_t1*m_u);
        b_error(4) = k_cor*(m_t2*m_u);

        if(m_limit_offset)
          m_limit->get_stabilization_term(b_error,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_stabilization_term(b_error,m_motor_offset);
      }


      void get_low_limits(vector_range & lo)const
      {
        assert(lo.size()==m_rows || !"HingeJoint::get_low_limits(): incorrect dimensions");
        lo(0) = OpenTissue::math::detail::lowest<real_type>();
        lo(1) = OpenTissue::math::detail::lowest<real_type>();
        lo(2) = OpenTissue::math::detail::lowest<real_type>();
        lo(3) = OpenTissue::math::detail::lowest<real_type>();
        lo(4) = OpenTissue::math::detail::lowest<real_type>();
        if(m_limit_offset)
          m_limit->get_low_limits(lo,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_low_limits(lo,m_motor_offset);

      }

      void get_high_limits(vector_range & hi)const
      {
        assert(hi.size()==m_rows || !"HingeJoint::get_high_limits(): incorrect dimensions");
        hi(0) = OpenTissue::math::detail::highest<real_type>();
        hi(1) = OpenTissue::math::detail::highest<real_type>();
        hi(2) = OpenTissue::math::detail::highest<real_type>();
        hi(3) = OpenTissue::math::detail::highest<real_type>();
        hi(4) = OpenTissue::math::detail::highest<real_type>();

        if(m_limit_offset)
          m_limit->get_high_limits(hi,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_high_limits(hi,m_motor_offset);
      }

      void get_dependency_indices(idx_vector_range & dep)const
      {
        assert(dep.size()==m_rows || !"HingeJoint::get_dependency_indices(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          dep(i) = OpenTissue::math::detail::highest<size_type>();
      }

      void get_dependency_factors(vector_range & factors)const
      {
        assert(factors.size()==m_rows || !"HingeJoint::get_dependency_factors(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          factors(i) = value_traits::zero();
      }

      void set_regularization(vector_range const & gamma)
      {
        assert(gamma.size()==m_rows || !"HingeJoint::set_regularization(): incorrect dimensions");
        math_policy::resize( m_gamma, 5);
        for(size_type i=0;i<5;++i)
        {
          assert(gamma(i)<=value_traits::one()  || !"HingeJoint::set_regularization(): gamma was greater than 1");
          assert(gamma(i)>=value_traits::zero() || !"HingeJoint::set_regularization(): gamma was less than 0");
          m_gamma(i) = gamma(i);
        }
        if(m_limit_offset)
          m_limit->set_regularization(gamma,m_limit_offset);
        if(m_motor_offset)
          m_motor->set_regularization(gamma,m_motor_offset);
      }

      void get_regularization(vector_range & gamma)const
      {
        assert(gamma.size()==m_rows || !"HingeJoint::get_regularization(): incorrect dimensions");
        if(m_gamma.size()==0)
        {
          for(size_type i=0;i<5;++i)
            gamma(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<5;++i)
            gamma(i) = m_gamma(i);
        }
        if(m_limit_offset)
          m_limit->get_regularization(gamma,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_regularization(gamma,m_motor_offset);
      }

      void set_solution(vector_range const & solution)
      {
        assert(solution.size()==m_rows || !"HingeJoint::set_solution(): incorrect dimensions");

        math_policy::resize( m_solution, 5);
        for(size_type i=0;i<5;++i)
          m_solution(i) = solution(i);
        if(m_limit_offset)
          m_limit->set_solution(solution,m_limit_offset);
        if(m_motor_offset)
          m_motor->set_solution(solution,m_motor_offset);
      }

      void get_solution(vector_range & solution)const
      {
        assert(solution.size()==m_rows || !"HingeJoint::get_solution(): incorrect dimensions");
        if(m_solution.size()==0)
        {
          for(size_type i=0;i<5;++i)
            solution(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<5;++i)
            solution(i) = m_solution(i);
        }
        if(m_limit_offset)
          m_limit->get_solution(solution,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_solution(solution,m_motor_offset);
      }

      void calibration()
      {
        assert(this->m_socketA || !"HingeJoint::calibration(): socket A was null");
        assert(this->m_socketB || !"HingeJoint::calibration(): socket B was null");

        quaternion_type Q_A,Q_B;
        this->m_socketA->get_body()->get_orientation(Q_A);
        this->m_socketB->get_body()->get_orientation(Q_B);

        //---  Q_initial : BF_A -> BF_B
        m_Q_initial = conj(Q_B) % Q_A;
        m_Q_initial_conj = conj(m_Q_initial);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_HINGE_JOINT_H
#endif 
