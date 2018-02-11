#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_SLIDER_JOINT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_SLIDER_JOINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_joint_interface.h>
#include <OpenTissue/dynamics/mbd/limits/mbd_linear_joint_limit.h>
#include <OpenTissue/dynamics/mbd/motors/mbd_linear_joint_motor.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {

    template<  typename mbd_types  >
    class SliderJoint 
      : public JointInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy                     math_policy;
      typedef typename mbd_types::math_policy::index_type         size_type;
      typedef typename mbd_types::math_policy::real_type          real_type;
      typedef typename mbd_types::math_policy::vector3_type       vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type     matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type    quaternion_type;
      typedef typename mbd_types::math_policy::vector_range       vector_range;
      typedef typename mbd_types::math_policy::idx_vector_range   idx_vector_range;
      typedef typename mbd_types::math_policy::matrix_range       matrix_range;
      typedef typename mbd_types::math_policy::vector_type        vector_type;
      typedef typename mbd_types::math_policy::value_traits       value_traits;

      typedef LinearJointMotor<mbd_types>     linear_motor_type;
      typedef LinearJointLimit<mbd_types>     linear_limit_type;


    protected:

      vector3_type    m_r_off_B;           ///< Initial (i.e. at calibration of joint) center difference of the two bodies (I.e. c = r_cm^B - r_cm^A) given in the body frame of body B.
      vector3_type    m_r_off_wcs;         ///< Initial (i.e. at calibration of joint) center difference of the two bodies (I.e. c = r_cm^B - r_cm^A) given in WCS.
      vector3_type    m_c;                 ///< Center difference of the two bodies (I.e. c = r_cm^B - r_cm^A).
      vector3_type    m_half_cXt1;         ///< Center difference of the two bodies (I.e. c = r_cm^B - r_cm^A) crossed with t1 and multiplied by 0.5.
      vector3_type    m_half_cXt2;         ///< Center difference of the two bodies (I.e. c = r_cm^B - r_cm^A) crossed with t2 and multiplied by 0.5.
      vector3_type    m_t1;                ///< A vector orthogonal to the joint axe (in WCS).
      vector3_type    m_t2;                ///< Another vector orthogonal to the joint axe and t1 (in WCS).
      vector3_type    m_u;                 ///< Error correction rotation axe. Ie. if joint is misaligned this vector is used to determine an rotation axe which can be used to align the joint along.
      real_type       m_theta_error;       ///< Misalignment error around the u-axe.
      quaternion_type m_Q_initial;         ///< The initial relative orientation between the two bodies (wrt. WCS) i.e. the rotation that aligns body A with body B.
      quaternion_type m_Q_initial_conj;    ///< The conjugate of Q_initial.
      quaternion_type m_Q_error;           ///< A quaterion used to compute the rotaional misalignment in the slider joint.

      linear_limit_type  *  m_limit;
      linear_motor_type  *  m_motor;

      size_type        m_rows;            ///< Number of jacobian rows.
      size_type        m_motor_offset;
      size_type        m_limit_offset;
      vector_type      m_gamma;           ///< Local vector of constraint force mixing terms.
      vector_type      m_solution;        ///< Local solution vector, ie. vector of lagrange multipliers.

    public:


      /**
      * Get Slider Offset.
      *
      * @return
      */
      real_type get_offset() const
      {
        assert(this->m_socketA || !"SliderJoint::get_offset(): socket A was null");
        assert(this->m_socketB || !"SliderJoint::get_offset(): socket B was null");

        //--- Get joint_type Axe in WCS
        vector3_type s_wcs = this->m_socketA->get_joint_axis_world();

        //--- Transform initial offset (between bod centers) into WCS
        quaternion_type Q;
        this->m_socketB->get_body()->get_orientation(Q);
        vector3_type r_off_wcs = Q.rotate(m_r_off_B);

        //--- Compute current offset between body centers in WCS
        vector3_type c,r_cm_A,r_cm_B;
        this->m_socketA->get_body()->get_position(r_cm_A);
        this->m_socketB->get_body()->get_position(r_cm_B);
        c = r_cm_B - r_cm_A;

        //--- The change in joint movement must be the difference
        //--- between intial and current offsets between the body
        //--- centers.
        vector3_type r_disp = c - r_off_wcs;

        //--- We want a signed measure of joint position, we get this
        //--- by taking the dot product with the joint axe.
        return s_wcs * r_disp;
      }

      /**
      * Get Offset Rate.
      * This method computes the (signed) speed, by which the joint
      * position moves along the joint axe.
      *
      * @return    The offset speed.
      */
      real_type get_offset_rate() const
      {
        assert(this->m_socketA || !"SliderJoint::get_offset_rate(): socket A was null");
        assert(this->m_socketB || !"SliderJoint::get_offset_rate(): socket B was null");

        //--- For a slider joint both incident bodies must
        //--- have the same angular velocity otherwise the
        //--- joint axe will break. The change in joint position
        //--- is therefore only dependent on the bodies relative
        //--- linear velocity.
        vector3_type v_cm_A,v_cm_B;
        this->m_socketA->get_body()->get_velocity(v_cm_A);
        this->m_socketB->get_body()->get_velocity(v_cm_B);
        return this->m_socketA->get_joint_axis_world()*(v_cm_A - v_cm_B);
      }

      vector3_type  get_slider_axis_world() const 
      { 
        return this->m_socketA->get_joint_axis_world(); 
      }

      void set_limit(linear_limit_type const & limit) 
      {  
        m_limit = const_cast<linear_limit_type*>(&limit); 
      }

      void set_motor(linear_motor_type const & motor) 
      {  
        m_motor = const_cast<linear_motor_type*>(&motor); 
      }

    public:

      SliderJoint()
        : m_limit(0)
        , m_motor(0)
        , m_rows(0)
      {}

      virtual ~SliderJoint() {}

    public:

      void evaluate()
      {
        using std::acos;

        m_rows = 0;

        if(!(this->m_socketA && this->m_socketB))//--- joint_type must be disconnected nothing to evaluate
          return;

        m_rows = 5;

        vector3_type s_A_wcs = this->m_socketA->get_joint_axis_world();
        vector3_type s_B_wcs = this->m_socketB->get_joint_axis_world();
        m_u = cross(s_A_wcs , s_B_wcs);
        m_t1 = this->m_socketA->get_axis1_world();
        m_t2 = this->m_socketA->get_axis2_world();


        //--- Compute center differences and cross products
        vector3_type r_cm_A;
        vector3_type r_cm_B;
        this->m_socketA->get_body()->get_position(r_cm_A);
        this->m_socketB->get_body()->get_position(r_cm_B);

        m_c = r_cm_B - r_cm_A;
        m_half_cXt1 = cross(m_c , m_t1)/value_traits::two();
        m_half_cXt2 = cross(m_c , m_t2)/value_traits::two();

        //--- Determine bend error of slider joint
        m_theta_error = acos(s_A_wcs * s_B_wcs);   //--- This is very sensitive to numerical errors, for
                                                   //--- instance if axe dot product is sligtly above value_traits::one() a
                                                   //--- NAN is returned

        //--- The ``bend'' error could also be computed by using quaterions.
        //--- The formula is:
        //---
        //---   Q_true = Q_initial    because relative orientation should be unchanged for a slider
        //---
        //---   Q_cur  = Q_error Q_true
        //---   Q_cur  conj(Q_true) = Q_error Q_true conj(Q_true)
        //---   Q_cur  conj(Q_true) = Q_error
        //---
        //--- From which it is clear that
        //---
        //---   Q_error = Q_cur conj(Q_initial)
        //---
        quaternion_type Q_cur;
        quaternion_type Q_A,Q_B;
        this->m_socketA->get_body()->get_orientation(Q_A);
        this->m_socketB->get_body()->get_orientation(Q_B);

        //---
        //---  Q_A :  BF_A -> WCS
        //---  Q_B :  BF_B -> WCS   => Q_B^* : WCS -> BF_B
        //---
        //---  Q_initial : BF_A -> BF_B  => Q_initial = Q_B^*  Q_A
        //---
        Q_cur = conj(Q_B) % Q_A;
        m_Q_error = Q_cur % m_Q_initial_conj;
        //--- Observe that Q_error measures how much BF_j should be
        //--- rotated in order to be perfectly aligned with BF_i, because
        //---
        //---  Q_initial^* : BF_j -> BF_i
        //---  Q_cur       : BF_i -> BF_j

        //--- Determine the value of the intial position in WCS.
        m_r_off_wcs = Q_B.rotate(m_r_off_B);


        m_limit_offset = 0;
        m_motor_offset = 0;
        if(m_limit)
        {
          m_limit->evaluate(
            get_offset()
            , get_slider_axis_world()
            , r_cm_A
            , r_cm_B
            );
          if(m_limit->is_active())
          {
            m_limit_offset = m_rows;
            ++m_rows;
          }
        }
        if(m_motor)
        {
          m_motor->evaluate(
            get_slider_axis_world()
            , r_cm_A
            , r_cm_B
            );
          if(m_motor->is_active())
          {
            m_motor_offset = m_rows;
            ++m_rows;
          }
        }
      }

      size_type get_number_of_jacobian_rows() const {return m_rows;}

      void get_linear_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"SliderJoint::get_linear_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"SliderJoint::get_linear_jacobian_A(): incorrect dimensions");
        J(0,0) = value_traits::zero();      J(0,1) = value_traits::zero();      J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero();      J(1,1) = value_traits::zero();      J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero();      J(2,1) = value_traits::zero();      J(2,2) = value_traits::zero();
        J(3,0) = m_t1(0); J(3,1) = m_t1(1); J(3,2) = m_t1(2);
        J(4,0) = m_t2(0); J(4,1) = m_t2(1); J(4,2) = m_t2(2);
        if(m_limit_offset)
          m_limit->get_linear_jacobian_A(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_linear_jacobian_A(J,m_motor_offset);
      }

      void get_linear_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"SliderJoint::get_linear_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"SliderJoint::get_linear_jacobian_B(): incorrect dimensions");
        J(0,0) = value_traits::zero();       J(0,1) = value_traits::zero();       J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero();       J(1,1) = value_traits::zero();       J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero();       J(2,1) = value_traits::zero();       J(2,2) = value_traits::zero();
        J(3,0) = -m_t1(0); J(3,1) = -m_t1(1); J(3,2) = -m_t1(2);
        J(4,0) = -m_t2(0); J(4,1) = -m_t2(1); J(4,2) = -m_t2(2);
        if(m_limit_offset)
          m_limit->get_linear_jacobian_B(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_linear_jacobian_B(J,m_motor_offset);
      }

      void get_angular_jacobian_A(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"SliderJoint::get_angular_jacobian_A(): incorrect dimensions");
        assert(J.size2()==3 || !"SliderJoint::get_angular_jacobian_A(): incorrect dimensions");
        J(0,0) = value_traits::one();             J(0,1) = value_traits::zero();             J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero();             J(1,1) = value_traits::one();             J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero();             J(2,1) = value_traits::zero();             J(2,2) = value_traits::one();
        J(3,0) = m_half_cXt1(0); J(3,1) = m_half_cXt1(1); J(3,2) = m_half_cXt1(2);
        J(4,0) = m_half_cXt2(0); J(4,1) = m_half_cXt2(1); J(4,2) = m_half_cXt2(2);
        if(m_limit_offset)
          m_limit->get_angular_jacobian_A(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_angular_jacobian_A(J,m_motor_offset);
      }

      void get_angular_jacobian_B(matrix_range & J)const
      {
        assert(J.size1()==m_rows || !"SliderJoint::get_angular_jacobian_B(): incorrect dimensions");
        assert(J.size2()==3 || !"SliderJoint::get_angular_jacobian_B(): incorrect dimensions");
        J(0,0) = -value_traits::one();            J(0,1) = value_traits::zero();            J(0,2) = value_traits::zero();
        J(1,0) = value_traits::zero();            J(1,1) = -value_traits::one();            J(1,2) = value_traits::zero();
        J(2,0) = value_traits::zero();            J(2,1) = value_traits::zero();            J(2,2) = -value_traits::one();
        J(3,0) = m_half_cXt1(0); J(3,1) = m_half_cXt1(1); J(3,2) = m_half_cXt1(2);
        J(4,0) = m_half_cXt2(0); J(4,1) = m_half_cXt2(1); J(4,2) = m_half_cXt2(2);
        if(m_limit_offset)
          m_limit->get_angular_jacobian_B(J,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_angular_jacobian_B(J,m_motor_offset);
      }

      void get_stabilization_term(vector_range & b_error)const
      {
        assert(b_error.size()==m_rows || !"SliderJoint::get_stabilization_term(): incorrect dimensions");

        real_type k_cor = this->get_frames_per_second()*this->get_error_reduction_parameter();

        //--- The first three elements will result in relative angular
        //--- velocity of the two bodies - this is set to bring them
        //--- back into alignment.
        //---
        //--- The correcting angular velocity is
        //---
        //---   |angular_velocity| = angle/time = erp*theta_error / stepsize
        //---                      = (erp*fps) * theta_error
        //---    angular_velocity  = |angular_velocity| * u
        //---                      = (erp*fps) * theta_error * u
        //---
        //--- Where rotation along unit length axis u by theta_error brings
        //--- body i's frame to Q_initial with respect to body j's frame.
        //---
        //--- Using a small angle approximation for sin():
        //---
        //---    angular_velocity  = (erp*fps) * 2 * v
        //---
        //--- Where the quaternion of the relative rotation between
        //--- the two bodies is
        //---
        //---    q_error = [cos(theta_error/2) sin(theta_error/2)*u] = [s v]
        //---
        //b_error(0) = k_cor * theta_error * u(0);
        //b_error(1) = k_cor * theta_error * u(1);
        //b_error(2) = k_cor * theta_error * u(2);

        b_error(0) = -2.*k_cor * m_Q_error.v()(0);
        b_error(1) = -2.*k_cor * m_Q_error.v()(1);
        b_error(2) = -2.*k_cor * m_Q_error.v()(2);
        vector3_type tmp =  m_c - m_r_off_wcs;
        b_error(3) = k_cor * m_t1*tmp;
        b_error(4) = k_cor * m_t2*tmp;
        if(m_limit_offset)
          m_limit->get_stabilization_term(b_error,m_limit_offset);
        if(m_motor_offset)
          m_motor->get_stabilization_term(b_error,m_motor_offset);
      }

      void get_low_limits(vector_range & lo)const
      {
        assert(lo.size()==m_rows || !"SliderJoint::get_low_limits(): incorrect dimensions");
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
        assert(hi.size()==m_rows || !"SliderJoint::get_high_limits(): incorrect dimensions");
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
        assert(dep.size()==m_rows || !"SliderJoint::get_dependency_indices(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          dep(i) = OpenTissue::math::detail::highest<size_type>();
      }

      void get_dependency_factors(vector_range & factors)const
      {
        assert(factors.size()==m_rows || !"SliderJoint::get_dependency_factors(): incorrect dimensions");
        for(size_type i=0;i<m_rows;++i)
          factors(i) = value_traits::zero();
      }

      void set_regularization(vector_range const & gamma)
      {
        assert(gamma.size()==m_rows || !"SliderJoint::set_regularization(): incorrect dimensions");
        math_policy::resize( m_gamma, 5);
        for(size_type i=0;i<5;++i)
        {
          assert(gamma(i)<=1 || !"SliderJoint::set_regularization(): gamma was greater than 1");
          assert(gamma(i)>=0 || !"SliderJoint::set_regularization(): gamma was less than 0");
          m_gamma(i) = gamma(i);
        }
        if(m_limit_offset)
          m_limit->set_regularization(gamma,m_limit_offset);
        if(m_motor_offset)
          m_motor->set_regularization(gamma,m_motor_offset);
      }

      void get_regularization(vector_range & gamma)const
      {
        assert(gamma.size()==m_rows || !"SliderJoint::get_regularization(): incorrect dimensions");
        if(m_gamma.size()==0)
        {
          for(size_type i=0;i<5;++i)
            gamma(i) = 0;
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
        assert(solution.size()==m_rows || !"SliderJoint::set_solution(): incorrect dimensions");

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
        assert(solution.size()==m_rows || !"SliderJoint::get_solution(): incorrect dimensions");
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
        assert(this->m_socketA || !"SliderJoint::calibration(): socket A was null");
        assert(this->m_socketB || !"SliderJoint::calibration(): socket B was null");

        //--- Compute initial relative orientation.
        quaternion_type Q_A,Q_B;
        this->m_socketA->get_body()->get_orientation(Q_A);
        this->m_socketB->get_body()->get_orientation(Q_B);

        //---
        //---  Q_A :  BF_A -> WCS
        //---  Q_B :  BF_B -> WCS   => Q_B^* : WCS -> BF_B
        //---
        //---  Q_initial : BF_A -> BF_B  => Q_initial = Q_B^*  Q_A
        //---
        m_Q_initial = conj(Q_B) % Q_A;
        m_Q_initial_conj = conj(m_Q_initial);

        //--- Extract center of mass of the bodies.
        vector3_type r_cm_A;
        vector3_type r_cm_B;
        this->m_socketA->get_body()->get_position(r_cm_A);
        this->m_socketB->get_body()->get_position(r_cm_B);

        //--- Compute center differences and cross products
        m_c = r_cm_B - r_cm_A;

        //--- Transform c into body j frame
        m_r_off_B = conj(Q_B).rotate(m_c);
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_JOINTS_MBD_SLIDER_JOINT_H
#endif 
