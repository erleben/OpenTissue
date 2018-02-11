//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
#include <OpenTissue/kinematics/inverse/inverse.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


typedef OpenTissue::math::default_math_types		  			                 math_types;
typedef math_types::vector3_type                                         vector3_type;
typedef math_types::value_traits                                         value_traits;
typedef math_types::real_type                                            real_type;
typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>              base_bone_traits;
typedef OpenTissue::kinematics::inverse::BoneTraits<base_bone_traits>    bone_traits;
typedef OpenTissue::skeleton::Types<math_types, bone_traits>             skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
typedef skeleton_types::bone_type                                        bone_type;

BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_set_joint_parameters);

BOOST_AUTO_TEST_CASE(test_cases)
{
  using std::fabs;

  real_type const tol      = boost::numeric_cast<real_type>(0.1);

  // Create some skeleton to test with
  skeleton_type skeleton;

  bone_type * b0 = skeleton.create_bone();
  bone_type * b1 = skeleton.create_bone(b0);
  bone_type * b2 = skeleton.create_bone(b1);

  // Skeleton hierarchy looks like this
  //
  //            |  hinge
  //            b0
  //    slider /  
  //          b1  
  //    ball  |
  //          b2
  //
  vector3_type const hinge_axis   = vector3_type(value_traits::zero(), value_traits::zero(), value_traits::one() );
  real_type    const hinge_angle  = value_traits::pi_half();
  vector3_type const slider_axis  = vector3_type(value_traits::one(), value_traits::zero(), value_traits::zero() );
  real_type    const slider_value = value_traits::four();

  real_type    const phi          = value_traits::pi_quarter();
  real_type    const psi          = value_traits::pi_half();
  real_type    const theta        = value_traits::pi_half();

  b0->type() = bone_traits::hinge_type;
  b0->bind_pose().T() = vector3_type(value_traits::zero(), value_traits::one(), value_traits::zero() );    
  b0->bind_pose().Q().Ru( hinge_angle, hinge_axis );

  b1->type() = bone_traits::slider_type; 
  b1->bind_pose().T() = slider_axis*slider_value;
  b1->bind_pose().Q().identity();

  b2->type() = bone_traits::ball_type;
  b2->bind_pose().T().clear();
  b2->bind_pose().Q() = OpenTissue::math::Rz(phi)*OpenTissue::math::Ry(psi)*OpenTissue::math::Rz(theta);

  OpenTissue::kinematics::inverse::set_joint_parameters( skeleton );

  BOOST_CHECK( b0->type() == bone_traits::hinge_type );
  BOOST_CHECK( b1->type() == bone_traits::slider_type );
  BOOST_CHECK( b2->type() == bone_traits::ball_type );
  BOOST_CHECK_CLOSE( b0->u()(0), hinge_axis(0), tol);
  BOOST_CHECK_CLOSE( b0->u()(1), hinge_axis(1), tol);
  BOOST_CHECK_CLOSE( b0->u()(2), hinge_axis(2), tol);

  BOOST_CHECK_CLOSE( OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta(*b0, 0), hinge_angle, tol);
  BOOST_CHECK_CLOSE( b1->u()(0), slider_axis(0), tol);
  BOOST_CHECK_CLOSE( b1->u()(1), slider_axis(1), tol);
  BOOST_CHECK_CLOSE( b1->u()(2), slider_axis(2), tol);
  BOOST_CHECK_CLOSE( OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta(*b1, 0), slider_value, tol);
  BOOST_CHECK_CLOSE( OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta(*b2, 0), phi, tol );
  BOOST_CHECK_CLOSE( OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta(*b2, 1), psi, tol );
  BOOST_CHECK_CLOSE( OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta(*b2, 2), theta, tol );
}

BOOST_AUTO_TEST_SUITE_END();
