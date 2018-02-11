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
typedef math_types::value_traits                                         value_traits;
typedef math_types::real_type                                            real_type;
typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>              base_bone_traits;
typedef OpenTissue::kinematics::inverse::BoneTraits<base_bone_traits>    bone_traits;
typedef OpenTissue::skeleton::Types<math_types, bone_traits>             skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
typedef skeleton_types::bone_type                                        bone_type;
typedef ublas::vector<real_type>                                         vector_type;
typedef OpenTissue::kinematics::inverse::Chain< skeleton_type >          chain_type;



BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_compute_weighted_diff);

BOOST_AUTO_TEST_CASE(test_cases)
{
  using std::fabs;

  real_type const too_tiny = boost::numeric_cast<real_type>(10e-15);

  // Create some skeleton to test with
  skeleton_type skeleton;
  bone_type * b[4];
  b[0] = skeleton.create_bone();
  b[1] = skeleton.create_bone(b[0]);
  b[2] = skeleton.create_bone(b[1]);
  b[3] = skeleton.create_bone(b[0]);

  // Now we try to set up some testing data
  //
  // Skeleton hierarchy looks like this
  //
  //            |  hinge
  //            b0
  //    slider /  \   ball
  //          b1   b3
  //    ball  |
  //          b2
  //
  b[0]->type() = bone_traits::hinge_type;
  b[1]->type() = bone_traits::slider_type;
  b[2]->type() = bone_traits::ball_type;
  b[3]->type() = bone_traits::ball_type;

  b[0]->u() = math_types::vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one() );
  b[0]->relative().T().clear();
  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[0]),  0u, value_traits::pi() );
  b[0]->box_limits().min_limit(0) = -value_traits::pi();
  b[0]->box_limits().max_limit(0) =  value_traits::pi();

  b[1]->u() = math_types::vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() );
  b[1]->relative().Q().identity();
  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[1]),  0u, value_traits::one() );
  b[1]->box_limits().min_limit(0) = -value_traits::two();
  b[1]->box_limits().max_limit(0) =  value_traits::two();

  b[2]->relative().T().clear();
  for(size_t j=0u;j<3u;++j)
  {
    OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[2]),  j, value_traits::pi_half() );    
    b[2]->box_limits().min_limit(j) = -value_traits::pi_quarter();
    b[2]->box_limits().max_limit(j) =  value_traits::pi_quarter();
  }
  b[2]->relative().T().clear();
  for(size_t j=0u;j<3u;++j)
  {
    OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[3]),  j, value_traits::zero() ); 
    b[3]->box_limits().min_limit(j) = -value_traits::pi_half();
    b[3]->box_limits().max_limit(j) =  value_traits::pi_half();
  }

  // we assume that get/set joint parameters work and use these for initializing data.
  vector_type theta;
  OpenTissue::kinematics::inverse::get_joint_parameters( skeleton, theta );
  OpenTissue::kinematics::inverse::set_joint_parameters( skeleton, theta );

  // Create some chains 
  std::vector<chain_type> chains;
  chains.resize( 2 );
  chains[0].init( b[0], b[2] );
  chains[1].init( b[0], b[3] );

  // Set some positional goal placements
  chains[0].p_global() =  math_types::vector3_type( - value_traits::two(), value_traits::zero(), value_traits::zero() );
  chains[1].p_global() =  math_types::vector3_type(   value_traits::one(), value_traits::zero(), value_traits::zero() );
  chains[0].p_local().clear();
  chains[1].p_local().clear();
  chains[0].only_position() = true;
  chains[1].only_position() = true;

  // Compute the difference vector
  vector_type delta;
  OpenTissue::kinematics::inverse::compute_weighted_difference( chains.begin(), chains.end(), delta );

  // Test if we received the expected values
  BOOST_CHECK( delta.size() == 6u );
  BOOST_CHECK( fabs(-value_traits::one() - delta(0)) < too_tiny );
  BOOST_CHECK( fabs(delta(1)) < too_tiny );
  BOOST_CHECK( fabs(delta(2)) < too_tiny );
  BOOST_CHECK( fabs(value_traits::one()-delta(3)) < too_tiny );
  BOOST_CHECK( fabs(delta(4)) < too_tiny );
  BOOST_CHECK( fabs(delta(5)) < too_tiny );

  //--- Next we will test a rotation part of goal placement ------
  chains[1].only_position() = false;
  chains[1].x_global() =  math_types::vector3_type(   value_traits::zero(), value_traits::zero(), value_traits::one() );
  chains[1].y_global() =  math_types::vector3_type(   value_traits::zero(), value_traits::one(), value_traits::zero() );
  chains[1].x_local()  =  math_types::vector3_type(   value_traits::one(),  value_traits::zero(), value_traits::zero() );
  chains[1].y_local()  =  math_types::vector3_type(   value_traits::zero(), value_traits::one(), value_traits::zero()  );

  // Set up some controlled relative transformation
  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[3]),  0u, value_traits::zero() );    
  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[3]),  1u, value_traits::pi_half() );    
  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[3]),  2u, value_traits::pi() );    
  
  // We use get/set- joint paramters to make sure everything is initialized
  OpenTissue::kinematics::inverse::get_joint_parameters( skeleton, theta );
  OpenTissue::kinematics::inverse::set_joint_parameters( skeleton, theta );

  // Compute difference vector
  OpenTissue::kinematics::inverse::compute_weighted_difference( chains.begin(), chains.end(), delta );

  // Test if we have the expected values
  BOOST_CHECK( delta.size() == 12u );
  BOOST_CHECK( fabs(-value_traits::one() - delta(0)) < too_tiny );
  BOOST_CHECK( fabs(delta(1)) < too_tiny );
  BOOST_CHECK( fabs(delta(2)) < too_tiny );
  BOOST_CHECK( fabs(value_traits::one()-delta(3)) < too_tiny );
  BOOST_CHECK( fabs(delta(4)) < too_tiny );
  BOOST_CHECK( fabs(delta(5)) < too_tiny );
  BOOST_CHECK( fabs(delta(6)) < too_tiny );
  BOOST_CHECK( fabs(delta(7)) < too_tiny );
  BOOST_CHECK( fabs(delta(8)) < too_tiny );
  BOOST_CHECK( fabs(delta(9)) < too_tiny );
  BOOST_CHECK( fabs(delta(10)) < too_tiny );
  BOOST_CHECK( fabs(delta(11)) < too_tiny );

}

BOOST_AUTO_TEST_SUITE_END();
