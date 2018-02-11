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



BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_joint_parameters);

BOOST_AUTO_TEST_CASE(test_cases)
{
  real_type const tol = boost::numeric_cast<real_type>(0.01);

  skeleton_type skeleton;
  bone_type * b[4];

  b[0] = skeleton.create_bone();
  b[1] = skeleton.create_bone(b[0]);
  b[2] = skeleton.create_bone(b[1]);
  b[3] = skeleton.create_bone(b[0]);

  // Test if default joint parameters are set correctly
  for(size_t i=0u;i<3u;++i)
  {
    BOOST_CHECK(b[i]->type() == bone_traits::ball_type );
    BOOST_CHECK(b[i]->active_dofs() == 3u );


    BOOST_CHECK_CLOSE(b[i]->u()(0), value_traits::zero(), tol );
    BOOST_CHECK_CLOSE(b[i]->u()(1), value_traits::zero(), tol );
    BOOST_CHECK_CLOSE(b[i]->u()(2), value_traits::one(), tol );

    BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[i]), 0), value_traits::zero(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().min_limit(0), -value_traits::pi(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().max_limit(0),  value_traits::pi(), tol );

    BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[i]), 1), value_traits::zero(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().min_limit(1), -value_traits::pi(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().max_limit(1),  value_traits::pi(), tol );

    BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[i]), 2), value_traits::zero(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().min_limit(2), -value_traits::pi(), tol );
    BOOST_CHECK_CLOSE(b[i]->box_limits().max_limit(2),  value_traits::pi() , tol );
  }

  // Now try to set up some testing data
  b[0]->type() = bone_traits::hinge_type;
  BOOST_CHECK(b[0]->type() == bone_traits::hinge_type );

  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[0]), 0, value_traits::two());
  b[0]->box_limits().min_limit(0) = -value_traits::one();
  b[0]->box_limits().max_limit(0) =  value_traits::one();

  b[1]->type() = bone_traits::slider_type;
  BOOST_CHECK(b[1]->type() == bone_traits::slider_type );

  OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[1]), 0, -value_traits::four());
  b[1]->box_limits().min_limit(0) = -value_traits::two();
  b[1]->box_limits().max_limit(0) =  value_traits::two();

  b[2]->type() = bone_traits::ball_type;
  BOOST_CHECK(b[2]->type() == bone_traits::ball_type );

  for(size_t j=0u;j<3u;++j)
  {
    OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[2]), j, value_traits::pi());  
    b[2]->box_limits().min_limit(j) = -value_traits::pi_quarter();
    b[2]->box_limits().max_limit(j) =  value_traits::pi_quarter();
  }

  b[3]->type() = bone_traits::ball_type;
  BOOST_CHECK(b[3]->type() == bone_traits::ball_type );
  for(size_t j=0u;j<3u;++j)
  {
    OpenTissue::kinematics::inverse::ACCESSOR::unsynch_set_theta( *(b[3]), j, -value_traits::pi());    
    b[3]->box_limits().min_limit(j) = -value_traits::pi_half();
    b[3]->box_limits().max_limit(j) =  value_traits::pi_half();
  }

  vector_type theta;
  OpenTissue::kinematics::inverse::get_joint_parameters( skeleton, theta );


  BOOST_CHECK( theta.size() == 8u );
  BOOST_CHECK_CLOSE( theta(0),  value_traits::two()  ,tol);
  BOOST_CHECK_CLOSE( theta(1), -value_traits::four() ,tol);
  BOOST_CHECK_CLOSE( theta(2),  value_traits::pi()   ,tol);
  BOOST_CHECK_CLOSE( theta(3),  value_traits::pi()   ,tol);
  BOOST_CHECK_CLOSE( theta(4),  value_traits::pi()   ,tol);
  BOOST_CHECK_CLOSE( theta(5), -value_traits::pi()   ,tol);
  BOOST_CHECK_CLOSE( theta(6), -value_traits::pi()   ,tol);
  BOOST_CHECK_CLOSE( theta(7), -value_traits::pi()   ,tol);

  vector_type min_theta;
  vector_type max_theta;
  OpenTissue::kinematics::inverse::box_limits::get_joint_limits( skeleton, min_theta, max_theta );

  BOOST_CHECK( min_theta.size() == 8u );
  BOOST_CHECK_CLOSE( min_theta(0), -value_traits::one()  ,tol);
  BOOST_CHECK_CLOSE( min_theta(1), -value_traits::two() ,tol);
  BOOST_CHECK_CLOSE( min_theta(2), -value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( min_theta(3), -value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( min_theta(4), -value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( min_theta(5), -value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( min_theta(6), -value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( min_theta(7), -value_traits::pi_half()   ,tol);

  BOOST_CHECK( max_theta.size() == 8u );
  BOOST_CHECK_CLOSE( max_theta(0), value_traits::one()  ,tol);
  BOOST_CHECK_CLOSE( max_theta(1), value_traits::two() ,tol);
  BOOST_CHECK_CLOSE( max_theta(2), value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( max_theta(3), value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( max_theta(4), value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( max_theta(5), value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( max_theta(6), value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( max_theta(7), value_traits::pi_half()   ,tol);

  OpenTissue::kinematics::inverse::compute_joint_limits_projection( skeleton, theta );

  BOOST_CHECK( theta.size() == 8u );
  BOOST_CHECK_CLOSE( theta(0),  value_traits::one()  ,tol);
  BOOST_CHECK_CLOSE( theta(1), -value_traits::two() ,tol);
  BOOST_CHECK_CLOSE( theta(2),  value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( theta(3),  value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( theta(4),  value_traits::pi_quarter()   ,tol);
  BOOST_CHECK_CLOSE( theta(5), -value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( theta(6), -value_traits::pi_half()   ,tol);
  BOOST_CHECK_CLOSE( theta(7), -value_traits::pi_half()   ,tol);

  OpenTissue::kinematics::inverse::set_joint_parameters( skeleton, theta );

      
  
  
  BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[0]), 0), value_traits::one(), tol );
  BOOST_CHECK_CLOSE(b[0]->box_limits().min_limit(0), -value_traits::one(), tol );
  BOOST_CHECK_CLOSE(b[0]->box_limits().max_limit(0), value_traits::one(), tol );

  BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[1]), 0), -value_traits::two(), tol );
  BOOST_CHECK_CLOSE(b[1]->box_limits().min_limit(0), -value_traits::two(), tol );
  BOOST_CHECK_CLOSE(b[1]->box_limits().max_limit(0), value_traits::two(), tol );

  for(size_t j=0;j<3u;++j)
  {
    BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[2]), j), value_traits::pi_quarter(), tol );
    BOOST_CHECK_CLOSE(b[2]->box_limits().min_limit(j), -value_traits::pi_quarter(), tol );
    BOOST_CHECK_CLOSE(b[2]->box_limits().max_limit(j), value_traits::pi_quarter(), tol );

    BOOST_CHECK_CLOSE(OpenTissue::kinematics::inverse::ACCESSOR::unsynch_get_theta( *(b[3]), j), -value_traits::pi_half(), tol );
    BOOST_CHECK_CLOSE(b[3]->box_limits().min_limit(j), -value_traits::pi_half(), tol );
    BOOST_CHECK_CLOSE(b[3]->box_limits().max_limit(j), value_traits::pi_half(), tol );
  }
}

BOOST_AUTO_TEST_SUITE_END();
