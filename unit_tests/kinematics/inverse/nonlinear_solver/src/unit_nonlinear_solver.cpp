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

typedef float																                             real_type;
typedef OpenTissue::math::default_math_types		  			                 math_types;
typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>              bone_traits;
typedef OpenTissue::kinematics::inverse::BoneTraits< bone_traits >       ik_bone_traits;
typedef OpenTissue::skeleton::Types<math_types,ik_bone_traits>           skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
typedef OpenTissue::kinematics::inverse::NonlinearSolver<skeleton_type>  solver_type;


BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_nonlinear_solver);

BOOST_AUTO_TEST_CASE(test_cases)
{
  skeleton_type skeleton;
  skeleton_type::bone_type * b0 = skeleton.create_bone();
  skeleton_type::bone_type * b1 = skeleton.create_bone(b0);
  skeleton_type::bone_type * b2 = skeleton.create_bone(b0);
  skeleton_type::bone_type * b3 = skeleton.create_bone(b2);
  solver_type solver = OpenTissue::kinematics::inverse::make_solver( skeleton );
  BOOST_CHECK( solver.skeleton() == &skeleton );
  BOOST_CHECK( solver.size() == 2u );
  solver_type::chain_iterator chain = solver.chain_begin();
  solver_type::chain_iterator end = solver.chain_end();
  BOOST_CHECK( chain != end);
  solver_type::chain_iterator c0 = chain; ++chain;
  BOOST_CHECK( chain != end);
  solver_type::chain_iterator c1 = chain; ++chain;
  BOOST_CHECK( c0 != c1);
  BOOST_CHECK( chain == end);
  BOOST_CHECK(c0->get_end_effector() == b1);
  BOOST_CHECK(c1->get_end_effector() == b3);
  skeleton_type::bone_type * b4 = skeleton.create_bone(b0);
  solver_type::chain_iterator c2 =  OpenTissue::kinematics::inverse::add_chain(solver,*b0,*b4);
  BOOST_CHECK( solver.size() == 3u );
  BOOST_CHECK( c0 != c1);
  BOOST_CHECK( c0 != c2);
  BOOST_CHECK( c1 != c2);
  BOOST_CHECK(c0->get_end_effector() == b1);
  BOOST_CHECK(c1->get_end_effector() == b3);
  BOOST_CHECK(c2->get_end_effector() == b4);
  solver.remove_chain(c2);
  BOOST_CHECK( solver.size() == 2u );
  BOOST_CHECK( c0 != c1);
  BOOST_CHECK(c0->get_end_effector() == b1);
  BOOST_CHECK(c1->get_end_effector() == b3);
  BOOST_CHECK_NO_THROW( solver.solve() );

  BOOST_CHECK_NO_THROW( solver.solve(0,0) );
  BOOST_CHECK_NO_THROW( solver.solve( &solver_type::default_BFGS_settings() ) );
  BOOST_CHECK_NO_THROW( solver.solve( &solver_type::default_SD_settings() ) );
  BOOST_CHECK_NO_THROW( solver.solve( &solver_type::default_SDF_settings() ) );

  solver_type::Settings settings = solver_type::default_BFGS_settings();
  solver_type::Output output;
  BOOST_CHECK_NO_THROW( solver.solve( &settings, &output ) );
}

BOOST_AUTO_TEST_SUITE_END();
