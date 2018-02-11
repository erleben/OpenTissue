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
typedef math_types::vector3_type                                         vector3_type;
typedef math_types::value_traits                                         value_traits;

typedef OpenTissue::skeleton::DefaultBoneTraits<math_types>                 default_bone_traits;
typedef OpenTissue::kinematics::inverse::BoneTraits< default_bone_traits >  bone_traits;

typedef OpenTissue::skeleton::Types<math_types, bone_traits>             skeleton_types;
typedef skeleton_types::skeleton_type                                    skeleton_type;
typedef OpenTissue::kinematics::inverse::NonlinearSolver<skeleton_type>  solver_type;

template< typename bone_type>
void test_bones(bone_type const * A, bone_type const * B)
{
  using std::fabs;

  BOOST_CHECK(A->get_name() == B->get_name() );

  real_type const tol = 10e-7;

  BOOST_CHECK( fabs( A->bind_pose().T()(0) - B->bind_pose().T()(0) ) < tol );
  BOOST_CHECK( fabs( A->bind_pose().T()(1) - B->bind_pose().T()(1) ) < tol );
  BOOST_CHECK( fabs( A->bind_pose().T()(2) - B->bind_pose().T()(2) ) < tol );

  BOOST_CHECK( fabs( A->bone_space().T()(0) - B->bone_space().T()(0) ) < tol );
  BOOST_CHECK( fabs( A->bone_space().T()(1) - B->bone_space().T()(1) ) < tol );
  BOOST_CHECK( fabs( A->bone_space().T()(2) - B->bone_space().T()(2) ) < tol );

  BOOST_CHECK( fabs( A->bone_space().Q().s() - B->bone_space().Q().s() ) < tol );
  BOOST_CHECK( fabs( A->bone_space().Q().v()(0) - B->bone_space().Q().v()(0) ) < tol );
  BOOST_CHECK( fabs( A->bone_space().Q().v()(1) - B->bone_space().Q().v()(1) ) < tol );
  BOOST_CHECK( fabs( A->bone_space().Q().v()(2) - B->bone_space().Q().v()(2) ) < tol );

  BOOST_CHECK( fabs( A->bind_pose().Q().s() - B->bind_pose().Q().s() ) < tol );
  BOOST_CHECK( fabs( A->bind_pose().Q().v()(0) - B->bind_pose().Q().v()(0) ) < tol );
  BOOST_CHECK( fabs( A->bind_pose().Q().v()(1) - B->bind_pose().Q().v()(1) ) < tol );
  BOOST_CHECK( fabs( A->bind_pose().Q().v()(2) - B->bind_pose().Q().v()(2) ) < tol );

  BOOST_CHECK( A->type() == B-> type() );
  if(A->active_dofs() == 1u)
  {
    BOOST_CHECK( fabs( A->box_limits().min_limit(0) - B->box_limits().min_limit(0) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().max_limit(0) - B->box_limits().max_limit(0) ) < tol );
    BOOST_CHECK( fabs( A->u()(0) - B->u()(0)  ) < tol );
    BOOST_CHECK( fabs( A->u()(1) - B->u()(1)  ) < tol );
    BOOST_CHECK( fabs( A->u()(2) - B->u()(2)  ) < tol );
  }
  else
  {
    BOOST_CHECK( fabs( A->box_limits().min_limit(0) - B->box_limits().min_limit(0) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().min_limit(1) - B->box_limits().min_limit(1) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().min_limit(2) - B->box_limits().min_limit(2) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().max_limit(0) - B->box_limits().max_limit(0) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().max_limit(1) - B->box_limits().max_limit(1) ) < tol );
    BOOST_CHECK( fabs( A->box_limits().max_limit(2) - B->box_limits().max_limit(2) ) < tol );
  }
  if(A->parent())
  {
    BOOST_CHECK(B->parent());
    BOOST_CHECK(A->parent()->get_name() == B->parent()->get_name() );
  }
}

template<typename chain_type>
void test_chains( chain_type const * A, chain_type const * B)
{
  BOOST_CHECK( A->get_root()->get_name() == B->get_root()->get_name() );
  BOOST_CHECK( A->get_end_effector()->get_name() == B->get_end_effector()->get_name() );
  BOOST_CHECK( A->weight_p() == B->weight_p() );
  BOOST_CHECK( A->weight_x() == B->weight_x() );
  BOOST_CHECK( A->weight_y() == B->weight_y() );
  BOOST_CHECK( A->p_global() == B->p_global() );
  BOOST_CHECK( A->x_global() == B->x_global() );
  BOOST_CHECK( A->y_global() == B->y_global() );
  BOOST_CHECK( A->p_local() == B->p_local() );
  BOOST_CHECK( A->x_local() == B->x_local() );
  BOOST_CHECK( A->y_local() == B->y_local() );
}

template<typename solver_type>
void test_solvers( solver_type const & A, solver_type const & B)
{
  BOOST_CHECK( A.size() == B.size() );
  typename solver_type::skeleton_type::const_bone_iterator bone1 =  A.skeleton()->begin();
  typename solver_type::skeleton_type::const_bone_iterator bone2 =  B.skeleton()->begin();
  typename solver_type::skeleton_type::const_bone_iterator bend1 =  A.skeleton()->end();
  typename solver_type::skeleton_type::const_bone_iterator bend2 =  B.skeleton()->end();
  for(;bone1!=bend1;++bone1, ++bone2)
    test_bones( &(*bone1), &(*bone2) );
  typename solver_type::const_chain_iterator chain1 =  A.chain_begin();
  typename solver_type::const_chain_iterator chain2 =  B.chain_begin();
  typename solver_type::const_chain_iterator cend1  =  A.chain_end();
  typename solver_type::const_chain_iterator cend2  =  B.chain_end();
  for(;chain1!=cend1;++chain1, ++chain2)
    test_chains( &(*chain1), &(*chain2) );
}


BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_ik_io);

BOOST_AUTO_TEST_CASE(test_cases)
{
  // Test that we get an exception if skeleton is missing!
  {
    solver_type solver;
    BOOST_CHECK_THROW( OpenTissue::kinematics::inverse::xml_write( "test.xml", solver ), std::invalid_argument );
    BOOST_CHECK_THROW( OpenTissue::kinematics::inverse::xml_read( "test.xml", solver ), std::invalid_argument );
  }

  // Test cases where we have a simple skeleton
  {
    // First we create a skeleton and fill it with some dummy data
    skeleton_type skeleton1;
    skeleton_type::bone_type * b0 = skeleton1.create_bone();
    skeleton_type::bone_type * b1 = skeleton1.create_bone(b0);
    skeleton_type::bone_type * b2 = skeleton1.create_bone(b0);
    skeleton_type::bone_type * b3 = skeleton1.create_bone(b2);

    b0->relative().T() = vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() );
    b1->relative().T() = vector3_type( value_traits::one(), value_traits::one(),  value_traits::zero() );
    b2->relative().T() = vector3_type( value_traits::one(), value_traits::zero(), value_traits::one() );
    b3->relative().T() = vector3_type( value_traits::one(), value_traits::two(),  value_traits::zero() );
    b0->relative().Q().Rx(value_traits::pi_quarter() );
    b1->relative().Q().Ry(value_traits::pi_half() );
    b2->relative().Q().Rz(value_traits::pi() );
    b3->relative().Q().Rx(value_traits::pi_half() );

    b0->bind_pose().T() = vector3_type( value_traits::two(), value_traits::zero(), value_traits::zero() );
    b1->bind_pose().T() = vector3_type( value_traits::two(), value_traits::two(),  value_traits::zero() );
    b2->bind_pose().T() = vector3_type( value_traits::two(), value_traits::zero(), value_traits::two() );
    b3->bind_pose().T() = vector3_type( value_traits::two(), value_traits::one(),  value_traits::zero() );
    b0->bind_pose().Q().Rz(value_traits::pi_quarter() );
    b1->bind_pose().Q().Ry(value_traits::pi_half() );
    b2->bind_pose().Q().Rx(value_traits::pi() );
    b3->bind_pose().Q().Rz(value_traits::pi_half() );

    b0->bone_space().T() = vector3_type( value_traits::four(), value_traits::zero(), value_traits::zero() );
    b1->bone_space().T() = vector3_type( value_traits::four(), value_traits::four(),  value_traits::zero() );
    b2->bone_space().T() = vector3_type( value_traits::four(), value_traits::zero(), value_traits::four() );
    b3->bone_space().T() = vector3_type( value_traits::four(), value_traits::two(),  value_traits::zero() );
    b0->bone_space().Q().Ry(value_traits::pi_quarter() );
    b1->bone_space().Q().Rx(value_traits::pi_half() );
    b2->bone_space().Q().Rz(value_traits::pi() );
    b3->bone_space().Q().Ry(value_traits::pi_half() );

    BOOST_CHECK( skeleton1.rename_bone( b0, "my_first_bone" )  );
    BOOST_CHECK( skeleton1.rename_bone( b1, "my_second_bone" ) );
    BOOST_CHECK( skeleton1.rename_bone( b2, "my_third_bone" )  );
    BOOST_CHECK( skeleton1.rename_bone( b3, "my_fourth_bone" ) );

    b0->type() = bone_traits::hinge_type;
    b1->type() = bone_traits::slider_type;
    b2->type() = bone_traits::ball_type;
    b3->type() = bone_traits::slider_type;

    b0->u() = vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() );
    b1->u() = vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero() );
    b2->u() = vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one() );
    b3->u() = vector3_type( value_traits::two(), value_traits::zero(), value_traits::zero() );

    b0->box_limits().min_limit( 0 ) = value_traits::one();
    b0->box_limits().max_limit( 0 ) = value_traits::two();
    b1->box_limits().min_limit( 0 ) = value_traits::one();
    b1->box_limits().max_limit( 0 ) = value_traits::two();
    b2->box_limits().min_limit( 0 ) = -value_traits::pi_quarter();
    b2->box_limits().min_limit( 1 ) = -value_traits::pi_half();
    b2->box_limits().min_limit( 2 ) = -value_traits::pi();
    b2->box_limits().max_limit( 0 ) =  value_traits::pi_quarter();
    b2->box_limits().max_limit( 1 ) =  value_traits::pi_half();
    b2->box_limits().max_limit( 2 ) =  value_traits::pi();
    b3->box_limits().min_limit( 0 ) = value_traits::four();
    b3->box_limits().max_limit( 0 ) = value_traits::four();

    // Now we create a solver based on that skeleton.
    solver_type solver1 = OpenTissue::kinematics::inverse::make_solver( skeleton1 );

    // See if we can write the solver and skeleton data to an xml file
    BOOST_CHECK_NO_THROW( OpenTissue::kinematics::inverse::xml_write( "test.xml", solver1, true ) );

    // Now we will try to create a copy of the first solver by reading from a test file
    skeleton_type skeleton2;
    solver_type solver2;
    solver2.init( skeleton2 );
    BOOST_CHECK_NO_THROW( OpenTissue::kinematics::inverse::xml_read( "test.xml", solver2, true ) );

    // Next we compare the original solver with the one that we read back from the xml file.
    test_solvers( solver1, solver2);

    // Next we will re-create solver 1, ( the idea is to make sure no cached data survives a read operation)
    BOOST_CHECK_NO_THROW( OpenTissue::kinematics::inverse::xml_read( "test.xml", solver1, true ) );

    // now solver 1 should stil match solver 2
    test_solvers( solver2, solver1);

    // Next we will re-create solver 2, but without reading skeleton data (the idea is to make sure that existing skeleton data should be un-affected)
    BOOST_CHECK_NO_THROW( OpenTissue::kinematics::inverse::xml_read( "test.xml", solver2, false ) );

    // The two solvers should stil match
    test_solvers( solver1, solver2);

  }

}

BOOST_AUTO_TEST_SUITE_END();
