//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
#include <OpenTissue/kinematics/inverse/inverse_compute_jacobian.h>
#include <OpenTissue/kinematics/inverse/inverse_chain.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


typedef float																             real_type;
typedef OpenTissue::math::default_math_types		  			 math_types;
typedef ublas::compressed_matrix<real_type>              matrix_type;

/**
* This is a dummy bone traits class implementation created just for
* the unit-test compile.
*
* The bone trait class was created to fill out the Jacobian
* matrix with the index of the bone. This way we can test
* which bones map into which sub-blocks in the resulting Jacobian matrix.
*
* param size and dofs were chosen to create Jacobian sub-blocks of
* size 1-by-1.
*/
class MyBoneTraits 
  : public OpenTissue::skeleton::DefaultBoneTraits<math_types>
{
public:

  size_t active_dofs() const { return 1; };

  template<typename bone_type, typename chain_type, typename matrix_range>
  static void compute_jacobian( bone_type & bone, chain_type & chain, matrix_range & J )
  {
    BOOST_CHECK( J.size1() == chain.get_goal_dimension());
    BOOST_CHECK( J.size2() == 1);

    for( size_t i=0;i<chain.get_goal_dimension();++i)
      J(i,0) = 1.0f*bone.get_number();
  }

};

typedef OpenTissue::skeleton::Types<math_types,MyBoneTraits>      skeleton_types;
typedef skeleton_types::skeleton_type                             skeleton_type;
typedef skeleton_types::bone_type                                 bone_type;
typedef skeleton_types::bone_traits                               bone_traits;
typedef OpenTissue::kinematics::inverse::Chain< skeleton_type >   chain_type;


BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_compute_jacobian);

BOOST_AUTO_TEST_CASE(test_cases)
{
  skeleton_type skeleton;

  bone_type * b0 = skeleton.create_bone();
  bone_type * b1 = skeleton.create_bone(b0);
  bone_type * b2 = skeleton.create_bone(b1);
  bone_type * b3 = skeleton.create_bone(b0);

  // Create a few different kinematic chains
  std::vector<chain_type> chains;
  chains.resize( 3 );
  chains[0].init( b0, b2 );
  chains[1].init( b0, b1 );
  chains[2].init( b0, b3 );

  // Assemble Jacobian matrix of the kinematic chains.
  matrix_type J;
  OpenTissue::kinematics::inverse::compute_jacobian( chains.begin(), chains.end(), skeleton.begin(), skeleton.end(), J);

  // Verify that the assembled Jacobian has the expeced pattern and data.
  BOOST_CHECK( J.size1() == 9);
  BOOST_CHECK( J.size2() == 4);


  for( size_t i=0;i<chains[0].get_goal_dimension();++i)
  {
    BOOST_CHECK( J(i,0) == 1.0f*b0->get_number() );
    BOOST_CHECK( J(i,1) == 1.0f*b1->get_number() );
    BOOST_CHECK( J(i,2) == 1.0f*b2->get_number() );
    BOOST_CHECK( J(i,3) == 0.0f                  );
  }

  size_t offset = chains[0].get_goal_dimension();
  for( size_t i=0;i<chains[1].get_goal_dimension();++i)
  {

    BOOST_CHECK( J(i+offset,0) == 1.0f*b0->get_number() );
    BOOST_CHECK( J(i+offset,1) == 1.0f*b1->get_number() );
    BOOST_CHECK( J(i+offset,2) == 0.0f                  );
    BOOST_CHECK( J(i+offset,3) == 0.0f                  );
  }

  offset += chains[1].get_goal_dimension();
  for( size_t i=0;i<chains[2].get_goal_dimension();++i)
  {

    BOOST_CHECK( J(i+offset,0) == 1.0f*b0->get_number() );
    BOOST_CHECK( J(i+offset,1) == 0.0f                  );
    BOOST_CHECK( J(i+offset,2) == 0.0f                  );
    BOOST_CHECK( J(i+offset,3) == 1.0f*b3->get_number() );
  }
}

BOOST_AUTO_TEST_SUITE_END();
