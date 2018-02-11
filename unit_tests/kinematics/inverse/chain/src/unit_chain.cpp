//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
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

/**
* This is a dummy bone traits class implementation created just for the unit-test compile.
*/
class MyBoneTraits : public OpenTissue::skeleton::DefaultBoneTraits<math_types>
{
public:

};

typedef OpenTissue::skeleton::Types<math_types,MyBoneTraits>      skeleton_types;
typedef skeleton_types::skeleton_type                             skeleton_type;
typedef skeleton_types::bone_type                                 bone_type;
typedef skeleton_types::bone_traits                               bone_traits;
typedef OpenTissue::kinematics::inverse::Chain< skeleton_type >   chain_type;


BOOST_AUTO_TEST_SUITE(opentissue_kinematics_inverse_chain);

BOOST_AUTO_TEST_CASE(test_cases)
{
  skeleton_type m_skeleton;

  bone_type * b0 = m_skeleton.create_bone();
  bone_type * b1 = m_skeleton.create_bone(b0);
  bone_type * b2 = m_skeleton.create_bone(b1);
  bone_type * b4 = m_skeleton.create_bone(b0);



  // Test if chain methods return meaningfull data if invoked on an empty chain (ie. a non-initialzied chain)
  {
    chain_type chain;
    bone_type * root         = chain.get_root();
    bone_type * end_effector = chain.get_end_effector();
    BOOST_CHECK( root == 0 );
    BOOST_CHECK( end_effector == 0 );

    chain_type::bone_iterator         bone      = chain.bone_begin();
    chain_type::bone_iterator         bone_end  = chain.bone_end();
    BOOST_CHECK( bone==bone_end );
    chain_type::reverse_bone_iterator rbone     = chain.rbone_begin();
    chain_type::reverse_bone_iterator rbone_end = chain.rbone_end();
    BOOST_CHECK( rbone==rbone_end );
  }

  // Test chain initialization with incorrect input
  {
    chain_type chain;
    bone_type * null = 0;
    BOOST_CHECK_THROW( chain.init( b0, null ), std::invalid_argument );
  }
  {
    chain_type chain;
    bone_type * null = 0;
    BOOST_CHECK_THROW( chain.init( null, b2 ), std::invalid_argument );
  }
  {
    chain_type chain;
    bone_type * null = 0;
    BOOST_CHECK_THROW( chain.init( null, null ), std::invalid_argument );
  }
  {
    chain_type chain;
    BOOST_CHECK_THROW( chain.init( b4, b2 ), std::logic_error );
  }
  {
    chain_type chain;
    BOOST_CHECK_THROW( chain.init( b2, b4 ), std::logic_error );
  }
  {
    chain_type chain;
    BOOST_CHECK_THROW( chain.init( b2, b0 ), std::logic_error );
  }

  // Initialize chain with one bone
  {
    chain_type chain;
    BOOST_CHECK_NO_THROW( chain.init( b0, b0 ) );
    chain_type::bone_iterator         bone      = chain.bone_begin();
    chain_type::bone_iterator         bone_end  = chain.bone_end();
    BOOST_CHECK( &(*bone) == b0 );
    ++bone;
    BOOST_CHECK( bone == bone_end );
    chain_type::reverse_bone_iterator rbone     = chain.rbone_begin();
    chain_type::reverse_bone_iterator rbone_end = chain.rbone_end();
    BOOST_CHECK( &(*rbone) == b0 );
    ++rbone;
    BOOST_CHECK( rbone== rbone_end );

    size_t param_size = chain.get_goal_dimension();

    BOOST_CHECK( param_size == 3 );
  }

  //Initialize chain with two bones
  {
    chain_type chain;
    BOOST_CHECK_NO_THROW( chain.init( b0, b1 ) );

    chain_type::bone_iterator         bone      = chain.bone_begin();
    chain_type::bone_iterator         bone_end  = chain.bone_end();
    BOOST_CHECK( &(*bone) == b0 );
    ++bone;
    BOOST_CHECK( &(*bone) == b1 );
    ++bone;
    BOOST_CHECK( bone == bone_end );

    chain_type::reverse_bone_iterator rbone     = chain.rbone_begin();
    chain_type::reverse_bone_iterator rbone_end = chain.rbone_end();
    BOOST_CHECK( &(*rbone) == b1 );
    ++rbone;
    BOOST_CHECK( &(*rbone) == b0 );
    ++rbone;
    BOOST_CHECK( rbone == rbone_end  );
  }

  // Initialize chain with three bones
  {
    chain_type chain;
    BOOST_CHECK_NO_THROW( chain.init( b0, b2 ) );

    chain_type::bone_iterator         bone      = chain.bone_begin();
    chain_type::bone_iterator         bone_end  = chain.bone_end();
    BOOST_CHECK( &(*bone) == b0 );
    ++bone;
    BOOST_CHECK( &(*bone) == b1 );
    ++bone;
    BOOST_CHECK( &(*bone) == b2 );
    ++bone;
    BOOST_CHECK( bone == bone_end );
    
    chain_type::reverse_bone_iterator rbone     = chain.rbone_begin();
    chain_type::reverse_bone_iterator rbone_end = chain.rbone_end();
    BOOST_CHECK( &(*rbone) == b2 );
    ++rbone;
    BOOST_CHECK( &(*rbone) == b1 );
    ++rbone;
    BOOST_CHECK( &(*rbone) == b0 );
    ++rbone;
    BOOST_CHECK( rbone == rbone_end );

    bone_type * root         = chain.get_root();
    bone_type * end_effector = chain.get_end_effector();
    BOOST_CHECK( root == b0 );
    BOOST_CHECK( end_effector == b2 );

  }

  // Do outboard and inboard bone methods testing
  {
    chain_type chain;
    BOOST_CHECK_NO_THROW( chain.init( b0, b2 ) );

    chain_type::bone_iterator         bone0   = chain.bone_begin();
    chain_type::bone_iterator         bone1 = bone0; ++bone1;   
    chain_type::bone_iterator         bone2 = bone1; ++bone2;   
    
    chain_type::reverse_bone_iterator         rbone0  = chain.rbone_begin();
    chain_type::reverse_bone_iterator         rbone1  = rbone0; ++rbone1;
    chain_type::reverse_bone_iterator         rbone2  = rbone1; ++rbone2;

  }
}

BOOST_AUTO_TEST_SUITE_END();
