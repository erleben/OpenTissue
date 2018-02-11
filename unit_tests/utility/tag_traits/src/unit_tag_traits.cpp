//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_tag_traits.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

using namespace OpenTissue;

struct bar {};

struct foo { typedef void has_tag; typedef int tag_type; int m_tag; foo() : m_tag(23) {} };

struct blah : public OpenTissue::utility::TagSupportedType<bool> { blah() { this->m_tag = false; }; };


BOOST_AUTO_TEST_SUITE(opentissue_utility_tag_traits);

BOOST_AUTO_TEST_CASE(test_bool)
{
  blah a;

  BOOST_CHECK( has_tag(a) );

  BOOST_CHECK( !tag_value(a) );

  set_tag(a, true);
  BOOST_CHECK( tag_value(a) );
}

BOOST_AUTO_TEST_CASE(test_set_tag)
{
  bar a;
  foo b;

  OpenTissue::utility::set_tag(a, 44);
  BOOST_CHECK( OpenTissue::utility::tag_value(a) == 0 );

  OpenTissue::utility::set_tag(b, 76);
  BOOST_CHECK( OpenTissue::utility::tag_value(b) == 76 );
}

BOOST_AUTO_TEST_CASE(test_tag)
{
  bar a;
  foo b;

  BOOST_CHECK( !OpenTissue::utility::has_tag(a) );
  BOOST_CHECK( OpenTissue::utility::has_tag(b) );
  
  BOOST_CHECK( OpenTissue::utility::tag_value(a) == 0 );
  BOOST_CHECK( OpenTissue::utility::tag_value(b) == 23 );
}

BOOST_AUTO_TEST_SUITE_END();
