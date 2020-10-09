//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/image/io/image_read.h>
#include <OpenTissue/gpu/image/io/image_write.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_io);

BOOST_AUTO_TEST_CASE(test_cases)
{
  {
    const std::string filename = "/home/rortiz/Projects/OpenTissue/build/unit_tests/io/pax.png";

    OpenTissue::image::Image<unsigned char> image;

    OpenTissue::image::read(filename, image);
    OpenTissue::image::write("/home/rortiz/Projects/OpenTissue/build/unit_tests/io/pax_out.png", image);
  }
}

BOOST_AUTO_TEST_SUITE_END();
