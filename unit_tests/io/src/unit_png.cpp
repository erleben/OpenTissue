//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/gpu/image/io/image_read.h>
#include <OpenTissue/gpu/image/io/image_write.h>

#define BOOST_AUTO_TEST_MAIN
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(opentissue_io);

BOOST_AUTO_TEST_CASE(test_cases)
{
  {
    // TODO: The read function does not extract optional chunks from the image, hence the result image lack those chunks.
    // In Either find an image that does not contain any of the chunks so that we can do fair comparision or create one and
    // store in the data directory. For now this tests that there is no mayor exceptions comming from libpng while reading
    // writting a png image.
    OpenTissue::image::Image<unsigned char> image;
    OpenTissue::image::read("pax.png", image);
    OpenTissue::image::write("pax_out.png", image);
  }
}

BOOST_AUTO_TEST_SUITE_END();
