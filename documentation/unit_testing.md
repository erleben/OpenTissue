# How to organize and create unit tests in OpenTissue

Thanks to Joergen Havsberg Seland for the inspiration to setup unit testing in OpenTissue. Some of the bits and pieces in below are more or less copied directly from Joergen's description on the 3Dot wiki (anno May 2007). With a few added OpenTissue tweaks made by myself (kenny).

<center><b>WARNING:</b> Only compatible with Boost 1_34_0</center>

Unit tests should be placed into the OpenTissue unit test folder, which is located at

<pre>
$(OPENTISSUE)/units
</pre>

The structure inside the unit-test folder corresponds to the structure in the OpenTissue library folder with the exception that each folder is pre-fixed with the string unit_. Each unit-test is placed into a sub-folder. The folder name of the folder containing the test should be pre-fixed with unit_ and the remaining part of the name should reflect what the unit is actually testing. For instance say one wants to create unit tests for the stuff in the basic_math_types.h header file in OpenTissue. Then one would create a unit test folder like

<pre>
$(OPENTISSUE)/units/unit_math/unit_basic_math_types/
</pre>

Now one can simply create cpp-files for doing the actual unit-testing inside a src-folder also CMakelists.txt files should be added. For the above example this could look like

<pre>
$(OPENTISSUE)/units/unit_math/unit_basic_math_types/src/unit_basic_math_types.cpp
$(OPENTISSUE)/units/unit_math/unit_basic_math_types/CMakelists.txt
</pre>

Remember to add your new unit-test to the CMakelist.txt file in the folder $(OPENTISSUE)/units/unit_math.

All unit tests should be put in a top level test suite named with the name space and name of the class or file being tested, separated by underscore (depending on whether the test subject is a class or just a group of functions). An example cpp-file is shown below.

    //
    // OpenTissue, A toolbox for physical based simulation and animation.
    // Copyright (C) 2007 Department of Computer Science, University of Copenhagen
    //
    #include <OpenTissue/configuration.h>

    // Add Include headers from the code being tested
    #include <OpenTissue/math/basic_math_types.h>

    #define BOOST_AUTO_TEST_MAIN
    #include <boost/test/auto_unit_test.hpp>

    // Boost Test declaration and Checking macros
    #include <boost/test/unit_test_suite.hpp>
    #include <boost/test/test_tools.hpp>

    BOOST_AUTO_TEST_SUITE(opentisue_math_basic_math_types);

    BOOST_AUTO_TEST_SUITE();

    BOOST_AUTO_TEST_CASE(my_first_test_case)
    {
      BOOST_CHECK_EQUAL( ...., true);
      BOOST_CHECK_EQUAL( ...., true);
    }

    BOOST_AUTO_TEST_CASE(my_second_test_case)
    {
      BOOST_CHECK_EQUAL( ...., true);
      BOOST_CHECK_EQUAL( ...., true);
    }

    BOOST_AUTO_TEST_CASE(my_third_test_case)
    {
      BOOST_CHECK_EQUAL( ...., true);
      BOOST_CHECK_EQUAL( ...., true);
    }

    BOOST_AUTO_TEST_SUITE_END();

    BOOST_AUTO_TEST_CASE(my_always_fail_test_case)
    {
      BOOST_CHECK(false);
    }

    BOOST_AUTO_TEST_SUITE_END();

Important points about this code, are:

<ol>
<li>Because we are using the boost::unit_test automatic test registration facility, we do not need a header file for the tests, and we do not need to write any extra code for registering them. We just make sure the cpp file containing the tests is included in the project, and the tests will be available.</li>
<li>You may freely nest test suites within each other.</li>
</ol>

See the [boost test library](http://www.boost.org/libs/test/doc/components/utf/index.html) for more information on writing unit tests.

Exposing the tests is handled automatically. Behind the scenes, the following stuff happens:

<ol>
<li>The BOOST_AUTO_TEST_CASE and BOOST_AUTO_TEST_SUITE macros create static initializer objects.</li>
<li>When the shared library or executable is loaded, the initializer objects are constructed, and register your test suites and test cases with the boost::unit_test::framework, in the boost-unit_test shared library. Note that this means that all test cases in all loaded libraries (all extensions etc.) are gathered in one place automatically.</li>
</ol>



# A Guide to Boost unit-test framework

This guide is only relevant for users that can not get the unit-testing to work probably. One should not have to do any of the steps below except installing boost correctly!

This little guide is a short introduction on how to get the Boost [Unit Testing Framework](http://www.boost.org/libs/test/doc/components/utf/index.html) up and running. It consist of three steps

<ul>
<li>Make sure boost is installed correctly</li>
<li>Create a cpp unit test file</li>
<li>Setup linker properties</li>
</ul>

The boost installment and linker steps are specific for a windows environment running VC80.

== Is Boost installed correctly?==

More details can be found at the boost [getting started](http://www.boost.org/more/getting_started.html) home page. An easy way of doing some of the steps below might be to use a [installer](http://www.boost-consulting.com/products/free/beta) that takes care of all the copying and binaries. When I (kenny) install the boost library on my windows machine running VC80, I perform the following steps

<ol>
<li>Download latest boost version</li>
<li>Unpack boost to whatever location one likes</li>
<li>Create an environment variable named %BOOST% pointing to the unpacked location</li>
<li>Build bjam (run build_dist.bat file in tools sub folder of the unpacked boost library), copy bjam.exe to whatever location one likes. I choice %BOOST%/</li>
<li>Change working folder to the boost location</li>
<li>Run:
<pre>
bjam --build_dir=%TMP% --toolset=msvc-8.0 stage
</pre>
</li>
<li>copy %BOOSt%/stage/libs to %BOOST%/libs</li>
<li>Delete the folders %BOOST%/stage and %BOOST%/bin.v2</li>
<li>Install the boost-bindings, simply by unpacking the bindings into the same location as one where one have unpacked boost.</li>
</ol>

One do not need the bindings in order for using the boost unit testing framework. Actually one do not need to create boost binaries these can be created along side ones test application (see more about this below) and the added cost of longer compilation times.

The BOOST environment variable is very important, because this is how OpenTissue gets all its information about where it is supposed to find all the boost stuff that is used.

A minimal setup would be to simply unpack boost and setup an environment variable, but if one wants every part of OpenTissue to work and reduce compile times, then all of the steps above must be done.

## Creating a Test (Suite) Application

When using unit tests, create a cpp-file and write

    #include <boost/test/unit_test.hpp>
    using boost::unit_test::test_suite;

    void some_test_function()
    {
      BOOST_CHECK( ??? );
    }

    test_suite*
    init_unit_test_suite( int, char* [] ) {
      test_suite* test= BOOST_TEST_SUITE( "Unit test example" );

      test->add( BOOST_TEST_CASE( &some_test_function ) );

      return test;
    }

This is in a sense a minimal unit test program. If one needs more advanced testing then please refer to the boost documentation on the [Unit Testing Framework](http://www.boost.org/libs/test/doc/components/utf/index.html).

## Shutting up the Linker
Next open the:

<center>project properties</center>

and in the link tab page under the item

<center>Additional Library Directories</center>

add

<center>$(BOOST)\lib</center>

VC80 will automatically figure out what lib-file that is needed,
so one does not have to do any more.

<b>Optional:</b> If one gets into lib-file linker hell and do not mind
longer compile times then one can simply add the header file include

    #include  <boost/test/included/unit_test_framework.hpp>

to ones cpp-file above. Then the boost unit test framework will get compiled
along side with ones test suite.
