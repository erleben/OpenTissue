//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/utility/dispatchers/dispatchers_dynamic_table_dispatcher.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

//-----------------------------------------------------------------------------
// This unit test program tests dynamical multiple dispatcher using the
// DTable implementation.
//
// In order to use DTable the programmer have to do the following 3 steps:
// Step 1: Create (or alter) a common base-class for the dispatchable classes, which
// inherits ClassIDInterface. The inheriting has to be virtual public to avoid 
// ambiguousity in the derived classes.
// Step 2: Let all dispatchable objects inherite from the base-class and from
// the ClassID template using themself as template parameter.
// Step 3: Create a dispatcher objects using the MultiDispatcher template and
// bind the dispatch functions.
//-----------------------------------------------------------------------------


// Declarations for the unit test wrapped in a namespace.
namespace DTable 
{

  // Step 1: Declare a base class.
  // This class must always inherite ClassIDInterface.
  class Base
    : virtual public OpenTissue::utility::ClassIDInterface
  {
  };

  // Step 2: Declare a few derived classes.
  // These must always inherite Base and ClassID<Self>, where "Self"
  // is the class' own name.
  class A
    : public Base
    , public OpenTissue::utility::ClassID<A> 
  {
  public:

    // This method should be added to get rid of stupid compiler warning about class_id is inherited via dominance
    size_t const class_id() const { return OpenTissue::utility::ClassID<A>::class_id(); }
  };

  class B
    : public Base
    , public OpenTissue::utility::ClassID<B> 
  {
  public:

    // This method should be added to get rid of stupid compiler warning about class_id is inherited via dominance
    size_t const class_id() const { return OpenTissue::utility::ClassID<B>::class_id(); }
  };

  //A simple enumerator used to identify myfunc().
  enum FuncID { AA, AB, BA, BB };

  // A very simple dispatch function, which simply returns it's own ID, R.
  template <class T1, class T2, FuncID R>
  FuncID myfunc(T1& t1, T2& t2)
  {
      return R;
  };

  // A very simple dispatch function, which simply returns it's own ID, R.
  // This one also takes a third argument, which is "touched" using it's
  // operator++().
  template <class T1, class T2, FuncID R, class T3>
  FuncID myfunc3(T1& t1, T2& t2, T3& t3)
  {
      t3++;
      return R;
  };
}  


using namespace boost::unit_test;

BOOST_AUTO_TEST_SUITE(opentisue_utility_dispatchers_dtable);

  //---------------------------------------------------------------------------
  // This test suite tests the basic features of the automatic ID generation.
  //---------------------------------------------------------------------------
  BOOST_AUTO_TEST_CASE(classid_test)
  {
    // Declare some instances of A and B
    DTable::A a;
    DTable::B b;

    // Make some "anonymous" references via their common base class
    DTable::Base& isA = a;
    DTable::Base& isB = b;

    // Test that the ID generator actually gives out increasing unique IDs.
    BOOST_CHECK_EQUAL(DTable::A::id(), 0u);
    BOOST_CHECK_EQUAL(DTable::B::id(), 1u);

    // Test that the virtual class_id() method returns the same as the
    // static id() method.
    BOOST_CHECK_EQUAL(a.class_id(), DTable::A::id());
    BOOST_CHECK_EQUAL(b.class_id(), DTable::B::id());

    // Test that the virtual class_id() method also works when called 
    // from a reference to the base class.
    BOOST_CHECK_EQUAL(isA.class_id(), DTable::A::id());
    BOOST_CHECK_EQUAL(isB.class_id(), DTable::B::id());
  }

  //---------------------------------------------------------------------------
  // This test suite tests the DTable dispatcher without mirroring
  //---------------------------------------------------------------------------
  BOOST_AUTO_TEST_CASE(dtable_test)
  {
    // Declare some instances of A and B
    DTable::A a;
    DTable::B b;
    // Make some "anonymous" references via their common base class
    DTable::Base& Ba(a);
    DTable::Base& Bb(b);

    // Step 3: Create a dispatcher
    // Note that this dispatcher doesn't use mirroring.
    OpenTissue::utility::dispatchers::DynamicTableDispatcher<DTable::Base, false, DTable::FuncID> v;

    // Bind dispatch functions to the dispatcher.
    // Note that the dispatcher discovers the type of the parameters
    // automatically.
    v.bind(DTable::myfunc<DTable::A, DTable::A, DTable::AA>);
    v.bind(DTable::myfunc<DTable::A, DTable::B, DTable::AB>);
    v.bind(DTable::myfunc<DTable::B, DTable::A, DTable::BA>);

    // Check that the dispatcher finds the right
    // for some combinations of A and B
    BOOST_CHECK_EQUAL(v(Ba, Ba), DTable::AA);
    BOOST_CHECK_EQUAL(v(Ba, Bb), DTable::AB);
    BOOST_CHECK_EQUAL(v(Bb, Ba), DTable::BA);

    // The dispathcer is expected to throw UnboundDispatchFunction as
    // no dispatch function has been bound for this combination.
    BOOST_CHECK_THROW(v(Bb, Bb), OpenTissue::utility::dispatchers::UnboundDispatchFunction);
    
    // Bind a dispatch function for the missing combination and check that
    // the dispatcher can find it afterwards.
    v.bind(DTable::myfunc<DTable::B, DTable::B, DTable::BB>);
    BOOST_CHECK_EQUAL(v(Bb, Bb), DTable::BB);
  }

  //---------------------------------------------------------------------------
  // This test suite tests the DTable dispatcher with mirroring
  //---------------------------------------------------------------------------
  BOOST_AUTO_TEST_CASE(dtable_mirror_test)
  {
    // Declare some instances of A and B
    DTable::A a;
    DTable::B b;
    // Make some "anonymous" references via their common base class
    DTable::Base& Ba(a);
    DTable::Base& Bb(b);

    // Step 3: Create a dispatcher
    // Note that this dispatcher uses mirroring.
    OpenTissue::utility::dispatchers::DynamicTableDispatcher<DTable::Base, true, DTable::FuncID> v;

    // Bind a single dispatch function to the dispatcher
    // and check that the dispatcher finds it correctly no matter
    // the order of the types.
    v.bind(DTable::myfunc<DTable::A, DTable::B, DTable::AB>);
    BOOST_CHECK_EQUAL(v(Ba, Bb), DTable::AB);
    BOOST_CHECK_EQUAL(v(Bb, Ba), DTable::AB);
    
    // Bind a new dispatch function to the dispatcher and
    // check that the dispatcher has replaced the old function
    // in both combinations.
    v.bind(DTable::myfunc<DTable::B, DTable::A, DTable::BA>);
    BOOST_CHECK_EQUAL(v(Ba, Bb), DTable::BA);
    BOOST_CHECK_EQUAL(v(Bb, Ba), DTable::BA);
  }

  //---------------------------------------------------------------------------
  // This test suite tests the DTable dispatcher with a third parameter but
  // without mirroring
  //---------------------------------------------------------------------------
  BOOST_AUTO_TEST_CASE(dtable_3var_test)
  {
    // Declare some instances of A and B
    DTable::A a;
    DTable::B b;
    // Make some "anonymous" references via their common base class
    DTable::Base& Ba(a);
    DTable::Base& Bb(b);

    // Step 3: Create a dispatcher
    // Note that this dispatcher takes a third argument but doesn't
    // use mirroring.
    OpenTissue::utility::dispatchers::DynamicTableDispatcher<DTable::Base, false, DTable::FuncID, int> v;

    // Bind dispatch functions to the dispatcher.
    // Note that the dispatcher discovers the type of the parameters
    // automatically.
    v.bind(DTable::myfunc3<DTable::A, DTable::A, DTable::AA, int>);
    v.bind(DTable::myfunc3<DTable::A, DTable::B, DTable::AB, int>);
    v.bind(DTable::myfunc3<DTable::B, DTable::A, DTable::BA, int>);
    v.bind(DTable::myfunc3<DTable::B, DTable::B, DTable::BB, int>);

    // Check that the dispatcher finds the right
    // for some combinations of A and B. Also check that the third 
    // variable (the counter) was touched (incremented) each time.
    int counter = 0;
    BOOST_CHECK_EQUAL(v(Ba, Ba, counter), DTable::AA);
    BOOST_CHECK_EQUAL(v(Ba, Bb, counter), DTable::AB);
    BOOST_CHECK_EQUAL(v(Bb, Ba, counter), DTable::BA);
    BOOST_CHECK_EQUAL(v(Bb, Bb, counter), DTable::BB);
    BOOST_CHECK_EQUAL(counter, 4);
  }

  //---------------------------------------------------------------------------
  // This test suite tests the DTable dispatcher with a third parameter and
  // mirroring
  //---------------------------------------------------------------------------
  BOOST_AUTO_TEST_CASE(dtable_3var_mirror_test)
  {
    // Declare some instances of A and B
    DTable::A a;
    DTable::B b;
    // Make some "anonymous" references via their common base class
    DTable::Base& Ba(a);
    DTable::Base& Bb(b);

    // Step 3: Create a dispatcher
    // Note that this dispatcher uses mirroring and takes a third argument.
    OpenTissue::utility::dispatchers::DynamicTableDispatcher<DTable::Base, true, DTable::FuncID, int> v;

    // Bind a single dispatch function to the dispatcher
    // and check that the dispatcher finds it correctly no matter
    // the order of the types. Also check that the third variable (the counter)
    // was touched (incremented) both times.
    int counter = 0;
    v.bind(DTable::myfunc3<DTable::A, DTable::B, DTable::AB, int>);
    BOOST_CHECK_EQUAL(v(Ba, Bb, counter), DTable::AB);
    BOOST_CHECK_EQUAL(v(Bb, Ba, counter), DTable::AB);
    BOOST_CHECK_EQUAL(counter, 2);
    
    // Bind a new dispatch function to the dispatcher and
    // check that the dispatcher has replaced the old function
    // in both combinations. Again the third variable should have been
    // touched both times.
    v.bind(DTable::myfunc3<DTable::B, DTable::A, DTable::BA, int>);
    BOOST_CHECK_EQUAL(v(Ba, Bb, counter), DTable::BA);
    BOOST_CHECK_EQUAL(v(Bb, Ba, counter), DTable::BA);
    BOOST_CHECK_EQUAL(counter, 4);
  }
BOOST_AUTO_TEST_SUITE_END();
