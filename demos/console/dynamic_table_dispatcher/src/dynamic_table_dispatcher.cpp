//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <iostream>
#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/utility/dispatchers/dispatchers_dynamic_table_dispatcher.h>

using namespace std;


// Step 1: Create (or alter) a common base-class for the
// dispatchable classes, which inherits ClassIDInterface. The
// inheritance has to be virtual to avoid ambiguousity by inheriting
// ClassIDInterface multiple times.
class Base
  : virtual public OpenTissue::utility::ClassIDInterface
{};

// Step 2: Let all dispatchable objects inherite from the base class
// and from the ClassID template using themself as template parameter.
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


// Declare some dispatch functions
void myfuncAA(A& a1, A& a2)
{
  cout << "This is myfunc A-A version" << endl;
};

void myfuncAB(A& a1, B& b1)
{
  cout << "This is myfunc A-B version" << endl;
};

void myfuncBA(B& b1, A& a1)
{
  cout << "This is myfunc B-A version" << endl;
};

void myfuncBB(B& b1, B& b2)
{
  cout << "This is myfunc B-B version" << endl;
};

int main(int argc, char** argv)
{
  // Step 3: Create a dispatcher object using the DTableDispatcher
  // template and bind the dispatch functions.
  OpenTissue::utility::dispatchers::DynamicTableDispatcher<Base, false, void> dispatcher;

  dispatcher.bind(myfuncAA);
  dispatcher.bind(myfuncAB);
  dispatcher.bind(myfuncBA);
  dispatcher.bind(myfuncBB);

  // Now the dispatcher is ready for use
  A a;
  B b;
  Base& isA(a);
  Base& isB(b);
  dispatcher(isA, isB); // Prints "This is myfuncAB"

  return 0;
};
