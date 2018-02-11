//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/containers_heap.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

using namespace OpenTissue;
using namespace OpenTissue::containers;





BOOST_AUTO_TEST_SUITE(opentissue_utility_heap);

  BOOST_AUTO_TEST_CASE(heap_testing)
  {
    typedef Heap<char,double>          heap_type;
    typedef heap_type::heap_iterator   heap_iterator;
    {
      heap_type H0;
      heap_type H1;
      heap_type H2;
      heap_type H3;
      heap_type H4;

      BOOST_CHECK( H0.size()==0 );
      BOOST_CHECK( H1.size()==0 );
      BOOST_CHECK( H2.size()==0 );
      BOOST_CHECK( H3.size()==0 );
      BOOST_CHECK( H4.size()==0 );

      heap_iterator A1 = H1.push('A');
      BOOST_CHECK( H1.size()== 1  );

      heap_iterator A2 = H2.push('A');
      heap_iterator B2 = H2.push('B');
      BOOST_CHECK( H2.size()== 2  );

      heap_iterator A3 = H3.push('A');
      heap_iterator B3 = H3.push('B');
      heap_iterator C3 = H3.push('C');
      BOOST_CHECK( H3.size()== 3  );

      heap_iterator A4 = H4.push('A');
      heap_iterator B4 = H4.push('B');
      heap_iterator C4 = H4.push('C');
      heap_iterator D4 = H4.push('D');
      BOOST_CHECK( H4.size()== 4  );

      H0.clear();
      H1.clear();
      H2.clear();
      H3.clear();
      H4.clear();
      BOOST_CHECK( H0.size()==0 );
      BOOST_CHECK( H1.size()==0 );
      BOOST_CHECK( H2.size()==0 );
      BOOST_CHECK( H3.size()==0 );
      BOOST_CHECK( H4.size()==0 );
    }
    {
      heap_type H0;
      heap_type H1;
      heap_type H2;
      heap_type H3;
      heap_type H4;

      heap_iterator A1 = H1.push('A');
      heap_iterator B2 = H2.push('B');
      heap_iterator A2 = H2.push('A');
      heap_iterator C3 = H3.push('C');
      heap_iterator B3 = H3.push('B');
      heap_iterator A3 = H3.push('A');
      heap_iterator D4 = H4.push('D');
      heap_iterator C4 = H4.push('C');
      heap_iterator B4 = H4.push('B');
      heap_iterator A4 = H4.push('A');

      BOOST_CHECK( A1->get_feature() == 'A' );
      BOOST_CHECK( A2->get_feature() == 'A' );
      BOOST_CHECK( A3->get_feature() == 'A' );
      BOOST_CHECK( A4->get_feature() == 'A' );
      BOOST_CHECK( B2->get_feature() == 'B' );
      BOOST_CHECK( B3->get_feature() == 'B' );
      BOOST_CHECK( B4->get_feature() == 'B' );
      BOOST_CHECK( C3->get_feature() == 'C' );
      BOOST_CHECK( C4->get_feature() == 'C' );
      BOOST_CHECK( D4->get_feature() == 'D' );

      A1->priority() = 40.0;
      A2->priority() = 40.0;
      A3->priority() = 40.0;
      A4->priority() = 40.0;
      B2->priority() = 30.0;
      B3->priority() = 30.0;
      B4->priority() = 30.0;
      C3->priority() = 20.0;
      C4->priority() = 20.0;
      D4->priority() = 10.0;

      {
        heap_iterator h = H0.begin();
        BOOST_CHECK( h == H0.end() );
      }
      {
        heap_iterator h = H1.begin();
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h == H1.end() );
      }
      {
        heap_iterator h = H2.begin();
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h == H2.end() );
      }
      {
        heap_iterator h = H3.begin();
        BOOST_CHECK( h->get_feature() == 'C' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h == H3.end() );
      }
      {
        heap_iterator h = H4.begin();
        BOOST_CHECK( h->get_feature() == 'D' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'C' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h == H4.end() );
      }

      H0.heapify();
      H1.heapify();
      H2.heapify();
      H3.heapify();
      H4.heapify();

      // iterators are persistent (iterator validity)
      BOOST_CHECK( A1->get_feature() == 'A' );
      BOOST_CHECK( A2->get_feature() == 'A' );
      BOOST_CHECK( A3->get_feature() == 'A' );
      BOOST_CHECK( A4->get_feature() == 'A' );
      BOOST_CHECK( B2->get_feature() == 'B' );
      BOOST_CHECK( B3->get_feature() == 'B' );
      BOOST_CHECK( B4->get_feature() == 'B' );
      BOOST_CHECK( C3->get_feature() == 'C' );
      BOOST_CHECK( C4->get_feature() == 'C' );
      BOOST_CHECK( D4->get_feature() == 'D' );

      BOOST_CHECK( A1->priority() == 40.0 );
      BOOST_CHECK( A2->priority() == 40.0 );
      BOOST_CHECK( A3->priority() == 40.0 );
      BOOST_CHECK( A4->priority() == 40.0 );
      BOOST_CHECK( B2->priority() == 30.0 );
      BOOST_CHECK( B3->priority() == 30.0 );
      BOOST_CHECK( B4->priority() == 30.0 );
      BOOST_CHECK( C3->priority() == 20.0 );
      BOOST_CHECK( C4->priority() == 20.0 );
      BOOST_CHECK( D4->priority() == 10.0 );

      {
        heap_iterator h = H0.begin();
        BOOST_CHECK( h == H0.end() );
      }
      {
        heap_iterator h = H1.begin();
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h == H1.end() );
      }
      {
        heap_iterator h = H2.begin();
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h == H2.end() );
      }
      {
        heap_iterator h = H3.begin();
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'C' );
        ++h;
        BOOST_CHECK( h == H3.end() );
      }
      {
        heap_iterator h = H4.begin();
        BOOST_CHECK( h->get_feature() == 'A' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'B' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'C' );
        ++h;
        BOOST_CHECK( h->get_feature() == 'D' );
        ++h;
        BOOST_CHECK( h == H4.end() );
      }
    }

    heap_type H;
    heap_iterator A = H.push('A');
    heap_iterator B = H.push('B');
    heap_iterator C = H.push('C');
    heap_iterator D = H.push('D');

    A->priority() = 10.0;
    B->priority() = 20.0;
    C->priority() = 30.0;
    D->priority() = 40.0;

    H.heapify();
    {
      heap_iterator h = H.begin();
      BOOST_CHECK( h->get_feature() == 'D' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'C' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'B' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'A' );
      ++h;
      BOOST_CHECK( h == H.end() );
    }

    heap_iterator top = H.top();
    BOOST_CHECK( top->get_feature() == D->get_feature() );

    D = H.get_heap_iterator('D');
    BOOST_CHECK( D->get_feature() == 'D' );

    D->priority() = 25.0;
    H.heapify(D);
    {
      heap_iterator h = H.begin();
      BOOST_CHECK( h->get_feature() == 'C' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'D' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'B' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'A' );
      ++h;
      BOOST_CHECK( h == H.end() );
    }

    top = H.top();
    BOOST_CHECK( top->get_feature() == 'C' );

    A = H.get_heap_iterator('A');
    BOOST_CHECK( A->get_feature() == 'A' );

    A->priority() = 40.0;
    H.heapify(A);
    {
      heap_iterator h = H.begin();
      BOOST_CHECK( h->get_feature() == 'A' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'C' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'D' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'B' );
      ++h;
      BOOST_CHECK( h == H.end() );
    }

    top = H.top();
    BOOST_CHECK( top->get_feature() == 'A' );

    H.erase('A');
    H.erase('B');
    {
      heap_iterator h = H.begin();
      BOOST_CHECK( h->get_feature() == 'C' );
      ++h;
      BOOST_CHECK( h->get_feature() == 'D' );
      ++h;
      BOOST_CHECK( h == H.end() );
    }

    heap_type H0;

    top = H.top();
    H.heapify( top );

    top = H0.top();
    BOOST_CHECK_THROW( H0.heapify( top ), std::logic_error );

    heap_iterator end = H.end();
    BOOST_CHECK_THROW( H.heapify(end) , std::logic_error );
    end = H0.end();
    BOOST_CHECK_THROW( H0.heapify(end) , std::logic_error );

    BOOST_CHECK_THROW( H0.erase('A') , std::logic_error );    
    BOOST_CHECK_THROW( H.erase('Q') , std::invalid_argument );    

    BOOST_CHECK_THROW( H0.get_heap_iterator('D') , std::logic_error );
    BOOST_CHECK_THROW( H.get_heap_iterator('Q') , std::invalid_argument );
  }

BOOST_AUTO_TEST_SUITE_END();
