//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_bounding_volume_hierarchy.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <iostream>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_bvh);

BOOST_AUTO_TEST_CASE(top_down_functionality_testing)
{
  typedef OpenTissue::bvh::BoundingVolumeHierarchy<int,char> bvh_type;

  typedef bvh_type::bv_ptr            bv_ptr;
  typedef bvh_type::bv_const_ptr      bv_const_ptr;
  typedef bvh_type::bv_ptr_container  bv_ptr_container;

  bvh_type H;

  // Initially H should be an empty tree
  BOOST_CHECK(  H.empty() );      // Tree do not contain any nodes yet
  BOOST_CHECK(  H.size() == 0 );  // Tree do not contain any nodes yet
  BOOST_CHECK(  !H.root() );      // root should be null

  // Next we create a root node of H
  bv_ptr A = H.insert( H.root() );

  BOOST_CHECK( H.size() == 1 );           // H now got one node
  BOOST_CHECK( !H.empty() );              // H is not empty
  BOOST_CHECK( H.root() == A );           // root of H should now be A
  BOOST_CHECK( A->size() == 0);           // new root do not got any children
  BOOST_CHECK( !(A->parent()) );          // parent of root is null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H

  // Next we create a with a parent from another tree
  // This should not have any effect at all on H, since the operation invalid
  bvh_type W;
  bv_ptr P = W.insert( W.root() );
  bv_ptr AA = H.insert( P );
  BOOST_CHECK( !AA );                     // The return value of the operation should be null
  BOOST_CHECK( H.size() == 1 );           // H still only got one node
  BOOST_CHECK( !H.empty() );              // H is still not empty
  BOOST_CHECK( H.root() == A );           // root of H should now be A
  BOOST_CHECK( A->size() == 0);           // new root do not got any children
  BOOST_CHECK( !(A->parent()) );          // parent of root is null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H

  // Create a child node B of A
  bv_ptr B = H.insert( A );
  BOOST_CHECK( H.size() == 2 );           // Now H should have two nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == A );           // Root of H should still be A
  BOOST_CHECK( A->size() == 1);           // Now A should have one child
  BOOST_CHECK( !(A->parent()) );          // parent of root should still be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( B->parent() == A );        // The parent of the new node should be A
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children

  // Create another child node C of A
  bv_ptr C = H.insert( A );
  BOOST_CHECK( H.size() == 3 );           // Now H should have three nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == A );           // Root of H should still be A
  BOOST_CHECK( A->size() == 2);           // Now A should have one child
  BOOST_CHECK( !(A->parent()) );          // parent of root should still be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( B->parent() == A );        // The parent of the B node should be A
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == A );        // The parent of the C node should be A
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children

  // Try to create a node with a null-pointer parent in a non-empty tree
  bv_ptr null_ptr;
  bv_ptr D = H.insert( null_ptr );        // H already got a root, so this shouldn't work!
  BOOST_CHECK(  !D );                     // d should be null
  BOOST_CHECK( H.size() == 3 );           // Now H should have three nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == A );           // Root of H should still be A
  BOOST_CHECK( A->size() == 2);           // Now A should have one child
  BOOST_CHECK( !(A->parent()) );          // parent of root should still be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( B->parent() == A );        // The parent of the B node should be A
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == A );        // The parent of the C node should be A
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children

  // Try to clear the tree
  H.clear();
  BOOST_CHECK(  H.empty() );
  BOOST_CHECK(  H.size() == 0 );
  BOOST_CHECK(  !H.root() );       // root should be null
  //BOOST_CHECK(  !A );              // a should be null
  //BOOST_CHECK(  !B );              // b should be null
  //BOOST_CHECK(  !C );              // c should be null
  BOOST_CHECK(  !D );              // d should be null

  // Make sure that multiple clears do not cause problems
  H.clear();
  BOOST_CHECK(  H.empty() );
  BOOST_CHECK(  H.size() == 0 );
  BOOST_CHECK(  !H.root() );        // root should be null
  //BOOST_CHECK(  !A );               // a should be null
  //BOOST_CHECK(  !B );               // b should be null
  //BOOST_CHECK(  !C );               // c should be null
  BOOST_CHECK(  !D );               // d should be null
}



BOOST_AUTO_TEST_CASE(traversal_functionality_testing)
{
  typedef OpenTissue::bvh::BoundingVolumeHierarchy<int,char> bvh_type;

  typedef bvh_type::bv_ptr            bv_ptr;
  typedef bvh_type::bv_const_ptr      bv_const_ptr;
  typedef bvh_type::bv_ptr_container  bv_ptr_container;

  bvh_type H;

  bvh_type::bv_traversal_iterator b = H.begin();
  BOOST_CHECK( b == H.end() );

  bv_ptr A = H.insert( H.root() );
  bv_ptr B = H.insert( A );
  bv_ptr C = H.insert( A );
  bv_ptr D = H.insert( C );
  bv_ptr E = H.insert( C );

  b = H.begin();
  BOOST_CHECK( b != H.end() );
  BOOST_CHECK( &(*b) == A.get() );
  ++b;

  BOOST_CHECK( b != H.end() );
  BOOST_CHECK( &(*b) == B.get() );
  ++b;

  BOOST_CHECK( b != H.end() );
  BOOST_CHECK( &(*b) == C.get() );
  ++b;

  BOOST_CHECK( b != H.end() );
  BOOST_CHECK( &(*b) == D.get() );
  ++b;

  BOOST_CHECK( b != H.end() );
  BOOST_CHECK( &(*b) == E.get() );
  ++b;

  BOOST_CHECK( b == H.end() );
}

BOOST_AUTO_TEST_CASE(bottom_up_functionality_testing)
{
  typedef OpenTissue::bvh::BoundingVolumeHierarchy<int,char> bvh_type;

  typedef bvh_type::bv_ptr            bv_ptr;
  typedef bvh_type::bv_const_ptr      bv_const_ptr;
  typedef bvh_type::bv_ptr_container  bv_ptr_container;

  bvh_type H;

  bv_ptr_container empty;
  bv_ptr A = H.insert( empty );
  BOOST_CHECK( H.size() == 1 );           // Now H should have one nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == A );           // Root of H should be A
  BOOST_CHECK( !A->parent()  );           // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children

  bv_ptr B = H.insert( empty );
  BOOST_CHECK( H.size() == 2 );           // Now H should have two nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( !H.root() );               // Root of H should still be null
  BOOST_CHECK( !A->parent()  );           // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( !B->parent()  );           // The parent of the A node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( B->size() == 0);           // A should not have any children

  bv_ptr C = H.insert( empty );
  BOOST_CHECK( H.size() == 3 );           // Now H should have three nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( !H.root() );               // Root of H should still be null
  BOOST_CHECK( !A->parent()  );           // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( !B->parent()  );           // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( !C->parent()  );           // The parent of the C node should be null
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children

  bv_ptr D = H.insert( empty );
  BOOST_CHECK( H.size() == 4 );           // Now H should have four nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( !H.root() );               // Root of H should still be null
  BOOST_CHECK( !A->parent()  );           // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( !B->parent()  );           // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( !C->parent()  );           // The parent of the C node should be null
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( !D->parent()  );           // The parent of the D node should be null
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children

  bv_ptr_container left;
  left.push_back( A );
  left.push_back( B );
  bv_ptr E = H.insert( left );
  BOOST_CHECK( H.size() == 5 );           // Now H should have five nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( !H.root() );               // Root of H should still be null
  BOOST_CHECK( A->parent() == E  );       // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( B->parent() == E );        // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( !C->parent()  );           // The parent of the C node should be null
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( !D->parent()  );           // The parent of the D node should be null
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children
  BOOST_CHECK( !E->parent()  );           // The parent of the E node should be null
  BOOST_CHECK( E->owner().get() == &H );  // The owner of E is H
  BOOST_CHECK( E->size() == 2);           // E should have two children
  // should verify that E's children are A and B

  bv_ptr_container right;
  right.push_back( C );
  right.push_back( D );
  bv_ptr F = H.insert( right );
  BOOST_CHECK( H.size() == 6 );           // Now H should have six nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( !H.root() );               // Root of H should still be null
  BOOST_CHECK( A->parent() == E  );       // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( B->parent() == E );        // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == F  );       // The parent of the C node should be F
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( D->parent() == F  );       // The parent of the D node should be F
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children
  BOOST_CHECK( !E->parent()  );           // The parent of the E node should be null
  BOOST_CHECK( E->owner().get() == &H );  // The owner of E is H
  BOOST_CHECK( E->size() == 2);           // E should have two children
  BOOST_CHECK( !F->parent()  );           // The parent of the F node should be null
  BOOST_CHECK( F->owner().get() == &H );  // The owner of F is H
  BOOST_CHECK( F->size() == 2);           // F should have two children
  // should verify that E's children are A and B
  // should verify that F's children are C and D

  bv_ptr_container mid;
  mid.push_back( E );
  mid.push_back( F );
  bv_ptr G = H.insert( mid );
  BOOST_CHECK( H.size() == 7 );           // Now H should have seven nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == G);            // Root of H should still be null
  BOOST_CHECK( A->parent() == E  );       // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( B->parent() == E );        // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == F  );       // The parent of the C node should be F
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( D->parent() == F  );       // The parent of the D node should be F
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children
  BOOST_CHECK( E->parent() == G  );       // The parent of the E node should be G
  BOOST_CHECK( E->owner().get() == &H );  // The owner of E is H
  BOOST_CHECK( E->size() == 2);           // E should have two children
  BOOST_CHECK( F->parent() == G );        // The parent of the F node should be G
  BOOST_CHECK( F->owner().get() == &H );  // The owner of F is H
  BOOST_CHECK( F->size() == 2);           // F should have two children
  BOOST_CHECK( !G->parent()  );           // The parent of the G node should be null
  BOOST_CHECK( G->owner().get() == &H );  // The owner of G is H
  BOOST_CHECK( G->size() == 2);           // G should have two children
  // should verify that E's children are A and B
  // should verify that F's children are C and D
  // should verify that G's children are E and F


  //See what happens if we try to create a node with children from another tree
  bvh_type W;
  bv_ptr AA = W.insert( empty );
  bv_ptr_container not_in_family;
  not_in_family.push_back( A );
  bv_ptr bastard = H.insert( not_in_family );
  BOOST_CHECK( !bastard );                // No node should be created
  BOOST_CHECK( H.size() == 7 );           // Now H should have seven nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == G);            // Root of H should still be null
  BOOST_CHECK( A->parent() == E  );       // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( B->parent() == E );        // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == F  );       // The parent of the C node should be F
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( D->parent() == F  );       // The parent of the D node should be F
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children
  BOOST_CHECK( E->parent() == G  );       // The parent of the E node should be G
  BOOST_CHECK( E->owner().get() == &H );  // The owner of E is H
  BOOST_CHECK( E->size() == 2);           // E should have two children
  BOOST_CHECK( F->parent() == G );        // The parent of the F node should be G
  BOOST_CHECK( F->owner().get() == &H );  // The owner of F is H
  BOOST_CHECK( F->size() == 2);           // F should have two children
  BOOST_CHECK( !G->parent()  );           // The parent of the G node should be null
  BOOST_CHECK( G->owner().get() == &H );  // The owner of G is H
  BOOST_CHECK( G->size() == 2);           // G should have two children

  // see what happens if we try to create a new node with nodes that already have been used
  bv_ptr K = H.insert( mid );
  BOOST_CHECK( !K );                      // No node should be created
  BOOST_CHECK( H.size() == 7 );           // Now H should have seven nodes
  BOOST_CHECK( !H.empty() );              // H should ne non-empty
  BOOST_CHECK( H.root() == G);            // Root of H should still be null
  BOOST_CHECK( A->parent() == E  );       // The parent of the A node should be null
  BOOST_CHECK( A->owner().get() == &H );  // The owner of A is H
  BOOST_CHECK( A->size() == 0);           // A should not have any children
  BOOST_CHECK( B->parent() == E );        // The parent of the B node should be null
  BOOST_CHECK( B->owner().get() == &H );  // The owner of B is H
  BOOST_CHECK( B->size() == 0);           // B should not have any children
  BOOST_CHECK( C->parent() == F  );       // The parent of the C node should be F
  BOOST_CHECK( C->owner().get() == &H );  // The owner of C is H
  BOOST_CHECK( C->size() == 0);           // C should not have any children
  BOOST_CHECK( D->parent() == F  );       // The parent of the D node should be F
  BOOST_CHECK( D->owner().get() == &H );  // The owner of D is H
  BOOST_CHECK( D->size() == 0);           // D should not have any children
  BOOST_CHECK( E->parent() == G  );       // The parent of the E node should be G
  BOOST_CHECK( E->owner().get() == &H );  // The owner of E is H
  BOOST_CHECK( E->size() == 2);           // E should have two children
  BOOST_CHECK( F->parent() == G );        // The parent of the F node should be G
  BOOST_CHECK( F->owner().get() == &H );  // The owner of F is H
  BOOST_CHECK( F->size() == 2);           // F should have two children
  BOOST_CHECK( !G->parent()  );           // The parent of the G node should be null
  BOOST_CHECK( G->owner().get() == &H );  // The owner of G is H
  BOOST_CHECK( G->size() == 2);           // G should have two children

}

BOOST_AUTO_TEST_CASE(remove_functionality_testing)
{
  typedef OpenTissue::bvh::BoundingVolumeHierarchy<int,char> bvh_type;

  typedef bvh_type::bv_ptr            bv_ptr;
  typedef bvh_type::bv_const_ptr      bv_const_ptr;
  typedef bvh_type::bv_ptr_container  bv_ptr_container;

  bvh_type H;

  bv_ptr A = H.insert( H.root() );
  bv_ptr B = H.insert( A );
  bv_ptr C = H.insert( A );
  bv_ptr D = H.insert( C );
  bv_ptr E = H.insert( C );
  bv_ptr F = H.insert( B );
  bv_ptr G = H.insert( B );

  bool succes = H.remove( A );
  BOOST_CHECK(!succes);
  
  succes = H.remove( B );
  BOOST_CHECK(!succes);
  
  succes = H.remove( C );
  BOOST_CHECK(!succes);

  succes = H.remove( G );
  BOOST_CHECK(succes);
  BOOST_CHECK(!G->owner());
  BOOST_CHECK(!G->parent());
  BOOST_CHECK(G->size()==0);
  BOOST_CHECK(B->size()==1);

  succes = H.remove( F );
  BOOST_CHECK(succes);
  BOOST_CHECK(!G->owner());
  BOOST_CHECK(!G->parent());
  BOOST_CHECK(G->size()==0);
  BOOST_CHECK(!F->owner());
  BOOST_CHECK(!F->parent());
  BOOST_CHECK(F->size()==0);
  BOOST_CHECK(B->size()==0);
  BOOST_CHECK(H.size()==5);
}


BOOST_AUTO_TEST_SUITE_END();
