#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_COLLISION_RESOLVER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_COLLISION_RESOLVER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_collision_resolver_interface.h>
#include <OpenTissue/dynamics/mbd/mbd_apply_impulse.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_relative_contact_velocity.h>
#include <OpenTissue/core/math/math_precision.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * A Sequential Collision Resolver.
    * This collision resovler can be used with any collision law by specifying
    * the collision law as a template policy.
    *
    * The algorithm iterates over all contacs in the specified group, in an
    * increasing order determined by their relative contact velocity in the
    * normal direction.
    *
    * In some cases such a scheme may fail, entering a inifinite loop, or
    * even computational death due to very slow convergence.ly off, but by
    * default it is on.
    *
    * Notice that if one by any chance uses negative values of coefficient
    * of restitution this algorithm will enter an infinite loop.
    */
    template< typename mbd_types, typename collision_law_policy  >
    class SequentialCollisionResolver
      : public CollisionResolverInterface<mbd_types>
      , public collision_law_policy
    {
    protected:

      typedef typename mbd_types::math_policy::real_type    real_type;
      typedef typename mbd_types::math_policy::vector3_type vector3_type;
      typedef typename mbd_types::group_type               group_type;
      typedef typename mbd_types::body_type                body_type;
      typedef typename mbd_types::edge_type                edge_type;

      typedef typename mbd_types::contact_type             contact_type;
      typedef typename mbd_types::material_type            material_type;
      typedef typename std::vector<contact_type*>            contact_ptr_heap;

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{ };

    protected:

      /**
      * This functor is used for sorting contacts in a heap data structure.
      *
      * Contacts should be sorted such that the first element has the
      * smallest relative normal contact velocity (i.e. largest
      * negative value).
      */
      struct ContactPointComparision
      {
        bool operator()(contact_type const * x, contact_type const  *  y) const
        {
          if(x->m_un < y->m_un)
            return true;
          return false;
        }
      };

    public:

      SequentialCollisionResolver() {}
      virtual ~SequentialCollisionResolver() {}

    public:

      void resolve_collisions(group_type & group)
      {
        if(group.size_contacts()==0)
        {
          return;
        }
        typename group_type::indirect_contact_iterator cbegin = group.contact_begin();
        typename group_type::indirect_contact_iterator cend = group.contact_end();
        contact_ptr_heap S;
        init_heap(cbegin,cend,S);
        vector3_type J_a,J_b;
        while(true) // TODO: refactor this to a proper test
        {
          contact_type * cp = minimum(S);
          // TODO: Check that this is what we want.
          real_type epsilon = OpenTissue::math::working_precision<real_type>();
          if(cp->m_un >= -epsilon)
            return;
          J_b = collision_law_policy::compute_impulse(cp);// from collision_law_policy
          J_a = - J_b;
          mbd::apply_impulse(cp->get_body_A(),cp->m_rA,J_a);
          mbd::apply_impulse(cp->get_body_B(),cp->m_rB,J_b);
          update_all_dependent_contacts(cp);
          update_heap(S);
        }
      }

    protected:

      /**
      * Initialize Heap
      *
      * @param cbegin    The position of the first contact in a range.
      * @param cend      The position one past the last contact in a range.
      * @param S  The heap.
      */
      template<typename iterator>
      void init_heap(iterator const & cbegin,iterator const & cend,contact_ptr_heap & S)
      {
        for(iterator contact=cbegin;contact!=cend;++contact)
        {
          update(&(*contact));
          S.push_back(&(*contact));
        }
        update_heap(S);
      }

      /**
      * Update Heap.
      *
      * @param S   The heap that should be updated.
      */
      void update_heap(contact_ptr_heap & S)
      {
        //--- KE 10-09 2004: STL heaps suck big time.  During
        //--- this update I know exactly what heap elements changed
        //--- their priorities, running a single heapify on elements
        //--- with changed priorities would be much more efficient.
        make_heap(S.begin(),S.end(),ContactPointComparision());
        sort_heap(S.begin(),S.end(),ContactPointComparision());
      }

      /**
      * Get Minimum element from Heap.
      *
      * @param S  The heap.
      *
      * @return   A pointer to the contact point with smallest contact normal velocity.
      */
      contact_type * minimum(contact_ptr_heap & S)
      {
        return S.front();
      }

      /**
      * Update All Dependent Contacts.
      * This method updates all contacts that shares a body with the
      * specified contact. The contact graph data structure is used
      * to quickly identify the contacts that needs to be updated.
      *
      * @param cp    The contact point indicating the bodies that are shared.
      */
      void update_all_dependent_contacts(contact_type * cp)
      {
        body_type * A = cp->get_body_A();
        body_type * B = cp->get_body_B();
        typename body_type::indirect_edge_iterator ebegin,eend,edge;
        typename edge_type::contact_iterator cbegin,cend,contact;
        ebegin = A->edge_begin();
        eend = A->edge_end();
        for(edge=ebegin;edge!=eend;++edge)
        {
          if(!edge->is_up_to_date())
            continue;
          cbegin = edge->contact_begin();
          cend = edge->contact_end();
          for(contact=cbegin;contact!=cend;++contact)
          {
            update(&(*contact));
          }
        }
        ebegin = B->edge_begin();
        eend = B->edge_end();
        for(edge=ebegin;edge!=eend;++edge)
        {
          if(!edge->is_up_to_date())
            continue;
          if(edge->get_body_A()==A)//--- Just to make sure that we do not process thee contacts twice
            continue;
          cbegin = edge->contact_begin();
          cend = edge->contact_end();
          for(contact=cbegin;contact!=cend;++contact)
          {
            update(&(*contact));
          }
        }
      }

      /**
      * Update Contact Point.
      *
      * @param cp    A pointer to the contact that should be
      *              updated. That is its relative normal contact
      *              velocity is re-computed.
      */
      void update(contact_type * cp)
      {
        assert(cp);
        body_type * A = cp->get_body_A();
        body_type * B = cp->get_body_B();
        assert(A);
        assert(B);
        vector3_type v_a,v_b,w_a,w_b;
        A->get_velocity(v_a);
        A->get_spin(w_a);
        B->get_velocity(v_b);
        B->get_spin(w_b);
        vector3_type u = mbd::compute_relative_contact_velocity(v_a,w_a,cp->m_rA,v_b,w_b,cp->m_rB);
        //--- Normal points from A to B, relative velocity is: u = (u_b - u_a)
        //--- so if u_n <0 then we have a collision
        cp->m_un = cp->m_n*u;
      }

    };// End of class SequentialCollisionResolver

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_COLLISION_RESOLVER_H
#endif
