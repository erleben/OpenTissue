#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_TRUNCATING_COLLISION_RESOLVER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_TRUNCATING_COLLISION_RESOLVER_H
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
    * even computational death due to very slow convergence. To alleviate
    * these problems the algorithm supports truncating impulses. This means
    * that if certain criteria are fulfilled then the normal restitution
    * is set to zero before computing an impulse.
    *
    * If one is too aggressive in setting up the truncation criteria's then
    * it can have the effect of low restitution values are seen as
    * zero-restitution. In our experience the default settings tend to
    * damp the restituion coefficient towards zero when in the range 0 -.5, above .5 does
    * not seem to be visually noticable.
    *
    * Also large stacks of objects, like 20 objects placed un top of each other seem
    * to make truncation kick in regardless of initial velocities and the values of
    * coefficient of restitution.
    *
    */
    template< typename mbd_types, typename collision_law_policy  >
    class SequentialTruncatingCollisionResolver 
      : public CollisionResolverInterface<mbd_types>
      , public collision_law_policy
    {
    public:

      typedef typename mbd_types::math_policy::index_type    size_type;
      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::math_policy::vector3_type  vector3_type;
      typedef typename mbd_types::math_policy::value_traits  value_traits;
      typedef typename mbd_types::group_type                 group_type;
      typedef typename mbd_types::body_type                  body_type;
      typedef typename mbd_types::edge_type                  edge_type;
      typedef typename mbd_types::contact_type               contact_type;
      typedef typename mbd_types::material_type              material_type;
      typedef typename std::vector<contact_type*>              contact_ptr_heap;

    public:

      class node_traits{};
      
      class edge_traits{};
      
      class constraint_traits
      {
      public:

        bool      m_stcr_truncated;        ///< Boolean flag, which is set when a truncation impulse were applied.
        size_type m_stcr_resolved;         ///< This member counts the number of times a contact was resolved.

      public:

        constraint_traits()
          : m_stcr_truncated(false)
          , m_stcr_resolved(0) 
        {}

      };

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

    protected:

      size_type m_resolve_limit;              ///< The maximum number of times a contact can be resolved before the impulse is truncated (default value is 20).
      real_type m_truncation_fraction;        ///< The fraction of largest initial approach velocity, which will result in a truncation impulse to be used (default value is 1000).

    public:

      SequentialTruncatingCollisionResolver()
        : m_resolve_limit(20)
        , m_truncation_fraction(1000.) 
      {}

      virtual ~SequentialTruncatingCollisionResolver() {}

    public:

      real_type const & truncation_fraction() const {return m_truncation_fraction;}

      /**
      * Set Truncation Fraction.
      *
      * @param value      A positive value indicating the new truncation fraction. That
      *                   is the fraction of the initial largest approach velocity, which
      *                   yields the velocity threshold for when a collision should be
      *                   truncated. That is when approach velocity of a collision is
      *                   less than the threshold the collision is truncated.
      */
      void set_truncation_fraction(real_type const & value)
      {
        if(value>0)
          m_truncation_fraction = value;
      }

      size_type const & truncation_limit() const {return this->m_truncation_limit;}
      size_type       & truncation_limit()       {return this->m_truncation_limit;}

    public:

      /**
      * Resolve Collisions
      *
      * @param group    The group on which to resolve collisions.
      */
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
        contact_type * cp = minimum(S);
        assert(cp);
        real_type truncation_threshold = 0;
        assert(m_truncation_fraction>0);
        truncation_threshold = cp->m_un/m_truncation_fraction;
        if(truncation_threshold>0)
          return;
        assert(m_resolve_limit>0);
        vector3_type J_a,J_b;
        while(true) // TODO: refactor this to a proper test
        {
          cp = minimum(S);
          // TODO: Check that this is what we want.
          real_type epsilon = OpenTissue::math::working_precision<real_type>();
          if(cp->m_un >= -epsilon)
            return;
          ++(cp->m_stcr_resolved);
          if(((cp->m_stcr_resolved > m_resolve_limit) || (cp->m_un > truncation_threshold)))
          {
            real_type e_n = cp->m_material->normal_restitution();
            cp->m_material->normal_restitution() = value_traits::zero();
            J_b = collision_law_policy::compute_impulse(cp); // from collision_law_policy
            cp->m_material->normal_restitution() = e_n;
            cp->m_stcr_truncated = true;
          }
          else
          {
            J_b = collision_law_policy::compute_impulse(cp);  // from collision_law_policy
          }
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
          contact->m_stcr_resolved = 0;
          contact->m_stcr_truncated = false;
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
        if(cp->m_stcr_truncated)
        {
          cp->m_un = value_traits::zero();
          vector3_type tmp;
          A->get_velocity(tmp);
          truncate(tmp);
          A->set_velocity(tmp);
          A->get_spin(tmp);
          truncate(tmp);
          A->set_spin(tmp);
          B->get_velocity(tmp);
          truncate(tmp);
          B->set_velocity(tmp);
          B->get_spin(tmp);
          truncate(tmp);
          B->set_spin(tmp);
          cp->m_stcr_truncated = false;
        }
        else
        {
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
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_SEQUENTIAL_TRUNCATING_COLLISION_RESOLVER_H
#endif
