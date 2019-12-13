#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_ITERATE_ONCE_COLLISION_RESOLVER_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_ITERATE_ONCE_COLLISION_RESOLVER_H
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

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Iteratate Once Collision Resolver.
    * This collision resolver simply iterates once over all the contacts in increasing
    * penetration depth order and apply the specified collision law.
    */
    template< typename mbd_types, typename collision_law_policy  >
    class IterateOnceCollisionResolver 
      : public CollisionResolverInterface<mbd_types>
      , public collision_law_policy
    {
    public:

      IterateOnceCollisionResolver() {}
      virtual ~IterateOnceCollisionResolver() {}

    protected:

      typedef typename mbd_types::math_policy::real_type        real_type;
      typedef typename mbd_types::math_policy::vector3_type     vector3_type;
      typedef typename mbd_types::group_type                    group_type;
      typedef typename mbd_types::body_type                     body_type;
      typedef typename mbd_types::contact_type                  contact_type;

    public:

      class node_traits{};
      class edge_traits{};
      class constraint_traits{ };

    protected:

      /**
      * Predicate for sorting contact points in increasing order
      * corresponding to their penetration depths.
      */
      struct ContactPointComparision
      {
        bool operator()(contact_type const * x, contact_type const  *  y) const
        {
          if(x->m_distance < y->m_distance)
            return true;
          return false;
        }
      };

    public:

      void resolve_collisions(group_type & group)
      {
        if(group.size_contacts()==0)
        {
          return;
        }
        vector3_type J_a,J_b;
        group.m_contacts.sort(ContactPointComparision());
        typename group_type::indirect_contact_iterator begin = group.contact_begin();
        typename group_type::indirect_contact_iterator end = group.contact_end();
        typename group_type::indirect_contact_iterator contact;
        for(contact=begin;contact!=end;++contact)
        {
          J_b = collision_law_policy::compute_impulse(&(*contact));  // from collision_law_policy
          J_a = - J_b;
          mbd::apply_impulse(contact->get_body_A(),contact->m_rA,J_a);
          mbd::apply_impulse(contact->get_body_B(),contact->m_rB,J_b);
        }
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_RESOLVERS_MBD_ITERATE_ONCE_COLLISION_RESOLVER_H
#endif
