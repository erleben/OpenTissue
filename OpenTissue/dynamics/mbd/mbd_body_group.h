#ifndef OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
#define OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types >
    class BodyGroup
    {
    public:

      typedef typename mbd_types::math_policy                 math_policy;

    protected:

      typedef typename mbd_types::math_policy::index_type     size_type;
      typedef typename mbd_types::contact_ptr_container       contact_ptr_container;
      typedef typename mbd_types::constraint_ptr_container    constraint_ptr_container;
      typedef typename mbd_types::body_ptr_container          body_ptr_container;

    public:  // TODO should be protected!

      contact_ptr_container      m_contacts;          ///< Container of all current constraints in the group.
      constraint_ptr_container   m_constraints;       ///< Container of all current constraints in the group.
      body_ptr_container         m_bodies;            ///< Container of all currently active ``physical'' bodies in the group.

    public:

      BodyGroup(){}

      ~BodyGroup(){ clear(); }

    public:

      typedef typename mbd_types::indirect_constraint_iterator              indirect_constraint_iterator;
      typedef typename mbd_types::const_indirect_constraint_iterator        const_indirect_constraint_iterator;
      typedef typename mbd_types::indirect_contact_iterator                 indirect_contact_iterator;
      typedef typename mbd_types::const_indirect_contact_iterator           const_indirect_contact_iterator;
      typedef typename mbd_types::indirect_body_iterator                    indirect_body_iterator;
      typedef typename mbd_types::const_indirect_body_iterator              const_indirect_body_iterator;

      indirect_constraint_iterator             constraint_begin()       { return indirect_constraint_iterator(m_constraints.begin());          }
      indirect_constraint_iterator             constraint_end()         { return indirect_constraint_iterator(m_constraints.end());            }

      const_indirect_constraint_iterator       constraint_begin() const { return const_indirect_constraint_iterator(m_constraints.begin());    }
      const_indirect_constraint_iterator       constraint_end()   const { return const_indirect_constraint_iterator(m_constraints.end());      }

      indirect_contact_iterator                contact_begin()          { return indirect_contact_iterator(m_contacts.begin());                }
      indirect_contact_iterator                contact_end()            { return indirect_contact_iterator(m_contacts.end());                  }

      const_indirect_contact_iterator          contact_begin()    const { return const_indirect_contact_iterator(m_contacts.begin());          }
      const_indirect_contact_iterator          contact_end()      const { return const_indirect_contact_iterator(m_contacts.end());            }

      indirect_body_iterator                   body_begin()             { return indirect_body_iterator(m_bodies.begin());                     }
      indirect_body_iterator                   body_end()               { return indirect_body_iterator(m_bodies.end());                       }

      const_indirect_body_iterator             body_begin()       const { return const_indirect_body_iterator(m_bodies.begin());               }
      const_indirect_body_iterator             body_end()         const { return const_indirect_body_iterator(m_bodies.end());                 }

      size_type size_bodies()   const { return m_bodies.size();   }
      size_type size_contacts() const { return m_contacts.size(); }

      void clear()
      {
        m_bodies.clear();
        m_constraints.clear();
        m_contacts.clear();
      }

    };

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_BODY_GROUP_H
#endif
