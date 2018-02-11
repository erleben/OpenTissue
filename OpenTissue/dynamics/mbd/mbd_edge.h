#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_EDGE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_EDGE_H
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
    class Edge 
      : public mbd_types::edge_traits
    {
    public:

      typedef typename mbd_types::math_policy::index_type    size_type;
      typedef typename mbd_types::body_type                  body_type;
      typedef typename mbd_types::edge_type                  edge_type;
      typedef typename mbd_types::material_type              material_type;
      typedef typename mbd_types::contact_iterator           contact_iterator;
      typedef typename mbd_types::const_contact_iterator     const_contact_iterator;

    protected:

      typedef typename mbd_types::contact_container          contact_container;
      typedef typename mbd_types::collision_detection_policy collision_detection_policy;

    protected:

      body_type * m_A;                      ///< Pointer to body with smallest index.
      body_type * m_B;                      ///< Pointer to body with highest index.
      contact_container m_contacts;         ///< All contacts between A and B.

      bool m_prunned;                       ///< Booelan flag indicating whether the edge has been prunned by some collision detection module during a collision query.

    public:

      material_type * m_material;           ///< A pointer to the material properties between A and B.
      bool m_relative_resting;              ///< Boolean value indicating whatever the bodies
                                            ///< are in relative rest
                                            ///< since the last invocation
                                            ///< of the collision detection engine. This
                                            ///< flag can be used to determine whatever
                                            ///< caching could be exploited.
                                            ///< (for instance to skip narrow phase collision
                                            ///< detection and contact determination).
      size_type m_updated_time_stamp;       ///< Timestamp of last time when this edge contains up-to-date
                                            ///< information. Compare this to the time stamp of the collision
                                            ///< detection engine, if they are equal the edge contains valid
                                            ///< cached contact information, if they are unequal the cached
                                            ///< information is non-valid and should not be used.
      collision_detection_policy * m_collision_detection; ///< A pointer to the collision detection engine that detected this edge.

    public:

      body_type const * get_body_A() const { return m_A; }
      body_type       * get_body_A()       { return m_A; }

      body_type const * get_body_B() const { return m_B; }
      body_type       * get_body_B()       { return m_B; }

      contact_iterator       contact_begin()       { return contact_iterator(m_contacts.begin());       }
      contact_iterator       contact_end()         { return contact_iterator(m_contacts.end());         }
      const_contact_iterator contact_begin() const { return const_contact_iterator(m_contacts.begin()); }
      const_contact_iterator contact_end()   const { return const_contact_iterator(m_contacts.end());   }

      size_type size_contacts() const { return m_contacts.size(); }

      contact_container   * get_contacts()       { return &m_contacts; }
      material_type const * get_material() const { return m_material;  }
      material_type       * get_material()       { return m_material;  }

    public:

      Edge()
        : m_prunned(false)
      {}

      virtual ~Edge(){}

    public:


      /** 
       * Get Prunned Flag.
       *
       * @return   A reference to a booelan flag indicating whether the edge has been prunned by some collision detection module during a collision query.
       */
      bool       & prunned()       { return m_prunned; }
      bool const & prunned() const { return m_prunned; }

      void init(body_type * const body_A, body_type * const body_B)
      {
        assert(body_A || !"Edge::init(): body A was null");
        assert(body_B || !"Edge::init(): body A was null");
        if(body_A->get_index() < body_B->get_index())
        {
          m_A = const_cast<body_type*>(body_A);
          m_B = const_cast<body_type*>(body_B);
        }
        else
        {
          m_A = const_cast<body_type*>(body_B);
          m_B = const_cast<body_type*>(body_A);
        }

        m_material            = 0;
        m_contacts.clear();
        m_relative_resting    = false;
        m_updated_time_stamp  = 0;
        m_collision_detection = 0;
      }

      bool operator==(edge_type const & edge) const
      {
        assert(m_A      || !"Edge::operator==(): body A was null");
        assert(m_B      || !"Edge::operator==(): body B was null");
        assert(edge.m_A || !"Edge::operator==(): body A on edge was null");
        assert(edge.m_B || !"Edge::operator==(): body B on edge was null");
        return (m_A==edge.m_A && m_B==edge.m_B)? true : false;
      } 
      bool operator!=(edge_type const & edge) const {  return !(*this==edge);  }

      bool operator<(edge_type const & edge)const
      {
        assert(m_A      || !"Edge::operator<(): body A was null");
        assert(m_B      || !"Edge::operator<(): body B was null");
        assert(edge.m_A || !"Edge::operator<(): body A on edge was null");
        assert(edge.m_B || !"Edge::operator<(): body B on edge was null");

        if(m_A<edge.m_A && m_B<edge.m_B)
          return true;
        if(m_A==edge.m_A && m_B<edge.m_B)
          return true;
        return false;
      }

      size_type hash_key() const { return edge_type::hash_key(m_A,m_B);  }

      /**
      * Get Hash Key.
      * This method is usefull if one want to get the hash key of a body
      * edge that needs to be looked up in a hash map.
      *
      * @param A
      * @param B
      * @return
      */
      static size_type hash_key(body_type const * A,body_type const * B)
      {
        assert(A      || !"Edge::hash_key(): body A was null");
        assert(B      || !"Edge::hash_key(): body B was null");
        assert(A->get_index()!=B->get_index() || !"Edge::hash_key(): index of A and B were the same");

        if(A->get_index()<B->get_index())
          return ((A->get_index()<<16)|(B->get_index()&0x0000FFFF)) ;
        return ((B->get_index()<<16)|(A->get_index()&0x0000FFFF));
      }

      /**
      * Is Up-To-Date Query.
      * This method determines whether the cached contact information in the edge is up-to-date.
      *
      * @return        If edge is up-to-date then the return value is true otherwise it is false.
      */
      bool is_up_to_date( ) const
      {
        assert(m_collision_detection || !"Edge::is_up_to_date(): collision detection pointer was null");
        return (m_updated_time_stamp == m_collision_detection->get_time_stamp());
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_EDGE_H
#endif
