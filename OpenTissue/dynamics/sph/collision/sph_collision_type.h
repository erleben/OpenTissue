#ifndef OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_TYPE_H
#define OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_TYPE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_empty_traits.h>

namespace OpenTissue
{
  namespace sph
  {
    /**
    * SPH Collision Type Class.
    * This class defines the type interface for a single collision.
    */
    template <
      typename Real_Type
      , typename Vector_Type
      , typename Collision_Traits = OpenTissue::utility::EmptyTraits
    >
    class CollisionType : public Collision_Traits
    {
    public:
      typedef Real_Type  real_type;
      typedef Vector_Type  vector;
      typedef Vector_Type  point;

    public:
      /**
      * Default Constructor.
      */
      CollisionType()
        : m_cp(0)
        , m_normal(0)
        , m_depth(0)
      {
      }

      /**
      * Default Constructor.
      */
      CollisionType(const point& contact, const vector& normal = vector(0), const real_type& depth = 0)
        : m_cp(contact)
        , m_normal(normal)
        , m_depth(depth)
      {
      }

      /**
      * Deconstructor.
      */
      virtual ~CollisionType()
      {
      }

    public:
      /**
      *
      */
      const point& contact() const
      {
        return m_cp;
      }

      /**
      *
      */
      point& contact()
      {
        return m_cp;
      }

      /**
      *
      */
      const vector& normal() const
      {
        return m_normal;
      }

      /**
      *
      */
      vector& normal()
      {
        return m_normal;
      }

      /**
      *
      */
      const real_type& penetration() const
      {
        return m_depth;
      }

      /**
      *
      */
      real_type& penetration()
      {
        return m_depth;
      }

    protected:
      point  m_cp;   ///< contact point (on the surface)
      vector  m_normal;   ///< surface normal (at cp)
      real_type  m_depth;   ///< penetration depth

    }; // End class CollisionType

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_TYPE_H
#endif
