#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TETRAHEDRON_TRAITS_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TETRAHEDRON_TRAITS_H
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
  namespace versatile
  {
    namespace detail
    {

      template <typename versatile_types>
      class TetrahedronTraits
      {
      public:

        typedef typename versatile_types::real_type         real_type;
        typedef typename versatile_types::vector3_type      vector3_type;
        typedef typename versatile_types::matrix3x3_type    matrix3x3_type;
        typedef typename versatile_types::tetrahedron_type  child_type;

      public:

        vector3_type min() const
        {
          using std::min;

          child_type const & self = static_cast<child_type const &>(*this);
          vector3_type & p0 = self.i()->m_coord;
          vector3_type & p1 = self.j()->m_coord;
          vector3_type & p2 = self.k()->m_coord;
          vector3_type & p3 = self.m()->m_coord;
          return min( p0, min( p1 , min( p2, p3) ) );
        }

        vector3_type max() const
        {
          using std::max;

          child_type const & self = static_cast<child_type const &>(*this);
          vector3_type & p0 = self.i()->m_coord;
          vector3_type & p1 = self.j()->m_coord;
          vector3_type & p2 = self.k()->m_coord;
          vector3_type & p3 = self.m()->m_coord;
          return max( p0, max( p1 , max( p2, p3) ) );
        }
      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_TETRAHEDRON_TRAITS_H
#endif
