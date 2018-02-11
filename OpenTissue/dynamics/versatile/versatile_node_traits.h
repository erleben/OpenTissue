#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_NODE_TRAITS_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_NODE_TRAITS_H
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
      class NodeTraits
      {
      public:

        typedef typename versatile_types::value_traits           value_traits;
        typedef typename versatile_types::real_type              real_type;
        typedef typename versatile_types::vector3_type           vector3_type;
        typedef typename versatile_types::matrix3x3_type         matrix3x3_type;

      public:

        bool         m_fixed;  ///< Node is fixed.
        vector3_type m_x0;     ///< Previous coordinates.
        vector3_type m_coord;  ///< Current coordinates.
        vector3_type m_v;      ///< Current velocity.
        real_type    m_mass;   ///< Total mass.
        vector3_type m_f_ext;  ///< Accumulator of external forces.
        vector3_type m_f_con;  ///< Placeholder for accumulator of constraint forces.
        vector3_type m_f_pen;  ///< Placeholder for accumulator of penalty forces.

      public:

        NodeTraits()
          : m_fixed(false)
          , m_v( value_traits::zero(), value_traits::zero(), value_traits::zero() )
          , m_mass( value_traits::zero() )
          , m_f_ext( value_traits::zero(), value_traits::zero(), value_traits::zero() )
          , m_f_con( value_traits::zero(), value_traits::zero(), value_traits::zero() )
        {}

      public:

        vector3_type position() const { return m_coord; }

      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_NODE_TRAITS_H
#endif
