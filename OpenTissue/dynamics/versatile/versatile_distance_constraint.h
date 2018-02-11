#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_DISTANCE_CONSTRAINT_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_DISTANCE_CONSTRAINT_H
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
      class DistanceConstraint
      {
      public:

        typedef typename versatile_types::value_traits  value_traits;
        typedef typename versatile_types::real_type     real_type;
        typedef typename versatile_types::vector3_type  vector3_type;
        typedef typename versatile_types::node_type     node_type;

      public:

        real_type    m_k;   ///< Spring coefficient.
        real_type    m_b;   ///< Damping coefficient.
        real_type    m_D0;  ///< Initial distance between nodes.
        node_type  * m_ni;  ///< Pointer to i'th node.
        node_type  * m_nj;  ///< Pointer to j'th node.

        real_type    m_c_yield;  ///< Plasticity yield threshold.
        real_type    m_c_creep;  ///< Plasticity creep threshold.
        real_type    m_c_max;    ///< Plasticity max threshold.
        real_type    m_l_plastic;  ///< Plastic elongation!

      public:

        DistanceConstraint()
          : m_k( value_traits::zero() )
          , m_b( value_traits::zero() )
          , m_D0( value_traits::zero() )
          , m_ni(0)
          , m_nj(0)
          , m_c_yield( value_traits::infinity() )
          , m_c_creep( value_traits::zero() )
          , m_c_max( value_traits::zero() )
          , m_l_plastic( value_traits::zero() )
        {}

      public:

        void initialize(node_type & ni, node_type  & nj)
        {
          using std::sqrt;

          m_ni = & ni;
          m_nj = & nj;
          vector3_type d = m_nj->m_coord - m_ni->m_coord;
          m_D0 = sqrt(d*d);
        }

        void apply()
        {
          using std::min;
          using std::sqrt;

          vector3_type e = m_nj->m_coord - m_ni->m_coord;
          real_type e_norm = sqrt(e*e);
          real_type C  =  ( (e_norm-m_D0) / m_D0);
          vector3_type dCdj = ( e/ (m_D0 * e_norm));
          vector3_type dCdi = - dCdj;
          real_type factor = value_traits::zero();
          if(m_k>value_traits::zero())
            factor -= m_k *C;
          if(m_b>value_traits::zero())
            factor -= m_b * (dCdi*m_ni->m_v + dCdj*m_nj->m_v);
          if(factor)
          {
            m_ni->m_f_con += factor*dCdi;
            m_nj->m_f_con += factor*dCdj;
          }
          //--- KE 04-07-2005: Hmm, the plasticity stuff below is just and I idea
          //--- I got, I have no idea if it actually works:-)
          real_type l = e_norm-m_D0;
          if(l>m_c_yield)
          {
            m_l_plastic += m_c_creep*(l-m_l_plastic);
            m_l_plastic = min(m_l_plastic,m_c_max);
          }
          vector3_type f_plastic = vector3_type(-m_k*m_l_plastic );
          m_ni->m_f_con -= f_plastic;
          m_nj->m_f_con += f_plastic;
        }

        real_type compute_internal_energy()
        {
          using std::sqrt;

          vector3_type e = m_nj->m_coord - m_ni->m_coord;
          real_type e_norm = sqrt(e*e);
          real_type C  =  ( (e_norm-m_D0) / m_D0);
          return value_traits::half()*m_k*C*C;
        }
      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_DISTANCE_CONSTRAINT_H
#endif
