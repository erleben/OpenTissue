#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_AREA_CONSTRAINT_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_AREA_CONSTRAINT_H
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
      class AreaConstraint
      {
      public:

        typedef typename versatile_types::value_traits  value_traits;
        typedef typename versatile_types::real_type     real_type;
        typedef typename versatile_types::vector3_type  vector3_type;
        typedef typename versatile_types::node_type     node_type;

      public:

        real_type    m_k;   ///< Spring coefficient.
        real_type    m_b;   ///< Damping coefficient.
        real_type    m_A0;  ///< Initial area.
        node_type  * m_ni;  ///< Pointer to i'th node.
        node_type  * m_nj;  ///< Pointer to j'th node.
        node_type  * m_nk;  ///< Pointer to k'th node.

      public:

        void initialize(node_type & ni, node_type & nj, node_type & nk)
        {
          using std::sqrt;

          m_ni = & ni;
          m_nj = & nj;
          m_nk = & nk;
          vector3_type A = cross( (m_nj->m_coord - m_ni->m_coord) , (m_nk->m_coord - m_ni->m_coord) );
          m_A0 = value_traits::half()*sqrt(A*A);
        }

        void apply() const
        {
          using std::sqrt;

          vector3_type e_jk = (m_nj->m_coord - m_nk->m_coord);
          vector3_type e_ki = (m_nk->m_coord - m_ni->m_coord);
          vector3_type e_ij = (m_ni->m_coord - m_nj->m_coord);

          vector3_type A =  cross(e_ki , e_ij);
          real_type A_norm = sqrt(A*A);
          real_type C  =  ( (value_traits::half()*A_norm - m_A0) / m_A0 );
          real_type nominator = value_traits::one() / (value_traits::two()*m_A0 * A_norm);
          vector3_type dCdi = cross( e_jk , A )*nominator;
          vector3_type dCdj = cross( e_ki , A )*nominator;
          vector3_type dCdk = cross( e_ij , A )*nominator;

          real_type factor = value_traits::zero();

          if(m_k>value_traits::zero())
            factor -= m_k *C;

          if(m_b>value_traits::zero())
            factor -= m_b * (dCdi*m_ni->m_v + dCdj*m_nj->m_v + dCdk*m_nk->m_v);

          if(factor)
          {
            m_ni->m_f_con += factor*dCdi;
            m_nj->m_f_con += factor*dCdj;
            m_nk->m_f_con += factor*dCdk;
          }
        }

        real_type compute_internal_energy()
        {
          vector3_type e_ki = (m_nk->m_coord - m_ni->m_coord);
          vector3_type e_ij = (m_ni->m_coord - m_nj->m_coord);
          vector3_type A =  cross(e_ki , e_ij);
          real_type A_norm = sqrt(A*A);
          real_type C  =  ( (value_traits::half()*A_norm-m_A0) / m_A0 );
          return value_traits::half()*m_k*C*C;
        }
      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_AREA_CONSTRAINT_H
#endif
