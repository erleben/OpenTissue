#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_VOLUME_CONSTRAINT_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_VOLUME_CONSTRAINT_H
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
      class VolumeConstraint
      {
      public:

        typedef typename versatile_types::value_traits  value_traits;
        typedef typename versatile_types::real_type     real_type;
        typedef typename versatile_types::vector3_type  vector3_type;
        typedef typename versatile_types::node_type     node_type;

      public:

        real_type    m_k;   ///< Spring coefficient.
        real_type    m_b;   ///< Damping coefficient.
        real_type    m_V0;  ///< Initial volume.
        node_type  * m_ni;  ///< Pointer to i'th node.
        node_type  * m_nj;  ///< Pointer to j'th node.
        node_type  * m_nk;  ///< Pointer to k'th node.
        node_type  * m_nm;  ///< Pointer to m'th node.

      public:

        void initialize(node_type & ni,node_type & nj,node_type & nk,node_type & nm)
        {
          real_type const six = boost::numeric_cast<real_type>(6.0);

          m_ni = & ni;
          m_nj = & nj;
          m_nk = & nk;
          m_nm = & nm;

          vector3_type e_ki = (m_nk->m_coord - m_ni->m_coord);
          vector3_type e_mi = (m_nm->m_coord - m_ni->m_coord);
          vector3_type e_ji = (m_nj->m_coord - m_ni->m_coord);

          m_V0 = e_mi * cross(e_ji , e_ki) / six;
        }

        void apply()const
        {
          real_type const six = boost::numeric_cast<real_type>(6.0);

          vector3_type e_mj = (m_nm->m_coord - m_nj->m_coord);
          vector3_type e_kj = (m_nk->m_coord - m_nj->m_coord);
          vector3_type e_ki = (m_nk->m_coord - m_ni->m_coord);
          vector3_type e_mi = (m_nm->m_coord - m_ni->m_coord);
          vector3_type e_ji = (m_nj->m_coord - m_ni->m_coord);

          real_type V = e_mi * cross(e_ji , e_ki) / six;
          real_type C = (V-m_V0) / m_V0;
          real_type nominator = value_traits::one() / six*m_V0;
          vector3_type dCdi = cross(e_mj , e_kj) * nominator;
          vector3_type dCdj = cross(e_ki , e_mi) * nominator;
          vector3_type dCdk = cross(e_mi , e_ji) * nominator;
          vector3_type dCdm = cross(e_ji , e_ki) * nominator;
          real_type factor = value_traits::zero();
          if(m_k>value_traits::zero())
            factor -= m_k *C;
          if(m_b>value_traits::zero())
            factor -= m_b * (dCdi*m_ni->m_v + dCdj*m_nj->m_v + dCdk*m_nk->m_v+ dCdm*m_nm->m_v);
          if(factor)
          {
            m_ni->m_f_con += factor*dCdi;
            m_nj->m_f_con += factor*dCdj;
            m_nk->m_f_con += factor*dCdk;
            m_nm->m_f_con += factor*dCdm;
          }
        }

        real_type compute_internal_energy()
        {
          real_type const six = boost::numeric_cast<real_type>(6.0);

          vector3_type e_ki = (m_nk->m_coord - m_ni->m_coord);
          vector3_type e_mi = (m_nm->m_coord - m_ni->m_coord);
          vector3_type e_ji = (m_nj->m_coord - m_ni->m_coord);
          real_type V = e_mi * cross(e_ji , e_ki) / six;
          real_type C = (V-m_V0) / m_V0;
          return value_traits::half()*m_k*C*C;
        }
      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_VOLUME_CONSTRAINT_H
#endif
