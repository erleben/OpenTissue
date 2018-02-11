#ifndef OPENTISSUE_COLLISION_GJK_GJK_REDUCE_TETRAHEDRON_H
#define OPENTISSUE_COLLISION_GJK_GJK_REDUCE_TETRAHEDRON_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_barycentric.h>

#include <OpenTissue/collision/gjk/gjk_simplex.h>
#include <OpenTissue/collision/gjk/gjk_outside_triangle.h>
#include <OpenTissue/collision/gjk/gjk_outside_vertex_edge_voronoi_plane.h>
#include <OpenTissue/collision/gjk/gjk_outside_edge_face_voronoi_plane.h>

#include <cmath>

namespace OpenTissue
{
  namespace gjk
  {

    namespace detail
    {
      /**
      * Reduce Tetrahedron.
      * This function implements the case where a simplex
      * is a tetrahedron. The function will compute the closest
      * point to p on the simplex and try to reduce the simplex
      * to the lowest dimensional face on the boundary of the
      * simplex containing the closest point.
      *
      * @param p   The test point.
      * @param S   Initially this argument holds the tetrahedron
      *            simplex. Upon return the argument holds the reduced
      *            simplex.
      */
      template< typename V >
      inline void reduce_tetrahedron( V const & p, Simplex<V> & S)
      {
        typedef typename V::value_type    T;
        typedef typename V::value_traits  value_traits;

        size_t const idx_A = 0u;
        size_t const idx_B = 1u;
        size_t const idx_C = 2u;
        size_t const idx_D = 3u;

        int const bit_A = 1;
        int const bit_B = 2;
        int const bit_C = 4;
        int const bit_D = 8;

        V const & A = S.m_v[idx_A];
        V const & B = S.m_v[idx_B];
        V const & C = S.m_v[idx_C];
        V const & D = S.m_v[idx_D];

        bool const outside_AB     = outside_vertex_edge_voronoi_plane(p, A, B);
        bool const outside_AC     = outside_vertex_edge_voronoi_plane(p, A, C);
        bool const outside_AD     = outside_vertex_edge_voronoi_plane(p, A, D);
        bool const outside_BA     = outside_vertex_edge_voronoi_plane(p, B, A);
        bool const outside_BC     = outside_vertex_edge_voronoi_plane(p, B, C);
        bool const outside_BD     = outside_vertex_edge_voronoi_plane(p, B, D);
        bool const outside_CA     = outside_vertex_edge_voronoi_plane(p, C, A);
        bool const outside_CB     = outside_vertex_edge_voronoi_plane(p, C, B);
        bool const outside_CD     = outside_vertex_edge_voronoi_plane(p, C, D);
        bool const outside_DA     = outside_vertex_edge_voronoi_plane(p, D, A);
        bool const outside_DB     = outside_vertex_edge_voronoi_plane(p, D, B);
        bool const outside_DC     = outside_vertex_edge_voronoi_plane(p, D, C);
        bool const outside_ABC_vp = outside_edge_face_voronoi_plane( p, A, B, C);
        bool const outside_ABD_vp = outside_edge_face_voronoi_plane( p, A, B, D);
        bool const outside_CAB_vp = outside_edge_face_voronoi_plane( p, C, A, B);
        bool const outside_CAD_vp = outside_edge_face_voronoi_plane( p, C, A, D);
        bool const outside_DAB_vp = outside_edge_face_voronoi_plane( p, D, A, B);
        bool const outside_ADC_vp = outside_edge_face_voronoi_plane( p, A, D, C);
        bool const outside_BCA_vp = outside_edge_face_voronoi_plane( p, B, C, A);
        bool const outside_BCD_vp = outside_edge_face_voronoi_plane( p, B, C, D);
        bool const outside_BDA_vp = outside_edge_face_voronoi_plane( p, B, D, A);
        bool const outside_DBC_vp = outside_edge_face_voronoi_plane( p, D, B, C);
        bool const outside_CDB_vp = outside_edge_face_voronoi_plane( p, C, D, B);
        bool const outside_DCA_vp = outside_edge_face_voronoi_plane( p, D, C, A);
        bool const outside_ABC    = outside_triangle( p, A, B, C, D);
        bool const outside_ABD    = outside_triangle( p, A, B, D, C);
        bool const outside_BCD    = outside_triangle( p, B, C, D, A);
        bool const outside_CAD    = outside_triangle( p, C, A, D, B);

        // Test Vertex Voronoi Regions
        if( outside_AB && outside_AC && outside_AD)
        {
          S.m_bitmask = bit_A;
          S.m_v[idx_B].clear();
          S.m_v[idx_C].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_B].clear();
          S.m_a[idx_C].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_B].clear();
          S.m_b[idx_C].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_A] = value_traits::one();
          S.m_w[idx_B] = value_traits::zero();
          S.m_w[idx_C] = value_traits::zero();
          S.m_w[idx_D] = value_traits::zero();    
          return;
        }
        if( outside_BA && outside_BC && outside_BD)
        {
          S.m_bitmask = bit_B;
          S.m_v[idx_A].clear();
          S.m_v[idx_C].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_C].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_C].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_B] = value_traits::one();
          S.m_w[idx_C] = value_traits::zero();
          S.m_w[idx_D] = value_traits::zero();    
          return;
        }
        if( outside_CA && outside_CB && outside_CD)
        {
          S.m_bitmask = bit_C;
          S.m_v[idx_A].clear();
          S.m_v[idx_B].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_B].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_B].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_B] = value_traits::zero();
          S.m_w[idx_C] = value_traits::one();
          S.m_w[idx_D] = value_traits::zero();    
          return;
        }
        if( outside_DA && outside_DB && outside_DC)
        {
          S.m_bitmask = bit_D;
          S.m_v[idx_A].clear();
          S.m_v[idx_B].clear();
          S.m_v[idx_C].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_B].clear();
          S.m_a[idx_C].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_B].clear();
          S.m_b[idx_C].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_B] = value_traits::zero();
          S.m_w[idx_C] = value_traits::zero();
          S.m_w[idx_D] = value_traits::one();
          return;
        }

        // Test Edge Voronoi Regions
        if(outside_ABC_vp && outside_ABD_vp && !outside_AB && !outside_BA)
        {
          S.m_bitmask = bit_A | bit_B;
          S.m_v[idx_C].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_C].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_C].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_C] = value_traits::zero();
          S.m_w[idx_D] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(A,B,p,S.m_w[idx_A],S.m_w[idx_B]);
          return;
        }
        if(outside_CAB_vp && outside_CAD_vp && !outside_CA && !outside_AC)
        {
          S.m_bitmask = bit_A | bit_C;
          S.m_v[idx_B].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_B].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_B].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_B] = value_traits::zero();
          S.m_w[idx_D] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(A,C,p,S.m_w[idx_A],S.m_w[idx_C]);
          return;
        }
        if(outside_DAB_vp && outside_ADC_vp && !outside_AD && !outside_DA)
        {
          S.m_bitmask = bit_A | bit_D;
          S.m_v[idx_B].clear();
          S.m_v[idx_C].clear();
          S.m_a[idx_B].clear();
          S.m_a[idx_C].clear();
          S.m_b[idx_B].clear();
          S.m_b[idx_C].clear();
          S.m_w[idx_B] = value_traits::zero();
          S.m_w[idx_C] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(A,D,p,S.m_w[idx_A],S.m_w[idx_D]);
          return;
        }
        if(outside_BCA_vp && outside_BCD_vp && !outside_BC && !outside_CB)
        {
          S.m_bitmask = bit_B | bit_C;
          S.m_v[idx_A].clear();
          S.m_v[idx_D].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_D] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(B,C,p,S.m_w[idx_B],S.m_w[idx_C]);
          return;
        }
        if(outside_BDA_vp && outside_DBC_vp && !outside_BD && !outside_DB)
        {
          S.m_bitmask = bit_B | bit_D;
          S.m_v[idx_A].clear();
          S.m_v[idx_C].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_C].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_C].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_C] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(B,D,p,S.m_w[idx_B],S.m_w[idx_D]);
          return;
        }
        if(outside_CDB_vp && outside_DCA_vp && !outside_CD && !outside_DC)
        {
          S.m_bitmask = bit_C | bit_D;
          S.m_v[idx_A].clear();
          S.m_v[idx_B].clear();
          S.m_a[idx_A].clear();
          S.m_a[idx_B].clear();
          S.m_b[idx_A].clear();
          S.m_b[idx_B].clear();
          S.m_w[idx_A] = value_traits::zero();
          S.m_w[idx_B] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric(C,D,p,S.m_w[idx_C],S.m_w[idx_D]);
          return;
        }

        // Test Face Voronoi Regions
        if (outside_ABC && !outside_ABC_vp && !outside_BCA_vp && !outside_CAB_vp)
        {
          S.m_bitmask = bit_A | bit_B | bit_C;
          S.m_v[idx_D].clear();
          S.m_a[idx_D].clear();
          S.m_b[idx_D].clear();
          S.m_w[idx_D]  = value_traits::zero();          
          OpenTissue::geometry::barycentric_geometric( A, B, C, p, S.m_w[idx_A], S.m_w[idx_B], S.m_w[idx_C]);                    
          return;
        }
        if (outside_ABD && !outside_ABD_vp && !outside_BDA_vp && !outside_DAB_vp)
        {
          S.m_bitmask = bit_A | bit_B | bit_D;
          S.m_v[idx_C].clear();
          S.m_a[idx_C].clear();
          S.m_b[idx_C].clear();
          S.m_w[idx_C]  = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric( A, B, D, p, S.m_w[idx_A], S.m_w[idx_B], S.m_w[idx_D]);
          return;
        }
        if (outside_BCD && !outside_BCD_vp && !outside_CDB_vp && !outside_DBC_vp)
        {
          S.m_bitmask = bit_B | bit_C | bit_D;
          S.m_v[idx_A].clear();
          S.m_a[idx_A].clear();
          S.m_b[idx_A].clear();
          S.m_w[idx_A] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric( B, C, D, p, S.m_w[idx_B], S.m_w[idx_C], S.m_w[idx_D]);
          return;
        }
        if (outside_CAD && !outside_CAD_vp && !outside_ADC_vp && !outside_DCA_vp)
        {
          S.m_bitmask = bit_C | bit_A | bit_D;
          S.m_v[idx_B].clear();
          S.m_a[idx_B].clear();
          S.m_b[idx_B].clear();
          S.m_w[idx_B] = value_traits::zero();
          OpenTissue::geometry::barycentric_geometric( C, A, D, p, S.m_w[idx_C], S.m_w[idx_A], S.m_w[idx_D]);
          return;
        }

        // Test tetrahedron internal region
        if( !outside_ABC  && !outside_ABD && !outside_BCD && !outside_CAD)
        {
          OpenTissue::geometry::barycentric_geometric( A, B, C, D, p, S.m_w[idx_A], S.m_w[idx_B], S.m_w[idx_C], S.m_w[idx_D]);
          return;
        }
      }


    } // namespace detail

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_REDUCE_TETRAHEDRON_H
#endif
