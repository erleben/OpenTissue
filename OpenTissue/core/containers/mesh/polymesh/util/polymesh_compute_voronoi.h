#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VORONOI_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VORONOI_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_delaunay2D.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_dual.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_valency.h>

#include <cmath> // needed for std::fabs
#include <stdexcept> // needed for std::logic_error


namespace OpenTissue
{
  namespace polymesh
  {


    /**
    * A vertex policy for computing the coordinates of vertices in a
    * Voroni diagram given a corresponding face from the Delaunay
    * triangulation.
    */
    struct voronoi_vertex_policy
    {

      template<typename face_type, typename vector3_type>
      static void compute_position( face_type const & face, vector3_type & coord )
      {
        using std::fabs;

        typedef typename face_type::mesh_type        mesh_type;
        typedef typename mesh_type::math_types       math_types;
        typedef typename math_types::vector3_type    vector3_type;
        typedef typename math_types::real_type  real_type;
        typedef typename math_types::value_traits    value_traits;

        typedef typename mesh_type::const_face_vertex_circulator circulator;

        if(valency( face ) != 3 )
          throw std::logic_error( "voronoi_vertex_policy: only triangular faces are allowed" );

        // extract corners coordinates of triangle
        circulator v(face);
        vector3_type x0 = v->m_coord;
        ++v;
        vector3_type x1 = v->m_coord;
        ++v;
        vector3_type x2 = v->m_coord;

        // compute two edge direction vectors
        vector3_type e10 = x1 - x0;
        vector3_type e21 = x2 - x1;

        // compute two points, each lying on each line
        vector3_type  p0 = (x1 + x0) / value_traits::two();
        vector3_type  p1 = (x2 + x1) / value_traits::two();

        // get direction vectors for the two lines
        vector3_type  d0(  - e10(1), e10(0), value_traits::zero() );
        vector3_type  d1(  - e21(1), e21(0), value_traits::zero() );

        if( fabs(d0(0)) > value_traits::zero() && fabs(d1(0)) > value_traits::zero())
        {
          // compute the slope of the two lines
          real_type a0 = d0(1)/d0(0);
          real_type a1 = d1(1)/d1(0);

          // compute the intersection between the two lines and the y-axis
          real_type b0 = p0(1) - a0*p0(0);
          real_type b1 = p1(1) - a1*p1(0);

          // compute the common intersection point between the two lines
          real_type x = (b1-b0)/(a0-a1);
          real_type y = a0*x+b0;

          coord(0) = x;
          coord(1) = y;
          coord(2) = value_traits::zero();
        }

        else if( fabs(d0(0)) > value_traits::zero())
        {
          // line 1 is vertical

          // compute the slope of line 0
          real_type a0 = d0(1)/d0(0);

          // compute the intersection between line 0 and the y-axis
          real_type b0 = p0(1) - a0*p0(0);

          // compute the common intersection point between the two lines
          real_type x = p1(0);
          real_type y = a0*x+b0;

          coord(0) = x;
          coord(1) = y;
          coord(2) = value_traits::zero();
        }

        else if( fabs(d1(0)) > value_traits::zero())
        {
          // line 0 is vertical

          // compute the slope of line 1
          real_type a1 = d1(1)/d1(0);

          // compute the intersection between line 1 and the y-axis
          real_type b1 = p1(1) - a1*p1(0);

          // compute the common intersection point between the two lines
          real_type x = p0(0);
          real_type y = a1*x+b1;

          coord(0) = x;
          coord(1) = y;
          coord(2) = value_traits::zero();

        }
        else
        {
          throw std::logic_error("voronoi_vertex_policy(): No common intersection point exist, giving up!");
        }
      }

      template<typename face_type, typename vector3_type>
      static void compute_normal( face_type const & face, vector3_type & normal )
      {
        normal.clear();
      }

    };






    /**
    * PolyMesh Compute Voronoi Diagram.
    *
    *
    * @param sites     The Voronoi cell sites.
    * @param mesh      The resulting mesh. Each face corresponds to a Voronoi
    *                  cell of the specified sites. Infinite Voronoi cells are
    *                  skipped. Therefor multiple site-points can be found in
    *                  the outside (''world'') cell.
    * @param delaunay  The corresponding delaunay triangulation.
    */
    template< typename point_container, typename mesh_type >
    void compute_voronoi(point_container const & sites, mesh_type & mesh, mesh_type & delaunay)
    {
      mesh.clear();
      compute_delaunay2D( sites, delaunay );      
      compute_dual(delaunay, mesh, voronoi_vertex_policy() );
    }


    /**
    * PolyMesh Compute Voronoi Diagram.
    *
    *
    * @param sites     The Voronoi cell sites.
    * @param mesh      The resulting mesh. Each face corresponds to a Voronoi
    *                  cell of the specified sites. Infinite Voronoi cells are
    *                  skipped. Therefor multiple site-points can be found in
    *                  the outside (''world'') cell.
    */
    template< typename point_container, typename mesh_type >
    void compute_voronoi(point_container const & sites, mesh_type & mesh)
    {
      mesh_type delaunay;
      compute_voronoi( sites, mesh, delaunay );
    }

  } // namespace polymesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_VORONOI_H
#endif
