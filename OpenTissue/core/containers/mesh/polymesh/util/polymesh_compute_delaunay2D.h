#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DELAUNAY2D_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DELAUNAY2D_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <Triangle/triangle.h>
#include <boost/vector_property_map.hpp>
#include <boost/cast.hpp> // needed for boost::numeric_cast
#include <cstring>        // needed for memset

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * PolyMesh Compute 2D Delaunay Triangulation.
    *
    *
    * @param vertices  The resulting vertices of the triangulation.
    * @param mesh      The resulting mesh.
    */
    template< typename point_container, typename mesh_type >
    void compute_delaunay2D(point_container const & vertices, mesh_type & mesh)
    {
      typedef typename mesh_type::math_types        math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename math_types::value_traits     value_traits;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::size_type         size_type;

      mesh.clear();

      if(vertices.empty())
        return;

      triangulateio in, out, vorout;
      memset(&in, 0, sizeof(triangulateio));
      memset(&out, 0, sizeof(triangulateio));
      memset(&vorout, 0, sizeof(triangulateio));

      in.numberofpoints = boost::numeric_cast<int>(vertices.size());

      std::vector<REAL> input_points( in.numberofpoints*2 ); 

      for(size_type i=0;i<vertices.size();++i)
      {
        real_type x = boost::numeric_cast<real_type>( vertices[i](0) );
        real_type y = boost::numeric_cast<real_type>( vertices[i](1) );

        input_points[i*2  ] = boost::numeric_cast<REAL>( x );
        input_points[i*2+1] = boost::numeric_cast<REAL>( y );
      }

      in.pointlist = &input_points[0];

      //char switches[] = "zvQ"; // z = zero index, v = voronoi, Q = quiet!
      char switches[] = "zDQ"; // z = zero index, D = conforming Delaynay, Q = quiet!
      ::triangulate(switches, &in, &out, &vorout);

      boost::vector_property_map<vertex_handle> handles;

      for (int n = 0; n < out.numberofpoints; ++n) 
      {
        vector3_type coord;
        coord[0] = boost::numeric_cast<real_type>( out.pointlist[n*2  ] );
        coord[1] = boost::numeric_cast<real_type>( out.pointlist[n*2+1] );    
        coord[2] = value_traits::zero();
        vertex_handle h = mesh.add_vertex( coord );
        handles[n] = h;
      }

      for (int n = 0; n < out.numberoftriangles; ++n) 
      {
        int idx0 = out.trianglelist[n*3  ];
        int idx1 = out.trianglelist[n*3 +1 ];
        int idx2 = out.trianglelist[n*3 +2 ];
        mesh.add_face( handles[idx0],handles[idx1],handles[idx2] );
      }
      
      // Clean up any memory used by ::triangulate
      free(vorout.pointlist);
      free(vorout.pointattributelist);
      free(vorout.edgelist);
      free(vorout.normlist);
      free(out.pointlist);
      free(out.pointmarkerlist);
      free(out.trianglelist);

    }

  } // namespace polymesh
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DELAUNAY2D_H
#endif
