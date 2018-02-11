#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_PLANE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/multi_array.hpp>

namespace OpenTissue
{
  namespace mesh
  {

    /**
    * Make Plane
    *
    * @return          If succesful the return value is true otherwise it is false;
    */
    template<typename mesh_type,typename real_type>
    bool make_plane(
      real_type dx
      , real_type dy
      , unsigned int X
      , unsigned int Y
      , mesh_type & mesh
      )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;
      typedef typename mesh_type::vertex_handle                     vertex_handle;

      assert(X > 0 || !"At least one subdivision on x-axis");
      assert(Y > 0 || !"At least one subdivision on y-axis");
      assert(dx > static_cast<real_type>(0.0)|| !"x-axis spacing must be positive");
      assert(dy > static_cast<real_type>(0.0)|| !"y-axis spacing must be positive");
      unsigned int I = X + 1;
      assert(I>=2);
      unsigned int J = Y + 1;
      assert(J>=2);
      mesh.clear();
      boost::multi_array<vertex_handle, 2>  grid(boost::extents[I][J]);

      vector3_type coord;
      for(unsigned int j=0;j<J;++j)
        for(unsigned int i=0;i<I;++i)
        {
          coord(0) = static_cast<real_type>(i*dx -  (X*dx/2.0));
          coord(1) = static_cast<real_type>(j*dy -  (Y*dy/2.0));
          coord(2) = static_cast<real_type>(0.0);
          grid[i][j] = mesh.add_vertex(coord);
          typename mesh_type::vertex_iterator v = mesh.get_vertex_iterator(grid[i][j]);
          v->m_normal = vector3_type(0,0,1);
          v->m_u = 1.0*i/X;
          v->m_v = 1.0*j/Y;
          v->m_color = vector3_type( v->m_u, 0.0, v->m_v);
        }
        for(unsigned int j=0;j<Y;++j)
          for(unsigned int i=0;i<X;++i)
          {
            std::list<vertex_handle> handles;

            handles.push_back( grid[  i][  j] );
            handles.push_back( grid[i+1][  j] );
            handles.push_back( grid[i+1][j+1] );
            handles.push_back( grid[  i][j+1] );
            mesh.add_face(handles.begin(),handles.end());
          }
          return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_MAKE_PLANE_H
#endif
