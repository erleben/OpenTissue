#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_PROFILE_SWEEP_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_PROFILE_SWEEP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <boost/multi_array.hpp>

namespace OpenTissue
{
  namespace mesh
  {

    /**
    * The profile sweeps out a mesh by an incremental
    * rotation of the specified number of slices around
    * the z-axe.
    *
    * @param profile_begin
    * @param profile_end
    * @param sweep_angle
    * @param slices
    * @param mesh
    * @return          If succesful the return value is true otherwise it is false;
    */
    template<typename mesh_type,typename vector3_iterator,typename real_type>
    bool profile_sweep(
      vector3_iterator profile_begin
      , vector3_iterator profile_end
      , real_type sweep_angle
      , unsigned int slices
      , mesh_type & mesh
      )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      //typedef typename math_types::real_type                        real_type;
      typedef typename mesh_type::vertex_handle                     vertex_handle;

      assert(slices >= 0 || !"At least one slice must be created");
      assert(sweep_angle > static_cast<real_type>(0.0)|| !"Sweep angle must be positive");
      assert(sweep_angle <= static_cast<real_type>(math::detail::pi<real_type>()*2.0)|| !"Sweep angle must be less than equal 2PI");


      unsigned int i,j,k;
      vector3_iterator p;

      unsigned int K = static_cast<unsigned int>( std::distance(profile_begin,profile_end) );
      assert(K>=2);

      unsigned int I = slices + 1;
      assert(I>=2);

      p = profile_begin;
      real_type max_z_value = (*p)(2);
      real_type min_z_value = (*p)(2);
      for(;p != profile_end;++p)
      {
        max_z_value = std::max(max_z_value, (*p)(2));
        min_z_value = std::min(min_z_value, (*p)(2));
      }
      real_type range_z_value = max_z_value - min_z_value;
      assert(range_z_value > 0 || !"z-value range must be positive");

      //p = profile_begin;
      //(*p)(0) = static_cast<real_type>(0.0);
      //(*p)(1) = static_cast<real_type>(0.0);
      //p = profile_end - 1;
      //(*p)(0) = static_cast<real_type>(0.0);
      //(*p)(1) = static_cast<real_type>(0.0);

      mesh.clear();

      //--- Allocate memory for vertex grid.

      boost::multi_array<vector3_type, 2>  vector3_grid(boost::extents[K][I]);
      //--- Create first slice.
      for(p = profile_begin,k=0; p != profile_end;++p,++k)
      {
        vector3_grid[k][0] = *p;
      }
      //--- Rotate profile around z-axe thereby creating remaining slices.
      real_type delta_angle = static_cast<real_type>(  sweep_angle  / (I - 1));
      real_type angle = delta_angle;
      for(i=1;i<I;++i)
      {
        real_type cosinus = static_cast<real_type>(std::cos(angle));
        real_type sinus = static_cast<real_type>(std::sin(angle));
        for(p = profile_begin,k=0; p != profile_end;++p,++k)
        {
          vector3_grid[k][i](0) = cosinus*(*p)(0) -    sinus*(*p)(1);
          vector3_grid[k][i](1) =   sinus*(*p)(0) +  cosinus*(*p)(1);
          vector3_grid[k][i](2) = (*p)(2);
        }
        angle += delta_angle;
      }

      //--- We now have a vertex coordinate grid, next we generate a vertex handle grid
      real_type tolerance = 10e-5;
      boost::multi_array<vertex_handle, 2>  grid(boost::extents[K][I]);
      for(k=0;k<K;++k)
      {
        for(i=0;i<I;++i)
        {
          vertex_handle  v;
          for(j=0;j<i;++j)
          {
            vector3_type diff = vector3_grid[k][j] - vector3_grid[k][i];
            if(diff*diff < tolerance)
            {
              v = grid[k][j];
              break;
            }
          }
          if(v.is_null())
          {
            v = mesh.add_vertex(vector3_grid[k][i]);
          }
          grid[k][i] = v;
          typename mesh_type::vertex_iterator vit = mesh.get_vertex_iterator(v);
          vit->m_normal = normalize(vit->m_coord);
          vit->m_u = (i*delta_angle)/sweep_angle;
          vit->m_v = (vit->m_coord(2)-min_z_value)/range_z_value;
          vit->m_color  = vector3_type( vit->m_u, 0.0, vit->m_v);
        }
      }
      //--- At this point we have created a vertex grid from which we can build the faces.
      for(i=0;i<I-1;++i)
      {
        for(k=0;k<(K-1);++k)
        {
          vertex_handle  v0 = grid[k+1][i];
          vertex_handle  v1 = grid[k][i];
          vertex_handle  v2 = grid[k][i+1];
          vertex_handle  v3 = grid[k+1][i+1];
          //
          //   i      i+1
          //   |       |
          //
          //   0       3          --------- k+1
          //
          //   1       2          ---------  k
          //
          if(v0!=v3 && v1!=v2)
          {
            vertex_handle tmp[4];
            tmp[0] = v0;
            tmp[1] = v1;
            tmp[2] = v2;
            tmp[3] = v3;
            vertex_handle * handles = &tmp[0];
            mesh.add_face(handles,handles+4);
          }
          else
          {
            if(v0!=v3)
              mesh.add_face(v0,v1,v3);
            if(v1!=v2)
              mesh.add_face(v1,v2,v3);
          }
        }
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_PROFILE_SWEEP_H
#endif
