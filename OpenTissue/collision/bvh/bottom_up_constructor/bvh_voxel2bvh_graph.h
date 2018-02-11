#ifndef OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOXEL2BVH_GRAPH_H
#define OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOXEL2BVH_GRAPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/math/math_vector3.h>
#include <vector>
#include <iostream>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Voxel Map to BVH graph converter.
    */
    template<typename voxels_type,typename graph_type>
    class Voxel2BVHGraph
    {
    public:

      //--- Convenience Stuff
      typedef typename graph_type::node_ptr_type   node_ptr_type;
      typedef typename graph_type::edge_ptr_type   edge_ptr_type;
      typedef typename graph_type::volume_type     volume_type;
      typedef typename voxels_type::value_type     voxel_type;
      typedef typename voxels_type::index_iterator index_iterator;

      typedef typename volume_type::math_types     math_types;
      typedef typename math_types::vector3_type    vector3_type;
      typedef          geometry::AABB<math_types>  aabb_type;
    public:


      void run(voxels_type & voxels, graph_type & graph)
      {
        typedef typename voxels_type::math_types         math_types;
        typedef OpenTissue::grid::Grid<node_ptr_type,math_types >           lookup_grid_type;

        node_ptr_type unused_value;
        lookup_grid_type lookup( unused_value );

        lookup.create(voxels.min_coord(),voxels.max_coord(),voxels.I(),voxels.J(),voxels.K());

        vector3_type ext = vector3_type(voxels.dx() /2., voxels.dy() /2., voxels.dz() /2.);

        int cnt = 0;
        for( index_iterator iter = voxels.begin(); iter != voxels.end(); ++iter)
        {
          if( (*iter)==1 )
          {
            aabb_type aabb;
            vector3_type p;
            OpenTissue::grid::idx2coord(iter,p);
            vector3_type pmin = p - ext;
            vector3_type pmax = p + ext;
            aabb.set(pmin,pmax);
            node_ptr_type node = graph.insert(aabb);
            lookup( iter.get_index() ) = node;
            cnt++;
          }
        }

        std::cout << "Voxel2BVHGraph::run():  " << cnt << " AABB nodes in graph" << std::endl;
        cnt = 0;
        for(unsigned int k=0;k<(voxels.K()-1);++k)
          for(unsigned int j=0;j<(voxels.J()-1);++j)
            for(unsigned int i=0;i<(voxels.I()-1);++i)
            {
              node_ptr_type node = lookup(i,j,k);
              if(!node)
                continue;
              node_ptr_type node_I = lookup(i+1,j,k);
              node_ptr_type node_J = lookup(i,j+1,k);
              node_ptr_type node_K = lookup(i,j,k+1);
              if(node_I)
              {
                graph.insert(node,node_I);
                cnt++;
              }
              if(node_J)
              {
                graph.insert(node,node_J);
                cnt++;
              }
              if(node_K)
              {
                graph.insert(node,node_K);
                cnt++;
              }
            }
            std::cout << "Voxel2BVHGraph::run(): Created " << cnt << " edges in graph" << std::endl;
      }

    };


  } // namespace bvh

} // namespace OpenTissue

//OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOXEL2BVH_GRAPH_H
#endif
