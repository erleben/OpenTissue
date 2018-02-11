#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_Z_SLICER_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_Z_SLICER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>

#include <iostream>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * OBB Z Slicer.
    * Vertex numbers:
    *
    *                         v4 +---------------+ v7
    *                           /|              /|
    *                          / |             / |
    *                         /  |            /  |
    *                        /   |           /   |
    *                       /    |          /    |
    *                      /     |         /     |
    *                  v5 +---------------+ v6   |
    *                     |      |        |      |
    *                     |      |        |      |
    *                     |   v0 +--------|------+ v3
    *                     |     /         |     /
    *                     |    /          |    / 
    *                     |   /           |   /  
    *                     |  /            |  /     
    *                     | /             | /
    *                     |/              |/
    *                  v1 +---------------+v2
    *
    *
    * @param vertices  Vertices of an OBB box.
    * @param z         The z-value of the z-plane.
    * @param slice     Upon return this array holds the vertices of the intersecting
    *                  polygonal slice.
    *
    * @return          The number of vertices in the polygonal intersection.
    */
    template <typename real_type, typename vector3_type>
    unsigned int obb_z_slice(vector3_type const vertices[8], real_type z, vector3_type slice[8])
    {        
      static real_type threshold = math::working_precision<real_type>();
      //  Vertex Numbers and Edge Lables
      //  
      //                                Front-side
      //
      //                         v4 +-------J-------+ v7
      //                           /|              /|
      //                          / |             / |
      //                         /  |            /  |
      //                        C   |           F   |
      //                       /    D          /    |
      //                      /     |         /     E                  z
      // Bottom-side      v5 +-------K-------+ v6   |   Top-side       ^     
      //                     |      |        |      |                  |
      //                     |      |        |      |                  |
      //                     |   v0 +-----I--|------+ v3               *---> y
      //                     |     /         |     /                  /
      //                     B    /          G    /                 |/_
      //                     |   A           |   H                 x
      //                     |  /            |  /     
      //                     | /             | /
      //                     |/              |/
      //                  v1 +------L--------+v2
      //
      //                             Back-side

      //--- Fill edges. Holds indices to end vertices.
      static int edges[12][2] = 
      {
        {0,1},  //--- edge A
        {1,5},  //--- edge B
        {4,5},  //--- edge C
        {0,4},  //--- edge D
        {3,7},  //--- edge E
        {6,7},  //--- edge F
        {2,6},  //--- edge G
        {2,3},  //--- edge H
        {0,3},  //--- edge I
        {4,7},  //--- edge J
        {5,6},  //--- edge K
        {1,2}   //--- edge L
      };
      //--- Fill edge flags. 
      //--- Every edge has a bit-patteren. Indicating the neighboring faces.
      //---
      //--- bit-pattern    ( unused, unused, top, bottom, left, right, front, back)
      //---
      //--- For instance edge D have bottom and left faces as neighbors, therefore 
      //---
      //---    edge_flags[D] = 00011000 = 0x18
      //---
      static int edge_flags[12] = 
      {
        0x11, // 0001 0001
        0x14, // 0001 0100
        0x12, // 0001 0010
        0x18, // 0001 1000
        0x28, // 0010 1000
        0x22, // 0010 0010
        0x24, // 0010 0100
        0x21, // 0010 0001
        0x09, // 0000 1001
        0x0A, // 0000 1010
        0x06, // 0000 0110
        0x05  // 0000 0101
      };

      unsigned int slice_size = 0;   //--- Holds the total number of edges that slices the z-plane
      int idx[ 12 ];                 //--- Keeps indices of edges sliced by the z-plane

      //--- Run through all edges
      real_type zp = z+threshold;
      real_type zm = z-threshold;

      for ( unsigned int i = 0; i < 12; ++i )
      {
        real_type z0 = vertices[ edges[ i ][ 0 ] ][ 2 ];
        real_type z1 = vertices[ edges[ i ][ 1 ] ][ 2 ];
        //--- Only slice if depth is between the two endpoints

        if (  
          ( ( z0 <= zp ) && ( zm <= z1  ) )
          || 
          ( ( z1 <= zp ) && ( zm <= z0 )  )
          )
          idx[ slice_size++ ] = i;
      }

      if(slice_size==0)
        return 0;

      //--- Sort intersections to form an order that yields a simple polygon
      //---
      //--- We wanted edges sorted in a list such that edges at position i and
      //--- position i+1 share a face
      //---
      //--- This is done using a insertion-sort like algorithm.
      //---
      if ( slice_size > 3 )
      {
        for ( unsigned int i = 0; i < (slice_size - 1); ++i )
        {
          for ( unsigned int j = i + 1; j <= (slice_size - 1); ++j )
          {
            //--- Edge i and j share a face
            if ( edge_flags[ idx[ i ] ] & edge_flags[ idx[ j ] ] ) 
            {
              //--- if j is too far to the right swap with edge right next to edge i
              if ( j > i + 1 )
              {
                // swap
                unsigned int tmp = idx[ i + 1 ];
                idx[ i + 1 ]     = idx[ j ];
                idx[ j ]         = tmp;
              }
              break;
            }
          }
        }
      }
      //--- Now we know the correct order of the intersection points, we
      //--- just need to compute the actual coordinates of the intersection
      //--- points
      for ( unsigned int i = 0; i < slice_size; ++i )
      {
        vector3_type const & p0 = vertices[ edges[ idx[ i ] ][ 0 ] ];
        vector3_type const & p1 = vertices[ edges[ idx[ i ] ][ 1 ] ];
        real_type  t = ( z - p0[ 2 ] ) / ( p1[ 2 ] - p0[ 2 ] );
        slice[ i ] = ((p1-p0)*t) + p0;
      }
      return slice_size;
    }

  } // namespace geometry
} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_Z_SLICER_H
#endif
