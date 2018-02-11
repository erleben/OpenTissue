#ifndef CURVATURE_MESH_BV_TRAITS_H
#define CURVATURE_MESH_BV_TRAITS_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>

class CurvatureMeshBVTraits
{
public:

  typedef OpenTissue::polymesh::PolyMesh<>   mesh_type;
  typedef mesh_type::vertex_type             vertex_type;
  typedef std::list<vertex_type *>           adjacency_container;
  typedef adjacency_container::iterator      adjacency_iterator;

public:

  unsigned int m_dir_lut;             ///< Lookup table for direction vectors.
  adjacency_container m_adjacency;    ///< List of vertices on the border, should be used to determine adjacency.

};

// CURVATURE_MESH_BV_TRAITS_H
#endif
