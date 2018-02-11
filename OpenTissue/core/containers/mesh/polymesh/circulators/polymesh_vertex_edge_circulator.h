#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_EDGE_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_EDGE_CIRCULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/circulators/polymesh_vertex_halfedge_circulator.h>

namespace OpenTissue
{
  namespace polymesh
  {


    /**
    * Vertex Edge Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshVertexEdgeCirculator<PolyMesh,edge> vertex_edge_circualtor;
    *  typedef PolyMeshVertexEdgeCirculator<PolyMesh,edge const> const_vertex_edge_circualtor;
    *
    *  vertex_edge_circualtor circ( &(*v_iter) );
    *  vertex_edge_circualtor end(  );
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshVertexEdgeCirculator
    {
    protected:

      typename PolyMesh::vertex_halfedge_circulator m_circ;

    public:

      PolyMeshVertexEdgeCirculator()
        : m_circ()
      {}

      explicit PolyMeshVertexEdgeCirculator(  typename PolyMesh::vertex_type const & v)
        : m_circ(v)
      {}

      template <class OtherValue>
      PolyMeshVertexEdgeCirculator(
        PolyMeshVertexEdgeCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshVertexEdgeCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshVertexEdgeCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshVertexEdgeCirculator & operator++()
      {
        ++m_circ;
        return *this;
      }

      PolyMeshVertexEdgeCirculator & operator--()
      {
        --m_circ;
        return *this;
      }

    public:

      Value & operator*() const { return *(m_circ->get_edge_iterator()); }

      Value * operator->() const { return &(*(m_circ->get_edge_iterator())); }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_EDGE_CIRCULATOR_H
#endif
