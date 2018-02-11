#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_VERTEX_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_VERTEX_CIRCULATOR_H
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
    * Vertex Vertex Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshVertexVertexCirculator<PolyMesh,vertex_type> vertex_vertex_circualtor;
    *  typedef PolyMeshVertexVertexCirculator<PolyMesh,vertex_type const> const_vertex_vertex_circualtor;
    *
    *  vertex_vertex_circualtor circ( &(*v_iter) );
    *  vertex_vertex_circualtor end(  );
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshVertexVertexCirculator
    {
    protected:

      typename PolyMesh::vertex_halfedge_circulator m_circ;

    public:

      PolyMeshVertexVertexCirculator()
        : m_circ()
      {}

      explicit PolyMeshVertexVertexCirculator(  typename PolyMesh::vertex_type const & v)
        : m_circ(v)
      {}

      template <class OtherValue>
      PolyMeshVertexVertexCirculator(
        PolyMeshVertexVertexCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshVertexVertexCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshVertexVertexCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshVertexVertexCirculator & operator++()
      {
        ++m_circ;
        return *this;
      }

      PolyMeshVertexVertexCirculator & operator--()
      {
        --m_circ;
        return *this;
      }

    public:

      Value & operator*() const { return *(m_circ->get_destination_iterator()); }

      Value * operator->() const { return &(*(m_circ->get_destination_iterator())); }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_VERTEX_CIRCULATOR_H
#endif
