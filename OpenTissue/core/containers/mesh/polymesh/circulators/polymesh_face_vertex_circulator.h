#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_VERTEX_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_VERTEX_CIRCULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/circulators/polymesh_face_halfedge_circulator.h>

namespace OpenTissue
{
  namespace polymesh
  {
    /**
    * Face Vertex Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshFaceVertexCirculator<PolyMesh,vertex_type> face_vertex_circualtor;
    *  typedef PolyMeshFaceVertexCirculator<PolyMesh,vertex_type const> const_face_vertex_circualtor;
    *
    *  face_vertex_circualtor circ( &(*f_iter) );
    *  face_vertex_circualtor end;
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshFaceVertexCirculator
    {
    protected:

      typename PolyMesh::face_halfedge_circulator m_circ;

    public:

      PolyMeshFaceVertexCirculator()
        : m_circ()
      {}

      explicit PolyMeshFaceVertexCirculator(  typename PolyMesh::face_type const & f)
        : m_circ(f)
      {}

      template <class OtherValue>
      PolyMeshFaceVertexCirculator(
        PolyMeshFaceVertexCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshFaceVertexCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshFaceVertexCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshFaceVertexCirculator & operator++()
      {
        ++m_circ;
        return *this;
      }

      PolyMeshFaceVertexCirculator & operator--()
      {
        --m_circ;
        return *this;
      }

    public:

      Value & operator*() const { return *(m_circ->get_origin_iterator()); }

      Value * operator->() const { return &(*(m_circ->get_origin_iterator())); }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_VERTEX_CIRCULATOR_H
#endif
