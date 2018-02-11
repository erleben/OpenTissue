#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_EDGE_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_EDGE_CIRCULATOR_H
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
    * Face Edge Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshFaceEdgeCirculator<PolyMesh,vertex_type> face_edge_circualtor;
    *  typedef PolyMeshFaceEdgeCirculator<PolyMesh,vertex_type const> const_face_edge_circualtor;
    *
    *  face_edge_circualtor circ( &(*f_iter) );
    *  face_edge_circualtor end(  );
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshFaceEdgeCirculator
    {
    protected:

      typename PolyMesh::face_halfedge_circulator m_circ;

    public:

      PolyMeshFaceEdgeCirculator()
        : m_circ()
      {}

      explicit PolyMeshFaceEdgeCirculator(  typename PolyMesh::face_type const & f)
        : m_circ(f)
      {}

      template <class OtherValue>
      PolyMeshFaceEdgeCirculator(
        PolyMeshFaceEdgeCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshFaceEdgeCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshFaceEdgeCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshFaceEdgeCirculator & operator++()
      {
        ++m_circ;
        return *this;
      }

      PolyMeshFaceEdgeCirculator & operator--()
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

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_EDGE_CIRCULATOR_H
#endif
