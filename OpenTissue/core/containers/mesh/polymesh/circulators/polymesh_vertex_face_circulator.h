#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_FACE_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_FACE_CIRCULATOR_H
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
    * Vertex Face Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshVertexFaceCirculator<PolyMesh,face> vertex_face_circualtor;
    *  typedef PolyMeshVertexFaceCirculator<PolyMesh,face const> const_vertex_face_circualtor;
    *
    *  vertex_face_circualtor circ( &(*v_iter) );
    *  vertex_face_circualtor end(  );
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshVertexFaceCirculator
    {
    protected:

      typename PolyMesh::vertex_halfedge_circulator m_circ;

    protected:

      bool is_end() const
      {
        // this works due to m_circ's operator== ignores the rhs argument!
        return m_circ == m_circ;/*.operator==(typename PolyMesh::vertex_halfedge_circulator() )*/;
      }

    public:

      PolyMeshVertexFaceCirculator()
        : m_circ()
      {}

      explicit PolyMeshVertexFaceCirculator(  typename PolyMesh::vertex_type const & v)
        : m_circ(v)
      {
        while(m_circ->get_face_handle().is_null() && 
              !is_end() )
        {
          ++m_circ;
        }
      }

      template <class OtherValue>
      PolyMeshVertexFaceCirculator(
        PolyMeshVertexFaceCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshVertexFaceCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshVertexFaceCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshVertexFaceCirculator & operator++()
      {
        //do {
        //  ++m_circ;
        //}
        //while(m_circ->get_face_handle().is_null()
        //      && !is_end()
        //      );
        ++m_circ;
        return *this;
      }

      PolyMeshVertexFaceCirculator & operator--()
      {
        --m_circ;

        return *this;
      }

    public:

      Value & operator*() const
      {
        assert(!m_circ->get_face_handle().is_null() || !"face was null");
        return *(m_circ->get_face_iterator());
      }

      Value * operator->() const
      {
        assert(!m_circ->get_face_handle().is_null() || !"face was null");
        return &(*(m_circ->get_face_iterator()));
      }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_FACE_CIRCULATOR_H
#endif
