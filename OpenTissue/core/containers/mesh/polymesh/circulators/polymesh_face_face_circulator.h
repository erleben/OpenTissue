#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_FACE_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_FACE_CIRCULATOR_H
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
    * Face Face Circulator.
    *
    * Example Usage:
    *
    *  typedef PolyMeshFaceFaceCirculator<PolyMesh,face> face_face_circualtor;
    *  typedef PolyMeshFaceFaceCirculator<PolyMesh,face const> const_face_face_circualtor;
    *
    *  face_face_circualtor circ( &(*f_iter) );
    *  face_face_circualtor end(  );
    *  std::for_each(circ,end, ... );
    *
    * Similar for the const iterator.
    */
    template<
      typename PolyMesh,
    class Value
    >
    class PolyMeshFaceFaceCirculator
    {
    protected:

      typename PolyMesh::face_halfedge_circulator m_circ;

    public:

      PolyMeshFaceFaceCirculator()
        : m_circ()
      {}

      explicit PolyMeshFaceFaceCirculator(  typename PolyMesh::face_type const & f)
        : m_circ(f)
      {}

      template <class OtherValue>
      PolyMeshFaceFaceCirculator(
        PolyMeshFaceFaceCirculator<PolyMesh,OtherValue> const& other
        )
        : m_circ( other.m_circ )
      {}

    public:

      template <class OtherValue>
      bool operator==(PolyMeshFaceFaceCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ == other.m_circ);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshFaceFaceCirculator<PolyMesh,OtherValue> const& other) const
      {
        return (m_circ != other.m_circ);
      }

      PolyMeshFaceFaceCirculator & operator++()
      {
        ++m_circ;
        return *this;
      }

      PolyMeshFaceFaceCirculator & operator--()
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

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_FACE_FACE_CIRCULATOR_H
#endif
