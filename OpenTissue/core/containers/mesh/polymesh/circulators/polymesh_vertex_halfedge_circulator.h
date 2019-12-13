#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_HALFEDGE_CIRCULATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_HALFEDGE_CIRCULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace polymesh
  {

    template< typename PolyMesh, class Value >
    class PolyMeshVertexHalfedgeCirculator
    {
    private:

      typedef typename PolyMesh::opt_halfedge_iter iterator_t;

      typename PolyMesh::kernel_type * m_kernel;
      iterator_t              m_first;
      iterator_t              m_cur;
      bool                             m_active;

    private:

      /**
      * Helper function to be used in the constructor initializer list (that is why it is static).
      * Ensures that if the mesh is not yet fully constructed, then the
      * member iterators are set to a default constructed op_halfedge_iter
      * (corresponding to boost::optional( boost::none ) ).
      * This further ensures that operator !=() will return false when the
      * mesh is not fully constructed, such that no circulation is performed
      * in a standard for(;it!=end;++it)-loop.
      *
      * @param v  The vertex to create a circulator for.
      * @return   An outgoing_halfedge_iterator if the corresponding handle is valid, or an optional.
      */
      static iterator_t init(typename PolyMesh::vertex_type const & v)
      {
        if ( v.get_outgoing_halfedge_handle().is_null() )
          return iterator_t();
        else
          return v.get_outgoing_halfedge_iterator();
      }

    public:

      PolyMeshVertexHalfedgeCirculator()
        : m_kernel(0)
        , m_first ()
        , m_cur   ()
        , m_active(false)
      { }

      explicit PolyMeshVertexHalfedgeCirculator(  typename PolyMesh::vertex_type const & v)
        : m_kernel( v.get_owner() )
        , m_first ( init(v)       )
        , m_cur   ( init(v)       )
        , m_active( false         )
      {}

      template <class OtherValue>
      PolyMeshVertexHalfedgeCirculator( PolyMeshVertexHalfedgeCirculator<PolyMesh,OtherValue> const& other )
        : m_kernel( other.m_kernel )
        , m_first ( other.m_first  )
        , m_cur   ( other.m_cur    )
        , m_active( other.m_active )
      {}

      template <class OtherValue>
      bool operator==(PolyMeshVertexHalfedgeCirculator<PolyMesh,OtherValue> const& /*other*/) const
      {
        if( !m_kernel || !m_cur )
          return true;
        return (m_active && m_first == m_cur);
      }

      template <class OtherValue>
      bool operator!=(PolyMeshVertexHalfedgeCirculator<PolyMesh,OtherValue> const& other) const
      {
        return !( (*this)==other );
      }

      PolyMeshVertexHalfedgeCirculator & operator++()
      {
        m_active = true;

        if( !m_kernel || !m_cur)
          return *this;

        m_cur = m_cur.get()->get_twin_iterator()->get_next_iterator();
        return *this;
      }

      PolyMeshVertexHalfedgeCirculator & operator--()
      {
        m_active = true;

        if( !m_kernel || !m_cur)
          return *this;

        m_cur = m_cur.get()->get_prev_iterator()->get_twin_iterator();
        return *this;
      }

      Value& operator*() const
      {
        return *( m_cur.get() );
      }
      Value * operator->() const
      {
        return &( *( m_cur.get() ) );
      }

    };

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_CIRCULATORS_POLYMESH_VERTEX_HALFEDGE_CIRCULATOR_H
#endif
