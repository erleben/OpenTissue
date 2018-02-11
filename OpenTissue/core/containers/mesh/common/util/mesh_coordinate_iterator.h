#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COORDINATE_ITERATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COORDINATE_ITERATOR_H
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
  namespace mesh
  {


    template<typename mesh_type>
    class CoordinateIterator
    {
    public:

      typedef typename mesh_type::math_types::vector3_type     value_type;
      typedef typename mesh_type::vertex_iterator              vertex_iterator;

    protected:

      vertex_iterator m_iterator;

    public:


      CoordinateIterator()
        : m_iterator()
      {}

      explicit CoordinateIterator(vertex_iterator const & v)
        : m_iterator(v)
      {}

      CoordinateIterator(CoordinateIterator const & c)
        : m_iterator(c.m_iterator)
      {}

      CoordinateIterator &operator=(vertex_iterator const & v) { m_iterator =v; };
      CoordinateIterator &operator=(CoordinateIterator const & c) {  m_iterator = c.m_iterator; };

    public:

      bool operator==(CoordinateIterator const& other) const    {      return (m_iterator == other.m_iterator);    }
      bool operator!=(CoordinateIterator const& other) const    {      return (m_iterator != other.m_iterator);    }

      CoordinateIterator & operator++()
      {
        ++m_iterator;
        return *this;
      }

      CoordinateIterator & operator--()
      {
        --m_iterator;
        return *this;
      }

      value_type& operator*() const { return m_iterator->m_coord; }
      value_type * operator->() const { return &(m_iterator->m_coord); }

    };
    
  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_COORDINATE_ITERATOR_H
#endif
