#ifndef OPENTISSUE_COLLISION_BVH_BVH_BV_TRAVERSAL_ITERATOR_H
#define OPENTISSUE_COLLISION_BVH_BVH_BV_TRAVERSAL_ITERATOR_H
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
  namespace bvh
  {

    /**
    * BV traversal iterator.
    * Runs a traversal on all bv nodes in BVH.
    */
    template<typename bvh_type>
    class BVTraversalIterator 
      : public std::iterator<std::forward_iterator_tag, typename bvh_type::bv_type> 
    {
    public:

      typedef typename bvh_type::bv_type          bv_type;
      typedef typename bvh_type::bv_ptr_container bv_ptr_container;
      typedef typename bvh_type::bv_ptr           bv_ptr;
      typedef typename bvh_type::bv_ptr_iterator  bv_ptr_iterator;
      typedef typename bvh_type::bv_const_ptr     bv_const_ptr;

    protected:

      bv_ptr           m_bv;     ///< Current node being visisted in traversal.
      bv_ptr_container m_queue;  ///< Remaining unvisited nodes in traversal.

    public:

      BVTraversalIterator(bv_ptr bv)
        : m_bv(bv)
      {
        if(bv)
        {
          bv_ptr_iterator begin = m_bv->child_ptr_begin();
          bv_ptr_iterator end   = m_bv->child_ptr_end();
          for(bv_ptr_iterator child = begin; child != end; ++child )
            m_queue.push_back( *child );
        }
      }

      bool operator== ( BVTraversalIterator const & other ) const{  return (other.m_bv==m_bv); }

      bool operator!= ( BVTraversalIterator const & other ) const{  return !((*this)==other); }

      bv_type & operator*() {return (*m_bv);}

      bv_ptr operator->() {return m_bv;}

      bv_type const  & operator*()const {return (*m_bv);}

      bv_const_ptr operator->()const {return bv_const_ptr(m_bv);}

      BVTraversalIterator & operator++()
      {
        if(m_queue.empty())
        {
          // Set bv to null pointer
          m_bv.reset();
          return (*this);
        }
        m_bv = m_queue.front();
        m_queue.pop_front();
        bv_ptr_iterator begin = m_bv->child_ptr_begin();
        bv_ptr_iterator end   = m_bv->child_ptr_end();
        for(bv_ptr_iterator child = begin; child != end; ++child )
          m_queue.push_back( *child );
        return (*this);
      }
    };

  } // namespace bvh
} // namespace OpenTissue

//OPENTISSUE_COLLISION_BVH_BVH_BV_TRAVERSAL_ITERATOR_H
#endif
