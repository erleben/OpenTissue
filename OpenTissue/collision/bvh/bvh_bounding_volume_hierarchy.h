#ifndef OPENTISSUE_COLLISION_BVH_BVH_BOUNDING_VOLUME_HIERARCHY_H
#define OPENTISSUE_COLLISION_BVH_BVH_BOUNDING_VOLUME_HIERARCHY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_bv_node.h>
#include <OpenTissue/collision/bvh/bvh_annotated_bv_node.h>
#include <OpenTissue/collision/bvh/bvh_bv_traversal_iterator.h>
#include <OpenTissue/utility/utility_empty_traits.h>

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/iterator/indirect_iterator.hpp>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Bounding Volume Hierarchy Class.
    */
    template <typename V, typename G, typename T = OpenTissue::utility::EmptyTraits>
    class BoundingVolumeHierarchy 
    {
    public:

      typedef BoundingVolumeHierarchy<V,G,T>               bvh_type;
      typedef V                                            volume_type;
      typedef G                                            geometry_type;
      typedef BV<bvh_type, T>                              bv_type;
      typedef AnnotatedBV<bvh_type, T>                     annotated_bv_type;
      typedef BVTraversalIterator<bvh_type>                bv_traversal_iterator;

      typedef boost::shared_ptr<bvh_type>                  bvh_ptr;
      typedef boost::weak_ptr<bvh_type>                    bvh_weak_ptr;
      typedef boost::shared_ptr<bvh_type const>            bvh_const_ptr;
      typedef boost::weak_ptr<bvh_type const>              bvh_const_weak_ptr;
      typedef boost::shared_ptr<volume_type>               volume_ptr;
      typedef boost::shared_ptr<geometry_type>             geometry_ptr;
      typedef boost::shared_ptr<bv_type>                   bv_ptr;
      typedef boost::weak_ptr<bv_type>                     bv_weak_ptr;
      typedef boost::shared_ptr<bv_type const>             bv_const_ptr;
      typedef boost::weak_ptr<bv_type const>               bv_const_weak_ptr;
      typedef boost::shared_ptr<annotated_bv_type>         annotated_bv_ptr;

      typedef typename std::list<bv_ptr>                   bv_ptr_container;
      typedef typename bv_ptr_container::iterator          bv_ptr_iterator;
      typedef typename bv_ptr_container::const_iterator    bv_const_ptr_iterator;

      typedef typename std::list<annotated_bv_ptr>                    annotated_bv_ptr_container;
      typedef typename annotated_bv_ptr_container::iterator           annotated_bv_ptr_iterator;
      typedef typename annotated_bv_ptr_container::const_iterator     annotated_bv_const_ptr_iterator;

      typedef boost::indirect_iterator<bv_ptr_iterator,bv_type>                   bv_iterator;
      typedef boost::indirect_iterator<bv_const_ptr_iterator,bv_type>             bv_const_iterator;
      typedef boost::indirect_iterator<bv_ptr_iterator,annotated_bv_type>         bv_annotated_iterator;
      typedef boost::indirect_iterator<bv_const_ptr_iterator,annotated_bv_type>   annotated_bv_const_iterator;

      typedef typename std::list<geometry_type>                                   geometry_container;
      typedef typename geometry_container::iterator                               geometry_iterator;
      typedef typename geometry_container::const_iterator                         geometry_const_iterator;

    protected:

      bv_ptr  m_root;         ///< Root Pointer.
      size_t  m_subtrees;     ///< Subtree counter, used to set root-pointer when building tree bottom-up.
      size_t  m_size;         ///< Number of nodes in the BVH.

    protected:

      bvh_ptr m_this;         ///< Shared this pointer.

      struct null_deleter
      {
        void operator()(void const *) const {}
      };

      bvh_ptr get_this()
      {
        // boost shared pointer has the enable_shared_from_this
        // construct. However, we can not use it because
        // the shared_from_this() method only works if
        // one already has got at least one shared pointer
        // to the object that one is trying to get a this
        // pointer for. In our case this may not be the case!

        // We decided to use a local method to create
        // the this-pointer using a null_deleter. One
        // could have initialized a shared pointer in
        // the constructor initializer list. However,
        // it is not nice to initialize a member with
        // the this-pointer. In particular the compiler
        // warns about it. Our current solution works and
        // it free of compiler warnings.
        if(m_this.get() != this)
        {
          // The null-deleter is used to avoid
          // that the shared pointer tries to deallocate this object.
          m_this.reset( this , null_deleter());
        }
        return m_this;
      }

    public:

      bv_traversal_iterator       begin()       { return bv_traversal_iterator(m_root);          }
      bv_traversal_iterator       end()         { return bv_traversal_iterator( bv_ptr() ); }

      const bv_traversal_iterator begin() const
      {
        bv_ptr root = boost::const_pointer_cast<bv_type>(m_root);
        return bv_traversal_iterator(root);
      }

      const bv_traversal_iterator end()  const  { return bv_traversal_iterator( bv_ptr()  ); }

    public:

      BoundingVolumeHierarchy()
        : m_root()
        , m_subtrees(0)
        , m_size(0)
      {}

      BoundingVolumeHierarchy( BoundingVolumeHierarchy const & other )
        : m_root( other.m_root )
        , m_subtrees( other.m_subtrees )
        , m_size( other.m_size )
      {}

      ~BoundingVolumeHierarchy(){ clear(); }

    public:

      bv_ptr       root()        { return m_root; }
      bv_const_ptr root()  const { return m_root; }
      size_t       size()  const { return m_size; }
      bool const   empty() const { return (m_size==0); }

      /**
      * Insert new BV.
      * This insertion routine is meant to be used in a bottom-up manner
      *
      * @param children
      * @param annotated
      * @return
      */
      bv_ptr insert( bv_ptr_container const & children, bool const annotated = false )
      {
        //--- Sanity Check
        bv_const_ptr_iterator begin = children.begin();
        bv_const_ptr_iterator end   = children.end();
        for (bv_const_ptr_iterator child = begin; child != end; ++child )
        {
          if ( !( (*child)->m_parent.expired() ) )
            return bv_ptr();
          if ( (*child)->m_owner.lock() != this->get_this() )
            return bv_ptr();
        }

        //--- Everything is OK, we can create a new BV
        //bv_ptr bv = 0;  //--- shared_ptr is null by default
        bv_ptr bv ;
        if ( annotated )
        {
          bv = bv_ptr( new annotated_bv_type() );
        }
        else
        {
          bv = bv_ptr( new bv_type() );
        }
        bv->m_owner = this->get_this();
        bv->m_parent.reset();

        for (bv_const_ptr_iterator child = begin; child != end;++child )
        {
          (*child)->m_parent = bv;
          bv_ptr ptr( *child );
          bv->m_children.push_back( ptr );
        }
        //--- Adjust subtree counter. Note if node has no children (i.e. is is a leaf)
        //--- then m_subtrees is infact incremented.
        m_subtrees -= children.size() - 1;
        if ( m_subtrees == 1 )
          m_root = bv;
        else
          m_root.reset();
        ++m_size;
        return bv;
      }

      /**
      * Insert new BV.
      * This insertion routine is meant to be used in a top-down manner.
      *
      * @param parent
      * @param annotated
      * @return
      */
      bv_ptr insert( bv_ptr parent, bool const annotated = false )
      {
        //--- If root already exist and parent is null then return null
        if ( m_root && !parent )
        {
          return bv_ptr();
        }
        //--- If there is no root and no parent, then it means the new node is the root
        else if ( !m_root && !parent )
        {
          if ( annotated )
          {
            m_root = bv_ptr( new annotated_bv_type() );
          }
          else
          {
            m_root = bv_ptr( new bv_type() );
          }
          m_root->m_owner = this->get_this();
          ++m_size;
          return m_root;
        }
        //--- If root exist and parent belongs to this bvh then
        //--- insert new bv as child of parent
        else if ( m_root && parent && parent->m_owner.lock() == this->get_this() )
        {
          bv_ptr bv;
          if ( annotated )
          {
            bv = bv_ptr( new annotated_bv_type() );
          }
          else
          {
            bv = bv_ptr( new bv_type() );
          }
          bv->m_owner = this->get_this();
          bv->m_parent =  parent;
          parent->m_children.push_back( bv );
          ++m_size;
          return bv;
        }
        return bv_ptr();
      }

      /**
      * Remove BV.
      *
      * @param bv
      * @return
      */
      bool const remove( bv_ptr bv )
      {
        assert( bv                                     || !"BoundingVolumeHierarchy::remove(): bv pointer was null");
        assert( bv->m_owner.lock() == this->get_this() || !"BoundingVolumeHierarchy::remove(): bv belongs to another BVH");
        if ( bv->children() )
          return false;
        bv_ptr parent = bv->m_parent.lock();
        if ( parent )
        {
          parent->m_children.remove( bv );
          bv->m_parent.reset();
        }
        bv->m_owner.reset();
        if ( bv == m_root )
          m_root.reset();
        bv.reset();
        --m_size;
        return true;
      }

      void clear()
      {
        m_root.reset();
        m_subtrees = 0;
        m_size = 0;
      }

    };

  } // namespace bvh
} // namespace OpenTissue

//OPENTISSUE_COLLISION_BVH_BVH_BOUNDING_VOLUME_HIERARCHY_H
#endif
