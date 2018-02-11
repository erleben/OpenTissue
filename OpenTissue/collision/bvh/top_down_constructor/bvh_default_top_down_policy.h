#ifndef OPENTISSUE_BVH_BVH_DEFAULT_TOPDOWN_POLICY_H
#define OPENTISSUE_BVH_BVH_DEFAULT_TOPDOWN_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp> //needed for boost::static_pointer_cast
#include <vector>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * A Default Top Down Construction Policy.
    */
    template<typename bvh_type>
    class DefaultTopDownPolicy
    {
    public:

      typedef DefaultTopDownPolicy<bvh_type>             top_down_type;
      typedef typename bvh_type::bv_ptr                  bv_ptr;
      typedef typename bvh_type::annotated_bv_type       annotated_bv_type;
      typedef typename bvh_type::annotated_bv_ptr        annotated_bv_ptr;
      typedef typename bvh_type::volume_type             volume_type;
      typedef typename bvh_type::geometry_type           geometry_type;
      typedef typename std::vector<geometry_type>        geometry_container;

    protected:

      geometry_container m_geometry;     ///< The entire geometry that should be represented by the resulting BVH.

    public:

      class partition_type
      {
      public:

        friend class DefaultTopDownPolicy<bvh_type>;

      public:

        typedef std::vector<partition_type>              partition_container;
        typedef typename partition_container::iterator   partition_iterator;

      protected:

        partition_container    m_sub_partitions;
        unsigned int           m_left;
        unsigned int           m_right;
        top_down_type        * m_owner;

      public:

        partition_type()
          : m_left(0)
          , m_right(0)
          , m_owner(0)
        {}

        partition_type(top_down_type * owner,unsigned int left,unsigned int  right)
          : m_left(left)
          , m_right(right)
          , m_owner(owner)
        {}

        bool annotated() const { return size()==1; }
        unsigned int size() const { return (m_right-m_left+1); }
        bool empty() { return (size()==0); }
        void split() {  m_owner->split((*this)); }
        partition_iterator sub_partition_begin() { return m_sub_partitions.begin(); }
        partition_iterator sub_partition_end() { return m_sub_partitions.end(); }
        void fit(bv_ptr bv){ m_owner->fit(bv,(*this)); }

      };

    public:

      typedef typename partition_type::partition_container   partition_container;
      typedef typename partition_type::partition_iterator    partition_iterator;

    public:

      partition_type all() { return partition_type(this,0,m_geometry.size()-1); }

      template<typename iterator>
      void init(iterator begin,iterator end)
      {
        //unsigned int N = size(begin,end);
        //m_geometry.resize(N);
        //unsigned int idx = 0;
        //for(iterator geometry = begin;geometry!=end;++geometry,++idx)
        //  m_geometry[idx] = &(*geometry);
        m_geometry.clear();
        std::copy( begin, end, std::back_inserter( m_geometry ) );
      }

    protected:

      unsigned int degree() const {return 3;}

      void split(partition_type & partition)
      {
        using std::min;

        unsigned int total_size  = partition.size();
        unsigned int cnt         = min(total_size,degree());
        unsigned int subset_size = total_size/cnt;
        partition.m_sub_partitions.resize(cnt);
        for(unsigned int i=0;i<cnt;++i)
        {
          unsigned int left = i*subset_size;
          unsigned int right = (i==cnt-1)?total_size  - 1:(i+1)*subset_size  - 1;
          left += + partition.m_left;
          right += + partition.m_left;
          partition.m_sub_partitions[i] = partition_type(this,left,right);
          assert(!partition.m_sub_partitions[i].empty());
        }
      }


      void fit(bv_ptr bv,partition_type & partition)
      {
        if(partition.annotated())
        {
          annotated_bv_ptr A = boost::static_pointer_cast<annotated_bv_type>(bv);
          A->insert( m_geometry[partition.m_left] );
        }
      }

    };

  } // namespace bvh
} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_DEFAULT_TOPDOWN_POLICY_H
#endif
