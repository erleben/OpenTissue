#ifndef OPENTISSUE_COLLISION_SDF_SDF_TOP_DOWN_POLICY_H
#define OPENTISSUE_COLLISION_SDF_SDF_TOP_DOWN_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_compute_smallest_sphere.h>

namespace OpenTissue
{

  namespace sdf
  {

    /**
    * Top Down Splitting for building a sphere BVH for the sample
    * points in the signed distance field geomtery.  
    */
    template<typename bvh_type>
    class TopDownPolicy
    {
    public:
      //--- Conenience stuff for better readability
      typedef TopDownPolicy<bvh_type>                       top_down_type;
      typedef typename bvh_type::bv_ptr                         bv_ptr;
      typedef typename bvh_type::annotated_bv_type              annotated_bv_type;
      typedef typename bvh_type::annotated_bv_ptr               annotated_bv_ptr;
      typedef typename bvh_type::volume_type                    volume_type;
      typedef typename bvh_type::geometry_type                  geometry_type;
      typedef typename std::vector<geometry_type>               geometry_container;

      typedef typename volume_type::vector3_type                vector3_type;

    protected:

      geometry_container m_geometry;     ///< The entire geometry that should be represented by the resulting BVH.

    public:

      class partition_type
      {
      public:

        friend class TopDownPolicy<bvh_type>;

      public:

        typedef std::vector<partition_type>              partition_container;
        typedef typename partition_container::iterator   partition_iterator;

      protected:

        partition_container    m_sub_partitions;
        top_down_type        * m_owner;
        unsigned int           m_left;
        unsigned int           m_right;

      public:

        partition_type()
          : m_owner(0)
          , m_left(0)
          , m_right(0)
        {}

        partition_type(top_down_type * owner,unsigned int left,unsigned int  right)
          : m_owner(owner)
          , m_left(left)
          , m_right(right)
        {}

        bool annotated() const{ return size()==1; }
        unsigned int size() const{ return ( m_right-m_left+1); }
        bool empty() { return (size()==0); }
        void split() { m_owner->split((*this)); }
        partition_iterator sub_partition_begin() { return m_sub_partitions.begin(); }
        partition_iterator sub_partition_end() { return m_sub_partitions.end(); }
        void fit(bv_ptr bv) { m_owner->fit(bv,(*this)); }
      };

    public:
      typedef typename partition_type::partition_container   partition_container;
      typedef typename partition_type::partition_iterator    partition_iterator;
    public:

      partition_type all() { return partition_type(this,0, static_cast<unsigned int>(m_geometry.size()-1)); }

      template<typename iterator>
      void init(iterator begin, iterator end)
      {
        unsigned int N = std::distance(begin,end);
        m_geometry.resize(N);
        unsigned int idx = 0;
        for(iterator geometry = begin;geometry!=end;++geometry,++idx)
          m_geometry[idx] = &(*geometry);
        //m_geometry.clear();
        //std::copy( begin, end, std::back_inserter( m_geometry ) );
      }

    protected:

      void split(partition_type & partition)
      {
        using std::min;

        //--- Test if we need to compute a splitting-plane
        unsigned int total_size = partition.size();
        if(total_size<=8u)
        {
          unsigned int cnt         = min(total_size,8u);
          unsigned int subset_size = total_size/cnt;
          assert(subset_size==1);
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
          return;
        }
        //--- Compute center
        vector3_type center = vector3_type(0,0,0);
        for(unsigned int local_idx=0;local_idx<total_size;++local_idx)
        {
          unsigned int global_idx = local_idx + partition.m_left;
          center += (*m_geometry[global_idx]);
        }
        center /= total_size;
        //--- Split into octree
        geometry_container tmp[8];
        for(unsigned int local_idx=0;local_idx<total_size;++local_idx)
        {
          unsigned int global_idx = local_idx + partition.m_left;
          vector3_type diff = (*m_geometry[global_idx]) - center;
          int mask = ((diff(2)>=0)<<2) | ((diff(1)>=0)<<1)  | (diff(0)>=0);
          assert(mask>=0);
          assert(mask<8);
          tmp[mask].push_back(m_geometry[global_idx]);
        }
        //--- create sub-partitions
        unsigned int i=0;
        unsigned int cnt = 0;
        for(i=0;i<8u;++i)
        {
          if(tmp[i].size()>0)
            ++cnt;
        };
        assert(cnt>1);
        partition.m_sub_partitions.resize(cnt);
        unsigned int global_left_idx = partition.m_left;
        unsigned int j=0;
        for(i=0;i<8u;++i)
        {
          if(tmp[i].empty())
            continue;
          unsigned int size = static_cast<unsigned int>( tmp[i].size() );
          for(unsigned int local_idx=0;local_idx<size;++local_idx)
          {
            unsigned int global_idx = local_idx + global_left_idx;
            m_geometry[global_idx] = tmp[i][local_idx];
          }
          unsigned int global_right_idx = global_left_idx + size - 1;
          partition.m_sub_partitions[j] = partition_type(this, global_left_idx, global_right_idx);
          ++j;
          global_left_idx = global_right_idx + 1;
        }
        assert(global_left_idx==partition.m_right+1);
      }

      void fit(bv_ptr bv,partition_type & partition)
      {
        if(partition.annotated())
        {
          annotated_bv_ptr A = boost::static_pointer_cast<annotated_bv_type>(bv);

          A->insert( m_geometry[partition.m_left] );
        }
        vector3_type  *  points = new vector3_type[partition.size()];
        for(unsigned int i=0;i<partition.size();++i)
          points[i] = *(m_geometry[i + partition.m_left]);
        OpenTissue::geometry::compute_smallest_sphere( points, points + partition.size(), bv->volume() );
        delete [] points;
      }
    };


  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_TOP_DOWN_POLICY_H
#endif
