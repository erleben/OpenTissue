#ifndef OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TOP_DOWN_POLICY_H
#define OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TOP_DOWN_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_obb_fit.h>

#include <cassert>

namespace OpenTissue
{

  namespace obb_tree
  {

    template<
      typename bvh_type
      , typename obb_tree_types_
    >
    class TopDownPolicy
    {
    public:

      typedef          obb_tree_types_                   obb_tree_types;
      typedef typename obb_tree_types::obb_type          obb_type;
      typedef typename obb_tree_types::plane_type        plane_type;


      typedef typename obb_tree_types::math_types    math_types;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::value_traits      value_traits;


      typedef          TopDownPolicy<bvh_type,obb_tree_types>     top_down_type;

      typedef typename bvh_type::bv_ptr                  bv_ptr;
      typedef typename bvh_type::annotated_bv_type       annotated_bv_type;
      typedef typename bvh_type::annotated_bv_ptr        annotated_bv_ptr;
      typedef typename bvh_type::volume_type             volume_type;
      typedef typename bvh_type::geometry_type           geometry_type;
      typedef typename std::vector<geometry_type>        geometry_container;

      typedef typename obb_tree_types::face_ptr_type     face_ptr_type;


    protected:

      geometry_container m_geometry;     ///< The entire geometry that should be represented by the resulting BVH.

      size_t m_degree;      ///< The degree of the resulting hiearchy, ie. the cardinaltiy. Default value is 2.

    public:

      class partition_type
      {
      public:

        friend class TopDownPolicy<bvh_type,obb_tree_types>;

      public:

        typedef std::vector<partition_type>              partition_container;
        typedef typename partition_container::iterator   partition_iterator;

      protected:

        partition_container    m_sub_partitions;
        size_t  		       m_left;
        size_t 	        	   m_right;
        top_down_type        * m_owner;
        bv_ptr                 m_bv;

      public:

        partition_type()
          : m_left(0)
          , m_right(0)
          , m_owner(0)
          , m_bv()
        {}

        partition_type(top_down_type * owner,unsigned int left,unsigned int  right)
          : m_left(left)
          , m_right(right)
          , m_owner(owner)
          , m_bv()
        {}

        bool annotated() const { return size()==1; }
        unsigned int size() const { return (m_right-m_left+1); }
        bool empty() { return (size()==0); }
        void split() { m_owner->split((*this)); }
        partition_iterator sub_partition_begin() { return m_sub_partitions.begin(); }
        partition_iterator sub_partition_end() { return m_sub_partitions.end(); }

        void fit(bv_ptr bv)
        {
          m_bv = bv;
          m_owner->fit(bv,(*this));
        }

      };

    public:

      typedef typename partition_type::partition_container   partition_container;
      typedef typename partition_type::partition_iterator    partition_iterator;

    protected:

      class vector3_iterator 
        : public std::iterator<std::forward_iterator_tag, vector3_type> 
      {
      protected:

        top_down_type * m_owner;
        unsigned int    m_idx;
        unsigned int    m_sub_idx;

      public:

        vector3_iterator(top_down_type * owner,unsigned int idx)
          : m_owner(owner)
          , m_idx(idx)
          , m_sub_idx()
        {}

      public:

        bool operator==(vector3_iterator const& other) const {  return (m_idx==other.m_idx && m_sub_idx==other.m_sub_idx); }
        bool operator!=(vector3_iterator const& other) const {  return !(m_idx==other.m_idx && m_sub_idx==other.m_sub_idx); }

        vector3_iterator & operator++()
        {
          if(m_sub_idx==3)
          {
            ++m_idx;
            m_sub_idx = 0;
          }
          else
          {
            ++m_sub_idx;
          }
          return *this;
        }

        vector3_type & operator*() const
        {
          face_ptr_type  f = m_owner->m_geometry[m_idx];
          if(m_sub_idx==0)
            return *(f->m_v0);
          else if (m_sub_idx==1)
            return *(f->m_v1);
          return *(f->m_v2);
        }

      };

    public:

      TopDownPolicy()
        : m_degree(2)
      {}


      partition_type all() { return partition_type(this,0,static_cast<unsigned int>(m_geometry.size()-1)); }

      template<typename iterator>
      void init(iterator begin,iterator end)
      {
        m_geometry.clear();
        std::copy( begin, end, std::back_inserter( m_geometry ) );
      }

      /**
       * Set Hierarchy Cardinality.
       *
       * @param new_degree   The new degree that should be used when building a hierarchy. Valid values are 2, 4 and 8.
       */
      void set_degree(size_t const & new_degree)
      {
        assert( new_degree == 2u || new_degree == 4u || new_degree == 8u || !"TopDownPolicy::set_degree(): invalid degree value!");
        m_degree = new_degree;
      }

    protected:

      /**
       * Retrieve the wanted cardinality of the hiearchy.
       */
      size_t const & degree() const {return m_degree;}

      /**
      * This method splits a sequencial set of geometry elements into two disjoint subsets, a left and a right.
      *
      * @param plane        The plane used to split the set
      * @param set          The set (i.e. a partitioning of the geometry)
      * @param left_subset  Upon return this argument holds the left subset.
      * @param right_subset Upon return this argument holds the right subset.
      *
      */
      void make_plane_split(plane_type const & plane, partition_type const & set,  partition_type & left_subset,  partition_type & right_subset )
      {
        assert( set.size() > 1u || !"make_plane_split(): Impossible to split a set of one element into two subsets");

        using std::min;

        vector3_type center;

        size_t current = set.m_left;
        size_t last    = set.m_right;

        for(;current<last;)
        {
          OpenTissue::mesh::compute_face_center( (*m_geometry[current]), center);
          if( plane.signed_distance(center) > value_traits::zero() )
          {
            std::swap(m_geometry[current],m_geometry[last]);
            --last;
          }
          else 
          {
            ++current;
          }
        }

        size_t first_right = min(last+1, set.m_right);
        size_t last_left   = first_right - 1;
        size_t first_left  = set.m_left;
        size_t last_right  = set.m_right;

        left_subset  = partition_type( this, first_left,  last_left  );
        right_subset = partition_type( this, first_right, last_right );       
      }

      void split(partition_type & partition)
      {
        using std::min;

        assert(partition.m_bv || !"TopDownPolicy::split(...): bounding volume was null?");

        // Compute possible split-planes
        obb_type    & obb    = partition.m_bv->volume();
        vector3_type  center = obb.center();

        size_t order[3];
        OpenTissue::math::get_increasing_order( obb.ext(), order );

        vector3_type  axis1 = obb.orientation().column( order[2] );
        vector3_type  axis2 = obb.orientation().column( order[1] );
        vector3_type  axis3 = obb.orientation().column( order[0] );

        plane_type    split_plane1(axis1,center);
        plane_type    split_plane2(axis2,center);
        plane_type    split_plane3(axis3,center);

        // allocate space for the resulting paritions after having performed the splits
        partition.m_sub_partitions.resize( this->degree() );

        // Now perform the split of the partition into new sub partitions.
        if(this->degree()==2)
        {
          make_plane_split( split_plane1, partition, partition.m_sub_partitions[0], partition.m_sub_partitions[1] );
        }
        else if(this->degree()==4)
        {
          partition_type tmp_left;
          partition_type tmp_right;
          make_plane_split( split_plane1, partition, tmp_left, tmp_right );

          make_plane_split( split_plane2, tmp_left,  partition.m_sub_partitions[0], partition.m_sub_partitions[1] );
          make_plane_split( split_plane2, tmp_right, partition.m_sub_partitions[2], partition.m_sub_partitions[3] );
        }
        else if(this->degree()==8)
        {
          partition_type tmp_left;
          partition_type tmp_right;
          make_plane_split( split_plane1, partition, tmp_left, tmp_right );

          partition_type tmp_left_left;
          partition_type tmp_left_right;
          partition_type tmp_right_left;
          partition_type tmp_right_right;
          make_plane_split( split_plane2, tmp_left,  tmp_left_left, tmp_left_right );
          make_plane_split( split_plane2, tmp_right, tmp_right_left, tmp_right_right );

          make_plane_split( split_plane3, tmp_left_left,   partition.m_sub_partitions[0], partition.m_sub_partitions[1] );
          make_plane_split( split_plane3, tmp_left_right,  partition.m_sub_partitions[2], partition.m_sub_partitions[3] );
          make_plane_split( split_plane3, tmp_right_left,  partition.m_sub_partitions[4], partition.m_sub_partitions[5] );
          make_plane_split( split_plane3, tmp_right_right, partition.m_sub_partitions[6], partition.m_sub_partitions[7] );
        }
        //real_type value = obb.ext()(0);
        //unsigned int column = 0;
        //
        //if(obb.ext()(1) > value)
        //{
        //  value = obb.ext()(1);
        //  column = 1;
        //}
        //if(obb.ext()(2) > value)
        //{
        //  column = 2;
        //}
        //
        //vector3_type  axis   = vector3_type( obb.orientation()(0,column), obb.orientation()(1,column), obb.orientation()(2,column));
        //plane_type    split_plane(axis,center);
        //
        //unsigned int first = partition.m_left;
        //unsigned int last  = partition.m_right;
        //unsigned int i     = first;
        //for(;i<last;)
        //{
        //  mesh::compute_face_center( (*m_geometry[i]), center);
        //  if(split_plane.signed_distance(center) > 0)
        //  {
        //    std::swap(m_geometry[i],m_geometry[last]);
        //    --last;
        //  }
        //  else 
        //  {
        //    ++i;
        //  }
        //}
        //unsigned int sub_right = min(last+1,partition.m_right);
        //unsigned int sub_left  = sub_right - 1;
        //
        //partition.m_sub_partitions.resize(2);
        //partition.m_sub_partitions[0] = partition_type( this, partition.m_left, sub_left          );
        //partition.m_sub_partitions[1] = partition_type( this,   sub_right     , partition.m_right );
      }

      void fit(bv_ptr bv,partition_type & partition)
      {
        partition.m_bv = bv;

        if(partition.annotated())
        {
          annotated_bv_ptr A = boost::static_pointer_cast<annotated_bv_type>(bv);
          A->insert( m_geometry[partition.m_left] );
        }
        vector3_iterator begin( this, partition.m_left     );
        vector3_iterator end(   this, partition.m_right+1  );
        OpenTissue::geometry::obb_fit(begin,end,bv->volume());
      }

    };

  } // namespace obb_tree

} // namespace OpenTissue

//OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TOP_DOWN_POLICY_H
#endif
