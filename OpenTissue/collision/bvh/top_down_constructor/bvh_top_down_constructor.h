#ifndef OPENTISSUE_BVH_BVH_TOP_DOWN_CONSTRUCTOR_H
#define OPENTISSUE_BVH_BVH_TOP_DOWN_CONSTRUCTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/top_down_constructor/bvh_default_top_down_policy.h>

#include <list>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Top Down Construction Algorithm.
    *
    * The top down policy must have two methods:
    *
    *  partition_type all(void)
    *  init(iterator begin,iterator end)
    *
    * It must also define two types
    *
    *   partition_type
    *   parition_iterator
    *
    * The partition type must have the following methods:
    *
    *   bool empty()
    *   unsigned int size()
    *   void fit(bv_ptr_type)
    *   split(partition_type & partition)
    *   partition_iterator sub_partition_begin()
    *   partition_iterator sub_partition_end()
    *
    */
    template <
      typename bvh_type,
      typename top_down_policy = DefaultTopDownPolicy<bvh_type>
    >
    class TopDownConstructor : public top_down_policy
    {
    public:

      typedef top_down_policy                                top_down_type;
      typedef typename bvh_type::bv_ptr                      bv_ptr;
      typedef typename bvh_type::bv_ptr_container            bv_ptr_container;
      typedef typename top_down_type::partition_type         partition_type;
      typedef typename top_down_type::partition_iterator     partition_iterator;
      typedef typename std::list<partition_type>             partition_queue;

    public:

      /**
      * Run Algorithm.
      *
      * @param begin      Iterator to first data (i.e. geometry or volume if BVH is non-annotated).
      * @param end        Iterator to one position past last data (i.e. geometry or volume if BVH is non-annotated).
      * @param bvh        Upon return this argument holds the resulting BVH.
      *
      */
      template< typename iterator>
      void run(iterator begin, iterator end, bvh_type & bvh)
      {
        partition_queue Q;
        bv_ptr_container parents;

        this->init(begin, end);//--- top down policy

        bvh.clear();

        bv_ptr null_ptr;

        parents.push_back( null_ptr );

        partition_type all = this->all(); //--- top down policy
        Q.push_back(  all );

        while(!Q.empty())
        {
          partition_type partition( Q.front() );
          Q.pop_front();
          bv_ptr parent( parents.front() );
          parents.pop_front();

          assert(!partition.empty() || !"TopDownConstructor::run() Empty partition encountered!");
          
          bv_ptr bv =  bvh.insert( parent, partition.annotated() );

          partition.fit(bv);
          if(partition.size()==1)
            continue;

          partition.split();

          partition_iterator s   = partition.sub_partition_begin();
          partition_iterator end = partition.sub_partition_end();
          for(;s!=end;++s)
          {
            Q.push_back( (*s) );
            parents.push_back(bv);
          }

        }
        std::cout << "TopDownConstructor::run(): created " << bvh.size() << " nodes" << std::endl;
      }
    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_TOP_DOWN_CONSTRUCTOR_H
#endif
