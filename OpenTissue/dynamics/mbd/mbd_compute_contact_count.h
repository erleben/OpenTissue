#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
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
  namespace mbd
  {

    /**
    * Compute Contact Count.
    * This template function computes the number of contact points
    * between a specified sequence of bodies. This is usefull for
    * debugging or profiling information.
    *
    * Example of usage:
    *
    * std::cout << "|C| = " << mbd::compute_contact_count(configuration.body_begin(),configuration.body_end()) << std::endl;
    *
    */
    template< typename indirect_body_iterator>
    size_t compute_contact_count(indirect_body_iterator begin, indirect_body_iterator end)
    {
      typedef typename indirect_body_iterator::value_type   body_type;
      typedef typename body_type::indirect_edge_iterator    indirect_edge_iterator;

      size_t cnt = 0;

      for(indirect_body_iterator body=begin;body!=end;++body)
      {
        for(indirect_edge_iterator edge=body->edge_begin();edge!=body->edge_end();++edge)
        {
          if(edge->is_up_to_date() )
            cnt += edge->size_contacts();
        }
      }
      //--- Edges have been visited twice once in each
      //--- direction, so we need to divide by 2
      return (cnt/2);
    }

  } // namespace mbd

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_H
#endif
