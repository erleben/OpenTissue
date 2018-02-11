#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_MATRIX_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_MATRIX_H
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
    * Compute Contact Count Matrix.
    * This template function computes the number of contact points
    * between pairs of bodies. Upon return the result is stored into a matrix.
    *
    * This is usefull for debugging information. Example of usage:
    *
    * matrix_type C;
    * mbd::compute_contact_count_matrix(configuration.body_begin(),configuration.body_end(),C);
    * std::cout << "C = " << C << ";" << std::endl;
    *
    */
    template< typename indirect_body_iterator,typename matrix_type>
    void compute_contact_count_matrix(indirect_body_iterator begin, indirect_body_iterator end,matrix_type & C)
    {
      typedef typename indirect_body_iterator::value_type   body_type;
      typedef typename body_type::math_policy               math_policy;
      typedef typename body_type::indirect_edge_iterator    indirect_edge_iterator;


      typedef typename matrix_type::value_type              value_type;

      size_t n = std::distance(begin,end);

      math_policy::resize(C,n,n);
      C.clear();

      typename body_type::size_type i = 0;
      for(indirect_body_iterator body=begin;body!=end;++body,++i)
        body->m_tag = i;

      for(indirect_body_iterator body=begin;body!=end;++body)
      {
        for(indirect_edge_iterator edge=body->edge_begin();edge!=body->edge_end();++edge)
        {
          if(edge->is_up_to_date())
          {
            C(edge->get_body_A()->m_tag,edge->get_body_B()->m_tag) = value_type(edge->size_contacts());
          }
        }
      }
    }

  } // namespace mbd

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_COMPUTE_CONTACT_COUNT_MATRIX_H
#endif
