#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_LIMIT_VECTORS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_LIMIT_VECTORS_H
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
    namespace detail
    {
      /**
      * Get Constraint Limits.
      *
      * Further it is assumed that the vector library used provides a vector
      * proxy function called subrange, which is capable of returning a
      * vector range. (see for instance in Boost uBLAS for an example). 
      *
      * @param group        The group corresponding to the A-matrix.
      *
      * @param m            The number of active constraints in the group (i.e. the
      *                     number of rows in the Jacobian matrix).
      *
      * @param lo   Upon return this vectors holds the values of the lower constraint limit. Vector must
      *                     have size m, where m is total number of jacobian rows.
      * @param hi   Upon return this vectors holds the values of the upper constraint limit. Vector must
      *                     have size m, where m is total number of jacobian rows.
      */
      template<typename group_type,typename vector_type>
      void get_limit_vectors(  
          group_type const & group  
        , size_t const & m  
        , vector_type & lo
        , vector_type & hi  
        )
      {
        typedef typename group_type::math_policy                                math_policy;
        typedef typename group_type::const_indirect_constraint_iterator         const_indirect_constraint_iterator;
        typedef typename group_type::const_indirect_contact_iterator            const_indirect_contact_iterator;
        typedef typename vector_type::size_type                                 size_type;
        typedef typename math_policy::vector_range                              vector_range;

        math_policy::resize( lo, m);
        math_policy::resize( hi, m);

        for(const_indirect_constraint_iterator constraint = group.constraint_begin();constraint!=group.constraint_end();++constraint)
        {
          if(constraint->is_active())
          {
            size_type const start = constraint->get_jacobian_index();
            size_type const end = start + constraint->get_number_of_jacobian_rows();
            vector_range low_vector_range = math_policy::subrange(lo,start,end);
            constraint->get_low_limits( low_vector_range );
            vector_range high_vector_range = math_policy::subrange(hi,start,end);
            constraint->get_high_limits( high_vector_range );
          }
        }
        for(const_indirect_contact_iterator contact = group.contact_begin();contact!=group.contact_end();++contact)
        {
          if(contact->is_active())
          {
            size_type const start = contact->get_jacobian_index();
            size_type const end = start + contact->get_number_of_jacobian_rows();
            vector_range low_vector_range = math_policy::subrange(lo,start,end);
            contact->get_low_limits( low_vector_range );
            vector_range high_vector_range = math_policy::subrange(hi,start,end);
            contact->get_high_limits( high_vector_range );
          }
        }
      }

    } //--- end of namespace detail
  } //--- end of namespace mbd
} //--- end of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_LIMIT_VECTORS_H
#endif
