#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_CACHED_SOLUTION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_CACHED_SOLUTION_VECTOR_H
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
    * Extract Cached Solution.
    *
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
    * @param x            Upon return this vector holds the cached solution. Vector must
    *                     have size m, where m is total number of jacobian rows.
    */
    template<typename group_type,typename vector_type>
    void get_cached_solution_vector(  
        group_type const & group  
      , size_t const & m  
      , vector_type & x
      )
    {
      typedef typename group_type::math_policy                         math_policy;
      typedef typename math_policy::vector_range                       vector_range;
      typedef typename group_type::const_indirect_constraint_iterator  const_indirect_constraint_iterator;
      typedef typename group_type::const_indirect_contact_iterator     const_indirect_contact_iterator;
      typedef typename vector_type::size_type                          size_type;

      math_policy::resize( x, m);

      for(const_indirect_constraint_iterator constraint = group.constraint_begin();constraint!=group.constraint_end();++constraint)
      {
        if(constraint->is_active())
        {
          size_type const start = constraint->get_jacobian_index();
          size_type const end   = start + constraint->get_number_of_jacobian_rows();
          vector_range tmp_vector_range = math_policy::subrange(x,start,end);
          constraint->get_solution( tmp_vector_range );
        }
      }
      for(const_indirect_contact_iterator contact = group.contact_begin();contact!=group.contact_end();++contact)
      {
        if(contact->is_active())
        {
          size_type const start = contact->get_jacobian_index();
          size_type const end   = start + contact->get_number_of_jacobian_rows();
	  vector_range tmp_vector_range = math_policy::subrange(x,start,end);
          contact->get_solution( tmp_vector_range );
        }
      }
    }


  } //--- end of namespace mbd
} //--- end of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_CACHED_SOLUTION_VECTOR_H
#endif
