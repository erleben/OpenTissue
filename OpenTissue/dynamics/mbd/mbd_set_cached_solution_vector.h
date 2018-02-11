#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_CACHED_SOLUTION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_CACHED_SOLUTION_VECTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * Set Cached Solution.
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
    * @param x            This vector holds a solution that should be cached. Vector must
    *                     have size m, where m is total number of jacobian rows.
    */
    template<typename group_type,typename vector_type>
    void set_cached_solution_vector(  
      group_type & group  
      , size_t const & m  
      , vector_type & x
      )
    {
      typedef typename group_type::math_policy                 math_policy;
      typedef typename group_type::indirect_constraint_iterator         indirect_constraint_iterator;
      typedef typename group_type::indirect_contact_iterator   indirect_contact_iterator;
      typedef typename vector_type::size_type                  size_type;

      assert(x.size()==m || !"set_cached_solution(): wrong dimension");

      for(indirect_constraint_iterator constraint = group.constraint_begin();constraint!=group.constraint_end();++constraint)
      {
        if(constraint->is_active())
        {
          size_type const start = constraint->get_jacobian_index();
          size_type const end   = start + constraint->get_number_of_jacobian_rows();
          constraint->set_solution( math_policy::subrange( x,start,end ) );
        }
      }
      for(indirect_contact_iterator contact = group.contact_begin();contact!=group.contact_end();++contact)
      {
        if(contact->is_active())
        {
          size_type const start = contact->get_jacobian_index();
          size_type const end   = start + contact->get_number_of_jacobian_rows();
          contact->set_solution( math_policy::subrange( x,start,end ) );
        }
      }
    }

  } //--- end of namespace mbd
} //--- end of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_SET_CACHED_SOLUTION_VECTOR_H
#endif
