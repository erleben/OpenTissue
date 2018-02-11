#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_DEPENDENCIES_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_DEPENDENCIES_VECTOR_H
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
      * Get Dependencies Vector.
      * For some variables their upper and lower founds are modelled
      * as a linear dependency on some other variable.
      *
      *
      *  That is for the $j$'th variable we might have
      *
      *     lo_j = -mu_j x_i
      *     hi_j =  mu_j x_i
      *
      * This method extract the index of the depedent variable. That is
      *
      *   pi_j = i
      *
      * If the j'th entry stores the maximum possible value of the underlying
      * data-type in the vector then it means that there are no dpendencies.
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
      * @param dep   Upon return this vectors holds the indices of the dependent constraints.
      */
      template<typename group_type,typename idx_vector_type>
      void get_dependencies_vector(  
        group_type const & group  
        , size_t const & m  
        , idx_vector_type & pi
        )
      {
        typedef typename group_type::math_policy                                math_policy;
        typedef typename group_type::const_indirect_constraint_iterator         const_indirect_constraint_iterator;
        typedef typename group_type::const_indirect_contact_iterator            const_indirect_contact_iterator;
        typedef typename idx_vector_type::size_type                             size_type;
        typedef typename math_policy::idx_vector_range                          idx_vector_range;

        math_policy::resize(pi,m);

        for(const_indirect_constraint_iterator constraint = group.constraint_begin();constraint!=group.constraint_end();++constraint)
        {
          if(constraint->is_active())
          {
            size_type const start = constraint->get_jacobian_index();
            size_type const end = start + constraint->get_number_of_jacobian_rows();            
            idx_vector_range tmp_vector_range = math_policy::subrange(pi,start,end);
            constraint->get_dependency_indices( tmp_vector_range );
          }
        }
        for(const_indirect_contact_iterator contact = group.contact_begin();contact!=group.contact_end();++contact)
        {
          if(contact->is_active())
          {
            size_type const start = contact->get_jacobian_index();
            size_type const end = start + contact->get_number_of_jacobian_rows();
            idx_vector_range tmp_vector_range = math_policy::subrange(pi,start,end);
            contact->get_dependency_indices( tmp_vector_range );
          }
        }
      }
    } //--- End of namespace detail
  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_DEPENDENCIES_VECTOR_H
#endif
