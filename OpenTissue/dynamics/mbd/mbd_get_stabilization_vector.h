#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_STABILIZATION_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_STABILIZATION_VECTOR_H
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
      * Get Stabilization Terms.
      * The evaluate_constraints() method is supposed to be invoked prior to this function.
      *
      * This function extracts a vector $\vec b_{text{stabilization}}$. This vector makes it
      * possible to add extra terms to the b-vector in the linear relation
      *
      *  \vec y = A \vec x + \vec b
      *
      * Where $\vec y$ measures the relative velocities of the constraints, and 
      * vector x is a Lagrange multiplier vector. Thus, one have
      * 
      *  \vec b^{\prime} = \vec b + \vec b_{text{stabilization}}
      *
      * And the linear relation then becomes
      *
      *  \vec y = A \vec x + \vec b^{\prime}
      *
      * The vector $\vec b_{text{stabilization}}$ can be used for constraint
      * stabilization and bounce (simple Newton Impact type of law).
      *
      * Observe that for constanct points the kind of terms added to the vector
      * $\vec b_{text{stabilization}}$  dependes on the arguments given to
      * the method evaluate_constraints.
      *
      *
      * Further it is assumed that the vector library used provides a vector
      * proxy function called subrange, which is capable of returning a
      * vector range. (see for instance in Boost uBLAS for an example). 
      *
      * @param group        The group corresponding to the A-matrix.
      * @param m            The number of active constraints in the group (i.e. the
      *                     number of rows in the Jacobian matrix).
      * @param b            Upon return this vector holds a measuer of constraint
      *                     error for each constraint variable. This vector can for
      *                     instance be used for constraint stabilization.
      */
      template<typename group_type,typename vector_type>
      void get_stabilization_vector(   
          group_type const & group  
        , size_t const & m  
        , vector_type & b
        )
      {
        typedef typename group_type::math_policy                         math_policy;
        typedef typename group_type::const_indirect_constraint_iterator  const_indirect_constraint_iterator;
        typedef typename group_type::const_indirect_contact_iterator     const_indirect_contact_iterator;
        typedef typename vector_type::size_type                          size_type;
        typedef typename math_policy::vector_range                       vector_range;

        math_policy::resize( b, m);

        for(const_indirect_constraint_iterator constraint = group.constraint_begin();constraint != group.constraint_end();++constraint)
        {
          if(constraint->is_active())
          {
            size_type const start = constraint->get_jacobian_index();
            size_type const end = start + constraint->get_number_of_jacobian_rows();
            vector_range tmp_vector_range = math_policy::subrange( b,start,end );
            constraint->get_stabilization_term( tmp_vector_range );
          }
        }
        for(const_indirect_contact_iterator contact = group.contact_begin();contact != group.contact_end();++contact)
        {
          if(contact->is_active())
          {
            size_type const start = contact->get_jacobian_index();
            size_type const end = start + contact->get_number_of_jacobian_rows();
            vector_range tmp_vector_range =  math_policy::subrange( b,start,end );
            contact->get_stabilization_term( tmp_vector_range );
          }
        }
      }

    } //--- end of namespace detail
  } //--- end of namespace mbd
} //--- end of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_STABILIZATION_VECTOR_H
#endif
