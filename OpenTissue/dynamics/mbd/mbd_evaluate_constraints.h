#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_EVALUATE_CONSTRAINTS_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_EVALUATE_CONSTRAINTS_H
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
      * Evaluate Constraints.
      * This method evaluates each constraint to determine whether is is active or
      * not, and to set up any parameter values. Upon return the total number
      * of active constraint variables (ie. number of rows in the jacobian) is
      * returned.
      *
      * @param fps                  Frames per second.
      * @param use_stabilization    Boolean flag indicating whether stabiliation should be turned off or on.
      * @param use_friction         Boolean flag indicating whether friction should be turned off on all contact points in group.
      * @param use_bounce           Boolean flag indicating whether collision law should be turned off on all contact points in group (This makes all contacts completely inelastic).
      * @param use_erp           
      *
      * @return                     Total number of active constraints.
      */
      template<typename group_type,typename real_type>
      size_t evaluate_constraints(  
          group_type & group
        , real_type const & fps 
        , bool const & use_stabilization
        , bool const & use_friction
        , bool const & use_bounce
        , bool const & use_erp
        )
      {
        typedef typename group_type::indirect_constraint_iterator         indirect_constraint_iterator;
        typedef typename group_type::indirect_contact_iterator            indirect_contact_iterator;

        assert( fps >= 0 || !"evaluate_constraints(): fps must be positive");
        size_t cnt = 0;    

        for(indirect_constraint_iterator constraint = group.constraint_begin();constraint!=group.constraint_end();++constraint)
        {
          constraint->set_frames_per_second(fps);
          constraint->use_erp() = use_erp;
          constraint->evaluate();
          if(constraint->is_active())
          {
            constraint->set_jacobian_index(cnt);
            cnt += constraint->get_number_of_jacobian_rows();
          }
        }
        for(indirect_contact_iterator contact = group.contact_begin();contact!=group.contact_end();++contact)
        {
          contact->set_frames_per_second(fps);
          contact->use_erp() = use_erp;
          contact->set_use_stabilization(use_stabilization);
          contact->set_use_friction(use_friction);
          contact->set_use_bounce(use_bounce);
          contact->evaluate();
          if(contact->is_active())
          {
            contact->set_jacobian_index(cnt);
            cnt += contact->get_number_of_jacobian_rows();
          }
        }
        return cnt;
      }
    } //--- End of namespace detail
  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_EVALUATE_CONSTRAINTS_H
#endif
