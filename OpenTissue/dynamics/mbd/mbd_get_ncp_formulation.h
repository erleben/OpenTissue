#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_NCP_FORMULATION_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_NCP_FORMULATION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/mbd_evaluate_constraints.h>
#include <OpenTissue/dynamics/mbd/mbd_get_jacobian_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_get_dependencies_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_factors_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_limit_vectors.h>
#include <OpenTissue/dynamics/mbd/mbd_get_regularization_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_stabilization_vector.h>
#include <OpenTissue/dynamics/mbd/mbd_get_inverse_mass_matrix.h>

namespace OpenTissue
{
  namespace mbd
  {
    template<typename group_type, typename real_type, typename matrix_type, typename vector_type, typename idx_vector_type>
    void get_ncp_formulation(
        group_type & group
      , real_type const & fps 
      , matrix_type & J
      , matrix_type & invM
      , vector_type & lo
      , vector_type & hi
      , idx_vector_type & pi
      , vector_type & mu
      , vector_type & gamma
      , vector_type & b
      , bool const & use_stabilization
      , bool const & use_friction     
      , bool const & use_bounce
      , bool const & use_erp
      )
    {
      size_t m = detail::evaluate_constraints(group,fps,use_stabilization,use_friction,use_bounce,use_erp);
      detail::get_jacobian_matrix      (group, m, J    );
      detail::get_dependencies_vector  (group, m, pi   );
      detail::get_factors_vector       (group, m, mu   );
      detail::get_limit_vectors        (group, m, lo,hi);
      get_inverse_mass_matrix           (group,    invM );
      detail::get_regularization_vector(group, m, gamma);
      detail::get_stabilization_vector (group, m, b    );
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_NCP_FORMULATION_H
#endif
