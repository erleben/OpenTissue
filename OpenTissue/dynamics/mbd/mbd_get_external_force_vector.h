#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_EXTERNAL_FORCE_VECTOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_EXTERNAL_FORCE_VECTOR_H
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
    * Extract external Forces and Torques.
    * Incl. velocity dependent terms acting on bodies.
    *
    * The method implicitly assume that caller has restored position and
    * velocities at the point in time where he/she wants to compute the
    * external forces and torques.
    *
    * @param begin   An iterator to the first body in the sequence.
    * @param end     An iterator to the one past the last body in the sequence.
    * @param f_ext                 Upon return this array holds the extracted values.
    * @param compute_velocity_forces       Boolean flag indicating wheter velocity dependent forces should be added
    *                              to the extracted vector. Default value is true meaning that velocity
    *                              forces is by default added to the external force vector.
    */
    template<typename indirect_body_iterator,typename vector_type>
    void get_external_force_vector(indirect_body_iterator begin, indirect_body_iterator end, vector_type & f_ext,  bool compute_velocity_forces)
    {
      typedef typename indirect_body_iterator::value_type     body_type;
      typedef typename body_type::math_policy                 math_policy;
      typedef typename body_type::value_traits                value_traits;
      typedef typename body_type::vector3_type                vector3_type;
      typedef typename body_type::matrix3x3_type              matrix3x3_type;
      typedef typename body_type::quaternion_type             quaternion_type;
      typedef typename vector_type::size_type                 size_type;

      vector3_type velocity_forces;
      vector3_type force;
      vector3_type torque;
      vector3_type omega;
      matrix3x3_type invI;

      size_type n = std::distance(begin,end);
      
      math_policy::resize( f_ext, 6*n);      

      typename vector_type::iterator fval = f_ext.begin();

      for(indirect_body_iterator body = begin;body!=end;++body)
      {
        assert(body->is_active() || !"get_external_force_vector(): body was not active");

        if(body->is_fixed() || body->is_scripted())
        {
          *fval++ = value_traits::zero();
          *fval++ = value_traits::zero();
          *fval++ = value_traits::zero();
          *fval++ = value_traits::zero();
          *fval++ = value_traits::zero();
          *fval++ = value_traits::zero();
          continue;
        }
        body->compute_forces_and_torques(force,torque);
        assert(is_number(force(0)) || !"get_external_force_vector(): non number encountered");
        assert(is_number(force(1)) || !"get_external_force_vector(): non number encountered");
        assert(is_number(force(2)) || !"get_external_force_vector(): non number encountered");
        assert(is_number(torque(0)) || !"get_external_force_vector(): non number encountered");
        assert(is_number(torque(1)) || !"get_external_force_vector(): non number encountered");
        assert(is_number(torque(2)) || !"get_external_force_vector(): non number encountered");

        *fval++ = force(0);
        *fval++ = force(1);
        *fval++ = force(2);

        velocity_forces.clear();
        if(compute_velocity_forces)
        {
          body->get_spin(omega);
          assert(is_number(omega(0))|| !"get_external_force_vector(): non number encountered");
          assert(is_number(omega(1))|| !"get_external_force_vector(): non number encountered");
          assert(is_number(omega(2))|| !"get_external_force_vector(): non number encountered");
          body->get_inertia_wcs(invI);
          velocity_forces = cross( omega , invI*omega);
          assert(is_number(velocity_forces(0)) || !"get_external_force_vector(): non number encountered");
          assert(is_number(velocity_forces(1)) || !"get_external_force_vector(): non number encountered");
          assert(is_number(velocity_forces(2)) || !"get_external_force_vector(): non number encountered");
          *fval++ = torque(0) - velocity_forces(0);
          *fval++ = torque(1) - velocity_forces(1);
          *fval++ = torque(2) - velocity_forces(2);
          continue;
        }
        *fval++ = torque(0);
        *fval++ = torque(1);
        *fval++ = torque(2);
      }
    }

    template<typename group_type,typename vector_type>
    void get_external_force_vector(group_type const & group, vector_type & f_ext, bool compute_velocity_forces )
    {
      get_external_force_vector(group.body_begin(),group.body_end(),f_ext,compute_velocity_forces);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_GET_EXTERNAL_FORCE_VECTOR_H
#endif
