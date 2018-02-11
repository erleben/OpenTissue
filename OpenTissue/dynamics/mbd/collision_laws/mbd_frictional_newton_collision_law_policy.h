#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_FRICTIONAL_NEWTON_COLLISION_LAW_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_FRICTIONAL_NEWTON_COLLISION_LAW_POLICY_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/mbd_compute_collision_matrix.h>
#include <OpenTissue/dynamics/mbd/mbd_compute_relative_contact_velocity.h>

namespace OpenTissue
{
  namespace mbd
  {
    namespace collision_laws
    {

      /**
      * Newton Impact with Friction.
      *
      * The implementation herein is similar to the algebraic collision law desribed in:
      *
      *            "Nonconvex Rigid Bodies with Stacking" by Guendelman, Bridson, and Fedkiw,
      *            SIGGRAPH 2003, ACM TOG 22, 871-878 (2003).
      *
      * @param contact      A pointer to a contact point, where the collision impulse
      *                     should be applied.
      *
      * @return             The collision impulse that should be applied to object B, object A
      *                     should be applied by an equal and opposite impulse.
      */
      template<typename contact_type>
      typename contact_type::vector3_type 
        compute_frictional_newton_impulse(contact_type const * contact)
      {
        typedef typename contact_type::body_type            body_type;
        typedef typename contact_type::material_type        material_type;
        typedef typename contact_type::real_type            real_type;
        typedef typename contact_type::vector3_type         vector3_type;
        typedef typename body_type::matrix3x3_type          matrix3x3_type;
        typedef typename body_type::value_traits            value_traits;

        material_type * material = contact->m_material;
        body_type * A = contact->get_body_A();
        body_type * B = contact->get_body_B();
        real_type e_n = material->normal_restitution();
        real_type mu  = material->get_isotropic_friction_coefficient();
        vector3_type v_a,v_b,w_a,w_b,r_a,r_b;
        A->get_velocity(v_a);
        A->get_spin(w_a);
        r_a = contact->m_rA;
        B->get_velocity(v_b);
        B->get_spin(w_b);
        r_b = contact->m_rB;
        real_type inv_m_a = A->get_inverse_mass();
        real_type inv_m_b = B->get_inverse_mass();
        matrix3x3_type invI_a,invI_b;
        A->get_inverse_inertia_wcs(invI_a);
        B->get_inverse_inertia_wcs(invI_b);
        matrix3x3_type K = mbd::compute_collision_matrix(inv_m_a,invI_a,r_a,inv_m_b,invI_b,r_b);
        matrix3x3_type invK = inverse(K);

        //--- Asume sticking, that is:
        //---
        //--- u_t_after = 0
        //--- u_n_after = -eps u_n_before N
        //---
        //--- Now from the impulse momentum law
        //---
        //---  u_after = u_before + K J
        //---
        //--- compute
        //---
        //--- J = inv(K)(u_after- u_before )
        //---
        vector3_type u_before = compute_relative_contact_velocity(v_a,w_a,r_a,v_b,w_b,r_b);
        real_type u_n_before = contact->m_n * u_before;
        if(u_n_before>=0)
          return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
        vector3_type u_after = contact->m_n*(-e_n*u_n_before);
        vector3_type J = invK *(u_after-u_before);
        //---
        //---  Compute the impulse component in the tangent plane
        //---
        //---    J_N = (J.N)N
        //---    J_T = J-J_N
        //---
        //--- if
        //---
        //---      |J_T|  <= mu |J_N|
        //---
        //--- then J is in the friction cone and we use it. Otherwise we need to consider sliding friction.
        real_type j_n = contact->m_n*J;
        vector3_type J_N = contact->m_n*j_n;
        vector3_type J_T = J - J_N;

        real_type norm_j_t = sqrt(J_T*J_T);
        real_type friction_limit = std::fabs(mu*j_n);
        if(mu==0 || norm_j_t < friction_limit)
        {
          return J;
        }
        //--- Determine directin of sliding
        //---
        //---    T = u_rel,t/|u_rel,t|
        //---
        //---
        vector3_type u_t_before = u_before - u_n_before*contact->m_n;
        real_type norm_u_t_before = sqrt(u_t_before*u_t_before);
        if(!norm_u_t_before)
        {
          //--- We are in trouble, easy solution project J back onto friction cone
          //--- and return the projected impulse
          if(norm_j_t)
          {
            J = J_N + J_T *(friction_limit/norm_j_t);
            return J;
          }
          return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
        }
        vector3_type T   = u_t_before / norm_u_t_before;
        //--- Define impulse as
        //---
        //---  J = j_n N - mu j_n T
        //---
        //--- Take the impulse-momentum law and dot it with the normal
        //---
        //---   u_n_after = u_n_before + N^T K J
        //---
        //--- Use Newton's Impact Law
        //---
        //---  - eps u_n_before   = u_n_before + N^T K J
        //---
        //--- Insert definition of J
        //---
        //---  - eps u_n_before   = u_n_before + N^T K (j_n N - mu j_n T)
        //---
        //--- Solve for j_n
        //---
        //---  - (1+ eps) u_n_before =  N^T K (j_n N - mu j_n T)
        //---  - (1+ eps) u_n_before =  N^T K N j_n  - N^T K T mu j_n
        //---  - (1+ eps) u_n_before =  N^T K (N - T mu) j_n
        //---  j_n = (- (1+ eps) u_n_before)/  N^T K (N - T mu)
        //---
        j_n = (-(1.+e_n)*u_n_before)/  (contact->m_n * (K*(contact->m_n- T*mu)));
        J = contact->m_n*j_n - T*mu* j_n;
        return J;
      }

      class FrictionalNewtonCollisionLawPolicy
      {
      public:
        template<typename contact_type>
        typename contact_type::vector3_type compute_impulse(contact_type const * contact) const
        {
          return compute_frictional_newton_impulse(contact);
        }
      };

    } //End of namespace collision_laws
  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_FRICTIONAL_NEWTON_COLLISION_LAW_POLICY_H
#endif 
