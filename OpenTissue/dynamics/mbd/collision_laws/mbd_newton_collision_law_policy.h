#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_NEWTON_COLLISION_LAW_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_NEWTON_COLLISION_LAW_POLICY_H
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
      * Newton's Collision Law.
      * This collision law, uses the idea of relating the contact normal
      * velocities before and after collision with a coefficient of restitution.
      *
      * It contains no friction, only normal impulses are computed.
      *
      * @param contact      A pointer to a contact point, where the collision impulse
      *                     should be applied.
      *
      * @return             The collision impulse that should be applied to object B, object A
      *                     should be applied by an equal and opposite impulse.
      */
      template<typename contact_type>
      typename contact_type::vector3_type 
        compute_newton_impulse(contact_type const * contact)
      {
        typedef typename contact_type::body_type            body_type;
        typedef typename contact_type::material_type        material_type;
        typedef typename contact_type::real_type            real_type;
        typedef typename contact_type::vector3_type         vector3_type;
        typedef typename body_type::matrix3x3_type          matrix3x3_type;
        typedef typename body_type::value_traits            value_traits;

        //---  u_n_after = u_n_before+ N^T K N j_n
        //---  u_n_after = - eps u_n_before
        //---  =>
        //---  - eps u_n_before - u_n_before = N^T K N j_n
        //---  - (1+eps) u_n_before / N^T K N =  j_n
        material_type * material = contact->m_material;
        body_type * A = contact->get_body_A();
        body_type * B = contact->get_body_B();
        real_type e_n = material->normal_restitution();
        vector3_type v_a,v_b,w_a,w_b,r_a,r_b;
        A->get_velocity(v_a);
        A->get_spin(w_a);
        r_a = contact->m_rA;
        B->get_velocity(v_b);
        B->get_spin(w_b);
        r_b = contact->m_rB;
        vector3_type u = mbd::compute_relative_contact_velocity(v_a,w_a,r_a,v_b,w_b,r_b);
        real_type u_before = u * contact->m_n;
        if(u_before>=value_traits::zero())
          return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
        real_type inv_m_a = A->get_inverse_mass();
        real_type inv_m_b = B->get_inverse_mass();
        matrix3x3_type invI_a,invI_b;
        A->get_inverse_inertia_wcs(invI_a);
        B->get_inverse_inertia_wcs(invI_b);
        matrix3x3_type K = mbd::compute_collision_matrix(inv_m_a,invI_a,r_a,inv_m_b,invI_b,r_b);
        vector3_type J = K * contact->m_n;
        real_type nKn = contact->m_n * J;
        real_type minus_1_en_u_before = -(1.+e_n)*u_before;
        J = contact->m_n*(minus_1_en_u_before/nKn);

        return J;
      }

      class NewtonCollisionLawPolicy
      {
      public:
        template<typename contact_type>
        typename contact_type::vector3_type compute_impulse(contact_type const * contact) const
        {
          return compute_newton_impulse(contact);
        }
      };

    } //End of namespace collision_laws
  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_NEWTON_COLLISION_LAW_POLICY_H
#endif 
