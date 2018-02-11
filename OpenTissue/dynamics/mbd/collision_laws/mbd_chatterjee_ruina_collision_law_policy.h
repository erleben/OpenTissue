#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_CHATTERJEE_RUINA_COLLISION_LAW_POLICY_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_CHATTERJEE_RUINA_COLLISION_LAW_POLICY_H
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
      * Non-increasing Energy Collision Law.
      * This is an algebraic collision law presented in:
      *
      *    "A new algebraic rigid body collision law based on impulse space considerations" by Chatterjee and Ruina.
      *    Journal of Applied Mechanics, Vol 65, #4, 939-951, Dec 1998.
      *
      * Can be downloaded from: http://www.tam.cornell.edu/~ruina/hplab/collision_papers.html
      *
      *
      * @param contact      A pointer to a contact point, where the collision impulse
      *                     should be applied.
      *
      * @return             The collision impulse that should be applied to object B, object A
      *                     should be applied by an equal and opposite impulse.
      */
      template<typename contact_type>
      typename contact_type::vector3_type 
        compute_chatterjee_ruina_impulse(contact_type const * contact)
      {
        typedef typename contact_type::body_type            body_type;
        typedef typename contact_type::material_type        material_type;
        typedef typename contact_type::real_type            real_type;
        typedef typename contact_type::vector3_type         vector3_type;
        typedef typename body_type::matrix3x3_type          matrix3x3_type;
        typedef typename body_type::value_traits            value_traits;

        using std::sqrt;

        material_type * material = contact->m_material;
        body_type const * A = contact->get_body_A();
        body_type const * B = contact->get_body_B();
        real_type e_n = material->normal_restitution();
        real_type e_t = material->tangential_restitution();
        real_type mu  = material->get_isotropic_friction_coefficient();
        vector3_type v_a,v_b,w_a,w_b,r_a,r_b;
        A->get_velocity(v_a);
        A->get_spin(w_a);
        r_a = contact->m_rA;
        B->get_velocity(v_b);
        B->get_spin(w_b);
        r_b = contact->m_rB;
        vector3_type u = mbd::compute_relative_contact_velocity(v_a,w_a,r_a,v_b,w_b,r_b);
        real_type u_n =  contact->m_n * u;
        if(u_n>=0)
          return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
        real_type inv_m_a = A->get_inverse_mass();
        real_type inv_m_b = B->get_inverse_mass();
        matrix3x3_type invI_a,invI_b;
        A->get_inverse_inertia_wcs(invI_a);
        B->get_inverse_inertia_wcs(invI_b);
        matrix3x3_type K = mbd::compute_collision_matrix(inv_m_a,invI_a,r_a,inv_m_b,invI_b,r_b);
        matrix3x3_type invK = inverse(K);
        //--- Perfectly plastic and fritionless impulse: J_I = \left( \frac{ - \vec n \cdot \vec u_i }{ \vec n^T K \vec n } \right) \vec n
        //--- This impulse brings the normal velocity to zero.
        vector3_type JI = K * contact->m_n;
        real_type nKn = contact->m_n * JI;
        JI = contact->m_n*(-u_n/nKn);
        //--- Perfectly plastic and sticking impulse: J_{II} = - K^-1 \vec u_i
        vector3_type JII = -invK*u;
        //--- Compute candidate impulse
        //--- \vec \hat J = \left( 1 + e \right) \vec J_I + \left( 1 + et \right) \left( \vec J_{II}-\vec J_I \right)
        vector3_type Jhat = JI*( value_traits::one() + e_n) + (JII - JI)*( value_traits::one() + e_t);
        //--- Friction cone test
        real_type Pn = Jhat * contact->m_n;
        vector3_type tmp = Jhat - contact->m_n*Pn;
        real_type norm = sqrt(tmp*tmp);
        if((mu>0) && (norm > mu*Pn))
        {
          //--- kappa =
          //---        \frac{
          //---           \mu \left( 1+e \right) \vec n \cdot \vec J_I
          //---        }{
          //---           ||
          //---                \vec J_{II}
          //---                   -
          //---                \left( \vec n \cdot \vec J_{II} \right) \vec n
          //---           ||
          //---            -
          //---           \mu \vec n \cdot \left( \vec J_{II} - \vec J_I \right)
          //---        }
          //---
          vector3_type tmp = JII - contact->m_n*(JII*contact->m_n);
          real_type w = sqrt(tmp*tmp);
          tmp = JII - JI;
          w -= mu*contact->m_n*tmp;
          real_type kappa = mu*(1. + e_n)*(contact->m_n*JI) / w;
          //--- \vec J = \left( 1 + e \right) \vec J_I + \kappa \left( \vec J_{II}-\vec J_I \right)
          vector3_type J = JI*(value_traits::one() + e_n) + (JII-JI)*kappa;
          return J;
        }
        else
        {
          //--- kappa = (1+et) =>
          //--- \vec J = \left( 1 + e \right) \vec J_I + \kappa \left( \vec J_{II}-\vec J_I \right)
          return Jhat;
        }
      }

      class ChatterjeeRuinaCollisionLawPolicy
      {
      public:

        template<typename contact_type>
        typename contact_type::vector3_type compute_impulse(contact_type const * contact) const
        {
          return compute_chatterjee_ruina_impulse(contact);
        }

      };

    } //End of namespace collision_laws
  } //End of namespace mbd
} //End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_COLLISION_LAWS_MBD_CHATTERJEE_RUINA_COLLISION_LAW_POLICY_H
#endif 
