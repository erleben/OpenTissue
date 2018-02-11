#ifndef	OPENTISSUE_DYNAMICS_MBD_UTIL_COMPUTE_COLLISION_MATRIX_H
#define	OPENTISSUE_DYNAMICS_MBD_UTIL_COMPUTE_COLLISION_MATRIX_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>


namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Compute Collision Matrix.
    * This method computes the collision matrix, K, which is part
    * of the impulse-momemtum relation between two bodies in a
    * single point collision.
    *
    *    K = (1/Ma + 1/Mb) Identity  - ( star(ra) Ia^-1 star(ra)  +  star(rb) Ib^-1 star(rb))
    *
    * Where star() is the cross product matrix and Identity is a
    * diagonal matrix of ones.
    *
    * The computations have been optimized to exploit symmetry and zero pattern's as
    * much as possible. It is still possible to optimize further by exploiting
    * common sub-terms in the computations.
    *
    * @param m_a  Inverse mass of object A.
    * @param I_a  Inverse Intertia Tensor of object A.
    * @param r_a  The arm from center of mas of object A to the point of contact.
    * @param m_b  Inverse mass of object B.
    * @param I_b  Inverse Intertia Tensor of object B.
    * @param r_b  The arm from center of mas of object B to the point of contact.
    *
    * @return     The collision matrix K.
    */
    template<typename real_type,typename vector3_type,typename matrix3x3_type>
    matrix3x3_type compute_collision_matrix(
      real_type const & m_a
      , matrix3x3_type const & I_a
      , vector3_type const & r_a
      , real_type const & m_b
      , matrix3x3_type const & I_b
      , vector3_type const & r_b
      )
    {
      matrix3x3_type K;
      real_type K00 =  r_a(2)*I_a(1,1)*r_a(2) - 2*r_a(2)*I_a(1,2)*r_a(1) + r_a(1)*I_a(2,2)*r_a(1);
      real_type K01 =  - r_a(2)*I_a(0,1)*r_a(2) + r_a(1)*I_a(0,2)*r_a(2) + r_a(2)*I_a(1,2)*r_a(0) - r_a(1)*I_a(2,2)*r_a(0);
      real_type K02 =  r_a(2)*I_a(0,1)*r_a(1) - r_a(1)*I_a(0,2)*r_a(1) - r_a(2)*I_a(1,1)*r_a(0) + r_a(1)*I_a(1,2)*r_a(0);
      real_type K11 =  r_a(2)*I_a(0,0)*r_a(2) - 2*r_a(0)*I_a(0,2)*r_a(2) + r_a(0)*I_a(2,2)*r_a(0);
      real_type K12 =  - r_a(2)*I_a(0,0)*r_a(1) + r_a(2)*I_a(0,1)*r_a(0) + r_a(0)*I_a(0,2)*r_a(1) - r_a(0)*I_a(1,2)*r_a(0);
      real_type K22 =  r_a(1)*I_a(0,0)*r_a(1) - 2*r_a(0)*I_a(0,1)*r_a(1) + r_a(0)*I_a(1,1)*r_a(0);
      K(0,0) = m_a + K00;
      K(0,1) = K01;
      K(0,2) = K02;
      K(1,0) = K01;
      K(1,1) = m_a + K11;
      K(1,2) = K12;
      K(2,0) = K02;
      K(2,1) = K12;
      K(2,2) = m_a + K22;
      K00 =  r_b(2)*I_b(1,1)*r_b(2) - 2*r_b(2)*I_b(1,2)*r_b(1) + r_b(1)*I_b(2,2)*r_b(1);
      K01 =  - r_b(2)*I_b(0,1)*r_b(2) + r_b(1)*I_b(0,2)*r_b(2) + r_b(2)*I_b(1,2)*r_b(0) - r_b(1)*I_b(2,2)*r_b(0);
      K02 =  r_b(2)*I_b(0,1)*r_b(1) - r_b(1)*I_b(0,2)*r_b(1) - r_b(2)*I_b(1,1)*r_b(0) + r_b(1)*I_b(1,2)*r_b(0);
      K11 =  r_b(2)*I_b(0,0)*r_b(2) - 2*r_b(0)*I_b(0,2)*r_b(2) + r_b(0)*I_b(2,2)*r_b(0);
      K12 =  - r_b(2)*I_b(0,0)*r_b(1) + r_b(2)*I_b(0,1)*r_b(0) + r_b(0)*I_b(0,2)*r_b(1) - r_b(0)*I_b(1,2)*r_b(0);
      K22 =  r_b(1)*I_b(0,0)*r_b(1) - 2*r_b(0)*I_b(0,1)*r_b(1) + r_b(0)*I_b(1,1)*r_b(0);
      K(0,0) += m_b + K00;
      K(0,1) += K01;
      K(0,2) += K02;
      K(1,0) += K01;
      K(1,1) += m_b + K11;
      K(1,2) += K12;
      K(2,0) += K02;
      K(2,1) += K12;
      K(2,2) += m_b + K22;
      return K;
    }

  } //End of namespace mbd

} //End of namespace OpenTissue

#endif // OPENTISSUE_DYNAMICS_MBD_UTIL_COMPUTE_COLLISION_MATRIX_H
