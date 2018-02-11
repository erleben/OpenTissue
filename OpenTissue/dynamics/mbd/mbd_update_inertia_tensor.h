#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_UPDATE_INERTIA_TENSOR_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_UPDATE_INERTIA_TENSOR_H
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
    /**
    * Inertia Update Method.
    * Computes, W = R B R^T. This method have been optimized to
    * avoid having to transpose the orientation matrix and exploit
    * symmetry of the inertia tensor.
    *
    * Note: Further optimization may be possible by exploiting common sub-terms.
    *
    * @param R   The orientation as a rotation matrix.
    * @param B   An inertia tensor entity in body frame
    * @param W   The inertia tensor entity in the corresponding world frame.
    */
    template<typename matrix3x3_type>
    void update_inertia_tensor(matrix3x3_type const & R,matrix3x3_type const & B,matrix3x3_type & W)
    {
      //--- KE 30-08-2004: The formulas below was computed using Matlab, that is
      //---
      //--- syms R00 R01 R02  R10 R11 R12 R20 R21 R22 real;
      //--- syms I00 I01 I02  I10 I11 I12 I20 I21 I22 real;
      //--- I = [ I00, I01 , I02; I01, I11, I12; I02, I12, I22]
      //--- R = [ R00, R01 , R02; R10, R11, R12; R20, R21, R22]
      //--- W = R*I*R'
      //--- simplify(W)
      //---
      W(0,0) = R(0,0)*R(0,0)*B(0,0) + 2*R(0,0)*R(0,1)*B(0,1)+2*R(0,0)*R(0,2)*B(0,2)+R(0,1)*R(0,1)*B(1,1)+2*R(0,1)*R(0,2)*B(1,2)+R(0,2)*R(0,2)*B(2,2);
      W(1,1) = R(1,0)*R(1,0)*B(0,0)+2*R(1,0)*R(1,1)*B(0,1)+2*R(1,0)*R(1,2)*B(0,2)+R(1,1)*R(1,1)*B(1,1)+2*R(1,1)*R(1,2)*B(1,2)+R(1,2)*R(1,2)*B(2,2);
      W(2,2) = R(2,0)*R(2,0)*B(0,0)+2*R(2,0)*R(2,1)*B(0,1)+2*R(2,0)*R(2,2)*B(0,2)+R(2,1)*R(2,1)*B(1,1)+2*R(2,1)*R(2,2)*B(1,2)+R(2,2)*R(2,2)*B(2,2);
      W(1,0) = W(0,1) = R(1,0)*R(0,0)*B(0,0)+R(1,0)*R(0,1)*B(0,1)+R(1,0)*R(0,2)*B(0,2)+R(1,1)*R(0,0)*B(0,1)+R(1,1)*R(0,1)*B(1,1)+R(1,1)*R(0,2)*B(1,2)+R(1,2)*R(0,0)*B(0,2)+R(1,2)*R(0,1)*B(1,2)+R(1,2)*R(0,2)*B(2,2);
      W(2,0) = W(0,2) = R(2,0)*R(0,0)*B(0,0)+R(2,0)*R(0,1)*B(0,1)+R(2,0)*R(0,2)*B(0,2)+R(2,1)*R(0,0)*B(0,1)+R(2,1)*R(0,1)*B(1,1)+R(2,1)*R(0,2)*B(1,2)+R(2,2)*R(0,0)*B(0,2)+R(2,2)*R(0,1)*B(1,2)+R(2,2)*R(0,2)*B(2,2);
      W(2,1) = W(1,2) = R(1,0)*R(2,0)*B(0,0)+R(1,0)*R(2,1)*B(0,1)+R(1,0)*R(2,2)*B(0,2)+R(1,1)*R(2,0)*B(0,1)+R(1,1)*R(2,1)*B(1,1)+R(1,1)*R(2,2)*B(1,2)+R(1,2)*R(2,0)*B(0,2)+R(1,2)*R(2,1)*B(1,2)+R(1,2)*R(2,2)*B(2,2);
    }

  } //--- End of namespace mbd
} //--- End of namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_UPDATE_INERTIA_TENSOR_H
#endif
