#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_OBB_OBB_SAT_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_OBB_OBB_SAT_H
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
  namespace intersect
  {

    /**
    * OBB Separation Axis Test (SAT)
    *
    *
    * Typical usage:
    *
    *   CoordSys<real_type>  Awcs = ....
    *   CoordSys<real_type>  Bwcs = ....
    *   CoordSys<real_type>  B2A = model_update(Awcs,Bwcs);
    *   OBB<real_type> A = ....
    *   OBB<real_type> B = ....
    *   OBB<real_type> BinA = B;
    *   BinA.xform(B2A);
    *
    *   bool separation = obb_sat(A,BinA);
    *
    * @param A       A oriented bounding box.
    * @param BinA    Another oriented bounding box given in OBB A's local model frame.
    * @return        if the return value is true an separation axis was found otherwise
    *                the two OBBs are not separated.
    */
    template<typename obb_type>
    bool obb_obb_sat(obb_type const & A, obb_type const & BinA )
    {
      using std::fabs;

      typedef typename obb_type::real_type        real_type;
      typedef typename obb_type::vector3_type     vector3_type;
      typedef typename obb_type::matrix3x3_type   matrix3x3_type;

      matrix3x3_type const & R_ba = BinA.orientation();
      vector3_type   const & p_ba = BinA.center();
      vector3_type   const & b    = BinA.eps();
      vector3_type   const & a    = A.eps();

      static real_type threshold = math::working_precision<real_type>();
      real_type Q00 = fabs(R_ba(0,0))+threshold;
      real_type Q01 = fabs(R_ba(0,1))+threshold;
      real_type Q02 = fabs(R_ba(0,2))+threshold;
      real_type Q10 = fabs(R_ba(1,0))+threshold;
      real_type Q11 = fabs(R_ba(1,1))+threshold;
      real_type Q12 = fabs(R_ba(1,2))+threshold;
      real_type Q20 = fabs(R_ba(2,0))+threshold;
      real_type Q21 = fabs(R_ba(2,1))+threshold;
      real_type Q22 = fabs(R_ba(2,2))+threshold;

#define TST(expr1,expr2) if (fabs(expr1) - (expr2) > 0) return true; 
      TST(  p_ba(0),                                                     a(0) + b(0)*Q00 + b(1)*Q10 + b(2)*Q20    );
      TST(  p_ba(1),                                                     a(1) + b(0)*Q01 + b(1)*Q11 + b(2)*Q21    );
      TST(  p_ba(2),                                                     a(2) + b(0)*Q02 + b(1)*Q12 + b(2)*Q22    );
      TST(  p_ba(0)*R_ba(0,0) + p_ba(1)*R_ba(1,0) + p_ba(2)*R_ba(2,0),   b(0) + a(0)*Q00 + a(1)*Q10 + a(2)*Q20    );
      TST(  p_ba(0)*R_ba(0,1) + p_ba(1)*R_ba(1,1) + p_ba(2)*R_ba(2,1),   b(1) + a(0)*Q01 + a(1)*Q11 + a(2)*Q21    );
      TST(  p_ba(0)*R_ba(0,2) + p_ba(1)*R_ba(1,2) + p_ba(2)*R_ba(2,2),   b(2) + a(0)*Q02 + a(1)*Q12 + a(2)*Q22    );
      TST(  p_ba(2)*R_ba(1,0) - p_ba(1)*R_ba(2,0),                       a(1)*Q20 + a(2)*Q10 + b(1)*Q02 + b(2)*Q01);
      TST(  p_ba(2)*R_ba(1,1) - p_ba(1)*R_ba(2,1),                       a(1)*Q21 + a(2)*Q11 + b(2)*Q00 + b(0)*Q02);
      TST(  p_ba(2)*R_ba(1,2) - p_ba(1)*R_ba(2,2),                       a(1)*Q22 + a(2)*Q12 + b(0)*Q01 + b(1)*Q00);
      TST(  p_ba(0)*R_ba(2,0) - p_ba(2)*R_ba(0,0),                       a(2)*Q00 + a(0)*Q20 + b(1)*Q12 + b(2)*Q11);
      TST(  p_ba(0)*R_ba(2,1) - p_ba(2)*R_ba(0,1),                       a(2)*Q01 + a(0)*Q21 + b(2)*Q10 + b(0)*Q12);
      TST(  p_ba(0)*R_ba(2,2) - p_ba(2)*R_ba(0,2),                       a(2)*Q02 + a(0)*Q22 + b(0)*Q11 + b(1)*Q10);
      TST(  p_ba(1)*R_ba(0,0) - p_ba(0)*R_ba(1,0),                       a(0)*Q10 + a(1)*Q00 + b(1)*Q22 + b(2)*Q21);
      TST(  p_ba(1)*R_ba(0,1) - p_ba(0)*R_ba(1,1),                       a(0)*Q11 + a(1)*Q01 + b(2)*Q20 + b(0)*Q22);
      TST(  p_ba(1)*R_ba(0,2) - p_ba(0)*R_ba(1,2),                       a(0)*Q12 + a(1)*Q02 + b(0)*Q21 + b(1)*Q20);
#undef TST
      return false;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_OBB_OBB_SAT_H
#endif
