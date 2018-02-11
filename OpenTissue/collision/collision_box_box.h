#ifndef OPENTISSUE_COLLISION_COLLISION_BOX_BOX_H
#define OPENTISSUE_COLLISION_COLLISION_BOX_BOX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_coordsys.h>
#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/geometry/geometry_compute_closest_points_line_line.h>
#include <OpenTissue/collision/intersect/intersect_rect_quad.h>

namespace OpenTissue
{
  namespace collision
  {

    /**
    * Box Box Collision Test.
    *
    * The collision test is based on a separation axis test, which works great for
    * detecting whether there is a collision or not.
    *
    * Following this a separation axis is chosen to as the contact normal direction, however
    * in some cases such as when penetrations are signigicant and when objects are slightly
    * unaligned, a edge-edge separation axis is chosen eventhough a face normal would have
    * been far better. This is a problem since the generated contact in these edge-edge cases
    * might not even lie inside the contact region.
    *
    * Therefor we recommend only using this method when penetrations are expected
    * to be very small.
    *
    * @param p_a       Center of box A in WCS.
    * @param p_b       Center of box B in WCS
    * @param R_a       Box A's orientation in WCS
    * @param R_b       Box B's orientation in WCS
    * @param a         Extents of box A, i.e. half edge sizes.
    * @param b         Extents of box B, i.e. half edge sizes.
    * @param envelope  The size of the collision envelope. If cloest point are separted by more than this distance then there is no contact.
    * @param p         Pointer to array of contact points, must have room for at least eight vectors.
    * @param n         Upon return this argument holds the contact normal pointing from box A towards box B.
    * @param distance  Pointer to array of separation (or penetration) distances. Must have room for at least eight values.
    *
    * @return          If contacts exist then the return value indicates the number of contacts, if no contacts exist the return valeu is zero.
    */
    template<typename real_type,typename vector3_type,typename matrix3x3_type>
    unsigned int box_box(
      vector3_type  p_a
      , matrix3x3_type R_a
      , vector3_type const & a
      , vector3_type  p_b
      , matrix3x3_type R_b
      , vector3_type const & b
      , real_type const & envelope
      , vector3_type * p
      , vector3_type & n
      , real_type * distances
      )
    {
      using std::sqrt;
      using std::fabs;
      typedef          CoordSys<real_type>             coordsys_type;
      typedef typename coordsys_type::quaternion_type  quaternion_type;
      assert(p);
      assert(distances);

      //--- First we extract and set up information about boxes, such as
      //--- centers, orientations and size
      quaternion_type Q_a = R_a;
      quaternion_type Q_b = R_b;
      coordsys_type BtoA = model_update(p_b,Q_b,p_a,Q_a);

      vector3_type p_ba = BtoA.T();      //--- center of box B in box A's model frame.
      matrix3x3_type  R_ba = BtoA.Q();   //--- Box B's orientation in box A's model frame

      //--- For convience we extract column vectors of the orientation
      //--- matrices, these will be needed to determine the contact normal.
      vector3_type i_ba,j_ba,k_ba;
      i_ba(0) = R_ba(0,0);    i_ba(1) = R_ba(1,0);    i_ba(2) = R_ba(2,0);
      j_ba(0) = R_ba(0,1);    j_ba(1) = R_ba(1,1);    j_ba(2) = R_ba(2,1);
      k_ba(0) = R_ba(0,2);    k_ba(1) = R_ba(1,2);    k_ba(2) = R_ba(2,2);

      vector3_type A[3];
      A[0](0) = R_a(0,0);   A[0](1) = R_a(1,0);   A[0](2) = R_a(2,0);
      A[1](0) = R_a(0,1);   A[1](1) = R_a(1,1);   A[1](2) = R_a(2,1);
      A[2](0) = R_a(0,2);   A[2](1) = R_a(1,2);   A[2](2) = R_a(2,2);

      vector3_type B[3];
      B[0](0) = R_b(0,0);   B[0](1) = R_b(1,0);   B[0](2) = R_b(2,0);
      B[1](0) = R_b(0,1);   B[1](1) = R_b(1,1);   B[1](2) = R_b(2,1);
      B[2](0) = R_b(0,2);   B[2](1) = R_b(1,2);   B[2](2) = R_b(2,2);

      //--- For the Separation axis test, we need the absolute matrix of
      //--- R_ba, a small threshold is added to each entry to combat
      //--- problems with numerical precision and roundoff.
      real_type eps = boost::numeric_cast<real_type>(1e-15);
      real_type Q00 = fabs(R_ba(0,0))+eps;
      real_type Q01 = fabs(R_ba(0,1))+eps;
      real_type Q02 = fabs(R_ba(0,2))+eps;
      real_type Q10 = fabs(R_ba(1,0))+eps;
      real_type Q11 = fabs(R_ba(1,1))+eps;
      real_type Q12 = fabs(R_ba(1,2))+eps;
      real_type Q20 = fabs(R_ba(2,0))+eps;
      real_type Q21 = fabs(R_ba(2,1))+eps;
      real_type Q22 = fabs(R_ba(2,2))+eps;

      //--- For the separaton axis test we need a bunch of variables to keep
      //--- information about the results of the testing.

      vector3_type normal;       //--- Upon completion of the 15 tests, this vector
      //--- holds the contact normal. For the face cases
      //--- the normal will be in WCS, for the Edge cases
      //--- the normal will be in box A's local frame.

      real_type separation  = 0; //--- This double is used to hold a measure of the
      //--- separation along an axis being tested. It is
      //--- only needed for the testing (see macro's below).

      real_type distance  = math::detail::lowest<real_type>();    //--- Upon completion this double holds a measure of
      //--- deepst penetration along the axis that determines
      //--- the contact normal.

      bool flip_normal = false;          //--- If this boolean is set to true after having performed the
      //--- 15 axis tests, then the normal-vector should be flipped.
      //--- This is because we apply the convention that the normal
      //--- is pointing from box A towards box B.

      unsigned int code = 0;             //--- Upon completion of the axis tests, this unsigned integer
      //--- holds a value between 0 and 15, if it is zero it means that
      //--- a separation axis was found, If it is non-zero it means
      //--- that a penetration was found along the axis identified by
      //--- the value stored in this variable.

      const real_type fudge_factor = real_type(1.05);  //--- This is a little numerical trick, to help out when
      //--- testing the edge-edge cases.

      real_type length = 0;             //--- This is a temporary variable used to normalize the normal
      //--- when we are testing an edge-edge case.
      //--- Macro for doing a separation axis test along a face normal

#define TST(expr1,expr2,norm,axis_code) \
  separation = fabs(expr1) - (expr2); \
  if (separation > 0) return 0; \
  if (separation > distance) { \
  distance = separation; \
  normal = norm; \
  flip_normal = ((expr1) < 0); \
  code = (axis_code); \
  }
      TST( p_ba(0), a(0) + b(0)*Q00 + b(1)*Q10 + b(2)*Q20, A[0], 1);
      TST( p_ba(1), a(1) + b(0)*Q01 + b(1)*Q11 + b(2)*Q21, A[1], 2);
      TST( p_ba(2), a(2) + b(0)*Q02 + b(1)*Q12 + b(2)*Q22, A[2], 3);
      TST( p_ba(0)*R_ba(0,0) + p_ba(1)*R_ba(1,0) + p_ba(2)*R_ba(2,0), b(0) + a(0)*Q00 + a(1)*Q10 + a(2)*Q20, B[0], 4);
      TST( p_ba(0)*R_ba(0,1) + p_ba(1)*R_ba(1,1) + p_ba(2)*R_ba(2,1), b(1) + a(0)*Q01 + a(1)*Q11 + a(2)*Q21, B[1], 5);
      TST( p_ba(0)*R_ba(0,2) + p_ba(1)*R_ba(1,2) + p_ba(2)*R_ba(2,2), b(2) + a(0)*Q02 + a(1)*Q12 + a(2)*Q22, B[2], 6);
#undef TST
      //--- Macro for doing a separation axis test along a edge-edge axis, notice that
      //--- the main difference with the face-testing above lies in how the normal
      //--- information is passed to the macro, and that a normalization is taking plane.
#define TST(expr1,expr2,nx,ny,nz,axis_code) \
  separation = fabs(expr1) - (expr2); \
  if (separation > 0) return 0; \
  length = sqrt ((nx)*(nx) + (ny)*(ny) + (nz)*(nz)); \
  if (length > 0) { \
  separation /= length; \
  if (separation*fudge_factor > distance) { \
  distance = separation; \
  normal(0) = (nx)/length; normal(1) = (ny)/length; normal(2) = (nz)/length; \
  flip_normal = ((expr1) < 0); \
  code = (axis_code); \
  } \
  }
      //--- Notice that in the face-cases we stored the normal in the WCS
      //--- coordinate frame, for the edge-edge cases we store it in
      //--- box A's local frame. This becomes important later on when we
      //--- need to convert the normal into the WCS coordinate frame.
      //---
      //--- The reason for doing this is that the cross-product that gives
      //--- the direction of the axis/normal, becomes very easy to compute,
      //--- since one of its argument only have a single non-zero entry, which
      //--- have the value one (see computation in comments below).
      //---

      real_type zero = real_type(0);
      TST(  p_ba(2)*R_ba(1,0) - p_ba(1)*R_ba(2,0),  a(1)*Q20 + a(2)*Q10 + b(1)*Q02 + b(2)*Q01,       zero, -i_ba(2),  i_ba(1),  7); //--- (1,0,0) x i_ba
      TST(  p_ba(2)*R_ba(1,1) - p_ba(1)*R_ba(2,1),  a(1)*Q21 + a(2)*Q11 + b(2)*Q00 + b(0)*Q02,       zero, -j_ba(2),  j_ba(1),  8); //--- (1,0,0) x j_ba
      TST(  p_ba(2)*R_ba(1,2) - p_ba(1)*R_ba(2,2),  a(1)*Q22 + a(2)*Q12 + b(0)*Q01 + b(1)*Q00,       zero, -k_ba(2),  k_ba(1),  9); //--- (1,0,0) x k_ba
      TST(  p_ba(0)*R_ba(2,0) - p_ba(2)*R_ba(0,0),  a(2)*Q00 + a(0)*Q20 + b(1)*Q12 + b(2)*Q11,   i_ba(2),      zero, -i_ba(0), 10); //--- (0,1,0) x i_ba
      TST(  p_ba(0)*R_ba(2,1) - p_ba(2)*R_ba(0,1),  a(2)*Q01 + a(0)*Q21 + b(2)*Q10 + b(0)*Q12,   j_ba(2),      zero, -j_ba(0), 11); //--- (0,1,0) x j_ba
      TST(  p_ba(0)*R_ba(2,2) - p_ba(2)*R_ba(0,2),  a(2)*Q02 + a(0)*Q22 + b(0)*Q11 + b(1)*Q10,   k_ba(2),      zero, -k_ba(0), 12); //--- (0,1,0) x k_ba
      TST(  p_ba(1)*R_ba(0,0) - p_ba(0)*R_ba(1,0),  a(0)*Q10 + a(1)*Q00 + b(1)*Q22 + b(2)*Q21,  -i_ba(1),  i_ba(0),      zero, 13); //--- (0,0,1) x i_ba
      TST(  p_ba(1)*R_ba(0,1) - p_ba(0)*R_ba(1,1),  a(0)*Q11 + a(1)*Q01 + b(2)*Q20 + b(0)*Q22,  -j_ba(1),  j_ba(0),      zero, 14); //--- (0,0,1) x j_ba
      TST(  p_ba(1)*R_ba(0,2) - p_ba(0)*R_ba(1,2),  a(0)*Q12 + a(1)*Q02 + b(0)*Q21 + b(1)*Q20,  -k_ba(1),  k_ba(0),      zero, 15); //--- (0,0,1) x k_ba
#undef TST
      //--- Test to see whether we found a separation axis
      if (!code)
        return 0;
      //--- Flip normal so we are sure that it is pointing towards B
      if(flip_normal)
        normal = - normal;
      //--- Test to see if we have an edge-edge contact. Notice that the
      //--- only possibility is a single contact point!!!
      if(code>6)
      {
        //--- Compute Normal in global WCS coordinates
        n = R_a * normal;

        //--- Find a point p_a on the edge from box A
        for(int i=0;i<3;++i)
          if(n*A[i] > 0)  p_a += a(i)*A[i];  else   p_a -= a(i)*A[i];
        //--- Find a point p_b on the edge from box B
        for(int i=0;i<3;++i)
          if(n*B[i] < 0)  p_b += b(i)*B[i];  else   p_b -= b(i)*B[i];
        //--- Determine the indices of two unit edge direction vectors (columns of rotation matrices in WCS)
        int columnA = ((code)-7)/3;
        int columnB = ((code)-7)%3;


        //--- Compute the edge-paramter values s and t corresponding to the closest
        //--- points between the two infinite lines parallel to the two edges.
        real_type s,t;
        OpenTissue::geometry::compute_closest_points_line_line(p_a, A[columnA], p_b, B[columnB], s, t);
        //--- Use the edge parameter values to compute the closest
        //--- points between the two edges.
        p_a += A[columnA]*s;
        p_b += B[columnB]*t;
        //--- Let the contact point be given by the mean of the closest points
        p[0] = (p_a + p_b)*.5;
        distances[0] = distance;
        return 1;
      }
      n = normal;
      //--- Make sure that we work in the frame of the box that defines the contact
      //--- normal. This coordinate frame is nice, because the contact-face is a axis
      //--- aligned rectangle. We will refer to this frame as the reference frame, and
      //--- use the letter 'r' or 'R' for it. The other box is named the incident box,
      //--- its closest face towards the reference face is called the incidient face, and
      //--- is denoted by the letter 'i' or 'I'.

      vector3_type * R_r,* R_i;          //--- Box direction vectors in WCS
      vector3_type ext_r,ext_i;          //--- Box extents
      vector3_type p_r,p_i;              //--- Box centers in WCS

      if (code <= 3)
      {
        //--- This means that box A is defining the reference frame
        R_r = A;
        R_i = B;
        p_r = p_a;
        p_i = p_b;
        ext_r = a;
        ext_i = b;
      }
      else
      {
        //--- This means that box B is defining the reference frame
        R_r = B;
        R_i = A;
        p_r = p_b;
        p_i = p_a;
        ext_r = b;
        ext_i = a;
      }
      //--- Following vectors are used for computing the corner points of the incident
      //--- face. At first they are used to determine the axis of the incidient box
      //--- pointing towards the reference box.
      //---
      //--- n_r_wcs = normal pointing away from reference frame in WCS coordinates.
      //--- n_r = normal vector of reference face dotted with axes of incident box.
      //--- abs_n_r = absolute values of n_r.
      vector3_type n_r_wcs,n_r,abs_n_r;
      if (code <= 3)
      {
        n_r_wcs = normal;

      }
      else
      {
        n_r_wcs = -normal;
      }
      //--- Each of these is a measure for how much the axis' of the incident box
      //--- points in the direction of n_r_wcs. The largest absolute value give
      //--- us the axis along which will find the closest face towards the reference
      //--- box. The sign will tell us if we should take the positive or negative
      //--- face to get the closest incident face.
      n_r(0) = R_i[0] * n_r_wcs;
      n_r(1) = R_i[1] * n_r_wcs;
      n_r(2) = R_i[2] * n_r_wcs;

      abs_n_r = fabs (n_r);
      //--- Find the largest compontent of abs_n_r: This corresponds to the normal
      //--- for the indident face. The axis number is stored in a3. the other
      //--- axis numbers of the indicent face are stored in a1,a2.
      int a1,a2,a3;
      if (abs_n_r(1) > abs_n_r(0))
      {
        if (abs_n_r(1) > abs_n_r(2))
        {
          a1 = 2;
          a2 = 0;
          a3 = 1;
        }
        else
        {
          a1 = 0;
          a2 = 1;
          a3 = 2;
        }
      }
      else
      {
        if (abs_n_r(0) > abs_n_r(2))
        {
          a1 = 1;
          a2 = 2;
          a3 = 0;
        }
        else
        {
          a1 = 0;
          a2 = 1;
          a3 = 2;
        }
      }
      //--- Now we have information enough to determine the incidient face, that means we can
      //--- compute the center point of incident face in WCS coordinates.
      vector3_type center_i_wcs;
      if (n_r(a3) < 0)
      {
        center_i_wcs = p_i + ext_i(a3) * R_i[a3];
      }
      else
      {
        center_i_wcs = p_i - ext_i(a3) * R_i[a3];
      }
      //--- Compute difference of center point of incident face with center of reference coordinates.
      vector3_type center_ir = center_i_wcs - p_r;
      //--- Find the normal and non-normal axis numbers of the reference box
      int code1,code2,code3;
      if (code <= 3)
        code3 = code-1;  //123
      else
        code3 = code-4;  //456
      if (code3==0)
      {
        code1 = 1;
        code2 = 2;
      }
      else if (code3==1)
      {
        code1 = 2;
        code2 = 0;
      }
      else
      {
        code1 = 0;
        code2 = 1;
      }

      //--- Find the four corners of the incident face, in reference-face coordinates
      real_type quad[8]; //--- 2D coordinate of incident face (stored as x,y pairs).
      //--- Project center_ri onto reference-face coordinate system (has origo
      //--- at the center of the reference face, and the two orthogonal unit vectors
      //--- denoted by R_r[code1] and R_r[code2] spaning the face-plane).
      real_type c1 = R_r[code1] * center_ir;
      real_type c2 = R_r[code2] * center_ir;
      //--- Compute the projections of the axis spanning the incidient
      //--- face, onto the axis spanning the reference face.
      //---
      //--- This will allow us to determine the coordinates in the reference-face
      //--- when we step along a direction of the incident face given by either
      //--- a1 or a2.
      real_type m11 = R_r[code1] * R_i[a1];
      real_type m12 = R_r[code1] * R_i[a2];
      real_type m21 = R_r[code2] * R_i[a1];
      real_type m22 = R_r[code2] * R_i[a2];
      {
        real_type k1 = m11 * ext_i(a1);
        real_type k2 = m21 * ext_i(a1);
        real_type k3 = m12 * ext_i(a2);
        real_type k4 = m22 * ext_i(a2);
        quad[0] = c1 - k1 - k3;
        quad[1] = c2 - k2 - k4;
        quad[2] = c1 - k1 + k3;
        quad[3] = c2 - k2 + k4;
        quad[4] = c1 + k1 + k3;
        quad[5] = c2 + k2 + k4;
        quad[6] = c1 + k1 - k3;
        quad[7] = c2 + k2 - k4;
      }
      //--- find the size of the reference face
      real_type rect[2];
      rect[0] = ext_r(code1);
      rect[1] = ext_r(code2);
      //--- Intersect the incident and reference faces
      real_type ret[16];
      int detected = OpenTissue::intersect::rect_quad(rect,quad,ret);
      if(detected<1)
        return 0;
      assert(detected<=8);
      //--- Convert the intersection points into reference-face coordinates,
      //--- and compute the contact position and depth for each point.
      real_type det1 = real_type(1.)/(m11*m22 - m12*m21);
      m11 *= det1;
      m12 *= det1;
      m21 *= det1;
      m22 *= det1;
      int cnt = 0;
      for (int j=0; j < detected; ++j)
      {
        real_type k1 =  m22*(ret[j*2]-c1) - m12*(ret[j*2+1]-c2);
        real_type k2 = -m21*(ret[j*2]-c1) + m11*(ret[j*2+1]-c2);
        //--- Intersection point in (almost) WCS.
        vector3_type point = center_ir + k1*R_i[a1] + k2*R_i[a2];
        //--- Depth of intersection point
        real_type depth = n_r_wcs*point - ext_r(code3);
        if(depth<envelope)
        {
          p[cnt] = point + p_r;
          distances[cnt] = depth;
          ++cnt;
        }
      }
      return cnt;
    }

  } //End of namespace collision

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_COLLISION_BOX_BOX_H
#endif
