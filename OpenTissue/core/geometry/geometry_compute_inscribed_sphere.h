#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_INSCRIBED_SPHERE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_INSCRIBED_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_invert4x4.h>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute inscribed sphere of tetrahedron.
    *
    * Note this method assumes that the tetrahedron is right-handed (i.e. has positive volume).
    * If one wants to make sure that the method is invoked probably then one can use the following code
    *
    *      Tetrahedron< ... > tetrahedron(p0,p1,p2,p3);
    *
    *      if(tetrahedron.volume()>0)
    *        compute_tetrahedron_inscribed_sphere(p0,p1,p2,p3,center,radius);
    *      else
    *        compute_tetrahedron_inscribed_sphere(p1,p0,p2,p3,center,radius);
    *
    * Tetrahedra with negative volume can occur durring simulation, if deformations
    * is too large then a tetrahedron may get inverted. Implying negative volume.
    *
    *
    * @param pi   The first point of the tetrahedron.
    * @param pj   The second point of the tetrahedron.
    * @param pk   The third point of the tetrahedron.
    * @param pm   This fourth point of the tetrahedron. Note this point should lie
    *             on the front-side of the triangle given by points pi, pj, and pk.
    *
    * @param center   Upon return this argument holds the computed center.
    * @param radius   Upon return this argument holds the computed radius.
    */
    template<typename vector3_type, typename real_type>
    void compute_tetrahedron_inscribed_sphere(
      vector3_type const & pi
      , vector3_type const & pj
      , vector3_type const & pk
      , vector3_type const & pm
      , vector3_type       & center
      , real_type          & radius
      )
    {
      //---
      //--- The center of the inscreibed sphere is equidistant to each of the face
      //--- planes of the tetrahedron, and the distance is the radius of the inscribed
      //--- sphere. Let the center and radius of the inscreibed sphere be denoted by c and r.
      //---
      //---
      //--- Let the plane consisting of points j,k, and m be denoted by i then 
      //---
      //---    n_i * c - w_i = r
      //---
      //--- where
      //---
      //---    n_i = \frac{(p_m-p_j)\times(p_k-p_j)}{\norm{(p_m-p_j)\times(p_k-p_j)}}
      //---    w_i = n_i \cdot p_j
      //---
      //--- Similar we can setup three equations for the planes j,k and m (by permuting).
      //--- This results in the system of linear equations:
      //---
      //---   n_{m,x}  c_x + n_{m,y}  c_y + n_{m,z}  c_z - w_m = r
      //---   n_{i,x}  c_x + n_{i,y}  c_y + n_{i,z}  c_z - w_i = r
      //---   n_{j,x}  c_x + n_{j,y}  c_y + n_{j,z}  c_z - w_j = r
      //---   n_{k,x}  c_x + n_{k,y}  c_y + n_{k,z}  c_z - w_k = r
      //--- 
      //--- Or in as a matrix equation
      //---
      //--- | n_{m,x}   n_{m,y}   n_{m,z}  -1 | | c_x | = | w_m |
      //--- | n_{i,x}   n_{i,y}   n_{i,z}  -1 | | c_y | = | w_i |
      //--- | n_{j,x}   n_{j,y}   n_{j,z}  -1 | | c_z | = | w_j |
      //--- | n_{k,x}   n_{k,y}   n_{k,z}  -1 | |  r  | = | w_k |
      //--- 
      //---            A                           x    = b
      //---
      //--- This is four equations with four unknowns and can be solved by inversion of the A-matrix.
      //---
      vector3_type nm = unit( (pj-pi)%(pk-pi) );
      vector3_type ni = unit( (pm-pj)%(pk-pj) );
      vector3_type nj = unit( (pm-pk)%(pi-pk) );
      vector3_type nk = unit( (pm-pi)%(pj-pi) );

      real_type wm = nm*pi;
      real_type wi = ni*pj;
      real_type wj = nj*pk;
      real_type wk = nk*pi;

      real_type M[4][4];

      M[0][0] = nm(0); M[0][1] = nm(1); M[0][2] = nm(2); M[0][3] = -1;
      M[1][0] = ni(0); M[1][1] = ni(1); M[1][2] = ni(2); M[1][3] = -1;
      M[2][0] = nj(0); M[2][1] = nj(1); M[2][2] = nj(2); M[2][3] = -1;
      M[3][0] = nk(0); M[3][1] = nk(1); M[3][2] = nk(2); M[3][3] = -1;

      math::invert4x4(M);

      center(0) = M[0][0]*wm + M[0][1]*wi + M[0][2]*wj + M[0][3]*wk;
      center(1) = M[1][0]*wm + M[1][1]*wi + M[1][2]*wj + M[1][3]*wk;
      center(2) = M[2][0]*wm + M[2][1]*wi + M[2][2]*wj + M[2][3]*wk;
      radius    = M[3][0]*wm + M[3][1]*wi + M[3][2]*wj + M[3][3]*wk;
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_TETRAHEDRON_INSCRIBED_SPHERE_H
#endif
