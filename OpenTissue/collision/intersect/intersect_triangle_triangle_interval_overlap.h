#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_INTERVAL_OVERLAP_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_INTERVAL_OVERLAP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_precision.h>

#include <cmath>

namespace OpenTissue
{
  namespace intersect
  {
    /* Triangle/triangle intersection test routine,
    * by Tomas Moller, 1997.
    * See article "A Fast Triangle-Triangle Intersection Test",
    * Journal of Graphics Tools, 2(2), 1997
    * updated: 2001-06-20 (added line of intersection)
    *
    * int tri_tri_intersect(float V0[3],float V1[3],float V2[3],
    *                       float U0[3],float U1[3],float U2[3])
    *
    * parameters: vertices of triangle 1: V0,V1,V2
    *             vertices of triangle 2: U0,U1,U2
    * result    : returns 1 if the triangles intersect, otherwise 0
    *
    * Here is a version withouts divisions (a little faster)
    * int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
    *                      float U0[3],float U1[3],float U2[3]);
    *
    * This version computes the line of intersection as well (if they are not coplanar):
    * int tri_tri_intersect_with_isectline(float V0[3],float V1[3],float V2[3],
    *               float U0[3],float U1[3],float U2[3],int *coplanar,
    *               float isectpt1[3],float isectpt2[3]);
    * coplanar returns whether the tris are coplanar
    * isectpt1, isectpt2 are the endpoints of the line of intersection
    */

    namespace Moller
    {

      namespace detail
      {
        // this edge to edge test is based on Franlin Antonio's gem:
        // "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202

        template <typename vector3_type, typename real_type>
        bool edge_edge_test(vector3_type const& V0, vector3_type const& U0, vector3_type const& U1,
          unsigned int i0, unsigned int i1, real_type Ax, real_type Ay)
        {
          real_type Bx=U0[i0]-U1[i0];
          real_type By=U0[i1]-U1[i1];
          real_type Cx=V0[i0]-U0[i0];
          real_type Cy=V0[i1]-U0[i1];
          real_type f=Ay*Bx-Ax*By;
          real_type d=By*Cx-Bx*Cy;
          if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))
          {
            real_type e=Ax*Cy-Ay*Cx;
            if(f>0)
            {
              if(e>=0 && e<=f) return true;
            }
            else
            {
              if(e<=0 && e>=f) return true;
            }
          }
          else
            return false;
        }

        template <typename vector3_type, typename real_type>
        bool edge_against_tri_edges(vector3_type const& V0, vector3_type const& V1,
          vector3_type const& U0, vector3_type const& U1, vector3_type const& U2,
          unsigned int i0, unsigned int i1)
        {
          real_type Ax=V1[i0]-V0[i0];
          real_type Ay=V1[i1]-V0[i1];
          // test edge U0,U1 against V0,V1
          if ( edge_edge_test(V0,U0,U1,i0,i1,Ax,Ay) ) return true;
          // test edge U1,U2 against V0,V1
          if ( edge_edge_test(V0,U1,U2,i0,i1,Ax,Ay) ) return true;
          // test edge U2,U1 against V0,V1
          if ( edge_edge_test(V0,U2,U0,i0,i1,Ax,Ay) ) return true;
          return false;
        }

        template <typename vector3_type, typename real_type>
        bool point_in_tri(vector3_type const& V0
          vector3_type const& U0, vector3_type const& U1, vector3_type const& U2,
          unsigned int i0, unsigned int i1)
        {
          // is T1 completly inside T2?
          // check if V0 is inside tri(U0,U1,U2)
          real_type a=U1[i1]-U0[i1];
          real_type b=-(U1[i0]-U0[i0]);
          real_type c=-a*U0[i0]-b*U0[i1];
          real_type d0=a*V0[i0]+b*V0[i1]+c;

          a=U2[i1]-U1[i1];
          b=-(U2[i0]-U1[i0]);
          c=-a*U1[i0]-b*U1[i1];
          real_type d1=a*V0[i0]+b*V0[i1]+c;

          a=U0[i1]-U2[i1];
          b=-(U0[i0]-U2[i0]);
          c=-a*U2[i0]-b*U2[i1];
          real_type d2=a*V0[i0]+b*V0[i1]+c;
          if(d0*d1>0.0)
          {
            if(d0*d2>0.0) return true;
          }
          return false;
        }

        template <typename vector3_type>
        bool coplanar_tri_tri(vector3_type const& N,
          vector3_type const& V0, vector3_type const& V1, vector3_type const& V2,
          vector3_type const& U0, vector3_type const& U1, vector3_type const& U2)
        {
          typedef typename vector3_type::value_type real_type;

          unsigned int i0,i1;
          // first project onto an axis-aligned plane, that maximizes the area
          // of the triangles, compute indices: i0,i1.
          real_type A0=std::fabs(N[0]);
          real_type A1=std::fabs(N[1]);
          real_type A2=std::fabs(N[2]);
          if(A0>A1)
          {
            if(A0>A2)
            {
              i0=1;      // A[0] is greatest
              i1=2;
            }
            else
            {
              i0=0;      // A[2] is greatest
              i1=1;
            }
          }
          else   // A[0]<=A[1]
          {
            if(A2>A1)
            {
              i0=0;      // A[2] is greatest
              i1=1;
            }
            else
            {
              i0=0;      // A[1] is greatest
              i1=2;
            }
          }

          // test all edges of triangle 1 against the edges of triangle 2
          if( edge_against_tri_edges(V0,V1,U0,U1,U2,i0,i1) ) return true;
          if( edge_against_tri_edges(V1,V2,U0,U1,U2,i0,i1) ) return true;
          if( edge_against_tri_edges(V2,V0,U0,U1,U2,i0,i1) ) return true;

          // finally, test if tri1 is totally contained in tri2 or vice versa
          if( point_in_tri(V0,U0,U1,U2,i0,i1) ) return true;
          if( point_in_tri(U0,V0,V1,V2,i0,i1) ) return true;

          return false;
        }

        template <typename vector3_type, typename real_type>
        void isect2(vector3_type const& VTX0, vector3_type const& VTX1, vector3_type const& VTX2,
          real_type VV0, real_type VV1, real_type VV2,
          real_type D0, real_type D1, real_type D2,
          real_type & isect0, real_type & isect1, vector3_type & isectpoint0, vector3_type & isectpoint1)
        {
          real_type tmp;
          vector3_type diff;
          tmp=D0/(D0-D1);
          isect0 = VV0+(VV1-VV0)*tmp;
          diff = VTX1-VTX2;
          diff *= tmp;
          isectpoint0 = diff+VTX0;
          tmp=D0/(D0-D2);
          isect1 = VV0+(VV2-VV0)*tmp;
          diff = VTX2-VTX0;
          diff *= tmp;
          isectpoint1 = VTX0 + diff;
        }

        template <typename vector3_type, typename real_type>
        bool compute_intervals_isectline(vector3_type const& VERT0, vector3_type const& VERT1, vector3_type const& VERT2,
          real_type VV0, real_type VV1, real_type VV2, real_type D0, real_type D1, real_type D2,
          real_type D0D1, real_type D0D2,
          real_type & isect0, real_type & isect1,
          vector3_type & isectpoint0, vector3_type & isectpoint1)
        {
          if(D0D1>0.0f)
          {
            // here we know that D0D2<=0.0
            // that is D0, D1 are on the same side, D2 on the other or on the plane
            isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
          }
          else if(D0D2>0.0f)
          {
            // here we know that d0d1<=0.0
            isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          else if(D1*D2>0.0f || D0!=0.0f)
          {
            // here we know that d0d1<=0.0 or that D0!=0.0
            isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          else if(D1!=0.0f)
          {
            isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          else if(D2!=0.0f)
          {
            isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
          }
          else
          {
            // triangles are coplanar
            return true;
          }
          return false;
        }

        template <typename vector3_type>
        bool triangle_triangle_interval_overlap(   vector3_type const & V0, vector3_type const & V1, vector3_type const & V2
          , vector3_type const & U0, vector3_type const & U1, vector3_type const & U2
          , bool & coplanar        , vector3_type & isectpt1, vector3_type & isectpt2
          , bool epsilon_test = true )
        {
          typedef typename vector3_type::value_type real_type;

          // compute plane equation of triangle(V0,V1,V2)
          vector3_type E1 = V1-V0;
          vector3_type E2 = V2-V0;
          vector3_type N1 = cross(E1,E2);
          real_type d1 = -dot(N1,V0);
          // plane equation 1: N1.X+d1=0

          // put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
          real_type du0 = dot(N1,U0)+d1;
          real_type du1 = dot(N1,U1)+d1;
          real_type du2 = dot(N1,U2)+d1;

          // coplanarity robustness check
          if (epsilon_test)
          {
            if( std::fabs(du0) < math::working_precision<real_type>() ) du0=0.0;
            if( std::fabs(du1) < math::working_precision<real_type>() ) du1=0.0;
            if( std::fabs(du2) < math::working_precision<real_type>() ) du2=0.0;
          }

          real_type du0du1=du0*du1;
          real_type du0du2=du0*du2;

          if(du0du1>0.0f && du0du2>0.0f) // same sign on all of them + not equal 0 ?
            return false;                // no intersection occurs

          // compute plane of triangle (U0,U1,U2)
          E1 = U1-U0;
          E2 = U2,U0;
          vector3_type N2 = cross(E1,E2);
          real_type d2 = -dot(N2,U0);
          // plane equation 2: N2.X+d2=0

          // put V0,V1,V2 into plane equation 2
          real_type dv0 = dot(N2,V0)+d2;
          real_type dv1 = dot(N2,V1)+d2;
          real_type dv2 = dot(N2,V2)+d2;

          // coplanarity robustness check
          if (epsilon_test)
          {
            if( std::fabs(dv0) < math::working_precision<real_type>() ) dv0=0.0;
            if( std::fabs(dv1) < math::working_precision<real_type>() ) dv1=0.0;
            if( std::fabs(dv2) < math::working_precision<real_type>() ) dv2=0.0;
          }

          real_type dv0dv1 = dv0*dv1;
          real_type dv0dv2 = dv0*dv2;

          if(dv0dv1>0.0f && dv0dv2>0.0f) // same sign on all of them + not equal 0 ?
            return false;                // no intersection occurs

          // compute direction of intersection line
          vector3_type D = cross(N1,N2);

          // compute and index to the largest component of D
          real_type max=std::fabs(D[0]);
          unsigned int index=0;
          real_type b=std::fabs(D[1]);
          real_type c=std::fabs(D[2]);
          if(b>max) max=b,index=1;
          if(c>max) max=c,index=2;

          // this is the simplified projection onto L
          real_type vp0 = V0[index];
          real_type vp1 = V1[index];
          real_type vp2 = V2[index];

          real_type up0 = U0[index];
          real_type up1 = U1[index];
          real_type up2 = U2[index];

          real_type isect1[2], isect2[2];
          vector3_type isectpointA1, isectpointA2;

          // compute interval for triangle 1
          coplanar=compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
            dv0dv1,dv0dv2,&isect1[0],&isect1[1],isectpointA1,isectpointA2);
          if(coplanar) return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);

          vector3_type isectpointB1, isectpointB2;

          // compute interval for triangle 2
          compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
            du0du1,du0du2,&isect2[0],&isect2[1],isectpointB1,isectpointB2);

          // sort so that a<=b
          unsigned int smallest1 = 0;
          if (isect1[0] > isect1[1])
          {
            swap(isect1[0], isect1[1]);
            smallest1=1;
          }
          unsigned int smallest2 = 0;
          if (isect2[0] > isect2[1])
          {
            swap(isect2[0], isect2[1]);
            smallest2=1;
          }

          if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return false;

          // at this point, we know that the triangles intersect

          if(isect2[0]<isect1[0])
          {
            isectpt1 = smallest1==0 ? isectpointA1 : isectpointA2;

            if(isect2[1]<isect1[1])
            {
              isectpt2 = smallest2==0 ? isectpointB2 : isectpointB1;
            }
            else
            {
              isectpt2 = smallest1==0 ? isectpointA2 : isectpointA1;
            }
          }
          else
          {
            isectpt1 = smallest2==0 ? isectpointB1 : isectpointB2;

            if(isect2[1]>isect1[1])
            {
              isectpt2 = smallest1==0 ? isectpointA2 : isectpointA1;
            }
            else
            {
              isectpt2 = smallest2==0 ? isectpointB2 : isectpointB1;
            }
          }
          return true;
        }

      } // namespace detail


      template <typename triangle_type>
      bool triangle_triangle_interval_overlap( triangle_type const & t0, triangle_type const & t1, bool epsilon_test = true )
      {
        return triangle_triangle_interval_overlap(t0.p0(), t0.p1(), t0.p2(), t1.p0(), t1.p1(), t1.p2(),epsilon_test);
      }

      template <typename triangle_type, typename vector3_type>
      bool triangle_triangle_interval_overlap(   triangle_type const & t0, triangle_type const & t1
        , bool & coplanar, vector3_type & p0, vector3_type & p1
        , bool epsilon_test = true )
      {
        return triangle_triangle_interval_overlap(t0.p0(), t0.p1(), t0.p2(), t1.p0(), t1.p1(), t1.p2(), coplanar, p0, p1, epsilon_test);
      }

      template <typename vector3_type>
      bool triangle_triangle_interval_overlap(   vector3_type const & v0, vector3_type const & v1, vector3_type const & v2
        , vector3_type const & u0, vector3_type const & u1, vector3_type const & u2
        , bool epsilon_test = true )
      {
        bool dummy_bool;
        vector3_type dummy_v0;
        vector3_type dummy_v1;
        return triangle_triangle_interval_overlap(v0,v1,v2,u0,u1,u2,dummy_bool,dummy_v1,dummy_v2,epsilon_test);

      }
      template <typename vector3_type>
      bool triangle_triangle_interval_overlap(   vector3_type const & v0, vector3_type const & v1, vector3_type const & v2
        , vector3_type const & u0, vector3_type const & u1, vector3_type const & u2
        , bool & coplanar        , vector3_type       & p0, vector3_type       & p1
        , bool epsilon_test = true )
      {
        return detail::triangle_triangle_interval_overlap(v0,v1,v2,u0,u1,u2,coplanar,p0,p1,epsilon_test);
      }

      // NOTE: This is the old macro-code. To be deleted when above has been tested.
      namespace old_stuff_soon_dies
      {
        bool wrapNoDivTriTriIntersect(const Triangle & triA,const Triangle & triB);
        bool wrapTriTriIntersectLine(const Triangle & triA,const Triangle & triB,bool & coplanar,Vector3<double> & pt1,Vector3<double> & pt2);
        bool wrapTriTriIntersect(const Triangle & triA,const Triangle & triB);

        bool NoDivTriTriIsect(float V0[3],float V1[3],float V2[3], float U0[3],float U1[3],float U2[3]);
        bool tri_tri_intersect_with_isectline(float V0[3],float V1[3],float V2[3], float U0[3],float U1[3],float U2[3],int *coplanar, float isectpt1[3],float isectpt2[3]);
        bool tri_tri_intersect(float V0[3],float V1[3],float V2[3], float U0[3],float U1[3],float U2[3]);
        /* Triangle/triangle intersection test routine,
        * by Tomas Moller, 1997.
        * See article "A Fast Triangle-Triangle Intersection Test",
        * Journal of Graphics Tools, 2(2), 1997
        * updated: 2001-06-20 (added line of intersection)
        *
        * int tri_tri_intersect(float V0[3],float V1[3],float V2[3],
        *                       float U0[3],float U1[3],float U2[3])
        *
        * parameters: vertices of triangle 1: V0,V1,V2
        *             vertices of triangle 2: U0,U1,U2
        * result    : returns 1 if the triangles intersect, otherwise 0
        *
        * Here is a version withouts divisions (a little faster)
        * int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
        *                      float U0[3],float U1[3],float U2[3]);
        *
        * This version computes the line of intersection as well (if they are not coplanar):
        * int tri_tri_intersect_with_isectline(float V0[3],float V1[3],float V2[3],
        *               float U0[3],float U1[3],float U2[3],int *coplanar,
        *               float isectpt1[3],float isectpt2[3]);
        * coplanar returns whether the tris are coplanar
        * isectpt1, isectpt2 are the endpoints of the line of intersection
        */

        // TODO: Macros Everywhere. Yikes!

#include <math.h>

#define FABS(x) ((float)std::fabs(x))        /* implement as is fastest on your machine */

        /* if USE_EPSILON_TEST is true then we do a check:
        if |dv|<EPSILON then dv=0.0;
        else no check is done (which is less robust)
        */
#define USE_EPSILON_TEST TRUE
#define EPSILON 0.000001


        /* some macros */
#define CROSS(dest,v1,v2)                      \
  dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
  dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
  dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; dest[1]=v1[1]-v2[1]; dest[2]=v1[2]-v2[2];

#define ADD(dest,v1,v2) dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2];

#define MULT(dest,v,factor) dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2];

#define SET(dest,src) dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2];

        /* sort so that a<=b */
#define SORT(a,b)       \
  if(a>b)    \
        {          \
        float MACROTEMP; \
        MACROTEMP=a;     \
        a=b;     \
        b=MACROTEMP;     \
        }

#define ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1) \
  isect0=VV0+(VV1-VV0)*D0/(D0-D1);    \
  isect1=VV0+(VV2-VV0)*D0/(D0-D2);


#define COMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1) \
  if(D0D1>0.0f)                                         \
        {                                                     \
        /* here we know that D0D2<=0.0 */                   \
        /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
        ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
        }                                                     \
  else if(D0D2>0.0f)                                    \
        {                                                     \
        /* here we know that d0d1<=0.0 */                   \
        ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
        }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
        {                                                     \
        /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
        ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1);          \
        }                                                     \
  else if(D1!=0.0f)                                     \
        {                                                     \
        ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
        }                                                     \
  else if(D2!=0.0f)                                     \
        {                                                     \
        ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
        }                                                     \
  else                                                  \
        {                                                     \
        /* triangles are coplanar */                        \
        return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
        }



        /* this edge to edge test is based on Franlin Antonio's gem:
        "Faster Line Segment Intersection", in Graphics Gems III,
        pp. 199-202 */
#define EDGE_EDGE_TEST(V0,U0,U1)                      \
  Bx=U0[i0]-U1[i0];                                   \
  By=U0[i1]-U1[i1];                                   \
  Cx=V0[i0]-U0[i0];                                   \
  Cy=V0[i1]-U0[i1];                                   \
  f=Ay*Bx-Ax*By;                                      \
  d=By*Cx-Bx*Cy;                                      \
  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  \
        {                                                   \
        e=Ax*Cy-Ay*Cx;                                    \
        if(f>0)                                           \
        {                                                 \
        if(e>=0 && e<=f) return 1;                      \
        }                                                 \
    else                                              \
        {                                                 \
        if(e<=0 && e>=f) return 1;                      \
        }                                                 \
        }

#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
        {                                              \
        float Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
        Ax=V1[i0]-V0[i0];                            \
        Ay=V1[i1]-V0[i1];                            \
        /* test edge U0,U1 against V0,V1 */          \
        EDGE_EDGE_TEST(V0,U0,U1);                    \
        /* test edge U1,U2 against V0,V1 */          \
        EDGE_EDGE_TEST(V0,U1,U2);                    \
        /* test edge U2,U1 against V0,V1 */          \
        EDGE_EDGE_TEST(V0,U2,U0);                    \
        }

#define POINT_IN_TRI(V0,U0,U1,U2)           \
        {                                           \
        float a,b,c,d0,d1,d2;                     \
        /* is T1 completly inside T2? */          \
        /* check if V0 is inside tri(U0,U1,U2) */ \
        a=U1[i1]-U0[i1];                          \
        b=-(U1[i0]-U0[i0]);                       \
        c=-a*U0[i0]-b*U0[i1];                     \
        d0=a*V0[i0]+b*V0[i1]+c;                   \
        \
        a=U2[i1]-U1[i1];                          \
        b=-(U2[i0]-U1[i0]);                       \
        c=-a*U1[i0]-b*U1[i1];                     \
        d1=a*V0[i0]+b*V0[i1]+c;                   \
        \
        a=U0[i1]-U2[i1];                          \
        b=-(U0[i0]-U2[i0]);                       \
        c=-a*U2[i0]-b*U2[i1];                     \
        d2=a*V0[i0]+b*V0[i1]+c;                   \
        if(d0*d1>0.0)                             \
        {                                         \
        if(d0*d2>0.0) return 1;                 \
        }                                         \
        }

        bool coplanar_tri_tri(float N[3],float V0[3],float V1[3],float V2[3],
          float U0[3],float U1[3],float U2[3])
        {
          float A[3];
          short i0,i1;
          /* first project onto an axis-aligned plane, that maximizes the area */
          /* of the triangles, compute indices: i0,i1. */
          A[0]=std::fabs(N[0]);
          A[1]=std::fabs(N[1]);
          A[2]=std::fabs(N[2]);
          if(A[0]>A[1])
          {
            if(A[0]>A[2])
            {
              i0=1;      /* A[0] is greatest */
              i1=2;
            }
            else
            {
              i0=0;      /* A[2] is greatest */
              i1=1;
            }
          }
          else   /* A[0]<=A[1] */
          {
            if(A[2]>A[1])
            {
              i0=0;      /* A[2] is greatest */
              i1=1;
            }
            else
            {
              i0=0;      /* A[1] is greatest */
              i1=2;
            }
          }

          /* test all edges of triangle 1 against the edges of triangle 2 */
          EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
          EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
          EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);

          /* finally, test if tri1 is totally contained in tri2 or vice versa */
          POINT_IN_TRI(V0,U0,U1,U2);
          POINT_IN_TRI(U0,V0,V1,V2);

          return false;
        }



        /**
        * OpenTissue Wrapper.
        * Makes it a little more easy to interface with OpenTissue data structures.
        */
        bool wrapTriTriIntersect(const Triangle & triA,const Triangle & triB)
        {
          float V0[3];
          V0[0] = static_cast<float>(triA.p0()[0]);
          V0[1] = static_cast<float>(triA.p0()[1]);
          V0[2] = static_cast<float>(triA.p0()[2]);
          float V1[3];
          V1[0] = static_cast<float>(triA.p1()[0]);
          V1[1] = static_cast<float>(triA.p1()[1]);
          V1[2] = static_cast<float>(triA.p1()[2]);
          float V2[3];
          V2[0] = static_cast<float>(triA.p2()[0]);
          V2[1] = static_cast<float>(triA.p2()[1]);
          V2[2] = static_cast<float>(triA.p2()[2]);

          float U0[3];
          U0[0] = static_cast<float>(triB.p0()[0]);
          U0[1] = static_cast<float>(triB.p0()[1]);
          U0[2] = static_cast<float>(triB.p0()[2]);
          float U1[3];
          U1[0] = static_cast<float>(triB.p1()[0]);
          U1[1] = static_cast<float>(triB.p1()[1]);
          U1[2] = static_cast<float>(triB.p1()[2]);
          float U2[3];
          U2[0] = static_cast<float>(triB.p2()[0]);
          U2[1] = static_cast<float>(triB.p2()[1]);
          U2[2] = static_cast<float>(triB.p2()[2]);
          return tri_tri_intersect(V0,V1,V2,U0,U1,U2);
        }


        bool tri_tri_intersect(float V0[3],float V1[3],float V2[3],
          float U0[3],float U1[3],float U2[3])
        {
          float E1[3],E2[3];
          float N1[3],N2[3],d1,d2;
          float du0,du1,du2,dv0,dv1,dv2;
          float D[3];
          float isect1[2], isect2[2];
          float du0du1,du0du2,dv0dv1,dv0dv2;
          short index;
          float vp0,vp1,vp2;
          float up0,up1,up2;
          float b,c,max;

          /* compute plane equation of triangle(V0,V1,V2) */
          SUB(E1,V1,V0);
          SUB(E2,V2,V0);
          CROSS(N1,E1,E2);
          d1=-DOT(N1,V0);
          /* plane equation 1: N1.X+d1=0 */

          /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
          du0=DOT(N1,U0)+d1;
          du1=DOT(N1,U1)+d1;
          du2=DOT(N1,U2)+d1;

          /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
          if(std::fabs(du0)<EPSILON) du0=0.0;
          if(std::fabs(du1)<EPSILON) du1=0.0;
          if(std::fabs(du2)<EPSILON) du2=0.0;
#endif
          du0du1=du0*du1;
          du0du2=du0*du2;

          if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                    /* no intersection occurs */

          /* compute plane of triangle (U0,U1,U2) */
          SUB(E1,U1,U0);
          SUB(E2,U2,U0);
          CROSS(N2,E1,E2);
          d2=-DOT(N2,U0);
          /* plane equation 2: N2.X+d2=0 */

          /* put V0,V1,V2 into plane equation 2 */
          dv0=DOT(N2,V0)+d2;
          dv1=DOT(N2,V1)+d2;
          dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
          if(std::fabs(dv0)<EPSILON) dv0=0.0;
          if(std::fabs(dv1)<EPSILON) dv1=0.0;
          if(std::fabs(dv2)<EPSILON) dv2=0.0;
#endif

          dv0dv1=dv0*dv1;
          dv0dv2=dv0*dv2;

          if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                    /* no intersection occurs */

          /* compute direction of intersection line */
          CROSS(D,N1,N2);

          /* compute and index to the largest component of D */
          max=std::fabs(D[0]);
          index=0;
          b=std::fabs(D[1]);
          c=std::fabs(D[2]);
          if(b>max) max=b,index=1;
          if(c>max) max=c,index=2;

          /* this is the simplified projection onto L*/
          vp0=V0[index];
          vp1=V1[index];
          vp2=V2[index];

          up0=U0[index];
          up1=U1[index];
          up2=U2[index];

          /* compute interval for triangle 1 */
          // TODO: Comparing floats with ==
          COMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,isect1[0],isect1[1]);

          /* compute interval for triangle 2 */
          // TODO: Comparing floats with ==
          COMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,isect2[0],isect2[1]);

          SORT(isect1[0],isect1[1]);
          SORT(isect2[0],isect2[1]);

          if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return false;
          return true;
        }


#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
        { \
        if(D0D1>0.0f) \
        { \
        /* here we know that D0D2<=0.0 */ \
        /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
        A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
  else if(D0D2>0.0f)\
        { \
        /* here we know that d0d1<=0.0 */ \
        A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
  else if(D1*D2>0.0f || D0!=0.0f) \
        { \
        /* here we know that d0d1<=0.0 or that D0!=0.0 */ \
        A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; \
        } \
  else if(D1!=0.0f) \
        { \
        A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
  else if(D2!=0.0f) \
        { \
        A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
  else \
        { \
        /* triangles are coplanar */ \
        return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2); \
        } \
        }




        /**
        * OpenTissue Wrapper.
        * Makes it a little more easy to interface with OpenTissue data structures.
        */
        bool wrapNoDivTriTriIntersect(const Triangle & triA,const Triangle & triB)
        {
          float V0[3];
          V0[0] = static_cast<float>(triA.p0()[0]);
          V0[1] = static_cast<float>(triA.p0()[1]);
          V0[2] = static_cast<float>(triA.p0()[2]);
          float V1[3];
          V1[0] = static_cast<float>(triA.p1()[0]);
          V1[1] = static_cast<float>(triA.p1()[1]);
          V1[2] = static_cast<float>(triA.p1()[2]);
          float V2[3];
          V2[0] = static_cast<float>(triA.p2()[0]);
          V2[1] = static_cast<float>(triA.p2()[1]);
          V2[2] = static_cast<float>(triA.p2()[2]);

          float U0[3];
          U0[0] = static_cast<float>(triB.p0()[0]);
          U0[1] = static_cast<float>(triB.p0()[1]);
          U0[2] = static_cast<float>(triB.p0()[2]);
          float U1[3];
          U1[0] = static_cast<float>(triB.p1()[0]);
          U1[1] = static_cast<float>(triB.p1()[1]);
          U1[2] = static_cast<float>(triB.p1()[2]);
          float U2[3];
          U2[0] = static_cast<float>(triB.p2()[0]);
          U2[1] = static_cast<float>(triB.p2()[1]);
          U2[2] = static_cast<float>(triB.p2()[2]);
          return NoDivTriTriIsect(V0,V1,V2,U0,U1,U2);
        }



        bool NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
          float U0[3],float U1[3],float U2[3])
        {
          float E1[3],E2[3];
          float N1[3],N2[3],d1,d2;
          float du0,du1,du2,dv0,dv1,dv2;
          float D[3];
          float isect1[2], isect2[2];
          float du0du1,du0du2,dv0dv1,dv0dv2;
          short index;
          float vp0,vp1,vp2;
          float up0,up1,up2;
          float bb,cc,max;
          float a,b,c,x0,x1;
          float d,e,f,y0,y1;
          float xx,yy,xxyy,tmp;

          /* compute plane equation of triangle(V0,V1,V2) */
          SUB(E1,V1,V0);
          SUB(E2,V2,V0);
          CROSS(N1,E1,E2);
          d1=-DOT(N1,V0);
          /* plane equation 1: N1.X+d1=0 */

          /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
          du0=DOT(N1,U0)+d1;
          du1=DOT(N1,U1)+d1;
          du2=DOT(N1,U2)+d1;

          /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
          if(FABS(du0)<EPSILON) du0=0.0;
          if(FABS(du1)<EPSILON) du1=0.0;
          if(FABS(du2)<EPSILON) du2=0.0;
#endif
          du0du1=du0*du1;
          du0du2=du0*du2;

          if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                    /* no intersection occurs */

          /* compute plane of triangle (U0,U1,U2) */
          SUB(E1,U1,U0);
          SUB(E2,U2,U0);
          CROSS(N2,E1,E2);
          d2=-DOT(N2,U0);
          /* plane equation 2: N2.X+d2=0 */

          /* put V0,V1,V2 into plane equation 2 */
          dv0=DOT(N2,V0)+d2;
          dv1=DOT(N2,V1)+d2;
          dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
          if(FABS(dv0)<EPSILON) dv0=0.0;
          if(FABS(dv1)<EPSILON) dv1=0.0;
          if(FABS(dv2)<EPSILON) dv2=0.0;
#endif

          dv0dv1=dv0*dv1;
          dv0dv2=dv0*dv2;

          if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
            return false;                    /* no intersection occurs */

          /* compute direction of intersection line */
          CROSS(D,N1,N2);

          /* compute and index to the largest component of D */
          max=(float)FABS(D[0]);
          index=0;
          bb=(float)FABS(D[1]);
          cc=(float)FABS(D[2]);
          if(bb>max) max=bb,index=1;
          if(cc>max) max=cc,index=2;

          /* this is the simplified projection onto L*/
          vp0=V0[index];
          vp1=V1[index];
          vp2=V2[index];

          up0=U0[index];
          up1=U1[index];
          up2=U2[index];

          /* compute interval for triangle 1 */
          // TODO: Comparing floats with ==
          NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

          /* compute interval for triangle 2 */
          // TODO: Comparing floats with ==
          NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

          xx=x0*x1;
          yy=y0*y1;
          xxyy=xx*yy;

          tmp=a*xxyy;
          isect1[0]=tmp+b*x1*yy;
          isect1[1]=tmp+c*x0*yy;

          tmp=d*xxyy;
          isect2[0]=tmp+e*xx*y1;
          isect2[1]=tmp+f*xx*y0;

          SORT(isect1[0],isect1[1]);
          SORT(isect2[0],isect2[1]);

          if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return false;
          return true;
        }

        /* sort so that a<=b */
#define SORT2(a,b,smallest)       \
  if(a>b)         \
        {             \
        float MACRO_C;\
        MACRO_C=a;    \
        a=b;          \
        b=MACRO_C;    \
        smallest=1;   \
        }             \
  else smallest=0;


        void isect2(float VTX0[3],float VTX1[3],float VTX2[3],float VV0,float VV1,float VV2,
          float D0,float D1,float D2,float *isect0,float *isect1,float isectpoint0[3],float isectpoint1[3])
        {
          float tmp=D0/(D0-D1);
          float diff[3];
          *isect0=VV0+(VV1-VV0)*tmp;
          SUB(diff,VTX1,VTX0);
          MULT(diff,diff,tmp);
          ADD(isectpoint0,diff,VTX0);
          tmp=D0/(D0-D2);
          *isect1=VV0+(VV2-VV0)*tmp;
          SUB(diff,VTX2,VTX0);
          MULT(diff,diff,tmp);
          ADD(isectpoint1,VTX0,diff);
        }


#if 0
#define ISECT2(VTX0,VTX1,VTX2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1) \
  tmp=D0/(D0-D1);                    \
  isect0=VV0+(VV1-VV0)*tmp;          \
  SUB(diff,VTX1,VTX0);               \
  MULT(diff,diff,tmp);               \
  ADD(isectpoint0,diff,VTX0);        \
  tmp=D0/(D0-D2);
        /*              isect1=VV0+(VV2-VV0)*tmp;          \ */
        /*              SUB(diff,VTX2,VTX0);               \     */
        /*              MULT(diff,diff,tmp);               \   */
        /*              ADD(isectpoint1,VTX0,diff);           */
#endif

        bool compute_intervals_isectline(float VERT0[3],float VERT1[3],float VERT2[3],
          float VV0,float VV1,float VV2,float D0,float D1,float D2,
          float D0D1,float D0D2,float *isect0,float *isect1,
          float isectpoint0[3],float isectpoint1[3])
        {
          if(D0D1>0.0f)
          {
            /* here we know that D0D2<=0.0 */
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */
            isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
          }
          else if(D0D2>0.0f)
          {
            /* here we know that d0d1<=0.0 */
            isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          // TODO: Comparing floats with !=
          else if(D1*D2>0.0f || D0!=0.0f)
          {
            /* here we know that d0d1<=0.0 or that D0!=0.0 */
            isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          // TODO: Comparing floats with !=
          else if(D1!=0.0f)
          {
            isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
          }
          // TODO: Comparing floats with !=
          else if(D2!=0.0f)
          {
            isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
          }
          else
          {
            /* triangles are coplanar */
            return true;
          }
          return false;
        }

#define COMPUTE_INTERVALS_ISECTLINE(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1,isectpoint0,isectpoint1) \
  if(D0D1>0.0f)                                         \
        {                                                     \
        /* here we know that D0D2<=0.0 */                   \
        /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
        isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
        }
#if 0
  else if(D0D2>0.0f)                                    \
  {                                                     \
  /* here we know that d0d1<=0.0 */                   \
  isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
  /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
  isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
  isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
  isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else                                                  \
  {                                                     \
  /* triangles are coplanar */                        \
  coplanar=1;                                         \
  return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
  }
#endif







  /**
  * OpenTissue Wrapper.
  * Makes it a little more easy to interface with OpenTissue data structures.
  */
  bool wrapTriTriIntersectLine(const Triangle & triA,const Triangle & triB,bool & coplanar,Vector3<double> & pt1,Vector3<double> & pt2)
  {
    float V0[3];
    V0[0] = static_cast<float>(triA.p0()[0]);
    V0[1] = static_cast<float>(triA.p0()[1]);
    V0[2] = static_cast<float>(triA.p0()[2]);
    float V1[3];
    V1[0] = static_cast<float>(triA.p1()[0]);
    V1[1] = static_cast<float>(triA.p1()[1]);
    V1[2] = static_cast<float>(triA.p1()[2]);
    float V2[3];
    V2[0] = static_cast<float>(triA.p2()[0]);
    V2[1] = static_cast<float>(triA.p2()[1]);
    V2[2] = static_cast<float>(triA.p2()[2]);

    float U0[3];
    U0[0] = static_cast<float>(triB.p0()[0]);
    U0[1] = static_cast<float>(triB.p0()[1]);
    U0[2] = static_cast<float>(triB.p0()[2]);
    float U1[3];
    U1[0] = static_cast<float>(triB.p1()[0]);
    U1[1] = static_cast<float>(triB.p1()[1]);
    U1[2] = static_cast<float>(triB.p1()[2]);
    float U2[3];
    U2[0] = static_cast<float>(triB.p2()[0]);
    U2[1] = static_cast<float>(triB.p2()[1]);
    U2[2] = static_cast<float>(triB.p2()[2]);

    int tmp = 0;
    float isectpt1[3];
    float isectpt2[3];
    bool result = tri_tri_intersect_with_isectline(V0,V1,V2,U0,U1,U2,&tmp,isectpt1,isectpt2);

    coplanar = false;
    if(tmp)
      coplanar = true;
    pt1 = Vector3<double>(isectpt1[0],isectpt1[1],isectpt1[2]);
    pt2 = Vector3<double>(isectpt2[0],isectpt2[1],isectpt2[2]);
    return result;
  }

  bool tri_tri_intersect_with_isectline(float V0[3],float V1[3],float V2[3],
    float U0[3],float U1[3],float U2[3],int *coplanar,
    float isectpt1[3],float isectpt2[3])
  {
    float E1[3],E2[3];
    float N1[3],N2[3],d1,d2;
    float du0,du1,du2,dv0,dv1,dv2;
    float D[3];
    float isect1[2], isect2[2];
    float isectpointA1[3],isectpointA2[3];
    float isectpointB1[3],isectpointB2[3];
    float du0du1,du0du2,dv0dv1,dv0dv2;
    short index;
    float vp0,vp1,vp2;
    float up0,up1,up2;
    float b,c,max;
    //float tmp,diff[3];   //--- KE 25-05-2004: Unreferenced local variable?
    int smallest1,smallest2;

    /* compute plane equation of triangle(V0,V1,V2) */
    SUB(E1,V1,V0);
    SUB(E2,V2,V0);
    CROSS(N1,E1,E2);
    d1=-DOT(N1,V0);
    /* plane equation 1: N1.X+d1=0 */

    /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
    du0=DOT(N1,U0)+d1;
    du1=DOT(N1,U1)+d1;
    du2=DOT(N1,U2)+d1;

    /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
    if(std::fabs(du0)<EPSILON) du0=0.0;
    if(std::fabs(du1)<EPSILON) du1=0.0;
    if(std::fabs(du2)<EPSILON) du2=0.0;
#endif
    du0du1=du0*du1;
    du0du2=du0*du2;

    if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
      return false;                    /* no intersection occurs */

    /* compute plane of triangle (U0,U1,U2) */
    SUB(E1,U1,U0);
    SUB(E2,U2,U0);
    CROSS(N2,E1,E2);
    d2=-DOT(N2,U0);
    /* plane equation 2: N2.X+d2=0 */

    /* put V0,V1,V2 into plane equation 2 */
    dv0=DOT(N2,V0)+d2;
    dv1=DOT(N2,V1)+d2;
    dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
    if(std::fabs(dv0)<EPSILON) dv0=0.0;
    if(std::fabs(dv1)<EPSILON) dv1=0.0;
    if(std::fabs(dv2)<EPSILON) dv2=0.0;
#endif

    dv0dv1=dv0*dv1;
    dv0dv2=dv0*dv2;

    if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
      return false;                    /* no intersection occurs */

    /* compute direction of intersection line */
    CROSS(D,N1,N2);

    /* compute and index to the largest component of D */
    max=std::fabs(D[0]);
    index=0;
    b=std::fabs(D[1]);
    c=std::fabs(D[2]);
    if(b>max) max=b,index=1;
    if(c>max) max=c,index=2;

    /* this is the simplified projection onto L*/
    vp0=V0[index];
    vp1=V1[index];
    vp2=V2[index];

    up0=U0[index];
    up1=U1[index];
    up2=U2[index];

    /* compute interval for triangle 1 */
    *coplanar=compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
      dv0dv1,dv0dv2,&isect1[0],&isect1[1],isectpointA1,isectpointA2);
    if(*coplanar) return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);


    /* compute interval for triangle 2 */
    compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
      du0du1,du0du2,&isect2[0],&isect2[1],isectpointB1,isectpointB2);

    SORT2(isect1[0],isect1[1],smallest1);
    SORT2(isect2[0],isect2[1],smallest2);

    if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return false;

    /* at this point, we know that the triangles intersect */

    if(isect2[0]<isect1[0])
    {
      if(smallest1==0) { SET(isectpt1,isectpointA1); }
      else { SET(isectpt1,isectpointA2); }

      if(isect2[1]<isect1[1])
      {
        if(smallest2==0) { SET(isectpt2,isectpointB2); }
        else { SET(isectpt2,isectpointB1); }
      }
      else
      {
        if(smallest1==0) { SET(isectpt2,isectpointA2); }
        else { SET(isectpt2,isectpointA1); }
      }
    }
    else
    {
      if(smallest2==0) { SET(isectpt1,isectpointB1); }
      else { SET(isectpt1,isectpointB2); }

      if(isect2[1]>isect1[1])
      {
        if(smallest1==0) { SET(isectpt2,isectpointA2); }
        else { SET(isectpt2,isectpointA1); }
      }
      else
      {
        if(smallest2==0) { SET(isectpt2,isectpointB2); }
        else { SET(isectpt2,isectpointB1); }
      }
    }
    return true;
  }



      } // namespace old_stuff_soon_dies




    } //namespace Moller

  } // namespace intersect
} // namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_TRIANGLE_TRIANGLE_INTERVAL_OVERLAP_H
#endif
