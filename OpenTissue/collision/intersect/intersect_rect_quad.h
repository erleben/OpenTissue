#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_H
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
    * Rectangle Quadrilateral Intersection Testing.
    *
    * Intersection test between a rectanlge, given by the two half-edge extents
    * rect[0] and rect[1], i.e. the rectangle is given by all points, p, lying in side
    *
    *     -rect[0] <=  p(0)  <= rect[0] and   -rect[1] <=  p(1)  <= rect[1]
    *
    * The quadrangle (orientated rectangle) are given by the 4 corner points:
    *
    *    q1(0) = quad[0] and q1(1) = quad[1]
    *    q2(0) = quad[2] and q2(1) = quad[3]
    *    q3(0) = quad[4] and q3(1) = quad[5]
    *    q4(0) = quad[6] and q4(1) = quad[7]
    *
    *
    * This method also returns interior points, not only edge-edge crossings.
    *
    *
    * @param rect       A pointer to an array (2 reals) that defines the rectangle.
    * @param quad       A pointer to an array (8 reals) that defines the quadrilateral.
    * @param ret        Upon return holds the coordinates of the intersection points.
    *
    * @return           The number of intersection points.
    */
    template<typename real_type>
    int rect_quad(real_type * rect,real_type * quad,real_type * ret)
    {
      int cnt = 0;
      real_type * r = ret;
      {
        //--- Test the four vertices of the quad to see if they are
        //--- inside the rect or lies on the boundary
        real_type * q = quad;
        for(int i=0;i<4;++i)
        {
          real_type qx = *q;  ++q;
          real_type qy = *q;  ++q;
          if( (-rect[0] <= qx) && (qx <= rect[0]) && (-rect[1] <= qy) && (qy <= rect[1]))
          {
            *r = qx; ++r;
            *r = qy; ++r;
            ++cnt;
          }
        }
      }
      {
        //--- Test the four edges of the quad for crossing the edges of the rect.
        real_type qx0,qy0,qx1,qy1;
        real_type * q = quad;
        for(int i=0;i<4;++i)
        {
          qx0 = *q;     ++q;
          qy0 = *q;     ++q;

          real_type * nextq = (i==3)?quad:q;
          qx1 = *nextq; ++nextq;
          qy1 = *nextq;
          //--- Math derivations for finding crossings with vertical rect-sides
          //
          // The formula for the x-coordinate of a straight line is:
          //
          //  qx0 + t (qx1-qx0) = qxt
          //
          //  Knowing qxt, which we do since it is given by the rect side, we can isolate the t-parameter
          //
          //  t  = (qxt-qx0) / (qx1-qx0)
          //
          //  Now the formula for the y-coordinate is:
          //
          //  qy0 + t (qy1-qy0) = qyt
          //
          //  Substituting the t-parameter value, allow us to compute the y-value of the crossing point.
          //
          //  qy0 + ( (qxt-qx0)/(qx1-qx0) ) (qy1-qy0) = qyt
          //
          // Cleaning up the parenteses we have
          //
          //  qyt = qy0 + (qxt-qx0)*(qy1-qy0)/(qx1-qx0)
          //
          real_type dx = (qx1-qx0);
          real_type dy = (qy1-qy0);
          if(fabs(dx)>0)
          {
            real_type alpha = dy/dx;
            if(
              ((qx0 < -rect[0]) && (qx1 > -rect[0]))
              ||
              ((qx0 > -rect[0]) && (qx1 < -rect[0]))
              )
            {
              real_type qxt = -rect[0];
              real_type qyt = qy0 + (qxt-qx0)*alpha;
              if( (-rect[1] < qyt) &&   (qyt < rect[1]))
              {
                *r = qxt; ++r;
                *r = qyt; ++r;
                ++cnt;
              }
            }
            if(
              ((qx0 < rect[0]) && (qx1 > rect[0]))
              ||
              ((qx0 > rect[0]) && (qx1 < rect[0]))
              )
            {
              real_type qxt = rect[0];
              real_type qyt = qy0 + (qxt-qx0)*alpha;
              if( (-rect[1] < qyt) &&   (qyt < rect[1]))
              {
                *r = qxt; ++r;
                *r = qyt; ++r;
                ++cnt;
              }
            }
          }
          //--- Math derivations for finding crossings with horizontal rect-sides
          //
          // The formula for the y-coordinate of a straight line is:
          //
          //  qy0 + t (qy1-qy0) = qyt
          //
          //  Knowing qyt, which we do since it is given by the rect side, we can isolate the t-parameter
          //
          //  t  = (qyt-qy0) / (qy1-qy0)
          //
          //  Now the formula for the x-coordinate is:
          //
          //  qxt = qx0 + t (qx1-qx0)
          //
          //  Substituting the t-parameter value, allow us to compute the x-value of the crossing point.
          //
          //  qxt = qx0 + (qyt-qy0)(qx1-qx0)/(qy1-qy0)
          //
          //
          if(fabs(dy)>0)
          {
            real_type inv_alpha = dx/dy;
            if(
              ((qy0 < -rect[1]) && (qy1 > -rect[1]))
              ||
              ((qy0 > -rect[1]) && (qy1 < -rect[1]))
              )
            {
              real_type qyt = -rect[1];
              real_type qxt = qx0 + (qyt-qy0)*inv_alpha;
              if( (-rect[0] < qxt)&&(qxt < rect[0]))
              {
                *r = qxt;  ++r;
                *r = qyt;  ++r;
                ++cnt;
              }
            }
            if(
              ((qy0 < rect[1]) && (qy1 > rect[1]))
              ||
              ((qy0 > rect[1]) && (qy1 < rect[1]))
              )
            {
              real_type qyt = rect[1];
              real_type qxt = qx0 + (qyt-qy0)*inv_alpha;
              if( (-rect[0] < qxt)&&(qxt < rect[0]))
              {
                *r = qxt;  ++r;
                *r = qyt;  ++r;
                ++cnt;
              }
            }
          }
        }
      }

      return cnt;
    }

  } //End of namespace intersect

} //End of namespace OpenTissue

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_H
#endif
