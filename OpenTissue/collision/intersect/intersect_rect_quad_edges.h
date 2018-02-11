#ifndef OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_EDGES_H
#define OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_EDGES_H
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
    * Rectangle Quadrilateral Edges Intersection Testing.
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
    * This method is only concerned about edge-edge crossings, interior points are completely ignored.
    *
    *
    * @param rect       A pointer to an array (2 reals) that defines the rectangle.
    * @param quad       A pointer to an array (8 reals) that defines the quadrilateral.
    * @param inside     A pointer to an array (4 booleans) holding the inside status of  quadrilateral corners.
    * @param ret        Upon return holds the coordinates of the intersection points.
    *
    * @return           The number of intersection points.
    */
    template<typename real_type>
    int rect_quad_edges(real_type * rect, real_type * quad, bool * inside, real_type * ret)
    {
      int cnt = 0;
      real_type * r = ret;
      {
        //--- Test the four edges of the quad for crossing the edges of the rect.
        real_type qx0,qy0,qx1,qy1,tst;
        real_type * q = quad;
        for(int i=0;i<4;++i)
        {
          qx0 = *q;     ++q;
          qy0 = *q;     ++q;
          real_type * nextq = (i==3)?quad:q;
          qx1 = *nextq; ++nextq;
          qy1 = *nextq;
          bool inside0 = inside[i];
          bool inside1 = inside[(i+1)%4];
          if(inside0 && inside1)
            continue;
          real_type dx = (qx1-qx0);
          real_type dy = (qy1-qy0);
          if(dx)
          {
            real_type alpha = dy/dx;
            tst = - rect[0];     //--- left side
            if( ((qx0 < tst) && (qx1 > tst)) || ((qx0 > tst) && (qx1 < tst)) )
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
            tst = rect[0];
            if( ((qx0 < tst) && (qx1 > tst)) || ((qx0 > tst) && (qx1 < tst)) )
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
          if(dy)
          {
            real_type inv_alpha = dx/dy;
            tst = - rect[1];     //--- bottom side
            if( ((qy0 < tst) && (qy1 > tst)) || ((qy0 > tst) && (qy1 < tst)) )
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
            tst =  rect[1];     //--- top side
            if( ((qy0 < tst) && (qy1 > tst)) || ((qy0 > tst) && (qy1 < tst)) )
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

// OPENTISSUE_COLLISION_INTERSECT_INTERSECT_RECT_QUAD_EDGES_H
#endif
