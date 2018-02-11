#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GRAHAM_SCAN_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GRAHAM_SCAN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace OpenTissue
{
  namespace geometry
  {

    template<typename vector3_iterator,typename vector3_container, typename vector3_type>
    void graham_scan(vector3_iterator begin,vector3_iterator end, vector3_container & hull, vector3_type const & n)
    {
      typedef typename vector3_type::value_type        real_type;

      assert(begin!=end || !"graham_scan(...) Empty range of points");

      hull.clear();
      unsigned int count = std::distance(begin,end);
      std::vector<vector3_type *> points(count);
      {
        unsigned int idx = 0;
        for(vector3_iterator v = begin;v!=end;++v,++idx)
        {
          points[idx] = &(*v);
        }
      }

      //--- Make unique by swapping redundant vertices to end and decreasing count.
      for (unsigned int u = 0; u<count-1; ++u)
      {
        for(unsigned int i=u+1; i<count; ++i)
        {
          if( *(points[i]) == *(points[u]) )
          {
            count--;
            std::swap(points[i],points[count]);
          }
        }
      }

      //--- Now perform classical graham scan!!!, first find left-bottom most corner
      unsigned int used = 0;
      unsigned int j=0;
      for(unsigned int i=1; i<count; ++i)
      {
        if(  *(points[i]) < *(points[j]) )
          j = i;
      }
      std::swap(points[j],points[0]);
      ++used;

      vector3_type a,b,c;
      unsigned int last = 0;  //--- index into the points array of the last detected convex hull vertex
      unsigned int best = 0;  //--- index into the points array of the best known next convex hull vertex
      do
      {
        best = used%count;
        a = *(points[best]) - *(points[last]);

        for(unsigned int i=(used+1); i<(count+1); ++i)
        {
          unsigned int k = ((i)%count);
          if(is_zero(a))
          {
            best = k;
            a = *(points[best]) - *(points[last]);
            continue;
          }

          b = *(points[k]) - *(points[last]);
          c = (a % b);
          real_type dot = n * c;
          if((dot < 0 && b*b>a*a) || dot < -0.001)
          {
            best = k;
            a = *(points[best]) - *(points[last]);
          }
          // TODO: Comparing floats with ==
          else if( (dot==0) && (a*b>0) && (b*b>a*a) )
          {
            best = k;
            a = *(points[best]) - *(points[last]);
          }
        }
        if(best==0)//--- we are back to the first convex hull vertex
          break;

        std::swap(points[used],points[best]);
        last = used;++used;

      }while(best!=0);

      //--- copy convex hull vertices into hull-container
      for(unsigned int idx = 0;idx<used;++idx)
        hull.push_back(   *(points[idx]) );
    }

    template<typename vector3_iterator,typename vector3_container>
    void graham_scan(vector3_iterator begin,vector3_iterator end, vector3_container & hull)
    {
      typedef typename vector3_container::value_type   vector3_type;
      graham_scan(begin,end,hull,vector3_type(0,0,1));
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GRAHAM_SCAN_H
#endif
