#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVEX_HULL_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVEX_HULL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_qhull.h>

namespace OpenTissue
{
  namespace mesh
  {


    template<
      typename mesh_type
      , typename vector3_iterator
    >
    void convex_hull(vector3_iterator begin, vector3_iterator end, mesh_type & mesh,bool print_summary = false)
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;

      typedef typename mesh_type::vertex_handle                     vertex_handle;

      mesh.clear();
      size_t N =  std::distance(begin,end);
      int dim = 3;
      coordT *coords = new coordT[dim*N];
      boolT ismalloc = False;
      char flags[] = "qhull QJ";//QJ
      FILE *outfile = print_summary ? stdout : 0;
      FILE *errfile = stderr;
      //FILE *outfile = 0;
      //FILE *errfile = 0;
      int exitcode;
      int j = 0;
      for(vector3_iterator p=begin;p!=end;++p)
      {
        coords[j++] = (*p)(0);
        coords[j++] = (*p)(1);
        coords[j++] = (*p)(2);
      }
      std::vector<vertex_handle> handles(N);
      int iN = boost::numeric_cast<int,size_t>(N);
      exitcode= qh_new_qhull (dim,  iN, coords, ismalloc, flags, outfile, errfile);
      if(!exitcode)
      {
        facetT *facet;
        vertexT *vertex, **vertexp;
        qh visit_id++;
        FORALLfacets
        {
          facet->visitid = qh visit_id;
          std::list<vertex_handle> tmp;
          FOREACHvertex_(facet->vertices)
          {
            int idx = qh_pointid(vertex->point);
            if(handles[idx].is_null())
              handles[idx] = mesh.add_vertex( vector3_type( vertex->point[0],vertex->point[1],vertex->point[2] )  );
            tmp.push_back(handles[idx]);
          }
          if(facet->toporient)//--- Make sure coords are in CCW order
            tmp.reverse();
          mesh.add_face(tmp.begin(),tmp.end());
        }
      }
      qh_freeqhull(!qh_ALL);
      int curlong, totlong;
      qh_memfreeshort (&curlong, &totlong);
      if (curlong || totlong)
        fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
      delete [] coords;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVEX_HULL_H
#endif
