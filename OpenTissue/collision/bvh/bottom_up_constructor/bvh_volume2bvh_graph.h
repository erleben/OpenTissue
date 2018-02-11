#ifndef OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOLUME2BVH_GRAPH_H
#define OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOLUME2BVH_GRAPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_qhull.h>  //Needed for delaunay triangulation
#include <OpenTissue/collision/gjk/gjk.h>  // Needed for collision testing

#include <cassert>
#include <vector>
#include <map>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Volume to BVH graph converter.
    *
    * Given a set of volumes, this class contains different methods for
    * building an initial graph, that can be used for a bottom-up
    * construction of a BVH.
    */
    template<typename graph_type>
    class Volume2BVHGraph
    {
    public:

      typedef typename graph_type::node_ptr_type    node_ptr_type;
      typedef typename graph_type::edge_ptr_type    edge_ptr_type;
      typedef typename graph_type::volume_type      volume_type;

    public:

      /**
      * Use Delaunay triangulation of volume centers to created edges in graph.
      *
      * @param begin
      * @param end
      * @param graph
      */
      template<typename volume_iterator>
      void delaunay(volume_iterator & begin,volume_iterator & end,graph_type & graph)
      {
        int N = 0;
        std::map<unsigned int,node_ptr_type > node_lookup;
        for(volume_iterator volume=begin;volume!=end;++volume)
        {
          node_ptr_type node = graph.insert( (*volume) );
          node_lookup[N] = node;
          ++N;
        }
        assert(N>3);

        int dim = 3;
        coordT *coords = new coordT[dim*N];
        boolT ismalloc = False;
        char flags[] = "qhull v QJ";
        FILE *outfile = stdout;
        FILE *errfile = stderr;
        int exitcode;
        int j = 0;

        for(volume_iterator volume=begin;volume!=end;++volume)
        {
          coords[j++] = volume->center()(0);
          coords[j++] = volume->center()(1);
          coords[j++] = volume->center()(2);
        }

        exitcode= qh_new_qhull (dim, N, coords, ismalloc, flags, outfile, errfile);
        if(!exitcode)
        {
          vertexT *vertex;
          facetT *neighbor,**neighborp;
          qh visit_id++;
          bool ** exist = new bool*[N];
          for(int i=0;i<N;++i)
          {
            exist[i] = new bool[N];
            for(int j=0;j<N;++j)
              exist[i][j] = false;
          }
          FORALLvertices
          {
            vertex->visitid = qh visit_id;
            unsigned int idA = qh_pointid (vertex->point);
            FOREACHneighbor_(vertex)
            {
              if(neighbor->visitid != qh visit_id)
              {
                neighbor->visitid = qh visit_id;
                vertexT *vertex, **vertexp;
                FOREACHvertex_(neighbor->vertices)
                {
                  if(vertex->visitid != qh visit_id)
                  {
                    unsigned int idB = qh_pointid (vertex->point);
                    if(!exist[idA][idB])
                    {
                      exist[idA][idB] = true;
                      exist[idB][idA] = true;
                      node_ptr_type nodeA = node_lookup[idA];
                      node_ptr_type nodeB = node_lookup[idB];
                      graph.insert(nodeA,nodeB);
                    }
                  }
                }
              }
            }
          }
          for(int i=0;i<N;++i)
          {
            delete [] exist[i];
          }
          delete [] exist;
        }
        qh_freeqhull(!qh_ALL);
        int curlong, totlong;
        qh_memfreeshort (&curlong, &totlong);
        if (curlong || totlong)
          fprintf (errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
        delete [] coords;
      }


      /**
      *
      * @param begin
      * @param end
      * @param graph
      */
      template<typename volume_iterator>
      void all_pair(volume_iterator & begin,volume_iterator & end,graph_type & graph)
      {
        int N = 0;
        std::map<unsigned int,node_ptr_type > node_lookup;
        for(volume_iterator volume=begin;volume!=end;++volume)
        {
          node_ptr_type node = graph.insert( (*volume) );
          node_lookup[N] = node;
          ++N;
        }
        for(int i=0;i<N;++i)
        {
          for(int j=(i+1);j<N;++j)
          {
            node_ptr_type  nodeA = node_lookup[i];
            node_ptr_type  nodeB = node_lookup[j];
            graph.insert(nodeA,nodeB);
          }
        }
      }

      /**
      *
      * @param volumes
      * @param graph
      */
      template<typename volume_iterator>
      void colliding(const volume_iterator & begin,const volume_iterator & end, graph_type & graph)
      {

        int N = 0;
        std::map<unsigned int,node_ptr_type > node_lookup;
        for(volume_iterator volume=begin;volume!=end;++volume)
        {
          node_ptr_type node = graph.insert( (*volume) );
          node_lookup[N] = node;
          ++N;
        }

        std::vector<volume_type> enlarged(N);
        int i = 0;
        for(volume_iterator volume=begin;volume!=end;++volume)
        {
          enlarged[i] = (*volume);
          enlarged[i].scale(1.25);
          ++i;
        }

        OpenTissue::gjk::obsolete::detail::GJK< math::Vector3<double> > gjk;

        for(int i=0;i<N;++i)
        {
          for(int j=(i+1);j<N;++j)
          {
            volume_type * volA = &enlarged[i];
            volume_type * volB = &enlarged[j];
            math::Vector3<double> v = volA->center() - volB->center();
            bool intersecting = gjk.is_intersecting(*volA,*volB,v);
            if(intersecting)
            {
              node_ptr_type nodeA = node_lookup[i];
              node_ptr_type nodeB = node_lookup[j];
              graph.insert(nodeA,nodeB);
            }
          }
        }
      }

    };

  } // namespace bvh

} // namespace OpenTissue

//OPENTISSUE_BVH_BOTTOM_UP_CONSTRUCTOR_BVH_VOLUME2BVH_GRAPH_H
#endif
