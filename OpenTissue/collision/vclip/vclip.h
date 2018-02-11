#ifndef OPENTISSUE_COLLISION_VCLIP_VCLIP_H
#define OPENTISSUE_COLLISION_VCLIP_VCLIP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/vclip/vclip_mesh.h>
#include <OpenTissue/core/math/math_coordsys.h>

#include <vector>
#include <cmath>

namespace OpenTissue
{
  namespace vclip
  {

    /**
    * Voronoi Clip Algorithm.
    * One have to setup a vclip mesh in order to use this collision algorithm. An
    * vclip mesh is a polygon mesh annotated with voronoi planes of the external
    * voronoi regions of each mesh feature.
    *
    * An utility tool have been created for setting up a vclip mesh. An example:
    *
    *   polymesh::PolyMesh<....>   my_usual_mesh; 
    *   mesh::obj_read(my_favorite_polymesh_file,my_usual_mesh);
    *   vclip_mesh_type  my_vclip_mesh;
    *   convert(my_usual_mesh,my_vclip_mesh);
    *
    * Now you can simply invoke the run method on this class to do a collision test
    * between two vclip meshes.
    *
    *
    *
    * Here is a little longer example. First at startup one initializes the geometries
    *
    *   OpenTissue::vclip::vclip_mesh_type A;
    *   OpenTissue::vclip::vclip_mesh_type B;
    *
    *   polymesh::PolyMesh<> tmp;
    *   mesh::make_box(1.0,1.0,1.0, tmp);
    *   mesh::make_box(1.0,1.0,1.0, tmp);
    *
    *   OpenTissue::vclip::convert(tmp,A);
    *   OpenTissue::vclip::convert(tmp,B);
    *
    * Then during runtime, one determines where the geometries are places in the world
    *
    *   coordsys_type  Awcs = coordsys_type( vector3_type(-1.0,0.0,0.0), quaternion_type());   
    *   coordsys_type  Bwcs = coordsys_type( vector3_type( 1.0,0.0,0.0), quaternion_type());      
    *   coordsys_type AtoB = model_update(Awcs,Bwcs);
    *   coordsys_type BtoA = inverse(AtoB);
    *
    * Finally one can query the vclip algorithm about cloest features and separation distance.
    *
    *   OpenTissue::vclip::Feature * seed_a = &( * ea );
    *   OpenTissue::vclip::Feature * seed_b = &( * fb );
    *   real_type distance = vclip.run(A,B,AtoB,BtoA,&seed_a,&seed_b,p_a,p_b,max_iterations);
    */
    class VClip
    {
    public:

      typedef vclip_mesh_type::vertex_halfedge_circulator  vertex_halfedge_circulator;
      typedef vclip_mesh_type::face_halfedge_circulator    face_halfedge_circulator;
      typedef vclip_mesh_type::face_iterator               face_iterator;
      typedef vclip_mesh_type::vertex_type                 vertex_type;
      typedef vclip_mesh_type::halfedge_type               halfedge_type;
      typedef vclip_mesh_type::face_type                   face_type;
      typedef vertex_type *                                vertex_pointer;
      typedef halfedge_type *                              halfedge_pointer;
      typedef face_type *                                  face_pointer;
      typedef Feature *                                    feature_pointer;
      typedef halfedge_type::plane_type                    plane_type;

      typedef vclip_mesh_type::math_types					         math_types;
      typedef math_types::vector3_type					           vector3_type;
      typedef math_types::real_type						             real_type;

      typedef unsigned char                                state_type;
      typedef enum {PENETRATION,CONTINUE,DISJOINT}         result_type;  ///< used to return the result of a state in the v-clip algorithm.
      typedef OpenTissue::math::CoordSys<real_type>        coordsys_type;

    public:

      /**
      * State type constants.
      */
      const static state_type vertex_vertex_state =  0;  //--- (typeA << 2 | typeB) :  00 00
      const static state_type vertex_edge_state   =  1;  //--- (typeA << 2 | typeB) :  00 01
      const static state_type edge_vertex_state   =  4;  //--- (typeA << 2 | typeB) :  01 00
      const static state_type vertex_face_state   =  2;  //--- (typeA << 2 | typeB) :  00 10
      const static state_type face_vertex_state   =  8;  //--- (typeA << 2 | typeB) :  10 00
      const static state_type edge_edge_state     =  5;  //--- (typeA << 2 | typeB) :  01 01
      const static state_type edge_face_state     =  6;  //--- (typeA << 2 | typeB) :  01 10
      const static state_type face_edge_state     =  9;  //--- (typeA << 2 | typeB) :  10 01

      /**
      * Constants used to describe an edges relative
      * position to a edge-face voronoi plane.
      *
      * @see m_code
      */
      const static int INSIDE  = 0x000;
      const static int OUTSIDE = 0x001;
      const static int MIN     = 0x002;
      const static int MAX     = 0x004;

    public:

      feature_pointer m_vertexA;   ///< Temporary Storage. All these are used to pass along information about the two current withness features to each state handler.
      feature_pointer m_vertexB;
      feature_pointer m_vertex;
      feature_pointer m_edgeA;
      feature_pointer m_edgeB;
      feature_pointer m_edge;
      feature_pointer m_face;

      real_type m_distance; ///< The Separation Distance.

      /**
      * Edge Clip Code.
      * When an edge is clipped against the edge-face voronoi
      * planes along the boundary of a face then this data
      * structure is used to keep track of the edges position
      * relatively to each voronoi plane.
      *
      * That is an edge can be either inside, outside,
      * minimum-cross or maximum-cross of a voronoi plane.
      */
      std::vector<int>       m_code;

      /**
      * Edge Clipping Parameter Storage.
      * This array is used in a similar way to the
      * code-array except this time we store the
      * actual value of the clipping parameter.
      */
      std::vector<real_type> m_parameter;

    private:

      result_type vertex_vertex(
        coordsys_type const & AtoB
        , coordsys_type const & BtoA
        , vector3_type & cpA
        , vector3_type & cpB
        )
      {
        vertex_pointer vA = static_cast<vertex_pointer>(m_vertexA);
        vertex_pointer vB = static_cast<vertex_pointer>(m_vertexB);

        //--- Test if vB lies in VR(vA)
        vector3_type p = vB->m_coord;
        BtoA.xform_point(p);
        vertex_halfedge_circulator hA(*vA),end;
        for(;hA!=end;++hA)
        {
          if(hA->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(p)<0)
          {
            m_vertexA = &(*hA);
            return CONTINUE;
          }
        }

        //--- Test if vA lies in VR(vB)
        p = vA->m_coord;
        AtoB.xform_point(p);
        vertex_halfedge_circulator hB(*vB);
        for(;hB!=end;++hB)
        {
          if(hB->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(p)<0)
          {
            m_vertexB = &(*hB);
            return CONTINUE;
          }
        }

        cpA = vA->m_coord;
        cpB = vB->m_coord;
        vector3_type diff = cpA-cpB;
        m_distance = std::sqrt( diff*diff );
        //--- VA and VB could be in touching contact.
        return (m_distance>0)?DISJOINT:PENETRATION;
      };


      result_type vertex_face(
        coordsys_type const & VtoF
        , vector3_type & cpV
        , vector3_type & cpF
        , vclip_mesh_type * mesh
        )
      {
        vertex_pointer v = static_cast<vertex_pointer>(m_vertex);
        face_pointer   f = static_cast<face_pointer>(m_face);

        vector3_type p;
        real_type d    = static_cast<real_type>(0.0);
        real_type dmin = static_cast<real_type>(0.0);
        bool updated = false;

        //--- Test if V lies inside all VP's along F's boundary
        //--- if not then we have discovered an exclusion.
        p =  v->m_coord;
        VtoF.xform_point(p);

        {
          face_halfedge_circulator h(*f),hend;
          for(;h!=hend;++h)
          {
            if((d = h->m_voronoi_plane_EF.signed_distance(p))<dmin)
            {
              m_face = &(*(h));
              dmin = d;
              updated = true;
            }
          }
        }
        //--- Handle exclusion if it was found, note that by searching for
        //--- minimum distance VP, we avoid the unpleasant characteristica
        //--- of edge-face voronoi planes.
        if(updated)
        {
          return CONTINUE;
        }
        //--- Test if V lies on F, If we discover this at this stage
        //--- then we know V must be in touching contact with F.
        if((d = f->m_plane.signed_distance(p)) == 0)
        {
          cpV = v->m_coord;
          cpF = p;
          return PENETRATION;
        }
        //--- By now we will look for any incident edges to V, which
        //--- could descrease the distance between V and E.
        //--- That is we are looking for an edge pointing towards F.
        //--- If such an edge exist it must be a better candidate
        //--- than V.
        //---
        //--- This part of the algorithm corresponds to testing
        //--- whatever F in VR(V).
        vector3_type q;

        {
          vertex_halfedge_circulator h(*v),hend;
          for(;h!=hend;++h)
          {
            q = h->get_destination_iterator()->m_coord;
            VtoF.xform_point(q);
            real_type d2 = f->m_plane.signed_distance(q);
            if( ((d<0)&&(d2>d)) || ((d>0)&&(d2<d)) )
            {
              m_vertex = &(*h);
              return CONTINUE;
            }
          }
        }
        //--- Test if V lies above F's plane. With this test
        //--- we have actually completed the V in VR(F) test.
        if(d>0)
        {
          m_distance = d;
          cpV = v->m_coord;
          cpF = p - f->m_plane.n()*d;
          return DISJOINT;
        }
        //--- By now we know V lies on the backside of F, but
        //--- this does not necessarily mean that there is a
        //--- penetration. V could lie on the opposite side
        //--- of F's mesh.
        //---
        //--- This case is also called local minimum, because
        //--- all the incident edges of V would increase the
        //--- distance between V and F (this was tested in
        //--- the last for-loop).
        {
          vclip_mesh_type::face_iterator ft   = mesh->face_begin();
          vclip_mesh_type::face_iterator fend = mesh->face_end();
          for(;ft!=fend;++ft)
          {
            real_type d2 = ft->m_plane.signed_distance(p);
            if(d2>d)
            {
              d = d2;
              m_face =  &(*ft) ;
            }
          }
        }
        //--- Test if we succeded in escaping the local minimum.
        if(d>0)
        {
          return CONTINUE;
        }
        //--- At this point we know we could not escape the local
        //--- minimum, and therefore we also know that there must
        //--- be a penetration.
        m_distance = d;
        cpV = v->m_coord;
        cpF = p - f->m_plane.n()*d;
        return PENETRATION;
      };


      result_type vertex_edge(
        coordsys_type const & VtoE
        , coordsys_type const & EtoV
        , vector3_type & cpV
        , vector3_type & cpE
        )
      {
        vertex_pointer   v = static_cast<vertex_pointer>(m_vertex);
        halfedge_pointer e = static_cast<halfedge_pointer>(m_edge);
        vector3_type p;

        //--- Test if V in VR(E)
        p = v->m_coord;
        VtoE.xform_point(p);
        if(e->m_voronoi_plane_VE.signed_distance(p)>0)
        {
          m_edge = &(*(e->get_destination_iterator()));
          return CONTINUE;
        }
        if(e->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(p)>0)
        {
          m_edge = &(*(e->get_origin_iterator()));
          return CONTINUE;
        }
        if(e->m_voronoi_plane_EF.signed_distance(p)>0)
        {
          m_edge = &(*(e->get_face_iterator()));
          return CONTINUE;
        }
        if(e->get_twin_iterator()->m_voronoi_plane_EF.signed_distance(p)>0)
        {
          m_edge = &(*(e->get_twin_iterator()->get_face_iterator()));
          return CONTINUE;
        }
        //--- Test if E in VR(V), this is done by first clipping
        //--- E against all VP's of VR(V)
        vector3_type origin      = e->get_origin_iterator()->m_coord;
        vector3_type destination = e->get_destination_iterator()->m_coord;
        EtoV.xform_point(origin);
        EtoV.xform_point(destination);

        real_type min_t = static_cast<real_type>(0.0);
        real_type max_t = static_cast<real_type>(1.0);
        feature_pointer min_neighbor = 0;
        feature_pointer max_neighbor = 0;
        real_type t;

        vertex_halfedge_circulator h(*v),hend;
        for(;h!=hend;++h)
        {
          real_type dO = h->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(origin);
          real_type dD = h->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(destination);
          if(dO >= 0)
          {
            if(dD>=0)
              continue;
            if((t = dO/(dO-dD))<max_t)
            {
              max_t = t;
              max_neighbor  = &(*h);
              if(max_t<min_t)//--- Compound exclusion test
                break;
            }
          }
          else //--- DO < 0
          {
            if(dD<0)//--- Simply exclusion
            {
              min_neighbor = max_neighbor = &(*h);
              break;
            }
            if((t = dO/(dO-dD))>min_t)
            {
              min_t = t;
              min_neighbor = &(*h);
              if(min_t>max_t)//--- Compound exclusion test
                break;
            }
          }
        }

        //--- Handle simply or compound exclusion
        if( h!=hend && (min_neighbor == max_neighbor))
        {
          m_vertex = min_neighbor;
          return CONTINUE;
        }

        //---
        //--- Edge crosses VR(V), Analyse derivatives at boundaries
        //---
        vector3_type offset;
        vector3_type segment = destination - origin;

        if((min_neighbor)||(max_neighbor))
        {
          if(min_neighbor)
          {
            offset = origin + segment*min_t;
            offset -= v->m_coord;
            if(is_zero(offset))
            {
              cpV = v->m_coord;
              cpE = p;
              return PENETRATION;
            }
            if(offset*segment>0)
            {
              m_vertex = min_neighbor;
              return CONTINUE;
            }
          }
          if(max_neighbor)
          {
            offset = origin + segment*max_t;
            offset -= v->m_coord;
            if(is_zero(offset))
            {
              cpV = v->m_coord;
              cpE = p;
              return PENETRATION;
            }
            if(offset*segment<0)
            {
              m_vertex = max_neighbor;
              return CONTINUE;
            }
          }
        }
        //--- By now we know that E must be completely within VR(V).
        //--- We conclude that the features must be disjoint.
        cpV = v->m_coord;
        offset =  p - e->get_origin_iterator()->m_coord;
        //--- This is nothing more than the projection of V onto E
        cpE = e->get_origin_iterator()->m_coord   +   e->m_u * (offset * (e->m_u));
        vector3_type diff = cpE - p;
        m_distance = std::sqrt(diff*diff);
        return DISJOINT;
      };


      result_type edge_edge_subtest(
        vector3_type const & origin
        , vector3_type const & destination
        , vector3_type const & segment
        , vector3_type & cp
        )
      {
        halfedge_pointer  e = static_cast<halfedge_pointer>(m_edge);
        //--- First we are going to clip against E's vertex voronoi planes.
        real_type min_t = static_cast<real_type>(0.0);
        real_type max_t = static_cast<real_type>(1.0);     
        feature_pointer min_neighbor = 0;
        feature_pointer max_neighbor = 0;

        //--- Clip against VP(E,E.O)
        real_type dO = - e->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(origin);
        real_type dD = - e->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(destination);
        if(dO < 0)
        {
          if(dD<0)//--- Simple Exclusion
          {
            m_edge = &(*e->get_origin_iterator());
            return CONTINUE;
          }
          min_t = dO/(dO-dD);
          min_neighbor = &(*e->get_origin_iterator());
        }
        else if (dD < 0)
        {
          max_t = dO/(dO-dD);
          max_neighbor = &(*e->get_origin_iterator());
        }
        //--- Clip against VP(E,E.D)
        dO = - e->m_voronoi_plane_VE.signed_distance(origin);
        dD = - e->m_voronoi_plane_VE.signed_distance(destination);
        if(dO < 0)
        {
          if(dD<0)//--- Simple Exclusion
          {
            m_edge = &(*e->get_destination_iterator());
            return CONTINUE;
          }
          min_t = dO/(dO-dD);
          min_neighbor = &(*(e->get_destination_iterator()));
        }
        else if (dD < 0)
        {
          max_t = dO/(dO-dD);
          max_neighbor = &(*e->get_destination_iterator());
        }
        //--- Store results for later usage
        feature_pointer vMinNbr = min_neighbor;
        feature_pointer vMaxNbr = max_neighbor;
        real_type       vmin    = min_t;
        real_type       vmax    = max_t;

        //--- Now we are going to clip against E's face voronoi planes.
        int i;
        real_type t;
        plane_type * plane = 0;
        feature_pointer nbr = 0;
        vector3_type point;
        for(i=0;i<2;++i)
        {
          if(i==1)
          {
            plane = &(e->get_twin_iterator()->m_voronoi_plane_EF);
            nbr   = &(*(e->get_twin_iterator()->get_face_iterator()));
          }
          else
          {
            plane = &(e->m_voronoi_plane_EF);
            nbr   = &(*(e->get_face_iterator()));
          }
          dO = - plane->signed_distance(origin);
          dD = - plane->signed_distance(destination);
          if(dO<0)
          {
            if(dD<0)
            {
              //--- Completely clipped by face plane
              //--- This could be a simple exclusion, however
              //--- we need to check vertex derivatives in order
              //--- to determine so.
              if(vMinNbr)
              {
                point = origin + segment*vmin;
                point -= static_cast<vertex_pointer>(vMinNbr)->m_coord;
                if(is_zero(point))
                {
                  cp = static_cast<vertex_pointer>(min_neighbor)->m_coord;
                  return PENETRATION;
                }
                if(point*segment>0)
                {
                  m_edge = &(*vMinNbr);
                  return CONTINUE;
                }
              }
              if(vMaxNbr)
              {
                point = origin + segment*vmax;
                point -= static_cast<vertex_pointer>(vMaxNbr)->m_coord;
                if(is_zero(point))
                {
                  cp = static_cast<vertex_pointer>(max_neighbor)->m_coord;
                  return PENETRATION;
                }
                if(point*segment<0)
                {
                  m_edge = &(*vMaxNbr);
                  return CONTINUE;
                }
              }
              m_edge = nbr;
              return CONTINUE;
            }
            else if((t =dO/(dO-dD))>min_t)
            {
              min_t = t;
              min_neighbor = nbr;
              if(min_t>max_t)
                break;//--- Compound exclusion detected.
            }
          }
          else if(dD < 0)
          {
            if((t = dO/(dO-dD))<max_t)
            {
              max_t = t;
              max_neighbor = nbr;
              if(max_t<min_t)
                break;//--- Compound exclusion detected.
            }
          }
        }
        //--- Handle any compound exclusion if one has been discovered.
        //--- Note this will only be the case if the for-loop was
        //--- broken, which means i<2.
        real_type dmin;
        if(i<2)
        {
          if(min_neighbor->m_type==Feature::VERTEX)
          {
            point = origin + segment*min_t;
            point -= static_cast<vertex_pointer>(min_neighbor)->m_coord;
            if(is_zero(point))
            {
              cp = static_cast<vertex_pointer>(min_neighbor)->m_coord;
              return PENETRATION;
            }
            //--- hmm, why >=0? Well if '=' case then e(t) lies
            //--- in vertex-edge plane and vertices have precedence
            //--- over faces. In any case there is nothing illegal
            //--- in this, since the location of the best known
            //--- minimum distance gets better.
            m_edge = (point*segment>=0) ? min_neighbor : max_neighbor;
            return CONTINUE;
          }
          if(max_neighbor->m_type==Feature::VERTEX)
          {
            point = origin + segment*max_t;
            point -= static_cast<vertex_pointer>(max_neighbor)->m_coord;
            if(is_zero(point))
            {
              cp = static_cast<vertex_pointer>(max_neighbor)->m_coord;
              return PENETRATION;
            }
            m_edge = (point * segment<=0) ? max_neighbor : min_neighbor;
            return CONTINUE;
          }
          //--- If we reach this point in the algorithm then we
          //--- know the edge must be completely clipped by both
          //--- face planes.
          //---
          //--- That is compound excluded by face planes only.
          //---
          //--- DEVELOPER's NOTE in Mirtich's paper this is
          //--- explained as the edge passes beneath E piercing
          //--- the planes of E's neighboring faces. It is also
          //--- stated that the derivative-test algorithm would
          //--- handle the case correctly. However the code below
          //--- was taken from Mirtich's own implementation and
          //--- it does not bare any resemblance to the
          //--- derivative-test algorithm? What is going on?
          dO = static_cast<face_pointer>(min_neighbor)->m_plane.signed_distance(origin);
          dD = static_cast<face_pointer>(min_neighbor)->m_plane.signed_distance(destination);
          dmin = dO + min_t*(dD-dO);
          if(dmin==0)
          {
            cp = origin + segment*min_t;
            return PENETRATION;
          }
          m_edge = (dmin>0) ? ((dO < dD) ? min_neighbor : max_neighbor) : ((dO > dD) ? min_neighbor : max_neighbor);
          return CONTINUE;
        }
        //--- Edge might intersect voronoi region, if this is
        //--- the case we will analyse the derivatives at the
        //--- crossings in order to determine how to proceed.
        real_type dmax;
        if(min_neighbor)
        {
          if(min_neighbor->m_type==Feature::FACE)
          {
            dO = static_cast<face_pointer>(min_neighbor)->m_plane.signed_distance(origin);
            dD = static_cast<face_pointer>(min_neighbor)->m_plane.signed_distance(destination);
            dmin = dO + min_t*(dD-dO);
            dmax = (max_neighbor) ? dO + max_t*(dD-dO) : dD;
            if(dmin==0)
            {
              cp = origin + segment*min_t;
              return PENETRATION;
            }
            if(((dmin>0)&&(dmin<dmax))||((dmin<0)&&(dmin>dmax)))
            {
              m_edge = min_neighbor;
              return CONTINUE;
            }
          }
          else //--- minimum neighbor must be a vertex
          {
            point = origin + segment*min_t;
            point -= static_cast<vertex_pointer>(min_neighbor)->m_coord;
            if(is_zero(point))
            {
              cp = static_cast<vertex_pointer>(min_neighbor)->m_coord;
              return PENETRATION;
            }
            if(point*segment>0)
            {
              m_edge = min_neighbor;
              return CONTINUE;
            }
          }
        }
        if(max_neighbor)
        {
          if(max_neighbor->m_type==Feature::FACE)
          {
            dO = static_cast<face_pointer>(max_neighbor)->m_plane.signed_distance(origin);
            dD = static_cast<face_pointer>(max_neighbor)->m_plane.signed_distance(destination);
            dmin = (min_neighbor) ? dO + min_t*(dD-dO) : dO;
            dmax = dO + max_t*(dD-dO);
            if(dmax==0)
            {
              cp = origin + segment*max_t;
              return PENETRATION;
            }
            if(((dmax>0)&&(dmax<dmin))||((dmax<0)&&(dmax>dmin)))
            {
              m_edge = max_neighbor;
              return CONTINUE;
            }
          }
          else //--- maximum neighbor must be a vertex
          {
            point = origin + segment*max_t;
            point -= static_cast<vertex_pointer>(max_neighbor)->m_coord;
            if(is_zero(point))
            {
              cp = static_cast<vertex_pointer>(max_neighbor)->m_coord;
              return PENETRATION;
            }
            if(point * segment<0)
            {
              m_edge = max_neighbor;
              return CONTINUE;
            }
          }
        }
        //--- Well there is only one possibility
        //--- left. Edges must be disjoint.
        return DISJOINT;
      };

      result_type edge_edge(
        coordsys_type const & AtoB
        , coordsys_type const & BtoA
        , vector3_type & cpA
        , vector3_type & cpB
        )
      {
        result_type result;

        halfedge_pointer eA = static_cast<halfedge_pointer>(m_edgeA);
        halfedge_pointer eB = static_cast<halfedge_pointer>(m_edgeB);

        //--- Clip EA against VR(EB)
        vector3_type origin      = eA->get_origin_iterator()->m_coord;
        vector3_type destination = eA->get_destination_iterator()->m_coord;
        AtoB.xform_point(origin);
        AtoB.xform_point(destination);
        vector3_type segment = destination - origin;

        m_edge = m_edgeB;
        if((result=edge_edge_subtest(origin,destination,segment,cpB))==PENETRATION)
        {
          cpA = cpB;
          BtoA.xform_point(cpA);
        }
        m_edgeB = m_edge;
        if(result!=DISJOINT)
        {
          return result;
        }
        //---
        //--- Clip EB against VR(EA)
        //---
        origin      = eB->get_origin_iterator()->m_coord;
        destination = eB->get_destination_iterator()->m_coord;
        BtoA.xform_point(origin);
        BtoA.xform_point(destination);
        segment = destination - origin;

        m_edge = m_edgeA;
        if((result=edge_edge_subtest(origin,destination,segment,cpA))==PENETRATION)
        {
          cpB = cpA;
          AtoB.xform_point(cpB);
        }
        m_edgeA = m_edge;
        if(result!=DISJOINT)
        {
          return result;
        }
        //--- If we reach this point then we know edges must be disjoint.
        //--- So we simply compute closest points and distance before
        //--- we return.
        real_type lambda;
        vector3_type p;
        vector3_type h;
        vector3_type h2;
        vector3_type xdir;

        xdir = eB->m_u;
        BtoA.xform_vector(xdir);

        real_type k = xdir * eA->m_u;
        h = origin - eA->get_origin_iterator()->m_coord;
        h2 = eA->m_u -  xdir*k;
        real_type num = h * h2;
        real_type denom = 1 - k*k;
        if(denom==0)
        {
          if(num>0)
          {
            cpA = eA->get_destination_iterator()->m_coord;
          }
          else
          {
            cpA = eA->get_origin_iterator()->m_coord;
          }
        }
        else
        {
          lambda = num/denom;
          if(lambda<0)
          {
            lambda = static_cast<real_type>(0.0);
          }
          else if(lambda > eA->m_length)
          {
            lambda = eA->m_length;
          }
          cpA = eA->get_origin_iterator()->m_coord + eA->m_u * lambda;
        }

        p = cpA;
        AtoB.xform_point(p);
        h = p - eB->get_origin_iterator()->m_coord;
        lambda = h * eB->m_u;
        cpB = eB->get_origin_iterator()->m_coord +  eB->m_u*lambda;
        vector3_type diff = cpB  - p;
        m_distance = std::sqrt(diff*diff);
        return DISJOINT;
      };

      result_type edge_face(
        coordsys_type const & EtoF
        , vector3_type & cpE
        , vector3_type & cpF
        )
      {
        halfedge_pointer e = static_cast<halfedge_pointer>(m_edge);
        face_pointer f = static_cast<face_pointer>(m_face);

        //--- Just in case we have not allocated enough memory,
        //--- expand arrays. This is hopefulle very unlikely
        //--- to happen. But better safe than sorry.
        unsigned int N = valency(*f);
        m_code.resize(N);
        m_parameter.resize(N);

        vector3_type point;
        vector3_type origin;
        vector3_type destination;
        vector3_type segment;

        origin = e->get_origin_iterator()->m_coord;
        EtoF.xform_point(origin);
        destination = e->get_destination_iterator()->m_coord;
        EtoF.xform_point(destination);
        segment = destination - origin;

        //--- Clip edge against the edge-face voronoi
        //--- planes of VR(F).
        halfedge_pointer minCn = 0;
        halfedge_pointer maxCn = 0;
        halfedge_pointer chopCn = 0;

        real_type min_t = 0.0f;
        real_type max_t = 1.0f;
        real_type dO,dD,t;

        face_halfedge_circulator h(*f),hend;
        for(int j = 0;h!=hend;++h,++j)
        {
          h->m_tag = j;

          dO = h->m_voronoi_plane_EF.signed_distance(origin);
          dD = h->m_voronoi_plane_EF.signed_distance(destination);
          if(dO>=0)
          {
            if(dD>=0)
            {
              m_code[j] = INSIDE;
            }
            else //--- dD < 0
            {
              m_code[j] = MAX;
              if((m_parameter[j]=dO/(dO-dD))<max_t)
              {
                max_t = m_parameter[j];
                maxCn = &(*h);
              }
            }
          }
          else //--- dO < 0
          {
            if(dD<0)
            {
              m_code[j] = OUTSIDE;
              chopCn = &(*h);
            }
            else //--- dD >= 0
            {
              m_code[j] = MIN;
              if((m_parameter[j]=dO/(dO-dD))>min_t)
              {
                min_t = m_parameter[j];
                minCn = &(*h);
              }
            }
          }
        }
        //--- In case of compound or simple exclusion handle
        //--- these.
        //---
        //--- Fortunately both compound and simple exclusion
        //--- can be handled in the same way.


        halfedge_pointer curCn = 0;

        if(chopCn ||(min_t>max_t))
        {
          if(chopCn)
          {
            curCn = chopCn;
          }
          else
          {
            //--- Heuristic: Chosse minCn or maxCn, based
            //--- on which corresponding region contains
            //--- more of edge being clipped.
            curCn = ((min_t + max_t) > 1.0) ? minCn : maxCn;
          }
          halfedge_pointer prev = 0;
          halfedge_pointer next = curCn;
          halfedge_pointer s = 0;

          bool intersect = false;
          //--- We are going to scan along the perimeter of
          //--- F, in order to find the closest edge or vertex
          //--- on the boundary.
          while(next!=prev)
          {
            prev = curCn;
            curCn = next;
            s = curCn;
            vertex_pointer minv = 0;
            vertex_pointer maxv = 0;

            //--- Test edge plane, actually we have already
            //--- done, this, so we just retrieve our results.
            int i = curCn->m_tag;  // curCn.m_index;
            if(m_code[i]==INSIDE)
              break;
            else if(m_code[i]==OUTSIDE)
            {
              min_t = static_cast<real_type>(0.0);
              max_t = static_cast<real_type>(1.0);
            }
            else if(m_code[i]==MIN)
            {
              min_t = static_cast<real_type>(0.0);
              max_t = m_parameter[i];
            }
            else if(m_code[i]==MAX)
            {
              min_t = m_parameter[i];
              max_t = static_cast<real_type>(1.0);
            }
            //--- Test edge-vertex plane at origin vertex.
            dO = - s->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(origin);
            dD = - s->get_twin_iterator()->m_voronoi_plane_VE.signed_distance(destination);
            if(dO>=0)
            {
              if(dD<0)
              {
                if((t=dO/(dO-dD))<max_t)
                {
                  max_t = t;
                  maxv = static_cast<vertex_pointer>(&(*(s->get_origin_iterator())));
                  if(min_t>max_t)
                  {
                    if(intersect)
                      break;
                    next = static_cast<halfedge_pointer>(&(*(s->get_prev_iterator())) );
                    continue;
                  }
                }
              }
            }
            else//--- dO < 0
            { 
              if(dD<0)
              {
                next = static_cast<halfedge_pointer>( &(*( s->get_prev_iterator() ) ) );
                continue;
              }
              if((t=dO/(dO-dD))>min_t)
              {
                min_t = t;
                minv = static_cast<vertex_pointer> ( &(*( s->get_origin_iterator() )) );
                if(min_t>max_t)
                {
                  if(intersect)
                    break;
                  next = static_cast<halfedge_pointer>( &(*(s->get_prev_iterator())) );
                  continue;
                }
              }
            }

            //---
            //--- Test edge-vertex plane at destination vertex.
            //---
            dO = - s->m_voronoi_plane_VE.signed_distance(origin);
            dD = - s->m_voronoi_plane_VE.signed_distance(destination);
            if(dO>=0)
            {
              if(dD<0)
              {
                if((t=dO/(dO-dD))<max_t)
                {
                  max_t = t;
                  maxv = static_cast<vertex_pointer> ( &(*( s->get_destination_iterator() ) ) );
                  if(min_t>max_t)
                  {
                    if(intersect)
                      break;
                    next = static_cast<halfedge_pointer> ( &(*( s->get_next_iterator() ) ) );
                    continue;
                  }
                }
              }
            }
            else //--- dO < 0
            { 
              if(dD<0)
              {
                next = static_cast<halfedge_pointer> ( &(*(s->get_next_iterator())) );
                continue;
              }
              if((t=dO/(dO-dD))>min_t)
              {
                min_t = t;
                minv = static_cast<vertex_pointer> (  &(*( s->get_destination_iterator() ))  );
                if(min_t>max_t)
                {
                  if(intersect)
                    break;
                  next = static_cast<halfedge_pointer> (  &(*( s->get_next_iterator() )) );
                  continue;
                }
              }
            }
            //--- At this point in time we know we have found
            //--- an edge Voronoi region that's intersected.
            intersect = true;
            if(minv)
            {
              point = origin + segment* min_t;
              point -= minv->m_coord;
              if(point * segment>0)
              {
                next = ( &(*(s->get_origin_iterator()))==minv)?  &(*s->get_prev_iterator()) : &(*s->get_next_iterator());
                continue;
              }
            }
            if(maxv)
            {
              point = origin + segment*max_t;
              point -= maxv->m_coord;
              if(point*segment<0)
              {
                next = (  &(*s->get_destination_iterator()) ==maxv)? &(*s->get_next_iterator()) : &(*s->get_prev_iterator());
                continue;
              }
            }
            m_face = s;
            return CONTINUE;
          }
          m_face = ( &(*curCn->get_next_iterator()) == prev) ? &(*s->get_destination_iterator()) : &(*s->get_origin_iterator());
          return CONTINUE;
        }

        //--- At this stage we know that the edge intersects
        //--- the faces cone or lies completely within it.
        //---
        //--- We have however forgotten the face plane itself,
        //--- so before we continue, we need to test if we
        //--- piece this plane.
        dO = f->m_plane.signed_distance(origin);
        dD = f->m_plane.signed_distance(destination);

        real_type dmin = (minCn) ? (dO + min_t*(dD-dO)) : dO;
        real_type dmax = (maxCn) ? (dO + max_t*(dD-dO)) : dD;
        //--- At this point in the algorithm Mirtich has an error
        //--- in his implementation. If an edge has dmin=0 and
        //--- dmax<0 then it should be detected as penetrating,
        //--- but the flow of control:
        //---
        //---   if(dmin<=0){
        //---     if(dmax>=0)....
        //---   }else if(dmax<=0)......
        //---
        //--- Will never catch this case, so we have rewritten this.
        if((dmin==0)||((dmin<0)&&(dmax>0)))
        {
          m_distance = dmin;
          cpE = e->get_origin_iterator()->m_coord + e->m_u * (min_t * e->m_length);
          cpF = origin + segment*min_t;
          cpF -= f->m_plane.n()*dmin;
          return PENETRATION;
        }
        else if((dmax==0)||((dmin>0)&&(dmax<0)))
        {
          m_distance = dmax;
          cpE = e->get_origin_iterator()->m_coord + e->m_u * (max_t*e->m_length);
          cpF = origin +  segment*max_t;
          cpF -= f->m_plane.n()*dmax;
          return PENETRATION;
        }
        //--- Now we know that the edge does not pierce the face
        //--- plane, so we check derivatives in order to figure
        //--- out how to update F or E.
        //---
        //--- At this point we also know that dmin & dmax are
        //--- both positive or both negative
        if(((dmin>0)&&(dO<=dD))||((dmin<0)&&(dO>=dD)))
        {
          if(minCn)
          {
            m_face = minCn;
          }
          else
          {
            m_edge = &(*e->get_origin_iterator());
          }
        }
        else
        {
          if(maxCn)
          {
            m_face = maxCn;
          }
          else
          {
            m_edge = &(*e->get_destination_iterator());
          }
        }
        return CONTINUE;
      };

    public:

      /**
      * Run V-Clip algorithm.
      * When this method is invoked the v-clip algorithm is run
      * on the two specified polyhedra.
      *
      *
      *
      *
      *
      *
      * @param meshA   The mesh of object A (in model frame).
      * @param meshB   The mesh of object B (in model frame).
      * @param AtoB    A coordinate transform from model frame of object A to the model frame of object B.
      * @param BtoA    The inverse coordinate transform of AtoB.
      * @param seedA   A feature from the mesh of object A. This is used to seed the algorithm.
      * @param seedB   A feature from the mesh of object B. This is used to seed the algorithm.
      * @param cpA     Upon return this argument holds the closest point on A.
      * @param cpB     Upon return this argument holds the closest point on B.
      *
      *
      * @param max_iterations   This is an iterative algorithm, one can use this argument
      *                         to either do single steps (usefull for visualizing the
      *                         internal feature tracking) or simply guard against infinite
      *                         loops.
      *
      * @return        The separation distance if polyhedra are
      *                disjoint, if polyhedra are exactly touching
      *                the value is zero and in case of penetration
      *                the value is negative.
      */
      real_type run(
          vclip_mesh_type const & meshA
        , vclip_mesh_type const & meshB
        , coordsys_type const & AtoB
        , coordsys_type const & BtoA
        , feature_pointer * seedA
        , feature_pointer * seedB
        , vector3_type & cpA
        , vector3_type & cpB
        , unsigned int const max_iterations = 500
        )
      {
        feature_pointer witnessA = *seedA;
        feature_pointer witnessB = *seedB;

        if(!witnessA)
        {
          throw std::invalid_argument("vclip(): witness A was null?");
          return real_type();
        }
        if(!witnessB)
        {
          throw std::invalid_argument("vclip(): witness B was null?");
          return real_type();
        }

        if((witnessA->m_type==Feature::FACE && witnessB->m_type==Feature::FACE))
        {
          throw std::invalid_argument("vclip(): both witness' were faces?");
          return real_type();
        }

        result_type result     = CONTINUE;
        m_distance             = static_cast<real_type>(0.0);
        unsigned int iteration = 0;

        for(; (result==CONTINUE) && (iteration < max_iterations); ++iteration)
        {
          state_type state = (witnessA->m_type<<2)+witnessB->m_type;
          switch(state)
          {
          case vertex_vertex_state:
            m_vertexA = witnessA;
            m_vertexB = witnessB;
            result    = vertex_vertex(AtoB,BtoA,cpA,cpB);
            witnessA  = m_vertexA;
            witnessB  = m_vertexB;
            break;
          case vertex_edge_state:
            m_vertex = witnessA;
            m_edge   = witnessB;
            result   = vertex_edge(AtoB,BtoA,cpA,cpB);
            witnessA = m_vertex;
            witnessB = m_edge;
            break;
          case edge_vertex_state:
            m_edge   = witnessA;
            m_vertex = witnessB;
            result   = vertex_edge(BtoA,AtoB,cpB,cpA);
            witnessA = m_edge;
            witnessB = m_vertex;
            break;
          case vertex_face_state:
            m_vertex = witnessA;
            m_face   = witnessB;
            result   = vertex_face(AtoB,cpA,cpB,const_cast< vclip_mesh_type *>( &meshB ));
            witnessA = m_vertex;
            witnessB = m_face;
            break;
          case face_vertex_state:
            m_face   = witnessA;
            m_vertex = witnessB;
            result   = vertex_face(BtoA,cpB,cpA,const_cast< vclip_mesh_type *>(&meshA));
            witnessA = m_face;
            witnessB = m_vertex;
            break;
          case edge_edge_state:
            m_edgeA  = witnessA;
            m_edgeB  = witnessB;
            result   = edge_edge(AtoB,BtoA,cpA,cpB);
            witnessA = m_edgeA;
            witnessB = m_edgeB;
            break;
          case edge_face_state:
            m_edge   = witnessA;
            m_face   = witnessB;
            result   = edge_face(AtoB,cpA,cpB);
            witnessA = m_edge;
            witnessB = m_face;
            break;
          case face_edge_state:
            m_face   = witnessA;
            m_edge   = witnessB;
            result   = edge_face(BtoA,cpB,cpA);
            witnessA = m_face;
            witnessB = m_edge;
            break;
          default: 
            throw std::logic_error("vclip(): unrecognized state, this should never happen!"); 
            break;
          }
        }

        *seedA = witnessA;
        *seedB = witnessB;

        return m_distance;
      }

    };

  } // namespace vclip

} // namespace OpenTissue

//OPENTISSUE_COLLISION_VCLIP_VCLIP_H
#endif
