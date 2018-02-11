#ifndef CONNECTED_MESH_TOP_DOWN_POLICY_H
#define CONNECTED_MESH_TOP_DOWN_POLICY_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh.h>

#include <boost/shared_ptr.hpp>

template<typename bvh_type_,typename mesh_type_>
class ConnectedMeshTopDownPolicy
{
public:

  typedef          bvh_type_                               bvh_type;
  typedef          mesh_type_                              mesh_type;
  typedef ConnectedMeshTopDownPolicy<bvh_type,mesh_type>   top_down_type;
  typedef typename mesh_type::face_type                    face_type;

  typedef typename mesh_type::math_types              math_types;
  typedef typename math_types::vector3_type           vector3_type;
  typedef typename math_types::value_traits           value_traits;

  typedef typename bvh_type::bv_ptr                   bv_ptr;
  typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
  typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
  typedef typename bvh_type::volume_type              volume_type;
  typedef typename bvh_type::geometry_type            geometry_type;
  typedef typename std::list<face_type*>              face_type_ptr_queue;

protected:

  face_type   * m_face;                   ///< Pointer to initial mesh face.
  unsigned int  m_size;                   ///< Total mesh size (i.e total number of faces).
  unsigned int  m_next_region_number;     ///< Next freely available region number.

public:

  std::vector<unsigned int> m_region;    ///< Region table lookup.

public:

  class partition_type
  {
  public:

    friend class ConnectedMeshTopDownPolicy<bvh_type,mesh_type>;

  public:

    typedef std::vector<partition_type>              partition_container;
    typedef typename partition_container::iterator   partition_iterator;

  protected:

    partition_container    m_sub_partitions;
    top_down_type        * m_owner;
    unsigned int           m_region_number;
    unsigned int           m_size;
    face_type            * m_face;

  public:

    partition_type(void)
      : m_owner(0)
      , m_region_number(0)
      , m_size(0)
      , m_face(0)
    {}

    partition_type(top_down_type * owner, unsigned int region_number, unsigned int sz, face_type * face)
      : m_owner(owner)
      , m_region_number(region_number)
      , m_size(sz)
      , m_face(face)
    {}

    bool annotated() const { return m_size==1;}
    unsigned int size() const { return m_size;}
    bool empty() { return m_size==0; }
    void split() {  m_owner->split((*this));  }
    partition_iterator sub_partition_begin() { return m_sub_partitions.begin(); }
    partition_iterator sub_partition_end() { return m_sub_partitions.end(); }
    void fit(bv_ptr bv){ m_owner->fit(bv,(*this)); }
  };

public:

  typedef typename partition_type::partition_container   partition_container;
  typedef typename partition_type::partition_iterator    partition_iterator;

public:

  partition_type all() {  return partition_type(this,0,m_size,m_face);  }

  template<typename iterator>
  void init(iterator begin,iterator end)
  {
    m_size = std::distance(begin,end);
    assert(m_size>0);
    m_region.resize(m_size);
    std::fill(m_region.begin(),m_region.end(),0);
    m_next_region_number = 1;
    m_face = &(*begin);
    unsigned int idx = 0;
    for(iterator geometry = begin;geometry!=end;++geometry,++idx)
      geometry->m_tag = idx;
  }

protected:

  unsigned int degree(void)const {return 8;}

  void split(partition_type & partition)
  {
    unsigned int total_size  = partition.size();
    unsigned int cnt         = std::min(total_size,degree());
    partition.m_sub_partitions.resize(cnt);
    std::vector<bool> seen(m_size);
    std::fill(seen.begin(),seen.end(),false);
    std::vector<face_type*> faces;
    get_faces(partition.m_face,faces);
    std::random_shuffle(faces.begin(),faces.end());
    std::map<unsigned int,unsigned int> sizes;
    face_type_ptr_queue Q;
    for(unsigned int i=0;i<cnt;++i)
    {
      face_type * face = faces[i];
      m_region[face->m_tag] = m_next_region_number;
      sizes[m_next_region_number] = 1;
      ++m_next_region_number;
      Q.push_back(face);
      seen[face->m_tag] = true;
    }
    while(!Q.empty())
    {
      face_type * face = Q.front();
      Q.pop_front();

      typename mesh_type::face_halfedge_circulator h(*face),hend;
      for(;h!=hend;++h)
      {
        if(h->get_twin_iterator()->get_face_handle().is_null())
          continue;

        face_type * neighbor = &(*h->get_twin_iterator()->get_face_iterator());
        bool same_region = (m_region[neighbor->m_tag] == partition.m_region_number);
        bool unseen = !seen[neighbor->m_tag];
        if( same_region && unseen )
        {
          Q.push_back(neighbor);
          unsigned int region = m_region[face->m_tag];
          m_region[neighbor->m_tag] = region;
          sizes[region] = sizes[region]+1;
          seen[neighbor->m_tag] = true;
        }
      }
    }
    for(unsigned int i=0;i<cnt;++i)
    {
      face_type * face = faces[i];
      unsigned int region = m_region[face->m_tag];
      unsigned int size = sizes[region];
      partition.m_sub_partitions[i] = partition_type(this,region,size,face);
    }
  }

  void get_faces(face_type * face1, std::vector<face_type*> & faces)
  {
    std::vector<bool> seen(m_size);
    std::fill(seen.begin(),seen.end(),false);
    unsigned int region = m_region[face1->m_tag];
    faces.clear();
    face_type_ptr_queue Q;
    Q.push_back(face1);
    seen[face1->m_tag] = true;
    while(!Q.empty())
    {
      face_type * face = Q.front();Q.pop_front();
      faces.push_back(face);
      typename mesh_type::face_halfedge_circulator h(*face),hend;
      for(;h!=hend;++h)
      {
        if(h->get_twin_iterator()->get_face_handle().is_null())
          continue;
        face_type * neighbor = &(*h->get_twin_iterator()->get_face_iterator());
        bool same_region = (m_region[neighbor->m_tag] == region);
        bool unseen = !seen[neighbor->m_tag];
        if( same_region && unseen)
        {
          Q.push_back(neighbor);
          seen[neighbor->m_tag] = true;
        }
      }
    }
  }

  void fit(bv_ptr bv,partition_type & partition)
  {
    if(partition.annotated())
    {
      assert(partition.size()==1);

      annotated_bv_ptr A = boost::static_pointer_cast<annotated_bv_type>(bv);

      A->insert( partition.m_face );
    }
    std::vector<bool> visited(m_size);
    std::fill(visited.begin(),visited.end(),false);
    vector3_type & pmin = bv->volume().min();
    vector3_type & pmax = bv->volume().max();
    pmin = vector3_type(value_traits::infinity(),value_traits::infinity(),value_traits::infinity());
    pmax = -pmin;
    face_type_ptr_queue Q;
    Q.push_back(partition.m_face);
    visited[partition.m_face->m_tag] = true;
    unsigned int count = 0;
    while(!Q.empty())
    {
      face_type * face = Q.front();Q.pop_front();
      assert(m_region[face->m_tag]==partition.m_region_number);
      ++count;
      typename mesh_type::face_halfedge_circulator h(*face),hend;
      for(;h!=hend;++h)
      {
        vector3_type & coord = h->get_origin_iterator()->m_coord;
        if(coord(0) < pmin(0))
          pmin(0) = coord(0);
        if(coord(1) < pmin(1))
          pmin(1) = coord(1);
        if(coord(2) < pmin(2))
          pmin(2) = coord(2);
        if(coord(0) > pmax(0))
          pmax(0) = coord(0);
        if(coord(1) > pmax(1))
          pmax(1) = coord(1);
        if(coord(2) > pmax(2))
          pmax(2) = coord(2);

        if(h->get_twin_iterator()->get_face_handle().is_null())
          continue;

        face_type * neighbor = &*(h->get_twin_iterator()->get_face_iterator());
        assert(neighbor);
        if((m_region[neighbor->m_tag] == partition.m_region_number) && !visited[neighbor->m_tag])
        {
          Q.push_back(neighbor);
          visited[neighbor->m_tag] = true;
        }
      }
    }
    assert(partition.m_size==count);
  }

};

// CONNECTED_MESH_TOP_DOWN_POLICY_H
#endif 
