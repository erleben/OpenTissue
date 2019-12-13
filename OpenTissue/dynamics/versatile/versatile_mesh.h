#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_MESH_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_MESH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/dynamics/versatile/versatile_node_traits.h>
#include <OpenTissue/dynamics/versatile/versatile_tetrahedron_traits.h>

#include <OpenTissue/dynamics/versatile/versatile_distance_constraint.h>
#include <OpenTissue/dynamics/versatile/versatile_area_constraint.h>
#include <OpenTissue/dynamics/versatile/versatile_volume_constraint.h>


namespace OpenTissue
{
  namespace versatile
  {

    template <typename versatile_types>
    class Mesh 
      : public OpenTissue::t4mesh::T4Mesh< 
      versatile_types
      , OpenTissue::versatile::detail::NodeTraits<versatile_types>
      , OpenTissue::versatile::detail::TetrahedronTraits<versatile_types> 
      >
    {
    public:

      typedef OpenTissue::t4mesh::T4Mesh< 
        versatile_types
        , OpenTissue::versatile::detail::NodeTraits<versatile_types>
        , OpenTissue::versatile::detail::TetrahedronTraits<versatile_types> 
      >  base_class;

      typedef typename versatile_types::value_traits     value_traits;
      typedef typename versatile_types::real_type        real_type;
      typedef typename versatile_types::vector3_type     vector3_type;
      typedef typename versatile_types::matrix3x3_type   matrix3x3_type;
      typedef typename base_class::node_type             node_type;
      typedef typename base_class::node_iterator         node_iterator;
      typedef typename base_class::tetrahedron_iterator  tetrahedron_iterator;

      typedef OpenTissue::versatile::detail::DistanceConstraint<versatile_types>         distance_constraint;
      typedef OpenTissue::versatile::detail::AreaConstraint<versatile_types>             area_constraint;
      typedef OpenTissue::versatile::detail::VolumeConstraint<versatile_types>           volume_constraint;

      typedef std::vector<distance_constraint>  distance_container;
      typedef std::vector<area_constraint>      area_container;
      typedef std::vector<volume_constraint>    volume_container;

      typedef typename distance_container::iterator      distance_iterator;
      typedef typename area_container::iterator          area_iterator;
      typedef typename volume_container::iterator        volume_iterator;

    protected:

      distance_container  m_distance_constraints;  ///< Collection of all distance constraints.
      area_container      m_area_constraints;      ///< Collection of all area constraints.
      volume_container    m_volume_constraints;    ///< Collection of all volume constraints.

    public:

      void clear()
      {
        base_class::clear();
        m_distance_constraints.clear();
        m_area_constraints.clear();
        m_volume_constraints.clear();
      }

      void initialize()
      {
        typedef OpenTissue::t4mesh::T4Edges<
          Mesh<versatile_types>, OpenTissue::t4mesh::DefaultT4EdgeTraits > Edges;

        Edges edges(*this);

        for(typename Edges::edge_iterator edge=edges.begin();edge!=edges.end();++edge)
        {
          distance_constraint constraint;
          node_type & ni = *base_class::node(edge->idxA());
          node_type & nj = *base_class::node(edge->idxB());
          constraint.initialize(ni,nj);
          m_distance_constraints.push_back(constraint);
        }

        typedef OpenTissue::t4mesh::T4BoundaryFaces<
          Mesh<versatile_types>
          , OpenTissue::t4mesh::DefaultT4FaceTraits 
        > Boundary;

        Boundary boundary(*this);

        for(typename Boundary::face_iterator face=boundary.begin();face!=boundary.end();++face)
        {
          area_constraint constraint;
          node_type & ni = *base_class::node(face->idx0());
          node_type & nj = *base_class::node(face->idx1());
          node_type & nk = *base_class::node(face->idx2());
          constraint.initialize(ni,nj,nk);
          m_area_constraints.push_back(constraint);
        }

        for(tetrahedron_iterator volume=this->tetrahedron_begin();volume!=this->tetrahedron_end();++volume)
        {
          volume_constraint constraint;
          node_type & ni = *volume->i();
          node_type & nj = *volume->j();
          node_type & nk = *volume->k();
          node_type & nm = *volume->m();
          constraint.initialize(ni,nj,nk,nm);
          m_volume_constraints.push_back(constraint);
        }

        std::cout << "|D| = " << m_distance_constraints.size() << std::endl;
        std::cout << "|A| = " << m_area_constraints.size() << std::endl;
        std::cout << "|V| = " << m_volume_constraints.size() << std::endl;
      }

      void set_distance_coefficients(real_type const & stiffness, real_type const & damping)
      {
        for(distance_iterator d=m_distance_constraints.begin();d!=m_distance_constraints.end();++d)
        {
          d->m_k = stiffness;
          d->m_b = damping;
        }
      }

      void set_area_coefficients(real_type const & stiffness, real_type const & damping)
      {
        for(area_iterator a=m_area_constraints.begin();a!=m_area_constraints.end();++a)
        {
          a->m_k = stiffness;
          a->m_b = damping;
        }
      }

      void set_volume_coefficients(real_type const & stiffness, real_type const & damping)
      {
        for(volume_iterator v=m_volume_constraints.begin();v!=m_volume_constraints.end();++v)
        {
          v->m_k = stiffness;
          v->m_b = damping;
        }
      }

      void set_plasticity(real_type const & c_yield, real_type const & c_creep, real_type const & c_max)
      {
        for(distance_iterator d=m_distance_constraints.begin();d!=m_distance_constraints.end();++d)
        {
          d->m_c_yield = c_yield;
          d->m_c_creep = c_creep;
          d->m_c_max   = c_max;
        }
      }

      void clear_constraint_forces()
      {
        node_iterator begin = this->node_begin();
        node_iterator end = this->node_end();
        node_iterator node;
        for(node=begin;node!=end;++node)
          node->m_f_con.clear();
      }

      void clear_penalty_forces()
      {
        node_iterator begin = this->node_begin();
        node_iterator end = this->node_end();
        node_iterator node;
        for(node=begin;node!=end;++node)
          node->m_f_pen.clear();
      }

      void apply_constraint_forces()
      {
        for(distance_iterator d=m_distance_constraints.begin();d!=m_distance_constraints.end();++d)
          d->apply();
        for(area_iterator a=m_area_constraints.begin();a!=m_area_constraints.end();++a)
          a->apply();
        for(volume_iterator v=m_volume_constraints.begin();v!=m_volume_constraints.end();++v)
          v->apply();
      }

      real_type compute_internal_energy()
      {
        real_type energy = value_traits::zero();
        for(distance_iterator d=m_distance_constraints.begin();d!=m_distance_constraints.end();++d)
          energy += d->compute_internal_energy();
        for(area_iterator a=m_area_constraints.begin();a!=m_area_constraints.end();++a)
          energy += a->compute_internal_energy();
        for(volume_iterator v=m_volume_constraints.begin();v!=m_volume_constraints.end();++v)
          energy += v->compute_internal_energy();
        return energy;
      }

      void integrate(real_type const & dt)
      {
        real_type dt2 = dt*dt;
        node_iterator begin = this->node_begin();
        node_iterator end = this->node_end();
        node_iterator node;
        for(node=begin;node!=end;++node)
        {
          if(node->m_fixed)
            continue;

          vector3_type tmp = node->m_coord;
          node->m_coord    = value_traits::two()*node->m_coord - node->m_x0 + (dt2/node->m_mass)*( node->m_f_ext + node->m_f_con + node->m_f_pen);
          node->m_v        = (node->m_coord - node->m_x0)/(value_traits::two()*dt);
          node->m_x0       = tmp;
        }
      }
    };
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_MESH_H
#endif
