#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

/**
* Main SPH include file.
* Recursively includes required headers.
*/
#include <OpenTissue/dynamics/sph/sph_emitter.h>
#include <OpenTissue/dynamics/sph/sph_idealgas.h>
#include <OpenTissue/dynamics/sph/sph_integrator.h>
#include <OpenTissue/dynamics/sph/sph_kernel.h>
#include <OpenTissue/dynamics/sph/sph_material.h>
#include <OpenTissue/dynamics/sph/sph_particle.h>
#include <OpenTissue/dynamics/sph/sph_solver.h>
#include <OpenTissue/dynamics/sph/sph_system.h>

#include <OpenTissue/dynamics/sph/collision/sph_collision.h>
#include <OpenTissue/dynamics/sph/emitters/sph_emitters.h>
#include <OpenTissue/dynamics/sph/integrators/sph_integrators.h>
#include <OpenTissue/dynamics/sph/kernels/sph_kernels.h>
#include <OpenTissue/dynamics/sph/materials/sph_materials.h>
#include <OpenTissue/dynamics/sph/solvers/sph_solvers.h>

#include <vector>
#include <utility>

namespace OpenTissue
{
  namespace sph
  {

    template < typename type >
    class fixed_size_vector {
    public:
      typedef std::vector<type>  container;
      typedef typename container::const_iterator  const_iterator;
      const const_iterator begin() const {return m_elems.begin();}
      const const_iterator end() const {return m_elems.begin()+m_size;}
      void clear() {m_size=0;}
      void resize(size_t fixed_size) {m_elems.resize(fixed_size);m_size=0;}
      void push_back(const type& elem) {m_elems[m_size++]=elem;}
      fixed_size_vector(size_t fixed_size=0) {resize(fixed_size);}
    private:
      container  m_elems;
      size_t  m_size;
    };

    /**
    * SPH Type Binder Class.
    * Use this class to define the sph user types.
    */
    template <
      typename Real_Type
      , template<typename> class Vector_Type
      , typename Particle_Type
      , typename Collision_Detection
      , typename Hash_Function
      , template<typename, typename, typename, typename> class Hash_Grid
      , template<typename, typename> class Point_Query
    >
    class Types
    {
    public:
      typedef Real_Type  real_type;
      typedef Vector_Type<real_type>  vector;
      typedef Particle_Type  particle;
      typedef Collision_Detection  collision_detection;
      typedef std::vector<particle>  particle_container;
      typedef std::vector<particle*>  particle_ptr_container;
      typedef fixed_size_vector<const particle*>  particle_cptr_container;
      typedef std::pair<const particle*,const particle*>  particle_cptr_pair;
      typedef std::vector<particle_cptr_pair>  particle_cptr_pair_container;
      template<typename User_Policy, typename Data_Type>
      class hashing
      {
      public:
        typedef Hash_Grid<Vector_Type<real_type>, Vector_Type<long>, Data_Type, Hash_Function> hash_grid;
        typedef Point_Query<hash_grid, User_Policy> point_query;
      };
    };

  } // namespace sph
} //namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_H
#endif
