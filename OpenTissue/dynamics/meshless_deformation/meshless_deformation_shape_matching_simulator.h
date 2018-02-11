#ifndef OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_SHAPE_MATCHING_SIMULATOR_H
#define OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_SHAPE_MATCHING_SIMULATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/meshless_deformation/meshless_deformation_particle.h>
#include <OpenTissue/dynamics/meshless_deformation/meshless_deformation_cluster.h>

#include <list>

namespace OpenTissue
{
  namespace meshless_deformation
  {
    /**
    * Meshless Deformation based on Shape Matching.
    *
    * This implementation is based on the paper
    *
    *    @article{1073216,
    *     author = {Matthias M\"{u}ller and Bruno Heidelberger and Matthias Teschner and Markus Gross},
    *     title = {Meshless deformations based on shape matching},
    *     journal = {ACM Trans. Graph.},
    *     volume = {24},
    *     number = {3},
    *     year = {2005},
    *     issn = {0730-0301},
    *     pages = {471--478},
    *     doi = {http://doi.acm.org/10.1145/1073204.1073216},
    *     publisher = {ACM},
    *     address = {New York, NY, USA},
    *     }
    *
    * One uses the method by doing the following steps
    *
    *  1) Create all clusters
    *  2) Create all partciles
    *  3) Bind particles to the clusters one wants them in.
    *  4) Bind particle positions to any external data (for instance coordinates of vertices in a mesh).
    *
    */
    template<typename math_types>
    class ShapeMatchingSimulator
    {
    public:

      typedef typename math_types::real_type            real_type;
      typedef typename math_types::vector3_type         vector3_type;
      typedef          detail::Particle<math_types>     particle_type;
      typedef typename std::list<particle_type>         particle_container;
      typedef typename particle_container::iterator     particle_iterator;
      typedef          detail::Cluster<math_types>      cluster_type;
      typedef typename std::list<cluster_type>          cluster_container;
      typedef typename cluster_container::iterator      cluster_iterator;

    protected:

      cluster_container      m_clusters;   ///< Collection of all clusters.
      particle_container     m_particles;  ///< Collection of all particles.

    public:

      particle_type * create_particle()
      {
        m_particles.push_back(particle_type());
        return &(m_particles.back());
      }

      cluster_type * create_cluster()
      {
        m_clusters.push_back(cluster_type());
        return &(m_clusters.back());
      }

      size_t size_particles() const {  return  m_particles.size(); }
      size_t size_clusters() const {  return  m_clusters.size(); }

      particle_iterator particle_begin(){  return m_particles.begin(); }
      particle_iterator particle_end(){  return m_particles.end(); }

    public:

      void clear()
      {
        m_particles.clear();
        m_clusters.clear();
      }

      void init()
      {
        cluster_iterator begin = m_clusters.begin();
        cluster_iterator end   = m_clusters.end();
        cluster_iterator cluster;
        for(cluster=begin;cluster!=end;++cluster)
          cluster->init();
      }

      /**
      *
      * Remember to setup external forces prior to invokation.
      *
      * @param dt   Time step size.
      */
      void run(real_type const & dt)
      {
        particle_iterator pbegin = m_particles.begin();
        particle_iterator pend   = m_particles.end();
        cluster_iterator  cbegin = m_clusters.begin();
        cluster_iterator  cend   = m_clusters.end();
        particle_iterator particle;
        cluster_iterator  cluster;

        for(particle=pbegin;particle!=pend;++particle)
          particle->m_f_goal.clear();

        for(cluster=cbegin;cluster!=cend;++cluster)
          cluster->run(dt);

        for(particle=pbegin;particle!=pend;++particle)
        {
          if(particle->m_fixed)
            continue;

          particle->m_v += particle->m_f_goal + (dt/particle->m_mass)*particle->m_f_ext;
          particle->x() += dt * particle->m_v;
        }
      }

    };

  } // namespace meshless_deformation
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_SHAPE_MATCHING_SIMULATOR_H
#endif
