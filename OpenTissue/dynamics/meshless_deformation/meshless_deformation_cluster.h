#ifndef OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_CLUSTER_H
#define OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_CLUSTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/meshless_deformation/meshless_deformation_particle.h>
#include <OpenTissue/core/math/math_polar_decomposition.h>
#include <OpenTissue/core/math/big/big_lu.h>

#include <boost/iterator/indirect_iterator.hpp>
#include <boost/cast.hpp> // needed for boost::numeric_cast

#include <list>

namespace OpenTissue
{
  namespace meshless_deformation
  {
    namespace detail
    {

      /**
      * A Cluster of Particles.
      */
      template<typename math_types>
      class Cluster
      {
      public:

        typedef Particle<math_types>                                           particle_type;
        typedef typename math_types::value_traits                              value_traits;
        typedef typename math_types::real_type                                 real_type;
        typedef typename math_types::vector3_type                              vector3_type;
        typedef typename math_types::matrix3x3_type                            matrix3x3_type;
        typedef typename std::list<particle_type*>                             particle_ptr_container;
        typedef typename particle_ptr_container::iterator                      particle_ptr_iterator;
        typedef boost::indirect_iterator<particle_ptr_iterator,particle_type>  particle_iterator;

      protected:

        particle_ptr_container  m_particles;  ///< Collection of all particles in the cluster.

      private:

        vector3_type            m_t;          ///< Current center of mass position.
        vector3_type            m_t0;         ///< Original center of mass position.
        real_type               m_mass;       ///< Total mass of all particles in cluster.
        matrix3x3_type          m_A_qq[3][3];
        matrix3x3_type          m_A_pq[3];
        matrix3x3_type          m_A;          ///< Shear and stretch modes.
        matrix3x3_type          m_Q;          ///< Bend modes.
        matrix3x3_type          m_M;          ///< Twist modes.
        matrix3x3_type          m_R;          ///< Pure rotational warp.
        matrix3x3_type          m_S;          ///< Symmetric part of polar decomposition.
        matrix3x3_type          m_Sp;         ///< Plasticity matrix, initially identity matrix.

      private:

        //--- Parameters that determine the behavior of the motion

        real_type m_beta;       ///< Weighted of affine transform and rotational warp, i.e T = beta*A+(1-beta)*R and g = T*q + t
        real_type m_tau;        ///< Time constant used to compute alpha = dt/tau, which is the weight (strength) of goal positions.
        real_type m_c_yield;    ///< Plasticity yield.
        real_type m_c_creep;    ///< Plasticity creep.
        real_type m_c_max;      ///< Plasticity max.

      public:

        Cluster()
          : m_Sp(
          value_traits::one() ,value_traits::zero(),value_traits::zero()
          , value_traits::zero(),value_traits::one() ,value_traits::zero()
          , value_traits::zero(),value_traits::zero(),value_traits::one()
          )
          , m_beta(value_traits::half() )
          , m_tau( value_traits::one() )
          , m_c_yield( value_traits::infinity() )
          , m_c_creep( value_traits::zero() )
          , m_c_max( value_traits::zero() )
        {}

      public:

        void bind_particle(particle_type & particle) { m_particles.push_back(&particle); }

        void set_beta(real_type const & beta)
        {
          assert(beta>=value_traits::zero() || !"set_beta(): beta must be non-negative");
          assert(beta<=value_traits::one()  || !"set_beta(): beta must be less than one");
          m_beta = beta;
        }

        real_type const & get_beta()const { return m_beta; }

        void set_tau(real_type const & tau)
        {
          assert(tau>=value_traits::zero() || !"set_tau(): tau must be non-negative");
          m_tau = tau;
        }

        real_type const & get_tau()const { return m_tau; }

        void set_yield(real_type const & c_yield)
        {
          assert(c_yield>=value_traits::zero() || !"set_yield(): yield must be non-negative");
          m_c_yield = c_yield;
        }

        real_type const & get_yield()const { return m_c_yield; }

        void set_creep(real_type const & c_creep)
        {
          assert(c_creep>=value_traits::zero() || !"set_creep(): creep must be non-negative");
          m_c_creep = c_creep;
        }

        real_type const & get_creep()const { return m_c_creep; }

        void set_max(real_type const & c_max)
        {
          assert(c_max>=value_traits::zero() || !"set_max(): max must be non-negative");
          m_c_max = c_max;
        }

        real_type const & get_max()const { return m_c_max; }

      public: // should be protected

        void init()
        {
          particle_iterator begin = m_particles.begin();
          particle_iterator end   = m_particles.end();
          particle_iterator particle;

          m_mass = value_traits::zero();
          for(particle=begin;particle!=end;++particle)
            m_mass += particle->m_mass;

          m_t0.clear();
          for(particle=begin;particle!=end;++particle)
            m_t0 += particle->m_mass * particle->m_x0;
          m_t0 /= m_mass;

          compute_q();
        }

        void run(real_type const & dt)
        {
          if( m_c_yield < value_traits::infinity() ) //---plasticity is on
            compute_q();

          compute_p();

          matrix3x3_type B =  m_A_pq[0] * m_A_qq[0][0];
          OpenTissue::math::polar_decomposition::eigen( B, m_R, m_S);

          plasticity_update(dt);

          compute_goal(dt);
        }

      private:

        void compute_q()
        {
          particle_iterator begin = m_particles.begin();
          particle_iterator end   = m_particles.end();
          particle_iterator particle;

          for(particle=begin;particle!=end;++particle)
            particle->m_q[0] =  m_Sp*(particle->m_x0 - m_t0);

          for(particle=begin;particle!=end;++particle)
            particle->m_q[1] =  vector3_type (
              particle->m_q[0](0)*particle->m_q[0](0)
            , particle->m_q[0](1)*particle->m_q[0](1)
            , particle->m_q[0](2)*particle->m_q[0](2)
            );

          for(particle=begin;particle!=end;++particle)
            particle->m_q[2] =  vector3_type (
              particle->m_q[0](0)*particle->m_q[0](1)
            , particle->m_q[0](1)*particle->m_q[0](2)
            , particle->m_q[0](2)*particle->m_q[0](0)
            );

          //           |q0|                       | q0 q0^T    q0 q1^T    q0 q2^T |
          //    A =    |q1|   [q0^T q1^T q2^T]  = | q1 q0^T    q1 q1^T    q1 q2^T |
          //           |q2|                       | q2 q0^T    q2 q1^T    q2 q2^T |
          m_A_qq[0][0].clear();
          m_A_qq[0][1].clear();
          m_A_qq[0][2].clear();

          m_A_qq[1][0].clear();
          m_A_qq[1][1].clear();
          m_A_qq[1][2].clear();

          m_A_qq[2][0].clear();
          m_A_qq[2][1].clear();
          m_A_qq[2][2].clear();
          for(particle=begin;particle!=end;++particle)
          {
            m_A_qq[0][0] +=  particle->m_mass * outer_prod(particle->m_q[0],particle->m_q[0]);
            m_A_qq[1][1] +=  particle->m_mass * outer_prod(particle->m_q[1],particle->m_q[1]);
            m_A_qq[2][2] +=  particle->m_mass * outer_prod(particle->m_q[2],particle->m_q[2]);
            m_A_qq[0][1] +=  particle->m_mass * outer_prod(particle->m_q[0],particle->m_q[1]);
            m_A_qq[1][2] +=  particle->m_mass * outer_prod(particle->m_q[1],particle->m_q[2]);
            m_A_qq[2][0] +=  particle->m_mass * outer_prod(particle->m_q[2],particle->m_q[0]);
          }
          m_A_qq[1][0] = trans(m_A_qq[0][1]);
          m_A_qq[2][1] = trans(m_A_qq[1][2]);
          m_A_qq[0][2] = trans(m_A_qq[2][0]);
          //--- Oh! I need to invert A_qq? Hmm, there must be an easier way, that exploits the partitioning of A_qq?
          {
            typename ublas::matrix<real_type> A(9,9),invA(9,9);

            for(size_t r=0;r<9;++r)
              for(size_t c=0;c<9;++c)
                A(r,c) = m_A_qq[r/3][c/3](r%3,c%3);

            // 2008-04-13 kenny: Maybe the matrix inversion should be a policy?
            math::big::lu_invert(A,invA);

            for(size_t r=0;r<9;++r)
              for(size_t c=0;c<9;++c)
                m_A_qq[r/3][c/3](r%3,c%3) = invA(r,c);
          }
        }

        void compute_p()
        {
          particle_iterator begin = m_particles.begin();
          particle_iterator end   = m_particles.end();
          particle_iterator particle;

          m_t.clear();
          for(particle=begin;particle!=end;++particle)
            m_t += particle->m_mass * particle->x();
          m_t /= m_mass;

          for(particle=begin;particle!=end;++particle)
            particle->m_p =  particle->x() - m_t;

          //
          //   [p]   [q0^T q1^T q2^T]  = | p q0^T    p q1^T    p q2^T |
          //
          m_A_pq[0].clear();
          m_A_pq[1].clear();
          m_A_pq[2].clear();
          for(particle=begin;particle!=end;++particle)
          {
            m_A_pq[0] +=  particle->m_mass * outer_prod(particle->m_p,particle->m_q[0]);
            m_A_pq[1] +=  particle->m_mass * outer_prod(particle->m_p,particle->m_q[1]);
            m_A_pq[2] +=  particle->m_mass * outer_prod(particle->m_p,particle->m_q[2]);
          }
        }

        void compute_goal(real_type const & dt)
        {
          using std::min;

          static real_type const third = value_traits::one()/value_traits::three();
          static real_type const tiny  = boost::numeric_cast<real_type>(10e-7);

          particle_iterator begin = m_particles.begin();
          particle_iterator end   = m_particles.end();
          particle_iterator particle;

          m_A = m_A_pq[0]*m_A_qq[0][0] + m_A_pq[1]*m_A_qq[1][0] + m_A_pq[2]*m_A_qq[2][0];
          m_Q = m_A_pq[0]*m_A_qq[0][1] + m_A_pq[1]*m_A_qq[1][1] + m_A_pq[2]*m_A_qq[2][1];
          m_M = m_A_pq[0]*m_A_qq[0][2] + m_A_pq[1]*m_A_qq[1][2] + m_A_pq[2]*m_A_qq[2][2];
          {
            m_A /= pow(det(m_A),third);
            //--- What about Q and M?
            //m_Q /= pow(det(m_Q),third);
            //m_M /= pow(det(m_M),third);
          }

          m_A = m_beta*m_A  + (value_traits::one() - m_beta)*m_R;
          m_Q = m_beta*m_Q;
          m_M = m_beta*m_M;

          //--- To counter numerical precision problems!!!
          m_Q = truncate(m_Q,tiny);
          m_M = truncate(m_M,tiny);

          for(particle=begin;particle!=end;++particle)
            particle->m_g =  (m_A * particle->m_q[0]) + (m_Q * particle->m_q[1]) + (m_M * particle->m_q[2]) + m_t;

          real_type alpha = min( m_tau/dt, value_traits::one() );
          for(particle=begin;particle!=end;++particle)
            particle->m_f_goal  += alpha*(particle->m_g - particle->x())/dt;
        }

        void plasticity_update( real_type const & dt  )
        {
          static real_type const third = value_traits::one()/value_traits::three();
          static matrix3x3_type const I = math::diag(value_traits::one());

          if(norm_2(m_S-I) < m_c_yield)
            return;
          m_Sp = (I + dt*m_c_creep*(m_S-I))*m_Sp;
          if(norm_2(m_Sp-I) >= m_c_max)
            m_Sp = I + m_c_max*(m_Sp-I)/ norm_2(m_Sp-I);
          m_Sp /= pow(det(m_Sp),third);
        }

      };

    } // namespace detail
  } // namespace meshless_deformation
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_MESHLESS_DEFORMATION_MESHLESS_DEFORMATION_CLUSTER_H
#endif
