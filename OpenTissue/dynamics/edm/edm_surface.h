#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_SURFACE_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_SURFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_functions.h>

#include <cmath>
#include <cassert>
#include <vector>


namespace OpenTissue
{

  namespace edm
  {

    template<typename edm_types>
    class Surface
      : public edm_types::model_type
    {
    public:

      typedef typename edm_types::model_type     base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type  real_type;
      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::tensor2_type  tensor2_type;
      typedef typename edm_types::Particle  particle_type;
      typedef typename base_type::EDMMatrix  EDMMatrix;
      typedef typename base_type::EDMVector  EDMVector;

      struct SurfaceParticle
        : public particle_type
      {
        SurfaceParticle()
          : particle_type()
        {}
        virtual ~SurfaceParticle(){}

        // 2009-03-11 kenny: improper OT style, member names should use 'm_' 
        tensor2_type  e;   ///< surface tension (eta)
        tensor2_type  x;   ///< surface rigidity (xi)
        tensor2_type G0;  ///< surface natural metric shape
        tensor2_type B0;  ///< surface natural curvature shape
      };

      typedef std::vector<SurfaceParticle>  SurfaceParticles;

    protected:
      
      size_t  m_M, m_N;
      bool  m_wrap_M, m_wrap_N;
      
    private:
      
      real_type  m_h1, m_h2;
      SurfaceParticles  m_P;
    
    public:

      Surface()
        : base_type(edm_types::EDM_Surface)
        , m_M(0)
        , m_N(0)
        , m_wrap_M(false)
        , m_wrap_N(false)
        , m_h1(value_traits::one())
        , m_h2(value_traits::one())
      {}

      virtual ~Surface() {}

    public:

      bool initialize(size_t M, size_t N)
      {
        if ( !m_P.empty() || M < 3 || N < 3 )     // if already created or too few particles
          return false;

        // create particles
        m_P.resize( ( m_M = M ) * ( m_N = N ) );
        const real_type du = m_h1 = 1. / ( M - 1 ), dv = m_h2 = 1. / ( N - 1 );
        real_type v, u;
        v = u = value_traits::zero();
        for ( size_t n = 0; n < N; n++, v += dv, u = 0 )
          for ( size_t m = 0; m < M; m++, u += du )
            grid( m, n ).r = position( this->m_rest, u, v );  // calculate the natural shape position for this particle

        // create natural shape metric tensor matrices G0, and curvature tensor matrices B0
        for ( size_t n = 0; n < N; ++n )
          for ( size_t m = 0; m < M; ++m )
          {
            SurfaceParticle& a = grid( m, n );
            a.G0 = metric( m, n );
            a.n = normal( m, n );  // curvature is dependent of the surface normal
            a.B0 = curvature( m, n );
          }

        v = u = value_traits::zero();
        for ( size_t n = 0; n < N; n++, v += dv, u = 0 )
          for ( size_t m = 0; m < M; m++, u += du )
          {
            SurfaceParticle& a = grid( m, n );
            a.r = position( this->m_init, u, v );  // calculate the initial shape position for this particle
            a.o = a.r;
            a.v *= 0;  // reset velocity
            a.t.u = math::clamp_zero_one(u);
            a.t.v = math::clamp_zero_one(v);
          }

        // create initial surface normals
        compute_surface_normals();

        return true;
      }

      Surface & wrapping(bool M, bool N)
      {
        m_wrap_M = M;
        m_wrap_N = N;
        return *this;
      }

      size_t get_num_M(bool include_wrapping = false) const
      {
        return size_t(m_M + (!include_wrapping ? 0 : m_wrap_M ? 1 : 0));
      }

      size_t get_num_N(bool include_wrapping = false) const
      {
        return size_t(m_N + (!include_wrapping ? 0 : m_wrap_N ? 1 : 0));
      }

      SurfaceParticle const & particle(size_t m, size_t n) const
      {
        return grid(m, n);
      }

      // creation of the parametric patch
      size_t nodes() const
      {
        return size_t(this->m_max_nodes);
      }

      size_t index_adjust( long m, long n ) const
      {
        // TODO: use math::clamp instead of min/max
        using std::min;
        using std::max;

        const size_t m_ = m_wrap_M ? m % ( ( m < 0 ? -1 : 1 ) * static_cast<long>( m_M ) ) + ( m < 0 ? m_M : 0 ) : static_cast<size_t>( max( 0L, min( m, static_cast<long>( m_M - 1 ) ) ) );
        const size_t n_ = m_wrap_N ? n % ( ( n < 0 ? -1 : 1 ) * static_cast<long>( m_N ) ) + ( n < 0 ? m_N : 0 ) : static_cast<size_t>( max( 0L, min( n, static_cast<long>( m_N - 1 ) ) ) );
        return size_t(n_*m_M + m_);
      }

      Surface & set_natural_position(size_t idx, vector3_type const & r)
      {
        assert(idx < this->m_max_nodes || !"set_natural_position: idx out of range.");
        this->m_rest[idx] = r;
        return set_initial_position(idx, r);  // default behavior if init pos isn't called explicitly
      }

      Surface & set_initial_position(size_t idx, vector3_type const & r)
      {
        assert(idx < this->m_max_nodes || !"set_natural_position: idx out of range.");
        this->m_init[idx] = r;
        return *this;
      }

      // particles (initialize must be called before);
      Surface & set_fixed(size_t m, size_t n, bool fixed)
      {
        assert(m < m_M && n < m_N);
        grid(m, n).f = fixed ;
        return *this;
      }

      Surface & set_mass(size_t m, size_t n, real_type const & mass)
      {
        assert(m < m_M && n < m_N);
        grid(m, n).m = mass;
        return *this;
      }

      Surface & set_damping(size_t m, size_t n, real_type const & damping)
      {
        assert(m < m_M && n < m_N);
        grid(m, n).g = damping;
        return *this;
      }

      Surface & set_tension(size_t m, size_t n, tensor2_type const & tension)
      {
        assert(m < m_M && n < m_N);
        grid(m, n).e = tension;
        return *this;
      }

      Surface & set_rigidity(size_t m, size_t n, tensor2_type const & rigidity)
      {
        assert(m < m_M && n < m_N);
        grid(m, n).x = rigidity;
        return *this;
      }

    protected:

      bool index_check(long m, long n) const
      {
        return ( m_wrap_M ? true : 0 <= m && m < static_cast<long>( m_M ) ) && ( m_wrap_N ? true : 0 <= n && n < static_cast<long>( m_N ) );
      }

      SurfaceParticle const & grid_adjust(long m, long n) const
      {
        return m_P[index_adjust(m, n)];
      }

      vector3_type const & r(long m, long n) const
      {
        return grid_adjust(m, n).r;
      }

    private:

      virtual vector3_type position(vector3_type const * a, real_type const & u, real_type const & v) const = 0;
      virtual vector3_type normal(size_t m, size_t n) const = 0;

    private:

      void compute_stiffness(EDMMatrix & K) const
      {
        using std::fabs;
        typedef std::vector<tensor2_type> Tensors;

        //precalculation of alpha and beta for every particle
        Tensors alpha( m_P.size() );
        Tensors beta( m_P.size() );
        for ( size_t n = 0; n < m_N; ++n )
          for ( size_t m = 0; m < m_M; ++m )
          {
            size_t const i = index_adjust( m, n );
            SurfaceParticle const & a = grid( m, n );

            tensor2_type& alpha_ = alpha[ i ];
            tensor2_type m_ = metric( m, n );
            alpha_.t0[ 0 ] = this->m_strength * a.e.t0[ 0 ] * ( m_.t0[ 0 ] - a.G0.t0[ 0 ] );
            alpha_.t0[ 1 ] = this->m_strength * a.e.t0[ 1 ] * ( m_.t0[ 1 ] - a.G0.t0[ 1 ] );
            alpha_.t1[ 0 ] = this->m_strength * a.e.t1[ 0 ] * ( m_.t1[ 0 ] - a.G0.t1[ 0 ] );
            alpha_.t1[ 1 ] = this->m_strength * a.e.t1[ 1 ] * ( m_.t1[ 1 ] - a.G0.t1[ 1 ] );

            //TODO: Using fixed beta
            tensor2_type& beta_ = beta[ i ];
            tensor2_type c_ = curvature( m, n );
            beta_.t0[ 0 ] = fabs( this->m_strength * a.x.t0[ 0 ] * ( c_.t0[ 0 ] - a.B0.t0[ 0 ] ) );
            beta_.t0[ 1 ] = fabs( this->m_strength * a.x.t0[ 1 ] * ( c_.t0[ 1 ] - a.B0.t0[ 1 ] ) );
            beta_.t1[ 0 ] = fabs( this->m_strength * a.x.t1[ 0 ] * ( c_.t1[ 0 ] - a.B0.t1[ 0 ] ) );
            beta_.t1[ 1 ] = fabs( this->m_strength * a.x.t1[ 1 ] * ( c_.t1[ 1 ] - a.B0.t1[ 1 ] ) );
            /*
            //TODO: We've got a serious problem!
            //      The system only works when it wants to flattn,
            //      if it wants to be more curved it'll explode :(
            beta_._0[0] = std::fabs(a.x._0[0]*(c_._0[0] - a.B0._0[0]));
            beta_._0[1] = std::fabs(a.x._0[1]*(c_._0[1] - a.B0._0[1]));
            beta_._1[0] = std::fabs(a.x._1[0]*(c_._1[0] - a.B0._1[0]));
            beta_._1[1] = std::fabs(a.x._1[1]*(c_._1[1] - a.B0._1[1]));
            beta_._0[0] = a.x._0[0]*(c_._0[0] - a.B0._0[0]);
            beta_._0[1] = a.x._0[1]*(c_._0[1] - a.B0._0[1]);
            beta_._1[0] = a.x._1[0]*(c_._1[0] - a.B0._1[0]);
            beta_._1[1] = a.x._1[1]*(c_._1[1] - a.B0._1[1]);
            beta_._0[0] = a.x._0[0];
            beta_._0[1] = a.x._0[1];
            beta_._1[0] = a.x._1[0];
            beta_._1[1] = a.x._1[1];
            */
          }

        real_type const inv_h1h1 = 1. / ( m_h1 * m_h1 );
        real_type const inv_h1h2 = 1. / ( m_h1 * m_h2 );  // == inv_h2h1
        real_type const inv_h2h2 = 1. / ( m_h2 * m_h2 );
        real_type const inv_h1h1h1h1 = inv_h1h1 * inv_h1h1;
        real_type const inv_h1h1h2h2 = inv_h1h2 * inv_h1h2;
        real_type const inv_h2h2h2h2 = inv_h2h2 * inv_h2h2;
        size_t const MN = m_M * m_N;

        for ( size_t n = 0; n < m_N; ++n )
          for ( size_t m = 0; m < m_M; ++m )
          {
            // alpha (tension) 3x3 stencil part
            tensor2_type const & am_n   = alpha[index_adjust(m, n)];
            tensor2_type const & amM1_n = alpha[index_adjust(m-1, n)];
            tensor2_type const & am_nM1 = alpha[index_adjust(m, n-1)];

            real_type Am_nM1 = 0;
            real_type AmP1_nM1 = 0;
            real_type AmM1_n = 0;
            real_type Am_n = 0;
            real_type AmP1_n = 0;
            real_type AmM1_nP1 = 0;
            real_type Am_nP1 = 0;

            // D1P(r[m,n]) = r[m+1,n]-r[m,n]
            if ( index_check( m + 1, n ) )
            {
              real_type const tmp = inv_h1h1 * am_n.t0[ 0 ];
              AmP1_n += - tmp;
              Am_n += + tmp;
            }

            // D1P(r[m-1,n]) = r[m,n]-r[m-1,n]
            if ( index_check( m - 1, n ) )
            {
              real_type const tmp = inv_h1h1 * amM1_n.t0[ 0 ];
              Am_n += + tmp;
              AmM1_n += - tmp;
            }

            // D2P(r[m,n]) = r[m,n+1]-r[m,n]
            if ( index_check( m, n + 1 ) )
            {
              real_type const tmp = inv_h1h2 * am_n.t0[ 1 ];
              Am_nP1 += - tmp;
              Am_n += + tmp;
            }

            // D2P(r[m-1,n]) = r[m-1,n+1]-r[m-1,n]
            if ( index_check( m - 1, n + 1 ) )
            {
              real_type const tmp = inv_h1h2 * amM1_n.t0[ 1 ];
              AmM1_nP1 += + tmp;
              AmM1_n += - tmp;
            }

            // D1P(r[m,n]) = r[m+1,n]-r[m,n]
            if ( index_check( m + 1, n ) )
            {
              real_type const tmp = inv_h1h2 * am_n.t1[ 0 ];
              AmP1_n += - tmp;
              Am_n += + tmp;
            }

            // D1P(r[m,n-1]) = r[m+1,n-1]-r[m,n-1]
            if ( index_check( m + 1, n - 1 ) )
            {
              real_type const tmp = inv_h1h2 * am_nM1.t1[ 0 ];
              AmP1_nM1 += + tmp;
              Am_nM1 += - tmp;
            }

            // D2P(r[m,n]) = r[m,n+1]-r[m,n]
            if ( index_check( m, n + 1 ) )
            {
              real_type const tmp = inv_h2h2 * am_n.t1[ 1 ];
              Am_nP1 += - tmp;
              Am_n += + tmp;
            }

            // D2P(r[m,n-1]) = r[m,n]-r[m,n-1]
            if ( index_check( m, n - 1 ) )
            {
              real_type const tmp = inv_h2h2 * am_nM1.t1[ 1 ];
              Am_n += + tmp;
              Am_nM1 += - tmp;
            }

            // beta (rigidity) 5x5 stencil part
            tensor2_type const & bm_n     = beta[index_adjust(m, n)];
            tensor2_type const & bmM1_n   = beta[index_adjust(m-1, n)];
            tensor2_type const & bmP1_n   = beta[index_adjust(m+1, n)];
            tensor2_type const & bm_nM1   = beta[index_adjust(m, n-1)];
            tensor2_type const & bm_nP1   = beta[index_adjust(m, n+1)];
            tensor2_type const & bmM1_nM1 = beta[index_adjust(m-1, n-1)];

            real_type Bm_nM2 = 0;
            real_type BmM1_nM1 = 0;
            real_type Bm_nM1 = 0;
            real_type BmP1_nM1 = 0;
            real_type BmM2_n = 0;
            real_type BmM1_n = 0;
            real_type Bm_n = 0;
            real_type BmP1_n = 0;
            real_type BmP2_n = 0;
            real_type BmM1_nP1 = 0;
            real_type Bm_nP1 = 0;
            real_type BmP1_nP1 = 0;
            real_type Bm_nP2 = 0;

            // D11(r[m+1,n]) = r[m+2,n]-2*r[m+1,n]+r[m,n]
            if ( index_check( m + 2, n ) )
            {
              real_type const tmp = inv_h1h1h1h1 * bmP1_n.t0[ 0 ];
              BmP2_n += + tmp;
              BmP1_n += - 2. * tmp;
              Bm_n += + tmp;
            }

            // D11(r[m,n]) = r[m+1,n]-2*r[m,n]+r[m-1,n]
            if ( index_check( m + 1, n ) && index_check( m - 1, n ) )
            {
              real_type const tmp = inv_h1h1h1h1 * 2. * bm_n.t0[ 0 ];
              BmP1_n += - tmp;
              Bm_n += + 2. * tmp;
              BmM1_n += - tmp;
            }

            // D11(r[m-1,n]) = r[m,n]-2*r[m-1,n]+r[m-2,n]
            if ( index_check( m - 2, n ) )
            {
              real_type const tmp = inv_h1h1h1h1 * bmM1_n.t0[ 0 ];
              Bm_n += + tmp;
              BmM1_n += - 2. * tmp;
              BmM2_n += + tmp;
            }

            // D12P(r[m,n]) = r[m+1,n+1]-r[m+1,n]-r[m,n+1]+r[m,n]
            if ( index_check( m + 1, n + 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bm_n.t0[ 1 ];
              BmP1_nP1 += + tmp;
              BmP1_n += - tmp;
              Bm_nP1 += - tmp;
              Bm_n += + tmp;
            }

            // D12P(r[m,n-1]) = r[m+1,n]-r[m+1,n-1]-r[m,n]+r[m,n-1]
            if ( index_check( m + 1, n - 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bm_nM1.t0[ 1 ];
              BmP1_n += - tmp;
              BmP1_nM1 += + tmp;
              Bm_n += + tmp;
              Bm_nM1 += - tmp;
            }

            // D12P(r[m-1,n]) = r[m,n+1]-r[m,n]-r[m-1,n+1]+r[m-1,n]
            if ( index_check( m - 1, n + 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bmM1_n.t0[ 1 ];
              Bm_nP1 += - tmp;
              Bm_n += + tmp;
              BmM1_nP1 += + tmp;
              BmM1_n += - tmp;
            }

            // D12P(r[m-1,n-1]) = r[m,n]-r[m,n-1]-r[m-1,n]+r[m-1,n-1]
            if ( index_check( m - 1, n - 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bmM1_nM1.t0[ 1 ];
              Bm_n += + tmp;
              Bm_nM1 += - tmp;
              BmM1_n += - tmp;
              BmM1_nM1 += + tmp;
            }

            // D21P(r[m,n]) = r[m+1,n+1]-r[m+1,n]-r[m,n+1]+r[m,n]
            if ( index_check( m + 1, n + 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bm_n.t1[ 0 ];
              BmP1_nP1 += + tmp;
              BmP1_n += - tmp;
              Bm_nP1 += - tmp;
              Bm_n += + tmp;
            }

            // D21P(r[m,n-1]) = r[m+1,n]-r[m+1,n-1]-r[m,n]+r[m,n-1]
            if ( index_check( m + 1, n - 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bm_nM1.t1[ 0 ];
              BmP1_n += - tmp;
              BmP1_nM1 += + tmp;
              Bm_n += + tmp;
              Bm_nM1 += - tmp;
            }

            // D21P(r[m-1,n]) = r[m,n+1]-r[m,n]-r[m-1,n+1]+r[m-1,n]
            if ( index_check( m - 1, n + 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bmM1_n.t1[ 0 ];
              Bm_nP1 += - tmp;
              Bm_n += + tmp;
              BmM1_nP1 += + tmp;
              BmM1_n += - tmp;
            }

            // D21P(r[m-1,n-1]) = r[m,n]-r[m,n-1]-r[m-1,n]+r[m-1,n-1]
            if ( index_check( m - 1, n - 1 ) )
            {
              real_type const tmp = inv_h1h1h2h2 * bmM1_nM1.t1[ 0 ];
              Bm_n += + tmp;
              Bm_nM1 += - tmp;
              BmM1_n += - tmp;
              BmM1_nM1 += + tmp;
            }

            // D22(r[m,n+1]) = r[m,n+2]-2*r[m,n+1]+r[m,n]
            if ( index_check( m, n + 2 ) )
            {
              real_type const tmp = inv_h2h2h2h2 * bm_nP1.t1[ 1 ];
              Bm_nP2 += + tmp;
              Bm_nP1 += - 2. * tmp;
              Bm_n += + tmp;
            }

            // D22(r[m,n]) = r[m,n+1]-2*r[m,n]+r[m,n-1]
            if ( index_check( m, n + 1 ) && index_check( m, n - 1 ) )
            {
              real_type const tmp = inv_h2h2h2h2 * 2. * bm_n.t1[ 1 ];
              Bm_nP1 += - tmp;
              Bm_n += + 2. * tmp;
              Bm_nM1 += - tmp;
            }

            // D22(r[m,n-1]) = r[m,n]-2*r[m,n-1]+r[m,n-2]
            if ( index_check( m, n - 2 ) )
            {
              real_type const tmp = inv_h2h2h2h2 * bm_nM1.t1[ 1 ];
              Bm_n += + tmp;
              Bm_nM1 += - 2. * tmp;
              Bm_nM2 += + tmp;
            }

            // merge the whole 5x5 stencil
            real_type const Sm_nM2 = 0 + Bm_nM2;
            real_type const SmM1_nM1 = 0 + BmM1_nM1;
            real_type const Sm_nM1 = Am_nM1 + Bm_nM1;
            real_type const SmP1_nM1 = AmP1_nM1 + BmP1_nM1;
            real_type const SmM2_n = 0 + BmM2_n;
            real_type const SmM1_n = AmM1_n + BmM1_n;
            real_type const Sm_n = Am_n + Bm_n;
            real_type const SmP1_n = AmP1_n + BmP1_n;
            real_type const SmP2_n = 0 + BmP2_n;
            real_type const SmM1_nP1 = AmM1_nP1 + BmM1_nP1;
            real_type const Sm_nP1 = Am_nP1 + Bm_nP1;
            real_type const SmP1_nP1 = 0 + BmP1_nP1;
            real_type const Sm_nP2 = 0 + Bm_nP2;

            EDMVector row( MN );
            row.clear();
            
            row[ index_adjust( m , n - 2 ) ] += zeroize( Sm_nM2 );
            row[ index_adjust( m - 1, n - 1 ) ] += zeroize( SmM1_nM1 );
            row[ index_adjust( m , n - 1 ) ] += zeroize( Sm_nM1 );
            row[ index_adjust( m + 1, n - 1 ) ] += zeroize( SmP1_nM1 );
            row[ index_adjust( m - 2, n ) ] += zeroize( SmM2_n );
            row[ index_adjust( m - 1, n ) ] += zeroize( SmM1_n );
            row[ index_adjust( m , n ) ] += zeroize( Sm_n );
            row[ index_adjust( m + 1, n ) ] += zeroize( SmP1_n );
            row[ index_adjust( m + 2, n ) ] += zeroize( SmP2_n );
            row[ index_adjust( m - 1, n + 1 ) ] += zeroize( SmM1_nP1 );
            row[ index_adjust( m , n + 1 ) ] += zeroize( Sm_nP1 );
            row[ index_adjust( m + 1, n + 1 ) ] += zeroize( SmP1_nP1 );
            row[ index_adjust( m , n + 2 ) ] += zeroize( Sm_nP2 );

            ublas::row(K, index_adjust(m, n)) = row;
          }
      }

      SurfaceParticle & grid(size_t m, size_t n)
      {
        return m_P[ n * m_M + m ];
      }

      SurfaceParticle const & grid(size_t m, size_t n) const
      {
        return m_P[n * m_M + m];
      }

      tensor2_type metric(size_t m, size_t n) const
      {
        const vector3_type v1 = D1Pb(m, n);
        const vector3_type v2 = D2Pb(m, n);

        tensor2_type G;
        G.t0[ 0 ] = v1 * v1;  // G11
        G.t0[ 1 ] = G.t1[ 0 ] = v1 * v2;  // G12 & G21
        G.t1[ 1 ] = v2 * v2;  // G22
        return tensor2_type(G);
      }

      tensor2_type curvature(size_t m, size_t n) const
      {
        const vector3_type v11 = D11b(m, n);
        const vector3_type v22 = D22b(m, n);
        const vector3_type v12 = D12Pb(m, n);

        //const vector3 n_ = normal(m,n);
        const vector3_type &n_ = particle( m, n ).n;
        tensor2_type B;
        B.t0[ 0 ] = n_ * v11;
        B.t0[ 1 ] = B.t1[ 0 ] = n_ * v12;
        B.t1[ 1 ] = n_ * v22;
        return tensor2_type(B);
      }

      void compute_surface_normals()
      {
        for (size_t n = 0; n < m_N; ++n)
          for (size_t m = 0; m < m_M; ++m)
            grid(m, n).n = normal(m, n);
      }

      size_t particle_count() const
      {
        return size_t(static_cast<size_t>(m_P.size()));
      }

      particle_type const & get_particle(size_t idx) const
      {
        return m_P[idx];
      }

      particle_type & get_particle(size_t idx)
      {
        return m_P[idx];
      }

      vector3_type D1(size_t m, size_t n) const
      {
        return vector3_type((r(m+1,n)-r(m-1,n))/(2.*m_h1));
      }
      vector3_type D2(size_t m, size_t n) const
      {
        return vector3_type((r(m,n+1)-r(m,n-1))/(2.*m_h2));
      }
      vector3_type D1Pb(size_t m, size_t n) const
      {
        return vector3_type(index_check(m+1,n)?(r(m+1,n)-r(m,n))/m_h1:vector3_type(0));
      }
      vector3_type D2Pb(size_t m, size_t n) const
      {
        return vector3_type(index_check(m,n+1)?(r(m,n+1)-r(m,n))/m_h2:vector3_type(0));
      }
      vector3_type D11b(size_t m, size_t n) const
      {
        return vector3_type(index_check(m+1,n)&&index_check(m-1,n)?(r(m+1,n)-2.*r(m,n)+r(m-1,n))/(m_h1*m_h1):vector3_type(0));
      }
      vector3_type D22b(size_t m, size_t n) const
      {
        return vector3_type(index_check(m,n+1)&&index_check(m,n-1)?(r(m,n+1)-2.*r(m,n)+r(m,n-1))/(m_h2*m_h2):vector3_type(0));
      }
      vector3_type D12Pb(size_t m, size_t n) const
      {
        return vector3_type(index_check(m+1,n+1)?(r(m+1,n+1)-r(m,n+1)-r(m+1,n)+r(m,n))/(m_h1*m_h2):vector3_type(0));
      }
      vector3_type D21Pb(size_t m, size_t n) const
      {
        return vector3_type(D12Pb(m,n));
      }

      vector3_type D1P(size_t m, size_t n) const
      {
        return vector3_type((r(m+1,n)-r(m,n))/m_h1);
      }

      vector3_type D2P(size_t m, size_t n) const
      {
        return vector3_type((r(m,n+1)-r(m,n))/m_h2);
      }

      vector3_type D1M(size_t m, size_t n) const
      {
        return vector3_type((r(m,n)-r(m-1,n))/m_h1);
      }

      vector3_type D2M(size_t m, size_t n) const
      {
        return vector3_type((r(m,n)-r(m,n-1))/m_h2);
      }

      vector3_type D11(size_t m, size_t n) const
      {
        return vector3_type((r(m+1,n)-2.*r(m,n)+r(m-1,n))/(m_h1*m_h1));
      }

      vector3_type D22(size_t m, size_t n) const
      {
        return vector3_type((r(m,n+1)-2.*r(m,n)+r(m,n-1))/(m_h2*m_h2));
      }

      vector3_type D12P(size_t m, size_t n) const
      {
        return vector3_type((r(m+1,n+1)-r(m,n+1)-r(m+1,n)+r(m,n))/(m_h1*m_h2));
      }

    }; // end of class Surface

  }  // namespace edm

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_SURFACE_H
#endif
