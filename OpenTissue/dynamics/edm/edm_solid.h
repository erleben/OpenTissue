#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_SOLID_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_SOLID_H
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
    class Solid 
      : public edm_types::model_type
    {
    public:
      typedef typename edm_types::model_type    base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::tensor2_type       tensor2_type;
      typedef typename edm_types::tensor3_type       tensor3_type;
      typedef typename edm_types::Particle      particle_type;
      typedef typename base_type::EDMMatrix     EDMMatrix;
      typedef typename base_type::EDMVector     EDMVector;

      struct SolidParticle
        : public particle_type
      {
        SolidParticle()
          : particle_type()
        {}
        virtual ~SolidParticle() {}

        tensor3_type  e;   ///< solid tension (eta)
        tensor3_type  p;   ///< solid piller (rho)
        tensor2_type  u;   ///< solid spatial (upsilon)
        tensor3_type G0;  ///< solid natural xtended metric (metrix) shape
        tensor3_type P0;  ///< solid natural pillar shape
        tensor2_type N0;  ///< solid natural spatial shape
      };

      typedef std::vector<SolidParticle>  SolidParticles;

    protected:
      
      size_t  m_L, m_M, m_N;
      bool  m_wrap_L, m_wrap_M, m_wrap_N;
      
    private:
      
      real_type  m_h1, m_h2, m_h3;
      SolidParticles  m_P;
      
    public:

      Solid()
        : base_type(edm_types::EDM_Solid)
        , m_L(0)
        , m_M(0)
        , m_N(0)
        , m_wrap_L(false)
        , m_wrap_M(false)
        , m_wrap_N(false)
        , m_h1(value_traits::one())
        , m_h2(value_traits::one())
        , m_h3(value_traits::one())
      {}

      virtual ~Solid() {}

    public:

      bool initialize(size_t L, size_t M, size_t N)
      {
        if ( !m_P.empty() || L < 3 || M < 3 || N < 3 )   // if already created or too few particles
          return false;

        // create particles
        m_P.resize((m_L = L)*(m_M = M)*(m_N = N));
        real_type const du = m_h1 = value_traits::one() / ( L - 1 );
        real_type const dv = m_h2 = value_traits::one() / ( M - 1 );
        real_type const dw = m_h3 = value_traits::one() / ( N - 1 );
        real_type u, v, w;
        u = v = w = value_traits::zero();
        for ( size_t n = 0; n < N; n++, w += dw, v = 0 )
          for ( size_t m = 0; m < M; m++, v += dv, u = 0 )
            for ( size_t l = 0; l < L; l++, u += du )
              grid( l, m, n ).r = position( this->m_rest, u, v, w );  // calculate the natural shape position for this particle

        // create natural extended shape metrix tensor G0, spatial tensor N0, and pillar tensor P0.
        for ( size_t n = 0; n < N; ++n )
          for ( size_t m = 0; m < M; ++m )
            for ( size_t l = 0; l < L; ++l )
            {
              SolidParticle & a = grid( l, m, n );
              a.G0 = metrix( l, m, n );
              a.N0 = spatial( l, m, n );
              a.P0 = pillar( l, m, n );
            }

        u = v = w = value_traits::zero();
        for ( size_t n = 0; n < N; n++, w += dw, v = 0 )
          for ( size_t m = 0; m < M; m++, v += dv, u = 0 )
            for ( size_t l = 0; l < L; l++, u += du )
            {
              SolidParticle & a = grid( l, m, n );
                a.r = position( this->m_init, u, v, w );  // calculate the initial shape position for this particle
              a.o = a.r;
              a.v *= value_traits::zero();

              //TODO: Textures on some edges that share the same UV TexCoords are not correct,
              //      due to the fact that we only have 1 texture to use on all 6 faces!
              if ( l == 0 || l == L - 1 )
              {
                a.t.u = math::clamp_zero_one(w);
                a.t.v = math::clamp_zero_one(v);
              }
              else if ( m == 0 || m == M - 1 )
              {
                a.t.u = math::clamp_zero_one(w);
                a.t.v = math::clamp_zero_one(u);
              }
              else if ( n == 0 || n == N - 1 )
              {
                a.t.u = math::clamp_zero_one(v);
                a.t.v = math::clamp_zero_one(u);
              }
            }

        // create initial surface normals
        compute_surface_normals();

        return true;
      }

      Solid & wrapping(bool L, bool M, bool N)
      {
        m_wrap_L = L;
        m_wrap_M = M;
        m_wrap_N = N;
        return *this;
      }

      size_t get_num_L(bool include_wrapping = false) const
      {
        return size_t(m_L + (!include_wrapping ? 0 : m_wrap_L ? 1 : 0));
      }

      size_t get_num_M(bool include_wrapping = false) const
      {
        return size_t(m_M + (!include_wrapping ? 0 : m_wrap_M ? 1 : 0));
      }

      size_t get_num_N(bool include_wrapping = false) const
      {
        return size_t(m_N + (!include_wrapping ? 0 : m_wrap_N ? 1 : 0));
      }

      SolidParticle const & particle(size_t l, size_t m, size_t n) const
      {
        return grid(l, m, n);
      }

      // creation of the parametric solid
      size_t nodes() const
      {
        return size_t(this->m_max_nodes);
      }

      size_t index_adjust(long l, long m, long n) const
      {
        // TODO: use math::clamp instead of min/max
        using std::min;
        using std::max;

        size_t const l_ = m_wrap_L ? l % ( ( l < 0 ? -1 : 1 ) * static_cast<long>( m_L ) ) + ( l < 0 ? m_L : 0 ) : static_cast<size_t>( max( 0L, min( l, static_cast<long>( m_L - 1 ) ) ) );
        size_t const m_ = m_wrap_M ? m % ( ( m < 0 ? -1 : 1 ) * static_cast<long>( m_M ) ) + ( m < 0 ? m_M : 0 ) : static_cast<size_t>( max( 0L, min( m, static_cast<long>( m_M - 1 ) ) ) );
        size_t const n_ = m_wrap_N ? n % ( ( n < 0 ? -1 : 1 ) * static_cast<long>( m_N ) ) + ( n < 0 ? m_N : 0 ) : static_cast<size_t>( max( 0L, min( n, static_cast<long>( m_N - 1 ) ) ) );
        return size_t(n_*(m_L*m_M) + m_*m_L + l_);
      }

      Solid & set_natural_position(size_t idx, vector3_type const & r)
      {
        assert(idx < this->m_max_nodes || !"set_natural_position: idx out of range.");
        this->m_rest[idx] = r;
        return set_initial_position(idx, r);  // default behavior if init pos isn't called explicitly
      }

      Solid & set_initial_position(size_t idx, vector3_type const & r)
      {
        assert(idx < this->m_max_nodes || !"set_natural_position: idx out of range.");
        this->m_init[idx] = r;
        return *this;
      }

      // particles (initialize must be called before);
      Solid & set_fixed(size_t l, size_t m, size_t n, bool fixed)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).f = fixed ;
        return *this;
      }

      Solid & set_mass(size_t l, size_t m, size_t n, real_type const & mass)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).m = mass;
        return *this;
      }

      Solid & set_damping(size_t l, size_t m, size_t n, real_type const & damping)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).g = damping;
        return *this;
      }

      Solid & set_tension(size_t l, size_t m, size_t n, tensor3_type const & tension)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).e = tension;
        set_pillar(l, m, n, tension);  // original default behavior: pillar uses the same as tension
        return *this;
      }

      Solid & set_pillar(size_t l, size_t m, size_t n, tensor3_type const & pillar)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).p = pillar;
        return *this;
      }

      Solid & set_spatial(size_t l, size_t m, size_t n, tensor2_type const & spatial)
      {
        assert(l < m_L && m < m_M && n < m_N);
        grid(l, m, n).u = spatial;
        return *this;
      }

    protected:

      bool index_check(long l, long m, long n) const
      {
        return ( m_wrap_L ? true : 0 <= l && l < static_cast<long>( m_L ) ) && ( m_wrap_M ? true : 0 <= m && m < static_cast<long>( m_M ) ) && ( m_wrap_N ? true : 0 <= n && n < static_cast<long>( m_N ) );
      }

      SolidParticle const & grid_adjust(long l, long m, long n) const
      {
        return m_P[index_adjust(l, m, n)];
      }

      vector3_type const & r(long l, long m, long n) const
      {
        return grid_adjust(l, m, n).r;
      }

    private:

      virtual vector3_type position(vector3_type const * a, real_type const & u, real_type const & v, real_type const & w ) const = 0;
      virtual vector3_type normal(size_t l, size_t m, size_t n) const = 0;

    private:

      void compute_stiffness(EDMMatrix & K) const
      {
        typedef std::vector<tensor3_type> Tensors;
        typedef std::vector<tensor2_type> Tensorx;

        //precalculation of tensors
        Tensors alpha( m_P.size() );
        Tensors rho( m_P.size() );
        Tensorx nu( m_P.size() );
        for ( size_t n = 0; n < m_N; ++n )
          for ( size_t m = 0; m < m_M; ++m )
            for ( size_t l = 0; l < m_L; ++l )
            {
              size_t const i = index_adjust( l, m, n );
              SolidParticle const & a = grid( l, m, n );

              tensor3_type& alpha_ = alpha[ i ];
              tensor3_type m_ = metrix( l, m, n );
              alpha_.t0[ 0 ] = this->m_strength * a.e.t0[ 0 ] * ( m_.t0[ 0 ] - a.G0.t0[ 0 ] );
              alpha_.t1[ 1 ] = this->m_strength * a.e.t1[ 1 ] * ( m_.t1[ 1 ] - a.G0.t1[ 1 ] );
              alpha_.t2[ 2 ] = this->m_strength * a.e.t2[ 2 ] * ( m_.t2[ 2 ] - a.G0.t2[ 2 ] );
              alpha_.t0[ 1 ] = this->m_strength * a.e.t0[ 1 ] * ( m_.t0[ 1 ] - a.G0.t0[ 1 ] );
              alpha_.t1[ 0 ] = this->m_strength * a.e.t1[ 0 ] * ( m_.t1[ 0 ] - a.G0.t1[ 0 ] );
              alpha_.t0[ 2 ] = this->m_strength * a.e.t0[ 2 ] * ( m_.t0[ 2 ] - a.G0.t0[ 2 ] );
              alpha_.t2[ 0 ] = this->m_strength * a.e.t2[ 0 ] * ( m_.t2[ 0 ] - a.G0.t2[ 0 ] );
              alpha_.t1[ 2 ] = this->m_strength * a.e.t1[ 2 ] * ( m_.t1[ 2 ] - a.G0.t1[ 2 ] );
              alpha_.t2[ 1 ] = this->m_strength * a.e.t2[ 1 ] * ( m_.t2[ 1 ] - a.G0.t2[ 1 ] );

              tensor3_type& rho_ = rho[ i ];
              tensor3_type p_ = pillar( l, m, n );
              rho_.t0[ 0 ] = this->m_strength * a.p.t0[ 0 ] * ( p_.t0[ 0 ] - a.P0.t0[ 0 ] );
              rho_.t1[ 1 ] = this->m_strength * a.p.t1[ 1 ] * ( p_.t1[ 1 ] - a.P0.t1[ 1 ] );
              rho_.t2[ 2 ] = this->m_strength * a.p.t2[ 2 ] * ( p_.t2[ 2 ] - a.P0.t2[ 2 ] );

              tensor2_type& nu_ = nu[ i ];
              tensor2_type x_ = spatial( l, m, n );
              nu_.t0[ 0 ] = this->m_strength * a.u.t0[ 0 ] * ( x_.t0[ 0 ] - a.N0.t0[ 0 ] );
              nu_.t0[ 1 ] = this->m_strength * a.u.t0[ 1 ] * ( x_.t0[ 1 ] - a.N0.t0[ 1 ] );
              nu_.t1[ 0 ] = this->m_strength * a.u.t1[ 0 ] * ( x_.t1[ 0 ] - a.N0.t1[ 0 ] );
              nu_.t1[ 1 ] = this->m_strength * a.u.t1[ 1 ] * ( x_.t1[ 1 ] - a.N0.t1[ 1 ] );
            }

        real_type const inv_h1h1 = 1. / ( m_h1 * m_h1 );
        real_type const inv_h2h2 = 1. / ( m_h2 * m_h2 );
        real_type const inv_h3h3 = 1. / ( m_h3 * m_h3 );
        real_type const inv_len2 = 1. / ( m_h1 * m_h1 + m_h2 * m_h2 + m_h3 * m_h3 );
        real_type const inv_4h1h1 = 0.25 * inv_h1h1;
        real_type const inv_4h2h2 = 0.25 * inv_h2h2;
        real_type const inv_4h3h3 = 0.25 * inv_h3h3;
        real_type const iLL12 = 1. / ( m_h1 * m_h1 + m_h2 * m_h2 );
        real_type const iLL13 = 1. / ( m_h1 * m_h1 + m_h3 * m_h3 );
        real_type const iLL23 = 1. / ( m_h2 * m_h2 + m_h3 * m_h3 );
        size_t const LMN = m_L*m_M*m_N;

        for ( size_t n = 0; n < m_N; ++n )
          for ( size_t m = 0; m < m_M; ++m )
            for ( size_t l = 0; l < m_L; ++l )
            {
              // rho (pillar) 5x5x5 stencil part
              tensor3_type const & plM1_m_n = rho[index_adjust(l-1, m, n)];
              tensor3_type const & plP1_m_n = rho[index_adjust(l+1, m ,n)];
              tensor3_type const & pl_mM1_n = rho[index_adjust(l, m-1, n)];
              tensor3_type const & pl_mP1_n = rho[index_adjust(l, m+1, n)];
              tensor3_type const & pl_m_nM1 = rho[index_adjust(l, m, n-1)];
              tensor3_type const & pl_m_nP1 = rho[index_adjust(l, m, n+1)];

              real_type Pl_m_n = 0;
              real_type PlP2_m_n = 0;
              real_type PlM2_m_n = 0;
              real_type Pl_mP2_n = 0;
              real_type Pl_mM2_n = 0;
              real_type Pl_m_nP2 = 0;
              real_type Pl_m_nM2 = 0;

              // - 1/4h1h1*v11[l+1,m,n] * (r[l+2,m,n]-r[l,m,n])
              if ( index_check( l + 2, m, n ) )
              {
                real_type const tmp = - inv_4h1h1 * plP1_m_n.t0[ 0 ];
                PlP2_m_n += + tmp;
                Pl_m_n += - tmp;
              }

              // + 1/4h1h1*v11[l-1,m,n] * (r[l,m,n]-r[l-2,m,n])
              if ( index_check( l - 2, m, n ) )
              {
                real_type const tmp = + inv_4h1h1 * plM1_m_n.t0[ 0 ];
                Pl_m_n += + tmp;
                PlM2_m_n += - tmp;
              }

              // - 1/4h2h2*v22[l,m+1,n] * (r[l,m+2,n]-r[l,m,n])
              if ( index_check( l, m + 2, n ) )
              {
                real_type const tmp = - inv_4h2h2 * pl_mP1_n.t1[ 1 ];
                Pl_mP2_n += + tmp;
                Pl_m_n += - tmp;
              }

              // + 1/4h2h2*v22[l,m-1,n] * (r[l,m,n]-r[l,m-2,n])
              if ( index_check( l, m - 2, n ) )
              {
                real_type const tmp = + inv_4h2h2 * pl_mM1_n.t1[ 1 ];
                Pl_m_n += + tmp;
                Pl_mM2_n += - tmp;
              }

              // - 1/4h3h3*v33[l,m,n+1] * (r[l,m,n+2]-r[l,m,n])
              if ( index_check( l, m, n + 2 ) )
              {
                real_type const tmp = - inv_4h3h3 * pl_m_nP1.t2[ 2 ];
                Pl_m_nP2 += + tmp;
                Pl_m_n += - tmp;
              }

              // + 1/4h3h3*v33[l,m,n-1] * (r[l,m,n]-r[l,m,n-2])
              if ( index_check( l, m, n - 2 ) )
              {
                real_type const tmp = + inv_4h3h3 * pl_m_nM1.t2[ 2 ];
                Pl_m_n += + tmp;
                Pl_m_nM2 += - tmp;
              }


              // nu (spatial) 3x3x3 stencil part
              tensor2_type const & vl_m_n = nu[index_adjust(l, m, n)];
              tensor2_type const & vlP1_mM1_nM1 = nu[index_adjust(l+1, m-1, n-1)];
              tensor2_type const & vlP1_mP1_nM1 = nu[index_adjust(l+1, m+1, n-1)];
              tensor2_type const & vlM1_mM1_nM1 = nu[index_adjust(l-1, m-1, n-1)];
              tensor2_type const & vlM1_mP1_nM1 = nu[index_adjust(l-1, m+1, n-1)];

              real_type Vl_m_n = 0;
              real_type VlM1_mP1_nP1 = 0;
              real_type VlP1_mM1_nM1 = 0;
              real_type VlM1_mM1_nP1 = 0;
              real_type VlP1_mP1_nM1 = 0;
              real_type VlP1_mP1_nP1 = 0;
              real_type VlM1_mM1_nM1 = 0;
              real_type VlP1_mM1_nP1 = 0;
              real_type VlM1_mP1_nM1 = 0;

              // - iLen2*v11[l,m,n] * (r[l-1,m+1,n+1]-r[l,m,n])
              if ( index_check( l - 1, m + 1, n + 1 ) )
              {
                real_type const tmp = - inv_len2 * vl_m_n.t0[ 0 ];
                VlM1_mP1_nP1 += + tmp;
                Vl_m_n += - tmp;
              }

              // + iLen2*v11[l+1,m-1,n-1] * (r[l,m,n]-r[l+1,m-1,n-1])
              if ( index_check( l + 1, m - 1, n - 1 ) )
              {
                real_type const tmp = + inv_len2 * vlP1_mM1_nM1.t0[ 0 ];
                Vl_m_n += + tmp;
                VlP1_mM1_nM1 += - tmp;
              }

              // - iLen2*v13[l,m,n] * (r[l-1,m-1,n+1]-r[l,m,n])
              if ( index_check( l - 1, m - 1, n + 1 ) )
              {
                real_type const tmp = - inv_len2 * vl_m_n.t0[ 1 ];
                VlM1_mM1_nP1 += + tmp;
                Vl_m_n += - tmp;
              }

              // + iLen2*v13[l+1,m+1,n-1] * (r[l,m,n]-r[l+1,m+1,n-1])
              if ( index_check( l + 1, m + 1, n - 1 ) )
              {
                real_type const tmp = + inv_len2 * vlP1_mP1_nM1.t0[ 1 ];
                Vl_m_n += + tmp;
                VlP1_mP1_nM1 += - tmp;
              }

              // - iLen2*v31[l,m,n] * (r[l+1,m+1,n+1]-r[l,m,n])
              if ( index_check( l + 1, m + 1, n + 1 ) )
              {
                real_type const tmp = - inv_len2 * vl_m_n.t1[ 0 ];
                VlP1_mP1_nP1 += + tmp;
                Vl_m_n += - tmp;
              }

              // + iLen2*v31[l-1,m-1,n-1] * (r[l,m,n]-r[l-1,m-1,n-1])
              if ( index_check( l - 1, m - 1, n - 1 ) )
              {
                real_type const tmp = + inv_len2 * vlM1_mM1_nM1.t1[ 0 ];
                Vl_m_n += + tmp;
                VlM1_mM1_nM1 += - tmp;
              }

              // - iLen2*v33[l,m,n] * (r[l+1,m-1,n+1]-r[l,m,n])
              if ( index_check( l + 1, m - 1, n + 1 ) )
              {
                real_type const tmp = - inv_len2 * vl_m_n.t1[ 1 ];
                VlP1_mM1_nP1 += + tmp;
                Vl_m_n += - tmp;
              }

              // + iLen2*v33[l-1,m+1,n-1] * (r[l,m,n]-r[l-1,m+1,n-1])
              if ( index_check( l - 1, m + 1, n - 1 ) )
              {
                real_type const tmp = + inv_len2 * vlM1_mP1_nM1.t1[ 1 ];
                Vl_m_n += + tmp;
                VlM1_mP1_nM1 += - tmp;
              }


              // create the 3x3x3 alpha stencil A
              tensor3_type const & al_m_n = alpha[index_adjust(l, m, n)];
              tensor3_type const & alM1_m_n = alpha[index_adjust(l-1, m, n)];
              tensor3_type const & al_mM1_n = alpha[index_adjust(l, m-1, n)];
              tensor3_type const & al_m_nM1 = alpha[index_adjust(l, m, n-1)];
              tensor3_type const & alM1_mM1_n = alpha[index_adjust(l-1, m-1, n)];
              tensor3_type const & alP1_mM1_n = alpha[index_adjust(l+1, m-1, n)];
              tensor3_type const & alM1_m_nM1 = alpha[index_adjust(l-1, m, n-1)];
              tensor3_type const & alP1_m_nM1 = alpha[index_adjust(l+1, m, n-1)];
              tensor3_type const & al_mM1_nM1 = alpha[index_adjust(l, m-1, n-1)];
              tensor3_type const & al_mP1_nM1 = alpha[index_adjust(l ,m+1, n-1)];

              real_type SlP2_m_n = PlP2_m_n;
              real_type SlM2_m_n = PlM2_m_n;
              real_type Sl_mP2_n = Pl_mP2_n;
              real_type Sl_mM2_n = Pl_mM2_n;
              real_type Sl_m_nP2 = Pl_m_nP2;
              real_type Sl_m_nM2 = Pl_m_nM2;
              real_type SlP1_mM1_nP1 = VlP1_mM1_nP1;
              real_type SlM1_mP1_nM1 = VlM1_mP1_nM1;
              real_type SlP1_mM1_nM1 = VlP1_mM1_nM1;
              real_type SlM1_mM1_nP1 = VlM1_mM1_nP1;
              real_type SlP1_mP1_nM1 = VlP1_mP1_nM1;
              real_type SlM1_mP1_nP1 = VlM1_mP1_nP1;
              real_type SlP1_mP1_nP1 = VlP1_mP1_nP1;
              real_type SlM1_mM1_nM1 = VlM1_mM1_nM1;
              real_type SlP1_m_n = 0;
              real_type Sl_mP1_n = 0;
              real_type Sl_m_nP1 = 0;
              real_type Sl_m_n = Vl_m_n + Pl_m_n;
              real_type SlM1_m_n = 0;
              real_type Sl_mM1_n = 0;
              real_type Sl_m_nM1 = 0;
              real_type SlM1_mP1_n = 0;
              real_type SlM1_m_nP1 = 0;
              real_type SlP1_mM1_n = 0;
              real_type Sl_mM1_nP1 = 0;
              real_type SlP1_m_nM1 = 0;
              real_type Sl_mP1_nM1 = 0;
              real_type SlP1_mP1_n = 0;
              real_type SlM1_mM1_n = 0;
              real_type SlP1_m_nP1 = 0;
              real_type SlM1_m_nM1 = 0;
              real_type Sl_mP1_nP1 = 0;
              real_type Sl_mM1_nM1 = 0;

              // D1P(r[l,m,n]) = r[l+1,m,n]-r[l,m,n]
              if ( index_check( l + 1, m, n ) )
              {
                real_type const tmp = inv_h1h1 * al_m_n.t0[ 0 ];
                SlP1_m_n += - tmp;
                Sl_m_n += + tmp;
              }

              // D1P(r[l-1,m,n]) = r[l,m,n]-r[l-1,m,n]
              if ( index_check( l - 1, m, n ) )
              {
                real_type const tmp = inv_h1h1 * alM1_m_n.t0[ 0 ];
                Sl_m_n += + tmp;
                SlM1_m_n += - tmp;
              }

              // D2P(r[l,m,n]) = r[l,m+1,n]-r[l,m,n]
              if ( index_check( l, m + 1, n ) )
              {
                real_type const tmp = inv_h2h2 * al_m_n.t1[ 1 ];
                Sl_mP1_n += - tmp;
                Sl_m_n += + tmp;
              }

              // D2P(r[l,m-1,n]) = r[l,m,n]-r[l,m-1,n]
              if ( index_check( l, m - 1, n ) )
              {
                real_type const tmp = inv_h2h2 * al_mM1_n.t1[ 1 ];
                Sl_m_n += + tmp;
                Sl_mM1_n += - tmp;
              }

              // D3P(r[l,m,n]) = r[l,m,n+1]-r[l,m,n]
              if ( index_check( l, m, n + 1 ) )
              {
                real_type const tmp = inv_h3h3 * al_m_n.t2[ 2 ];
                Sl_m_nP1 += - tmp;
                Sl_m_n += + tmp;
              }

              // D3P(r[l,m,n-1]) = r[l,m,n]-r[l,m,n-1]
              if ( index_check( l, m, n - 1 ) )
              {
                real_type const tmp = inv_h3h3 * al_m_nM1.t2[ 2 ];
                Sl_m_n += + tmp;
                Sl_m_nM1 += - tmp;
              }

              // - iLL12*v12[l,m,n] * (r[l+1,m+1,n]-r[l,m,n])
              if ( index_check( l + 1, m + 1, n ) )
              {
                real_type const tmp = - iLL12 * al_m_n.t0[ 1 ];
                SlP1_mP1_n += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL12*v12[l-1,m-1,n] * (r[m,n,l]-r[l-1,m-1,n])
              if ( index_check( l - 1, m - 1, n ) )
              {
                real_type const tmp = + iLL12 * alM1_mM1_n.t0[ 1 ];
                Sl_m_n += + tmp;
                SlM1_mM1_n += - tmp;
              }

              // - iLL12*v21[l,m,n] * (r[l-1,m+1,n]-r[l,m,n])
              if ( index_check( l - 1, m + 1, n ) )
              {
                real_type const tmp = - iLL12 * al_m_n.t1[ 0 ];
                SlM1_mP1_n += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL12*v21[l+1,m-1,n] * (r[l,m,n]-r[l+1,m-1,n])
              if ( index_check( l + 1, m - 1, n ) )
              {
                real_type const tmp = + iLL12 * alP1_mM1_n.t1[ 0 ];
                Sl_m_n += + tmp;
                SlP1_mM1_n += - tmp;
              }

              // - iLL13*v13[l,m,n] * (r[l+1,m,n+1]-r[l,m,n])
              if ( index_check( l + 1, m, n + 1 ) )
              {
                real_type const tmp = - iLL13 * al_m_n.t0[ 2 ];
                SlP1_m_nP1 += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL13*v13[l-1,m,n-1] * (r[m,n,l]-r[l-1,m,n-1])
              if ( index_check( l - 1, m, n - 1 ) )
              {
                real_type const tmp = + iLL13 * alM1_m_nM1.t0[ 2 ];
                Sl_m_n += + tmp;
                SlM1_m_nM1 += - tmp;
              }

              // - iLL13*v31[l,m,n] * (r[l-1,m,n+1]-r[l,m,n])
              if ( index_check( l - 1, m, n + 1 ) )
              {
                real_type const tmp = - iLL13 * al_m_n.t2[ 0 ];
                SlM1_m_nP1 += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL13*v31[l+1,m,n-1] * (r[l,m,n]-r[l+1,m,n-1])
              if ( index_check( l + 1, m, n - 1 ) )
              {
                real_type const tmp = + iLL13 * alP1_m_nM1.t2[ 0 ];
                Sl_m_n += + tmp;
                SlP1_m_nM1 += - tmp;
              }

              // - iLL23*v23[l,m,n] * (r[l,m+1,n+1]-r[l,m,n])
              if ( index_check( l, m + 1, n + 1 ) )
              {
                real_type const tmp = - iLL23 * al_m_n.t1[ 2 ];
                Sl_mP1_nP1 += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL23*v23[l,m-1,n-1] * (r[m,n,l]-r[l,m-1,n-1])
              if ( index_check( l, m - 1, n - 1 ) )
              {
                real_type const tmp = + iLL23 * al_mM1_nM1.t1[ 2 ];
                Sl_m_n += + tmp;
                Sl_mM1_nM1 += - tmp;
              }

              // - iLL23*v32[l,m,n] * (r[l,m-1,n+1]-r[l,m,n])
              if ( index_check( l, m - 1, n + 1 ) )
              {
                real_type const tmp = - iLL23 * al_m_n.t2[ 1 ];
                Sl_mM1_nP1 += + tmp;
                Sl_m_n += - tmp;
              }

              // + iLL23*v32[l,m+1,n-1] * (r[l,m,n]-r[l,m+1,n-1])
              if ( index_check( l, m + 1, n - 1 ) )
              {
                real_type const tmp = + iLL23 * al_mP1_nM1.t2[ 1 ];
                Sl_m_n += + tmp;
                Sl_mP1_nM1 += - tmp;
              }

              EDMVector row( LMN );
              row.clear();  // NOTICE: This was the culprit in the upgrade to boost_1_32_0! Construction is uninitialized.
              row( index_adjust( l + 2, m , n ) ) += zeroize( SlP2_m_n );
              row( index_adjust( l - 2, m , n ) ) += zeroize( SlM2_m_n );
              row( index_adjust( l , m + 2, n ) ) += zeroize( Sl_mP2_n );
              row( index_adjust( l , m - 2, n ) ) += zeroize( Sl_mM2_n );
              row( index_adjust( l , m , n + 2 ) ) += zeroize( Sl_m_nP2 );
              row( index_adjust( l , m , n - 2 ) ) += zeroize( Sl_m_nM2 );
              row( index_adjust( l + 1, m + 1, n ) ) += zeroize( SlP1_mP1_n );
              row( index_adjust( l - 1, m - 1, n ) ) += zeroize( SlM1_mM1_n );
              row( index_adjust( l + 1, m , n + 1 ) ) += zeroize( SlP1_m_nP1 );
              row( index_adjust( l - 1, m , n - 1 ) ) += zeroize( SlM1_m_nM1 );
              row( index_adjust( l , m + 1, n + 1 ) ) += zeroize( Sl_mP1_nP1 );
              row( index_adjust( l , m - 1, n - 1 ) ) += zeroize( Sl_mM1_nM1 );
              row( index_adjust( l + 1, m - 1, n + 1 ) ) += zeroize( SlP1_mM1_nP1 );
              row( index_adjust( l - 1, m + 1, n - 1 ) ) += zeroize( SlM1_mP1_nM1 );
              row( index_adjust( l + 1, m - 1, n - 1 ) ) += zeroize( SlP1_mM1_nM1 );
              row( index_adjust( l - 1, m - 1, n + 1 ) ) += zeroize( SlM1_mM1_nP1 );
              row( index_adjust( l + 1, m + 1, n - 1 ) ) += zeroize( SlP1_mP1_nM1 );
              row( index_adjust( l - 1, m + 1, n + 1 ) ) += zeroize( SlM1_mP1_nP1 );
              row( index_adjust( l - 1, m - 1, n - 1 ) ) += zeroize( SlM1_mM1_nM1 );
              row( index_adjust( l + 1, m + 1, n + 1 ) ) += zeroize( SlP1_mP1_nP1 );
              row( index_adjust( l , m , n - 1 ) ) += zeroize( Sl_m_nM1 );
              row( index_adjust( l + 1, m , n - 1 ) ) += zeroize( SlP1_m_nM1 );
              row( index_adjust( l , m + 1, n - 1 ) ) += zeroize( Sl_mP1_nM1 );
              row( index_adjust( l , m - 1, n ) ) += zeroize( Sl_mM1_n );
              row( index_adjust( l + 1, m - 1, n ) ) += zeroize( SlP1_mM1_n );
              row( index_adjust( l - 1, m , n ) ) += zeroize( SlM1_m_n );
              row( index_adjust( l , m , n ) ) += zeroize( Sl_m_n );
              row( index_adjust( l + 1, m , n ) ) += zeroize( SlP1_m_n );
              row( index_adjust( l - 1, m + 1, n ) ) += zeroize( SlM1_mP1_n );
              row( index_adjust( l , m + 1, n ) ) += zeroize( Sl_mP1_n );
              row( index_adjust( l , m - 1, n + 1 ) ) += zeroize( Sl_mM1_nP1 );
              row( index_adjust( l - 1, m , n + 1 ) ) += zeroize( SlM1_m_nP1 );
              row( index_adjust( l , m , n + 1 ) ) += zeroize( Sl_m_nP1 );

              ublas::row(K, index_adjust( l, m, n) ) = row;
            }
      }

      SolidParticle & grid(size_t l, size_t m, size_t n)
      {
        return m_P[n*(m_L*m_M) + m*m_L + l];
      }

      SolidParticle const & grid(size_t l, size_t m, size_t n) const
      {
        return m_P[n*(m_L*m_M) + m*m_L + l];
      }

      tensor3_type metrix(size_t l, size_t m, size_t n) const
      {
        vector3_type const v1 = D1Pb( l, m, n );
        vector3_type const v2 = D2Pb( l, m, n );
        vector3_type const v3 = D3Pb( l, m, n );

        vector3_type const v12 = index_check( l + 1, m + 1, n ) ? ( r( l + 1, m + 1, n ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v21 = index_check( l - 1, m + 1, n ) ? ( r( l - 1, m + 1, n ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v13 = index_check( l + 1, m, n + 1 ) ? ( r( l + 1, m, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v31 = index_check( l - 1, m, n + 1 ) ? ( r( l - 1, m, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v23 = index_check( l, m + 1, n + 1 ) ? ( r( l, m + 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v32 = index_check( l, m - 1, n + 1 ) ? ( r( l, m - 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        real_type const iLL12 = 1. / ( m_h1 * m_h1 + m_h2 * m_h2 );
        real_type const iLL13 = 1. / ( m_h1 * m_h1 + m_h3 * m_h3 );
        real_type const iLL23 = 1. / ( m_h2 * m_h2 + m_h3 * m_h3 );

        tensor3_type G;
        G.t0[ 0 ] = v1 * v1;  // G11
        G.t1[ 1 ] = v2 * v2;  // G22
        G.t2[ 2 ] = v3 * v3;  // G33

        G.t0[ 1 ] = ( v12 * v12 ) * iLL12;
        G.t1[ 0 ] = ( v21 * v21 ) * iLL12;
        G.t0[ 2 ] = ( v13 * v13 ) * iLL13;
        G.t2[ 0 ] = ( v31 * v31 ) * iLL13;
        G.t1[ 2 ] = ( v23 * v23 ) * iLL23;
        G.t2[ 1 ] = ( v32 * v32 ) * iLL23;

        /*
          G._0[1] = (v12*v12); G._0[1] *= G._0[1]*iLL12;
          G._1[0] = (v21*v21); G._1[0] *= G._1[0]*iLL12;
          G._0[2] = (v13*v13); G._0[2] *= G._0[2]*iLL13;
          G._2[0] = (v31*v31); G._2[0] *= G._2[0]*iLL13;
          G._1[2] = (v23*v23); G._1[2] *= G._1[2]*iLL23;
          G._2[1] = (v32*v32); G._2[1] *= G._2[1]*iLL23;
        */
        return tensor3_type(G);
      }

      tensor2_type spatial(size_t l, size_t m, size_t n) const
      {
        vector3_type const v1 = index_check( l - 1, m + 1, n + 1 ) ? ( r( l - 1, m + 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v2 = index_check( l - 1, m - 1, n + 1 ) ? ( r( l - 1, m - 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v3 = index_check( l + 1, m + 1, n + 1 ) ? ( r( l + 1, m + 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        vector3_type const v4 = index_check( l + 1, m - 1, n + 1 ) ? ( r( l + 1, m - 1, n + 1 ) - r( l, m, n ) ) : vector3_type();
        real_type const iLen = 1. / ( m_h1 * m_h1 + m_h2 * m_h2 + m_h3 * m_h3 );

        tensor2_type N;
        /*
          N._0[0] = (v1*v1)*iLen;
          N._0[1] = (v2*v2)*iLen;
          N._1[0] = (v3*v3)*iLen;
          N._1[1] = (v4*v4)*iLen;
        */
        N.t0[ 0 ] = ( v1 * v1 );
        N.t0[ 0 ] *= N.t0[ 0 ] * iLen;
        N.t0[ 1 ] = ( v2 * v2 );
        N.t0[ 1 ] *= N.t0[ 1 ] * iLen;
        N.t1[ 0 ] = ( v3 * v3 );
        N.t1[ 0 ] *= N.t1[ 0 ] * iLen;
        N.t1[ 1 ] = ( v4 * v4 );
        N.t1[ 1 ] *= N.t1[ 1 ] * iLen;

        return tensor2_type(N);
      }

      tensor3_type pillar(size_t l, size_t m, size_t n) const
      {
        vector3_type const v1 = index_check( l + 1, m, n ) && index_check( l - 1, m, n ) ? ( r( l + 1, m, n ) - r( l - 1, m, n ) ) / ( 2. * m_h1 ) : vector3_type();
        vector3_type const v2 = index_check( l, m + 1, n ) && index_check( l, m - 1, n ) ? ( r( l, m + 1, n ) - r( l, m - 1, n ) ) / ( 2. * m_h2 ) : vector3_type();
        vector3_type const v3 = index_check( l, m, n + 1 ) && index_check( l, m, n - 1 ) ? ( r( l, m, n + 1 ) - r( l, m, n - 1 ) ) / ( 2. * m_h3 ) : vector3_type();

        tensor3_type P;
        P.t0[ 0 ] = v1 * v1;
        P.t1[ 1 ] = v2 * v2;
        P.t2[ 2 ] = v3 * v3;

        return tensor3_type(P);
      }

      void compute_surface_normals()
      {
        for (size_t n = 0; n < m_N; ++n)
          for (size_t m = 0; m < m_M; ++m)
            for (size_t l = 0; l < m_L; ++l)
              grid(l, m, n).n = normal(l, m, n);
      }

      size_t particle_count() const
      {
        return size_t(static_cast<size_t>(m_P.size()));
      }

      particle_type & get_particle(size_t idx)
      {
        return m_P[idx];
      }

      particle_type const & get_particle(size_t idx) const
      {
        return m_P[idx];
      }

      vector3_type D1Pb(size_t l, size_t m, size_t n) const
      {
        return vector3_type(index_check(l+1,m,n)?(r(l+1,m,n)-r(l,m,n))/m_h1:vector3_type(value_traits::zero()));
      }

      vector3_type D2Pb(size_t l, size_t m, size_t n) const
      {
        return vector3_type(index_check(l,m+1,n)?(r(l,m+1,n)-r(l,m,n))/m_h2:vector3_type(value_traits::zero()));
      }

      vector3_type D3Pb(size_t l, size_t m, size_t n) const
      {
        return vector3_type(index_check(l,m,n+1)?(r(l,m,n+1)-r(l,m,n))/m_h3:vector3_type(value_traits::zero()));
      }

     };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_SOLID_H
#endif
