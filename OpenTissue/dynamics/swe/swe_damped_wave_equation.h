#ifndef OPENTISSUE_DYNAMICS_SWE_SWE_DAMPED_WAVE_EQUATIONS_H
#define OPENTISSUE_DYNAMICS_SWE_SWE_DAMPED_WAVE_EQUATIONS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_empty_traits.h>

#include <cassert>
#include <vector>


namespace OpenTissue
{
  namespace swe
  {

    /**
    * Wave Equation with Damping (Simplified Linearized Shallow Water Equations)
    *
    * Implementation is based on the paper:
    *
    *   Gennadiy Nikishkov and Youhei Nishidate,
    *   "Fast Water Animation Using the Wave Equation with Damping",
    *   Workshop on Computer Graphics and Geometric Modeling, CGGM 2005, pp. 232-239.
    *
    *   http://www.u-aizu.ac.jp/~niki/papers/2005%20Fast%20water%20animation.pdf
    */
    template <typename math_types_, typename dwe_particle_traits = OpenTissue::utility::EmptyTraits >
    class DampedWaveEquations
    {
    public:

      typedef math_types_                         math_types;
      typedef typename math_types::real_type      real_type;
      typedef typename math_types::index_type     index_type;
      typedef typename math_types::vector3_type   vector3_type;

    public:

      class DWEParticle : public dwe_particle_traits
      {
      public:

        typedef typename math_types::real_type  real_type;

      public:
        DWEParticle()
          : m_Hold(0)
          , m_Hcur(0)
          , m_Hnew(0)
          , m_fixed(false)
        {}
        virtual ~DWEParticle()
        {}

      public:
        real_type const& height() const {return m_Hcur;}
        real_type& height() {return m_Hcur;}
        bool const& fixed() const {return m_fixed;}
        bool& fixed() {return m_fixed;}

      protected:
        friend class DampedWaveEquations;
        real_type m_Hold, m_Hcur, m_Hnew;
        bool m_fixed;

      };

      typedef DWEParticle  dwe_particle;
      typedef std::vector<dwe_particle>  particle_container;
      typedef typename particle_container::iterator  particle_iterator;
      typedef typename particle_container::const_iterator  particle_const_iterator;

    public:
      DampedWaveEquations()
        : m_I(0)
        , m_J(0)
        , m_k(0.5)
        , m_c(50)
        , m_D(5)
        , m_smooth(true)
      {}
      virtual ~DampedWaveEquations()
      {}

    public:
      /**
      * Initialize the DWE system.
      * - Must be called before anything can happen.
      *
      * @param columns          amount of particles in the "x"-direction.
      * @param rows             amount of particles in the "y"-direction.
      * @param default_height   default water height.
      */
      bool init(index_type columns, index_type rows, real_type const& default_height = 0)
      {
        if (columns < 1 || rows < 1)
          return false;
        m_pars.clear();
        m_pars.resize((m_I=columns)*(m_J=rows));
        particle_iterator end = m_pars.end();
        for (particle_iterator p = m_pars.begin(); p != end; ++p)
          p->m_Hold = p->m_Hcur = p->m_Hnew = default_height;
        return true;
      }

      /**
      * Execute one physical simulation step.
      *
      * @param dt   delta time == time step.
      */
      void simulate(real_type const& dt)
      {
        const real_type damping = 1-m_k*dt;
        const real_type advection = (dt*dt*m_c*m_c)/(m_D*m_D);
        for (int j = 0; j < int(m_J); ++j) for (int i = 0; i < int(m_I); ++i) {
          dwe_particle& par = particle(i,j);
          if (par.m_fixed) {
            par.m_Hcur = par.m_Hnew;  // just reset particle height if it has been modified explicitly.
            continue;
          }
          const real_type fdm = m_smooth
            ? -6*par.m_Hcur+get(i+1,j).m_Hcur+get(i-1,j).m_Hcur+get(i,j+1).m_Hcur+get(i,j-1).m_Hcur
            +0.5*(get(i+1,j+1).m_Hcur+get(i+1,j-1).m_Hcur+get(i-1,j+1).m_Hcur+get(i-1,j-1).m_Hcur)
            : get(i+1,j).m_Hcur+get(i-1,j).m_Hcur+get(i,j+1).m_Hcur+get(i,j-1).m_Hcur-4*par.m_Hcur;
          par.m_Hnew = par.m_Hcur + damping*(par.m_Hcur-par.m_Hold) + advection*fdm;
        }
        particle_iterator end = m_pars.end();
        for (particle_iterator p = m_pars.begin(); p != end; ++p) {
          dwe_particle& par = *p;
          par.m_Hold = par.m_Hcur;
          par.m_Hcur = par.m_Hnew;
        }
      }

    public:
      real_type& damping() {return m_k;}
      real_type const& damping() const {return m_k;}
      real_type& speed() {return m_c;}
      real_type const& speed() const {return m_c;}
      real_type& spacing() {return m_D;}
      real_type const& spacing() const {return m_D;}
      bool& smooth() {return m_smooth;}
      bool const& smooth() const {return m_smooth;}

      index_type const& columns() const {return m_I;}
      index_type const& rows() const {return m_J;}
      index_type size() const {return m_pars.size();}
      dwe_particle const& particle(index_type n) const
      {
        assert(n < m_pars.size());
        return m_pars[n];
      }
      dwe_particle& particle(index_type n)
      {
        assert(n < m_pars.size());
        return m_pars[n];
      }
      dwe_particle const& particle(index_type i, index_type j) const
      {
        assert(i < m_I && j < m_J);
        return m_pars[j*m_I+i];
      }
      dwe_particle& particle(index_type i, index_type j)
      {
        assert(i < m_I && j < m_J);
        return m_pars[j*m_I+i];
      }

      vector3_type normal(index_type i, index_type j) const
      {
        assert(i < m_I && j < m_J);
        const index_type i0 = m_I==i+1?i:i+1;
        const index_type i1 = 0==i?i:i-1;
        const index_type j0 = m_J==j+1?j:j+1;
        const index_type j1 = 0==j?j:j-1;
        const real_type nx = -m_D*(particle(i0, j).m_Hcur - particle(i1, j).m_Hcur);
        const real_type ny = 1;
        const real_type nz = -m_D*(particle(i, j0).m_Hcur - particle(i, j1).m_Hcur);
        //      const vector3_type v0 = vector3_type(i0*scale*m_D, particle(i0, j).m_Hcur, j*scale*m_D)-vector3_type(i1*scale*m_D, particle(i1, j).m_Hcur, j*scale*m_D);
        //      const vector3_type v1 = vector3_type(i*scale*m_D, particle(i, j0).m_Hcur, j0*scale*m_D)-vector3_type(i*scale*m_D, particle(i, j1).m_Hcur, j1*scale*m_D);
        //      return unit(v0%v1);
        return unit(vector3_type(nx, ny, nz));
      }

    protected:
      dwe_particle const& get(int i, int j) const
      {
        using std::max;
        using std::min;
        i = max(0, min(int(m_I)-1, i));
        j = max(0, min(int(m_J)-1, j));
        return m_pars[j*m_I+i];
      }

    protected:
      index_type  m_I;
      index_type  m_J;
      real_type  m_k;  ///< Damping koefficient.
      real_type  m_c;  ///< Wave speed.
      real_type  m_D;  ///< Delta step size (spacing) between grid vertices => Dx = Dy.
      bool       m_smooth;  ///< Smooth dwe/fdm solver flag.
      particle_container  m_pars;

    };

  } // namespace swe
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SWE_SWE_DAMPED_WAVE_EQUATIONS_H
#endif
