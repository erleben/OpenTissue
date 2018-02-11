#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_SPRING_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_SPRING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace psys
  {

    template<typename types>
    class Spring
      : public types::force_type
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;
      typedef typename types::system_type           system_type;

    protected:

      particle_type * m_A;           ///< Pointer to one of the affected particles.
      particle_type * m_B;           ///< Pointer to one other affected particle.
      real_type       m_length;      ///< Rest length between the two particles.
      real_type       m_length_sqr;  ///< Rest length squared.
      real_type       m_k;           ///< Spring Constant.
      real_type       m_c;           ///< Damping Constant.


    public:

      particle_type       * A()       { return m_A; }
      particle_type       * B()       { return m_B; }
      particle_type const * A() const { return m_A; }
      particle_type const * B() const { return m_B; }

      real_type       & damping()           { return m_c; }
      real_type const & damping()     const { return m_c; }
      real_type       & stiffness()         { return m_k; }
      real_type const & stiffness()   const { return m_k; }
      real_type const & rest_length() const { return m_length; }

    public:

      Spring()
        : m_A(0)
        , m_B(0)
        , m_length(0)
        , m_length_sqr(0)
        , m_k(0)
        , m_c(0)
      {
        set_critical_damped(0.01);
      }

      virtual ~Spring()  {  }

    public:

      /**
      * Init Spring.
      *
      * @param A
      * @param B
      */
      void init(particle_type * A, particle_type * B)
      {
        assert(A!=B || !"Spring::set(): Particle A and B were the same");
        assert(A    || !"Spring::set(): Particle A was null");
        assert(B    || !"Spring::set(): Particle B was null");

        this->m_A = A;
        this->m_B = B;

        set_rest_length( length(B->position()-A->position()));
        set_critical_damped(0.01);
      }

      /**
      * Set Rest Length.
      *
      * @param length
      */
      void set_rest_length(real_type l)
      {
        assert(l>=0 || !"Spring::set_length(): Spring rest length was negative");
        this->m_length = l;
        this->m_length_sqr = l*l;
      }

      /**
      *
      * @param tau
      */
      void set_critical_damped( real_type const & tau)
      {
        if(tau>0)
        {
          m_c = 2./tau;
          m_k = 1./(tau*tau);
        }
      }

    public:

      void apply()
      {
        using std::fabs;

        //--- F_A = - (k (l-l0)  + c (v_a-v_b).((r_A-r_B)/ |(r_A-r_B)|)  )        (r_A-r_B) / |(r_A-r_B)|
        //--- F_B = - F_A
        vector3_type dr = m_A->position() - m_B->position();
        vector3_type dv = m_A->velocity() - m_B->velocity();
        real_type l = length(dr);
        real_type l0 = rest_length();
        if(fabs(l)>0)
        {
          dr = dr/l;
          vector3_type F_a = -(m_k*(l-l0) + m_c*(dv * dr)) * dr;
          m_A->force() += F_a;
          m_B->force() -= F_a;
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_SPRING_H
#endif
