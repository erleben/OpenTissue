#ifndef OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_STICK_H
#define OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_STICK_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/psys_constraint.h>

#include <cassert>
#include <cmath>

namespace OpenTissue
{
  namespace psys
  {

    template<typename types>
    class Stick
      : public Constraint< types >
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;

    protected:

      particle_type * m_A;           ///< Pointer to one of the affected particles.
      particle_type * m_B;           ///< Pointer to one other affected particle.
      real_type       m_length;      ///< Rest length between the two particles.
      real_type       m_length_sqr;  ///< Rest length squared.
      unsigned int    m_choice;       ///< Member indicating which satisfy type that is used (1,2,3,...?) default 2


    public:

      particle_type       * A()       { return m_A; }
      particle_type       * B()       { return m_B; }
      particle_type const * A() const { return m_A; }
      particle_type const * B() const { return m_B; }

      real_type const    & rest_length() const { return m_length; }
      unsigned int       & choice()            { return m_choice; }
      unsigned int const & choice()      const { return m_choice; }

    public:

      Stick()
        : m_A(0)
        , m_B(0)
        , m_length(0)
        , m_length_sqr(0)
        , m_choice(2)
      { }

      virtual ~Stick()  {  }

    public:

      /**
      * Init Stick.
      *
      * @param A
      * @param B
      */
      void init(particle_type * A, particle_type * B)
      {
        assert(A!=B || !"Stick::set(): Particle A and B were the same");
        assert(A    || !"Stick::set(): Particle A was null");
        assert(B    || !"Stick::set(): Particle B was null");

        this->m_A = A;
        this->m_B = B;

        set_rest_length( length(B->position()-A->position()));
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

    public:

      void satisfy()
      {
        switch(m_choice)
        {
        case 1:      satisfy_type1();      break;
        case 2:      satisfy_type2();      break;
        case 3:      satisfy_type3();      break;
        };
      }

    public:

      /**
      * Satisfy Constraint.
      * Normal way of satisfying a stick constraint
      */
      void satisfy_type1()
      {
        vector3_type delta = m_A->position() - m_B->position();
        real_type delta_sqr = delta*delta;
        real_type delta_length = sqrt(delta_sqr);
        real_type diff = (delta_length - m_length)/delta_length;
        m_A->position() -= delta*0.5*diff;
        m_B->position() += delta*0.5*diff;
      }

      /**
      * Satisfy Constraint.
      * Using square root approximation for satisfying stick constraint
      */
      void satisfy_type2()
      {
        vector3_type delta = m_A->position() - m_B->position();
        real_type delta_sqr = delta * delta;
        real_type approx = m_length_sqr / (delta_sqr + m_length_sqr) - 0.5;
        delta *= approx;
        m_A->position() += delta;
        m_B->position() -= delta;
      }

      /**
      * Satisfy Constraint.
      * Like normal way for satisfying stick constraint, but respecting masses.
      */
      void satisfy_type3()
      {
        vector3_type delta = m_A->position() - m_B->position();
        real_type delta_length = std::sqrt(delta*delta);
        real_type diff = (delta_length - m_length)/(delta_length*(m_A->inv_mass()+m_B->inv_mass()));
        m_A->position() -=  delta * (diff *  m_A->inv_mass());
        m_B->position() +=  delta * (diff * m_B->inv_mass());
      }

    public:

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_STICK_H
#endif
