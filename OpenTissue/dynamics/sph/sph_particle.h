#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_PARTICLE_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_PARTICLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_runtime_type.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * Particle Template Class.
    * to use for Smoothed Particle Hydrodynamics [SPH]
    * in the field of Computational Fluid Dynamics [CFD]
    * - så fik vi det på plads
    */
    template<
      typename real_
      , template<typename> class Vector
      , struct OpenTissue::utility::RuntimeType<real_> * Radius
    >
    class Particle
    {
    public:
      typedef real_  real_type;
      typedef Vector<real_type>  vector;
      typedef vector  point_type;

    public:
      Particle()
        : m_radius(*Radius)
        , m_radiusSqr(*Radius**Radius)
        , m_m(1)
        , m_r(1)
        , m_p(0)
        , m_fixed(false)
      {}

      virtual ~Particle()
      {}

    public:

      /**
      * minimum point used for hashing.
      */
      point_type min() const
      {
        return point_type(m_x - point_type(m_radius));
        //      return m_x-m_radius;
      }

      /**
      * maximum point used for hashing.
      */
      point_type max() const
      {
        return point_type(m_x+point_type(m_radius));
        //      return m_x+m_radius;
      }

      /**
      * checks if two particles (actually two positions) are within an allowed radius.
      */
      bool check(const point_type& candidate) const
      {
        return m_radiusSqr >= sqr_length(point_type(m_x-candidate));
        //      point_type diff(m_x-candidate);
        //      return m_radiusSqr >= diff*diff;
        //      return m_radiusSqr >= (m_x[0]-candidate[0])*(m_x[0]-candidate[0])+(m_x[1]-candidate[1])*(m_x[1]-candidate[1])+(m_x[2]-candidate[2])*(m_x[2]-candidate[2]);
      }

      /**
      * Position (read only).
      *
      * @return  const reference to the current position vector.
      */
      const point_type& position() const
      {
        return m_x;
      }

      /**
      * Position.
      *
      * @return  reference to the current position vector.
      */
      point_type& position()
      {
        return m_x;
      }

      /**
      * Position (read only).
      *
      * @return  const reference to the old position vector.
      */
      const point_type& position_old() const
      {
        return m_o;
      }

      /**
      * Position.
      *
      * @return  reference to the old position vector.
      */
      point_type& position_old()
      {
        return m_o;
      }

      /**
      * Velocity (read only).
      *
      * @return  const reference to the velocity vector.
      */
      const vector &velocity() const
      {
        return m_v;
      }

      /**
      * Velocity.
      *
      * @return  reference to the velocity vector.
      */
      vector &velocity()
      {
        return m_v;
      }

      /**
      * Acceleration (read only).
      *
      * @return  const reference to the acceleration vector.
      */
      const vector &acceleration() const
      {
        return m_a;
      }

      /**
      * Acceleration.
      *
      * @return  reference to the acceleration vector.
      */
      vector &acceleration()
      {
        return m_a;
      }

      /**
      * Force (read only).
      *
      * @return  const reference to the external force vector.
      */
      const vector &force() const
      {
        return m_f;
      }

      /**
      * Force.
      *
      * @return  reference to the external force vector.
      */
      vector &force()
      {
        return m_f;
      }

      /**
      * Mass (read only).
      *
      * @return  const reference to the mass scalar.
      */
      const real_type &mass() const
      {
        return m_m;
      }

      /**
      * Mass.
      *
      * @return  reference to the mass scalar.
      */
      real_type &mass()
      {
        return m_m;
      }

      /**
      * Density (read only).
      *
      * @return  const reference to the density value.
      */
      const real_type &density() const
      {
        return m_r;
      }

      /**
      * Density.
      *
      * @return  reference to the density value.
      */
      real_type &density()
      {
        return m_r;
      }

      /**
      * Pressure (read only).
      *
      * @return  const reference to the pressure value.
      */
      const real_type &pressure() const
      {
        return m_p;
      }

      /**
      * Pressure.
      *
      * @return  reference to the pressure value.
      */
      real_type &pressure()
      {
        return m_p;
      }

      /**
      * Normal (read only).
      *
      * @return  const reference to the surface normal.
      */
      const vector &normal() const
      {
        return m_n;
      }

      /**
      * Normal.
      *
      * @return  reference to the surface normal.
      */
      vector &normal()
      {
        return m_n;
      }

      /**
      * Fixed (read only).
      *
      * @return  const reference to the surface normal.
      */
      const bool &fixed() const
      {
        return m_fixed;
      }

      /**
      * Fixed.
      *
      * @return  reference to the surface normal.
      */
      bool &fixed()
      {
        return m_fixed;
      }

      /**
      * Fixed.
      *
      * @return  reference to the surface normal.
      */
      const real_type &radius() const
      {
        return m_radius;
      }

    protected:
      //point_type  m_radius;  ///< Radius point
      real_type  m_radius;  ///< Radius point
      real_type  m_radiusSqr;  ///< Radius point
      point_type  m_x;   ///< Current position
      point_type  m_o;   ///< Old position
      vector  m_v;   ///< Current velocity
      vector  m_a;   ///< Current acceleration
      vector  m_f;   ///< Current sum of external forces acting on the particle
      vector  m_n;   ///< Surface normal (if close to/on the free surface, else zero)
      real_type  m_m;   ///< Mass (constant)
      real_type  m_r;   ///< Density (vary)
      real_type  m_p;   ///< Pressure (vary)
      bool  m_fixed;   ///< Is this particle fixe (locked)?

    }; // End class Particle

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_PARTICLE_H
#endif
