#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_KERNEL_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_KERNEL_H
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
  namespace sph
  {

    /**
    * Smoothing Kernel Base Class.
    * This class uses a varying support radius
    */
    template< typename Types >
    class SmoothingKernel
    {
    public:
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;

    public:
      /**
      * Default Constructor.
      */
      SmoothingKernel()
      {
      }

      /**
      * Deconstructor.
      */
      virtual ~SmoothingKernel()
      {
      }

    public:

      /**
      * Evaluate.
      *
      * @param r   The difference vector (or relative position) r.
      * @param h   The finite support radius.
      * @return   The evaluation of the Kernel at r.
      */
      virtual real_type evaluate(const vector& r, const real_type& h) const = 0;

      /**
      * Gradient.
      *
      * @param r   The difference vector (or relative position) r.
      * @param h   The finite support radius.
      * @return   The gradient of the Kernel at r.
      */
      virtual vector gradient(const vector& r, const real_type& h) const = 0;

      /**
      * Laplacian.
      *
      * @param r   The difference vector (or relative position) r.
      * @param h   The finite support radius.
      * @return   The laplacian of the Kernel at r.
      */
      virtual real_type laplacian(const vector& r, const real_type& h) const = 0;

    protected:
      /**
      * Check Range.
      *
      * @param r   The difference vector undergoing the check.
      * @param h   The finite support radius.
      * @return   True if 0 <= |r| <= h, else false.
      */
      bool checkRange(const vector& r, const real_type& h) const
      {
        return h*h >= r*r;
      }

    };  // End class SmoothingKernel


    /**
    * Fixed Smoothing Kernel Base Class.
    * This class uses a fixed support radius
    */
    template< typename Types, bool CheckRange >
    class FixedSmoothingKernel
    {
    public:
      typedef SmoothingKernel<Types>  Base;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;

    public:
      /**
      * Default Constructor.
      */
      FixedSmoothingKernel(const real_type& radius) : m_radius(radius), m_radiusSqr(radius*radius)
      {
      }

      /**
      * Deconstructor.
      */
      virtual ~FixedSmoothingKernel()
      {
      }

    public:

      /**
      * Radius (read only).
      *
      * @return   The current core radius.
      */
      const real_type& radius() const
      {
        return m_radius;
      }

      /**
      * Evaluate.
      *
      * @param r   The difference vector (or relative position) r.
      * @return   The evaluation of the Kernel at r.
      */
      virtual real_type evaluate(const vector& r) const = 0;

      /**
      * Gradient.
      *
      * @param r   The difference vector (or relative position) r.
      * @return   The gradient of the Kernel at r.
      */
      virtual vector gradient(const vector& r) const = 0;

      /**
      * Laplacian.
      *
      * @param r   The difference vector (or relative position) r.
      * @return   The laplacian of the Kernel at r.
      */
      virtual real_type laplacian(const vector& r) const = 0;

      //  protected:
    public:
      /**
      * Check Range.
      *
      * @param r   The difference vector undergoing the check.
      * @return   True if 0 <= |r| <= radius, else false.
      */
      bool checkRange(const vector& r) const
      {
        if (!CheckRange) return true;  //--- ToDo KE 2006-01-21: DAmn it, use partial specialization for such things:-)
        return m_radiusSqr >= r*r;
      }

    protected:
      real_type  m_radius;      ///< Core smoothing kernel radius (finite support)
      real_type  m_radiusSqr;   ///< Radius squared (used in quick range check)

    };  // End class SmoothingKernel


  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_KERNEL_H
#endif
