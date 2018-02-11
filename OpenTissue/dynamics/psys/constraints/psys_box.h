#ifndef OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_BOX_H
#define OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_BOX_H
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

namespace OpenTissue
{
  namespace psys
  {

    template<typename types>
    class Box
      : public Constraint< types >
    {
    public:

      typedef typename types::math_types                 math_types;
      typedef typename math_types::vector3_type          vector3_type;
      typedef typename types::particle_type              particle_type;
      typedef typename types::system_type                system_type;
      typedef typename types::aabb_type                  aabb_type;

    protected:

      aabb_type m_aabb;   ///< An AABB box defined a region of space where particles are restricted to be inside.

    public:

      aabb_type       & aabb()       { return m_aabb; }
      aabb_type const & aabb() const { return m_aabb; }

    public:

      Box()
        : m_aabb(0,0,0,1000,1000,1000)
      {}

      virtual ~Box(){}

    public:

      void satisfy()
      {
        typedef typename system_type::particle_iterator   particle_iterator;
        using std::min;
        using std::max;

        vector3_type r;
        vector3_type m = m_aabb.min();
        vector3_type M = m_aabb.max();

        particle_iterator p   = this->owner()->particle_begin();
        particle_iterator end = this->owner()->particle_end();
        for(;p!=end;++p)
          p->position() =  min(  M,  max(m, p->position()) );
      }

    };
  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_BOX_H
#endif
