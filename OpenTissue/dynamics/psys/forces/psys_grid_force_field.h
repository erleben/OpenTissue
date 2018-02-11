#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRID_FORCE_FIELD_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRID_FORCE_FIELD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/grid/util/grid_value_at_point.h>

#include <cassert>

namespace OpenTissue
{
  namespace psys
  {

    template<typename types>
    class GridForceField 
      : public types::force_type
    {
    public:

      typedef typename types::math_types          math_types;
      typedef typename math_types::real_type      real_type;
      typedef typename math_types::vector3_type   vector3_type;
      typedef typename types::system_type         system_type;   
      typedef OpenTissue::grid::Grid<vector3_type,math_types>        grid_type;

    protected:

      grid_type * m_field;   ///< The force field

    public:

      grid_type       & field()       {  return *m_field; }
      grid_type const & field() const {  return *m_field; }

    public:

      GridForceField()
        : m_field(0)
      {}

      virtual ~GridForceField(){}

    public:

      void apply()
      {
        typedef typename system_type::particle_iterator   particle_iterator;

        if(!m_field)
          return;

        particle_iterator p   = this->owner()->particle_begin();
        particle_iterator end = this->owner()->particle_end();
        for(;p!=end;++p)
        {
          if( p->inv_mass() <= 0 )
            continue;
          p->force() += OpenTissue::grid::value_at_point(*m_field, p->position() );
        }        
      }

    public:

      void init( grid_type const & field )
      {
        if(field.empty())
        {
          std::cerr << " GridForceField::init(...): field was empty, did you forget to initialize it?" << std::endl;
          m_field = 0;
          return;
        }
        m_field = const_cast<grid_type *>( &field );
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRID_FORCE_FIELD_H
#endif
