#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_DIRECTION_LUT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_DIRECTION_LUT_H
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

  namespace geometry
  {

    template<typename lut_container>
    void create_lut6(lut_container & lut)
    {
      typedef typename lut_container::value_type  vector3_type;
      typedef typename vector3_type::value_traits value_traits;

      lut.clear();
      lut.push_back( vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(),-value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(),-value_traits::one()) );
    }

    template<typename lut_container>
    void create_lut14(lut_container & lut)
    {
      typedef typename lut_container::value_type  vector3_type;
      typedef typename vector3_type::value_traits value_traits;

      lut.clear();
      lut.push_back( vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(),-value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(),-value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(),-value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(),-value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(),-value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::one(), value_traits::one()) );
    }

    template<typename lut_container>
    void create_lut26(lut_container & lut)
    {
      typedef typename lut_container::value_type  vector3_type;
      typedef typename vector3_type::value_traits value_traits;

      lut.clear();
      lut.push_back( vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::zero(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(),-value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::zero(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(),-value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(),-value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(),-value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(),-value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(),-value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::zero(),-value_traits::one(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::one(),-value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type( value_traits::one(), value_traits::zero(),-value_traits::one()) );
      lut.push_back( vector3_type( value_traits::zero(),-value_traits::one(), value_traits::one()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::one(), value_traits::zero()) );
      lut.push_back( vector3_type(-value_traits::one(), value_traits::zero(), value_traits::one()) );
      lut.push_back( vector3_type( value_traits::zero(), value_traits::one(),-value_traits::one()) );
    }


    /**
    * Get Direction Bit Patteren for Vector v.
    *
    * @param begin    Iterator to first direction vector.
    * @param end      Iterator to last direction vector.
    * @param v        The vector to test with
    * @return         A bitmask pattern. If i'th bit is set, then v points
    *                 in the same direction as the i'th direction vector
    *                 in the lookup table.
    */
    template<typename lut_iterator, typename vector3_type>
    unsigned int direction_bitmask( lut_iterator begin, lut_iterator end, vector3_type const & v)
    {
      typedef typename vector3_type::value_traits value_traits;

      unsigned int mask = 0;
      for(lut_iterator lut = begin; lut != end;++lut)
      {
        vector3_type const & dir  = *(lut);  //--- extract direction vector from lookup table.
        mask = mask << 1;
        if((v * dir) > value_traits::zero() )
        {
          //--- valid dir, set bit i'th to high
          mask = (mask | 0x001);
        }         
      }
      return mask;
    }

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_DIRECTION_LUT_H
#endif
