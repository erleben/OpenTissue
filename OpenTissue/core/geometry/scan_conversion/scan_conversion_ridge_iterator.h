#ifndef OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_RIDGE_ITERATOR_H
#define OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_RIDGE_ITERATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> // needed for boost::numeric_cast

#include <cassert>

namespace OpenTissue 
{
  namespace scan_conversion
  {
    namespace detail
    {
      template<typename vector3_type>
      class RigdeIterator 
      {
      protected:

        // Is the iterator data valid
        bool m_valid;

        // An RigdeIterator might have more than one edge. We have hardwired this implementation to have atmost two edges.
        bool      m_has_multiple_edges;
        vector3_type m_next_vertex;
        vector3_type m_next_normal;

        // Local variables concerned with the vertices
        vector3_type m_start_vertex;
        vector3_type m_end_vertex;

        int m_start_x;
        int m_start_y;

        int m_end_x;
        int m_end_y;

        int  m_current_x;
        int  m_current_y;

        int  m_dx;
        int  m_step_x;

        int  m_dy;
        int  m_step_y;

        int  m_denominator;
        int  m_accumulator;

        // Local variables concerned with the normals
        vector3_type m_start_normal;
        vector3_type m_end_normal;
        vector3_type m_current_normal;

      public:

        bool          valid() const { return this->m_valid;          }
        int               x() const { return this->m_current_x;      }
        int               y() const { return this->m_current_y;      }
        vector3_type normal() const { return this->m_current_normal; }

      public:

        RigdeIterator() 
          : m_valid(false)
        {}

        virtual ~RigdeIterator(){}

      public:

        RigdeIterator(
          vector3_type const & v1
          , vector3_type const & n1
          , vector3_type const & v2
          , vector3_type const & n2
          )
          : m_has_multiple_edges(false)
        {
          this->initialize(v1,n1,v2,n2);
        }

        RigdeIterator(
          vector3_type const & v1
          , vector3_type const & n1
          , vector3_type const & v2
          , vector3_type const & n2
          , vector3_type const & v3
          , vector3_type const & n3
          )
          : m_has_multiple_edges(true)
          , m_next_vertex(v3)
          , m_next_normal(n3)
        {
          this->initialize(v1,n1,v2,n2);
        }

        RigdeIterator(RigdeIterator const& iter) 
        {  
          (*this) = iter; 
        }

        RigdeIterator const & operator=(RigdeIterator const & iter)
        {
          this->m_valid              = iter.m_valid;
          this->m_has_multiple_edges = iter.m_has_multiple_edges;
          this->m_next_vertex        = iter.m_next_vertex;
          this->m_next_normal        = iter.m_next_normal;
          this->m_start_vertex       = iter.m_start_vertex;
          this->m_end_vertex         = iter.m_end_vertex;
          this->m_start_x            = iter.m_start_x;
          this->m_start_y            = iter.m_start_y;
          this->m_end_x              = iter.m_end_x;
          this->m_end_y              = iter.m_end_y;
          this->m_current_x          = iter.m_current_x;
          this->m_current_y          = iter.m_current_y;
          this->m_dx                 = iter.m_dx;
          this->m_step_x             = iter.m_step_x;
          this->m_dy                 = iter.m_dy;
          this->m_step_y             = iter.m_step_y;
          this->m_denominator        = iter.m_denominator;
          this->m_accumulator        = iter.m_accumulator;
          this->m_start_normal       = iter.m_start_normal;
          this->m_end_normal         = iter.m_end_normal;
          this->m_current_normal     = iter.m_current_normal;
          return *this;
        }

      public:

        bool operator()() const {  return this->m_valid; }

        bool operator++()
        {
          this->m_valid = this->update_vertex();
          if (this->m_valid) 
          {
            this->m_valid = this->update_normal();
          }
          return this->m_valid;
        }

      protected:

        void initialize(
          vector3_type const & v1
          , vector3_type const & n1
          , vector3_type const & v2
          , vector3_type const & n2        
          )
        {
          this->m_valid = true;

          vector3_type V1( round( v1 ) );
          vector3_type V2( round( v2 ) );

          // We scan-convert from low y-values to high y-values
          // Interchange the end points if necessary.
          // That secures that Dy is always positive.
          if (V1(1) <= V2(1)) 
          {
            this->m_start_vertex   = V1;
            this->m_current_normal = n1;
            this->m_start_normal   = n1;
            this->m_end_vertex     = V2;
            this->m_end_normal     = n2;
          }
          if (V1(1) > V2(1)) 
          {
            this->m_start_vertex   = V2;
            this->m_current_normal = n2;
            this->m_start_normal   = n2;
            this->m_end_vertex     = V1;
            this->m_end_normal     = n1;
          }
          if (V1(1) == V2(1)) 
          {
            // Horizontal edge_type, throw it away, because we draw triangles/polygons - not edges.
            // Horizontal Edges in triangles/polygons are thrown away, because the left and right 
            // edges will handle Xstart and Xstop. See Foley.
            this->m_valid = false;
          }

          this->m_start_x = boost::numeric_cast<int>( this->m_start_vertex(0) );
          this->m_start_y = boost::numeric_cast<int>( this->m_start_vertex(1) );
          this->m_end_x   = boost::numeric_cast<int>( this->m_end_vertex(0)  );
          this->m_end_y   = boost::numeric_cast<int>( this->m_end_vertex(1)  );

          // Kenny: I optimized this!!!
          //this->m_dx = this->m_end_x - this->m_start_x;
          //if (0 <= this->m_dx)
          //  // If Dx is non-negative we scan-convert from left to right
          //  this->m_step_x = 1;
          //else {
          //  // If Dx is negative we scan-convert from right to left
          //  this->m_dx = -this->m_dx;   // Use the absolute value
          //  this->m_step_x = -1;       // And let the Xstep be negative
          //}
          this->m_dx     = (this->m_end_x >= this->m_start_x) ? (this->m_end_x - this->m_start_x) : (this->m_start_x - this->m_end_x);
          this->m_step_x = (this->m_end_x >= this->m_start_x) ? 1 :-1;

          this->m_dy     = this->m_end_y - this->m_start_y;
          this->m_step_y = 1;

          // Initialize the Accumulator (the numerator)
          // If it is an edge with a positive slope, let the the Accumulator be equal to Dx
          // that will make a right shift the first time, which must be done.
          // Else, initialize the Accumulator to +1, because if a and d are integers
          // then a >= d ==> a + 1 > d, and that is what is needed if we only have 
          // the the > operator
          this->m_accumulator = (0 <= this->m_step_x) ? this->m_dy : 1;
          this->m_denominator = this->m_dy;

          this->m_current_x = this->m_start_x;
          this->m_current_y = this->m_start_y;

          this->m_valid = (this->m_start_y == this->m_end_y) ? false : true;
        }

        bool update_vertex()
        {
          if (this->m_current_y >= this->m_end_y) 
          {
            this->m_valid = false;
          }
          else 
          {
            this->m_accumulator += this->m_dx;
            while (this->m_accumulator > this->m_denominator) 
            {
              this->m_current_x += this->m_step_x;
              this->m_accumulator -= this->m_denominator;
            }
            this->m_current_y += this->m_step_y;
          }
          if (this->m_current_y >= this->m_end_y) 
          {
            if (!this->m_has_multiple_edges)
              this->m_valid = false;
            else 
            {
              this->initialize(
                this->m_end_vertex
                , this->m_end_normal
                , this->m_next_vertex
                , this->m_next_normal          
                );
              this->m_has_multiple_edges = false;
            }
          }
          return this->m_valid;
        }

        bool update_normal()
        {
          // Ncurrent must be normalized, to do the interpolation in integers (rational numbers)
          int numerator   = this->m_current_y - this->m_start_y;
          int denominator = this->m_end_y    - this->m_start_y;
          this->m_current_normal = this->m_start_normal * (denominator - numerator) + this->m_end_normal * numerator;
          this->m_current_normal = unit(this->m_current_normal);
          return this->m_valid;
        }

      };

    } // end of namespace detail
  } // end of namespace scan_conversion
} // end of namespace OpenTissue
// OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_RIDGE_ITERATOR_H
#endif
