#ifndef OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_FRAGMENT_ITERATOR_H
#define OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_FRAGMENT_ITERATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/scan_conversion/scan_conversion_ridge_iterator.h>

namespace OpenTissue
{
  namespace scan_conversion
  {

    /**
    * Fragment Iterator.
    * Typical usage is as follows:
    *
    *  typedef FragmentIterator<vector3_type> fragment_iterator;
    *
    *  vector3_type v1;
    *  vector3_type v2;
    *  vector3_type v3;
    *  vector3_type n1;
    *  vector3_type n2;
    *  vector3_type n3;
    *
    *  pragment_iterator pixel ( v1, n1 , v2 , n2, v3 , n4);
    *
    *  while( pixel() )
    *  {
    *     std::cout << "sreen x = " 
    *               << pixel.x()
    *               << "sreen y = " 
    *               << pixel.y()
    *               << "normal = " 
    *               << pixel.normal()
    *               << std::endl;
    *     ++pixel;
    *  }
    *  
    */
    template<typename vector3_type>
    class FragmentIterator 
    {
    protected:

      typedef typename OpenTissue::scan_conversion::detail::RigdeIterator< vector3_type >  ridge_iterator;

      vector3_type m_vertex[3];     ///< The 3D Transformed vertices
      vector3_type m_normal[3];     ///< The original 3D normals

      int m_lower_left_idx;
      int m_upper_left_idx;
      int m_the_other_idx;

      ridge_iterator m_left_ridge_iterator;
      ridge_iterator m_right_ridge_iterator;

      // Types and variables for scan-convertion
      typedef enum {
        find_nonempty_scanline_state
        , in_scanline_state
        , scanline_found_state
        , finished_state
      } state_type;

      state_type m_state;

      int m_start_x;     // Screen coordinates
      int m_start_y;
      int m_end_x;
      int m_end_y;
      int m_current_x;
      int m_current_y;

    protected:

      /**
      * returns the index of the vertex with the smallest y-coordinate
      * If there is a horizontal edge, the vertex with the smallest 
      * x-coordinate is chosen.
      */
      int find_lower_left_vertex_index()
      {
        int ll = 0;
        for (int i = ll + 1; i < 3; ++i) 
        {
          if (
            (this->m_vertex[i][1] < this->m_vertex[ll][1]) 
            ||
            (
            (this->m_vertex[i][1] == this->m_vertex[ll][1]) 
            &&
            (this->m_vertex[i][0] < this->m_vertex[ll][0])
            )
            )
          {
            ll = i;
          }
        }
        return ll;
      }

      /**
      * returns the index of the vertex with the greatest y-coordinate
      * If there is a horizontal edge, the vertex with the smallest 
      * x-coordinate is chosen.
      */
      int find_upper_left_vertex_index()
      {
        int ul = 0;
        for (int i = ul + 1; i < 3; ++i) 
        {
          if (
            (this->m_vertex[i][1] > this->m_vertex[ul][1]) 
            ||
            (
            (this->m_vertex[i][1] == this->m_vertex[ul][1]) 
            && 
            (this->m_vertex[i][0] < this->m_vertex[ul][0])
            )
            )
          {
            ul = i;
          }
        }
        return ul;
      }

      ridge_iterator get_left_ridge_iterator() const
      {
        typedef typename vector3_type::value_type real_type;

        // Let u be the vector from 'lowerleft' to 'upperleft', and let v be the vector
        // from 'lowerleft' to 'theother'. If the cross product u x v is positive then
        // the point 'theother' is to the left of u.
        vector3_type u = this->m_vertex[this->m_upper_left_idx] - this->m_vertex[this->m_lower_left_idx];
        vector3_type v = this->m_vertex[this->m_the_other_idx]  - this->m_vertex[this->m_lower_left_idx];

        real_type cross_product = u(0) * v(1) - u(1) * v(0);

        if (cross_product >= 0) 
        {
          // Here there is no need to check for a horizontal edge, because it
          // would be a top edge, and therefore it would not be drawn anyway.
          // The point 'theother' is to the left of the vector u
          return ridge_iterator(
            this->m_vertex[this->m_lower_left_idx]
          , this->m_normal[this->m_lower_left_idx]
          , this->m_vertex[this->m_the_other_idx]
          , this->m_normal[this->m_the_other_idx]
          , this->m_vertex[this->m_upper_left_idx]
          , this->m_normal[this->m_upper_left_idx]
          );
        }
        else 
        {
          // The point 'theother' is to the right of the vector u
          return ridge_iterator(
            this->m_vertex[this->m_lower_left_idx]
          , this->m_normal[this->m_lower_left_idx]
          , this->m_vertex[this->m_upper_left_idx]
          , this->m_normal[this->m_upper_left_idx]
          );
        }
      }

      ridge_iterator get_right_ridge_iterator() const
      {
        typedef typename vector3_type::value_type  real_type;

        // Let u be the vector from 'lowerleft' to 'upperleft', and let v be the vector
        // from 'lowerleft' to 'theother'. If the cross product u x v is negative then
        // the point 'theother' is to the right of u.

        vector3_type u = this->m_vertex[this->m_upper_left_idx] - this->m_vertex[this->m_lower_left_idx];
        vector3_type v = this->m_vertex[this->m_the_other_idx]  - this->m_vertex[this->m_lower_left_idx];

        real_type cross_product = u(0) * v(1) - u(1) * v(0);

        if (cross_product < 0) 
        {
          // The point 'theother' is to the right of the vector u
          // Check if there is a horizontal edge
          if (this->m_vertex[this->m_lower_left_idx](1) != this->m_vertex[this->m_the_other_idx](1) ) 
          {
            return ridge_iterator(
              this->m_vertex[this->m_lower_left_idx]
            , this->m_normal[this->m_lower_left_idx]
            , this->m_vertex[this->m_the_other_idx]
            , this->m_normal[this->m_the_other_idx]
            , this->m_vertex[this->m_upper_left_idx]
            , this->m_normal[this->m_upper_left_idx]
            );
          }
          else 
          {
            // Horizontal edge - ('lowerleft', 'theother') - throw it away
            return ridge_iterator(
              this->m_vertex[this->m_the_other_idx]
            , this->m_normal[this->m_the_other_idx]
            , this->m_vertex[this->m_upper_left_idx]
            , this->m_normal[this->m_upper_left_idx]
            );
          }
        }
        else 
        {
          // The point 'theother' is to the left of the u vector
          return ridge_iterator(
            this->m_vertex[this->m_lower_left_idx]
          , this->m_normal[this->m_lower_left_idx]
          , this->m_vertex[this->m_upper_left_idx]
          , this->m_normal[this->m_upper_left_idx]
          );
        }
      }

      bool find_nonempty_scanline()
      {
        while (m_left_ridge_iterator()) 
        {
          this->m_start_x   = this->m_left_ridge_iterator.x();
          this->m_end_x    = this->m_right_ridge_iterator.x();

          this->m_current_x = this->m_left_ridge_iterator.x();
          this->m_current_y = this->m_left_ridge_iterator.y();

          if (this->m_current_x < this->m_end_x) 
          {
            this->m_state = in_scanline_state;
            return this->valid();
          }
          else 
          {
            ++(this->m_left_ridge_iterator);
            ++(this->m_right_ridge_iterator);
            this->m_state = find_nonempty_scanline_state;
          }
        }
        // We ran our of left_ridge_iterator...
        return this->valid();
      }

    public:

      bool          valid() const  {    return this->m_left_ridge_iterator.valid();  }

      int               x() const  {    return this->m_current_x;  }

      int               y() const  {    return this->m_current_y;  }

      vector3_type normal() const
      {
        vector3_type left_normal = this->m_left_ridge_iterator.normal();
        vector3_type right_normal = this->m_right_ridge_iterator.normal();
        // current normal must be normalized, so to the interpolation in integers (rational numbers)
        int numerator   = this->m_current_x - this->m_start_x;
        int denominator = this->m_end_x     - this->m_start_x;
        return vector3_type( unit( left_normal * (denominator - numerator) + right_normal * numerator ) );
      }

    public:

      virtual ~FragmentIterator(){}

      /**
      * 
      * vertex coordinates are assumed to be transformed into screen
      * space of the camera. Normals can be given in whatever coordinate
      * frame one wants. Just remember that normals are lineary
      * interpolated in screen space.
      * 
      */
      FragmentIterator(
        vector3_type const & v1
        , vector3_type const & n1
        , vector3_type const & v2
        , vector3_type const & n2
        , vector3_type const & v3
        , vector3_type const & n3
        ) 
      {
        this->initialize(v1,n1,v2,n2,v3,n3);
      }

      /**
      * 
      * vertex coordinates are assumed to be transformed into screen
      * space of the camera. 
      * 
      */
      FragmentIterator(
        vector3_type const & v1
        , vector3_type const & v2
        , vector3_type const & v3
        ) 
      {
        vector3_type n;
        n.clear();
        this->initialize(v1,n,v2,n,v3,n);
      }

      FragmentIterator(FragmentIterator const & iter)  {    (*this) = iter;  }

      FragmentIterator const & operator=(FragmentIterator const & iter)
      {
        this->m_vertex[0] = iter.m_vertex[0];
        this->m_normal[0] = iter.m_normal[0];
        this->m_vertex[1] = iter.m_vertex[1];
        this->m_normal[1] = iter.m_normal[1];
        this->m_vertex[2] = iter.m_vertex[2];
        this->m_normal[2] = iter.m_normal[2];

        this->m_lower_left_idx = iter.m_lower_left_idx;
        this->m_upper_left_idx = iter.m_upper_left_idx;
        this->m_the_other_idx  = iter.m_the_other_idx;

        this->m_left_ridge_iterator  = iter.m_left_ridge_iterator;
        this->m_right_ridge_iterator = iter.m_right_ridge_iterator;

        this->m_state = iter.m_state;

        this->m_start_x   = iter.m_start_x;
        this->m_start_y   = iter.m_start_y;
        this->m_end_x    = iter.m_end_x;
        this->m_end_y    = iter.m_end_y;
        this->m_current_x = iter.m_current_x;
        this->m_current_y = iter.m_current_y;

        return *this;
      }

      /**
      * 
      *
      * @return   If there are any more fragments left then the
      *           return value is true otherwise it is false.
      */
      bool operator()() const  {    return this->m_left_ridge_iterator.valid();  }

      /**
      * Advances the iterator to the next fragment. Initially the
      * fragement iterator points to the first fragment (if any).
      * 
      */
      bool operator++()
      {
        if (this->m_state == find_nonempty_scanline_state) 
        {
          if (this->find_nonempty_scanline()) 
          {
            this->m_state = scanline_found_state;
          }
        }
        if (this->m_state == in_scanline_state) 
        {
          if (this->m_current_x < this->m_end_x - 1) 
          {
            ++(this->m_current_x);
          }
          else 
          {
            ++(this->m_left_ridge_iterator);
            ++(this->m_right_ridge_iterator);
            if (this->find_nonempty_scanline()) 
            {
              this->m_state = in_scanline_state;
            }
          }
        }
        if (this->m_state == scanline_found_state) 
        {
          this->m_state = in_scanline_state;
        }
        return this->valid();
      }

    public:

      /**
      * 
      * vertex coordinates are assumed to be transformed into screen
      * space of the camera. Normals can be given in whatever coordinate
      * frame one wants. Just remember that normals are lineary
      * interpolated in screen space.
      * 
      */
      void initialize(
        vector3_type const & v1
        , vector3_type const & n1
        , vector3_type const & v2
        , vector3_type const & n2
        , vector3_type const & v3
        , vector3_type const & n3
        )
      {
        this->m_vertex[0] = round( v1 );
        this->m_normal[0] = n1;
        this->m_vertex[1] = round( v2 );
        this->m_normal[1] = n2;
        this->m_vertex[2] = round( v3 );
        this->m_normal[2] = n3;

        this->m_lower_left_idx = this->find_lower_left_vertex_index();
        this->m_upper_left_idx = this->find_upper_left_vertex_index();

        if(this->m_lower_left_idx == this->m_upper_left_idx)
        {
          // We must have encountered a degenerate case! Either a point or horizontal line
          int a = (this->m_lower_left_idx + 1) % 3;
          int b = (a + 1) % 3;
          this->m_upper_left_idx = (this->m_vertex[a][0] < this->m_vertex[b][0]) ? a : b;
        }

        this->m_the_other_idx  = 3 - (this->m_lower_left_idx + this->m_upper_left_idx);
        assert( this->m_lower_left_idx != this->m_upper_left_idx || !"FragmentIterator::initialize() Invalid indices");
        assert( this->m_lower_left_idx != this->m_the_other_idx || !"FragmentIterator::initialize() Invalid indices");
        assert( this->m_upper_left_idx != this->m_the_other_idx || !"FragmentIterator::initialize() Invalid indices");

        this->m_left_ridge_iterator  = this->get_left_ridge_iterator();
        this->m_right_ridge_iterator = this->get_right_ridge_iterator();

        this->m_state = find_nonempty_scanline_state;
        this->find_nonempty_scanline();
      }

    };

  } // end of namespace scan_conversion
} // end of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_SCAN_CONVERSION_SCAN_CONVERSION_FRAGMENT_ITERATOR_H
#endif 
