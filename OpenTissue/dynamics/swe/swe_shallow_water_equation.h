#ifndef OPENTISSUE_DYNAMICS_SWE_SWE_SHALLOW_WATER_EQUATIONS_H
#define OPENTISSUE_DYNAMICS_SWE_SWE_SHALLOW_WATER_EQUATIONS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/graphics/core/gl/gl_util.h>    // 2007-02-01 KE: Design issue: Having member hardwired to openGL is bad. These should be refactored to non-member template functions
#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_conjugate_gradient.h>
#include <OpenTissue/utility/utility_timer.h>

#include <boost/multi_array.hpp>

#include <cmath>
#include <cassert>
#include <iostream>

namespace OpenTissue
{
  namespace swe
  {

    /**
    * Shallow Water Equations.
    *
    * Implementation is based on the paper:
    *
    *   Anita T. Layton and Michiel van de Panne,
    *   `` A Numerically Efficient and Stable Algorithm for Animating Water Waves ,''
    *   the Visual Computer, Vol. 18, No. 1, pp. 41-53, 2002
    *
    *   http://www.amath.unc.edu/Faculty/layton/research/water/index.html
    */
    template<typename real_type_ = double>
    class ShallowWaterEquations
    {
    public:

      typedef  real_type_                          real_type;
      typedef  boost::multi_array<real_type, 2>    array_type;
      typedef  unsigned int                        index_type;
      typedef  ublas::compressed_matrix<real_type> matrix_type;
      typedef  ublas::vector<real_type>            vector_type;

    protected:

      real_type g;          ///< Gravitation constant.
      real_type timestep;   ///< The current timestep size used.
      unsigned int X;       ///< Number of grid nodes along the x-direction.
      unsigned int Y;       ///< Number of grid nodes along the y-direction.
      real_type dx;         ///< Step size between two neighboring nodes in the x-direction.
      real_type dy;         ///< Step size between two neighboring nodes in the y-direction.
      array_type water;     ///< Boolean flag indicating which grid nodes are water and which ones are solid ``earth''
      array_type b;         ///< Bottom height
      array_type h;         ///< Water height
      array_type u;         ///< Water velocity in x-direction
      array_type v;         ///< Water velocity in y-direction
      array_type d;         ///< Water Depth Grid, defined as: d = h - b
      array_type h_tilde;   ///< Water height at departure points
      array_type u_tilde;   ///< Water velocity in x-direction at departure points
      array_type v_tilde;   ///< Water velocity in y-direction at departure points
      array_type x_tilde;   ///< The x-coordinate of the departure points
      array_type y_tilde;   ///< The y-coordinate of the departure points
      matrix_type A;
      vector_type hnew;
      vector_type rhs;

    public:

      ShallowWaterEquations()
        : g(9.81)
        , timestep(0)
        , X(0)
        , Y(0)
        , dx(0)
        , dy(0)
        , water(boost::extents[0][0])
        , b(boost::extents[0][0])
        , h(boost::extents[0][0])
        , u(boost::extents[0][0])
        , v(boost::extents[0][0])
        , d(boost::extents[0][0])
        , h_tilde(boost::extents[0][0])
        , u_tilde(boost::extents[0][0])
        , v_tilde(boost::extents[0][0])
        , x_tilde(boost::extents[0][0])
        , y_tilde(boost::extents[0][0])
      {}

      ~ShallowWaterEquations()  {    clear();  }

    public:

      /**
      *
      * @param M
      * @param N
      * @param width
      * @param height
      */
      void init( const unsigned int M, const unsigned int N, const real_type width, const real_type height )
      {
        clear();

        g = 9.81; //--- gravitational constant

        this->X = M;
        this->Y = N;
        this->dx = width / X;
        this->dy = height / Y;

        water.resize(boost::extents[X][Y]);
        b.resize(boost::extents[X][Y]);
        h.resize(boost::extents[X][Y]);
        u.resize(boost::extents[X][Y]);
        v.resize(boost::extents[X][Y]);
        d.resize(boost::extents[X][Y]);
        h_tilde.resize(boost::extents[X][Y]);
        u_tilde.resize(boost::extents[X][Y]);
        v_tilde.resize(boost::extents[X][Y]);
        x_tilde.resize(boost::extents[X][Y]);
        y_tilde.resize(boost::extents[X][Y]);

        for ( index_type x = 0; x < X; ++x )
          for ( index_type y = 0; y < Y; ++y )
          {
            water[ x ][ y ] = 1;
            b[ x ][ y ] = 0;
            h[ x ][ y ] = 2;
            u[ x ][ y ] = 0;
            v[ x ][ y ] = 0;
            d[ x ][ y ] = 0;
            h_tilde[ x ][ y ] = 0;
            u_tilde[ x ][ y ] = 0;
            v_tilde[ x ][ y ] = 0;
            x_tilde[ x ][ y ] = 0;
            y_tilde[ x ][ y ] = 0;
          }

          int nvars = X * Y;
          // TODO: Syntax has changed in boost_1_32_0.
          //       Needs to find out if it is optimal/correct to perform a resize() followed by a clear().
          A.resize( nvars, nvars, false ); // current boost-cvs doesn't implement preserve
          A.clear();

          hnew.resize( nvars );
          hnew.clear();
          rhs.resize( nvars );
          rhs.clear();
      }

      void clear()
      {
        X = 0;
        Y = 0;
        dx = 0;
        dy = 0;
      }

      // 2007-02-01 KE: Design issue: Having member hardwired to openGL is bad. These should be refactored to non-member template functions
      /**
      * Draw water height field
      *
      * @param debugMode    Boolean variable indicating wheter the water simulation
      *                     is drawn in debug mode or not.
      */
      void draw( const bool debugMode ) const
      {
        //--- Nothing to draw here!
        if(X<=1 || Y<=1)
          return;

        if ( debugMode )
        {
          for ( index_type i = 0; i < X - 1; ++i )
          {
            for ( index_type j = 0; j < Y - 1; ++j )
            {
              real_type x = i * dx;
              real_type y = j * dx;
              real_type z00 = h[ i ][ j ];
              real_type z10 = h[ i + 1 ][ j ];
              real_type z01 = h[ i ][ j + 1 ];
              real_type z11 = h[ i + 1 ][ j + 1 ];
              gl::ColorPicker( 0.0, 0.0, 0.5 );
              glBegin( GL_LINE_LOOP );
              glVertex3f( x, y, z00 );
              glVertex3f( x + dx, y, z10 );
              glVertex3f( x + dx, y + dy, z11 );
              glVertex3f( x, y + dy, z01 );
              glEnd();
              real_type z00_tilde = h_tilde[ i ][ j ];
              real_type z10_tilde = h_tilde[ i + 1 ][ j ];
              real_type z01_tilde = h_tilde[ i ][ j + 1 ];
              real_type z11_tilde = h_tilde[ i + 1 ][ j + 1 ];
              gl::ColorPicker( 0., 1., 0. );
              glBegin( GL_LINE_LOOP );
              glVertex3f( x, y, z00_tilde );
              glVertex3f( x + dx, y, z10_tilde );
              glVertex3f( x + dx, y + dy, z11_tilde );
              glVertex3f( x, y + dy, z01_tilde );
              glEnd();
              real_type b00 = b[ i ][ j ];
              real_type b10 = b[ i + 1 ][ j ];
              real_type b01 = b[ i ][ j + 1 ];
              real_type b11 = b[ i + 1 ][ j + 1 ];
              gl::ColorPicker( 0.5, 0.1, 0 );
              glBegin( GL_LINE_LOOP );
              glVertex3f( x, y, b00 );
              glVertex3f( x + dx, y, b10 );
              glVertex3f( x + dx, y + dy, b11 );
              glVertex3f( x, y + dy, b01 );
              glEnd();
            }
          }
          for ( index_type i = 0; i < X; ++i )
          {
            for ( index_type j = 0; j < Y; ++j )
            {
              real_type x = i * dx;
              real_type y = j * dx;
              real_type z00 = h[ i ][ j ];
              real_type z00_tilde = h_tilde[ i ][ j ];
              gl::ColorPicker( 0, 0.5, 0.5 );
              glBegin( GL_LINES );
              glVertex3f( x, y, z00 );
              glVertex3f( x, y, z00_tilde );
              glEnd();
              real_type vx = u[ i ][ j ];
              real_type vy = v[ i ][ j ];
              gl::ColorPicker( 0.5, 0.5, 0 );
              glBegin( GL_LINES );
              glVertex3f( x, y, z00 );
              glVertex3f( x + vx, y + vy, z00 );
              glEnd();
              real_type px = x_tilde[ i ][ j ];
              real_type py = y_tilde[ i ][ j ];
              gl::ColorPicker( 0, 0.5, 0 );
              glBegin( GL_LINES );
              glVertex3f( x, y, z00 );
              glVertex3f( px, py, z00 );
              glEnd();
              real_type wx = u_tilde[ i ][ j ];
              real_type wy = v_tilde[ i ][ j ];
              gl::ColorPicker( 0.5, 0, 0.5 );
              glBegin( GL_LINES );
              glVertex3f( x, y, z00 );
              glVertex3f( x + wx, y + wy, z00 );
              glEnd();
            }
          }
        }
        else
        {
          glBegin( GL_QUADS );
          gl::ColorPicker( 0.3, 0.2, 0.1, 1.0 );
          for ( index_type i = 0; i < X - 1; ++i )
          {
            for ( index_type j = 0; j < Y - 1; ++j )
            {
              real_type x = i * dx;
              real_type y = j * dx;
              real_type b00 = b[ i ][ j ];
              real_type b10 = b[ i + 1 ][ j ];
              real_type b01 = b[ i ][ j + 1 ];
              real_type b11 = b[ i + 1 ][ j + 1 ];

              // i,  j, k
              //dx,  0, dBi
              // 0, dy, dBj
              float nx = -dy * ( b[ i + 1 ][ j ] - b[ i ][ j ] );
              float ny = -( dx * ( b[ i ][ j + 1 ] - b[ i ][ j ] ) );
              float nz = dx * dy;

              glNormal3f( nx, ny, nz );
              glVertex3f( x, y, b00 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x + dx, y, b10 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x + dx, y + dy, b11 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x, y + dy, b01 );
            }
          }
          gl::ColorPicker( 0.2, 0.2, 0.2, 1.0 );
          for ( index_type i = 0; i < X - 1; ++i )
          {
            for ( index_type j = 0; j < Y - 1; ++j )
            {
              real_type x = i * dx;
              real_type y = j * dx;
              // TODO: Comparing floats with == or != is not safe
              bool s00 = ( water[ i ][ j ] == 0 );
              bool s10 = ( water[ i + 1 ][ j ] == 0 );
              bool s01 = ( water[ i ][ j + 1 ] == 0 );
              bool s11 = ( water[ i + 1 ][ j + 1 ] == 0 );
              if ( s00 && s10 && s01 && s11 )
              {
                glNormal3f( 0, 0, 1 );
                glVertex3f( x, y, 4 );
                glNormal3f( 0, 0, 1 );
                glVertex3f( x + dx, y, 4 );
                glNormal3f( 0, 0, 1 );
                glVertex3f( x + dx, y + dy, 4 );
                glNormal3f( 0, 0, 1 );
                glVertex3f( x, y + dy, 4 );
              }
              else
              {
                if ( s00 && s01 )
                {
                  glNormal3f( 1, 0, 0 );
                  glVertex3f( x, y, 4 );
                  glNormal3f( 1, 0, 0 );
                  glVertex3f( x, y, 0 );
                  glNormal3f( 1, 0, 0 );
                  glVertex3f( x, y + dy, 0 );
                  glNormal3f( 1, 0, 0 );
                  glVertex3f( x, y + dy, 4 );
                }
                if ( s10 && s11 )
                {
                  glNormal3f( -1, 0, 0 );
                  glVertex3f( x + dx, y, 0 );
                  glNormal3f( -1, 0, 0 );
                  glVertex3f( x + dx, y, 4 );
                  glNormal3f( -1, 0, 0 );
                  glVertex3f( x + dx, y + dy, 4 );
                  glNormal3f( -1, 0, 0 );
                  glVertex3f( x + dx, y + dy, 0 );
                }
                if ( s00 && s10 )
                {
                  glNormal3f( 0, 1, 0 );
                  glVertex3f( x, y, 0 );
                  glNormal3f( 0, 1, 0 );
                  glVertex3f( x, y, 4 );
                  glNormal3f( 0, 1, 0 );
                  glVertex3f( x + dx, y, 4 );
                  glNormal3f( 0, 1, 0 );
                  glVertex3f( x + dx, y, 0 );
                }
                if ( s01 && s11 )
                {
                  glNormal3f( 0, -1, 0 );
                  glVertex3f( x, y + dy, 0 );
                  glNormal3f( 0, -1, 0 );
                  glVertex3f( x + dx, y + dy, 0 );
                  glNormal3f( 0, -1, 0 );
                  glVertex3f( x + dx, y + dy, 4 );
                  glNormal3f( 0, -1, 0 );
                  glVertex3f( x, y + dy, 4 );
                }
              }
            }
          }
          real_type x = 0;
          for ( index_type j = 0; j < Y - 1; ++j )
          {
            real_type y = j * dx;
            // TODO: Comparing floats with == or != is not safe
            bool s00 = ( water[ 0 ][ j ] == 0 );
            bool s01 = ( water[ 0 ][ j + 1 ] == 0 );
            if ( s00 && s01 )
            {
              glNormal3f( 1, 0, 0 );
              glVertex3f( x, y, 4 );
              glNormal3f( 1, 0, 0 );
              glVertex3f( x, y, 0 );
              glNormal3f( 1, 0, 0 );
              glVertex3f( x, y + dy, 0 );
              glNormal3f( 1, 0, 0 );
              glVertex3f( x, y + dy, 4 );
            }
          }
          x = ( X - 2 ) * dx;
          for ( index_type j = 0; j < Y - 1; ++j )
          {
            real_type y = j * dx;
            // TODO: Comparing floats with == or != is not safe
            bool s10 = ( water[ X - 1 ][ j ] == 0 );
            bool s11 = ( water[ X - 1 ][ j + 1 ] == 0 );
            if ( s10 && s11 )
            {
              glNormal3f( -1, 0, 0 );
              glVertex3f( x + dx, y, 0 );
              glNormal3f( -1, 0, 0 );
              glVertex3f( x + dx, y, 4 );
              glNormal3f( -1, 0, 0 );
              glVertex3f( x + dx, y + dy, 4 );
              glNormal3f( -1, 0, 0 );
              glVertex3f( x + dx, y + dy, 0 );
            }
          }
          real_type y = 0;
          for ( index_type i = 0; i < X - 1; ++i )
          {
            real_type x = i * dx;
            // TODO: Comparing floats with == or != is not safe
            bool s00 = ( water[ i ][ 0 ] == 0 );
            bool s10 = ( water[ i + 1 ][ 0 ] == 0 );
            if ( s00 && s10 )
            {
              glNormal3f( 0, 1, 0 );
              glVertex3f( x, y, 0 );
              glNormal3f( 0, 1, 0 );
              glVertex3f( x, y, 4 );
              glNormal3f( 0, 1, 0 );
              glVertex3f( x + dx, y, 4 );
              glNormal3f( 0, 1, 0 );
              glVertex3f( x + dx, y, 0 );
            }
          }
          y = ( Y - 2 ) * dx;
          for ( index_type i = 0; i < X - 1; ++i )
          {
            real_type x = i * dx;
            // TODO: Comparing floats with == or != is not safe
            bool s01 = ( water[ i ][ Y - 1 ] == 0 );
            bool s11 = ( water[ i + 1 ][ Y - 1 ] == 0 );
            if ( s01 && s11 )
            {
              glBegin( GL_POLYGON );
              glNormal3f( 0, -1, 0 );
              glVertex3f( x, y + dy, 0 );
              glNormal3f( 0, -1, 0 );
              glVertex3f( x + dx, y + dy, 0 );
              glNormal3f( 0, -1, 0 );
              glVertex3f( x + dx, y + dy, 4 );
              glNormal3f( 0, -1, 0 );
              glVertex3f( x, y + dy, 4 );
              glEnd();
            }
          }

          bool disableBlend = false;
          if ( !glIsEnabled( GL_BLEND ) )
          {
            glEnable( GL_BLEND );
            glBlendFunc( GL_ONE, GL_ONE_MINUS_SRC_ALPHA );
            disableBlend = true;
          }

          gl::ColorPicker( 0.0, 0.15, 0.5, 0.35 );

          for ( index_type i = 0; i < X - 1; ++i )
          {
            for ( index_type j = 0; j < Y - 1; ++j )
            {
              real_type x = i * dx;
              real_type y = j * dx;
              real_type z00 = h[ i ][ j ];
              real_type z10 = h[ i + 1 ][ j ];
              real_type z01 = h[ i ][ j + 1 ];
              real_type z11 = h[ i + 1 ][ j + 1 ];
              // i,  j, k
              //dx,  0, dBi
              // 0, dy, dBj
              float nx = -dy * ( h[ i + 1 ][ j ] - h[ i ][ j ] );
              float ny = -( dx * ( h[ i ][ j + 1 ] - h[ i ][ j ] ) );
              float nz = dx * dy;
              glNormal3f( nx, ny, nz );
              glVertex3f( x, y, z00 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x + dx, y, z10 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x + dx, y + dy, z11 );
              glNormal3f( nx, ny, nz );
              glVertex3f( x, y + dy, z01 );
            }
          }
          glEnd();
          if ( disableBlend )
          {
            glDisable( GL_BLEND );
          }

        }
      }


      /**
      *
      * @param time_step
      */
      void run( const real_type time_step, bool statistics = false )
      {
        assert( time_step > 0 && "timestep must be positive");

        OpenTissue::utility::Timer<double> timer;
        double timeSetup = 0;
        double timeAssemble = 0;
        double timeCG = 0;
        double timeUpdate = 0;
        timer.start();

        this->timestep = time_step;

        compute_departure_points();
        compute_value_at_departure_points( h, h_tilde );
        compute_value_at_departure_points( u, u_tilde );
        compute_value_at_departure_points( v, v_tilde );
        compute_depth();

        timer.stop();
        timeSetup = timer();
        timer.start();

        assemble( A, rhs );

        timer.stop();
        timeAssemble = timer();
        timer.start();

        math::big::conjugate_gradient(A, hnew, rhs);

        timer.stop();
        timeCG = timer();
        timer.start();

        set_height( hnew );
        update_u();
        update_v();

        apply_velocity_constraints();
        nullify();

        timer.stop();
        timeUpdate = timer();

        if (statistics)
        {
          std::cout << "CFD_ShallowWaterEquations::run(" << timestep << ") took (in milisecs.):" << std::endl;
          std::cout << "  Setup    : " << timeSetup << std::endl;
          std::cout << "  Assemble : " << timeAssemble << std::endl;
          std::cout << "  ConjGrad : " << timeCG << std::endl;
          std::cout << "  Update   : " << timeUpdate << std::endl;
        }
      }

      /**
      * Set Water Velocity
      *
      * @param i    The index of water node grid along x-axe direction
      * @param j    The index of water node grid along x-axe direction
      *
      * @param vx    The new x-direction velocity of the (i,j) water grid node.
      * @param vy    The new x-direction velocity of the (i,j) water grid node.
      */
      void setSeaVelocity( const unsigned int i, const unsigned int j, const real_type vx, const real_type vy )
      {
        assert( i < X && "index out of bounds");
        assert( j < Y && "index out of bounds");
        u[ i ][ j ] = vx;
        v[ i ][ j ] = vy;
      }

      /**
      * Set Water Height
      *
      * @param i    The index of water node grid along x-axe direction
      * @param j    The index of water node grid along x-axe direction
      *
      * @param height    The new height value of the (i,j) water grid node.
      */
      void setSeaHeight( const unsigned int i, const unsigned int j, const real_type height )
      {
        assert( i < X && "index out of bounds" );
        assert( j < Y && "index out of bounds" );
        h[ i ][ j ] = height;
        if ( h[ i ][ j ] < b[ i ][ j ] )
          b[ i ][ j ] = h[ i ][ j ];
      }

      /**
      * Set Bottom Height
      *
      * @param i    The index of water node grid along x-axe direction
      * @param j    The index of water node grid along x-axe direction
      *
      * @param height    The new bottom height value of the (i,j) water grid node.
      */
      void setSeaBottom( const unsigned int i, const unsigned int j, const real_type bottom )
      {
        assert( i < X && "index out of bounds" );
        assert( j < Y && "index out of bounds" );
        b[ i ][ j ] = bottom;
        if ( b[ i ][ j ] > h[ i ][ j ] )
          h[ i ][ j ] = b[ i ][ j ];
      }

      /**
      * Set Shore
      *
      * @param i    The index of water node grid along x-axe direction
      * @param j    The index of water node grid along x-axe direction
      *
      * @param shore    If true the (i,j) grid node is set to be shore if false the node is water.
      */
      void setShore( const unsigned int i, const unsigned int j, const bool shore )
      {
        assert( i < X && "index out of bounds" );
        assert( j < Y && "index out of bounds" );
        water[ i ][ j ] = !shore;
      }

    protected:

      void set_height( vector_type & h_new )
      {
        typename vector_type::iterator hval = h_new.begin();
        for ( index_type x = 0; x < X; ++x )
        {
          for ( index_type y = 0; y < Y; ++y )
          {
            h[ x ][ y ] = *hval++;
            //--- KE 17-04-2004: I have found this little sanity-test to be very usefull, in
            //--- this model we can expect errors of size O(h), where h is the typical discrentization
            //--- size, this means that the model sometimes predics a water height below the bottom.
            //--- This is indeed unphysical, so I correct the error by projecting the water
            //--- surface back to the sea bottom.
            if ( h[ x ][ y ] < b[ x ][ y ] )
              h[ x ][ y ] = b[ x ][ y ];
          }
        }
      }


      void assemble( matrix_type & A, vector_type & rhs ) const
      {
        A.clear();  //--- Set all entries to zero.
        // Given m and n, we know by construction that:
        //
        //  h[i,j] = h(i*Y + j)
        //
        //
        //--- Precomputed constants, no need to compute these for every grid node!!!
        real_type dxdx = dx * dx;
        real_type dydy = dy * dy;
        real_type dt = timestep;
        real_type dtdt = dt * dt;
        real_type dt_2dx = dt / ( 2 * dx );
        real_type dt_2dy = dt / ( 2 * dy );
        real_type dtdtgdxdx = ( dtdt * g ) / ( dxdx );
        real_type dtdtgdydy = ( dtdt * g ) / ( dydy );
        real_type dtdtgdxdx_4 = 0.25 * dtdtgdxdx;
        real_type dtdtgdydy_4 = 0.25 * dtdtgdydy;

        for ( index_type i = 0;i < X;++i )
        {
          for ( index_type j = 0;j < Y;++j )
          {
            unsigned int row = i * Y + j;   //--- Row of A which gives us the differences for the h[i.j] height value

            unsigned int idx_ip1_j = ( i + 1 ) * Y + j;
            unsigned int idx_i_j = i * Y + j;
            unsigned int idx_i_jp1 = i * Y + ( j + 1 );
            unsigned int idx_im1_j = ( i - 1 ) * Y + j;
            // TODO: Not used?
            //        unsigned int idx_im1_jp1 = ( i - 1 ) * Y + ( j + 1 );
            //        unsigned int idx_ip1_jm1 = ( i + 1 ) * Y + ( j - 1 );
            unsigned int idx_i_jm1 = i * Y + ( j - 1 );

            A( row, idx_i_j ) += 1;

            real_type dBi = 0;
            if ( ( i + 1 ) < X )
              dBi = b[ i + 1 ][ j ];
            else
              dBi = b[ i ][ j ];
            if ( i >= 1 )
              dBi -= b[ i - 1 ][ j ];
            else
              dBi -= b[ i ][ j ];

            // TODO: Comparing floats with == or != is not safe
            if ( dBi )
            {
              //real_type k = (0.25*dtdt*g*dBi)/(dxdx);
              real_type k = dtdtgdxdx_4 * dBi;
              if ( ( i + 1 ) < X )
                A( row, idx_ip1_j ) += k;
              else                     //--- HACK
                A( row, idx_i_j ) += 2 * k;   //--- HACK

              if ( i >= 1 )
                A( row, idx_im1_j ) -= k;
              else                     //--- HACK
                A( row, idx_i_j ) -= 2 * k;   //--- HACK
            }

            real_type dBj = 0;
            if ( ( j + 1 ) < Y )
              dBj = b[ i ][ j + 1 ];
            else
              dBj = b[ i ][ j ];
            if ( j >= 1 )
              dBj -= b[ i ][ j - 1 ];
            else
              dBj -= b[ i ][ j ];

            // TODO: Comparing floats with == or != is not safe
            if ( dBj )
            {
              //real_type h = (0.25*dtdt*g*dBj)/(dydy);
              real_type h = dtdtgdydy_4 * dBj;
              if ( ( j + 1 ) < Y )
                A( row, idx_i_jp1 ) += h;
              else                     //--- HACK
                A( row, idx_i_j ) += 2 * h;   //--- HACK

              if ( j >= 1 )
                A( row, idx_i_jm1 ) -= h;
              else                     //--- HACK
                A( row, idx_i_j ) -= 2 * h;   //--- HACK
            }

            // TODO: Comparing floats with == or != is not safe
            if ( d[ i ][ j ] )
            {
              //real_type w = dtdt*g*d[i][j]/(dxdx);
              real_type w = dtdtgdxdx * d[ i ][ j ];
              if ( i >= 1 )
                A( row, idx_im1_j ) -= w;
              else                     //--- HACK
                A( row, idx_i_j ) -= w;   //--- HACK
              A( row, idx_i_j ) += 2 * w;
              if ( ( i + 1 ) < X )
                A( row, idx_ip1_j ) -= w;
              else                     //--- HACK
                A( row, idx_i_j ) -= w;   //--- HACK

              //real_type q = dtdt*g*d[i][j]/(dydy);
              real_type q = dtdtgdydy * d[ i ][ j ];
              if ( j >= 1 )
                A( row, idx_i_jm1 ) -= q;
              else                       //--- HACK
                A( row, idx_i_j ) -= q;     //--- HACK
              A( row, idx_i_j ) += 2 * q;
              if ( ( j + 1 ) < Y )
                A( row, idx_i_jp1 ) -= q;
              else                       //--- HACK
                A( row, idx_i_j ) -= q;     //--- HACK
            }

            //rhs(row) = h_tilde[i][j] + dt*(u_tilde[i][j]*dBi/(2*dx) + v_tilde[i][j]*dBj/(2*dy));
            rhs( row ) = h_tilde[ i ][ j ] + u_tilde[ i ][ j ] * dBi * dt_2dx + v_tilde[ i ][ j ] * dBj * dt_2dy;

            real_type dU = 0;
            if ( ( i + 1 ) < X )
              dU = u_tilde[ i + 1 ][ j ];
            if ( i >= 1 )
              dU -= u_tilde[ i - 1 ][ j ];

            real_type dV = 0;
            if ( ( j + 1 ) < Y )
              dV = v_tilde[ i ][ j + 1 ];
            if ( j >= 1 )
              dV -= v_tilde[ i ][ j - 1 ];

            //rhs(row) -= dt*d[i][j]*(dU/(2.*dx) + dV/(2.*dy));
            rhs( row ) -= d[ i ][ j ] * ( dU * dt_2dx + dV * dt_2dy );
          }
        }
      }


      /**
      * Computes water depth, assumes that water height have been computed prior to invocation.
      */
      void compute_depth()
      {
        static real_type epsilon = math::working_precision<real_type>();

        for ( unsigned int i = 0; i < X; ++i )
        {
          for ( unsigned int j = 0; j < Y; ++j )
          {
            d[ i ][ j ] = h[ i ][ j ] - b[ i ][ j ];
            //--- KE 17-04-2004: I have found this little sanity-test to be very usefull, in
            //--- this model we can expect errors of size O(h), where h is the typical discrentization
            //--- size, this means that the model sometimes predics a water height below the bottom.
            //--- This is indeed unphysical, so I correct the error by projecting the water
            //--- surface back to the sea bottom.
            if ( d[ i ][ j ] < epsilon )
              d[ i ][ j ] = 0;
          }
        }
      }

      /**
      * Computes departure points, implicitly assumes that velocities have been computed prior to invocation
      */
      void compute_departure_points()
      {
        real_type x = 0;
        for ( unsigned int i = 0;i < X;++i )
        {
          real_type y = 0;
          for ( unsigned int j = 0;j < Y;++j )
          {
            x_tilde[ i ][ j ] = x - timestep * u[ i ][ j ];
            y_tilde[ i ][ j ] = y - timestep * v[ i ][ j ];
            y += dy;
          }
          x += dx;
        }
      }


      /**
      * Implicitly assumes that departure points have been computed prior to invocation
      *
      */
      void compute_value_at_departure_points( array_type & value, array_type & value_tilde )
      {
        using std::floor;

        for ( unsigned int i = 0; i < X; ++i )
        {
          for ( unsigned int j = 0; j < Y; ++j )
          {
            real_type tmp_x = x_tilde[ i ][ j ] / dx;
            real_type tmp_y = y_tilde[ i ][ j ] / dy;
            int i00 = static_cast<int>( floor( tmp_x ) );
            int j00 = static_cast<int>( floor( tmp_y ) );
            real_type x_low = i00 * dx;
            real_type y_low = j00 * dy;
            real_type x_high = x_low + dx;
            real_type y_high = y_low + dy;

            real_type val00 = 0;
            real_type val10 = 0;
            real_type val01 = 0;
            real_type val11 = 0;

            if ( i00 >= 0 && j00 >= 0 && static_cast<unsigned int>(i00) < X && static_cast<unsigned int>(j00) < Y )
              val00 = value[ i00 ][ j00 ];
            if ( ( i00 + 1 ) >= 0 && j00 >= 0 && static_cast<unsigned int>( i00 + 1 ) < X && static_cast<unsigned int>(j00) < Y )
              val10 = value[ i00 + 1 ][ j00 ];
            if ( i00 >= 0 && ( j00 + 1 ) >= 0 && static_cast<unsigned int>(i00) < X && static_cast<unsigned int>( j00 + 1 ) < Y )
              val01 = value[ i00 ][ j00 + 1 ];
            if ( ( i00 + 1 ) >= 0 && ( j00 + 1 ) >= 0 && static_cast<unsigned int>( i00 + 1 ) < X && static_cast<unsigned int>( j00 + 1 ) < Y )
              val11 = value[ i00 + 1 ][ j00 + 1 ];

            real_type s1 = x_high - x_tilde[ i ][ j ];
            real_type s0 = x_tilde[ i ][ j ] - x_low;

            real_type tmp0 = ( s0 * val10 + s1 * val00 ) / dx;
            real_type tmp1 = ( s0 * val11 + s1 * val01 ) / dx;

            real_type t1 = y_high - y_tilde[ i ][ j ];
            real_type t0 = y_tilde[ i ][ j ] - y_low;

            value_tilde[ i ][ j ] = ( t0 * tmp1 + t1 * tmp0 ) / dy;
          }
        }
      }

      /**
      * Implicitly assumes that new height field have been stored.
      */
      void update_u()
      {
        for ( unsigned int i = 0; i < X; ++i )
        {
          for ( unsigned int j = 0; j < Y; ++j )
          {
            real_type Dh = 0;
            if ( ( i + 1 ) < X && water[ i + 1 ][ j ] )
              Dh = h[ i + 1 ][ j ];
            else
              Dh = h[ i ][ j ];
            if ( i >= 1 && water[ i - 1 ][ j ] )
              Dh -= h[ i - 1 ][ j ];
            else
              Dh -= h[ i ][ j ];
            Dh /= ( 2 * dx );
            u[ i ][ j ] = u_tilde[ i ][ j ] - timestep * g * Dh;
          }
        }
      }

      /**
      * Implicitly assumes that new height field have been stored.
      */
      void update_v()
      {
        for ( unsigned int i = 0; i < X; ++i )
        {
          for ( unsigned int j = 0; j < Y; ++j )
          {
            real_type Dh = 0;
            if ( ( j + 1 ) < Y && water[ i ][ j + 1 ] )
              Dh = h[ i ][ j + 1 ];
            else
              Dh = h[ i ][ j ];
            if ( j >= 1 && water[ i ][ j - 1 ] )
              Dh -= h[ i ][ j - 1 ];
            else
              Dh -= h[ i ][ j ];
            Dh /= ( 2 * dy );
            v[ i ][ j ] = v_tilde[ i ][ j ] - timestep * g * Dh;
          }
        }
      }

      /**
      *
      * Constaint velocities along wall's to have zero normal component
      */
      void apply_velocity_constraints()
      {
        for ( unsigned int i = 0;i < X;++i )
        {
          for ( unsigned int j = 0;j < Y;++j )
          {
            if ( water[ i ][ j ] )
            {
              //--- lookup neighborhood to see if we are on a wall boundary
              bool wall_left = false;
              bool wall_right = false;
              bool wall_up = false;
              bool wall_down = false;
              bool wall_left_up = false;
              bool wall_right_up = false;
              bool wall_left_down = false;
              bool wall_right_down = false;

              if ( i >= 1 )
                wall_left = ( water[ i - 1 ][ j ] == 0 );
              if ( ( i + 1 ) < X )
                wall_right = ( water[ i + 1 ][ j ] == 0 );
              if ( ( j + 1 ) < Y )
                wall_up = ( water[ i ][ j + 1 ] == 0 );
              if ( j >= 1 )
                wall_down = ( water[ i ][ j - 1 ] == 0 );
              if ( i >= 1 && ( j + 1 ) < Y )
                wall_left_up = ( water[ i - 1 ][ j + 1 ] == 0 );
              if ( ( i + 1 ) < X && ( j + 1 ) < Y )
                wall_right_up = ( water[ i + 1 ][ j + 1 ] == 0 );
              if ( i >= 1 && j >= 1 )
                wall_left_down = ( water[ i - 1 ][ j - 1 ] == 0 );
              if ( ( i + 1 ) < X && j >= 1 )
                wall_right_down = ( water[ i + 1 ][ j - 1 ] == 0 );

              //--- straight sides and concave corners
              if ( wall_left || wall_right )
                u[ i ][ j ] = 0;

              if ( wall_up || wall_down )
                v[ i ][ j ] = 0;

              //--- convex corners
              if ( wall_left_down && !wall_left && !wall_down )
              {
                real_type nx = 1;
                real_type ny = 1;
                real_type ux = u[ i ][ j ];
                real_type uy = v[ i ][ j ];
                ux = ux - ( ux * nx + uy * ny ) * nx;
                uy = uy - ( ux * nx + uy * ny ) * ny;
                u[ i ][ j ] = ux;
                v[ i ][ j ] = uy;

              }
              if ( wall_right_down && !wall_right && !wall_down )
              {
                real_type nx = -1;
                real_type ny = 1;
                real_type ux = u[ i ][ j ];
                real_type uy = v[ i ][ j ];
                ux = ux - ( ux * nx + uy * ny ) * nx;
                uy = uy - ( ux * nx + uy * ny ) * ny;
                u[ i ][ j ] = ux;
                v[ i ][ j ] = uy;
              }
              if ( wall_left_up && !wall_left && !wall_up )
              {
                real_type nx = 1;
                real_type ny = -1;
                real_type ux = u[ i ][ j ];
                real_type uy = v[ i ][ j ];
                ux = ux - ( ux * nx + uy * ny ) * nx;
                uy = uy - ( ux * nx + uy * ny ) * ny;
                u[ i ][ j ] = ux;
                v[ i ][ j ] = uy;
              }
              if ( wall_right_up && !wall_right && !wall_up )
              {
                real_type nx = -1;
                real_type ny = -1;
                real_type ux = u[ i ][ j ];
                real_type uy = v[ i ][ j ];
                ux = ux - ( ux * nx + uy * ny ) * nx;
                uy = uy - ( ux * nx + uy * ny ) * ny;
                u[ i ][ j ] = ux;
                v[ i ][ j ] = uy;
              }
            }
            else
            {
              u[ i ][ j ] = 0;
              v[ i ][ j ] = 0;
            }

          }
        }
      }


      void nullify()
      {
        real_type threshold = 10e-5;

        for ( unsigned int i = 0;i < X;++i )
        {
          for ( unsigned int j = 0;j < Y;++j )
          {

            if ( d[ i ][ j ] < threshold )
            {
              bool left = true;
              bool right = true;
              bool up = true;
              bool down = true;
              bool left_up = true;
              bool right_up = true;
              bool left_down = true;
              bool right_down = true;

              if ( i >= 1 )
                left = ( d[ ( i - 1 ) ][ j ] < threshold );
              if ( ( i + 1 ) < X )
                right = ( d[ ( i + 1 ) ][ j ] < threshold );
              if ( ( j + 1 ) < Y )
                up = ( d[ i ][ ( j + 1 ) ] < threshold );
              if ( j >= 1 )
                down = ( d[ i ][ ( j - 1 ) ] < threshold );
              if ( i >= 1 && ( j + 1 ) < Y )
                left_up = ( d[ ( i - 1 ) ][ ( j + 1 ) ] < threshold );
              if ( ( i + 1 ) < X && ( j + 1 ) < Y )
                right_up = ( d[ ( i + 1 ) ][ ( j + 1 ) ] < threshold );
              if ( i >= 1 && j >= 1 )
                left_down = ( d[ ( i - 1 ) ][ ( j - 1 ) ] < threshold );
              if ( ( i + 1 ) < X && j >= 1 )
                right_down = ( d[ ( i + 1 ) ][ ( j - 1 ) ] < threshold );

              if ( left && right && up && down && left_down && right_down && left_up && right_up )
              {
                u[ i ][ j ] = 0;
                v[ i ][ j ] = 0;
              }
            }
          }
        }
      }

    };

  } // namespace swe
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SWE_SWE_SHALLOW_WATER_EQUATIONS_H
#endif
