#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCES_PSYS_PRESSURE_SOFTBODY_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCES_PSYS_PRESSURE_SOFTBODY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/mass_spring_system/psys_surface_mesh.h>
#include <vector>
#include <iostream>

namespace OpenTissue
{
  namespace psys
  {

    /**
     * Adds a internal pressure force to a closed object.
     *
     * Note that the surrounds is seen as having zero-pressure. Thus unless one applies
     * some elastic surface forces to the object then the object will keep on expanding
     * its volume until the internal pressure have become zero. In general this is not
     * what one wants, therefore soft-springs should be added to the object to counter
     * the internal pressure. Further hard-stick constraints often do not create a nice
     * soft-blobby effect when combined with the pressure force. We recommend using soft-spring
     * forces instead although this have a dramatic impact on the time-step size.
     *
     */
    template< typename types  >
    class PressureSoftBody 
      : public types::force_type
    {
    public:

      typedef typename types::math_types                   math_types;
      typedef typename math_types::real_type               real_type;
      typedef typename math_types::vector3_type            vector3_type;
      typedef typename types::coupling_type                coupling_type;
      typedef typename types::mesh_type                    mesh_type;
      typedef typename mesh_type::face_type                face_type;
      typedef typename mesh_type::face_iterator            face_iterator;
      typedef typename mesh_type::face_vertex_circulator   face_vertex_circulator;

    protected:

      coupling_type   * m_coupling;   ///< A pointer to a mesh coupling.

      real_type m_P1;  ///< Internal variables used by volume computation.
      real_type m_Pa;
      real_type m_Pb; 
      real_type m_Fa;  ///< Internal variables used by volume computation.
      real_type m_Fb;
      real_type m_Fc; 

      enum { ///< Internal variables used by volume computation.
        m_X = 0,
        m_Y = 1,
        m_Z = 2
      };

      int m_A;   ///< Internal variables used by volume computation.
      int m_B;
      int m_C;

    protected:

      std::vector<real_type>    m_area;     ///< Storage for keeping the computed face areas.
      std::vector<real_type>    m_d;        ///< Storage for keeping the distance of face planes from origo.
      std::vector<vector3_type> m_normal;   ///< Storage for keepin the computed face normals.

      real_type m_initial_pressure_inside;  ///< Initial pressure inside.
      real_type m_initial_volume;           ///< Initial volume
      real_type m_nRT;                      ///< This values if computed from the initial pressure and value, by using P V = n R T

    public:

      PressureSoftBody()
        : m_initial_pressure_inside(10.0)
      {}

      virtual ~PressureSoftBody() {}

    public:

      void init(coupling_type const & coupling)
      {
        m_coupling = const_cast<coupling_type*>( &coupling );

        std::size_t N = m_coupling->mesh().size_faces();
        //--- Allocate space for internal data structures
        m_normal.resize( N);
        m_d.resize(N);
        m_area.resize(N);

        //--- Setup constants
        m_initial_volume = compute_volume_integral(m_coupling->mesh());
        m_nRT            = m_initial_pressure_inside * m_initial_volume;

        std::cout << "PressureSoftBody::init(): initial volume   = " 
          << m_initial_volume 
          << std::endl;
        std::cout << "PressureSoftBody::init(): initial pressure = " 
          << m_initial_pressure_inside 
          << std::endl;
        std::cout << "PressureSoftBody::init(): initial gas rhs = "  
          << m_nRT 
          << std::endl;
      }

      void set_initial_pressure(real_type const & pressure)
      {
        if(pressure>0)
        {
          m_initial_pressure_inside = pressure;
          if(m_initial_volume)
          {
            m_nRT = m_initial_pressure_inside * m_initial_volume;
          }
          else
          {
            m_nRT = 0;
          }
        }
      }

      real_type const & initial_pressure() const  { return m_initial_pressure_inside;  }

    public:

      void apply()
      {
        using std::fabs;

        if(!m_coupling)
          return;

        //--- Compute and Apply Pressure Forces
        real_type V         = compute_volume_integral(m_coupling->mesh());
        real_type pressure  = m_nRT/V;

        if(pressure<=0) //--- This does not make sense so we give up!
          return;

        mesh::compute_angle_weighted_vertex_normals( m_coupling->mesh() );

        face_iterator f   = m_coupling->mesh().face_begin();
        face_iterator end = m_coupling->mesh().face_end();
        for (int i=0;f!= end;++f,++i)
        {
          real_type N = valency(*f);
          real_type face_pressure = (fabs(m_area[i])*0.5*pressure);
          real_type vertex_pressure = face_pressure/N;
          typename mesh_type::face_vertex_circulator v(*f),vend;
          for(;v!=vend;++v)
            m_coupling->particle( *v ).force() +=  vertex_pressure*v->m_normal;
        }
      }

    protected:

      real_type const compute_volume_integral(mesh_type & mesh)
      {
        real_type volume = 0;
        face_iterator f   = mesh.face_begin();
        face_iterator end = mesh.face_end();
        for (int i=0;f!= end;++f,++i)
        {
          face_vertex_circulator p1(*f);
          face_vertex_circulator p2(*f);++p2;
          face_vertex_circulator p3(*f);++p3;++p3;

          vector3_type   u1,u2,u1xu2;

          u1 = p2->m_coord - p1->m_coord;
          u2 = p3->m_coord - p2->m_coord;
          u1xu2 = u1 % u2;
          m_area[i] = std::sqrt( u1xu2*u1xu2 );
          m_normal[i] = unit(u1xu2);
          m_d[i] = m_normal[i] * p1->m_coord;

          vector3_type n(m_normal[i]);

          real_type nx = std::fabs(n(m_X));
          real_type ny = std::fabs(n(m_Y));
          real_type nz = std::fabs(n(m_Z));
          
          if(!(nx || ny || nz)) //--- zero area face encountered, drop it!
            continue;


          if(nx > ny && nx > nz)
            m_C = m_X;
          else
            m_C = (ny > nz) ? m_Y : m_Z;
          m_A = (m_C + 1) % 3;
          m_B = (m_C + 2) % 3;

          compute_face_integral(*f,n,m_d[i]);

          volume += n(m_X) * ((m_A == m_X) ? m_Fa : ((m_B == m_X) ? m_Fb : m_Fc));
        }
        return volume;
      }

      void compute_face_integral(face_type & f,vector3_type & n,const real_type & d)
      {
        compute_projection_integral(f);
        real_type w = -d; //--- Mirtich defines n*p+d=0 I have defined n*p-d=0
        real_type k1 = 1 / n(m_C);
        real_type k2 = k1 * k1;
        m_Fa   = k1 * m_Pa;
        m_Fb   = k1 * m_Pb;
        m_Fc   = -k2 * (n(m_A)*m_Pa + n(m_B)*m_Pb + w*m_P1);
      }

      void compute_projection_integral(face_type & f)
      {
        m_P1 = m_Pa = m_Pb = static_cast<real_type>(0.0);
        face_vertex_circulator v1(f),vend;
        face_vertex_circulator v2(f);++v2;
        for(;v1!=vend;++v1,++v2)
        {
          real_type a0 = v1->m_coord(m_A);
          real_type b0 = v1->m_coord(m_B);
          real_type a1 = v2->m_coord(m_A);
          real_type b1 = v2->m_coord(m_B);
          real_type da = a1 - a0;
          real_type db = b1 - b0;
          real_type a0_2 = a0 * a0;
          real_type b0_2 = b0 * b0;
          real_type C1 = a1 + a0;
          real_type Ca = a1*C1 + a0_2;
          real_type Cb = b1*(b1 + b0) + b0_2;
          m_P1 += db*C1;
          m_Pa += db*Ca;
          m_Pb += da*Cb;
        }
        m_P1 /= static_cast<real_type>( 2.0);
        m_Pa /= static_cast<real_type>( 6.0);
        m_Pb /= static_cast<real_type>(-6.0);
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCES_PSYS_PRESSURE_SOFTBODY_H
#endif
