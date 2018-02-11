#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_VOLUME_INTEGRATOR_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_VOLUME_INTEGRATOR_H
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
  namespace mesh
  {

    template<typename mesh_type>
    class VolumeIntegrator
    {
    public:

      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::value_traits                 value_traits;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename math_types::real_type                    real_type;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;

    protected:

      real_type P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;             ///< Projection integrals.
      real_type Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca; ///< Face integrals.
      real_type T0;                                                            ///< Volume integral.
      real_type T1[3];                                                         ///< Volume integral.
      real_type T2[3];                                                         ///< Volume integral.
      real_type TP[3];                                                         ///< Volume integral.

    public:

      VolumeIntegrator(  mesh_type const & mesh  )
      {
        compute_volume_integrals(mesh);
      };

    public:

      template<typename real_type2>
      void  get_volume( real_type2 & volume ) const
      {
        volume = T0;
      }

      template<typename real_type2>
      void get_mass(real_type2 const & density, real_type2 & mass) const
      {
        mass = density*T0;
      }

      template<typename real_type2,typename vector3_type2>
      void get_center_of_mass(real_type2 const & /*density*/,vector3_type2 & r) const
      {
        r(0) = T1[0] / T0;
        r(1) = T1[1] / T0;
        r(2) = T1[2] / T0;
      }

      template<typename real_type2,typename matrix3x3_type>
      void get_inertia_tensor(real_type2 const & density,matrix3x3_type & I) const
      {
        I(0,0) = density * (T2[1] + T2[2]);
        I(1,1) = density * (T2[2] + T2[0]);
        I(2,2) = density * (T2[0] + T2[1]);
        I(0,1) = I(1,0) = - density * TP[0];
        I(1,2) = I(2,1) = - density * TP[1];
        I(2,0) = I(0,2) = - density * TP[2];
      }

    protected:

      void compute_projection_integrals(const_face_iterator face,int A,int B,int /*C*/)
      {
        real_type a0, a1, da, b0, b1, db;
        real_type a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
        real_type a1_2, a1_3, b1_2, b1_3;
        real_type C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
        real_type Cab, Kab, Caab, Kaab, Cabb, Kabb;
        P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = static_cast<real_type>(0.0);

        const_face_vertex_circulator cur(*face),end;
        const_face_vertex_circulator next(*face);++next;
        for(;cur!=end;++cur,++next)
        {
          a0 = cur->m_coord(A);
          b0 = cur->m_coord(B);
          a1 = next->m_coord(A);
          b1 = next->m_coord(B);

          da = a1 - a0;
          db = b1 - b0;
          a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
          b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
          a1_2 = a1 * a1; a1_3 = a1_2 * a1;
          b1_2 = b1 * b1; b1_3 = b1_2 * b1;
          C1 = a1 + a0;
          Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
          Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
          Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
          Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
          Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
          Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;
          P1 += db*C1;
          Pa += db*Ca;
          Paa += db*Caa;
          Paaa += db*Caaa;
          Pb += da*Cb;
          Pbb += da*Cbb;
          Pbbb += da*Cbbb;
          Pab += db*(b1*Cab + b0*Kab);
          Paab += db*(b1*Caab + b0*Kaab);
          Pabb += da*(a1*Cabb + a0*Kabb);
        }

        P1 /= 2.0;
        Pa /= 6.0;
        Paa /= 12.0;
        Paaa /= 20.0;
        Pb /= -6.0;
        Pbb /= -12.0;
        Pbbb /= -20.0;
        Pab /= 24.0;
        Paab /= 60.0;
        Pabb /= -60.0;
      };

      void compute_face_integrals(const_face_iterator face, vector3_type const & n, int A, int B, int C)
      {
        real_type w, k1, k2, k3, k4;

        compute_projection_integrals(face,A,B,C);

        //--- let p be point on face!!!
        const_face_vertex_circulator p(*face);
        w = -n*p->m_coord;  //--- Mirtich defines n*p+w=0 I have defined n*p-w=0

        k1 = 1 / n(C);
        k2 = k1 * k1;
        k3 = k2 * k1;
        k4 = k3 * k1;

        Fa   = k1 * Pa;
        Fb   = k1 * Pb;
        Fc   = -k2 * (n(A)*Pa + n(B)*Pb + w*P1);

        Faa  = k1 * Paa;
        Fbb  = k1 * Pbb;
        Fcc  = k3 * ((n(A)*n(A))*Paa + 2*n(A)*n(B)*Pab +(n(B)*n(B))*Pbb + w*(2*(n(A)*Pa + n(B)*Pb) + w*P1));

        Faaa = k1 * Paaa;
        Fbbb = k1 * Pbbb;
        Fccc = -k4 * ((n(A)*n(A)*n(A))*Paaa + 3*(n(A)*n(A))*n(B)*Paab
          + 3*n(A)*(n(B)*n(B))*Pabb + (n(B)*n(B)*n(B))*Pbbb
          + 3*w*((n(A)*n(A))*Paa + 2*n(A)*n(B)*Pab + (n(B)*n(B))*Pbb)
          + w*w*(3*(n(A)*Pa + n(B)*Pb) + w*P1));

        Faab = k1 * Paab;
        Fbbc = -k2 * (n(A)*Pabb + n(B)*Pbbb + w*Pbb);
        Fcca = k3 * ((n(A)*n(A))*Paaa + 2*n(A)*n(B)*Paab + (n(B)*n(B))*Pabb
          + w*(2*(n(A)*Paa + n(B)*Pab) + w*Pa));
      };

      void compute_volume_integrals(mesh_type const & mesh)
      {
        T0 = T1[0] = T1[1] = T1[2]  = T2[0] = T2[1] = T2[2] = TP[0] = TP[1] = TP[2] = static_cast<real_type>(0.0);

        const_face_iterator end = mesh.face_end();
        const_face_iterator f   = mesh.face_begin();
        for (;f!=end;++f)
        {
          vector3_type n;
          compute_face_normal(*f,n);

          real_type nx = std::fabs(n(0));
          real_type ny = std::fabs(n(1));
          real_type nz = std::fabs(n(2));

          enum { X = 0, Y = 1, Z = 2};
          int A,B,C;

          if(nx > ny && nx > nz)
            C = X;
          else
            C = (ny > nz) ? Y : Z;
          A = (C + 1) % 3;
          B = (C + 2) % 3;

          compute_face_integrals(f,n,A,B,C);

          T0 += n(0) * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));
          T1[A] += n(A) * Faa;
          T1[B] += n(B) * Fbb;
          T1[C] += n(C) * Fcc;
          T2[A] += n(A) * Faaa;
          T2[B] += n(B) * Fbbb;
          T2[C] += n(C) * Fccc;
          TP[A] += n(A) * Faab;
          TP[B] += n(B) * Fbbc;
          TP[C] += n(C) * Fcca;
        }
        T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
        T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
        TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;
      };

    };

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_VOLUME_INTEGRATOR_H
#endif
