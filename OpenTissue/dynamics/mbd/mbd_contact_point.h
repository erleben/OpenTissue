#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_CONTACT_POINT_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_CONTACT_POINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace mbd
  {

    template<  typename types >
    class ContactPoint : public types::constraint_type
    {
    public:

      typedef typename types::math_policy            math_policy;
      typedef typename math_policy::size_type        size_type;
      typedef typename math_policy::real_type        real_type;
      typedef typename math_policy::value_traits     value_traits;
      typedef typename math_policy::vector3_type     vector3_type;
      typedef typename math_policy::matrix3x3_type   matrix3x3_type;
      typedef typename math_policy::quaternion_type  quaternion_type;
      typedef typename math_policy::vector_type      vector_type;
      typedef typename math_policy::vector_range     vector_range;
      typedef typename math_policy::idx_vector_range idx_vector_range;
      typedef typename math_policy::matrix_range     matrix_range;

      typedef typename types::body_type              body_type;
      typedef typename types::material_type          material_type;

    public:

      bool m_use_stabilization;       ///< Boolean value indicating whether stabilization should be ignored in the error terms.
      bool m_use_friction;            ///< Boolean value indicating whether firctional constaints should be ignored.
      bool m_use_bounce;              ///< Boolean value indicating whether newtons collision law should included in the error terms..

      material_type * m_material;     ///< The material properties, such as coefficient of friction.

      vector3_type m_n;               ///< The contact normal in WCS. Always pointing from A towards B (i.e. from lower index to higher index).
      vector3_type m_p;               ///< The contact point in WCS.
      real_type m_distance;           ///< Separation (or penetration, if negative) distance.

      vector3_type m_rA;              ///< The vector from center of mass of body A to the point of contact (in WCS).
      vector3_type m_rB;              ///< The vector from center of mass of body B to the point of contact (in WCS).
      real_type    m_un;              ///< The relative contact velocity in the normal direction.
      size_type    m_eta;             ///< Number of friction direction used.
      std::vector<vector3_type> m_t;  ///< Temporary Storage used for keeping the friction directions.

      bool m_bodies_flipped;          ///< By definition a contact point alwyas set body
      ///< A to have lowest index, however end user may
      ///< have initialized the contact points with body
      ///< A as highest index, if so this member is set to true.

      vector_type m_solution;         ///< Local solution vector, ie. vector of lagrange multipliers.

    public:

      ContactPoint()
        : m_use_stabilization(true)
        , m_use_friction(true)
        , m_use_bounce(true)
        , m_material(0)
      {}

      virtual ~ContactPoint(){}

    public:

      void set_use_friction     (bool value)    {      m_use_friction = value;         }
      void set_use_bounce       (bool value)    {      m_use_bounce = value;           }
      void set_use_stabilization(bool value)    {      m_use_stabilization = value;    }

      /**
      * Initialization Routine.
      *
      * @param bodyA         A pointer to one of the bodies in contact.
      * @param bodyB         A pointer to the other body in contact.
      * @param p             The actual point of contact in WCS.
      * @param n             A unit vector giving the value of the contact normal
      *                      in WCS. The normal is assumed to be oriented from body A to body B.
      * @param distance      The separation/penetration distance
      * @param material      A pointer to the material properties of the contact point.
      */
      void init(
        body_type * bodyA
        , body_type * bodyB
        , vector3_type const & p
        , vector3_type const & n
        , real_type const & distance
        , material_type * material
        )
      {
        assert(bodyA  || !"ContactPoint::init() body A was null");
        assert(bodyB  || !"ContactPoint::init() body B was null");

        assert(bodyA!=bodyB                             || !"ContactPoint::init() body A was the same as body B");
        assert(bodyA->get_index()!=bodyB->get_index()   || !"ContactPoint::init() body A and B have the same index");

        assert(is_number(p(0))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(p(1))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(p(2))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(n(0))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(n(1))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(n(2))     || !"ContactPoint::init() not a number encountered");
        assert(is_number(distance) || !"ContactPoint::init() not a number encountered");

        if(bodyA->get_index()<bodyB->get_index())
        {
          m_bodies_flipped = false;
          this->m_bodyA = bodyA;
          this->m_bodyB = bodyB;
          m_n = n;
        }
        else
        {
          m_bodies_flipped = true;
          this->m_bodyA = bodyB;
          this->m_bodyB = bodyA;
          m_n = -n;
        }

        m_p = p;
        m_distance = distance;
        m_material = material;

        vector3_type r_A_wcs;
        vector3_type r_B_wcs;
        this->m_bodyA->get_position(r_A_wcs);
        this->m_bodyB->get_position(r_B_wcs);
        m_rA = p - r_A_wcs;
        m_rB = p - r_B_wcs;
      }

      /**
      * @see ConstraintInterface.evaluate()
      */
      void evaluate()
      {
        assert(this->m_bodyA || !"ContactPoint::evaluate(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::evaluate(): body B was null");
        assert(m_material    || !"ContactPoint::evaluate(): Material was null");

        vector3_type u,v_a,w_a,v_b,w_b;

        this->get_body_A()->get_velocity(v_a);
        this->get_body_B()->get_velocity(v_b);
        this->get_body_A()->get_spin(w_a);
        this->get_body_B()->get_spin(w_b);

        u = mbd::compute_relative_contact_velocity(v_a,w_a,m_rA,v_b,w_b,m_rB);

        m_un = m_n * u;  //--- The relative contact velocity in the normal direction.

        //--- Now we can compute the friction direction vectors.
        m_eta = m_material->get_number_of_friction_directions();

        // Check whether global override is set?
        if(!m_use_friction)
          m_eta = 0;

        if(m_eta>0)
        {
          //Allocate space for the friction direction vectors
          if(m_eta != m_t.size())
            m_t.resize(m_eta);

          // In the following we will compute friction plane tangent
          // vectors, x and y, these vectors span the entire
          // friction plane.
          vector3_type x;
          vector3_type y;
          x.clear();
          y.clear();

          // First we must determine how the orientation of the
          // x-vector should be computed! In can either be picked
          // from relative sliding direction, or some prefixed
          // direction or be generated from some orthonormal set.
          if( m_material->get_use_sliding_direction() )
          {
            // A little neet trick, if the friction cone is badly approximated
            // (low eta), there might not even be a friction direction along
            // the relative sliding direction. To remedy this we try to pick/orientate
            // the tangent vectors such that the first vector lies in the direction
            // of relative sliding.
            //
            // Intuitively this will help reduce the error in the friction
            // approximation.
            vector3_type un = m_n * m_un;  //--- normal relative velocity
            vector3_type ut = u - un;      //--- tangential relative velocity            
            if( !is_zero(ut) )
            {
              x = unit(ut);
              y = unit( cross(m_n , x));
            }
            else
            {
              // No sliding direction could be computed, so as a fail-safe
              // we simply pick some orthonormal vectors
              orthonormal_vectors(x,y,m_n);
            }
          }
          else if(m_material->get_use_prefixed_direction() )
          {
            quaternion_type Q;
            vector3_type v = m_material->get_prefixed_direction();

            bool is_direction_fixed_in_A = m_material->get_prefixed_material_index() == this->get_body_A()->get_material_idx();
            bool is_direction_fixed_in_B = m_material->get_prefixed_material_index() == this->get_body_B()->get_material_idx();
            if( is_direction_fixed_in_A)
            {
              this->get_body_A()->get_orientation(Q);
            }
            else if( is_direction_fixed_in_B)
            {
              this->get_body_B()->get_orientation(Q);
            }
            else
            {
              // Direction was neither fixed in frame A or B so it must
              // be fixed in world frame.
              Q.identity();
            }
            // Now we rotate the prefixed direction into the world frame
            vector3_type w = unit( Q.rotate(v) );
            // Next we project w onto the tangent plane to get the x-direction
            x = w - (w*m_n)*m_n;      
            if( !is_zero(x) )
            {
              y = unit( cross(m_n , x));
            }
            else
            {
              // The projected vector was a zero vector, so as a fail-safe
              // we simply pick some orthonormal vectors.
              orthonormal_vectors(x,y,m_n);
            }
          }
          else
          {
            // If nothing else applies then simply choose some orthonormal vectors
            orthonormal_vectors(x,y,m_n);
          }

          // Now we have the span vectors, so now we can compute
          // the friction directions based on these orthonormal span
          // vectors. In particular the direction of the x-vector is
          // important, since this will determine the orientation the
          // friction cone entirely.
          m_t[0] = x;
          if(m_eta==2)
          {
            m_t[1] = y;
          }
          else if(m_eta>2)
          {
            // Finally the general case of n friction directions...
            real_type angle = value_traits::pi() / m_eta; // We are implicitly assuming a possitive span of friction direction!
            matrix3x3_type R;

            R = Ru(angle,m_n);
            for(size_t i=1;i<m_eta;++i)
            {
              m_t[i] = R*m_t[i-1];
            }
          }
        }
      }

      /**
      * @see ConstraintInterface.get_number_of_jacobian_rows()
      */
      size_type get_number_of_jacobian_rows() const  { return m_eta+1; }

      /**
      * @see ConstraintInterface.get_linear_jacobian_A()
      */
      void get_linear_jacobian_A(matrix_range & J)const
      {
        assert(this->m_bodyA || !"ContactPoint::get_linear_jacobian_A(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_linear_jacobian_A(): body B was null");

        assert(J.size1()==get_number_of_jacobian_rows() || !"ContactPoint::get_linear_jacobian_A(): incorrect dimension");
        assert(J.size2()==3                             || !"ContactPoint::get_linear_jacobian_A(): incorrect dimension");

        J(0,0) = -m_n(0);
        J(0,1) = -m_n(1);
        J(0,2) = -m_n(2);
        for(size_t i=0;i<m_eta;++i)
        {
          size_t j = i+1;
          J(j,0) = -m_t[i](0);
          J(j,1) = -m_t[i](1);
          J(j,2) = -m_t[i](2);
        }
      }

      /**
      * @see ConstraintInterface.get_linear_jacobian_B()
      */
      void get_linear_jacobian_B(matrix_range & J) const
      {
        assert(this->m_bodyA || !"ContactPoint::get_linear_jacobian_B(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_linear_jacobian_B(): body B was null");

        assert(J.size1()==get_number_of_jacobian_rows() || !"ContactPoint::get_linear_jacobian_B(): incorrect dimension");
        assert(J.size2()==3                             || !"ContactPoint::get_linear_jacobian_B(): incorrect dimension");

        J(0,0) = m_n(0);
        J(0,1) = m_n(1);
        J(0,2) = m_n(2);
        for(size_t i=0;i<m_eta;++i)
        {
          size_t j = i+1;
          J(j,0) = m_t[i](0);
          J(j,1) = m_t[i](1);
          J(j,2) = m_t[i](2);
        }
      }

      /**
      * @see ConstraintInterface.get_angular_jacobian_A()
      */
      void get_angular_jacobian_A(matrix_range & J) const
      {
        assert(this->m_bodyA || !"ContactPoint::get_angular_jacobian_A(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_angular_jacobian_A(): body B was null");

        assert(J.size1()==get_number_of_jacobian_rows() || !"ContactPoint::get_angular_jacobian_A(): incorrect dimension");
        assert(J.size2()==3                             || !"ContactPoint::get_angular_jacobian_A(): incorrect dimension");

        vector3_type tmp = cross(m_rA , m_n);
        J(0,0) = -tmp(0);
        J(0,1) = -tmp(1);
        J(0,2) = -tmp(2);
        for(size_t i=0;i<m_eta;++i)
        {
          size_t j = i+1;
          tmp = cross( m_rA , m_t[i] );
          J(j,0) = -tmp(0);
          J(j,1) = -tmp(1);
          J(j,2) = -tmp(2);
        }
      }

      /**
      * @see ConstraintInterface.get_angular_jacobian_B()
      */
      void get_angular_jacobian_B(matrix_range & J) const
      {
        assert(this->m_bodyA || !"ContactPoint::get_angular_jacobian_B(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_angular_jacobian_B(): body B was null");

        assert(J.size1()==get_number_of_jacobian_rows() || !"ContactPoint::get_angular_jacobian_B(): incorrect dimension");
        assert(J.size2()==3                             || !"ContactPoint::get_angular_jacobian_B(): incorrect dimension");

        vector3_type tmp = cross(m_rB , m_n);
        J(0,0) = tmp(0);
        J(0,1) = tmp(1);
        J(0,2) = tmp(2);
        for(size_t i=0;i<m_eta;++i)
        {
          size_t j = i+1;
          tmp = cross( m_rB , m_t[i] );
          J(j,0) = tmp(0);
          J(j,1) = tmp(1);
          J(j,2) = tmp(2);
        }
      }

      /**
      * @see ConstraintInterface.get_stabilization_term()
      */
      void get_stabilization_term(vector_range & b_error) const
      {
        using std::max;

        assert(b_error.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_stabilization_term(): incorrect dimension");

        if(!m_use_stabilization || m_distance>=0)
          b_error(0) = value_traits::zero();
        else
          b_error(0) = -(this->get_frames_per_second()*this->get_error_reduction_parameter()*m_distance);

        for(size_type i = 1;i<=m_eta;++i)
          b_error(i) = value_traits::zero();

        if(m_use_bounce)
        {
          //--- Hacking, introducing Newton's collision law to model non-zero normal restitution
          //--- The main idea is that (suggested by Baraff 89'):
          //---
          //---     u_n^+ >= - epsilon u_n^-
          //---
          real_type u_n_after = - m_material->normal_restitution() * m_un;
          b_error(0) += max(value_traits::zero(), u_n_after);
        }
        assert(is_number(b_error(0)) || !"ContactPoint::get_stabilization_term(): not a number encountered");
      }

      /**
      * @see ConstraintInterface.get_low_limits()
      */
      void get_low_limits(vector_range & lo) const
      {
        assert(m_material || !"ContactPoint::get_low_limits(): Material was null");
        assert(lo.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_low_limits(): incorrect dimension");

        lo(0) = value_traits::zero();

        // Todo Refactor into using iterators
        for(size_type i = 0;i<m_eta;++i)
          lo(i+1) = -m_material->get_friction_coefficient(i);
      }

      /**
      * @see ConstraintInterface.get_high_limits()
      */
      void get_high_limits(vector_range & hi) const
      {
        assert(m_material || !"ContactPoint::get_high_limits(): Material was null");
        assert(hi.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_high_limits(): incorrect dimension");

        hi(0) = OpenTissue::math::detail::highest<real_type>();

        // Todo Refactor into using iterators
        for(size_type i = 0;i<m_eta;++i)
          hi(i+1) = m_material->get_friction_coefficient(i);
      }

      /**
      * @see ConstraintInterface.getDependencyIndices()
      *
      */
      void get_dependency_indices(idx_vector_range & dep) const
      {
        assert(this->m_bodyA || !"ContactPoint::get_dependency_indices(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_dependency_indices(): body B was null");

        assert(dep.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_dependency_indices(): incorrect dimension");

        dep(0) = OpenTissue::math::detail::highest<size_type>();

        for(size_type i = 1;i<=m_eta;++i)
          dep(i) = this->get_jacobian_index();
      }

      /**
      * @see ConstraintInterface.getDepencyFactors()
      */
      void get_dependency_factors(vector_range & factors) const
      {
        assert(m_material || !"ContactPoint::get_dependency_factors(): Material was null");
        assert(this->m_bodyA || !"ContactPoint::get_dependency_factors(): body A was null");
        assert(this->m_bodyB || !"ContactPoint::get_dependency_factors(): body B was null");
        assert(factors.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_dependency_factors(): incorrect dimension");

        factors(0) = value_traits::zero();
        // Todo Refactor into using iterators
        for(size_type i = 0;i<m_eta;++i)
          factors(i+1) = m_material->get_friction_coefficient(i);
      }

      void set_regularization(vector_range const & gamma)
      {
        assert(false || !"ContactPoint::set_regularization(): This method should never be invoked directly, use Material::set_regularization instead");
      }

      void get_regularization(vector_range & gamma) const
      {
        assert(m_material || !"ContactPoint::get_regularization(): Material was null");
        assert(gamma.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_regularization(): incorrect dimension");      

        gamma(0) = m_material->get_normal_regularization();
        // Todo Refactor into using iterators
        for(size_t i=1;i<m_eta;++i)
          gamma(i) = value_traits::zero();
      }

      void set_solution(vector_range const & solution)
      {
        assert(solution.size()==get_number_of_jacobian_rows() || !"ContactPoint::set_solution(): incorrect dimension");

        math_policy::resize( m_solution, get_number_of_jacobian_rows());
        for(size_type i=0;i<get_number_of_jacobian_rows();++i)
          m_solution(i) = solution(i);
      }

      void get_solution(vector_range & solution) const
      {
        assert(solution.size()==get_number_of_jacobian_rows() || !"ContactPoint::get_solution(): incorrect dimension");

        if(m_solution.size()==0)
        {
          for(size_type i=0;i<get_number_of_jacobian_rows();++i)
            solution(i) = value_traits::zero();
        }
        else
        {
          for(size_type i=0;i<get_number_of_jacobian_rows();++i)
            solution(i) = m_solution(i);
        }
      }

      /**
      * @see ConstraintInterface.set_error_reduction_parameter()
      */
      void set_error_reduction_parameter(real_type const & erp)
      {
        //--- Overriding default behavior because contact points get
        //--- their erp-values from material properties.
        //---
        //--- Warning: Error reduction parameter should
        //--- not be set directly on contact point, use
        //--- materials for setting this
      }

      /**
      * @see ConstraintInterface.get_error_reduction_parameter()
      */
      real_type get_error_reduction_parameter() const
      {
        //--- Overriding default behavior because contact points get
        //--- their erp-values from material properties.
        //---
        if(this->use_erp())
          return m_material->get_error_reduction_parameter();
        return value_traits::one();
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_CONTACT_POINT_H
#endif
