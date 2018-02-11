#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp>  // needed for boost::numeric_cast

namespace OpenTissue
{

  namespace mbd
  {

    /**
    * material_type Property Class.
    *
    *
    * Materials in Contact      Coefficient of Static Friction         Coefficient of Kinetic Friction
    * Wood on wood                            0.5                                     0.3
    * Waxed ski on snow                       0.1                                     0.05
    * Ice on ice                              0.1                                     0.03
    * Rubber on concrete (dry)                value_traits::one()                                     0.8
    * Rubber on concrete (wet)                0.7                                     0.5
    * Glass on glass                          0.94                                    0.4
    * Steel on aluminum                       0.61                                    0.47
    * Steel on steel (dry)                    0.7                                     0.6
    * Steel on steel (lubricated)             0.12                                    0.07
    * Teflon on steel                         0.04                                    0.04
    * Teflon on Teflon                        0.04                                    0.04
    * Synovial joints (in humans)             0.01                                    0.01
    *
    */
    template<typename mbd_types>
    class Material
    {
    public:

      typedef typename mbd_types::math_policy    math_policy;
      typedef typename math_policy::real_type      real_type;
      typedef typename math_policy::value_traits   value_traits;
      typedef typename math_policy::index_type     index_type;
      typedef typename math_policy::index_type     size_type;
      typedef typename math_policy::vector_type    vector_type;
      typedef typename math_policy::vector_range   vector_range;
      typedef typename math_policy::vector3_type   vector3_type;

    protected:

      index_type   m_material_idx_A;          ///< Index of material A.
      index_type   m_material_idx_B;          ///< Index of material B.

      vector_type  m_mu;                      ///< Coefficient of friction. The size of this vector is equal to the number of friction directions.
      real_type    m_e_n;                     ///< Coefficient of normal restitution.
      real_type    m_e_t;                     ///< Coefficient of tangential restitution.
      real_type    m_erp;                     ///< Error reduction parameter (must be in the range 0..1).
      real_type    m_gamma;                   ///< The regularization (damping) term to be used for the normal force. Also known as constraint force mixing.
      
      bool         m_use_sliding_direction;   ///< A boolean value indicating whether the relative sliding direction at a contact point should be used to pick the x-vector of the contact plane span. Default value is true.
      bool         m_use_prefixed_direction;  ///< A boolean value indicating whether the relative sliding direction at a contact point is set by a prefixed direction. The use slidning direction takes precedience over this setting.  Default value is false.

      vector3_type m_prefixed_direction;      ///< The value of the prefixed direction. The default setting is the zero-vector.
      index_type   m_prefixed_material_idx;   ///< The value of the prefixed material idx. Default value is zero. If the material index of a rigid body match this value then it means that the prefixed direction is specified wrt. the local frame of that rigid body.
    public:

      Material(){        clear();      }



      virtual ~Material(){  }

      bool operator==(Material const & m) const
      {
        if(m_material_idx_A == m.m_material_idx_A && m_material_idx_B==m.m_material_idx_B)
          return true;
        return false;
      }

    public:

      /**
       * This method clears all settings of the material properties.
       * Thus every setting is returned to its default value.
       */
      void clear()
      {
        m_material_idx_A = 0;
        m_material_idx_B = 0;
        m_e_n = value_traits::half();
        m_e_t = value_traits::zero();
        m_erp = boost::numeric_cast<real_type>(0.8);
        m_gamma = value_traits::zero();
        m_use_sliding_direction = true;
        m_use_prefixed_direction = false;
        m_prefixed_direction = vector3_type( value_traits::zero(), value_traits::zero(), value_traits::zero() );
        m_prefixed_material_idx  = 0;
        math_policy::resize(m_mu, 2);
        std::fill( m_mu.begin(), m_mu.end(), value_traits::half() );
      }

      void set_material_indices(index_type A, index_type B)
      {
        if(A<B)
        {
          this->m_material_idx_A = A;
          this->m_material_idx_B = B;
        }
        else
        {
          this->m_material_idx_A = B;
          this->m_material_idx_B = A;
        }
      }

      index_type hash_key() const {  return Material::hash_key(m_material_idx_A,m_material_idx_B); }

      /**
      * Get Hash Key.
      * This method is usefull if one want to get the hash key of a material property
      * that needs to be looked up in a hash map.
      *
      * @param material_idx_A
      * @param material_idx_B
      * @return
      */
      static index_type hash_key(index_type A, index_type B){ return (A < B) ? ((A<<16)|(B&0x0000FFFF)) : ((B<<16)|(A&0x0000FFFF)) ; }

    public:

      /**
       * Set Friction Coefficient.
       * This method is convenient for setting the value of
       * the isotropic friction coefficient. It means that
       * the same friction coefficient are used along every
       * friction direction.
       *
       * @param mu   The isotropic friction value.
       */
      void set_friction_coefficient(real_type const & mu)
      {
        assert(mu>value_traits::zero() || !"Material::set_friction_coefficient(): value must be positive");
        assert(m_mu.size()>0           || !"Material::set_friction_coefficient(): No friction directions exist");
        std::fill( m_mu.begin(), m_mu.end(), value_traits::half() );
      }

      /**
       * Set Friction Coefficient.
       * This method can be used for setting the friction
       * coefficient values of each indvidual friction
       * direction. The method is convenient for setting
       * up anisotropic friction.
       *
       * @param idx     The index of the friction direction.
       * @param mu      The friction coefficient value that should be
       *                set for the specified friction direction.
       */
      void set_friction_coefficient(size_t const & idx, real_type const & mu)
      {
        assert(mu>value_traits::zero() || !"Material::set_friction_coefficient(): value must be positive");
        assert(m_mu.size()>0           || !"Material::set_friction_coefficient(): No friction directions exist");
        assert(idx < m_mu.size()       || !"Material::set_friction_coefficient(): No such friction direction exist");
        this->m_mu(idx) = mu;
      }

      /**
       * Get value of Friction coefficient.
       * This methos is usefull when retrieving information
       * about the friction coefficients. It is for instance
       * used in the class ContactPoint.
       *
       * @param idx     The index of the friction direction.
       * @return    The friction coefficient value of the specified friction direction.
       */
      real_type const & get_friction_coefficient(size_t const & idx) const
      {
        assert(m_mu.size()>0           || !"Material::get_friction_coefficient(): No friction directions exist");
        assert(idx < m_mu.size()       || !"Material::get_friction_coefficient(): No such friction direction exist");
        return this->m_mu(idx);
      }

      /**
       * Get Isotropic Friction Coefficient.
       * Not all physical models or numerical methods support
       * anisotropic friction. This method is supplemented to
       * support these methods/models. The value returned is
       * simply the friction coefficient of the first friction
       ^direction. If no such friction direction exist then the value zero is returned instead.
       *
       *
       *  @return   A friction coefficient value that is representive
       *            of an isotropic friction model.
       */
      real_type get_isotropic_friction_coefficient() const
      {
        if( m_mu.empty() )
          return value_traits::zero();
        return m_mu(0);
      }

      /**
       * Set Number of Friction Directions.
       * If the number of directions are increased then
       * the value of the friction coefficient of the last
       * old direction is copied onto the new directions as
       * the default value.
       *
       * @param eta     The new number of friction directions to be used.
       */
      void set_number_of_friction_directions(size_t const & eta)
      {
        using std::min;

        // Make local copy of old friction coefficient values
        size_t old_eta = m_mu.size();
        vector_type tmp;
        math_policy::resize(tmp, old_eta );
        tmp = m_mu;

        // Resize the friction coefficient value vector to match new number of friction directions.
        math_policy::resize( m_mu, eta );

        size_t N = min(old_eta,eta);
        if(N>0)
        {
          size_t i = 0;
          // Copy the old friction coefficient values into the resized vector
          for(;i<N;++i)
            m_mu(i) = tmp(i);
          // Assign the friction coefficient value of the last old direction to all the newly created directions.
          for(;i<eta;++i)
            m_mu(i) = tmp(N-1);
        }
      }

      /**
       * Get the number of friction directions.
       *
       * @return    The number of friction directions.
       */
      size_t get_number_of_friction_directions() const 
      {
        return m_mu.size();
      }

      real_type const & normal_restitution()            const { return this->m_e_n; }
      real_type       & normal_restitution()                  { return this->m_e_n; }

      real_type const & tangential_restitution()        const { return this->m_e_t; }
      real_type       & tangential_restitution()              { return this->m_e_t; }

      real_type const & get_error_reduction_parameter() const { return m_erp;       }
      void set_error_reduction_parameter(real_type const & erp)
      {
        assert(erp>=0 || !"Material::set_error_reduction_parameter(): value must be non-negative");
        assert(erp<=1 || !"Material::set_error_reduction_parameter(): value must be less than or equal 1");
        this->m_erp = erp;
      }

      void set_normal_regularization(real_type const & gamma)
      {
        assert(is_number(gamma)              || !"Material::set_regularization(): not a number was encountered");
        assert(gamma <= value_traits::one()  || !"Material::set_regularization(): gamma must be less than or equal to one");
        assert(gamma >= value_traits::zero() || !"Material::set_regularization(): gamma must be non-negative");
        m_gamma = gamma;
      }

      real_type const & get_normal_regularization() const  {   return m_gamma;  }

      /**
       * Get value of use sliding direction.
       *
       * @return  True if the relative sliding direction at a contact point should be used to pick the x-vector of the contact plane span. Default value is true.
       */
      bool get_use_sliding_direction() const { return m_use_sliding_direction; }

      /**
       * Set value of use sliding direction.
       *
       * @param value     A boolean value indicating whether the
       *                  relative sliding direction at a contact
       *                  point should be used to pick the x-vector
       *                  of the contact plane span.
       */
      void set_use_sliding_direction(bool const & value) { m_use_sliding_direction = value; }

      /**
       * Get value of ``use prefixed direction''.
       * The use slidning direction takes precedience over this setting.
       *
       * @return  True if the relative sliding direction at a
       *          contact point is set by a prefixed direction.
       *          Default value is false.
       */
      bool get_use_prefixed_direction() const { return m_use_prefixed_direction; }

      /**
       * Set value of ``use prefixed direction''.
       *
       * @param value     A boolean value indicating whether
       *                  the relative sliding direction at a
       *                  contact point is set by a prefixed direction.
       */
      void set_use_prefixed_direction(bool const & value) { m_use_prefixed_direction = value; }

      /**
       * Get Prefixed Direction.
       *
       * @return    The prefixed direction. If this has not
       *            been set then the default value is the
       *            zero vector.  If the material index of
       *            a rigid body match this value then it
       *            means that the prefixed direction is
       *            specified wrt. the local frame of that
       *            rigid body.
       */
      vector3_type const & get_prefixed_direction() const {  return m_prefixed_direction; }

      /**
       * Set Prefixed Direction.
       *
       * @param dir    The new prefixed direction. If the material index
       *               of a rigid body match this value then it means
       *               that the prefixed direction is specified wrt.
       *               the local frame of that rigid body.
       */
      void set_prefixed_direction(vector3_type const & dir)  {  m_prefixed_direction = unit( dir ); }

      /**
       * Set prefixed material index.
       *
       * @param idx   The new index value with respec to to which
       *              the prefixed friction direction has been specified.
       *              If the material index of a rigid body match this value
       *              then it means that the prefixed direction is specified
       *              wrt. the local frame of that rigid body.
       */
      void set_prefixed_material_index(index_type const & idx)
      {
        m_prefixed_material_idx = idx;
      }


      /**
       * Get fixed material index.
       *
       * @return    The index value with respec to to which
       *            the prefixed friction direction has been
       *            specified.  If the material index of a rigid
       *            body match this value then it means that the
       *            prefixed direction is specified wrt. the local
       *            frame of that rigid body. If the prefixed material
       *            index does not match any of the material indices
       *            of the rigid bodies then it implies that the prefixed
       *            direction is specified in the world coordinate frame.
       */
      index_type get_prefixed_material_index() const
      {
        return m_prefixed_material_idx;
      }
    
    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_H
#endif
