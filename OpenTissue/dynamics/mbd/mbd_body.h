#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_BODY_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_BODY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/mbd_update_inertia_tensor.h>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types  >
    class Body
      : public mbd_types::identifier_type
      , public mbd_types::node_traits
    {
    public:

      typedef typename mbd_types::math_policy                   math_policy;
      typedef typename mbd_types::math_policy::real_type        real_type;
      typedef typename mbd_types::math_policy::vector3_type     vector3_type;
      typedef typename mbd_types::math_policy::matrix3x3_type   matrix3x3_type;
      typedef typename mbd_types::math_policy::quaternion_type  quaternion_type;
      typedef typename mbd_types::math_policy::size_type        size_type;
      typedef typename mbd_types::math_policy::value_traits     value_traits;
      typedef typename mbd_types::node_traits                   node_traits;

    protected:

      typedef typename mbd_types::geometry_type                 geometry_type;
      typedef typename mbd_types::body_type                     body_type;
      typedef typename mbd_types::joint_type                    joint_type;
      typedef typename mbd_types::edge_type                     edge_type;
      typedef typename mbd_types::force_type                    force_type;
      typedef typename mbd_types::scripted_motion_type          scripted_motion_type;
      typedef typename mbd_types::joint_ptr_container           joint_ptr_container;
      typedef typename mbd_types::edge_ptr_container            edge_ptr_container;
      typedef typename mbd_types::force_ptr_container           force_ptr_container;

      typedef typename edge_ptr_container::iterator         edge_ptr_iterator;
      typedef typename edge_ptr_container::const_iterator   const_edge_ptr_iterator;
      typedef typename joint_ptr_container::iterator        joint_ptr_iterator;
      typedef typename joint_ptr_container::const_iterator  const_joint_ptr_iterator;

    public:

      joint_ptr_container    m_joints;           ///< A list containing pointers to all joints this body is part of.
      edge_ptr_container     m_edges;            ///< A list containing pointers to all incident edges to this body. That is all body pairs this body is part of.
      size_type              m_tag;              ///< Tag, can be used for anything, for instance traversing bodies...

    protected:

      force_ptr_container    m_forces;           ///< A list of pointers to all external forces influencing the rigid  body.
      scripted_motion_type * m_scripted_motion;  ///< A pointer to a scripted motion, if set this body is automatically scripted if null the body is either fixed or free moving.

    public:

      typedef typename mbd_types::indirect_joint_iterator          indirect_joint_iterator;
      typedef typename mbd_types::const_indirect_joint_iterator    const_indirect_joint_iterator;
      typedef typename mbd_types::indirect_edge_iterator           indirect_edge_iterator;
      typedef typename mbd_types::indirect_force_iterator          indirect_force_iterator;
      typedef typename mbd_types::const_indirect_force_iterator    const_indirect_force_iterator;

      indirect_joint_iterator         joint_begin()       { return indirect_joint_iterator(m_joints.begin());        }
      indirect_joint_iterator         joint_end()         { return indirect_joint_iterator(m_joints.end());          }
      const_indirect_joint_iterator   joint_begin() const { return const_indirect_joint_iterator(m_joints.begin());  }
      const_indirect_joint_iterator   joint_end()   const { return const_indirect_joint_iterator(m_joints.end());    }
      indirect_edge_iterator          edge_begin()        { return indirect_edge_iterator(m_edges.begin()); }
      indirect_edge_iterator          edge_end()          { return indirect_edge_iterator(m_edges.end());   }
      indirect_force_iterator         force_begin()       { return indirect_force_iterator(m_forces.begin());        }
      indirect_force_iterator         force_end()         { return indirect_force_iterator(m_forces.end());          }
      const_indirect_force_iterator   force_begin() const { return const_indirect_force_iterator(m_forces.begin());  }
      const_indirect_force_iterator   force_end()   const { return const_indirect_force_iterator(m_forces.end());    }

    protected:

      size_type m_material_idx;             ///< Unique index identifying the material properties  of the node.
      bool      m_active;                   ///< Boolean flag indicating whatever
      ///< this node is active in the
      ///< configuration.
      bool      m_sleepy;                   ///< Boolean flag indicating wheter the body is sleepy or not.
      bool      m_fixed;                    ///< Boolean flag indicating if the body is fixed or not.
      bool      m_scripted;                 ///< Boolean flag indicating if the body is scripted or not.
      bool      m_finite_rotation_update;   ///< Finite rotation update flag. To be used during position update.

      vector3_type    m_r_axis;             ///< Finite rotation axis, can be used during position update.
      vector3_type    m_r;                  ///<  The  current position of the node in WCS.
      quaternion_type m_Q;                  ///<  The  current orientation of the node in WCS.
      vector3_type    m_P;                  ///< Linear momentum on CM in WCS.
      vector3_type    m_L;                  ///< Angular momentum wrt. CM in WCS.
      vector3_type    m_V;                  ///< Linear velocity of CM in WCS .
      vector3_type    m_W;                  ///< Angular velocity around CM in WCS .
      matrix3x3_type  m_invI_WCS;           ///< Inverse Inertia tensor wrt CM in WCS.
      matrix3x3_type  m_invI_BF;            ///< Inverse Inertia tensor wrt CM in BF.
      matrix3x3_type  m_I_WCS;              ///< Inertia tensor wrt CM in WCS.
      matrix3x3_type  m_I_BF;               ///< Inertia tensor wrt CM in BF.
      real_type       m_inv_mass;           ///< Inverse mass.
      real_type       m_mass;               ///< mass.
      matrix3x3_type  m_R;                  ///< Orientation of BF in WCS (as a Matrix).

      geometry_type * m_geometry;           ///< The geometry of the rigid body.

    public:

      Body(){ this->clear(); }

      ~Body()
      {
        m_edges.clear();
        m_forces.clear();
      }


      Body (Body const & body)
      {
        assert(body.m_edges.empty() || !"Body(body): Could not make a copy of a body already in a configuration! It has live edges!");

        this->m_joints          = body.m_joints;
        this->m_edges           = body.m_edges;
        this->m_tag             = body.m_tag;
        this->m_forces          = body.m_forces;
        this->m_scripted_motion = body.m_scripted_motion;

        this->m_material_idx           = body.m_material_idx;
        this->m_active                 = body.m_active;
        this->m_sleepy                 = body.m_sleepy;
        this->m_fixed                  = body.m_fixed;
        this->m_scripted               = body.m_scripted;
        this->m_finite_rotation_update = body.m_finite_rotation_update;

        this->m_r_axis   = body.m_r_axis;
        this->m_r        = body.m_r;
        this->m_Q        = body.m_Q;
        this->m_P        = body.m_P;
        this->m_L        = body.m_L;
        this->m_V        = body.m_V;
        this->m_W        = body.m_W;
        this->m_invI_WCS = body.m_invI_WCS;
        this->m_invI_BF  = body.m_invI_BF;
        this->m_I_WCS    = body.m_I_WCS;
        this->m_I_BF     = body.m_I_BF;
        this->m_inv_mass = body.m_inv_mass;
        this->m_mass     = body.m_mass;
        this->m_R        = body.m_R;

        this->m_geometry = body.m_geometry;
      }

    public:

      void clear()
      {
        m_forces.clear();

        m_geometry = 0;

        m_scripted_motion = 0;
        m_scripted = false;

        m_active = true;
        m_sleepy = false;
        m_material_idx = 0;
        m_r_axis.clear();
        m_finite_rotation_update = false;
        m_fixed = false;
        m_tag = 0;

        m_r.clear();
        m_Q.identity();
        m_R = OpenTissue::math::diag(value_traits::one());
        m_I_WCS = OpenTissue::math::diag(value_traits::one());
        m_I_BF = OpenTissue::math::diag(value_traits::one());
        m_invI_WCS = OpenTissue::math::diag(value_traits::one());
        m_invI_BF = OpenTissue::math::diag(value_traits::one());
        m_inv_mass = value_traits::one();
        m_mass = value_traits::one();
        m_V.clear();
        m_W.clear();
        m_P.clear();
        m_L.clear();

        if(!m_edges.empty())
        {
          std::cout << "Body::clear(): WARNING clear was invoked on a body that still got edges?" << std::endl;
        }
      }

      /**
      * Compute External Forces and Torques.
      *
      * @param F  upon return this parameter holds the total external force acting on the center of mass of the body.
      * @param T  upon return this parameter holds the total external torque acting around the center of mass of the body.
      *
      */
      void compute_forces_and_torques(vector3_type & F,vector3_type & T)const
      {
        //--- Clear force accumulators
        F.clear();
        T.clear();

        vector3_type f,t;
        //--- Compute effects of all external forces
        for(const_indirect_force_iterator force = force_begin();force!=force_end();++force)
        {
          force->compute(*this,f,t);
          F += f;
          T += t;
        }
      }

      /**
      * Attach force_type Type.
      * Notice that forces are actually more like a type, than a single
      * instance of force, thus gravity can for instance be shared by
      * all bodies in a configuration.
      *
      * @param force   A pointer to the force that should be attached to this body. There is no test for multiple attachments.
      */
      void attach(force_type * force)
      {
        assert(force || !"Body::detach(): force_type pointer was null");
        if  ( find( m_forces.begin( ), m_forces.end( ), force ) == m_forces.end( ) )
        {
          m_forces.push_back(force);
        }
        else
        {
          std::cout << "Body::attach(): Sorry, you already added that force to this body!" << std::endl;
        }
      }

      /**
      * Detach force_type Type.
      *
      * @param force A pointer to the force type that should be removed.
      */
      void detach(force_type * force)
      {
        assert(force || !"Body::detach(): force_type pointer was null");

        if  ( find( m_forces.begin( ), m_forces.end( ), force ) == m_forces.end( ) )
        {
          std::cout << "Body::detach(): Sorry, the force wer not attached to this body!" << std::endl;
        }
        else
        {
          m_forces.remove(force);
        }

      }

      /**
      * Set Position.
      *
      * @param r   The new position of the center of mass of this body.
      */
      void set_position(vector3_type const & r)    {      m_r = r;    }

      /**
      * Set Orientation.
      * Note that this method will update the world coordinate system inertia tensor.
      *
      * @param Q   The new orientation, that is the rotation of the principal axes wrt. the world coordinate frame.
      */
      void set_orientation(quaternion_type const & Q)
      {
        m_Q = unit(Q);
        m_R = Q;

        mbd::update_inertia_tensor(m_R,m_I_BF,m_I_WCS);
        mbd::update_inertia_tensor(m_R,m_invI_BF,m_invI_WCS);
      }

      /**
      * Set Velocity.
      * This method implicitly effects the linear momemtum.
      *
      * @param V   The new linear velocity of the center of mass wrt. the world coordinate system.
      */
      void set_velocity(vector3_type const & V)
      {
        if(m_fixed)
          return;
        m_V = V;
        m_P = V *m_mass;
      }

      /**
      * Set Angular Velocity.
      * This method implicitly effects the angular momemtum.
      *
      * @param W    The angular velocity around the center of mass wrt. the world coordinate system.
      */
      void set_spin(vector3_type const & W)
      {
        if(m_fixed)
          return;
        m_W = W;
        m_L = m_I_WCS * m_W;
      }

      /**
      * Set Total body_type Mass.
      * Automatically computes the inverse mass, and implicitly effects the linear momentum.
      *
      * @param mass        The new total mass of the body, must be positive.
      */
      void set_mass(real_type const & mass)
      {
        assert(mass>value_traits::zero() || !"Body::set_mass(): mass must be positive");
        m_mass = mass;
        m_inv_mass = value_traits::one() / mass;
        m_P = m_V*m_mass;
      }

      /**
      * Set body_type Frame Inertia Tensor
      * The world frame inertia tensors are automatically updated, also the angular momentum is affected.
      *
      *
      * @param I_BF   The new body frame inertia tensor.
      */
      void set_inertia_bf(matrix3x3_type const & I_BF)
      {
        m_I_BF = I_BF;
        m_invI_BF = inverse(I_BF);

        mbd::update_inertia_tensor(m_R,m_I_BF,m_I_WCS);
        mbd::update_inertia_tensor(m_R,m_invI_BF,m_invI_WCS);

        m_L = m_I_WCS * m_W;
      }

      /**
      * Get Position.
      *
      * @param r    Upon return this argument holds the value of the
      *             position of the center of mass wrt. the world
      *             coordinate system.
      */
      void get_position(vector3_type & r) const { r = m_r; }

      /**
      * Get Orientation.
      *
      * @param Q   Upon return this argument holds the value of the
      *            orientation of the principal axes wrt. the world
      *            coordinate system representated as a quaterion.
      */
      void get_orientation(quaternion_type & Q) const
      {
        Q = m_Q;
      }

      /**
      * Get Orientation.
      *
      * @param R   Upon return this argument holds the value of the
      *            orientation of the principal axes wrt. the world
      *            coordinate system representated as a matrix.
      */
      void get_orientation(matrix3x3_type & R) const
      {
        R = m_R;
      }

      /**
      * Get Velocity.
      *
      * @param V   Upon return this argument holds the value of the linear velocity
      *            of the center of mass wrt. the world coordinate system.
      */
      void get_velocity(vector3_type & V) const
      {
        if(m_fixed)
          V.clear();
        else
          V = m_V;
      }

      /**
      * Get Spin
      *
      * @param W  Upon return this argument holds the value of the angular velocity
      *           around the center of mass wrt. the world coordinate system.
      */
      void get_spin(vector3_type & W) const
      {
        if(m_fixed)
          W.clear();
        else
          W = m_W;
      }

      /**
      * Get Inverse total Mass.
      *
      * @return    The value of the total inverse mass.
      */
      real_type get_inverse_mass() const { return (m_fixed || m_scripted) ? value_traits::zero() : m_inv_mass; }

      /**
      * Get the total mass.
      *
      * @return    The value of the total mass.
      */
      real_type get_mass() const { return (m_fixed || m_scripted) ? value_traits::infinity() : m_mass;  }

      /**
      * Get Inverse body_type Frame Inertia Tensor.
      *
      * @param invI_BF   Upon return this argument holds the value of the inverse body frame inertia tensor.
      */
      void get_inverse_inertia_bf(matrix3x3_type & invI_BF) const
      {
        if(m_fixed || m_scripted)
          invI_BF.clear();
        else
          invI_BF = m_invI_BF;
      }

      /**
      * Get Inverse World Frame Inertia Tensor.
      *
      * @param invI_WCS    Upon return this argument holds the value of the inverse world frame inertia tensor.
      */
      void get_inverse_inertia_wcs(matrix3x3_type & invI_WCS) const
      {
        if(m_fixed || m_scripted)
          invI_WCS.clear();
        else
          invI_WCS = m_invI_WCS;
      }

      /**
      *  Get the world frame inertia tensor.
      *
      * @param invI_WCS   Upon return this argument holds the value of the world frame inertia tensor.
      */
      void get_inertia_wcs(matrix3x3_type & I_WCS) const
      {
        if(m_fixed || m_scripted)
        {
          I_WCS = OpenTissue::math::diag( value_traits::infinity() );
        }
        else
          I_WCS = m_I_WCS;
      }

      /**
      * Retrieve Active Status.
      *
      * @return   If body is active then the return value
      *           is true otherwise it is false.
      */
      bool is_active() const { return m_active;  }

      /**
      * Set Active Status.
      *
      * @param active      The new active status value.
      */
      void set_active(bool const & active) { this->m_active = active; }

      /**
      * Retrieve Sleepy State.
      *
      * @return      If body is sleepy i.e. settling down to rest then
      *              the return value is true otherwise it is false.
      */
      bool is_sleepy() const {  return m_sleepy;  }

      /**
      * Set Sleepy Status.
      *
      * @param sleepy    The new sleepy status of the body.
      */
      void set_sleepy(bool const & sleepy){ this->m_sleepy = sleepy; }

      /**
      * Set material_type Index.
      * material_type properties are defined between two materials, a material
      * is uniquely identified by an index value. So if you want a material
      * like iron dedicate a unique index to mean ``iron'', if you have another
      * object made of say rubber, it will have another unique index. If the
      * iron and rubber objects come into contact, their material indices are
      * used to look up the material properties between iron and rubber.
      *
      * By default all bodies get material index zero, upon creation.
      *
      * @param idx  The new material index of the body.
      */
      void set_material_idx(size_type const & idx){  m_material_idx = idx;  }

      /**
      * Get material_type Index
      *
      * @return      The material index value of the body.
      */
      size_type get_material_idx() const { return m_material_idx; }

      /**
      * Finite Rotation Update Query.
      *
      * @return   If this body should be updated with a finite rotation
      *           update, then the return value is true otherwise it is
      *           false.
      */
      bool has_finite_rotation_update()const { return m_finite_rotation_update; }

      /**
      * Set Finite Rotation Update.
      *
      * @param active    Boolean flag indicating whetever finite rotation update is active or not.
      */
      void set_finite_rotation_update(bool const & active) { m_finite_rotation_update = active; }

      /**
      * Has Finite Rotation Axis Query.
      *
      * @return   If a finite rotation axis have been set then the
      *           return value is true otherwise it is false.
      */
      bool has_finite_rotation_axis() const {  return (m_r_axis(0)||m_r_axis(1)||m_r_axis(2));  }

      /**
      * Retrieve Finite Rotation Axis.
      *
      * param r_axis   Upon return this parameter contains the finite rotaiton axis.
      */
      void get_finite_rotation_axis(vector3_type & r_axis) const    {      r_axis = m_r_axis;    }

      /**
      * Set Finite Rotation Axis.
      * Note setting the rotation axis implies that finite rotation
      * update state is set to be on.
      *
      * @param r_axis    The new finite rotation axis to be used. If it
      *                  is zero then the finite rotation axis will be
      *                  turned off.
      */
      void set_finite_rotation_axis(vector3_type const & r_axis)
      {
        m_r_axis.clear();
        if(r_axis(0) || r_axis(1) ||r_axis(2) )
        {
          m_finite_rotation_update = true;
          m_r_axis = unit(r_axis);
        }
      }


      /**
      * Retrieve Scripted Status.
      *
      * @return    If body is a scripted body then the return
      *            value is true otherwise it is false.
      */
      bool is_scripted() const { return m_scripted; }

      /**
      * Set Scripted Motion.
      *
      * @param motion   A pointer to the scripted motion that should be used for the
      *                 scripted body. If null the body is set to be non-scripted. Thus
      *                 the end-user can toggle a rigid body to be scripted and
      *                 non-scripted.
      */
      void set_scripted_motion(scripted_motion_type * motion)
      {
        m_scripted_motion = motion;
        if(motion)
          m_scripted = true;
        else
          m_scripted = false;
      }

      /**
      * Compute Scripted Motion.
      * If body is scripted, then this method will compute the state
      * of the scripted motion at the specified time and update the
      * body state to the newly computed state.
      *
      * @param time   The time at which the scripted motion should be computed.
      */
      void compute_scripted_motion(real_type const & time)
      {
        if(m_scripted_motion)
        {
          vector3_type r,v,w;
          quaternion_type q;
          m_scripted_motion->compute((*this),time,r,q,v,w);
          set_position(r);
          set_orientation(q);
          set_velocity(v);
          set_spin(w);
        }
      }

      /**
      * Retrieve Fixed Status.
      *
      * @return    If body is a fixed body then the return
      *            value is true otherwise it is false.
      */
      bool is_fixed() const {  return m_fixed; }



      /**
      * Set fixed body.
      *
      * @param fixed        Boolean value indicating whetever the body is fixed or not.
      */
      void set_fixed(bool const & fixed) { this->m_fixed = fixed; }

      /**
      * Retrieve Total Kinetic Energy of body_type.
      * Note this is re-evaluated on every invocation.
      *
      * @return      The value of the kinetic energy.
      */
      real_type compute_kinetic_energy() const
      {
        real_type Ekin = value_traits::zero();
        if(!m_fixed)
          Ekin = (( (m_V*m_V) /m_inv_mass) + m_W*m_L)/value_traits::two();
        return Ekin;
      }

      /**
      * Retrieve joint_type Status.
      *
      * @return   If body is part of any joints then the return value is true otherwise it is false.
      */
      bool has_joints() const { return (!m_joints.empty()); }

      /**
      * Retrieve Active joint_type Status.
      *
      * @return   If body is part of any active joints then the return value is true otherwise it is false.
      */
      bool has_active_joints() const
      {
        for(const_indirect_joint_iterator joint = joint_begin();joint!=joint_end();++joint)
        {
          if(joint->is_active())
            return true;
        }
        return false;
      }

      /**
      * Retrieve joint_type Connection Status.
      *
      * @param   body   A pointer to another body.
      * @return         If body has a joint to the specified body
      *                 then the return value is true otherwise it is false.
      */
      bool has_joint_to(body_type const * body) const
      {
        assert(body       || !"Body::has_joint_to(): body was null");
        assert(this!=body || !"Body::has_joint_to(): body was the same as this body");
        for(const_indirect_joint_iterator joint = joint_begin();joint!=joint_end();++joint)
        {
          if(joint->get_socket_A()->get_body()==body || joint->get_socket_B()->get_body()==body)
            return true;
        }
        return false;
      }

      /**
      * Set Geometry.
      *
      * @param geometry     A pointer to the geometry type of this rigid body.
      */
      void set_geometry( geometry_type * geometry )
      {
        assert( geometry || !"Body::set_geometry(): geometry was null?");

        m_geometry = geometry;
      }

      /**
      * Get Geometry Type.
      *
      * @return          A pointer to the base interface of geometry types.
      */
      geometry_type * get_geometry() const { return m_geometry; }


      /**
      * Compute AABB box that encloses geometry of Body.
      *
      * @param r           The body position in WCS.
      * @param R           The body orientation in WCS.
      * @param min_coord   Upon return holds the minimum corner of the AABB.
      * @param max_coord   Upon return holds the maximum corner of the AABB.
      * @param envelope    The size of the collision envelope.
      */
      void compute_collision_aabb(
        vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        , real_type const & envelope
        ) const
      {
        assert(m_geometry || !"Body::compute_collision_aabb(): Geometry was null!");

        //--- Initialize to inifinite AABB
        min_coord(0) = OpenTissue::math::detail::lowest<real_type>();
        min_coord(1) = OpenTissue::math::detail::lowest<real_type>();
        min_coord(2) = OpenTissue::math::detail::lowest<real_type>();
        max_coord(0) = OpenTissue::math::detail::highest<real_type>();
        max_coord(1) = OpenTissue::math::detail::highest<real_type>();
        max_coord(2) = OpenTissue::math::detail::highest<real_type>();

        m_geometry->compute_collision_aabb( r, R, min_coord, max_coord );

        // Enlarge by envelope!
        min_coord(0) -= envelope;
        min_coord(1) -= envelope;
        min_coord(2) -= envelope;
        max_coord(0) += envelope;
        max_coord(1) += envelope;
        max_coord(2) += envelope;
      }


    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_BODY_H
#endif
