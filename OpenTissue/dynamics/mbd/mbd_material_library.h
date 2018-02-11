#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_LIBRARY_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_LIBRARY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/containers_hash_map.h>
#include <OpenTissue/utility/utility_map_data_iterator.h>

#include <boost/iterator/indirect_iterator.hpp>

namespace OpenTissue
{
  namespace mbd
  {

    /** 
     * 
     Modelling Material Properties.

First one should create some unique positive material indices,

<pre>
  size_t ground_material_idx = 1;
  size_t wheel_material_idx  = 2;
</pre>

The material index value of zero has special meaning since it is
hardwired to the default material properties. The default
material properties is used between two objects having both
material index zero.

Notice that iIf no materials exist with material indices matching
those of the rigid bodies then the engine works like the material
indices of the rigid bodies were both set to zero.

After having defined some material indices one can now assign the
materials to the rigid bodies that one like. In our little
example we will setup an interesting material between the wheels
of vechicle and a ground surface. The material indices should be
assigned in a maner similar to this

<pre>
  m_ground.set_material_idx(ground_material_idx);
  ...
  m_configuration.add(&m_ground);

  for(size_t i ....)
  {
    m_wheels[i].set_material_idx(wheel_material_idx);
    ...
    m_configuration.add(&m_wheels[i]);
  }
</pre>

Notice that the material indices are simply assigned using the
method set_material_idx. Material indices should be assigned
prior to the first run of a simulator. Perferably before adding a
rigid body to a configuration.

Next one should create the material properties and assign their
values. First we need a container for the material properties,


<pre>
  material_type          wheel_ground_material;

  wheel_ground_material.set_material_indices(wheel_material_idx, ground_material_idx);
</pre>

Following this one can assign how many friction directions that should be used

<pre>
    wheel_ground_material.set_number_of_friction_directions(2);
</pre>

Next one should decide whether an isotropic or an anisotrpic
friction cone should be used. If one choice an isotropic model
then the friction coefficient can be set for all friction
directions by a single invokation of the method
set_friction_coefficient,

<pre>
  real_type my_friction_value = ...;
  wheel_ground_material.set_friction_coefficient( my_friction_value );
</pre>

However, for a wheel like object one would most likely choose an
anisotropic model, and the friction coefficients for each
friction direction must be specified individually like this,

<pre>
    wheel_ground_material.set_friction_coefficient(0,2.0);
    wheel_ground_material.set_friction_coefficient(1,5.0);
</pre>


By default the friction directions of the friction cone is
determined by the relative slinding direction of two rigid bodies
at a single contact point. This behavior can be turned off and
another scheme can be used for determining the friction
directions.

<pre>
    wheel_ground_material.set_use_sliding_direction(false);
</pre>

The technique of using the relative slidning directin takes
precedence over any other settings, thus default behavior can be
established simply by turning this setting on again.

For a tire-ground contact it makes sense to use a prefixed
friction direction determined by the forward/backward moving
direction of the wheels. First we will tell the material
properties to use a prefixed direction,

<pre>
    wheel_ground_material.set_use_prefixed_direction(true);
</pre>

Following this we have to specify how the prefixed direction
should be determined. In our case we want to have the direction
prefixed wrt. the local frame of the wheel body. This is done by
specifying the material index of the rigid body wrt. which the
prefixed direction should be set.

<pre>
    wheel_ground_material.set_prefixed_material_index( wheel_material_idx );
</pre>

If one uses values not equal to any of the two rigid
bodies (wheel and gound in the case) then the prefixed direction
is given wrt. the world coordinate system. Now we can specify the
prefixed direction as a local vector in the wheel model frame.

<pre>
    wheel_ground_material.set_prefixed_direction( vector3_type( 0.0, 0.0, 1.0 ) );
</pre>

This is advantages because we want the friction coefficient along
the rolling direction to be smaller than the friction direction
across the rolling direction of the wheel. This will prevent a
car model from skiding when making sharp turns.

Next one could specify how bouncy a wheel object should be on the
ground surface

<pre>
   wheel_ground_material.normal_restitution() = 0.1;
</pre>

Hereafter one could make the wheel-ground contacts a little soft
by adding a sligt amount of regularization, it has the effect of
adding a certain amount of damping.

<pre>
   wheel_ground_material.set_normal_regularization(0.1);
</pre>

In the same go one would most likely also specify the value of
the error reduction paramter,

<pre>
   wheel_ground_material.wheel_ground_material.set_error_reduction_parameter(0.1);
</pre>

This effect will correspond to adding a small spring to prevent
penetrations at wheel ground contacts. Overall the
error-reduction-parameter and the regularization add some
elasticity to the wheel-ground contacts.

As a finally step we can add the new material properties to our
material library, like this

<pre>
  m_library.add( wheel_ground_material );
</pre>

In general one should also setup the default material properties,
these properties are used whenever the material liberay can not
find a pre-set material properties for a pair of rigid bodies.
First we need to obtain access to the defualt material
properties,

<pre>
  material_type * default_material = data.m_library.default_material();
</pre>

If one has already used the material properties in a previously
configuration then one can clear all previously settings by
invoking the clear method.

<pre>
  default_material->clear();
</pre>

Next one simply set the values as one please, here we simply
choice a low friction bouncy type of material.

<pre>
  default_material->set_friction_coefficient(0.25);
  default_material->normal_restitution() = (0.15);
</pre>

     */
    template<typename mbd_types>
    class MaterialLibrary
    {
    public:

      typedef typename mbd_types::math_policy::real_type                  real_type;
      typedef typename mbd_types::math_policy::index_type                 index_type;
      typedef typename mbd_types::math_policy::index_type                 size_type;
      typedef typename mbd_types::material_type                           material_type;

    protected:

      typedef typename stdext::hash_map<index_type, material_type*>         material_ptr_lut_type;
      typedef OpenTissue::utility::map_data_iterator<typename material_ptr_lut_type::iterator>   material_ptr_lut_iterator;

    public:

      typedef boost::indirect_iterator<material_ptr_lut_iterator>           material_iterator;

    protected:

      material_ptr_lut_type  m_storage;   ///< Storage of material properties.
      material_type          m_default;   ///< Default material property

    public:

      MaterialLibrary()
      {
        add(m_default);
      }

      virtual ~MaterialLibrary()  {   clear();  }

    public:

      void add(material_type const & m)
      {
        assert(m_storage.find(m.hash_key())==m_storage.end() || !"MaterialLibrary::add(): Material already exist");
        m_storage.insert( std::pair<index_type, material_type*>(m.hash_key(), const_cast<material_type*>(&m) ) );
      }

      material_type * get(index_type A, index_type B)
      {
        typename material_ptr_lut_type::iterator mit = m_storage.find(material_type::hash_key(A,B));
        if(mit==m_storage.end())
          return 0;
        return mit->second;
      }

      void clear()
      {
        m_storage.clear();
        m_default.clear();
        add(m_default);
      }

      material_type * default_material() { return &m_default; }

      size_type size_materials() const { return m_storage.size(); }
      
      material_iterator material_begin() { return material_iterator( material_ptr_lut_iterator( m_storage.begin() ) );}
      material_iterator material_end()   { return material_iterator( material_ptr_lut_iterator( m_storage.end()   ) );}

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_MATERIAL_LIBRARY_H
#endif
