#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_MESH_DEFAULT_TRAITS_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_MESH_DEFAULT_TRAITS_H
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

    /**
    * Default vertex traits for any kind of mesh type.
    * This template class takes one template argument. The template
    * argument is supposed to be a math types type binder. OpenTissue
    * provides a simple basic math type-binder in the math sub-library.
    * See OpenTissue::math::BasicMathTypes<real_type, size_type>
    */
    template<typename MT>
    class DefaultVertexTraits
    {
    public:

      typedef          MT                        math_types;
      typedef typename math_types::real_type     real_type;
      typedef typename math_types::vector3_type  vector3_type;

    public:

      vector3_type m_coord;
      vector3_type m_normal;
      real_type m_u;
      real_type m_v;
      vector3_type m_color;
      int m_tag;
    };

    class DefaultHalfEdgeTraits 
    {
    public:
      int m_tag;
    };

    class DefaultEdgeTraits 
    {
    public:
      int m_tag;
    };

    class DefaultFaceTraits
    {
    public:
      int m_tag;

    };

    class DefaultMeshTraits // empty by default 
    { };

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_MESH_DEFAULT_TRAITS_H
#endif
