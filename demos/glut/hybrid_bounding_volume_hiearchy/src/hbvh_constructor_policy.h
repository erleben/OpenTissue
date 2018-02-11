#ifndef HBVH_CONSTRUCTOR_POLICY_H
#define HBVH_CONSTRUCTOR_POLICY_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/core/geometry/geometry_hybrid.h>
#include <OpenTissue/core/geometry/geometry_hybrid_fit.h>


template<typename bvh_type>
class HBVHConstructorPolicy 
  : public OpenTissue::bvh::DefaultPriorityBottomUpPolicy<bvh_type>
{
public:

  typedef OpenTissue::bvh::BVHGraph<bvh_type>       graph_type;
  typedef typename graph_type::node_ptr_type   node_ptr_type;
  typedef typename graph_type::edge_ptr_type   edge_ptr_type;
  typedef typename graph_type::edge_iterator   edge_iterator;
  typedef typename bvh_type::volume_type       volume_type;
  typedef typename bvh_type::geometry_type     geometry_type;

  typedef typename volume_type::math_types     math_types;
  typedef typename graph_type::real_type       real_type;


protected:

  const unsigned int degree( void ) const {  return 3; }

  real_type priority ( edge_ptr_type edge )
  {
    node_ptr_type nA = edge->A();
    node_ptr_type nB = edge->B();

    std::vector< OpenTissue::geometry::VolumeShape<math_types> * > volumes;
    volumes.push_back( &nA->volume() );
    volumes.push_back( &nB->volume() );

    volume_type C;
    OpenTissue::geometry::hybrid_volume_fit(volumes.begin(),volumes.end(),C);

    real_type value = C.volume() - ( nA->volume().volume() + nB->volume().volume() );
    std::cout << "HBVHConstructorPolicy::priority(): value = " << value << std::endl;
    return value;
  }

public:

  template<typename geometry_iterator,typename volume_iterator>
  volume_type fit(geometry_iterator /*g0*/, geometry_iterator /*g1*/, volume_iterator v0, volume_iterator v1)
  {
    volume_type hybrid;
    OpenTissue::geometry::hybrid_volume_fit(v0,v1, hybrid);
    return hybrid;
  }

};


// HBVH_CONSTRUCTOR_POLICY_H
#endif
