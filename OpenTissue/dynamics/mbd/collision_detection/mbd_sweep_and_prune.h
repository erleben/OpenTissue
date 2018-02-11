#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SWEEP_AND_PRUNE_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SWEEP_AND_PRUNE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <list>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Interval IntervalEndpoint
    */
    template<typename types>
    class IntervalEndpoint
    {
    public:

      typedef typename types::math_policy::index_type  size_type;
      typedef typename types::math_policy::real_type   real_type;
      typedef typename types::body_type                body_type;

    public:

      IntervalEndpoint()
        : m_type(0)
        , m_body(0)
      {}

      void init(body_type * body, size_type const & type)
      {
        assert(body);
        assert(type==0 || type==1);
        this->m_body = body;
        this->m_type = type;
      };

    public:

      real_type       m_value;  ///< The actual value of the endpoint.
      body_type *   m_body;   ///< A pointer to the body.
      size_type m_type;   ///< If this is the starting end point then value is 0 if it is ending point then value is 1.

    };/* End class IntervalEndpoint */

    /**
    * Axis Aligned Bounding Box.
    */  
    template<typename types>
    class AABB
    {
    public:

      typedef typename types::math_policy::real_type      real_type;
      typedef typename types::math_policy::vector3_type    vector3_type;
      typedef typename types::math_policy::matrix3x3_type  matrix3x3_type;
      typedef typename types::body_type            body_type;

    public:

      /**
      * Default Constructor.
      */
      AABB(void):m_body(0){};

      /**
      * Update AABB Endpoints.
      * This method makes sure that the coordinate value of the
      * interval endpoints is updated, such that the AABB encloses
      * the current state of the corresponding configuration entity.
      *
      * @param body      A pointer to the body the AABB encloses.
      * @param envelope  The size of the collision envelope.
      */
      void update(body_type * body,real_type const & envelope)
      {
        if(!m_body)
        {
          m_body = body;
          m_beginX.init(body,0);
          m_beginY.init(body,0);
          m_beginZ.init(body,0);
          m_endX.init(body,1);
          m_endY.init(body,1);
          m_endZ.init(body,1);
        }
        assert(m_body);
        vector3_type pmin,pmax,r;
        matrix3x3_type R;
        body->get_position(r);
        body->get_orientation(R);
        body->compute_collision_aabb(r,R,pmin,pmax,envelope);
        m_beginX.m_value = pmin(0);
        m_beginY.m_value = pmin(1);
        m_beginZ.m_value = pmin(2);
        m_endX.m_value = pmax(0);
        m_endY.m_value = pmax(1);
        m_endZ.m_value = pmax(2);
      };

    public:

      body_type * m_body;                          ///< A pointer to the corresponding body this AABB encloses.

      IntervalEndpoint<types> m_beginX;
      IntervalEndpoint<types> m_beginY;
      IntervalEndpoint<types> m_beginZ;
      IntervalEndpoint<types> m_endX;
      IntervalEndpoint<types> m_endY;
      IntervalEndpoint<types> m_endZ;

    };/* End class AABB */


    /**
    * The Sweep N' Prune Broad Phase Collision Detection Algorithm.
    */
    template<typename types>
    class SweepNPrune
    {
    protected:

      typedef IntervalEndpoint<types> EndPoint;
      typedef std::list<EndPoint *> CoordinateAxis;
      typedef typename types::math_policy::real_type real_type;
      typedef typename types::configuration_type configuration_type;
      typedef typename types::contact_type contact_type;
      typedef typename types::body_type body_type;
      typedef typename types::edge_type edge_type;
      typedef typename types::edge_ptr_container edge_ptr_container;


      CoordinateAxis m_axisX;                  ///< X-Axe. The x-coordinates of AABB interval endpoints.
      CoordinateAxis m_axisY;                  ///< Y-Axe. The y-coordinates of AABB interval endpoints.
      CoordinateAxis m_axisZ;                  ///< Z-Axe. The z-coordinates of AABB interval endpoints.
      configuration_type * m_configuration;         ///< A pointer to the configuration that holds the configuration
      ///< that the broad phase collision detection algorithm is
      ///< intended to work on.
      edge_ptr_container m_reported;            ///< A list containing the currently reported overlaps.

    public:

      class node_traits
      {
      public:
        AABB<types> m_snp_aabb;  ///< An AABB Box enclosing the body.
      };

      class edge_traits
      {
      public:
        edge_traits():m_snp_axis_overlap_count(0),m_snp_overlap_reported(false){};
      public:
        short m_snp_axis_overlap_count;                              ///< Axis overlap counter.
        typename edge_ptr_container::iterator m_snp_reported_overlap;  ///< Iterator to any reported overlap.
        bool m_snp_overlap_reported;                                 ///< Boolean flag indicating whether a ovelap have been reported.
      };
      class constraint_traits {  };

    public:

      SweepNPrune()
        : m_configuration(0) 
      {}

    public:

      void clear()
      {
        m_axisX.clear();
        m_axisY.clear();
        m_axisZ.clear();
        m_reported.clear();
        this->m_configuration = 0;
      }

      void init(configuration_type & configuration)
      {
        clear();
        m_configuration = &configuration;
      }

      void add(body_type * body)
      {
        assert(m_configuration);
        m_axisX.push_back(&body->m_snp_aabb.m_beginX);
        m_axisX.push_back(&body->m_snp_aabb.m_endX);
        m_axisY.push_back(&body->m_snp_aabb.m_beginY);
        m_axisY.push_back(&body->m_snp_aabb.m_endY);
        m_axisZ.push_back(&body->m_snp_aabb.m_beginZ);
        m_axisZ.push_back(&body->m_snp_aabb.m_endZ);  
      }

      void remove(body_type * body)
      {
        assert(m_configuration);
        m_axisX.remove(&body->m_snp_aabb.m_beginX);
        m_axisX.remove(&body->m_snp_aabb.m_endX);
        m_axisY.remove(&body->m_snp_aabb.m_beginY);
        m_axisY.remove(&body->m_snp_aabb.m_endY);
        m_axisZ.remove(&body->m_snp_aabb.m_beginZ);
        m_axisZ.remove(&body->m_snp_aabb.m_endZ);  
        for(typename body_type::indirect_edge_iterator edge = body->edge_begin();edge!=body->edge_end();++edge)
        {
          if(edge->m_snp_overlap_reported)
          {
            m_reported.erase(edge->m_snp_reported_overlap);
            edge->m_snp_overlap_reported = false;
          }
        }
      }

      /**
      * Run SweepNPrune Algorithm.
      *
      *
      * @param edges   A pointer to a list which upon return contains
      *                all the detected overlaps, as pointers to the
      *                respective contact graph edges.
      *
      */
      void run(edge_ptr_container & edges)
      {
        assert(m_configuration);
        real_type envelope = m_configuration->get_collision_envelope();
        for(typename configuration_type::body_iterator body = m_configuration->body_begin();body!=m_configuration->body_end();++body)
        {
          body->m_snp_aabb.update(&(*body),envelope);
        }
        sort(m_axisX);
        sort(m_axisY);
        sort(m_axisZ);
        //--- KE 22-11-2002: NOTE 1 :
        //--- Observe that we copy the found overlaps into
        //--- the argument list. This might not be the most
        //--- efficient way, we could just as well return a
        //--- pointer to the member m_reported, however
        //--- this will not protect us from sideeffects, caused
        //--- by for instance the contact analysis module...
        edges.assign(m_reported.begin(),m_reported.end());
      }

    protected:

      /**
      * Coordinate Sorting Algorithm.
      * Temporal Coherence indicates that insertion sort is
      * cabaple of sorting the endpoints from the previous
      * iteration in expected linear time.
      *
      * This method encapsylates the inertion sort algorithm.
      *
      * @param axis  Coordinate Axis that should be sorted upon return.
      */
      void sort(CoordinateAxis & axis)
      {
        //--- KE 22-11-2002: It seems that the STL implementation
        //--- of list::sort also uses insertion sort, "their"
        //--- implementation is proberly faster than ours, however
        //--- we might need to add certain "update"-actions when we
        //--- swap an element into its right position, that is why
        //--- we have implemented the stuff our self....
        typename CoordinateAxis::iterator left;
        typename CoordinateAxis::iterator right;
        typename CoordinateAxis::iterator scan = axis.begin();
        left = scan;
        ++scan;//--- KE 22-11-2002: Note this algorithm will not work on an empty axe!!!!

        //--- Scan list from left to right
        for(;scan!=axis.end();)
        {
          right = scan;
          ++right;
          //--- Check if we encountered an element that was smaller
          //--- than its left neighbor
          if( isWrong(*left,*scan) )
          {
            //--- If so we contineously swap the element to the left
            //--- in the list until its left neighbor is no longer
            //--- bigger than itself
            typename CoordinateAxis::iterator _right = scan;
            typename CoordinateAxis::iterator _left  = left;
            do
            {
              //--- Before we do a swap we call this "interface" method, which
              //--- is responsible for updating any sort of internal data
              //--- structures we might want to use
              swapAction(*_left,*_right);
              std::iter_swap(_right,_left);
              _right = _left;
              --_left;
            }
            while(_right!=axis.begin()&&isWrong(*_left,*_right));
          }
          //--- Now we can pick up the overall left-to-right scan again
          left = scan;
          scan = right;
        }
      }

      /**
      * Wrongly Sorted Query Method.
      * This method determines whatever the two end
      * points appears in the proper order along a
      * coordinate axe.
      *
      * It is invoked by the method "sort(...)".
      *
      * @param left     A pointer to the endpoint that has the
      *                 left position in the list (head is assumed
      *                 to be left most and tail rightmost).
      * @param right    A pointer to the endpoint that has the
      *                 right position in the list.
      *
      * @return         If left and right should be swapped
      *                 then the return value it true otherwise
      *                 it is false.
      *
      */
      const bool isWrong(EndPoint * left,EndPoint * right)
      {
        //--- Values must be sorted in increasing order from left to right
        if(right->m_value<left->m_value)
          return true;
        //--- Endpoints must be sorted so ``begin '' comes before ``end''
        if(right->m_value==left->m_value)
        {
          if((right->m_type==0)&&(left->m_type==1))
            return true;
        }
        return false;
      }

      /**
      * Swap Action.
      * This method is supposed to be called by the insetion sort
      * algorithm whenever it swaps two endpoints.
      *
      * NOTE: The method is called before the actual swap!!!
      *
      * The method is assumed to perform all the appropriate
      * semantic actions, corresponding to the swap.
      *
      * The method is invoked by the method "sort(...)".
      *
      * @param left     A pointer to the endpoint that has the
      *                 left position in the list (head is assumed
      *                 to be left most and tail rightmost).
      * @param right    A pointer to the endpoint that has the
      *                 right position in the list.
      */
      void swapAction(EndPoint * left,EndPoint * right)
      {
        assert(m_configuration);
        edge_type * edge = m_configuration->get_edge(left->m_body,right->m_body);
        if(edge==0)
        {
          edge = m_configuration->add(left->m_body,right->m_body);
        }
        if((left->m_type==1) && (right->m_type==0))
        {      
          ++(edge->m_snp_axis_overlap_count);
          if(edge->m_snp_axis_overlap_count==3)
          {
            edge->m_snp_reported_overlap = m_reported.insert(m_reported.end(),edge);
            edge->m_snp_overlap_reported = true;
          }
        }
        if((left->m_type==0) && (right->m_type==1))
        {
          --(edge->m_snp_axis_overlap_count);
          if(edge->m_snp_axis_overlap_count==2)
          {
            if(edge->m_snp_overlap_reported)
            {
              m_reported.erase(edge->m_snp_reported_overlap);
              edge->m_snp_overlap_reported = false;
            }
          }
        }
      }

    public:

      const real_type getMin(CoordinateAxis & axis) {  return (axis.front())->m_value; };
      const real_type getMax(CoordinateAxis & axis) {  return (axis.back())->m_value;  };

      /**
      * Consistency Test Method.
      * This method should be used for debugging purpose only. It
      * runs through all the endpoints in the list and determines if
      * the list is sorted correctly.
      *
      * @param axis  Coordinate Axis to be tested.
      *
      * @return      If the list is consistent then the return value
      *              is true otherwise it is false.
      */
      const bool isConsistent(CoordinateAxis & axis)
      {
        typename CoordinateAxis::iterator scan = axis.begin();
        typename CoordinateAxis::iterator right = scan;
        ++right;
        for(;right!=axis.end();)
        {
          if((*scan)->m_value > (*right)->m_value)
            return false;
          scan = right;
          ++right;
        }
        return true;
      }

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SWEEP_AND_PRUNE_H
#endif
