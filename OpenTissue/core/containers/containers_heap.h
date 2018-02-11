#ifndef	OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HEAP_H
#define	OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HEAP_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <list>
#include <map>
#include <cassert>
#include <stdexcept>

namespace OpenTissue
{
  namespace containers
  {

    /**
    * Heap Utility Class.
    *
    * NOTE: This heap keeps priorities in descending order. That means
    * that the element with the largest priority is on top of the
    * heap!!!
    *
    * Often a priority heap is needed with the following properties:
    *
    *  1) Each priority value is coupled to some feature/element from another
    *     data structure. As an example one may store edges of a polygonal mesh
    *     in a priority queue. The priorites indicate the benefit of performing
    *     some operation on the edges. 
    *
    *     Thus when extracting the top element one gets the ``edge'' which will
    *     result in the largest gain when performing some operation on it.
    *
    *   2) After having performed an operation on a coupled feature, one often
    *      need to recompute the priority value, the operation may even affects
    *      ``neighboring'' features, which also need to get their priority values
    *      updated.
    *
    *      Thus we are often faced with the problem of changing only a small
    *      number of heap values, destroying the heap-property.
    *
    *      Since the number of heap elements is low it would be more attractive
    *      to have a heapify functionality that exploits this fact.
    *
    * This class was implemented to make it easy to deal with the priority-feature
    * coupling and to be able to heapify single elements rather than the entire heap.
    */
    template<typename feature_type_,typename real_type_>
    class Heap
    {
    public:

      typedef real_type_      real_type;
      typedef feature_type_   feature_type;

    public:

      class heap_element
      {
      protected:

        feature_type   m_feature;
        real_type      m_priority;

      public:

        heap_element(feature_type const & f) 
          : m_feature(f)
          , m_priority( math::detail::highest<real_type>() )
        { }

      public:

        real_type           & priority()           { return m_priority; }
        feature_type  const & get_feature()  const { return m_feature;  }

      public:

        operator real_type () { return m_priority; }

      };

    protected:

      typedef std::list<heap_element>              heap_type;

    public:

      typedef typename heap_type::iterator         heap_iterator;

    protected:

      typedef std::map<feature_type,heap_iterator> lut_type;
      typedef typename lut_type::iterator          lut_iterator;
      typedef typename lut_type::const_iterator    const_lut_iterator;

      heap_type m_heap;  ///< The heap storing the heap-elements.
      lut_type  m_lut;   ///< A heap iterator lookup table. This is used to quickly find the heap element corresponding to a given feature.

    public:

      Heap()
        : m_heap()
        , m_lut()
      {}


      ~Heap()  {  }


      void clear()
      {
        m_lut.clear();
        m_heap.clear();
      }

      /**
      * Size.
      * 
      * @return    The number of heap elements in the heap.
      */
      std::size_t  size() const { return m_heap.size(); }

      /**
      * Push a new feature onto the heap.
      *
      * Note that the new elements is not ``heapified''.
      *
      * @param f   The feature that should be inserted.
      *
      * @return    A heap iterator to the newly created heap
      *            element. This can for instance be used to
      *            heapify the new element.
      */
      heap_iterator push(feature_type const & f)
      {
        heap_iterator i = m_heap.insert(m_heap.end(), heap_element(f) );
        //m_lut[f]        = i;  //--- Yrk! Copy-constructor from singular iterator!!!
        m_lut.insert( std::make_pair(f, i ));
        return i;
      }

      /**
      * Erase Feature.
      *
      * @param  f    The feature indicating the corresponding heap
      *              element (if it exist) that be erased.
      * @return      If the heap element was succesfully deleted the return
      *              value is true otherwise it is false.
      */
      bool erase(feature_type const & f )
      {
        if(m_heap.empty())
        {
          throw std::logic_error("Heap::erase(): trying to erase from an empty heap!");
          return false;
        }
        lut_iterator lookup = m_lut.find(f);
        if(lookup == m_lut.end())
        {
          throw std::invalid_argument("Heap::erase(): feature not in heap!");
          return false;
        }
        m_heap.erase(lookup->second);
        m_lut.erase(lookup);
        return true;
      }

      /**
      * Get Heap Iterator.
      *
      * @param f    The feature.
      * @return     An iterator pointing to the heap element that corresponds to the
      *             feature or the position one past the end of the heap.
      */
      heap_iterator get_heap_iterator(feature_type const & f)
      {
        if(m_heap.empty())
        {
          throw std::logic_error("Heap::get_heap_iterator(): trying to get element from empty heap!");
          return m_heap.end();
        }
        lut_iterator lookup = m_lut.find(f);
        if(lookup == m_lut.end())
        {
          throw std::invalid_argument("Heap::get_heap_iterator(): feature not in heap!");
          return m_heap.end();
        }
        return lookup->second;
      }

      /**
      * Heapify Single Element.
      * This function assumes that only the specifed heap element has the wrong
      * position in the heap. The method works by scanning first to the left and
      * then to the right, to look for the correct position of the specified
      * heap element. Worst case time-complexity is thus O(n), where n is
      * the number of heap elements.
      *
      * Notice that as an side-effect all iterators may change after having
      * invoked the method. 
      *
      *
      */
      void heapify(heap_iterator i)
      {          
        // TODO 2006-09-26 KE: Add assertion to test if i belongs to this container

        if(m_heap.empty())
        {
          throw std::logic_error("Heap::heapify(): trying to heapify an empty heap!");
          return;
        }
        if(i==m_heap.end())
        {
          throw std::out_of_range("Heap::heapify(): can not use end position!");
          return;
        }

        if(i!=m_heap.begin())
        {
          heap_iterator j = i;
          --j;
          while(i!=m_heap.begin() && j->priority() < i->priority() )
          {
            feature_type A = i->get_feature();
            feature_type B = j->get_feature();
            heap_element tmp = *i;
            *i = *j;
            *j = tmp;

            assert( (m_lut.find(A) != m_lut.end()) || !"Heap::heapify(i): A was not stored in lut");
            assert( (m_lut.find(B) != m_lut.end()) || !"Heap::heapify(i): B was not stored in lut");

            m_lut[A] = j;
            m_lut[B] = i;
            if(j!=m_heap.begin())
              --j;
            --i;
          }
        }
        if(i!=m_heap.end())
        {
          heap_iterator j = i;
          ++j;
          while(j!=m_heap.end() && j->priority() > i->priority() )
          {
            feature_type A = i->get_feature();
            feature_type B = j->get_feature();
            heap_element tmp = *i;
            *i = *j;
            *j = tmp;

            assert( (m_lut.find(A) != m_lut.end()) || !"Heap::heapify(i): A was not stored in lut");
            assert( (m_lut.find(B) != m_lut.end()) || !"Heap::heapify(i): B was not stored in lut");

            m_lut[A] = j;
            m_lut[B] = i;
            ++j;
            ++i;
          }
        }
      }

      /**
      * Heapify All of the Heap.
      * This method has time-complexity o( n lg n), where n is the number of heap elements.
      */
      void heapify() { m_heap.sort( std::greater<real_type>() ); }

      /**
      * Get Top Element.
      *
      * @return    An iterator to the heap element with largest priority value.
      */
      heap_iterator top()  { return m_heap.begin(); }

      /**
      * Get First Heap Element.
      *
      * @return   An iterator to the first heap element.
      */
      heap_iterator begin() { return m_heap.begin(); }

      /**
      * Get End Heap Element.
      *
      * @return   An iterator to the position one past the last heap element.
      */
      heap_iterator end() { return m_heap.end(); }

    };

  } //End of namespace containers
} //End of namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_CONTAINERS_HEAP_H
#endif
