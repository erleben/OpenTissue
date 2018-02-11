#ifndef OPENTISSUE_UTILITY_UTILITY_INDEX_ITERATOR_H
#define OPENTISSUE_UTILITY_UTILITY_INDEX_ITERATOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

# include <boost/iterator/iterator_facade.hpp> 

# ifndef BOOST_NO_SFINAE 
#  include <boost/type_traits/is_convertible.hpp> 
#  include <boost/utility/enable_if.hpp> 
# endif 

#include <cassert>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * A Index Iterator.
    * This iterator is usefull for creating custom iterators based on
    * data stored in for instance a vector type container.
    */
    template <typename index_type, class container_type> 
    class IndexIterator 
      : public boost::iterator_facade< 
      IndexIterator<index_type,container_type> 
      , typename container_type::value_type 
      , boost::forward_traversal_tag 
      > 
    { 
    public:

# ifdef BOOST_NO_MEMBER_TEMPLATE_FRIENDS 
    public: 
# else 
    private: 
      template <typename, class> friend class IndexIterator; 
# endif 

      bool             m_valid;      ///< Boolean flag, set to true if the index value have been assigned. 
      index_type       m_idx;        ///< Index into container.
      container_type*  m_container;  ///< A pointer to a container.

    private: 

      struct enabler {};  // a private type avoids misuse 

    public: 
      typedef typename container_type::value_type value_type; 

      IndexIterator() 
        : m_valid(false)
        , m_idx()
        , m_container(0) 
      {} 

      explicit IndexIterator(container_type const & container, index_type const & idx) 
        : m_valid(true)
        , m_idx(idx) 
        , m_container(const_cast<container_type*>( &container) ) 
      {} 

      explicit IndexIterator(container_type const & container ) 
        : m_valid(true)
        , m_idx(0)
        , m_container(const_cast<container_type*>( &container) )  
      {} 

      template <typename other_idx, class other_container> 
      IndexIterator( 
        IndexIterator<other_idx,other_container> const& other 
# ifndef BOOST_NO_SFINAE 
        , typename boost::enable_if< 
        boost::is_convertible<other_container, container_type> 
        , enabler 
        >::type = enabler() 
# endif 
        )
        : m_valid(other.m_valid)
        , m_idx(other.m_idx)
        , m_container(other.m_container)
      {} 


# if !BOOST_WORKAROUND(__GNUC__, == 2) 
    private: // GCC2 can't grant friendship to template member functions    
      friend class boost::iterator_core_access; 
# endif 

      template <typename other_idx, class other_container> 
      bool equal(IndexIterator<other_idx,other_container> const& other) const 
      { 
        return (this->m_valid == other.m_valid) &&
          (this->m_idx == other.m_idx)     &&
          (this->m_container == other.m_container); 
      } 

      void increment() 
      { 
        if(!this->m_valid)
          throw std::logic_error("IndexIterator::increment(): can not increment an invalid iterator");
        if(this->m_idx >= this->m_container->size())
          throw std::out_of_range("IndexIterator::increment(): can not increment past the end");
        ++m_idx; 
      } 

      bool valid() const 
      { 
        return m_valid; 
      } 

      value_type  & dereference() const 
      { 
        if(!this->m_valid)
          throw std::logic_error("IndexIterator::dereference(): can not dereference an invalid iterator");
        if(this->m_idx >= this->m_container->size())
          throw std::out_of_range("IndexIterator::dereference(): can not increment past the end");
        return (*m_container)[m_idx]; 
      } 

    };

  } //End of namespace utility

} //End of namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_INDEX_ITERATOR_H
#endif
