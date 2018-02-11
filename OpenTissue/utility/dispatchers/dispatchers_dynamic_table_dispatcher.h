#ifndef OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_DYNAMIC_TABLE_DISPATCHER_H
#define OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_DYNAMIC_TABLE_DISPATCHER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/dispatchers/dispatchers_unbound_dispatch_function.h>
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>

namespace OpenTissue
{
  namespace utility
  {
    namespace dispatchers
    {

      /** A function table-based implementation of a dynamic multiple dispatcher.
      *
      * \par Usage:
      *   -# Create (or alter) a common base-class for the dispatchable classes,
      *      which inherits ClassIDInterface. The inheriting has to be virtual
      *      public to avoid ambiguousity in the derived classes.
      *   -# Let all dispatchable objects inherite from the base-class and from
      *      the ClassID template using themself as template parameter.
      *   -# Create a dispatcher objects using the MultiDispatcher template and
      *      bind the dispatch functions.
      *
      * @tparam Base
      *   The base class of the dispatchable classes supported by this dispatcher.
      * @tparam Mirrored
      *   Whether the dispatcher should use mirroring or not.
      * @tparam R
      *   The return type of the dispatch functions.
      * @tparam T3
      *   The type of an optional third parameter.
      */
      template <
        class Base
          , bool Mirrored = false
          , typename R    = void
          , typename T3   = void
      >
      class DynamicTableDispatcher
      {
      private:

        /** @internal The functor interface.
        */
        class FunctorBase
        {
        public:

          /** Destructor
          */
          virtual ~FunctorBase(){}

          /** Execute the dispatcher on 2 dispatchable objects and a optional third.
          * This method handles the typecasting before calling the real dispatch
          * function.
          * @see DynamicTableDispatcher::operator()
          *
          * @param[in] t1
          *   The first dispatchable object
          * @param[in] t2
          *   The second dispatchable object
          * @param[in] t3
          *   The third (non-dispatchable) object
          * @return
          *   The result from the dispatch function.
          */
          virtual R operator()(Base& t1, Base& t2, T3& t3) const = 0;
        };

        /** @internal Functor which encapsulates the otherwise unsafe typecast.
        *
        * @tparam T1
        *   The type of the first parameter for the dispatch function.
        * @tparam T2
        *   The type of the second parameter for the dispatch function.
        * @tparam Swap
        *   Whether to swap the first and second parameter, when calling the
        *   dispatch function.
        */
        template < class T1, class T2, bool Swap = false >
        class Functor
          : public FunctorBase
        {
        private:

          typedef R FuncType(T1&, T2&, T3&);

          FuncType* const m_func;

        public:

          /** Constructor
          *
          * @param[in] func
          *   A pointer to the function, which the functor should encapsulate
          */
          Functor(FuncType* func)
            : m_func(func)
          {}

          R operator()(Base& t1, Base& t2, T3& t3) const
          {
            // Any good compiler is expected to optimize the branch away
            // as Swap is constant for the type
            if (Swap)
            {
              return m_func(static_cast<T1&>(t2), static_cast<T2&>(t1), t3);
            }
            else
            {
              return m_func(static_cast<T1&>(t1), static_cast<T2&>(t2), t3);
            }
          }
        };

      private:

        typedef boost::shared_ptr<FunctorBase>    FunctorPtr;
        typedef boost::multi_array<FunctorPtr, 2> FunctorTableType;

        FunctorTableType m_functor_table;          

      public:

        DynamicTableDispatcher()
          : m_functor_table( boost::extents[0][0])
        {}

        /** Bind a dispatch function to the dispatcher.
        * @note
        *   There is no way to unbind a function once it's bound other than
        *   replacing it with another function. This is a deliberate design choice.
        *
        * @tparam T1
        *   The type of the first parameter for the dispatch function, f.
        * @tparam T2
        *   The type of the second parameter for the dispatch function, f.
        * @param[in] f
        *   The dispatch function to bind. The types of the the parameters are
        *   automatically extracted.
        */
        template < class T1, class T2 >
        void bind(R (*f)(T1&, T2&, T3&))
        {
          using std::max;

          boost::multi_array_types::index idx1 = T1::id();
          boost::multi_array_types::index idx2 = T2::id();

          // Resize the table if needed
          boost::multi_array_types::size_type max_idx = max(idx1, idx2);

          if (max_idx >= m_functor_table.shape()[0])
          {
            m_functor_table.resize(boost::extents[max_idx+1][max_idx+1]);
          }

          // Any good compiler should optimize the branch as Mirrored is
          // constant for the type
          if (Mirrored)
          {
            m_functor_table[idx2][idx1].reset(new Functor<T1,T2,true>(f));
          }

          m_functor_table[idx1][idx2].reset(new Functor<T1,T2,false>(f));
        }

        /** Execute the dispatcher on 2 dispatchable objects and an optional
        * third
        * @param[in] t1
        *   The first dispatchable object
        * @param[in] t2
        *   The second dispatchable object
        * @param[in] t3
        *   The third (non-dispatchable) object
        * @return
        *   The result from the dispatch function.
        * @exception UnboundDispatchFuntion
        *   The dispatcher have no dispatch function bound to this combination
        *   of classes.
        */
        R operator()(Base& t1, Base& t2, T3& t3)
        {
          using std::max;

          boost::multi_array_types::index idx1 = t1.class_id();
          boost::multi_array_types::index idx2 = t2.class_id();

          boost::multi_array_types::size_type max_idx = max(idx1, idx2);

          // Range check
          if (max_idx >= m_functor_table.shape()[0]) 
            throw UnboundDispatchFunction();

          FunctorPtr f = m_functor_table[idx1][idx2];

          // Sanity check the pointer
          if (f.get()==0) 
            throw UnboundDispatchFunction();

          return f->operator()(t1, t2, t3);
        }
      };


      /****************************************************************************************************/

      /** @copydoc DynamicTableDispatcher
      */
      template < class Base, bool Mirrored, typename R >
      class DynamicTableDispatcher<Base, Mirrored, R, void>
      {
      private:

        /** @internal The functor interface.
        */
        class FunctorBase
        {
        public:

          /** Destructor
          */
          virtual ~FunctorBase(){}

          /** Execute the dispatcher on 2 dispatchable objects.
          * This method handles the typecasting before calling the real dispatch
          * function.
          *
          * @see DynamicTableDispatcher::operator()
          *
          * @param[in] t1
          *   The first dispatchable object
          * @param[in] t2
          *   The second dispatchable object
          * @return
          *   The result from the dispatch function.
          */
          virtual R operator()(Base& t1, Base& t2) const = 0;

        };

        /** @internal Functor which encapsulates the otherwise unsafe typecast.
        *
        * @tparam T1
        *   The type of the first parameter for the dispatch function.
        * @tparam T2
        *   The type of the second parameter for the dispatch function.
        * @tparam Swap
        *   Whether to swap the first and second parameter, when calling the
        *   dispatch function.
        */
        template < class T1, class T2, bool Swap = false >
        class Functor
          : public FunctorBase
        {
        private:

          typedef R FuncType(T1&, T2&);

          FuncType* const m_func;

        public:

          /** Constructor
          *
          * @param[in] func
          *   A pointer to the function, which the functor should encapsulate
          */
          Functor(FuncType* func)
            : m_func(func)
          {}

          /**
          */
          R operator()(Base& t1, Base& t2) const
          {
            // Any good compiler is expected to optimize the branch away
            // as Swap is constant for the type
            if (Swap)
            {
              return m_func(static_cast<T1&>(t2), static_cast<T2&>(t1));
            }
            else
            {
              return m_func(static_cast<T1&>(t1), static_cast<T2&>(t2));
            }
          }
        };

      private:

        typedef boost::shared_ptr<FunctorBase>    FunctorPtr;
        typedef boost::multi_array<FunctorPtr, 2> FunctorTableType;

        FunctorTableType m_functor_table;

      public:

        /** Constructor
        */
        DynamicTableDispatcher()
          : m_functor_table(boost::extents[0][0])
        {}

        /** Bind a dispatch function to the dispatcher.
        * @note
        *   There is no way to unbind a function once it's bound other than
        *   replacing it with another function. This is a deliberate design choice.
        *
        * @tparam T1
        *   The type of the first parameter for the dispatch function, f.
        * @tparam T2
        *   The type of the second parameter for the dispatch function, f.
        * @param[in] f
        *   The dispatch function to bind. The types of the the parameters are
        *   automatically extracted.
        */
        template < class T1, class T2 >
        void bind(R (*f)(T1&, T2&))
        {
          using std::max;

          boost::multi_array_types::index idx1 = T1::id();
          boost::multi_array_types::index idx2 = T2::id();

          // Resize the table if needed
          boost::multi_array_types::size_type max_idx = max(idx1, idx2);

          if (max_idx >= m_functor_table.shape()[0])
          {
            m_functor_table.resize(boost::extents[max_idx+1][max_idx+1]);
          }

          // Any good compiler should optimize the branch as Mirrored is
          // constant for the type
          if (Mirrored)
          {
            m_functor_table[idx2][idx1].reset(new Functor<T1,T2,true>(f));
          } 
          m_functor_table[idx1][idx2].reset(new Functor<T1,T2,false>(f));
        }

        /** Execute the dispatcher on 2 dispatchable objects.
        * @param[in] t1
        *   The first dispatchable object
        * @param[in] t2
        *   The second dispatchable object
        * @return
        *   The result from the dispatch function.
        * @exception UnboundDispatchFuntion
        *   The dispatcher have no dispatch function bound to this combination
        *   of classes.
        */
        R operator()(Base& t1, Base& t2)
        {
          using std::max;

          boost::multi_array_types::index idx1 = t1.class_id();
          boost::multi_array_types::index idx2 = t2.class_id();

          boost::multi_array_types::size_type max_idx = max(idx1, idx2);

          // Range check
          if (max_idx >= m_functor_table.shape()[0]) 
            throw UnboundDispatchFunction();

          FunctorPtr f = m_functor_table[idx1][idx2];

          // Sanity check the pointer
          if (f.get()==0) 
            throw UnboundDispatchFunction();

          return f->operator()(t1, t2);
        }
      };

    } // namespace dispatchers
  } // namespace utility
} // namespace OpenTissue

// OPENTISSUE_UTILITY_DISPATCHERS_DISPATCHERS_DYNAMIC_TABLE_DISPATCHER_H
#endif 

