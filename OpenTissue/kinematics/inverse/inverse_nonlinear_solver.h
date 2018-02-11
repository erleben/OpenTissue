#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_NONLINEAR_SOLVER_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_NONLINEAR_SOLVER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/inverse/inverse_chain.h> // The basic data structure

// Utility functions for extracting data
#include <OpenTissue/kinematics/inverse/inverse_compute_jacobian.h>
#include <OpenTissue/kinematics/inverse/inverse_set_joint_parameters.h>
#include <OpenTissue/kinematics/inverse/inverse_get_joint_parameters.h>
#include <OpenTissue/kinematics/inverse/inverse_compute_joint_limits_projection.h>
#include <OpenTissue/kinematics/inverse/inverse_compute_weighted_difference.h>

// Numerical methods
#include <OpenTissue/core/math/optimization/optimization_projected_bfgs.h>
#include <OpenTissue/core/math/optimization/optimization_projected_steepest_descent.h>

#include <OpenTissue/core/math/big/big_prod_trans.h> // Needed for prod_trans

#include <OpenTissue/utility/utility_timer.h>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      namespace detail
      {

        /**
        * Function Calculator Functor class
        * This functor class provides functionality for calculating the objective
        * function of the inverse kinematics problem. It keeps a pointer to the 
        * solver class for easy update of its parameters and provides a () operator
        * for easy calling.
        *
        * @warning The class must be initialized before use
        *
        * @tparam solver_type The solver_type that this Function calculator is used on 
        */
        template< typename solver_type >
        class FunctionCalculator
        {
        public:

          typedef typename solver_type::vector_type           vector_type;
          typedef typename vector_type::value_type            real_type;

        protected:

          solver_type * m_S;       ///< A pointer to the solver
          vector_type   m_delta;   ///< Temporary storage, used to hold intermediate computations of the weighted difference vector.

        public:

          FunctionCalculator()
            : m_S(0)
          {}

          /**
          * Initialization.
          *
          * @param S A pointer to the solver class for which this class should work on.
          **/
          void init(solver_type* S) {  m_S = S; }

          /**
          * Function Operator.
          * This operator computes the expression:
          *
          * \f[ f(\theta) = (g - F(\theta))^T W (g - F(\theta)) \f]
          *
          * @param theta   The current value of the joint paramters.
          * @return        The resulting value of \f$f(\theta)$\f. This is the
          *                squared error measure of how close we are to reaching
          *                the desired goal placements.
          */
          real_type operator ()(vector_type const & theta) const
          {
            using ublas::inner_prod;
            OpenTissue::kinematics::inverse::set_joint_parameters( *(this->m_S->skeleton()), theta );

            vector_type & delta = const_cast<vector_type&>( this->m_delta );
            OpenTissue::kinematics::inverse::compute_weighted_difference( this->m_S->chain_begin(), this->m_S->chain_end(), delta );

            return inner_prod(delta,delta);
          }
        };

        /**
        * Gradient Calculator functor Class
        * This class provides a function for calculating the gradient of the
        * inverse kinematics object function.
        *
        * @warning The class must be initialized before use
        *
        * @tparam solver_type The solver_type that this Function calculator is used on 
        */
        template<typename solver_type >
        class GradientCalculator
        {
        public:

          typedef typename solver_type::math_types    math_types; 
          typedef typename solver_type::vector_type   vector_type; 
          typedef typename solver_type::matrix_type   matrix_type; 
          typedef typename math_types::value_traits   value_traits; 

        protected:

          solver_type * m_S;       ///< A pointer to the solver
          vector_type   m_delta;   ///< Temporary storage, used to hold intermediate computations of the weighted difference vector.
          matrix_type   m_J;       ///< Temporary storage, used to hold intermediate computations of Jacobian.
          vector_type   m_tmp;     ///< Temporary storage, used to hold intermediate computations.

        public:

          GradientCalculator()
            : m_S(0)
          {}

          /**
          * Initialization.
          *
          * @param S A pointer to the solver class for which this class should work
          */
          void init(solver_type* S ) { m_S = S; }

          /**
          * Function Operator.
          * This operator computes the expression
          *
          *  \f[ \nabla f(\theta) = - 2 J^T W ( g - F(\theta)) \f]
          *
          * By definition we have
          * \f[
          *  f(\vec \theta) = (\vec g - \vec F(\vec \theta))^T \mat W (\vec g - \vec F(\theta)),
          * ]\f
          * Then the differential can be computed as follows
          * \f[
          *  d f = d(\vec g - \vec F(\vec \theta))^T \mat W (\vec g-\vec F(\vec \theta))
          *  +
          *  (\vec g - \vec F(\vec \theta))^T \mat W d(\vec g - \vec F(\vec \theta)) 
          * ]\f
          * which reduces to
          * \f[
          *  d f = 2 (\vec g - \vec F(\theta))^T \mat W d(\vec g - \vec F(\vec \theta))
          * ]\f
          * where
          * \f[
          * d(\vec g - \vec F(\vec \theta)) &= - d \vec F(\vec \theta) =
          *  - \frac{\partial \vec F(\vec \theta)}{\partial \vec \theta} d\vec{\theta}\\
          *   &= - \mat J d \vec{\theta}
          * ]\f
          * Which means
          * \f[
          *   d f = - 2 (\vec g - \vec F(\vec \theta))^T \mat W \mat J d \vec{\theta}
          * ]\f
          * From this we have
          * \f[
          * \frac{d f}{d \vec \theta} = - 2 (\vec g - \vec F(\vec \theta))^T \mat W \mat J  
          * ]\f
          * and the gradient can now be written
          * \f[
          * \nabla f 
          *   = 
          *   \frac{d f}{d \vec \theta}^T = - 2 \mat J^T \mat W ( \vec g - \vec F(\vec
          *   \theta) )  
          * ]\f
          *
          * @param theta   The current value of the joint paramters.
          * @return        The resulting gradient value.
          */
          vector_type operator ()(vector_type const& theta) const
          {
            using ublas::prod;
            using ublas::trans;

            OpenTissue::kinematics::inverse::set_joint_parameters( *(this->m_S->skeleton()), theta );

            vector_type & delta = const_cast<vector_type&>( this->m_delta );
            OpenTissue::kinematics::inverse::compute_weighted_difference( this->m_S->chain_begin(), this->m_S->chain_end(), delta);

            matrix_type & J = const_cast<matrix_type&>( this->m_J );
            OpenTissue::kinematics::inverse::compute_jacobian( this->m_S->chain_begin(), this->m_S->chain_end(), this->m_S->skeleton()->begin(), this->m_S->skeleton()->end(), J  );

            vector_type & tmp = const_cast<vector_type&>( this->m_tmp );
            if(tmp.size() != J.size2())
              tmp.resize( J.size2() );
            OpenTissue::math::big::prod_trans( J, delta, tmp ); 
            tmp *= -value_traits::two();
            return tmp;
          }
        };

        /**
        * Projection Calculator Functor Class.
        * This class provides functionality for projection the current
        * joint paramter values onto the current joint limits.
        *
        * The functor is intended for easily integrating the inverse
        * kinematics solver with other numerical methods.
        *
        * @warning The class must be initialized before use
        *
        * @tparam solver_type The solver_type that this Function calculator is used on 
        */
        template<typename solver_type >
        class ProjectionCalculator
        {
        public:

          typedef typename solver_type::math_types    math_types; 
          typedef typename solver_type::vector_type   vector_type; 
          typedef typename math_types::real_type      real_type; 
          typedef typename math_types::value_traits   value_traits; 

        protected:

          solver_type * m_S; ///< A pointer to the solver

        public:

          ProjectionCalculator()
            : m_S(0)
          {}

          /**
          * Initialization.
          *
          * @param S A pointer to the solver class for which this class should work
          */
          void init(solver_type* S ){ this->m_S = S; }


          /**
          * Function Operator.
          * This operator computes the expression
          *
          * \f[ \theta = \max( \theta_{\min] \min( \theta, \theta_{\max})  )]\f
          *
          *
          * @param theta   The current value of the joint paramters.
          * @return        The projected value of theta.
          *
          */
          vector_type operator ()(vector_type const & theta) const
          {
            // 2008-07-06 kenny: Hmm is this efficient? Would this not create a un-needed temporary?
            vector_type ret = theta; 
            OpenTissue::kinematics::inverse::compute_joint_limits_projection(*(this->m_S->skeleton()), ret );
            return ret;
          }
        };

      } // namespace detail

      /**
      * A Non-linear Inverse Kinematics Problem Solver.
      *
      * Say we a given a branched mechanism containing \f$K\f$ kinematic chains and where
      * each chain have exactly one end-effector. Thus we agglomerate the \f$K\f$
      * end-effector functions into one function
      * \f[
      *   \vec y = 
      *   \begin{bmatrix}
      *     \vec y_1 \\
      *     \hdots\\
      *     \vec y_K
      *   \end{bmatrix}
      *   = 
      *   \begin{bmatrix}
      *     \vec F_1(\vec \theta) \\
      *     \vdots\\
      *     \vec F_K(\vec \theta) \\
      *   \end{bmatrix} = \vec F(\vec \theta),
      *   ]\f
      * where \f$\vec y_j$\f is the world coordinate position of the \f$j^{\text{th}}$\f
      * end-effector (tool-frame), and \f$\vec F_j(\vec \theta)$\f is the end-effector
      * function corresponding to the \f$j^{\text{th}}$\f kinematic chain. Using the
      * agglomerated end-effector function we create the objective function
      * \f[
      *   \label{eq:objective:function}
      *   f(\vec \theta) 
      *   = 
      *   (\vec g - \vec F(\vec \theta))^T 
      *   \mat W 
      *   (\vec g- \vec F(\vec \theta)),
      * ]\f
      * where \f$\mat W$\f is a symmetric positive definite and possible diagonal matrix and
      * \f$\vec g = \begin{bmatrix} \vec g_1^T & \cdots & \vec g_K^T \end{bmatrix}^T$\f is
      * the agglomerated vector of end-effector (tool frame) goals. The optimization
      * problem is,
      * \f[
      *   \vec \theta^* = \min_{\vec \theta} f(\vec \theta)
      * ]\f
      * subject to the linear box-constraints
      * \f[
      *   \vec \theta &\geq \vec l\\
      *   \vec \theta &\leq \vec u
      * ]\f
      * which models the minimum and maximum joint parameter values. Here \f$\vec l$\f is a
      * vector containing the minimum joint limits and \f$\vec u$\f is a vector of the
      * maximum joints limits. This implies \f$\vec l \leq \vec u$\f at all times.
      * 
      * Our formulation is a squared weighted norm measuring the distance between the
      * goal positions and the end-effector positions. If \f$\vec F$\f is sufficiently
      * smooth then when \f$\vec \theta \rightarrow \vec \theta^*$\f we have that
      * \f$\vec F$\f behaves almost as a linear function. This intuition suggest
      * that as we get close to a solution this formulation behaves as
      * a convex quadratic minimization problem. Further, by design all constraints are
      * linear functions defining a convex feasible region. Thus a simple constraint
      * qualification for the first-order necessary (Karush-Kuhn-Tucker) optimality
      * cconditions are always fulfilled by design of the formulation.
      *
      *
      * @tparam skeleton_type_   The skelton type that the solver
      *                          should work on. This type should
      *                          support the interface specified by
      *                          the inverse kinematics bone traits.
      */
      template<typename skeleton_type_ >
      class NonlinearSolver
      {
      public:

        typedef          skeleton_type_              skeleton_type;
        typedef typename skeleton_type::math_types   math_types;
        typedef typename skeleton_type::bone_traits  bone_traits;
        typedef          Chain<skeleton_type>		     chain_type;

      protected:

        typedef typename math_types::real_type       real_type;
        typedef typename math_types::vector3_type    vector3_type;
        typedef typename math_types::value_traits    value_traits;
        typedef std::list<chain_type>								 chain_container;

      public:

        typedef typename chain_container::iterator	               chain_iterator;
        typedef typename chain_container::const_iterator	         const_chain_iterator;
        typedef typename ublas::vector<real_type>	                 vector_type;  
        typedef typename ublas::vector_range< vector_type >        vector_range;
        typedef typename ublas::vector_range< const vector_type >  const_vector_range;
        typedef typename ublas::compressed_matrix<real_type>	     matrix_type;
        typedef typename ublas::identity_matrix<real_type>			   identity_matrix_type;

      protected:

        typedef          NonlinearSolver<skeleton_type>              solver_type;
        typedef typename detail::GradientCalculator<solver_type>     gradient_type;
        typedef typename detail::FunctionCalculator<solver_type >    function_type;
        typedef typename detail::ProjectionCalculator<solver_type >  projection_type;

      protected:

        gradient_type     m_gradient;   ///< Gradient of end-effector function evaluator.
        function_type     m_function;   ///< End-effector function evaluator.
        projection_type   m_projection; ///< The joint parameter value projection operator.
        chain_container   m_chains;     ///< A collection of inverse kinematic chains.
        skeleton_type*    m_skeleton;   ///< A pointer to the skeleton that the solver works on. Default value is null.
        vector_type       m_theta;      ///< The current value of the joint parameter vector.
        matrix_type       m_H;          ///< The current estimate of the Hessian matrix used as initial estimate for some of the numerical optimization methods.

      public:

        /**
        * Get Skeleton Pointer.
        *
        * @return    A pointer to the skeleton that the solver works on.
        */
        skeleton_type       * skeleton()       { return this->m_skeleton; }
        skeleton_type const * skeleton() const { return this->m_skeleton; }

        /**
        * Get Chain Iterator.
        *
        * @return An iterator pointing to the first element in the chain list
        */
        chain_iterator        chain_begin()       { return m_chains.begin(); }
        const_chain_iterator  chain_begin() const { return m_chains.begin(); }

        /**
        * Get Chain Iterator.
        *
        * @return  An iterator pointing to one past the last element in the chain list
        */
        chain_iterator        chain_end()       { return m_chains.end(); }
        const_chain_iterator  chain_end() const { return m_chains.end(); }

        /**
        * Get Number of Chains.
        * This method gives the number of kinematics chains and goals
        *
        * @return The number of kinematic chains
        */
        size_t size() const { return m_chains.size(); }

        /*
        * Add Chain.
        * This method adds a chain and returns an iterator for the newly created iterator.
        *
        * @param chain.   The chain that should be added to the solver.
        * @return         An iterator to the newly created chain.
        */
        chain_iterator add_chain(chain_type const & chain)
        {
          return m_chains.insert(m_chains.end(), chain);
        }

        /**
        * Remove Chain.
        * This method removes a chain from the chain list 
        *
        * @param chain   An iterator for the chain that should be removed.
        * @return        An iterator to some other chain in the solver.
        */
        chain_iterator remove_chain(chain_iterator & chain)
        {
          chain_iterator iter = this->m_chains.erase(chain);

          if( iter != this->m_chains.end() )
            return iter;
          return this->m_chains.begin();
        }

      public:

        NonlinearSolver()
          : m_gradient()
          , m_function()
          , m_projection()
          , m_skeleton(0)
        {
          this->m_gradient.init(this);
          this->m_function.init(this);
          this->m_projection.init(this);
        }

        NonlinearSolver(solver_type const & solver)  
          : m_gradient()
          , m_function()
          , m_projection()
        {
          *this = solver;
        }

        NonlinearSolver(skeleton_type & S)
        {
          this->init(S);
        }

        ~NonlinearSolver(){ }

        solver_type& operator=(solver_type const &S)
        {
          if(this!= &S)
          {
            this->m_skeleton   = S.m_skeleton;
            this->m_chains     = S.m_chains;
            this->m_gradient   = S.m_gradient;
            this->m_function   = S.m_function;
            this->m_projection = S.m_projection;
            this->m_theta      = S.m_theta;
            this->m_H          = S.m_H;

            this->m_gradient.init(this);
            this->m_function.init(this);
            this->m_projection.init(this);
          }
          return *this;
        }

      public:

        /**
        * Output Class.
        * Instances of this class can be used to retrieve information
        * about the computations done by the numerical method that is
        * used for solving the non-linear optimization problem.
        *
        * One uses the class as follows
        * /code
        *  solver_type solver;
        *
        *  solver_type::Output output;
        *  solver.solve( 0, &output );
        * /endcode
        *
        */
        class Output
        {
        protected:

          real_type   m_value;           ///< The value of the objective function.
          real_type   m_wall_time;       ///< The wall clock time it took for the numerical method to exit.
          size_t      m_status;          ///< The status of the numerical method when exiting.
          std::string m_error_message;   ///< A textual version of the status.
          size_t      m_iterations;      ///< The number of used iterations.
          real_type   m_gradient_norm;   ///< The value of the gradient norm.
          vector_type m_profiling;       ///< The values of the merit function at each iterates.

        public:

          real_type   const & value () const { return m_value;}
          real_type         & value ()       { return m_value;}

          real_type   const & wall_time () const { return m_wall_time;}
          real_type         & wall_time ()       { return m_wall_time;}

          size_t      const & status () const { return m_status;}
          size_t            & status ()       { return m_status;}

          std::string const & error_message () const { return m_error_message;}
          std::string       & error_message ()       { return m_error_message;}

          size_t      const & iterations () const { return m_iterations;}
          size_t            & iterations ()       { return m_iterations;}

          real_type   const & gradient_norm () const { return m_gradient_norm;}
          real_type         & gradient_norm ()       { return m_gradient_norm;}

          vector_type const * profiling () const { return &m_profiling;}
          vector_type       * profiling ()       { return &m_profiling;}

        public:

          Output()
            : m_value(value_traits::zero())
            , m_wall_time(value_traits::zero())
            , m_status( OpenTissue::math::optimization::OK )
            , m_error_message( "" )
            , m_iterations(0u)
            , m_gradient_norm( value_traits::infinity() )
          {}

          Output( Output const & o)
          {
            *this = o;
          }

          Output & operator=( Output const & o)
          {
            if( this != &o)
            {
              m_value         = o.m_value;
              m_wall_time     = o.m_wall_time;
              m_status        = o.m_status;
              m_error_message = o.m_error_message;
              m_iterations    = o.m_iterations;
              m_gradient_norm = o.m_gradient_norm;
              m_profiling     = o.m_profiling;
            }
            return *this;
          }

        };

      public:

        /**
        * Numerical Method Settings.
        * This class encapsulates all the settings that can be given
        * to the numerical methods used.
        *
        * One uses the class as follows:
        *
        * \code
        * solver_type solver;
        *
        * solver_type::Settings settings;
        *
        * ... set whatever settings one pleases ...
        *
        * solver.solve( &settings );
        * \endcode
        * If one do not want to tweak the settings oneself then a set of pre-set settings are supplied. These can be accesssed as follows
        *
        * \code
        *   solver.solve ( &solver_type::default_BFGS_settings() );
        *   solver.solve ( &solver_type::default_SD_settings() );
        *   solver.solve ( &solver_type::default_SDF_settings() );
        * \endcode
        * If no settings are specified then the default BFGS settings are used.
        */
        class Settings
        {
        protected:

          size_t    m_choice;                ///< This argument is used to choose which numerical method that should be used to solve the inverse kinematics problem. The value zero means that a BFGS type method is used as default. The value 1 selects a Steepest Descent method with a projected back-tracking line-search and the value 2 selects a Steepest Descent using a projected fixed step-length update.
          bool      m_restart;               ///< Boolean flag indicating whether the chosen numerical method should  be restarted. This could for instance imply that a Hessian approximation is re-initialized.
          size_t    m_max_iterations;        ///< This argument holds the value of the maximum allowed iterations.
          real_type m_absolute_tolerance;    ///< This argument holds the value used in the absolute stopping criteria. Setting the value to zero will make the test in-effective.
          real_type m_relative_tolerance;    ///< This argument holds the value used in the relative stopping criteria. Setting the value to zero will make the test in-effective.
          real_type m_stagnation_tolerance;  ///< This argument holds the value used in the stagnation test. It is an upper bound of the infinity-norm of the difference in the x-solution between two iterations.  Setting the value to zero will make the test in-effective.
          real_type m_alpha;                 ///< Armijo test paramter, should be in the range 0..1, this is the fraction of sufficient decrease that is needed by the line-search method. A good value is often 0.00001;
          real_type m_beta;                  ///< The step-length reduction parameter. Everytime the Armijo condition fails then the step length is reduced by this fraction. Usually alpha < beta < 1. A good value is often 0.5;
          real_type m_tau;                   ///< This arguments holds teh size of a fixed step length (used by one of the Steepest Descent methods).
          bool      m_resynchronization;     ///< This boolean flag is used to force resynchronization between IK data and relative bone coordinate transformations upon invokation of the solve method. Default behaviour is set to off. @see ACCESSOR::get_theta() for more details.

        public:

          size_t    const & choice () const { return m_choice; }
          size_t          & choice ()       { return m_choice; }

          bool      const & restart () const { return m_restart; }
          bool            & restart ()       { return m_restart; }

          size_t    const & max_iterations () const { return m_max_iterations; }
          size_t          & max_iterations ()       { return m_max_iterations; }

          real_type const & absolute_tolerance () const { return m_absolute_tolerance; }
          real_type       & absolute_tolerance ()       { return m_absolute_tolerance; }

          real_type const & relative_tolerance () const { return m_relative_tolerance; }
          real_type       & relative_tolerance ()       { return m_relative_tolerance; }

          real_type const & stagnation_tolerance () const { return m_stagnation_tolerance; }
          real_type       & stagnation_tolerance ()       { return m_stagnation_tolerance; }

          real_type const & alpha () const { return m_alpha; }
          real_type       & alpha ()       { return m_alpha; }

          real_type const & beta () const { return m_beta; }
          real_type       & beta ()       { return m_beta; }

          real_type const & tau () const { return m_tau; }
          real_type       & tau ()       { return m_tau; }

          bool      const & resynchronization () const { return m_resynchronization; }
          bool            & resynchronization ()       { return m_resynchronization; }
          
       public:

          Settings()
            : m_choice(0u)
            , m_restart(true)
            , m_max_iterations(0u)
            , m_absolute_tolerance( value_traits::zero() )
            , m_relative_tolerance( value_traits::zero() )
            , m_stagnation_tolerance( value_traits::zero() )
            , m_alpha( value_traits::zero() )
            , m_beta( value_traits::zero() )
            , m_tau( value_traits::zero() )
            , m_resynchronization( false )
          {}

          Settings(
            size_t    const & choice
            , bool      const & restart
            , size_t    const & max_iterations
            , real_type const & absolute_tolerance
            , real_type const & relative_tolerance
            , real_type const & stagnation_tolerance
            , real_type const & alpha
            , real_type const & beta
            , real_type const & tau
            )
            : m_choice(choice)
            , m_restart(restart)
            , m_max_iterations(max_iterations)
            , m_absolute_tolerance( absolute_tolerance )
            , m_relative_tolerance( relative_tolerance )
            , m_stagnation_tolerance( stagnation_tolerance )
            , m_alpha( alpha )
            , m_beta( beta )
            , m_tau( tau )
          {}

          Settings(Settings const & s){  *this = s; }

          Settings & operator=(Settings const & s)
          {
            if(this != &s)
            {
              m_choice               = s.choice();
              m_restart              = s.restart();
              m_max_iterations       = s.max_iterations();
              m_absolute_tolerance   = s.absolute_tolerance();
              m_relative_tolerance   = s.relative_tolerance();
              m_stagnation_tolerance = s.stagnation_tolerance();
              m_alpha                = s.alpha();
              m_beta                 = s.beta();
              m_tau                  = s.tau();
            }
            return *this;
          }

        };


        /**
        * Default Settings for Quasi Newton (BFGS) method using a Projected Back-Tracking Line-Search.
        *
        * @return   The value of the default settings.
        */
        static Settings const & default_BFGS_settings()
        {
          static Settings settings( 
            0u
            , true
            , 5u
            , boost::numeric_cast<real_type>(1e-3)
            , boost::numeric_cast<real_type>(0.00001)
            , boost::numeric_cast<real_type>(0.00001)
            , boost::numeric_cast<real_type>(0.0001)
            , boost::numeric_cast<real_type>(0.5)
            , value_traits::zero()
            );
          return settings;
        }

        /**
        * Default Settings for Steepest Descent with Projected Back-Tracking Line-Search.
        *
        * @return   The value of the default settings.
        */
        static Settings const & default_SD_settings()
        {
          static Settings settings( 
            1u
            , false
            , 10u
            , boost::numeric_cast<real_type>(1e-3)
            , boost::numeric_cast<real_type>(0.00001)
            , boost::numeric_cast<real_type>(0.00001)
            , boost::numeric_cast<real_type>(0.0001)
            , boost::numeric_cast<real_type>(0.5)
            , value_traits::zero()
            );
          return settings;
        }

        /**
        * Default Settings for Steepest Descent with Fixed Step Length.
        *
        * @return   The value of the default settings.
        */
        static Settings const & default_SDF_settings()
        {
          static Settings settings( 
            2u
            , false
            , 100u
            , boost::numeric_cast<real_type>(1e-3)
            , boost::numeric_cast<real_type>(0.00001)
            , boost::numeric_cast<real_type>(0.00001)
            , value_traits::zero()
            , value_traits::zero()
            , boost::numeric_cast<real_type>(0.001)
            );
          return settings;
        }

      public:

        /**
        * Initialize Solver.
        * This method initializes all data members to their default
        * values. Initially there are no kinematic chains. These must
        * be added afterwards by invoking the method add_chain.
        *
        * @param S   The skeleton to be used.
        */
        void init(skeleton_type const & S)
        {
          this->m_theta.resize(S.size()*6u);
          this->m_theta.clear();

          this->m_H.clear();
          this->m_H.assign(identity_matrix_type(this->m_H.size1(),this->m_H.size2()));
          this->m_chains.clear();
          this->m_skeleton = const_cast<skeleton_type*>(&S);

          this->m_gradient.init(this);
          this->m_function.init(this);
          this->m_projection.init(this);
        }

        /**
        * Solve Inverse Kinematics Problem.
        * This method sets up the inverse kinematics problem as a non-linear
        * optimization problem and invokes a numerical method (by the callers choice)
        * for finding a solution.
        *
        * @warning If one has directly manipulated relative bone coordinate transformations
        *          between invokations of the IK solver or between initialization of IK and
        *          invokation of the IK solver then one must resynchronize IK data with the
        *          current pose of the skeleton. This could be the case if one is mizing forward
        *          kinematics/keyframe animation with IK, One can do the resynchronization by
        *          turning on the resynchronization feature using the settings structure. @see
        *          Settings::m_resynchronization for more details.
        *
        * @see class Settings
        * @see class Output
        *
        * @param settings_   This is a pointer to the settings of the numerical method. See class Settings for details. The default value is null. Which means that the values of the method default_BFGS_settings() are used.
        * @param output      This is a pointer to the output of the numerical method. Default value is null. This means that no output (and profiling) is done.
        */
        void solve(
          Settings const * settings_ = 0
          , Output * output = 0
          )
        {   
          Settings const * settings = settings_ ? settings_ : &this->default_SD_settings();

          size_t        status        = 0u;
          size_t        iteration     = 0u;
          real_type     gradient_norm = value_traits::zero();
          vector_type * profiling     = output ? output->profiling() : 0;

          OpenTissue::utility::Timer<real_type> watch;

          if(output)
            watch.start();

          OpenTissue::kinematics::inverse::get_joint_parameters( *(this->m_skeleton), this->m_theta, settings->resynchronization() );

          switch( settings->choice() )
          {
          case 0: /*use BFGS */
            {
              //Before invoking solver we must determine the initial estimate of the Hessian matrix
              size_t const N = this->m_theta.size();
              if(  this->m_H.size1()!=N || this->m_H.size2()!=N )
              {
                this->m_H.resize( N, N, false);
                this->m_H.assign( identity_matrix_type(this->m_H.size1(),this->m_H.size2()) );
              }

              if(settings->restart())
              {
                this->m_H.assign(identity_matrix_type(this->m_H.size1(),this->m_H.size2()));
              }

              OpenTissue::math::optimization::projected_bfgs(
                this->m_function
                , this->m_gradient
                , this->m_H
                , this->m_theta
                , this->m_projection
                , settings->max_iterations()
                , settings->absolute_tolerance()
                , settings->relative_tolerance()
                , settings->stagnation_tolerance()
                , status
                , iteration
                , gradient_norm
                , settings->alpha()
                , settings->beta()
                , profiling
                );
            }
            break;
          case 1: /*use Steepest Descent with Line-Search */
            {
              OpenTissue::math::optimization::projected_steepest_descent(
                this->m_function
                , this->m_gradient
                , this->m_theta
                , this->m_projection
                , settings->max_iterations()
                , settings->absolute_tolerance()
                , settings->relative_tolerance()
                , settings->stagnation_tolerance()
                , status
                , iteration
                , gradient_norm
                , settings->alpha()
                , settings->beta()
                , profiling
                );
            }
            break;
          case 2: /*use Steepest Descent with Fixed Step-Length */
            {
              OpenTissue::math::optimization::projected_steepest_descent(
                this->m_function
                , this->m_gradient
                , this->m_theta
                , this->m_projection
                , settings->max_iterations()
                , settings->absolute_tolerance()
                , settings->relative_tolerance()
                , settings->stagnation_tolerance()
                , status
                , iteration
                , gradient_norm
                , settings->tau()
                , profiling
                );
            }
            break;
          default:
            assert(false || !"solve(): Could not determine the numerical method to be used");
            break;
          };

          OpenTissue::kinematics::inverse::set_joint_parameters( *(this->m_skeleton) , this->m_theta );


          if(output)
          {
            watch.stop();
            output->status()        = status;
            output->error_message() = OpenTissue::math::optimization::get_error_message(status);
            output->gradient_norm() = gradient_norm;
            output->iterations()    = iteration;
            output->value()         = this->m_function( this->m_theta );
            output->wall_time()     = watch();
 
          }

        }

      };

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_NONLINEAR_SOLVER_H
#endif
