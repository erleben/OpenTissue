#ifndef OPENTISSUE_KINEMATICS_INVERSE_ESTIMATE_JOINTS_H
#define OPENTISSUE_KINEMATICS_INVERSE_ESTIMATE_JOINTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_covariance.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_compute_contiguous_angle_interval.h>

#include <OpenTissue/kinematics/inverse/inverse_accessor.h>

#include <boost/cast.hpp>  // Needed for boost::numeric_cast

#include <vector>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      namespace box_limits
      {
        namespace detail
        {
          
          /**
           * Estimate Joint Type and Joint Paramters.
           * This function tries to analyse motion samples of the relative
           * motion of a skeleton bone. The function first tries to analyse
           * whether a bone is a hinge or ball type joint. Hereafter the function
           * tries to find a minimal boxed joint limit domain for the specifically
           * chosen joint type.
           *
           * @param begin    An iterator to the position of the first motion sample.
           * @param end      An iterator to one past the last motion sample.
           * @param bone     The bone corresponding to the joint that should be estimated.
           */
          template<typename sample_iterator, typename bone_type>
          inline void estimate_joint_type( sample_iterator const & begin, sample_iterator const & end, bone_type & bone)
          {
            typedef typename bone_type::math_types       math_types;
            typedef typename bone_type::bone_traits      bone_traits;
            typedef typename math_types::vector3_type    V;
            typedef typename math_types::real_type       T;
            typedef typename math_types::quaternion_type Q;
            typedef typename math_types::matrix3x3_type  M;
            typedef typename math_types::value_traits    value_traits;
            
            T const too_tiny = boost::numeric_cast<T>(0.0001); ///< Threshold value used to determine too small eigen values!
            
            size_t const N = std::distance( begin, end);
            
            std::vector<V> A;      
            std::vector<T> angles;
            std::vector<T> phi_angles;
            std::vector<T> psi_angles;
            std::vector<T> theta_angles;
            
            A.resize(N);
            angles.resize(N);
            phi_angles.resize(N);
            psi_angles.resize(N);
            theta_angles.resize(N);
            
            size_t i = 0u;                
            V rotation_axis = V(value_traits::zero(),value_traits::zero(),value_traits::zero());  
            
            // Loop over all the motion samples
            for(sample_iterator X = begin;X!=end;++X)
            {
              // Extract information about the relative motion sample
              OpenTissue::math::get_axis_angle(X->Q(), rotation_axis, angles[i]);
              OpenTissue::math::ZYZ_euler_angles(X->Q(), phi_angles[i], psi_angles[i], theta_angles[i]);
              A[i] =  X->Q().v();
              rotation_axis += A[i];
              ++i;
            }        
            rotation_axis = normalize(rotation_axis);
            
            // Now we have a lot of samples and we are wondering what kind of manifold that they lie on. 
            // We speculate that a hinge-type joint would have samples lying on a line segment whereas
            // ball-type joints must have samples on a spherical shell i.e 2 or 3 dimenional.
            //
            // We will use covariance analysis to detect the difference in dimension of such manifolds
            // and from this deduce the joint type. 
            //
            
            V m;   ///< Mean point
            M C;   ///< Covariance Matrix
            
            OpenTissue::math::covariance( A.begin(), A.end(), m, C);
            
            M R; ///< Column vectors are eigen vectors.
            V d; ///< Vector of eigen values
            
            OpenTissue::math::eigen(C, R, d);
            
            size_t dimension = fabs( d(0) ) > too_tiny ? 1u :0u;
            dimension += fabs( d(1) ) > too_tiny ? 1u :0u;
            dimension += fabs( d(2) ) > too_tiny ? 1u :0u;
            
            if(dimension == 0 )
            {
              
              bone.type() = bone_traits::ball_type;
              
              //
              // The joint is fixed and we already collected phi, psi and theta information
              // from all the motion samples. Ideally all these values are the same. So why
              // not simply use the first entry in the phi, psi and theta arrays?
              //
              // If there we some slight varition due to numerical precision and round-off
              // then we could perhaps use the mean-angles as ``better'' estimates of the
              // fixed pose?
              T const & phi   = phi_angles[0]; 
              T const & psi   = psi_angles[0]; 
              T const & theta = theta_angles[0]; 
              
              //// Since all the poses must be the same we just pick the current
              //// one flaw in this is that the bone must have an initial value here
              //// and it shouldnt be the bindpose so this may go wrong
              //OpenTissue::math::ZYZ_euler_angles(bone.relative().Q(),phi,psi,theta);// should be relative or something
              
              // Now we can store the correct joint parameter values into the bone
              bone.theta(0) = phi;
              bone.theta(1) = psi;
              bone.theta(2) = theta;
              // since this bone does not move we constrain its movement to be
              // the same as its initial position
              bone.box_limits().min_limit(0) = phi;
              bone.box_limits().min_limit(1) = psi;
              bone.box_limits().min_limit(2) = theta;
              
              bone.box_limits().max_limit(0) = phi;
              bone.box_limits().max_limit(1) = psi;
              bone.box_limits().max_limit(2) = theta;
            }
            else if(dimension == 1 )
            {
              bone.type() = bone_traits::hinge_type;
              
              bone.u() = rotation_axis;
              
              T min_angle;
              T max_angle;
              OpenTissue::math::compute_contiguous_angle_interval(angles.begin(),angles.end(),min_angle,max_angle);
              bone.box_limits().min_limit(0) = min_angle;
              bone.box_limits().max_limit(0) = max_angle;
            }
            else if(dimension == 2 || dimension == 3)
            {
              bone.type()   = bone_traits::ball_type;
              
              T phi         = value_traits::zero(); 
              T psi         = value_traits::zero(); 
              T theta       = value_traits::zero(); 
              
              T phi_max     = -value_traits::infinity();
              T psi_max     = -value_traits::infinity();
              T theta_max   = -value_traits::infinity();
              
              T phi_min     = value_traits::infinity();
              T psi_min     = value_traits::infinity();
              T theta_min   = value_traits::infinity();
              
              // For debug output, not really needed, so we have out-commented it!
              //std::ofstream file;          
              //file.open("test.m", std::ios::out | std::ios::app);
              //file << "phi"<< bone.get_number() << " = [ ";
              //for(size_t  i=0u;i<N;++i)
              //   file << phi_angles[i] << " ";
              //file << "];" << std::endl;
              //file << "psi"<< bone.get_number() << " = [ ";
              //for(size_t  i=0u;i<N;++i)
              //   file << psi_angles[i] << " ";
              //file << "];" << std::endl;
              //file << "theta"<< bone.get_number() << " = [ ";
              //for(size_t  i=0u;i<N;++i)
              //   file << theta_angles[i] << " ";
              //file << "];" << std::endl;
              //file.flush();
              //file.close();
              OpenTissue::math::compute_contiguous_angle_interval( phi_angles.begin(), phi_angles.end(), phi_min,phi_max);
              OpenTissue::math::compute_contiguous_angle_interval( psi_angles.begin(), psi_angles.end(), psi_min,psi_max);
              OpenTissue::math::compute_contiguous_angle_interval( theta_angles.begin(), theta_angles.end(), theta_min,theta_max);
              
              bone.box_limits().min_limit(0) = phi_min;
              bone.box_limits().max_limit(0) = phi_max;
              bone.theta(0) = (phi_min + phi_max)*value_traits::half();
              
              bone.box_limits().min_limit(1) = psi_min;
              bone.box_limits().max_limit(1) = psi_max;
              bone.theta(1) = (psi_min + psi_max)*value_traits::half();
              
              bone.box_limits().min_limit(2) = theta_min;
              bone.box_limits().max_limit(2) = theta_max;
              bone.theta(2) = (theta_min + theta_max)*value_traits::half();
            }
          }
          
        }// namespace detail
        
        
        /**
         * Estimate Joint types and Joint Settings.
         * This function tries to estimate the joints of a skeleton by
         * looking at motion samples.
         *
         * The intention is that this function can be used to provide an
         * end-user with a pre-set of joint types and joint limits, which
         * can be further fine-tuned by hand.
         *
         * @param scheduler    A motion blend scheduler, which is used for
         *                     sampling the motion.
         * @param skeleton     The skeleton for which we which to estimate
         *                     the joints for.
         */
        template <typename blend_scheduler_type, typename skeleton_type >
        inline void estimate_joints(blend_scheduler_type & scheduler, skeleton_type & skeleton)
        {
          size_t const N = 100u; ///< Number of samples
          
          typedef typename skeleton_type::bone_traits       bone_traits;
          typedef typename bone_traits::transform_type      transform_type;
          typedef typename skeleton_type::math_types        math_types;
          typedef typename math_types::vector3_type         V;
          typedef typename math_types::real_type            T;
          typedef typename math_types::quaternion_type      Q;
          typedef typename math_types::matrix3x3_type       M;
          typedef typename math_types::coordsys_type        coordsys_type;
          typedef typename math_types::value_traits         value_traits;
          typedef          std::vector< coordsys_type >     samples_container;
          
          // Allocate space for relative motion samples
          std::vector< samples_container > samples;
          
          samples.resize( skeleton.size() );
          for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
          {
            samples[bone->get_number()].resize( N );;
          }
          
          // Sample relative transforms for each bone throughout the entire motion 
          T duration = scheduler.compute_duration();
          T time_step = duration / (N-1);
          T time = value_traits::zero();
          for(size_t i = 0;i < N;++i)
          {
            scheduler.compute_pose(skeleton, time);
            for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
            {
              samples[bone->get_number()][i] = bone_traits::convert( bone->relative() );
            }
            time += time_step;    
          }
          
          // From the samples try to estimate joint types for each bone
          for(skeleton_type::bone_iterator bone = skeleton.begin();bone!=skeleton.end();++bone)
          {
            detail::estimate_joint_type( samples[bone->get_number()].begin(), samples[bone->get_number()].end(), *bone);
          }
        }
        
      }// namespace box_limits
    }//namespace inverse
  } //namespace kinematics
}// namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_ESTIMATE_JOINTS_H
#endif
