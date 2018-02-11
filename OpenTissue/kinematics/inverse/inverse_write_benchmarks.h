#ifndef OPENTISSUE_KINEMATICS_INVERSE_INVERSE_WRITE_BENCHMARKS_H
#define OPENTISSUE_KINEMATICS_INVERSE_INVERSE_WRITE_BENCHMARKS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/io/big_matlab_write.h>

#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {
      namespace detail
      {

        /**
        * Write Benchmarks.
        * This function was created to make processing of benchmark data more
        * easy. Below is a simple code sample illustrating how the function
        * should be used.
        *
        * \code
        *  std::list<solver_type::Output> outputs;
        *
        *  for(...)
        *  {
        *     ...
        *     solver_type::Output output;
        *     solver.solve(..., &output );
        *     outputs.push_back( output );
        *     ...
        *  }
        *  std::string prefix = ....;
        *  std::string matlab_file = ....;
        *  std::string latex_file = ....;
        *
        *  write_benchmarks( prefix, matlab_file, latex_file, outputs.begin(), outputs.end() );
        * \endcode
        *
        * @param prefix            A string value that is used to prefix all matlab figures. This is useful to make sure that the generated figures in the end all have unique names.
        * @param matlab_filename   The path and name of the resulting matlab script. Running this script from matlab will produce a set of figures. Also all figures are automatically exported as both eps and png files.
        * @param latex_filename    The path and name of the resulting latex file. Upon return this file will contain latex code for some of the statistics.
        * @param begin             An iterator to the position of the first Output class.
        * @param end               An iterator to the position one past the last Output class.
        */
        template<typename output_iterator>
        inline void write_benchmarks( 
          std::string const & prefix
          , std::string const & matlab_filename
          , std::string const & latex_filename
          , output_iterator begin
          , output_iterator end
          )
        {
          using namespace OpenTissue::math::big;
          using std::min;
          using std::max;

          std::ofstream matlab_file( matlab_filename.c_str(), std::ios::out);
          if (!matlab_file)
          {
            std::cerr << "Error unable to create file: "<< matlab_filename << std::endl;
            return;
          }

          std::ofstream latex_file( latex_filename.c_str(), std::ios::out);
          if (!latex_file)
          {
            std::cerr << "Error unable to create file: "<< latex_filename << std::endl;
            return;
          }

          // Make sure that matlab environment is ready
          matlab_file << "close all;" << std::endl;
          
          // Write the number of iterations into an array
          {
            size_t benchmark = 0u;
            matlab_file << "I = [";
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << " " <<  output->iterations();
            }
            matlab_file << " ];" << std::endl;
          }

          // Write the wall time into an array
          {
            size_t benchmark = 0u;
            matlab_file << "T = [";
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << " " <<  output->wall_time();
            }
            matlab_file << " ];" << std::endl;
          }

          // Write the objective values into an array
          {
            size_t benchmark = 0u;
            matlab_file << "V = [";
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << " " <<  output->value();
            }
            matlab_file << " ];" << std::endl;
          }

          // Write the gradient norm values into an array
          {
            size_t benchmark = 0u;
            matlab_file << "G = [";
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << " " <<  output->gradient_norm();
            }
            matlab_file << " ];" << std::endl;
          }

          //write status into an array
           {
            size_t benchmark = 0u;
            matlab_file << "S = [";
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << " " <<  output->status();
            }
            matlab_file << " ];" << std::endl;
          }
          // Write convergence rates into arrays
          {
            size_t benchmark = 0u;
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              matlab_file << "P" << benchmark << " = " << *(output->profiling()) << ";" << std::endl;
            }
          }
          std::string error_legends = "";
          // Collect overall status information
          {
            size_t cnt_status[7];
            cnt_status[0] = 0u;
            cnt_status[1] = 0u;
            cnt_status[2] = 0u;
            cnt_status[3] = 0u;
            cnt_status[4] = 0u;
            cnt_status[5] = 0u;
            cnt_status[6] = 0u;
            for(output_iterator output=begin;output!=end;++output)
            {
              cnt_status[output->status()] =  cnt_status[output->status()] + 1;
            }
            for(size_t i=0;i<7u;++i)
            {
              std::cout << cnt_status[i] << "\t:\t"<< OpenTissue::math::optimization::get_error_message( i ) << std::endl;
              latex_file << cnt_status[i] << "&"<< OpenTissue::math::optimization::get_error_message( i ) << std::endl;
            }
            std::cout << std::endl;
            latex_file << std::endl;
          

          // matlab version of the errormessage count
          matlab_file << "errors = [ ";
          for(size_t i=0;i<7u;++i)
            {
              //std::cout << cnt_status[i] << "\t:\t"<< OpenTissue::math::optimization::get_error_message( i ) << std::endl;
              matlab_file << cnt_status[i] << " " ;
            }
          matlab_file << "];" << std::endl;
         
          
          for(size_t i=0;i<7u;++i)
            {
              if(cnt_status[i] > 0){
              error_legends += "'";
              error_legends += OpenTissue::math::optimization::get_error_message( i ) ;
              error_legends += "', ";
              }
            }
          
          }
          // Perform simple statistics on run times and iterations
          {
            double min_time = 10000.0;
            double max_time = 0.0;
            double avg_time = 0.0;
            size_t min_iterations = 10000;
            size_t max_iterations = 0;
            double avg_iterations = 0.0;
            size_t cnt_absolute_converged = 0;
            size_t benchmark = 0u;
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              if(output->status() == OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
              {
                ++cnt_absolute_converged;
                min_time = min( output->wall_time(), min_time );
                max_time = max( output->wall_time(), max_time );
                avg_time += output->wall_time();
                min_iterations = min( output->iterations(), min_iterations );
                max_iterations = max( output->iterations(), max_iterations );
                avg_iterations += output->iterations();
              }
            }
            if(cnt_absolute_converged>0)
            {
              avg_time      /= cnt_absolute_converged;
              avg_iterations /= cnt_absolute_converged;

              latex_file << "Min (secs)"  << " & " << "Avg (secs)"  << " & " <<  "Max (secs)" << "\\\\" << std::endl;
              latex_file << min_time       << " & " << avg_time       << " & " <<  max_time << "\\\\" << std::endl;
              latex_file << "Min (\\#)"  << " & " << "Avg (\\#)"  << " & " <<  "Max (\\#)" << "\\\\" << std::endl;
              latex_file << min_iterations << " & " << avg_iterations << " & " <<  max_iterations << "\\\\" << std::endl;

              std::cout << "time (min,avg,max) : " << min_time       << " : " << avg_time       << " : " <<  max_time << std::endl;
              std::cout << "iter (min,avg,max) : " << min_iterations << " : " << avg_iterations << " : " <<  max_iterations << std::endl;
              std::cout << std::endl;
            }
          }

          // Create convergence plots
          {
            matlab_file << "filename1 = '" << prefix << "_convergence';" << std::endl;
            matlab_file << "figure(1);" << std::endl;
            matlab_file << "clf" << std::endl;
            matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            size_t benchmark = 0u;
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              if(output->iterations()>0u)
              {
                size_t color_choice = (benchmark%6);
                std::string color = "";
                switch(color_choice)
                {
                case 0: color = ",'b'"; break;
                case 1: color = ",'g'"; break;
                case 2: color = ",'r'"; break;
                case 3: color = ",'c'"; break;
                case 4: color = ",'m'"; break;
                case 5: color = ",'y'"; break;
                };
                matlab_file << "plot(P"<<benchmark<<"(1:"<< output->iterations() <<")"<< color <<");" << std::endl;
              }
            }
            matlab_file << "axis tight;" << std::endl;
            matlab_file << "xlabel('Iterations','fontsize',18);" << std::endl;
            matlab_file << "ylabel('\\theta','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f1','-depsc2', filename1);" << std::endl;
            matlab_file << "print('-f1','-dpng', filename1);" << std::endl;
          }

          {
            matlab_file << "filename7 = '" << prefix << "_convergence_logarithmic';" << std::endl;
            matlab_file << "figure(7);" << std::endl;
            matlab_file << "clf" << std::endl;
            //matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            size_t benchmark = 0u;
            for(output_iterator output=begin;output!=end;++output,++benchmark)
            {
              if(output->iterations()>0u)
              {
                size_t color_choice = (benchmark%6);
                std::string color = "";
                switch(color_choice)
                {
                case 0: color = ",'b'"; break;
                case 1: color = ",'g'"; break;
                case 2: color = ",'r'"; break;
                case 3: color = ",'c'"; break;
                case 4: color = ",'m'"; break;
                case 5: color = ",'y'"; break;
                };
                matlab_file << "loglog(P"<<benchmark<<"(1:"<< output->iterations() <<")"<< color <<");" << std::endl;
              }
            }
            //matlab_file << "axis tight;" << std::endl;
            matlab_file << "xlabel('Iterations','fontsize',18);" << std::endl;
            matlab_file << "ylabel('\\theta','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f7','-depsc2', filename7);" << std::endl;
            matlab_file << "print('-f7','-dpng', filename7);" << std::endl;
          }


          // calculate the xaxis values so we get the same precision on all histograms
          
          double bucket = 1.0;
          double max_I = 100;//should be set to the max value of I
          
          
          // Make iterations histogram plot
          {
            matlab_file << "filename2 = '" << prefix << "iterations';" << std::endl;
            matlab_file << "figure(2);" << std::endl;
            matlab_file << "clf" << std::endl;
            matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            matlab_file << "hist(I," << max_I << ");" << std::endl;         
            matlab_file << "xlabel('Iterations','fontsize',18);" << std::endl;
            matlab_file << "ylabel('Count','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f2','-depsc2', filename2);" << std::endl;
            matlab_file << "print('-f2','-dpng', filename2);" << std::endl;
          }

          bucket = 0.01;
          max_I = 0.5;
          // Make objective value histogram plot
          {
            matlab_file << "filename3 = '" << prefix << "objective values';" << std::endl;
            matlab_file << "figure(3);" << std::endl;
            matlab_file << "clf" << std::endl;
            matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            matlab_file << "hist(V, 100)" << std::endl;
            
            matlab_file << "xlabel('f(\\theta)','fontsize',18);" << std::endl;
            matlab_file << "ylabel('Count','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f3','-depsc2', filename3);" << std::endl;
            matlab_file << "print('-f3','-dpng', filename3);" << std::endl;
          }

          // Make wall-time histogram plot
          bucket = 0.001;
          max_I = 0.2;
          {
            matlab_file << "filename4 = '" << prefix << "time';" << std::endl;
            matlab_file << "figure(4);" << std::endl;
            matlab_file << "clf" << std::endl;
            matlab_file << "set(gca,'fontsize',18);" << std::endl;
            matlab_file << "hold on;" << std::endl;
            matlab_file << "hist(T, 100)" << std::endl;
            matlab_file << "xlabel('Wall time (in seconds)','fontsize',18);" << std::endl;
            matlab_file << "ylabel('Count','fontsize',18);" << std::endl;
            matlab_file << "hold off;" << std::endl;
            matlab_file << "print('-f4','-depsc2', filename4);" << std::endl;
            matlab_file << "print('-f4','-dpng', filename4);" << std::endl;
          }

          // Make status pie diagram  plot
          {
            matlab_file << "filename5 = '" << prefix << "status';" << std::endl;
            matlab_file << "figure(5);" << std::endl;
            matlab_file << "clf" << std::endl; 
            
            matlab_file << "pie(errors);" << std::endl;
            matlab_file << "legend( " << error_legends << "'Location','BestOutside'); ";
            
            matlab_file << "print('-f5','-depsc2', filename5);" << std::endl;
            matlab_file << "print('-f5','-dpng', filename5);" << std::endl;
          }

          matlab_file.flush();
          matlab_file.close();

          latex_file.flush();
          latex_file.close();
          std::cout << "done writting benchmarks..." << std::endl;
        }


      } // namespace detail
    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_INVERSE_WRITE_BENCHMARKS_H
#endif
