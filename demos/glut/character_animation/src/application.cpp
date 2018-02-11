//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/utility/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

#include <OpenTissue/core/math/math_basic_types.h>

#include <OpenTissue/kinematics/skeleton/io/skeleton_xml_read.h>
#include <OpenTissue/kinematics/skeleton/io/skeleton_cal3d_xml_read.h>

#include <OpenTissue/kinematics/animation/io/animation_keyframe_animation_xml_read.h>
#include <OpenTissue/kinematics/animation/io/animation_keyframe_animation_cal3d_xml_read.h>

#include <OpenTissue/kinematics/skinning/io/skinning_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_cal3d_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_material_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_material_cal3d_xml_read.h>

#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
#include <OpenTissue/kinematics/skinning/skinning_types.h>
#include <OpenTissue/kinematics/animation/animation_naive_blend_scheduler.h>
#include <OpenTissue/kinematics/animation/animation_keyframe_animation.h>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:

  typedef float                                                        real_type;
  typedef OpenTissue::math::default_math_types                         math_types;
  typedef math_types::matrix3x3_type                                   matrix3x3_type;
  typedef math_types::quaternion_type                                  quaternion_type;
  typedef math_types::vector3_type                                     vector3_type;
  typedef OpenTissue::skeleton::Types<math_types>                      skeleton_types;

//  typedef OpenTissue::skinning::Types< math_types, OpenTissue::skinning::SBSGPU >	 skin_types;
//  typedef OpenTissue::skinning::Types< math_types, OpenTissue::skinning::SBS >	 skin_types;
//  typedef OpenTissue::skinning::Types< math_types, OpenTissue::skinning::LBS >	 skin_types;
//  typedef OpenTissue::skinning::Types< math_types, OpenTissue::skinning::LBSGPU >	 skin_types;
  typedef OpenTissue::skinning::Types< math_types, OpenTissue::skinning::DBS >	 skin_types;

protected:

  bool b[256];          ///< Boolean array used to keep track of the user actions/selections.

  typedef skeleton_types::skeleton_type                    skeleton_type;
  typedef skeleton_types::bone_type                        bone_type;
  typedef skin_types::skin_type                            skin_type;
  typedef skin_types::skin_render_type                     skin_render_type;

  typedef OpenTissue::animation::KeyframeAnimation<skeleton_type>     keyframe_animation_type;
  typedef OpenTissue::animation::NaiveBlendScheduler<skeleton_type>   naive_blend_scheduler_type;

protected:

  //--- Data for an animated character...
  skeleton_type               m_skeleton;         ///< Skeleton (i.e. bones).
  keyframe_animation_type     m_animation[100];   ///< The raw animations.
  naive_blend_scheduler_type  m_blend_scheduler;  ///< The animation blend scheduler (combines raw animations into final animation).
  real_type                   m_time;             ///< The current animation time.
  real_type                   m_delta_time;       ///< Time inbetween two poses (i.e. 1/fps).
  real_type                   m_duration;         ///< Duration of animation.
  skin_type                   m_skin;             ///< Skin container.
  bool                        m_display_bones;    ///< Boolean flag used to turn on/off rendering of bones.
  bool                        m_display_skin;     ///< Boolean flag used to turn on/off rendering of skin.

  // Also, a GPU skin uploader/render, by spreak!
  skin_render_type            m_skin_render;

  // Some key values used for user-interface
  static unsigned char const TWO_KEY    = '2';
  static unsigned char const THREE_KEY  = '3';
  static unsigned char const FOUR_KEY   = '4';
  static unsigned char const FIVE_KEY   = '5';
  static unsigned char const SIX_KEY    = '6';
  static unsigned char const SEVEN_KEY  = '7';
  static unsigned char const EIGHT_KEY  = '8';
  
public:


  Application()
  {
    this->z_far() = 5000;
  }

  char const * do_get_title() const { return "Character Animation Application"; }

  void do_display()
  {
    if(m_display_bones)
    {
      skeleton_type::bone_iterator begin = m_skeleton.begin();
      skeleton_type::bone_iterator end = m_skeleton.end();
      skeleton_type::bone_iterator bone;
      for(bone=begin;bone!=end;++bone)
      {
        OpenTissue::gl::DrawBone(*bone);
        OpenTissue::gl::DrawFancyBone(*bone);
      }
    }
    if(m_display_skin)
    {
      m_skin_render.render( m_skin, m_skeleton );
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    b[choice] = ! b[choice];
    switch ( choice )
    {
    case 't':   run();                                 break;
    case 'b':   m_display_bones = !m_display_bones;    break;
    case 's':   m_display_skin = !m_display_skin;      break;
    case 'c':   
      {
        m_blend_scheduler.clear(); 
        m_duration = 0.0; 
        b[TWO_KEY] = false;
        b[THREE_KEY] = false;
        b[FOUR_KEY] = false;
        b[FIVE_KEY] = false;
        b[SIX_KEY] = false;
        b[SEVEN_KEY] = false;
        b[EIGHT_KEY] = false;
      }
      break;
    case 'r':   m_time = 0; break;
    case '1':
      break;
    case '2':
      if(b[TWO_KEY])
        m_blend_scheduler.add( &m_animation[0] );
      else
        m_blend_scheduler.remove( &m_animation[0] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '3':
      if(b[THREE_KEY])
        m_blend_scheduler.add( &m_animation[1] );
      else
        m_blend_scheduler.remove( &m_animation[1] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '4':
      if(b[FOUR_KEY])
        m_blend_scheduler.add( &m_animation[2] );
      else
        m_blend_scheduler.remove( &m_animation[2] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '5':
      if(b[FIVE_KEY])
        m_blend_scheduler.add( &m_animation[3] );
      else
        m_blend_scheduler.remove( &m_animation[3] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '6':
      if(b[SIX_KEY])
        m_blend_scheduler.add( &m_animation[4] );
      else
        m_blend_scheduler.remove( &m_animation[4] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '7':
      if(b[SEVEN_KEY])
        m_blend_scheduler.add( &m_animation[5] );
      else
        m_blend_scheduler.remove( &m_animation[5] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    case '8':
      if(b[EIGHT_KEY])
        m_blend_scheduler.add( &m_animation[6] );
      else
        m_blend_scheduler.remove( &m_animation[6] );
      m_duration = m_blend_scheduler.compute_duration();
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry("Load Character 1             [1]", '1');
    glutAddMenuEntry("Toggle idle                  [2]", '2');
    glutAddMenuEntry("Toggle jog                   [3]", '3');
    glutAddMenuEntry("Toggle shoot arrow           [4]", '4');
    glutAddMenuEntry("Toggle strut                 [5]", '5');
    glutAddMenuEntry("Toggle tornado kick          [6]", '6');
    glutAddMenuEntry("Toggle walk                  [7]", '7');
    glutAddMenuEntry("Toggle wave                  [8]", '8');
    glutAddMenuEntry("Clear all animations         [c]", 'c');
    glutAddMenuEntry("Single time-step             [t]", 't');
    glutAddMenuEntry("Toggle display bones         [b]", 'b');
    glutAddMenuEntry("Toggle display skin          [s]", 's');
    glutAddMenuEntry("Reset time                   [r]", 'r');
    glutSetMenu( main_menu );
    glutAddSubMenu( "animation", controls );
  }

  void do_init()
  {
    this->camera().move( -400 );

    m_blend_scheduler.clear();
    m_duration = 0.0; 
    b[TWO_KEY] = false;
    b[THREE_KEY] = false;
    b[FOUR_KEY] = false;
    b[FIVE_KEY] = false;
    b[SIX_KEY] = false;
    b[SEVEN_KEY] = false;
    b[EIGHT_KEY] = false;

    std::string data_path = opentissue_path;


    OpenTissue::skeleton::xml_read(data_path + "/demos/data/xml/character1.xml",m_skeleton);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_idle.xml",m_animation[0]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_jog.xml",m_animation[1]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_shoot_arrow.xml",m_animation[2]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_strut.xml",m_animation[3]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_tornado_kick.xml",m_animation[4]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_walk.xml",m_animation[5]);
    OpenTissue::animation::keyframe_animation_xml_read(data_path + "/demos/data/xml/character1_wave.xml",m_animation[6]);

    //--- ideally the weights should be controlled by the scheduler,
    //--- but for demo purpose we just set the weights here!!!
    m_animation[0].set_weight(1.0);
    m_animation[1].set_weight(1.0);
    m_animation[2].set_weight(5.0);
    m_animation[3].set_weight(1.0);
    m_animation[4].set_weight(1.0);
    m_animation[5].set_weight(1.0);
    m_animation[6].set_weight(5.0);

    for(size_t i=0u; i<m_skin.m_sz; ++i)
    {
      m_skin.m_skin_parts[i].clear();
      m_skin.m_material[i].set_default();
    }
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_calf_left.xml",m_skin.m_skin_parts[0]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_hand_right.xml",m_skin.m_skin_parts[1]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_ponytail.xml",m_skin.m_skin_parts[2]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_calf_right.xml",m_skin.m_skin_parts[3]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_head.xml",m_skin.m_skin_parts[4]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_thigh_left.xml",m_skin.m_skin_parts[5]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_chest.xml",m_skin.m_skin_parts[6]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_lowerarm_left.xml",m_skin.m_skin_parts[7]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_thigh_right.xml",m_skin.m_skin_parts[8]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_foot_left.xml",m_skin.m_skin_parts[9]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_lowerarm_right.xml",m_skin.m_skin_parts[10]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_upperarm_left.xml",m_skin.m_skin_parts[11]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_foot_right.xml",m_skin.m_skin_parts[12]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_neck.xml",m_skin.m_skin_parts[13]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_upperarm_right.xml",m_skin.m_skin_parts[14]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_hand_left.xml",m_skin.m_skin_parts[15]);
    OpenTissue::skinning::xml_read(data_path + "/demos/data/xml/character1_pelvis.xml",m_skin.m_skin_parts[16]);

    OpenTissue::skinning::material_xml_read(data_path + "/demos/data/xml/character1_skin_material.xml",m_skin.m_material[0]);
    OpenTissue::skinning::material_xml_read(data_path + "/demos/data/xml/character1_ponytail_material.xml",m_skin.m_material[1]);
    OpenTissue::skinning::material_xml_read(data_path + "/demos/data/xml/character1_chest_material.xml",m_skin.m_material[2]);
    OpenTissue::skinning::material_xml_read(data_path + "/demos/data/xml/character1_pelvis_material.xml",m_skin.m_material[3]);

    m_time       = 0;
    m_delta_time = 0.02;
    m_duration   = 0;

    m_display_bones    = false;
    m_display_skin     = true;


    //skin_type::init_skin_render();
    m_skin.init();

    m_skin_render.init( m_skin );

    // Create vertex array for GPU uploading
    //m_skin_render.createGPUBuffers( m_skin );
  }

  void do_run()
  {
    if(m_time < m_duration)
    {
      m_blend_scheduler.compute_pose(m_skeleton,m_time);
      m_time += m_delta_time;
    }
    else
      m_time = 0.0; //--- we just loop the animation playback (not the same as making a cycle!)
  }

  void do_shutdown()
  {
    //skin_type::cleanup_skin_render();
    m_skin.cleanup();
    m_skin_render.cleanup();
  }

};


OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
