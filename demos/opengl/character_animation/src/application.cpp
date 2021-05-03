//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2021 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/graphics/glfw/glfw_application.h>
#include <OpenTissue/kinematics/animation/animation_keyframe_animation.h>
#include <OpenTissue/kinematics/animation/animation_naive_blend_scheduler.h>
#include <OpenTissue/kinematics/animation/io/animation_keyframe_animation_cal3d_xml_read.h>
#include <OpenTissue/kinematics/animation/io/animation_keyframe_animation_xml_read.h>
#include <OpenTissue/kinematics/skeleton/io/skeleton_cal3d_xml_read.h>
#include <OpenTissue/kinematics/skeleton/io/skeleton_xml_read.h>
#include <OpenTissue/kinematics/skeleton/skeleton_types.h>
#include <OpenTissue/kinematics/skinning/io/skinning_cal3d_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_material_cal3d_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_material_xml_read.h>
#include <OpenTissue/kinematics/skinning/io/skinning_xml_read.h>
#include <OpenTissue/kinematics/skinning/skinning_types.h>

class Application : public OpenTissue::graphics::GlfwApplication
{
public:
  typedef float                                                        real_type;
  typedef OpenTissue::math::default_math_types                         math_types;
  typedef math_types::matrix3x3_type                                   matrix3x3_type;
  typedef math_types::quaternion_type                                  quaternion_type;
  typedef math_types::vector3_type                                     vector3_type;
  typedef OpenTissue::skeleton::Types<math_types>                      skeleton_types;
  typedef OpenTissue::graphics::GlfwApplication                        Base;

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
  typedef skin_type::skin_part_type                        skin_part_type;
  typedef skin_types::skin_render_type                     skin_render_type;

  typedef OpenTissue::animation::KeyframeAnimation<skeleton_type>     keyframe_animation_type;
  typedef OpenTissue::animation::NaiveBlendScheduler<skeleton_type>   naive_blend_scheduler_type;

protected:

  //--- Data for an animated character...
  skeleton_type               m_skeleton;         ///< Skeleton (i.e. bones).
  keyframe_animation_type     m_animation[100];   ///< The raw animations.
  naive_blend_scheduler_type  m_blend_scheduler;  ///< The animation blend scheduler (combines raw animations into final animation).
  real_type                   m_delta_time;       ///< Time inbetween two poses (i.e. 1/fps).
  real_type                   m_duration;         ///< Duration of animation.
  skin_type                   m_skin;             ///< Skin container.
  bool                        m_display_bones;    ///< Boolean flag used to turn on/off rendering of bones.
  bool                        m_display_skin;     ///< Boolean flag used to turn on/off rendering of skin.
  bool                        m_single_step;      ///< Boolean flag used to turn on/off single stepping.

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


  Application() : Base("Character Animation Application")
  {
    this->m_z_far = 5000;
  }

  ~Application()
  {
  }

  void update(double time) override
  {
    if(m_display_bones)
    {
      for(const auto &bone : m_skeleton)
      {
        OpenTissue::gl::DrawBone(bone);
        OpenTissue::gl::DrawFancyBone(bone);
      }
    }

    if(m_display_skin)
    {
      m_skin_render.render( m_skin, m_skeleton );
    }

    if(time < m_duration)
    {
      m_blend_scheduler.compute_pose(m_skeleton, m_time);
      time += m_delta_time;
    }
    else
    {
      time = 0.0;
    }
    m_time = time;
  }

  void action(unsigned char choice) override
  {
    // Toggle state
    b[choice] = ! b[choice];
    switch ( choice )
    {
      case 't':
      {
        this->update(m_time); break;
      };
      case 'b': m_display_bones = !m_display_bones;    break;
      case 's': m_display_skin = !m_display_skin;      break;
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
      case 'r': m_time = 0;  break;
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

  void init_right_click_menu()
  {
    std::unordered_map<unsigned char, const char*> menu_map;
    menu_map.insert(std::make_pair('1', "Load Character 1             [1]"));
    menu_map.insert(std::make_pair('2', "Toggle idle                  [2]"));
    menu_map.insert(std::make_pair('3', "Toggle jog                   [3]"));
    menu_map.insert(std::make_pair('4', "Toggle shoot arrow           [4]"));
    menu_map.insert(std::make_pair('5', "Toggle strut                 [5]"));
    menu_map.insert(std::make_pair('6', "Toggle tornado kick          [6]"));
    menu_map.insert(std::make_pair('7', "Toggle walk                  [7]"));
    menu_map.insert(std::make_pair('8', "Toggle wave                  [8]"));
    menu_map.insert(std::make_pair('c', "Clear all animations         [c]"));
    menu_map.insert(std::make_pair('t', "Single time-step             [t]"));
    menu_map.insert(std::make_pair('b', "Toggle display bones         [b]"));
    menu_map.insert(std::make_pair('s', "Toggle display skin          [s]"));
    menu_map.insert(std::make_pair('r', "Reset time                   [r]"));

    // this->add_sub_menu("animation", menu_map);
  }

  void init() override
  {
    this->init_right_click_menu();
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

    std::vector<std::string> skin_files = {
      data_path + "/demos/data/xml/character1_calf_left.xml",
      data_path + "/demos/data/xml/character1_hand_right.xml",
      data_path + "/demos/data/xml/character1_ponytail.xml",
      data_path + "/demos/data/xml/character1_calf_right.xml",
      data_path + "/demos/data/xml/character1_head.xml",
      data_path + "/demos/data/xml/character1_thigh_left.xml",
      data_path + "/demos/data/xml/character1_chest.xml",
      data_path + "/demos/data/xml/character1_lowerarm_left.xml",
      data_path + "/demos/data/xml/character1_thigh_right.xml",
      data_path + "/demos/data/xml/character1_foot_left.xml",
      data_path + "/demos/data/xml/character1_lowerarm_right.xml",
      data_path + "/demos/data/xml/character1_upperarm_left.xml",
      data_path + "/demos/data/xml/character1_foot_right.xml",
      data_path + "/demos/data/xml/character1_neck.xml",
      data_path + "/demos/data/xml/character1_upperarm_right.xml",
      data_path + "/demos/data/xml/character1_hand_left.xml",
      data_path + "/demos/data/xml/character1_pelvis.xml"
    };

    size_t i = 0;
    for(const auto &file : skin_files)
    {
      m_skin.m_skin_parts[i].clear();
      OpenTissue::skinning::xml_read(file, m_skin.m_skin_parts[i++]);
    }

    std::vector<std::string> material_files = {
      data_path + "/demos/data/xml/character1_skin_material.xml",
      data_path + "/demos/data/xml/character1_ponytail_material.xml",
      data_path + "/demos/data/xml/character1_chest_material.xml",
      data_path + "/demos/data/xml/character1_pelvis_material.xml"
    };

    i = 0;
    for(const auto &file : material_files)
    {
      m_skin.m_material[i].set_default();
      OpenTissue::skinning::material_xml_read(file, m_skin.m_material[i++]);
    }

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

};

OpenTissue::graphics::GlfwApplication::Ptr createApplication(int argc, char **argv)
{
  auto app = OpenTissue::graphics::GlfwApplication::New<Application>();
  return app;
}
