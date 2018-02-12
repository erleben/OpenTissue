<h1>Character animation programming guide</h1>

<h2>Introduction</h2>
The character animation system in OpenTissue supports deformation of skin meshes using a skeleton of bones. OpenTissue supports both Linear Blend Skinning and Spherical Blend Skinning implemented in both software and hardware (for NVIDIA GPUs series 6 and higher). Software skinning support an arbitrary number of bone influences for each skin vertex, while hardware skinning support up to 4 bone influences per skin vertex. It must be decided at compile time whether software or hardware skinning is to be used.

To employ the character animation system, the following header file must be included:
<pre>
#include <OpenTissue/character/character_types.h>
</pre>
A ready-to-compile-and-run demo using the character animation system is located in:
<pre>
/demos/opengl/character/
</pre>
Here it is shown how the system can be used to animate a running/walking human character.

<h2>Setting up the system</h2>
The character animation uses skeletons and skin meshes. To define a skeleton type, a mandatory math type argument must be provided. To use the default math type, the following type can be defined:
<pre>
typedef OpenTissue::Math::default_math_types    math_types;
</pre>
This type can then be provided to the skeleton:
<pre>
typedef OpenTissue::SkeletonTypes<math_types>    skeleton_types;
</pre>
To define a skin type, the math type must again be used along with an argument denoting the skinning method to be used. In the following hardware supported Spherical Blend Skinning is used:
<pre>
typedef OpenTissue::SkinTypes<math_types, OpenTissue::SBSGPU>    skin_types;
</pre>
Finally, the character types can be defined using:
<pre>
typedef OpenTissue::CharacterTypes<math_types,skin_types,skeleton_types>    character_types;
</pre>
And relevant types can then be extracted, and private members can be setup:
<pre>
typedef character_types::skeleton_type skeleton_type;
typedef character_types::bone_type bone_type;
typedef character_types::skin_type skin_type;
typedef character_types::keyframe_animation_type keyframe_animation_type;
typedef character_types::naive_blend_scheduler_type naive_blend_scheduler_type;
typedef character_types::skin_render_type skin_render_type;


skeleton_type               m_skeleton;         ///< Skeleton (i.e. bones).
keyframe_animation_type     m_animation[100];   ///< The raw animations.
naive_blend_scheduler_type  m_blend_scheduler;  ///< The animation blend scheduler (combines raw animations into final animation).
real_type                   m_time;             ///< The current animation time.
real_type                   m_delta_time;       ///< Time in between two poses (i.e. 1/fps).
real_type                   m_duration;         ///< Duration of animation.              ///< Skin container.
bool                        m_display_bones;    ///< Boolean flag used to turn on/off rendering of bones.
bool                        m_display_skin;     ///< Boolean flag used to turn on/off rendering of skin.
skin_type                   m_skin;             ///< The skin being deformed by the skeleton
skin_render_type            m_skin_render;      ///< The render used for skinning
</pre>

<h2>Loading character data</h2>
Before any animation can be done, skeleton and skin data must be loaded into the system. OpenTissue supports two formats: An xml formal and CAL3D's animation format. In the following it is shown how the xml reader can be used to load data into the character animation system.

First of all, the path to the folder containing data must be extracted:
<pre>
std::string data_path = OpenTissue::get_environment_variable("OPENTISSUE");
</pre>
Then data can be loaded into the skeleton container:
<pre>
skeleton_xml_read(data_path + "/data/xml/character1.xml",m_skeleton);
</pre>
Animation data can be loaded into the animation container:
<pre>
keyframe_animation_xml_read(data_path + "/data/xml/character1_idle.xml",m_animation[0]);
keyframe_animation_xml_read(data_path + "/data/xml/character1_jog.xml",m_animation[1]);
keyframe_animation_xml_read(data_path + "/data/xml/character1_shoot_arrow.xml",m_animation[2]);
...
</pre>
Skin and material data can be loaded into the skin container:
<pre>
skin_xml_read(data_path + "/data/xml/character1_calf_left.xml",m_skin.m_skin_parts[0]);
skin_xml_read(data_path + "/data/xml/character1_hand_right.xml",m_skin.m_skin_parts[1]);
skin_xml_read(data_path + "/data/xml/character1_ponytail.xml",m_skin.m_skin_parts[2]);
...

material_xml_read(data_path + "/data/xml/character1_skin_material.xml",m_skin.m_material[0]);
material_xml_read(data_path + "/data/xml/character1_ponytail_material.xml",m_skin.m_material[1]);
material_xml_read(data_path + "/data/xml/character1_chest_material.xml",m_skin.m_material[2]);
...
</pre>

finally, some initialization must be done...

<h2>Playing animations</h2>

<h2>Animation blending</h2>
...
