==The Shader Programming Guide==
===Setting Up Cg and Create a Shader Class===
OpenTissue supports Cg programming in order to get Cg up and running you need
the include header
<pre>
#include<OpenTissue/utility/cg_util.h>
</pre>
This include file will make sure to include any other OpenGL or other
dependencies you need. However, it is very important that you&nbsp;make sure
this include&nbsp;is the first include. The reason is due to automatic
handling of extensions by using GLEW. OpenGL includes must not occur before any
glew.h includes. Making sure cg_util.h is the first include guarantees that you
do not get into trouble. The next step is to get Cg started, this can be done
during initialization of your application, like this
<pre>
void Application::init()
{
Cg::startup();
...
}
</pre>

Of course you also need to clean up after you. This could for instance be done
in the destructor of the application:
<pre>
Application::~Application()
{
Cg::shutdown();
};
</pre>

Note you can not instantiate any Cg programs before you have stared Cg. This
is because Cg programs need a Cg context, which is created by invoking the
startup function. The approach we advocate for implementing shaders is to
create a class wrapper. A shader consist of both a vertex program and a
fragment program. These programs are represented by an instance of the class
CgProgram. This class is defined in the header file:
<pre>
#include<OpenTissue/utility/cg_program.h>
</pre>
Now we can create the basic interface of a shader:
<pre>
class  MyShader
{
protected:
CgProgram m_vertex_program;
CgProgram m_fragment_program;
public:
void pre_render(.... ) { .... }
void post_render( .... ) { .... }
};
</pre>

Because the class CgProgram needs a Cg Context, MyShader must be instantiated
after invocation of Cg::startup(). Thus we add the shader as a pointer&nbsp;in
the Application class.
<pre>
class Application
{
public:
typedef boost::shared_ptr<MyShader> shader_pointer;
...
protected:
shader_pointer  m_shader;
...
};
</pre>

The shader must be created during initialization as follows
<pre>
void Application::init()
{
Cg::startup();
m_shader.reset( new MyShader() );
...
}
</pre>

We do not need to worry about deallocation, since we are using a boost shared
pointer. After we have created the shader we are now ready to use it when we
render some geometry. This is done by adding the steps to the Application
display handler
<pre>
void Application::display()
{
m_shader->pre_render(...);
... draw something ...
m_shader->post_render();
}
</pre>

Carefully notice that the shader wraps around the geometry it should be
applied to. It consist of two phases a pre-render, which makes sure everything
is setup correctly and a post-render which cleans up after the shader, just in
case you want to do some other rendering afterwards.

===Implementing the Shader Class===
So far we have just set up the framework, now it is about time to tell the
framework what Cg programs to use, what textures to setup and how to fix OpenGL
state. etc..

Here is an example pre-render setup
<pre>
template<typename typename texture3d_pointer, texture2d_pointertypename>

void pre_render ( texture2d_pointer &amp; color_table, texture3d_pointer &amp; volume )
{
glEnable( GL_BLEND );
glBlendFunc( GL_ONE, GL_ONE_MINUS_SRC_ALPHA );

static string path = get_environment_variable("OPENTISSUE") + " path to cg programs";
if(!m_vertex_program.is_program_loaded())
m_vertex_program.load_from_file(CgProgram::vertex_program,path + "/my vertex shader.cg");

m_vertex_program.set_modelview_projection_matrix();
m_vertex_program.enable();

if(!m_fragment_program.is_program_loaded())
m_fragment_program.load_from_file(CgProgram::fragment_program,path + "/my fragment shader.cg");

m_fragment_program.set_input_texture( "color_table"  ,  color_table );
m_fragment_program.set_input_texture( "volume"       ,  volume );
m_fragment_program.enable();
}
</pre>

You should take notice of the following important facts
* You can add any number of arguments to the pre-render method, even template arguments. In the example we passed along pointers to textures as template arguments. This makes your shader implementation independent of the type of texture class you use.
* First we setup the OpenGL state. This is because we later on might want to set Cg program arguments that depends on the OpenGL state.
* Next we test to see if the Cg program is loaded, if not we load it from a file.
* Hereafter we setup any Cg program parameters (uniforms, textures etc..) before the Cg program is finally enabled.

The post render phase which clean up, would reverse the order of events, that
is it would first disable the Cg programs, before turning back the&nbsp;OpenGL
state.
<pre>
void post_render()
{
m_fragment_program.disable();
m_vertex_program.disable();
glDisable( GL_BLEND );
}
</pre>

The above example is not very meaningful, we used it merely to show how to
implement things, the code do not do anything useful.
