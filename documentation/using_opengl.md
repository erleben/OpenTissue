==The OpenGL Programming Guide==
===Working with OpenGL in OpenTissue===
OpenGL is the default graphics library used in OpenTissue. However, you are not
required or bound to use OpenGL if you decide to use OpenTissue. A clean
separation of the OpenGL visualization have been sought.

In this document we will describe how the OpenGL support has been implemented
and how to use it.

The OpenGL support in OpenTissue provides simple visualization methods that are
useful for visual debugging of animation methods. For instance methods for
visualizing built in vector and quaternion types or geometry types such as
sphere, boxes etc.. Most of these visualization methods have been implemented
for convenience and not for speed.

The primitives currently supported are
* Point, Vector, CoordSys
* AABB, OBB, Prism, Sphere, Tetrahedron, Triangle, Cylinder, Capsule, Ellipsoid, Plane
* String

There is also support for more advanced OpenGL features, such as handling of
* Frame buffer and render buffer object
* 2D and 3D Textures (see <a href="images_and_textures.html">The Image and Texture Programming Guide</a> and <a href="volume_visualization.html">The Volume Visualization Programming Guide</a>)
* Camera
* Frustum
* Trackball
* OpenGL matrix routines
* Picking, Camera to object space conversion, Material and Color utilities

A single OpenGL wrapper file is provided in the header file
<pre>
#include<OpenTissue/utility/gl_util.h>
</pre>
This header file will automatically include all OpenGL utilities located in the folder
<pre>
OpenTissue/utility/GL
</pre>

If only OpenGL support is wanted and none of the OpenTissue utilities is
required one can simply include the header file
<pre>
#include<OpenTissue/utility/GL/gl.h>
</pre>

OpenTissue handles OpenGL extensions by using GLEW. This requires that header
files are included in the correct order. It is therefore very important,
whenever you use OpenGL, to include either
<pre>
#include<OpenTissue/utility/gl_util.h>
</pre>
or
<pre>
OpenTissue/utility/GL
</pre>

As the very first header file. If you are using Cg you can simply use the header file
<pre>
#include<OpenTissue/utility/cg_util.h>
</pre>
Instead. This is explained in more detail in <a href="shader_programming.html">The Shader Programming Guide</a>.

===OpenGL Matrix Utilities===
In some applications it is convenient to extract 4-by-4 OpenGL matrices stored
in column major form and work directly with this brute representation.
OpenTissue provides a small collection of functions for doing so, these are
defined in the header file
<pre>
#include<OpenTissue/utility/GL/gl_matrix_util.h>
</pre>

A typical example usage is to extract model view and projection matrices and
manipulate these, for instance
<pre>
GLfloat model2eye_matrix[16];
GLfloat eye2model_matrix[16];
glGetFloatv( GL_MODELVIEW_MATRIX, model2eye_matrix );
invert( model2eye_matrix, eye2model_matrix );
</pre>

Having obtained the inverted model view transformation matrix, it can be used
directly to transform vertices by writing
<pre>
vector3_type V[N];
for(unsigned int i=0;i<N;++i)
V[i] = xform( model2eye_matrix, V[i]);
</pre>

Other functions include matrix multiplication (see mul), matrix transpose (see
transpose), and orthonormalization (see orthonormalize).

===Material, Color Picking and Error Checking===
A easy to use OpenGL material wrapper is defined in the header file
<pre>
#include<OpenTissue/utility/GL/gl_material.h>
</pre>

During initialization a material can be defined by writing
<pre>
...
float red   = ...;
float green = ...;
float blue  = ...;
float alpha = ...;
glMaterial mat;
mat.ambient( red, green, blue, alpha );
mat.specular( red, green, blue, alpha );
mat.diffuse( red, green, blue, alpha );
mat.shininess( 10 );
...
</pre>

When rendering the material can be applied by writing
<pre>
mat.use();
... render ...
</pre>

In case you really do not care about whether OpenGL materials are used or
not, a more simple utility is provided to set the color. It is defined in the
header file
<pre>
#include<OpenTissue/utility/GL/gl_color_picker.h>
</pre>
And you use it by writing
<pre>
glColorPicker( 1.0, 0.0, 0.0, 1.0);
... render ...
</pre>

This will render with a non-transparent red color. If you do not care about
specified alpha-values you can simply omit the fourth argument, and it will
default to one.

It may happen that you need to check whether some OpenGL error occurred. For this
purpose a convenience utility is provided in the header file
<pre>
#include<OpenTissue/utility/GL/check_gl_errors.h>
</pre>
And you use it by writing
<pre>
... some OpenGL stuff that might go wrong ...
check_gl_errors("Here I am");
</pre>

The function will query the OpenGL state and test whether an error has occurred.
If this is the case an error message is written to a stream (default is
std::cout) using the specified string value as a pre-fix for the error message.
One can omit the string value, or even specify another output stream.

If one have a set of x- and y screen coordinates and want to find the correspond
x, y, and z- coordinates of the world point currently visible at the pixel at
the specified screen coordinates, then one can use the utility in the header
file
<pre>
#include<OpenTissue/utility/GL/gl_screen2object.h>
</pre>
The utility is used as follows
<pre>
int sx = 5,
int sy = 15;
int ox,oy,oz;
glScreen2Object()(sx,sy, ox, oy, oz);

std::cout << "pixel at (" << sx << "," << sy << ")corresponds to point (" << ox << "," << oy << ","<< oz << ")" << std::endl;
</pre>

Finally there is a picking tool available in the header file
<pre>
#include<OpenTissue/utility/GL/gl_picking.h>
</pre>

We refer the reader to the header file for details.

===Creating an OpenGL based Application using OpenTissue===
A typical small application class may look like this
<pre>
class Application
{
public:
Application(){}
~Application(){ }
public:
void display()
void init()
protected:
typedef MyTypes                   types;
typedef types::real_type          real_type;
protected:
OpenTissue::glCamera<types>        m_camera;
bool                               m_zoom_mode;
bool                               m_pan_mode;
bool                               m_trackball_mode;
real_type                          m_begin_x;
real_type                          m_begin_y;
public:
void mouse_down(real_type sx,real_type sy,bool shift,bool left,bool middle,bool right)
void mouse_up(real_type sx,real_type sy,bool shift,bool left,bool middle,bool right)
void mouse_move(real_type sx,real_type sy)
};
</pre>

Observe that it consist of an initialization method, a display event handler and
mouse event handlers. In the initialization method one would set up the initial
state of the GUI, i.e. something like this
<pre>
void Application::init()
{
...
m_zoom_mode              = false;
m_pan_mode               = false;
m_trackball_mode         = false;
m_camera.target_locked() = true;
m_camera.init(vector3_type(0,0,10),vector3_type(0,0,0),vector3_type(0,1,0));
...
}
</pre>

Notice that the initial behavior and position of the camera is set. Now in the
display handler the camera can by used to setup the correct model-view
transformation, as follows
<pre>
void Application::display()
{
glMatrixMode( GL_MODELVIEW );
glLoadCameraMatrix(m_camera);
... do some fancy rendering ...
}
</pre>

Of course we want to be able to move the camera around, a perfect place for
changing the camera is in the mouse event handlers. For instance one could write
<pre>
void Application::mouse_down(real_type cur_x,real_type cur_y,bool shift,bool left,bool middle,bool /*right*/)
{
if (middle )
m_zoom_mode = true;
if ( shift &amp;&amp; left )
m_pan_mode = true;
if(!shift &amp;&amp; left)
{
m_camera.mouse_down( cur_x, cur_y );
m_trackball_mode = true;
}
m_begin_x = cur_x;
m_begin_y = cur_y;
}

void Application::mouse_up(real_type cur_x,real_type cur_y,bool /*shift*/,bool /*left*/,bool /*middle*/,bool /*right*/)
{
if ( m_zoom_mode )
{
m_camera.move( 0.25*(cur_y - m_begin_y) );
m_zoom_mode = false;
}
if ( m_pan_mode )
{
m_camera.pan( 0.25*(m_begin_x - cur_x) , 0.25*(cur_y - m_begin_y) );
m_pan_mode = false;
}
if ( m_trackball_mode )
{
m_camera.mouse_up( cur_x, cur_y );
m_trackball_mode = false;
}
m_begin_x = cur_x;
m_begin_y = cur_y;
}

void Application::mouse_move(real_type cur_x,real_type cur_y)
{
if ( m_zoom_mode )
m_camera.move( 0.25*(cur_y - m_begin_y) );
if ( m_pan_mode )
m_camera.pan( 0.25*(m_begin_x - cur_x) , 0.25*(cur_y - m_begin_y) );
if ( m_trackball_mode )
m_camera.mouse_move( cur_x, cur_y);
m_begin_x = cur_x;
m_begin_y = cur_y;
}
</pre>
This will support zooming, panning, and orbiting. For more details we refer
the reader to the header file
<pre>
#include<OpenTissue/utility/GL/gl_camera.h>
</pre>

If one wants to do frustum culling on the CPU then an utility class for doing
this is provided in the header file
<pre>
#include<OpenTissue/utility/GL/gl_frustum.h>
</pre>
It works simply by updating an instance of the frustum class before testing,
for instance one could add the following code to the display handler
<pre>
...
glFrustum<types> frustum;
frustum.update();
if( frustum.contains( aabb ) )
{
... draw stuff inside AABB
}
...
</pre>

Culling currently only supports spheres, AABBs and points.

===Using Framebuffer Object===
OpenTissue provides wrapper classes for OpenGL Framebuffer object and
Renderbuffer, these are located in the header files
<pre>
#include<OpenTissue/utility/GL/gl_frame_buffer_object.h>
</pre>
and
<pre>
#include<OpenTissue/utility/GL/gl_render_buffer.h>
</pre>

Framebuffer object is a convenient way to change the render target. Thus it
provides functionality for setting up render to texture (RTT) functionality or
ping-pong schemes.

As an example we will set up a render-to-texture ping-pong scheme using both a
depth and a stencil buffer. We thus need two textures, two Renderbuffers and one
Framebuffer object,
<pre>
texture2D_pointer    m_texture[2];
glFramebufferObject  m_fbo;
renderbuffer_pointer m_depth;
renderbuffer_pointer m_stencil;
</pre>

During initialization we must create the textures, the depth and stencil buffers
and attach all of them to the Framebuffer Object.
<pre>
void Application::init()
{
...
static const GLenum buffer[] = { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
for(unsigned int i=0;i<2;++i)
m_texture[i] = create_float_texture_rectangle( width, height, 4, 0 );
m_depth = create_depth_buffer(width,height);
m_stencil = create_stencil_buffer(width,height);
m_fbo.bind();
for(unsigned int i=0;i<2;++i)
m_fbo.attach_texture( buffer[i], m_texture[i] );
m_fbo.attach_render_buffer( GL_DEPTH_ATTACHMENT_EXT, m_depth );
m_fbo.attach_render_buffer( GL_STENCIL_ATTACHMENT_EXT, m_stencil );
m_fbo.is_valid();
glFramebufferObject::disable();
...
}
</pre>

Of course one do not need to attach depth and stencil buffers, these can be
omitted if there are no need for them. Notice that after having attached all
resources to the Framebuffer object the method is_valid() is invoked. This
method will test (in debug build mode only) if the Framebuffer object is a valid
render target. The global disable() call turns of all Framebuffer Objects, this
is what you typically want during some initialization.

After having initialized the Framebuffer Object you can use it in your display
handler as follows
<pre>
void Application::display()
{
...
static const GLenum buffer[] = { GL_COLOR_ATTACHMENT0_EXT, GL_COLOR_ATTACHMENT1_EXT };
int write = 0;
int read  = 1;

m_fbo.bind();
for(unsigned int i=0;i<ping_pongs;++i)
{
glDrawBuffer (buffer[write]);
draw_some_thing( ..., m_texture[read]);
std::swap(m_write,m_read);
}
glFramebufferObject::disable();
}
</pre>
Notice that before rendering the function glDrawBuffer is used to tell which
buffer should receive the ``rendering''. If you desire to read back from a
buffer (from GPU to CPU) you should use the function glReadBuffer to tell which
buffer you want to read from.

In the above example we have been a little careless. A Framebuffer object shares
the OpenGL state with all other Framebuffer objects (incl. the default system).
Therefore one have to protect the OpenGL state when switching to another
Framebuffer object unless it is the intention that the same OpenGL state should
be used. The state can be protected conveniently by using glPushAttrib and
glPopAttrib.

As a final note, if you do not desire to render-to-texture, but just need
another color buffer, then you can use render buffer for this. As an example
<pre>
void Application::some_method()
{
glFramebufferObject  fbo;
renderbuffer_pointer color;

glPushAttrib(GL_ALL_ATTRIB_BITS);
color.reset( new glRenderbuffer(GL_RGBA8_EXT, w, h) );
fbo.bind();
fbo.attach_render_buffer(GL_COLOR_ATTACHMENT0_EXT, color);
fbo.is_valid();
glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT);
... draw something ...
glFramebufferObject::disable();
}
</pre>

The Framebuffer Object class also have support for Texture3D, we redirect the
reader to the header file
<pre>
#include<OpenTissue/utility/GL/gl_frame_buffer_object.h>
</pre>
For more details.


<i>Please note:</i> Currently the FramebufferObject class does not support cube map textures. Also, on some drivers the FBO extension does not support GL_TEXTURE_RECTANGULAR_ARB textures. This means that setting the "rectangular" flag when creating a Texture2D object will make the texture unsuitable for use with FramebufferObject. Specifying dimensions for a non-square texture (i.e. width = 256, height = 128 and rectangular = false), however, will most often work.
