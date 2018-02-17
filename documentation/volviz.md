# Volume Visualization&nbsp;Programming Guide
## Loading a 3D Image
In OpenTissue 3D images are most conveniently stored in a Map data structure.
The implementation of this data structure is located in the header file:

    #include<OpenTissue/map/map.h>

It is a template data structure, which means you have to supply it with a few
types indicating what kind of data you want to work with. For instance

    typedef double                                  real_type;
    typedef unsigned short                          value_type;
    typedef OpenTissue::Map<value_type, real_type>  map_type;

The above type definition is great for reading in raw files. These are usually
stored as one component images, each component is a non-negative integer
intensity value of up to 16 bit precision. However since most of these images
are obtained from CT-scans or conversion of DICOM images, only 12 bits are
really used. We will talk more about this later on. After having defined the
map_type we can declare a map in our application.

    class Application
    {
      protected:
      ...
      map_type   m_volume;
      ...
    };

At some point during initialization or on some user request for reading an 3D
image, we need to allocate memory in the map. A typical raw file loading can be
done using

<pre>
void Application::init()
{
  string path = get_environment_variable("DATATISSUE");
  string filename = path+"/demos/common/raw/Teddybear.raw";
  m_volume.create( 128, 128, 62, 2.8,  2.8, 5.0 );
  read8bit_to16bit(m_volume, filename);
};
</pre>

In the first two lines we obtain a string representing the location where the
raw-file is stored on our hard-drive. In the third line we ask for a 3D image
(ie. a volume) containing 128x128x62 voxels with each voxel having a size of
(2.8, 2.8, 5.0) units. The unit used depends on where the image is obtained
from. The fourth line is an easy wrapper for reading in raw images. In this
particular example we know that we have 8bit raw file, so we invoke a function
that automatically convert the raw data into 16 bit. There are more functions
available for loading raw files, and these can be located in the header file

    #include<OpenTissue/map/io/rawMapIO.h>

These include

<pre>
read8bit(...);
read8bit_to16bit(...);
read16bit(...);
</pre>

The important things to remember from this example are
* The end user must know the precision of the voxel data stored in a raw file.
* <p> The end user must know the resolution and voxel size of a raw file. The raw file format only stores raw data.</p> <p> This of cause make it difficult to work with raw-files. Another important issue here is</p>
* The map data structure is only convenient for storing 1 component volumes.

In practice this is not a large limitation, since most data is available some
intensity images.

## Setting up a Color Table
Once a 3D image has been loaded, then we need some way to interpret the
intensity values stored in the 3D image as colors. For this we use a color
table. The idea is simply to take the value of a voxel in the 3D image and use
it to lookup a corresponding color in the color table. The volume visualization
tools described below uses such an approach and they represent a color table as
a 2D image having exactly one row, with as many pixels as there are intensity
values (12 bit = 4096 pixels), actually the volume visualization tools do not
really use the color table images but rather a 2D texture which the images are
loaded into.

This may sound very complicated, but it is really easy to set up. First you
need the include header

    #include<OpenTissue/visualization/util/color_table.h>

Herein you find the typedefs for a color table image, and a collection of
functions that create color table images for you. So now you would add the
following members to your application

<pre>
class Application
{
protected:
  ...
  color_table_type       m_color_table;
  texture2D_pointer      m_color_table_texture;
  ...
};
</pre>

Then during initialization you can simply setup the color table image and
texture by writing

<pre>
void Application::init()
{
  ...
  compute_linear_color_table(4096,m_color_table);
  m_color_table_texture = m_color_table.create_texture(GL_RGBA);
  ...
};
</pre>

This creates a very simple color table, there are other functions such as

<pre>
compute_noise_color_table(4096,m_color_table);
compute_historgram_color_table( histogram, m_color_table);
</pre>

that is capable of creating a more interesting color map. Or you could simply
read in a color map form an image file

<pre>
image_read(file_name, m_color_table);
</pre>

Note that you can also store color tables as image files, and reload them
later on if you so desire. Some volume visualization methods expects a
pre-integrated color table, the include header

    #include<OpenTissue/visualization/util/preintegrated_color_table.h>

Contains a utility function for creating a texture of a pre-integrated color
table, simply just write

<pre>
m_color_table_texture = compute_1D_preintegration_color_table(m_color_table);
</pre>

You should notice that
* A color table is really just a 2D image with only one row
* You can alter, change, load, and manipulate color tables with any image utilities and io-routines already in OpenTissue.
* You need a pointer to a 2D texture object with the color table image loaded into the texture object.


## Volume Rendering
First you need to decide what type of volume render you want to use.
Currently OpenTissue provides two types: A Texture Tile based render and a
smallest enclosing power 2 texture render.

The texture tile based render is defined in the header file

    #include<OpenTissue/visualization/direct_volume_rendering/tile_render.h>

and the smallest power 2 render is defined in

    #include<OpenTissue/visualization/direct_volume_rendering/render.h>

Both renders have the same interface and only differs in how they internally
manage allocation of texture memory. There are some inherent trade offs.

When using a non-power of 2 volume the tile based render is likely to minimize memory allocation compared to the
smallest power of 2 render. This is because the power of 2 render will allocate the
smallest power of 2 cubic texture enclosing the non-power of 2 volume. As an example
if you volume is of dimension 128x128x62 then the power of 2 render will allocate
one texture of dimension 128x128x128. The tile based render on the other hand
will cut the non-power of 2 volume into smaller fixed-size chunks.

The difference in memory allocation also means that the power of 2 render will
only need to render one huge texture tile, whereas the tile-based render may
need to render many smaller texture tiles. If your volumes shader (more about
this latter) have a large overhead when switching texture tile then the tile
based render is likely to have a performance degradation compared to the power
of 2 render.

To summarize, the general trends are
* Tile render: Poor performance but less allocated memory
* Power of 2 render: Better performance but more allocated memory

In order to use the two renders you need to define them, both takes
two template arguments the first is a compound types argument the
other is the volume shader that should be used. In code this would
look like

    struct types
    {
      typedef double                            real_type;
      typedef OpenTissue::vector3<real_type>    vector3_type;
    };

    typedef ..........  shader_type;
    typedef OpenTissue::TileRender<types,shader_type>   render1_type;
    typedef OpenTissue::Render<types,shader_type>       render2_type;

The shader type determines how a texture tile is going to be displayed.
OpenTissue currently provides two different types of volume shaders: A ray cast
shader and a view aligned slabbing shader. These are defined in the header files

    #include<OpenTissue/visualization/direct_volume_rendering/ray_cast_shader/ray_cast_shader.h>

and

    #include<OpenTissue/visualization/direct_volume_rendering/slabbing_shader/slabbing_shader.h>

In order to use the volume shaders you just simply pass them along as the second
template argument of your volume render, as follows


    struct types
    {
      typedef double                            real_type;
      typedef OpenTissue::vector3<real_type>    vector3_type;
    };

    typedef OpenTissue::RayCastShader   shader1_type;
    typedef OpenTissue::SlabbingShader  shader2_type;
    typedef OpenTissue::TileRender<types,shader1_type>   render1_type;
    typedef OpenTissue::Render<types,shader1_type>       render2_type;
    typedef OpenTissue::TileRender<types,shader2_type>   render3_type;
    typedef OpenTissue::Render<types,shader2_type>       render4_type;

The code above shows how to define all four possible volume render types
currently supported in OpenTissue. Their interfaces are all the same, thus for
simplicity we will let render_type denote any of the four above types in the
following discussion.

After having defined your volume render type you can add a render to your application

<pre>
class Application
{
protected:
  ...

  boost::shared_ptr< render_type > m_render;
  ...
};
</pre>

You need a pointer to the render, because Cg programs are used. This
requires that you must setup a Cg context before instantiating any Cg
programs (as explained in [The Shader
Programming Guide](using_shaders.md)). Thus during initialization you will have to
write

<pre>
void Application::init()
{
  ...
  Cg::startup()
  m_render.reset( new render_type() );
  ...
  create color table image and texture
  create 3D image
  ...
  m_render->init( m_volume)
};
</pre>

The last line initializes the render, so it is ready for displaying the 3D
image. In the display handler you will then tell the render to actual show the
3D image by writing

<pre>
void Application::display()
{
  ...
  m_render->display(m_color_table_texture);
  ....
};
</pre>

Notice here how easy it is to exchange the color table used without paying any
penalty in the visualization. You can even pre-compute and setup color tables
on priory and dynamically swap these at run time. Observe that there is no need
to deallocate the render because we use a boost shared pointer. However you
should remember to clean up Cg when the application closes, by writing
something like

<pre>
Application::~Application()
{
  Cg::shutdown();
};
</pre>

In this example you should pay special attention to the facts that
* A render must be instantiated after Cg have been started up (use a boost shared pointer for this if you can)
* A render must be initialized with the volume it is supposed to visualize
* A render shows the volume by invoking a display method. The display method takes a pointer to a color table texture.

That is it, now you can visualize arbitrary sized 3D images.

## Writing your own volume shader
It may be that you need a different volume shader than the ones provided by
OpenTissue. To make it easy for users to extend with their own shaders we have
implemented a volume shader concept using CRTP. First you need to include the
header file

    #include<OpenTissue/visualization/direct_volume_rendering/volume_shader.h>

Then you can create your own volume shader class, for instance by writing

      template<typename types>

      class MyShader : public VolumeShader< MyShader<types> >
      {
        public:

        typedef typename types::real_type      real_type;
        typedef typename types::vector3_type   vector3_type;

        ....
      };


Your shader must have a types template argument as shown in the example. You can
use this for passing application specific types to your shader. In the example
the shader extracts a real- and vector3 type from the types argument. By using
CRTP to implement the volume shader the compiler will make sure that your shader
conforms with the expected interface used by the tile based render and the
smallest power of 2 render (discussed earlier). The interface consist of five
methods.
* init
* front2back
* pre_render
* render
* post_render

These will be explained in the remainder.
The initialization method
<pre>
void init()
</pre>
This method should be used to initialize internal data. For
instance setting up textures (for RTT) or frame buffer objects. It
could also be used for compiling and loading GPU programs.
Front to back rendering method
<pre>
bool front2back() const
</pre>
If the return value is true then tiles should be rendered
in front to back order otherwise they should be rendered in
back to front order. The pre-render method

    template<typename texture2d_pointer>
    void pre_render ( texture2d_pointer & color_table )

This method prepares the rendering. This is a good place to perform
common computations that need to be done for all tiles.
For instance retrieving the model-view transformation, clearing
textures (for RTT) or setting GL state.
Rendering a single texture tile

    template<typename tile_type>
    void render(tile_type const & tile)

Note that your shader is responsible for handling any Ping-Pong schemes
in between tiles! A texture tile type is defined in the header file

    #include<OpenTissue/visualization/direct_volume_rendering/texture3d_tile.h>

You should look in this header file to see what kind of data members you can
access and use when implementing your rendering method. Post Rendering method

<pre>
void post_render()
</pre>

This method is invoked after having rendered all tiles. It is useful for
cleaning up GL state or transferring rendering result to frame-buffer (for RTT).
