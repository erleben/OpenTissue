==The Image and Texture Programming Guide==
===Working with Images===
The image data structure is a template class located in the header
file
<pre>
#include<OpenTissue/image/image.h>
</pre>

To work with images you need to specify what kind of data type that is
stored in the channels of the image. For instance to get an image that
supports 8bit channels you should write something like this
<pre>
typedef Image<unsigned char> image_type;
</pre>

A channel is like a color component, the image data structure thus allows you to create all kind of image types. Usually the most common is either 1 channel (intensity/grey scale images), 3 channels (red, green, and blue colors) or 4 channels (red, green, blue, and transparency)

In some cases it can be beneficial to use boost smart pointers, thus you may want to define the type
<pre>
typedef boost::smart_ptr<image_type> image_pointer;
</pre>
Now you can create a new image like this
<pre>
image_pointer img;
img.reset( new image_type( 256, 128 ,4) );
</pre>
This will create an image with 128 rows each with 256 pixels, and 4
components in each and every pixel. You can also create the image by
writing
<pre>
img.reset( new image_type( ) );
img->create(256, 128 ,4);
</pre>

The create() method is convenient for re-creating images on the fly or
if you do not know the dimensions of the image until later on. You can
access data in a image by using the () operator, for instance to set
all pixels to semi-transparent green color, you would write
<pre>
for(unsigned int j=0;j<128;++j)
for(unsigned int i=0;i<256;++i)
{
(*img)(i,j,0) = 0;
(*img)(i,j,1) = 255;
(*img)(i,j,2) = 0;
(*img)(i,j,3) = 155;
}
</pre>
Or a little more elegantly by querying the image of its dimensions
<pre>
for(unsigned int j=0;j<img->height();++j)
for(unsigned int i=0;i<img->width();++i)
{
(*img)(i,j,0) = 0;
(*img)(i,j,1) = 255;
(*img)(i,j,2) = 0;
(*img)(i,j,3) = 155;
}
</pre>

Actually you can also query the number of channels, such a hand-written clearing of the image could be written as
<pre>
for(unsigned int j=0;j<img->height();++j)
for(unsigned int i=0;i<img->width();++i)
for(unsigned int c=0;c<img->channels();++c)
{
(*img)(i,j,c) = 0;
}
</pre>

In some cases you may want to access the image data as a linear contiguous array, stored in row-format, for this you would write
<pre>
void * address = img->get_data();
</pre>
Notice that this returns the address of the starting position of the data as a void pointer. If you want a pointer to the image data type, you'll need to cast it
<pre>
typedef typename image_type::value_type value_type;
value_type * data = static_cast<value_type>(address);
</pre>

An easy to use interface have been implemented allowing end-users to read and write images from and to the hard drive. The functions implementing this is defined in the header files
<pre>
#include<OpenTissue/image/io/image_read.h>
#include<OpenTissue/image/io/image_write.h>
</pre>
Depending on the extension of the supplied filename various image file-formats are supported. For instance you can write
<pre>
string filename = "input.png";
image_read(filename,*img);
... do something with img ...
filename = "output.bmp";
image_write(filename,*img);
</pre>
Here in this example we choose to read in a png-file and write a bmp-file. Usually you would want to both read and write png-files. A number of different image utilities have also been implemented. A convenient utility is the screen_capture() utility. This is defined in the header file
<pre>
#include<OpenTissue/image/util/screen_capture.h>
</pre>

An returns a boost shared pointer to a 8bit four channel image containing whatever is currently in the frame-buffer. You can use it like this:
<pre>
image_pointer  image = screen_capture();
image_write("myfile.png",*image);
</pre>

There are many other utilities and we encourage you to explore the folder util sub folder in the image folder of OpenTissue.



===Working with 2D Textures===

The image data structure have been implemented to make it easy to create 2D textures. The 2D texture object data structure is defined in the header file
<pre>
#include<OpenTissue/texture/texture2D.h>
</pre>
Herein is a texture 2D pointer data type, usually this is used as follows
<pre>
texture2d_pointer tex = img->create_texture( GL_RGBA );
</pre>

This will create a new 2D texture object with the same width and height as the image, the argument specifies the internal format of the texture object. That is how many components that is created in the texture. In our example we have chosen a red, green, blue and alpha format. Once the texture pointer have been obtained it can be used by binding it before drawing, i.e.
<pre>
tex->bind();
</pre>

One can also re-load data into the texture object, for instance by writing
<pre>
tex->load( img->get_data() );
</pre>

The texture 2D object integrates nicely with the Cg program class, thus if one wants to setup a 2D texture for a Cg program, once simply writes
<pre>
CgProgram gpu_fun;
...
gpu_fun.set_input_texture("texture",  tex);
</pre>

There exist more ways of creating textures, which do not necessitate the use of an image, for examples see for instance in the header file
<pre>
#include<OpenTissue/texture/texture2D_util.h>
</pre>

===Working with 3D Textures===

The natural extension for 2D textures is of course 3D textures. The 3D texture object is defined in the header file
<pre>
#include<OpenTissue/texture/texture3D.h>
</pre>
An there exist an utility for creating such textures from 3D images, which is located in the header file
<pre>
#include<OpenTissue/texture/texture3D_util.h>
</pre>
See the volume visualization programming guide for an example of how to use this utility. Here we will discuss a few other details of the texture3D object.

It is possible to create an initial empty texture object by writing
<pre>
ext_format = external_format<unsigned short>(1);
ext_type = external_type<unsigned short>();
tex.reset( new Texture3D( GL_RGBA, 128, 128 ,128, external_format, external_type)
</pre>
This creates a cubic texture object of dimension 128x128x128. Internally voxels are stored as GL_RGBA, ie. a red, green, blue and alpha component. Externally, that is on the CPU side, data is given as 12bit 1 channel data. Typical the case of medical images. After having created an empty texture one can load data into it by invoking the methods
<pre>
ext_format = external_format<unsigned short>(1);
ext_type = external_type<unsigned short>();
tex->load_sub_image_into_texture( skip_i, skip_j,skip_k, size_i,size_j,size_k, offset_i, offset_j, offset_k, fill_i,fill_j,fill_k, data);
</pre>

The parameters set up an correspondence between a sub region of the CPU side data with the GPU side texture region, where data is loaded into.
