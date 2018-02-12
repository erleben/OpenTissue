=FAQ=

* Question: Argh why do I need to be bothered with having to use this glut framework, why can't I just use my own way to do glut-stuff
* Answer: If you want your code to be part of OpenTissue demos then your code must fit into this framework. There are mainly two reasons for this. It is easier for OpenTissue developers to maintain the code. If we find a bug in how glut is used then we only need to make the bug-fix in one place in order to make all the demo application run smoothly. Second, it is most nicer for new users to quickly get an overview of how a demo application work if all the demo applications have similar functionality.

=Tutorial=
An example Glut GUI application is located in the folder

<pre>
${OpenTissue}/demos/glut/gui_template
</pre>

One can use this as a basis for creating ones own new demo applications. Simply copy this folder, rename the copied folder to whatever name that is wanted. Then fix the CMakelists.txt files.

The glut framework header code is located in the sub-library folder

<pre>
${OpenTissue}/OpenTissue/utility/glut/
</pre>

When one uses the library one basically needs to do two steps

* Create an application class
* Instanciate an instance of ones application class and return a pointer to the new instance  to the glut framework

Below we will go through the details of these two steps. Afterwards we will explain how to customize the framework even further.

== Creating an Applicaition Class ==


One creates an application class by inherting from a base-class in the glut framework. As of this writting (May 2007) there are two base-class available

* OpenTissue::glut::Application
* OpenTissue::glut::PerspectiveViewApplication

The first base-class is pretty raw, it basically does nothing. It is suitable if one wants to create a fundamental different interaction, like a 2D or orthograpic visualization. The second base-class have been extended with a lot of convience functionality. It sets up a perspective projection, it handles trackball interaction, camera, frustum, mouse interaction etc.. Also it provides frame-grapping and movie recording support. One gets all this for free just be creating an inherited class.


Now create a cpp-file for your demo application, for instance my_app.cpp, then the first thing you need to do is to setup the include headers. This might look something like

<pre>
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/utility/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

... add all other kind of headers you need here...

</pre>

Notice the pre-directive DEFINE_GLUT_MAIN this is needed in order to tell the framework that it should create an application main entry point (ie. a int main(int argc, char **argv) function). Next you are ready to create the inherited applciation class. It should at very least look something like

<pre>
class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:

Application()
{
...
}

public:

char const * do_get_title() const { return "My Application"; }

void do_display()
{
...
}

void do_action(unsigned char choice)
{
...
}

void do_init_right_click_menu(int main_menu, void menu(int entry))
{
...
}

void do_init()
{
...
}

void do_run()
{
...
}

void do_shutdown()
{
...
}

};

</pre>

The base class is a pure abstract class so the compile will tell you if you forget to implement any of the needed interface method. The purpose of the member methods should be clear from their naming.

== Creating an Instance ==

The next step is to pass an instance of your new application to the glut framework. This is done by implementing the init_glut_application-function. It is very important that you get the name and signature of this function correct, otherwise you get compiler errors. For our example class we should implement the function as follows:

<pre>
OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
OpenTissue::glut::instance_pointer instance;
instance.reset( new Application() );
return instance;
}
</pre>


= Customization =

In most cases the default behavior of the OpenTissue::glut::PerspectiveViewApplication base class will fulfill most peoples need for creating simple demo applications. However, we have encountered a few special needs that required some extra tweaking. Below we will walk through some of these extra tweaks.

== Adding Runtime Arguments ==

If you want to pass run-time arguments to your application then this can be accomplised by creating a specialized constructor in your application class. For instance like this:

<pre>
...
Application(int argc, char **argv)
{
....
}
...
</pre>

Then when one implements the init_glut_application-function, one simple passes along argument to the specialized constructor like this:

<pre>
OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
OpenTissue::glut::instance_pointer instance;
instance.reset( new Application(argc,argv) );
return instance;
}
</pre>


== Mouse Control ==

In some cases one may wish to use the mouse for picking or moving objects interactively on the screen. In order to do this one would beed to override the default mouse event handlers in the base-class and extend with ones own functionality.

This could be done by adding the following member methods to your application class:


<pre>
void mouse_down(double cur_x,double cur_y,bool shift,bool ctrl,bool left,bool middle,bool right)
{
OpenTissue::glut::PerspectiveViewApplication::mouse_down(cur_x,cur_y,shift, ctrl, left, middle, right);

if( middle && ctrl)
{
... do my thingy ...
m_doing_it = true;
}
}

void mouse_move(double cur_x,double cur_y)
{
OpenTissue::glut::PerspectiveViewApplication::mouse_move(cur_x,cur_y);
if( m_doing_it)
{
....
}
}

void mouse_up(double cur_x,double cur_y,bool shift,bool ctrl,bool left,bool middle,bool right)
{
OpenTissue::glut::PerspectiveViewApplication::mouse_up(cur_x,cur_y,shift, ctrl, left, middle, right);

if( m_doing_it)
{
m_doing_it = false;
}
}
</pre>

Notice that we invoke the mouse event handlers of the base-class. If this is not done then one will not ``inherit'' the base-class mouse functionality.

== Change the Initial Camera Settings ==

Every application will start of with the default camera settings that is used in the OpenTissue::glut::PerspectiveViewApplication base-class. This is not always desireable. If one wants to change the default window-size, clippling planes, or fovy then this can be done in the constructor of the ones Application. For instance like this

<pre>
Application()
{
this->z_far() = 5000;
this->z_near() = 0.1;
...
this->width() = 128;
this->height() = 128;
}
</pre>

The glut framework will query these settings once it receives the instance-pointer and will therefore automatically adjust itself according to your settings. In order to alter the camera settings we recommend doing this in the do_init() method. This could look like this:

<pre>
...
void do_init()
{
...
vector3_type position = ...;
vector3_type target = ...;
vector3_type up = ...;
this->camera().init( position, target, up );
}
...
</pre>

The camera has a lot of convience functions for manipulating it. One can even get down and work on the bare matrix-representations if one desires to add some non-existing transformations.

== Creating Menus ==

The do_init_right_click_menu-method might be unfamiliar for those that do not know about Glut. This method is intended to extend the right-click menu with ones own menu items. This can for example be done by writting

<pre>
...
void do_init_right_click_menu(int main_menu, void menu(int entry))
{
int controls = glutCreateMenu( menu );
glutAddMenuEntry("do something             [1]", '1');
....
glutAddMenuEntry("reset                    [r]", 'r');
glutSetMenu( main_menu );
glutAddSubMenu( "my menu", controls );

}
...
</pre>
