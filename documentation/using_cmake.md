= Install CMake =

[http://www.cmake.org/HTML/Download.html link Download] CMake and install it onto your system. The installation is pretty straightforward, if you get into trouble have a look at the [http://www.cmake.org/HTML/Install.html link guide]. Note that if you are running Gentoo it is probably easiest to use Gentoo-Portage to install CMake onto your system.


= Running CMake on Windows =

Here follows a quick step-by-step specific for OpenTissue. In the example we are running windows and want Visual Studio 2005 project files.

== Step 1 ==

* First you open the CMake program (you probably got an icon on your desktop, so just double click on this icon)
* Specify the path for the OpenTissue source code. This is the folder location on your hard drive where you choice to check-out OpenTissue from the subversion repository.
* Specify a path where you want the ''binaries'' to be. This folder will contain the generated Visual Studio project files. I (kenny) prefer out-of-source tree build and like to put everything into a build sub folder.

== Step 2 ==


* Next click on the configure button. CMake now ask you want type of generator you want. This is where you pick what kind of ''makefiles'' you want in the build-folder you specified in the previous step. I simply selected the Visual Studio 8 2005 generator and click the OK-button.

== Step 3 ==

* Next CMake might ask you if it should create the build folder you specified. Just accept this
* Now you have to wait a while. While you wait CMake will examine all the CMakeList.txt files in your source-tree. In the end you will presented with the results of CMakes efforts.

Now you should see a lot of Cache values marked with a red-color.

== Step 4 ==

The red color simply mean that CMake asks you to decide whether you want to override/set any of the shown values. A quick look through the cache values reveal that the Boost_INCLUDE_DIR cache value is completely wrong but all other values looks good. For convenience I have made a quick explanation of some of the most magical cache values in the table below. If you do not care about what they are used for simply skip the table.

<center>
<table border = "1">
<tr><td>
CMAKE_BACKWARDS_COMPATIBILITY</td><td>simply says what version of CMake that is compatible with the CMakeList.txt files located in your source-tree.</td></tr>
<tr><td>CMAKE_INSTALL_PREFIX</td><td> this is the location on your hard drive where OpenTissue will be installed if you run the install target, (ie. build the install project in Visual Studio 8 2005).</td></tr>
<tr><td>ENABLE_CMAKE_TESTING</td><td> This is off by default if you turn it on, then CMake will add all the test-cases if finds in the CMakeList.txt files to a RUNTESTS target in the generated makefiles (ie. a RUNTESTS project in your visual studio solution).</td></tr>
<tr><td>EXECUTABLE_OUTPUT_PATH and LIBRARY_OUTPUT_PATH</td><td> these can be used to override the default location where your compiler puts its libs and exes.</td></tr>
</table>
</center>
* Specify where your boost installment is located on your system. NOTE use forward slashes when you write the path, although not important this will save you from other problems in the future.


== Step 5 ==

* Press the configure button once more, and wait while CMake process everything a second time around.

Now all the cache values should be colored in gray, this implies that everything is good to go and we can start creating the content of the build folder.

* Finally press the OK-button. Now CMake will create the content of the build folder and automatically close the CMake application.

== After CMake ==

After having run CMake you simple go to the build folder you specified and look for the file ''OpenTissue.sln'', double click on this file and you are up and running, ready to code:-)


== Behind the Scenes ==

Currently there are some outstanding CMake quirks, in particular when it comes to using Visual Studio. Below is a brief description of some of the issues. Hopefully CMake will solve some of these issues in the future or we will find better workarounds. As of writing (May 2007) these quirks are the price we pay for having a cross-platform make tool.


<center>
<table border = "1">

<tr>
<td>OUTPUT PATHs</td>
<td>
With CMake one can only change the output path of all targets to be the same. One can not set the output of individual targets to some specified path. This is cumbersome because OpenTissue have some third-party dependencies bundled as source code. These third party dependencies are supposed to be build and installed into the externals-folder structure. We used to handle this easily by changing the output path of the targets of our third-party dependencies.

Our work-around for this is to use custom build events for copying files from the build tree into the externals-folder located in the source-tree (yes it is a bit messy).

</td>
</tr>


<tr>
<td>Post Build Events</td>
<td>Fix custom post-build events on dependencies such that different configuration types (ie. release and debug) copies different files. On windows output libs are often named differently depending on whether they are debug or release versions. We use custom post-build events to copy our libs around. This means that as of this writing, both the release and debug configurations in OpenTissue tries to copy both the release and debug libs.</td>
</tr>

<tr>
<td>Bundled Third-party software</td>
<td>We would like to make CMake ignore dependencies to targets of bundled third-party software. Bundled third-party software is a build-once type of target, then the static libs and headers are installed at respective places in our externals folder. Demos and units should see these as third-party software that were bundled with OpenTissue as binaries.</td>
</tr>

<tr>
<td>PROJECT GROUPS</td>
<td>Add Project groups when available in CMake (as of writing ver. 2.4.6 does not support such a feature)</td>
</tr>

<tr>
<td>Header only targets</td>
<td>These are impossible with CMake, one needs at least one source file to create a target with CMake. We have adopted the work-around to let CMake create a fake cpp-file in the build tree, which is included in the header-only target.
</td>
</tr>

<tr>
<td>Loss of directory structure</td>
<td>
Visual Studio is nicely integrated with the file system of the operating system. Thus the folder structure is reflected in the projects and greatly eases the navigation. Using CMake to generate out-of-source tree targets results in complete loss of this information.

It inhibits the creative power of an environment such as Visual Studio, one looses oversight, one looses the means to interact directly with the source-code (moving files around by drag and drop) etc..

The INCLUDE_EXTERNAL_MSPROJECT CMake command seems to provide a way out of this, but it is a visual studio only solution (not very cross-platform) and it results in double-maintenance of the makefiles, something we are trying to avoid by shifting to CMake.

Another possible solution is to let the binary tree be the same as the source-tree. This way CMake will generate the makefiles directly into your source tree. However, this comes with a raft of other problems.
</td>
</tr>


<tr>
<td>CMakeList.txt pollution</td>
<td>
CMake will pollute the generated visual studio projects by including the CMakeList.txt files, but some of the auto-generated targets are not very meaningful in a visual studio environment anyway. For instance the build all target. The user interface of visual studio already provides such functionality. This pollution is a bit awkward for pure visual studio developers, but we can not completely hide CMake from them. The developers will need to interact with CMake in order to create new demos or new unit tests etc. In conclusion we can not completely hide CMake details from OpenTissue developers.

When developers need to create new stuff, it will basically consist of extending/adding SUBDIRS commands in existing CMakeList.txt files, and then copy and paste a CMakeList.txt file from an existing target similar to the new one the developer is trying to create. In the copied CMakeList.txt file the developer will have to change targets names and possible add extra compile/linker flags if needed. Once this is done the developer need to run CMake again to update his visual studio solution/project files with the newly created things.
</td>
</tr>


</table>
</center>



= Running CMake on Linux =

The first thing you should do is to go to the folder containing the source tree. That is the folder location where you placed the OpenTissue source code. Next you have to decide where the build-tree should be placed. The build-tree is the folder where CMake will generate the resulting makefiles for you.

On my system I (kenny) like to have OpenTissue check-out in a working folder in my home folder, so the first step is pretty easy

<pre>
cd
cd /work/OpenTissue
</pre>

I also like to place my build-tree inside the OpenTissue folder, so I create a folder for it like this:

<pre>
mkdir build
cd build
</pre>

Now I am ready to run CMake,

<pre>
ccmake ../
</pre>

Notice that the command is named ccmake and not cmake. The former will allow me to edit values that CMake do not know how to find. For instance it could be paths for third party software that is specific for your system in which case CMake would have little to no chance of finding the software itself. If the later command is run then cmake will do everything for you in one go without any user interaction.

The first argument to the CMake command is a relative path to the folder containing the topmost CMakeLists.txt file in the source tree. Both CMake command versions take a second argument. This argument can be used to tell CMake what kind of Makefiles it should generate for you. By default CMake generates unix makefiles, but if you for instance like KDevelop you can simply write

<pre>
ccmake ../ -G KDevelop3
</pre>

Now an ASCII user interface should pop up on your screen.

At the bottom of the screen several options is listed. First you need to run the configure option by pressing the c-key. This will make CMake scan through all the CMakeLists.txt file in the source tree and create a so called cache value table. The resulting cache values are displayed in the ASCII screen once CMake is done.

* Press c-key

At this point you should look through all the cache values that are displayed to see if they make sense. CMake make hints to you by displaying star ('*') in front of cache values that CMake things you should have a closer look at. Below of here there is a table explaining the most common cache values that you will see on your screen. All of these can be ignored.

* Edit <VAR>-NOTFOUND cache values

<center>
<table border = "1">
<tr><td>
CMAKE_BACKWARDS_COMPATIBILITY</td><td>simply says what version of CMake that is compatible with the CMakeList.txt files located in your source-tree.</td></tr>
<tr><td>CMAKE_INSTALL_PREFIX</td><td> this is the location on your hard drive where OpenTissue will be installed if you run the install target, (ie. build the install target).</td></tr>
<tr><td>ENABLE_CMAKE_TESTING</td><td> This is off by default if you turn it on, then CMake will add all the test-cases if finds in the CMakeList.txt files to a RUNTESTS target in the generated makefiles (ie. a RUN_TESTS target).</td></tr>
<tr><td>EXECUTABLE_OUTPUT_PATH and LIBRARY_OUTPUT_PATH</td><td> these can be used to override the default location where your compiler puts its libs and exes.</td></tr>
</table>
</center>

You should pay particular attention to any cache values containing a string value like <VAR>-NOTFOUND. You should edit these values such that they are correct. Note that you can toggle to advanced mode by pressing the t-key. In most cases this is not needed but if you would like to see all the things that CMake automatically found for you and if you might want to override any settings then this might be a useful thing to do.

Now CMake is ready to make a second pass through all the CMakeLists.txt files in the source-tree to make sure that whatever values you have enter do not conflict with any thing else. Thus you will have to press the c-key once more time.

* Press c-key

If CMake was successful then you will see a new option on the screen, the g-key. This option will tell CMake to generate your makefiles in the build-folder and then exit CMake. If CMake was unsuccessful then you need to keep on editing the cache-values and press the configure key until CMake is happy about your choices.


* Press g-key

Now you can build the source-tree simply by writing

<pre>
make
</pre>

= Create a New Demo Application with CMake =

The easiest way is as follows:

* Make a copy of the folder containing the gui template demo application.
* Rename your folder to whatever name you like.
* Open up the CMakeLists.txt file located inside the folder you just renamed in your favorite text editor.
* Alter the target names in all the CMake commands to the target name of your new demo application.
* Add the source files that you have to the add_executable command

Say your target name is XXX and that you got two source files named: src/application.cpp, and src/dodah.cpp then the CMakeLists.txt file should look like this

<pre>
ADD_EXECUTABLE(XXX src/application.cpp src/dodah.cpp)

TARGET_LINK_LIBRARIES(XXX ${OPENTISSUE_LIBS})

INSTALL(
TARGETS XXX
RUNTIME DESTINATION  bin/demos/glut/
)
</pre>

Now the new demo application is ready, but you need to tie it together with the rest of OpenTissue.

* Open up the CMakeLists.txt file located in the parent folder (typically this will be OpenTissue/demos/glut) in your favorite text editor.
* Add subdirs command with the folder name of your new demo application.

Say your new demo application is placed in the folder OpenTissue/demos/glut/XXX then you need to open the OpenTissue/demos/glut/CMakeLists.txt file and add the following line

<pre>
SUBDIRS(XXX)
</pre>

That is it. Now you just generate your makefiles with CMake as usual and the new demo will appear as a project/target.


= Create a New Unit-Test with CMake =

The steps are nearly identical to the way how you would add a new demo application.

* Create a folder in OpenTissue/units, remember to pre-fix the name with "unit_"
* Place your source code in src sub folder and create a CMakeLists.txt file.

Without loss of generality say your new unit-test is named unit_yy then the CMakeLists.txt file should look like this

<pre>
ADD_EXECUTABLE(unit_yyy src/unit_yyy.cpp)

TARGET_LINK_LIBRARIES(unit_yyy ${OPENTISSUE_LIBS} ${Boost_unit_test_framework_LIBRARY})

INSTALL(
TARGETS unit_yyy
RUNTIME DESTINATION  bin/units
)

ADD_TEST( unit_yyy unit_yyy )
</pre>

Now you are ready to add the new unit-test to OpenTissue

* Open the file OpenTissue/units/CMakeLists.txt
* Add a sudirs command

<pre>
SUBDIRS(unit_yyy)
</pre>

That is it. Now you just generate your makefiles with CMake as usual and the new unit test will appear as a project/target.
