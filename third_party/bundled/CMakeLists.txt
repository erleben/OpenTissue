#---------------------------------------------
#
# Movie Maker is strictly windows thing
#
# QHull, is distributed with Gentoo-Portage on linux, so we only need to built
# this ourselves on windows.
#
#  TetGen, TinyXML, and Triangle are as this writting (May 2007) only
#  distributed as source code, so we need to built these on both
#  windows and linux.
IF(WIN32)
  SUBDIRS(qhull)
ENDIF(WIN32)
SUBDIRS(TetGen)
SUBDIRS(TinyXML)
SUBDIRS(Triangle)
