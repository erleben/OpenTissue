//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#pragma once

int main(int argc, char** argv)
{
  auto app = ::createApplication(argc, argv);
  app->init();
  app->run();
}
