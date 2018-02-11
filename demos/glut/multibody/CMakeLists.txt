ADD_EXECUTABLE(multibody 
                src/scenes/setup_box.cpp 
                src/scenes/setup_cow.cpp 
                src/scenes/setup_cow_stack.cpp 
                src/scenes/setup_falling_cows.cpp 
                src/scenes/setup_silo.cpp 
                src/scenes/setup_diku.cpp 
                src/scenes/setup_silo_tower.cpp 
                src/scenes/setup_roof_tower.cpp 
                src/scenes/setup_domino_spiral.cpp 
                src/scenes/setup_domino_circle.cpp 
                src/scenes/setup_pyramid.cpp 
                src/scenes/setup_large_mass.cpp 
                src/scenes/setup_house.cpp 
                src/scenes/setup_card.cpp
                src/scenes/setup_revolute_joint.cpp
                src/scenes/setup_slider_joint.cpp
                src/scenes/setup_ball_joint.cpp
                src/scenes/setup_universal_joint.cpp
                src/scenes/setup_wheel_joint.cpp
                src/scenes/setup_box_stack.cpp 
                src/scenes/setup_jamm1.cpp 
                src/scenes/setup_jamm2.cpp 
                src/application.cpp
)

TARGET_LINK_LIBRARIES(multibody ${OPENTISSUE_LIBS})

INSTALL(
  TARGETS multibody
  RUNTIME DESTINATION  bin/demos/glut/
  )
