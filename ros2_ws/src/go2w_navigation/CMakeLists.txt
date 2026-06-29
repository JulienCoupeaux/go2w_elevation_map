cmake_minimum_required(VERSION 3.8)
project(go2w_navigation)

find_package(ament_cmake REQUIRED)

# Package de bringup/config pour la navigation MPPI 2.5D du go2w.
# (Le critic MPPI 3D custom — src/mppi_3d_critic.cpp + plugins.xml — n'est pas
#  encore implemente ; la cible de build a ete retiree pour que le package
#  compile. La reintroduire ici une fois le .cpp ecrit.)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
