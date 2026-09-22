cmake_minimum_required(VERSION 3.10)
project(racecar_test_assets)

find_package(ament_cmake REQUIRED)

# data resources ship with the package (share/racecar_test_assets/...)
install(DIRECTORY models worlds config launch test docs
        DESTINATION share/${PROJECT_NAME})

# tools are directly runnable (shebang + exec bit preserved on install)
install(PROGRAMS
  tools/asset_params.py
  tools/generate_cone_mesh.py
  tools/generate_world.py
  tools/generate_top_view.py
  tools/generate_sign_texture.py
  tools/check_models.py
  tools/check_world.py
  tools/cone_section.py
  tools/validate_assets.py
  tools/preview.sh
  tools/gazebo_load_check.sh
  tools/acceptance_install_check.sh
  DESTINATION share/${PROJECT_NAME}/tools)

# sourcing install/setup.bash prepends this package's models/ to
# GAZEBO_MODEL_PATH, so model://cone_red etc. resolve without manual copies
ament_environment_hooks("env-hooks/${PROJECT_NAME}.dsv")

ament_package()
