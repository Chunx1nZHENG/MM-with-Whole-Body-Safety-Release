cmake_minimum_required(VERSION 2.8)
project(cmake_clang_tools)

# Installation paths
set(PYTHON_SCRIPTS_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
set(CONFIG_FILES_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/cmake_clang_tools)
set(CMAKE_SCRIPTS_INSTALL_PATH ${CONFIG_FILES_INSTALL_PATH})

# Register the local build in case one doesn't use "make install"
export(PACKAGE cmake_clang_tools)

# ... for the build tree
set(PYTHON_SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CONFIG_FILE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(cmake_clang_tools-macros.cmake.in
  "${PROJECT_BINARY_DIR}/cmake_clang_tools-macros.cmake" @ONLY)

set(CMAKE_CLANG_TOOLS_MACROS_PATH ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY})
configure_file(cmake_clang_toolsConfig.cmake.in
  "${PROJECT_BINARY_DIR}/cmake_clang_toolsConfig.cmake" @ONLY)

# ... for the install tree
set(PYTHON_SCRIPTS_DIR "${PYTHON_SCRIPTS_INSTALL_PATH}")
set(CONFIG_FILE_DIR "${CONFIG_FILES_INSTALL_PATH}")
configure_file(cmake_clang_tools-macros.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_clang_tools-macros.cmake" @ONLY)

set(CMAKE_CLANG_TOOLS_MACROS_PATH ${CMAKE_SCRIPTS_INSTALL_PATH})
configure_file(cmake_clang_toolsConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_clang_toolsConfig.cmake" @ONLY)

# Install python scripts
install(
  FILES clang_format_check.py clang_tidy_check.py check_settings_change.py run_cmake_clang_tools_trigger.py
  DESTINATION ${PYTHON_SCRIPTS_INSTALL_PATH}
  PERMISSIONS WORLD_EXECUTE WORLD_READ GROUP_EXECUTE GROUP_READ OWNER_EXECUTE OWNER_WRITE OWNER_READ
)

# Install config files
install(
  FILES .clang-format .clang-tidy config.yaml
  DESTINATION ${CONFIG_FILES_INSTALL_PATH}
  PERMISSIONS WORLD_READ GROUP_READ OWNER_WRITE OWNER_READ
)

# Install cmake configurations
install(
  FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_clang_toolsConfig.cmake"
        "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake_clang_tools-macros.cmake"
  DESTINATION ${CMAKE_SCRIPTS_INSTALL_PATH}
)

install(
  FILES package.xml
  DESTINATION ${CONFIG_FILES_INSTALL_PATH}
)

#=============================================
# Add uninstall target
#=============================================
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
