# - Find WiringPi library
# Find the native WiringPi includes and library
#
#  WIRINGPI_INCLUDE_DIRS - where to find wiringPi.h, etc.
#  WIRINGPI_LIBRARIES    - List of libraries when using WiringPi.
#  WIRINGPI_FOUND        - True if WiringPi found.

# 查找头文件（wiringPi.h）
find_path(WIRINGPI_INCLUDE_DIR
  NAMES wiringPi.h
  HINTS /usr/include /usr/local/include
  PATH_SUFFIXES wiringpi
)

# 查找链接库（libwiringPi.so 或 libwiringPi.a）
find_library(WIRINGPI_LIBRARY
  NAMES wiringPi wiringPiDev
  HINTS /usr/lib /usr/local/lib
)

# 检查是否找到头文件和库
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(WiringPi
  DEFAULT_MSG
  WIRINGPI_LIBRARY WIRINGPI_INCLUDE_DIR
)

# 设置输出变量（兼容传统 CMake 用法）
if(WIRINGPI_FOUND)
  set(WIRINGPI_INCLUDE_DIRS ${WIRINGPI_INCLUDE_DIR})
  set(WIRINGPI_LIBRARIES ${WIRINGPI_LIBRARY})
  
  # 创建导入目标（现代 CMake 推荐用法）
  if(NOT TARGET WiringPi::WiringPi)
    add_library(WiringPi::WiringPi UNKNOWN IMPORTED)
    set_target_properties(WiringPi::WiringPi PROPERTIES
      IMPORTED_LOCATION "${WIRINGPI_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${WIRINGPI_INCLUDE_DIR}"
    )
  endif()
endif()

# 标记变量为高级（在 cmake-gui 中隐藏）
mark_as_advanced(WIRINGPI_INCLUDE_DIR WIRINGPI_LIBRARY)