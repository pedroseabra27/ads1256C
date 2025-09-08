cmake_minimum_required(VERSION 3.16)
project(ads1256_c_sampler C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(ENABLE_DEBUG "Enable debug flags" ON)

if(ENABLE_DEBUG)
  add_compile_definitions(DEBUG)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -Wall -Wextra -Wno-unused-parameter")
else()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -Wall -Wextra -Wno-unused-parameter")
endif()

file(GLOB SRC_FILES src/*.c)
include_directories(include)

add_executable(ads1256_sampler ${SRC_FILES})

find_package(Threads REQUIRED)

# Optional: libgpiod via pkg-config
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(GPIOD QUIET libgpiod)
  if(GPIOD_FOUND)
    target_include_directories(ads1256_sampler PRIVATE ${GPIOD_INCLUDE_DIRS})
    target_link_directories(ads1256_sampler PRIVATE ${GPIOD_LIBRARY_DIRS})
    target_link_libraries(ads1256_sampler PRIVATE ${GPIOD_LIBRARIES})
    target_compile_definitions(ads1256_sampler PRIVATE USE_GPIOD=1)
    message(STATUS "libgpiod found: ${GPIOD_VERSION}")
  else()
    message(STATUS "libgpiod not found; falling back to timed DRDY waits")
  endif()
else()
  message(STATUS "pkg-config not found; skipping libgpiod detection")
endif()

target_link_libraries(ads1256_sampler PRIVATE Threads::Threads m)

install(TARGETS ads1256_sampler RUNTIME DESTINATION bin)
