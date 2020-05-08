macro(quad_cmake_setup)

    message(STATUS "[${PROJECT_NAME}] cmake settings")

    if (NOT CMAKE_BUILD_TYPE)
      message(STATUS "CMAKE_BUILD_TYPE not set. Default to 'Release'")
      set(CMAKE_BUILD_TYPE Release)
    endif()

   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

   if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      message(STATUS "set compile flags for Linux system")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      message(STATUS "set compile flags for Darwin system")
      set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++")
   elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      message(STATUS "set compile flags for Windows system")
      message(AUTHOR_WARNING "TODO ... compile flags for Windows")
   else()
      message(SEND_ERROR "Unknown/unsupported(?) system")
   endif()

   if (CMAKE_BUILD_TYPE STREQUAL "Debug")
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
   endif()
endmacro()
