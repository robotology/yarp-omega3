#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(SDK_VERSION 3.10.1)
set(SDK_BUILD 3605)
set(SDK_URL https://www.forcedimension.com/downloads/sdk/sdk-${SDK_VERSION}.${SDK_BUILD}-linux-x86_64-gcc.tar.gz)
set(SDK_LOCAL ${CMAKE_CURRENT_SOURCE_DIR}/sdk.tar.gz)
set(SDK_HASH 8cbf0ba9f953dc9d0e26116cc4fb002fac223a7d0e7dee2aeea5b1d728c3ebcb36edc91469068cc6ff23e8d33e313d79e20c7d1b6577993c00e28b625b2ca33c)

function(download_file url filename)
  message("Downloading Force Dimension SDK...")
  file(DOWNLOAD ${url} ${filename} SHOW_PROGRESS STATUS DOWNLOAD_STATUS)

  list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
  list(GET DOWNLOAD_STATUS 1 ERROR_MESSAGE)

  if(${STATUS_CODE} EQUAL 0)
    message(STATUS "Download completed successfully!")
  else()
    message(FATAL_ERROR "Error occurred during download: ${ERROR_MESSAGE}")
  endif()
endfunction(download_file)

# Download the SDK if not available
if(NOT EXISTS ${SDK_LOCAL})
   download_file(${SDK_URL} ${SDK_LOCAL})
endif()

# Download the SDK if the file is available but wrong
file(SHA512 ${SDK_LOCAL} SDK_LOCAL_HASH)
if(NOT "${SDK_HASH}" STREQUAL "${SDK_LOCAL_HASH}")
   download_file(${SDK_URL} ${SDK_LOCAL})
endif()

# Extract the SDK
file(ARCHIVE_EXTRACT INPUT ${CMAKE_CURRENT_SOURCE_DIR}/sdk.tar.gz DESTINATION ${CMAKE_CURRENT_SOURCE_DIR})

# Configure the actual executable
set(EXE_TARGET_NAME yarp-omega3-server)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find required dependencies
find_package(Libusb1)
find_package(Threads REQUIRED)
find_package(YARP REQUIRED COMPONENTS idl_tools os sig)

# Header files
set(${EXE_TARGET_NAME}_HDR
    sdk-${SDK_VERSION}/include/dhdc.h
    sdk-${SDK_VERSION}/include/drdc.h
    include/Server.h
)

# Source files
set(${EXE_TARGET_NAME}_HDR
    src/Server.cpp
    src/main.cpp
)

# Generate thrift sources
set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/server_idl.thrift)
yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Add the executable
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${EXE_TARGET_NAME}
                           PRIVATE
                           ${Libusb1_INCLUDE_DIRS}
                           ${CMAKE_CURRENT_SOURCE_DIR}/sdk-${SDK_VERSION}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      Threads::Threads
                      YARP::YARP_init
                      YARP::YARP_os
                      YARP::YARP_sig
                      ${Libusb1_LIBRARIES}
                      ${CMAKE_CURRENT_SOURCE_DIR}/sdk-${SDK_VERSION}/lib/release/lin-x86_64-gcc/libdrd.a
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)
