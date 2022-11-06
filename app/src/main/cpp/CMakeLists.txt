cmake_minimum_required(VERSION 3.18.1)
project("xrstreamer")
set(CMAKE_CXX_STANDARD 17)
get_filename_component(OVR_MOBILE_SDK_DIR "$ENV{OVR_MOBILE_SDK_DIR}" ABSOLUTE)
get_filename_component(OPENXR_SDK_DIR "$ENV{OPENXR_SDK_DIR}" ABSOLUTE)

#
# native_app_glue as a static lib
#
set(TARGET_NAME native_app_glue)
set(ANDROID_NATIVE_APP_GLUE
    ${CMAKE_ANDROID_NDK}/sources/android/native_app_glue)
add_library(${TARGET_NAME} STATIC
            ${ANDROID_NATIVE_APP_GLUE}/android_native_app_glue.c)
target_include_directories(${TARGET_NAME} PUBLIC ${ANDROID_NATIVE_APP_GLUE})
# Export ANativeActivity_onCreate(), Refer to:
# https://github.com/android-ndk/ndk/issues/381.
target_link_options(${TARGET_NAME} PUBLIC -u ANativeActivity_onCreate)

#
# prebuild libopenxr_loader.so
#
set(TARGET_NAME openxr_loader)
add_library(${TARGET_NAME} SHARED IMPORTED)
set(OPENXR_LOADER_INCLUDES ${OPENXR_SDK_DIR}/include
                           ${OVR_MOBILE_SDK_DIR}/OpenXR/Include)
set_target_properties(
  ${TARGET_NAME}
  PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${OPENXR_LOADER_INCLUDES}"
    IMPORTED_LOCATION
    ${OVR_MOBILE_SDK_DIR}/OpenXR/Libs/Android/${ANDROID_ABI}/Release/libopenxr_loader.so
)

subdirs(SampleCommon)

#
# xrstreamer
#
set(TARGET_NAME xrstreamer)
add_library(
  ${TARGET_NAME} SHARED
  main.cpp
  XrApp.cpp
  HandTracking.cpp
  HandRenderer.cpp
  Hand.cpp
  Input/HandMaskRenderer.cpp
  Input/HandRenderer.cpp
  Render/Framebuffer.cpp)
target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_compile_options(${TARGET_NAME} PRIVATE -Wall -Werror)
target_link_libraries(
  ${TARGET_NAME}
  PRIVATE SampleCommon
          native_app_glue
          openxr_loader
          android
          log
          GLESv3
          EGL)
