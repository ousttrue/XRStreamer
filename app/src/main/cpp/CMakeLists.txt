cmake_minimum_required(VERSION 3.18.1)
project("xrstreamer")

find_library(log-lib log)

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
# xrstreamer
#
set(TARGET_NAME xrstreamer)
add_library(${TARGET_NAME} SHARED main.cpp)
target_compile_options(${TARGET_NAME} PRIVATE -Wall -Werror)
target_link_libraries(${TARGET_NAME} android native_app_glue ${log-lib} EGL GLESv1_CM)
