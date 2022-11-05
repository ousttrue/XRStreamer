cmake_minimum_required(VERSION 3.18.1)
project("xrstreamer")
get_filename_component(OVR_MOBILE_SDK_DIR "$ENV{OVR_MOBILE_SDK_DIR}" ABSOLUTE)
# file(TO_CMAKE_PATH "${OVR_MOBILE_SDK_DIR}" OVR_MOBILE_SDK_DIR)
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

#
# minizip
#
set(TARGET_NAME minizip)
add_library(
  ${TARGET_NAME} STATIC
  ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src/ioapi.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src/miniunz.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src/mztools.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src/unzip.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src/zip.c)
target_include_directories(${TARGET_NAME}
                           PUBLIC ${OVR_MOBILE_SDK_DIR}/3rdParty/minizip/src)
target_link_libraries(${TARGET_NAME} PRIVATE z)

set(TARGET_NAME stb)
add_library(
  ${TARGET_NAME} STATIC
  ${OVR_MOBILE_SDK_DIR}/3rdParty/stb/src/stb_image.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/stb/src/stb_image_write.c
  ${OVR_MOBILE_SDK_DIR}/3rdParty/stb/src/stb_vorbis.c)
target_include_directories(${TARGET_NAME}
                           PUBLIC ${OVR_MOBILE_SDK_DIR}/3rdParty/stb/src)

set(TARGET_NAME ktx)
add_library(${TARGET_NAME} SHARED IMPORTED)
set_target_properties(
  ${TARGET_NAME}
  PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
    ${OVR_MOBILE_SDK_DIR}/3rdParty/khronos/ktx/include
    IMPORTED_LOCATION
    ${OVR_MOBILE_SDK_DIR}/3rdParty/khronos/ktx/lib/android/arm64-v8a/libktx_read.so
    ${OVR_MOBILE_SDK_DIR}/3rdParty/khronos/ktx/lib/android/arm64-v8a/libktx.so
    ${OVR_MOBILE_SDK_DIR}/3rdParty/khronos/ktx/lib/android/arm64-v8a/libobjUtil.a
)

set(OVR_DIR ${OVR_MOBILE_SDK_DIR}/1stParty/OVR)
set(TARGET_NAME OVR)
add_library(${TARGET_NAME} INTERFACE)
target_include_directories(${TARGET_NAME} INTERFACE ${OVR_DIR}/Include)

set(SAMPLE_COMMON_DIR ${OVR_MOBILE_SDK_DIR}/SampleCommon/Src)
set(TARGET_NAME SampleCommon)
add_library(
  ${TARGET_NAME} STATIC
  ${SAMPLE_COMMON_DIR}/GUI/ActionComponents.cpp
  ${SAMPLE_COMMON_DIR}/GUI/AnimComponents.cpp
  ${SAMPLE_COMMON_DIR}/GUI/CollisionPrimitive.cpp
  ${SAMPLE_COMMON_DIR}/GUI/DefaultComponent.cpp
  ${SAMPLE_COMMON_DIR}/GUI/Fader.cpp
  ${SAMPLE_COMMON_DIR}/GUI/GazeCursor.cpp
  ${SAMPLE_COMMON_DIR}/GUI/GuiSys.cpp
  ${SAMPLE_COMMON_DIR}/GUI/MetaDataManager.cpp
  ${SAMPLE_COMMON_DIR}/GUI/Reflection.cpp
  ${SAMPLE_COMMON_DIR}/GUI/ReflectionData.cpp
  ${SAMPLE_COMMON_DIR}/GUI/SoundLimiter.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenu.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenuComponent.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenuEvent.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenuEventHandler.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenuMgr.cpp
  ${SAMPLE_COMMON_DIR}/GUI/VRMenuObject.cpp
  ${SAMPLE_COMMON_DIR}/Input/ArmModel.cpp
  ${SAMPLE_COMMON_DIR}/Input/AxisRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Input/ControllerRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Input/Skeleton.cpp
  ${SAMPLE_COMMON_DIR}/Input/SkeletonRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Input/TinyUI.cpp
  ${SAMPLE_COMMON_DIR}/Locale/OVR_Locale.cpp
  ${SAMPLE_COMMON_DIR}/Locale/tinyxml2.cpp
  ${SAMPLE_COMMON_DIR}/Misc/Log.c
  ${SAMPLE_COMMON_DIR}/Model/ModelCollision.cpp
  ${SAMPLE_COMMON_DIR}/Model/ModelFile_glTF.cpp
  ${SAMPLE_COMMON_DIR}/Model/ModelFile_OvrScene.cpp
  ${SAMPLE_COMMON_DIR}/Model/ModelFile.cpp
  ${SAMPLE_COMMON_DIR}/Model/ModelRender.cpp
  ${SAMPLE_COMMON_DIR}/Model/ModelTrace.cpp
  ${SAMPLE_COMMON_DIR}/Model/SceneView.cpp
  ${SAMPLE_COMMON_DIR}/OVR_BinaryFile2.cpp
  ${SAMPLE_COMMON_DIR}/OVR_FileSys.cpp
  ${SAMPLE_COMMON_DIR}/OVR_Lexer2.cpp
  ${SAMPLE_COMMON_DIR}/OVR_MappedFile.cpp
  ${SAMPLE_COMMON_DIR}/OVR_Stream.cpp
  ${SAMPLE_COMMON_DIR}/OVR_Uri.cpp
  ${SAMPLE_COMMON_DIR}/OVR_UTF8Util.cpp
  ${SAMPLE_COMMON_DIR}/PackageFiles.cpp
  ${SAMPLE_COMMON_DIR}/Render/BeamRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Render/BitmapFont.cpp
  ${SAMPLE_COMMON_DIR}/Render/DebugLines.cpp
  ${SAMPLE_COMMON_DIR}/Render/EaseFunctions.cpp
  ${SAMPLE_COMMON_DIR}/Render/Egl.c
  ${SAMPLE_COMMON_DIR}/Render/GeometryBuilder.cpp
  ${SAMPLE_COMMON_DIR}/Render/GeometryRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Render/GlBuffer.cpp
  ${SAMPLE_COMMON_DIR}/Render/GlGeometry.cpp
  ${SAMPLE_COMMON_DIR}/Render/GlProgram.cpp
  ${SAMPLE_COMMON_DIR}/Render/GlTexture.cpp
  ${SAMPLE_COMMON_DIR}/Render/PanelRenderer.cpp
  ${SAMPLE_COMMON_DIR}/Render/ParticleSystem.cpp
  ${SAMPLE_COMMON_DIR}/Render/PointList.cpp
  ${SAMPLE_COMMON_DIR}/Render/Ribbon.cpp
  ${SAMPLE_COMMON_DIR}/Render/SurfaceRender.cpp
  ${SAMPLE_COMMON_DIR}/Render/SurfaceTexture.cpp
  ${SAMPLE_COMMON_DIR}/Render/TextureAtlas.cpp
  ${SAMPLE_COMMON_DIR}/Render/TextureManager.cpp
  ${SAMPLE_COMMON_DIR}/System.cpp)
target_include_directories(
  ${TARGET_NAME} PUBLIC ${SAMPLE_COMMON_DIR}
                        ${OVR_MOBILE_SDK_DIR}/1stParty/utilities/include)
target_link_libraries(${TARGET_NAME} PUBLIC OVR minizip stb ktx)

set(SAMPLE_FRAMEWORK_DIR ${OVR_MOBILE_SDK_DIR}/SampleXrFramework/Src)
set(TARGET_NAME SampleXrFramewok)
add_library(
  ${TARGET_NAME} STATIC
  ${SAMPLE_FRAMEWORK_DIR}/XrApp.cpp
  ${SAMPLE_FRAMEWORK_DIR}/Input/HandMaskRenderer.cpp
  ${SAMPLE_FRAMEWORK_DIR}/Input/HandRenderer.cpp
  ${SAMPLE_FRAMEWORK_DIR}/Render/Framebuffer.cpp)
target_include_directories(${TARGET_NAME} PUBLIC ${SAMPLE_FRAMEWORK_DIR})
target_link_libraries(${TARGET_NAME} PUBLIC native_app_glue OVR SampleCommon
                                            openxr_loader)

#
# xrstreamer
#
set(TARGET_NAME xrstreamer)
add_library(${TARGET_NAME} SHARED ${OVR_MOBILE_SDK_DIR}/XrSamples/XrHandsFB/Src/main.cpp)
target_compile_options(${TARGET_NAME} PRIVATE -Wall -Werror)
target_link_libraries(
  ${TARGET_NAME}
  PRIVATE OVR
          SampleCommon
          SampleXrFramewok
          openxr_loader
          android
          log
          GLESv3
          EGL)
