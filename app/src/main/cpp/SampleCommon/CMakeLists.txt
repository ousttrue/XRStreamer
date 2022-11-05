#
# third_party
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

#
# SampleCommon
#
set(TARGET_NAME SampleCommon)
add_library(
  ${TARGET_NAME} STATIC
  GUI/ActionComponents.cpp
  GUI/AnimComponents.cpp
  GUI/CollisionPrimitive.cpp
  GUI/DefaultComponent.cpp
  GUI/Fader.cpp
  GUI/GazeCursor.cpp
  GUI/GuiSys.cpp
  GUI/MetaDataManager.cpp
  GUI/Reflection.cpp
  GUI/ReflectionData.cpp
  GUI/SoundLimiter.cpp
  GUI/VRMenu.cpp
  GUI/VRMenuComponent.cpp
  GUI/VRMenuEvent.cpp
  GUI/VRMenuEventHandler.cpp
  GUI/VRMenuMgr.cpp
  GUI/VRMenuObject.cpp
  Input/ArmModel.cpp
  Input/AxisRenderer.cpp
  Input/ControllerRenderer.cpp
  Input/Skeleton.cpp
  Input/SkeletonRenderer.cpp
  Input/TinyUI.cpp
  Locale/OVR_Locale.cpp
  Locale/tinyxml2.cpp
  Misc/Log.c
  Model/ModelCollision.cpp
  Model/ModelFile_glTF.cpp
  Model/ModelFile_OvrScene.cpp
  Model/ModelFile.cpp
  Model/ModelRender.cpp
  Model/ModelTrace.cpp
  Model/SceneView.cpp
  OVR_BinaryFile2.cpp
  OVR_FileSys.cpp
  OVR_Lexer2.cpp
  OVR_MappedFile.cpp
  OVR_Stream.cpp
  OVR_Uri.cpp
  OVR_UTF8Util.cpp
  PackageFiles.cpp
  Render/BeamRenderer.cpp
  Render/BitmapFont.cpp
  Render/DebugLines.cpp
  Render/EaseFunctions.cpp
  Render/Egl.c
  Render/GeometryBuilder.cpp
  Render/GeometryRenderer.cpp
  Render/GlBuffer.cpp
  Render/GlGeometry.cpp
  Render/GlProgram.cpp
  Render/GlTexture.cpp
  Render/PanelRenderer.cpp
  Render/ParticleSystem.cpp
  Render/PointList.cpp
  Render/Ribbon.cpp
  Render/SurfaceRender.cpp
  Render/SurfaceTexture.cpp
  Render/TextureAtlas.cpp
  Render/TextureManager.cpp
  System.cpp)
target_include_directories(
  ${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}
                        ${OVR_MOBILE_SDK_DIR}/1stParty/utilities/include)
target_link_libraries(${TARGET_NAME} PUBLIC minizip stb ktx)
