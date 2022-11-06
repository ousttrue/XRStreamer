#pragma once
#include "Input/AxisRenderer.h"
#include "Input/ControllerRenderer.h"
#include "Input/HandRenderer.h"
#include "Render/GeometryRenderer.h"

struct HandRenderer
{
    OVRFW::ControllerRenderer controllerRender;
    OVRFW::HandRenderer handRenderer;
    OVRFW::ovrAxisRenderer axisRenderer;
    bool handTracked = false;
    bool lastFrameClicked = false;
    bool renderMesh = true;
    bool renderJoints = true;
    bool renderCapsules = true;
    std::vector<OVRFW::GeometryRenderer> handJointRenderers;
    std::vector<OVRFW::GeometryRenderer> handCapsuleRenderers;
    XrHandTrackingMeshFB mesh;
    std::vector<XrPosef> jointBindLocations;
    std::vector<XrHandJointEXT> parentData;
    std::vector<float> jointRadii;
    std::vector<XrVector3f> vertexPositions;
    std::vector<XrVector3f> vertexNormals;
    std::vector<XrVector2f> vertexUVs;
    std::vector<XrVector4sFB> vertexBlendIndices;
    std::vector<XrVector4f> vertexBlendWeights;
    std::vector<int16_t> indices;
    XrHandTrackingCapsulesStateFB capsuleState;

    bool OnSessionInit(bool isLeft);
    void OnMeshSize();
    void OnMeshData(struct HandTracker *hand,
                    const OVR::Vector4f &jointColor_,
                    const OVR::Vector4f &capsuleColor_);

    void Shutdown()
    {
        controllerRender.Shutdown();
        axisRenderer.Shutdown();
        handRenderer.Shutdown();
    }
};
