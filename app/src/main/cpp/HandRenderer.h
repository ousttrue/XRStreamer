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

    void Shutdown()
    {
        controllerRender.Shutdown();
        axisRenderer.Shutdown();
        handRenderer.Shutdown();
    }
};
