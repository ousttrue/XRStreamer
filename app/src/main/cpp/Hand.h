#pragma once
#include "HandTracking.h"
#include "HandRenderer.h"
#include "Input/TinyUI.h"

struct Hand
{
    bool isLeft_;
    HandTracking tracking_;
    HandRenderer renderer_;

    bool Init(XrInstance &Instance, XrSession &Session,
              const OVR::Vector4f &jointColor_,
              const OVR::Vector4f &capsuleColor_,
              PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT_,
              PFN_xrGetHandMeshFB xrGetHandMeshFB_);
    void Update(const OVRFW::ovrApplFrameIn &in, OVRFW::TinyUI &ui_);
    void Render(const OVRFW::ovrApplFrameIn &in, OVRFW::ovrRendererOutput &out);
};
