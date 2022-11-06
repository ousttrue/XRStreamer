/*******************************************************************************

Filename    :   Main.cpp
Content     :   Simple test app to test openxr hands
Created     :   Sept 2020
Authors     :   Federico Schliemann
Language    :   C++
Copyright:  Copyright (c) Facebook Technologies, LLC and its affiliates. All
rights reserved.

*******************************************************************************/

#include <cstdint>
#include <cstdio>
#include <openxr/openxr.h>

#include "XrApp.h"
#include "Hand.h"

#include "Input/SkeletonRenderer.h"
#include "Render/SimpleBeamRenderer.h"

#define FORCE_ONLY_SIMPLE_CONTROLLER_PROFILE

class XrHandsApp : public OVRFW::XrApp
{
    /// Hands - extension functions
    PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT_ = nullptr;
    PFN_xrDestroyHandTrackerEXT xrDestroyHandTrackerEXT_ = nullptr;
    PFN_xrLocateHandJointsEXT xrLocateHandJointsEXT_ = nullptr;
    /// Hands - FB mesh rendering extensions
    PFN_xrGetHandMeshFB xrGetHandMeshFB_ = nullptr;

    Hand left_;
    Hand right_;

    OVRFW::TinyUI ui_;
    OVRFW::SimpleBeamRenderer beamRenderer_;
    std::vector<OVRFW::ovrBeamRenderer::handle_t> beams_;

    OVR::Vector4f jointColor_{0.4, 0.5, 0.2, 0.5};
    OVR::Vector4f capsuleColor_{0.4, 0.2, 0.2, 0.5};

public:
    XrHandsApp()
        : OVRFW::XrApp()
    {
        BackgroundColor = OVR::Vector4f(0.60f, 0.95f, 0.4f, 1.0f);
        left_.isLeft_ = true;
        right_.isLeft_ = false;
    }

    // Returns a list of OpenXr extensions needed for this app
    virtual std::vector<const char *> GetExtensions() override
    {
        std::vector<const char *> extensions = XrApp::GetExtensions();
        extensions.push_back(XR_EXT_HAND_TRACKING_EXTENSION_NAME);
        extensions.push_back(XR_FB_HAND_TRACKING_MESH_EXTENSION_NAME);
        extensions.push_back(XR_FB_HAND_TRACKING_AIM_EXTENSION_NAME);
        extensions.push_back(XR_FB_HAND_TRACKING_CAPSULES_EXTENSION_NAME);
        return extensions;
    }

#ifdef FORCE_ONLY_SIMPLE_CONTROLLER_PROFILE
    // Returns a map from interaction profile paths to vectors of suggested
    // bindings. xrSuggestInteractionProfileBindings() is called once for each
    // interaction profile path in the returned map. Apps are encouraged to
    // suggest bindings for every device/interaction profile they support.
    // Override this for custom action bindings, or modify the default bindings.
    std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>>
    GetSuggestedBindings(XrInstance instance) override
    {
        // Get base suggested bindings
        std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>>
            allSuggestedBindings = XrApp::GetSuggestedBindings(instance);

        std::unordered_map<XrPath, std::vector<XrActionSuggestedBinding>>
            onlySimpleSuggestedBindings{};

        XrPath simpleInteractionProfile = XR_NULL_PATH;
        OXR(xrStringToPath(instance, "/interaction_profiles/khr/simple_controller",
                           &simpleInteractionProfile));

        // Only copy over suggested bindings for the simple interaction profile
        onlySimpleSuggestedBindings[simpleInteractionProfile] =
            allSuggestedBindings[simpleInteractionProfile];

        return onlySimpleSuggestedBindings;
    }
#endif

    // Must return true if the application initializes successfully.
    virtual bool AppInit(const xrJava *context) override
    {
        if (false == ui_.Init(context, GetFileSys()))
        {
            ALOG("TinyUI::Init FAILED.");
            return false;
        }
        /// Build UI
        ui_.AddLabel("OpenXR Hands + FB extensions Sample", {0.1f, 1.25f, -2.0f},
                     {1300.0f, 100.0f});
        ui_.AddButton("Red", {0.0f, 1.75f, -2.0f}, {200.0f, 100.0f}, [=]()
                      { BackgroundColor = OVR::Vector4f(0.8f, 0.05f, 0.05f, 1.0f); });
        ui_.AddButton("Green", {0.0f, 2.0f, -2.0f}, {200.0f, 100.0f}, [=]()
                      { BackgroundColor = OVR::Vector4f(0.0f, 0.65f, 0.1f, 1.0f); });
        ui_.AddButton("Blue", {0.0f, 2.25f, -2.0f}, {200.0f, 100.0f}, [=]()
                      { BackgroundColor = OVR::Vector4f(0.0f, 0.25f, 1.0f, 1.0f); });

        ui_.AddButton("Mesh L", {-1.0f, 1.75f, -2.0f}, {200.0f, 100.0f},
                      [&l = left_]()
                      { l.renderer_.renderMesh = !l.renderer_.renderMesh; });
        ui_.AddButton("Joints L", {-1.0f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [&l = left_]()
                      { l.renderer_.renderJoints = !l.renderer_.renderJoints; });
        ui_.AddButton("Capsules L", {-1.0f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [&l = left_]()
                      { l.renderer_.renderCapsules = !l.renderer_.renderCapsules; });

        ui_.AddButton("Mesh R", {1.20f, 1.75f, -2.0f}, {200.0f, 100.0f},
                      [&r = right_]()
                      { r.renderer_.renderMesh = !r.renderer_.renderMesh; });
        ui_.AddButton("Joints R", {1.20f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [&r = right_]()
                      { r.renderer_.renderJoints = !r.renderer_.renderJoints; });
        ui_.AddButton("Capsules R", {1.20f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [&r = right_]()
                      { r.renderer_.renderCapsules = !r.renderer_.renderCapsules; });

        // Inspect hand tracking system properties
        XrSystemHandTrackingPropertiesEXT handTrackingSystemProperties{
            XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT};
        XrSystemProperties systemProperties{XR_TYPE_SYSTEM_PROPERTIES,
                                            &handTrackingSystemProperties};
        OXR(xrGetSystemProperties(Instance, GetSystemId(), &systemProperties));
        if (!handTrackingSystemProperties.supportsHandTracking)
        {
            // The system does not support hand tracking
            ALOG("xrGetSystemProperties XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT "
                 "FAILED.");
            return false;
        }
        else
        {
            ALOG("xrGetSystemProperties XR_TYPE_SYSTEM_HAND_TRACKING_PROPERTIES_EXT "
                 "OK - initiallizing hand tracking...");
        }

        /// Hook up extensions for hand tracking
        OXR(xrGetInstanceProcAddr(
            Instance, "xrCreateHandTrackerEXT",
            (PFN_xrVoidFunction *)(&xrCreateHandTrackerEXT_)));
        OXR(xrGetInstanceProcAddr(
            Instance, "xrDestroyHandTrackerEXT",
            (PFN_xrVoidFunction *)(&xrDestroyHandTrackerEXT_)));
        OXR(xrGetInstanceProcAddr(Instance, "xrLocateHandJointsEXT",
                                  (PFN_xrVoidFunction *)(&xrLocateHandJointsEXT_)));
        /// Hook up extensions for hand rendering
        OXR(xrGetInstanceProcAddr(GetInstance(), "xrGetHandMeshFB",
                                  (PFN_xrVoidFunction *)(&xrGetHandMeshFB_)));

        return true;
    }

    virtual void AppShutdown(const xrJava *context) override
    {
        /// unhook extensions for hand tracking
        xrCreateHandTrackerEXT_ = nullptr;
        xrDestroyHandTrackerEXT_ = nullptr;
        xrLocateHandJointsEXT_ = nullptr;
        xrGetHandMeshFB_ = nullptr;

        OVRFW::XrApp::AppShutdown(context);
        ui_.Shutdown();
    }

    virtual bool SessionInit() override
    {
        /// Disable scene navitgation
        GetScene().SetFootPos({0.0f, 0.0f, 0.0f});
        this->FreeMove = false;

        beamRenderer_.Init(GetFileSys(), nullptr, OVR::Vector4f(1.0f), 1.0f);

        left_.Init(GetInstance(), GetSession(), jointColor_, capsuleColor_, xrCreateHandTrackerEXT_, xrGetHandMeshFB_);
        right_.Init(GetInstance(), GetSession(), jointColor_, capsuleColor_, xrCreateHandTrackerEXT_, xrGetHandMeshFB_);

        return true;
    }

    virtual void SessionEnd() override
    {
        /// Hand Tracker
        if (xrDestroyHandTrackerEXT_)
        {
            OXR(xrDestroyHandTrackerEXT_(left_.tracking_.handTracker));
            OXR(xrDestroyHandTrackerEXT_(right_.tracking_.handTracker));
        }

        beamRenderer_.Shutdown();
        left_.renderer_.Shutdown();
        right_.renderer_.Shutdown();
    }

    // Update state
    virtual void Update(const OVRFW::ovrApplFrameIn &in) override
    {
        ui_.HitTestDevices().clear();

        if ((in.AllButtons & OVRFW::ovrApplFrameIn::kButtonY) != 0)
        {
            ALOG("Y button is pressed!");
        }
        if ((in.AllButtons & OVRFW::ovrApplFrameIn::kButtonMenu) != 0)
        {
            ALOG("Menu button is pressed!");
        }
        if ((in.AllButtons & OVRFW::ovrApplFrameIn::kButtonA) != 0)
        {
            ALOG("A button is pressed!");
        }
        if ((in.AllButtons & OVRFW::ovrApplFrameIn::kButtonB) != 0)
        {
            ALOG("B button is pressed!");
        }
        if ((in.AllButtons & OVRFW::ovrApplFrameIn::kButtonX) != 0)
        {
            ALOG("X button is pressed!");
        }

        /// Hands
        /// L
        left_.tracking_.Update(GetStageSpace(), in.PredictedDisplayTime);
        OXR(xrLocateHandJointsEXT_(left_.tracking_.handTracker, &left_.tracking_.locateInfo, &left_.tracking_.locations));
        left_.Update(in, ui_);

        /// R
        right_.tracking_.Update(GetStageSpace(), in.PredictedDisplayTime);
        OXR(xrLocateHandJointsEXT_(right_.tracking_.handTracker, &right_.tracking_.locateInfo, &right_.tracking_.locations));
        right_.Update(in, ui_);

        /// UI
        ui_.Update(in);
        beamRenderer_.Update(in, ui_.HitTestDevices());
    }

    // Render eye buffers while running
    virtual void Render(const OVRFW::ovrApplFrameIn &in,
                        OVRFW::ovrRendererOutput &out) override
    {
        /// Render UI
        ui_.Render(in, out);

        /// Render controllers
        left_.Render(in, out);
        right_.Render(in, out);

        /// Render beams
        beamRenderer_.Render(in, out);
    }
};

// ENTRY_POINT(XrHandsApp)
void android_main(struct android_app *app)
{
    auto appl = std::make_unique<XrHandsApp>();
    appl->Run(app);
}
