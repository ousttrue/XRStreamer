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
#include "HandsManager.h"
#include "HandRenderer.h"

#include "Input/SkeletonRenderer.h"
#include "Input/TinyUI.h"
#include "Render/SimpleBeamRenderer.h"

#define FORCE_ONLY_SIMPLE_CONTROLLER_PROFILE

class XrHandsApp : public OVRFW::XrApp
{
    HandsManager *hands_ = nullptr;

    /// Hands - FB mesh rendering extensions
    PFN_xrGetHandMeshFB xrGetHandMeshFB_ = nullptr;

    HandRenderer rendererL_;
    HandRenderer rendererR_;

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
                      [&r = rendererL_]()
                      { r.renderMesh = !r.renderMesh; });
        ui_.AddButton("Joints L", {-1.0f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [&r = rendererL_]()
                      { r.renderJoints = !r.renderJoints; });
        ui_.AddButton("Capsules L", {-1.0f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [&r = rendererL_]()
                      { r.renderCapsules = !r.renderCapsules; });

        ui_.AddButton("Mesh R", {1.20f, 1.75f, -2.0f}, {200.0f, 100.0f},
                      [&r = rendererR_]()
                      { r.renderMesh = !r.renderMesh; });
        ui_.AddButton("Joints R", {1.20f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [&r = rendererR_]()
                      { r.renderJoints = !r.renderJoints; });
        ui_.AddButton("Capsules R", {1.20f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [&r = rendererR_]()
                      { r.renderCapsules = !r.renderCapsules; });

        hands_ = HandsManager::Create(GetInstance(), GetSystemId());
        if (!hands_)
        {
            return false;
        }

        /// Hook up extensions for hand rendering
        OXR(xrGetInstanceProcAddr(GetInstance(), "xrGetHandMeshFB",
                                  (PFN_xrVoidFunction *)(&xrGetHandMeshFB_)));

        return true;
    }

    virtual void AppShutdown(const xrJava *context) override
    {
        /// unhook extensions for hand tracking
        delete hands_;
        hands_ = nullptr;
        xrGetHandMeshFB_ = nullptr;

        OVRFW::XrApp::AppShutdown(context);
        ui_.Shutdown();
    }

    bool InitA_Hand(bool isLeft, HandRenderer *renderer, HandTracker *hand)
    {
        // auto isLeft = handIndex == 0;
        // auto renderer = isLeft ? &rendererL_ : &rendererR_;
        // auto hand = isLeft ? &hands_->left_ : &hands_->right_;

        /// Alias everything for initialization
        if (!renderer->OnSessionInit(isLeft))
        {
            ALOG("AppInit::Init controller(%d) renderer FAILED.", isLeft);
            return false;
        }
        /// two-call pattern for mesh data
        /// call 1 - figure out sizes
        /// get mesh sizes
        OXR(xrGetHandMeshFB_(hand->handTracker, &renderer->mesh));
        renderer->OnMeshSize();
        /// call 2 - fill in the data
        /// get mesh data
        OXR(xrGetHandMeshFB_(hand->handTracker, &renderer->mesh));
        renderer->OnMeshData(hand, jointColor_, capsuleColor_);

        return true;
    }

    virtual bool SessionInit() override
    {
        /// Disable scene navitgation
        GetScene().SetFootPos({0.0f, 0.0f, 0.0f});
        this->FreeMove = false;

        beamRenderer_.Init(GetFileSys(), nullptr, OVR::Vector4f(1.0f), 1.0f);

        /// Hand Trackers
        hands_->OnSessionInit(GetInstance(), GetSession());

        /// Setup skinning meshes for both hands
        if (xrGetHandMeshFB_)
        {
            if (!InitA_Hand(true, &rendererL_, &hands_->left_))
            {
                return false;
            }
            if (!InitA_Hand(false, &rendererR_, &hands_->right_))
            {
                return false;
            }
        }

        return true;
    }

    virtual void SessionEnd() override
    {
        /// Hand Tracker
        hands_->Shutdown(GetInstance());
        beamRenderer_.Shutdown();
        rendererL_.Shutdown();
        rendererR_.Shutdown();
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
        hands_->Update(GetInstance(), GetStageSpace(), in.PredictedDisplayTime);

        std::vector<OVR::Posef> handJointsL;
        std::vector<OVR::Posef> handJointsR;

        // Determine which joints are actually tracked
        // XrSpaceLocationFlags isTracked =
        // XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT
        //    | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;

        // Tracked joints and computed joints can all be valid
        XrSpaceLocationFlags isValid = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                       XR_SPACE_LOCATION_POSITION_VALID_BIT;

        rendererL_.handTracked = false;
        rendererR_.handTracked = false;

        if (hands_->left_.locations.isActive)
        {
            for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
            {
                const OVR::Vector3f p0 =
                    FromXrVector3f(hands_->left_.capsuleState.capsules[i].points[0]);
                const OVR::Vector3f p1 =
                    FromXrVector3f(hands_->left_.capsuleState.capsules[i].points[1]);
                const OVR::Vector3f d = (p1 - p0);
                const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
                /// apply inverse scale here
                const float h = d.Length() / hands_->left_.scale.currentOutput;
                const OVR::Vector3f start =
                    p0 + look.Rotate(OVR::Vector3f(0, 0, -h / 2));
                OVRFW::GeometryRenderer &gr = rendererL_.handCapsuleRenderers[i];
                gr.SetScale(OVR::Vector3f(hands_->left_.scale.currentOutput));
                gr.SetPose(OVR::Posef(look, start));
                gr.Update();
            }
            for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
            {
                if ((hands_->left_.jointLocations[i].locationFlags & isValid) != 0)
                {
                    const auto p = FromXrPosef(hands_->left_.jointLocations[i].pose);
                    handJointsL.push_back(p);
                    rendererL_.handTracked = true;
                    OVRFW::GeometryRenderer &gr = rendererL_.handJointRenderers[i];
                    gr.SetScale(OVR::Vector3f(hands_->left_.scale.currentOutput));
                    gr.SetPose(p);
                    gr.Update();
                }
            }
            rendererL_.handRenderer.Update(&hands_->left_.jointLocations[0]);
            const bool didPinch = (hands_->left_.aimState.status &
                                   XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) != 0;
            ui_.AddHitTestRay(FromXrPosef(hands_->left_.aimState.aimPose),
                              didPinch && !rendererL_.lastFrameClicked);
            rendererL_.lastFrameClicked = didPinch;
        }
        rendererL_.axisRenderer.Update(handJointsL);
        if (in.LeftRemoteTracked)
        {
            rendererL_.controllerRender.Update(in.LeftRemotePose);
            const bool didPinch = in.LeftRemoteIndexTrigger > 0.5f;
            ui_.AddHitTestRay(in.LeftRemotePointPose, didPinch);
        }

        if (hands_->right_.locations.isActive)
        {
            for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
            {
                const OVR::Vector3f p0 =
                    FromXrVector3f(hands_->right_.capsuleState.capsules[i].points[0]);
                const OVR::Vector3f p1 =
                    FromXrVector3f(hands_->right_.capsuleState.capsules[i].points[1]);
                const OVR::Vector3f d = (p1 - p0);
                const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
                /// apply inverse scale here
                const float h = d.Length() / hands_->right_.scale.currentOutput;
                const OVR::Vector3f start =
                    p0 + look.Rotate(OVR::Vector3f(0, 0, -h / 2));
                OVRFW::GeometryRenderer &gr = rendererR_.handCapsuleRenderers[i];
                gr.SetScale(OVR::Vector3f(hands_->right_.scale.currentOutput));
                gr.SetPose(OVR::Posef(look, start));
                gr.Update();
            }
            for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
            {
                if ((hands_->right_.jointLocations[i].locationFlags & isValid) != 0)
                {
                    const auto p = FromXrPosef(hands_->right_.jointLocations[i].pose);
                    handJointsR.push_back(p);
                    rendererR_.handTracked = true;
                    OVRFW::GeometryRenderer &gr = rendererR_.handJointRenderers[i];
                    gr.SetScale(OVR::Vector3f(hands_->right_.scale.currentOutput));
                    gr.SetPose(p);
                    gr.Update();
                }
            }
            rendererR_.handRenderer.Update(&hands_->right_.jointLocations[0]);
            const bool didPinch = (hands_->right_.aimState.status &
                                   XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) != 0;

            ui_.AddHitTestRay(FromXrPosef(hands_->right_.aimState.aimPose),
                              didPinch && !rendererR_.lastFrameClicked);
            rendererR_.lastFrameClicked = didPinch;
        }
        rendererR_.axisRenderer.Update(handJointsR);
        if (in.RightRemoteTracked)
        {
            rendererR_.controllerRender.Update(in.RightRemotePose);
            const bool didPinch = in.RightRemoteIndexTrigger > 0.5f;
            ui_.AddHitTestRay(in.RightRemotePointPose, didPinch);
        }

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
        if (in.LeftRemoteTracked)
        {
            rendererL_.controllerRender.Render(out.Surfaces);
        }
        /// Render hand axes
        if (rendererL_.handTracked)
        {
            rendererL_.axisRenderer.Render(OVR::Matrix4f(), in, out);

            if (rendererL_.renderJoints)
            {
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = rendererL_.handJointRenderers[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (rendererL_.renderCapsules)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = rendererL_.handCapsuleRenderers[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (rendererL_.renderMesh)
            {
                rendererL_.handRenderer.Render(out.Surfaces);
            }
        }

        if (in.RightRemoteTracked)
        {
            rendererR_.controllerRender.Render(out.Surfaces);
        }
        if (rendererR_.handTracked)
        {
            rendererR_.axisRenderer.Render(OVR::Matrix4f(), in, out);

            if (rendererR_.renderJoints)
            {
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = rendererR_.handJointRenderers[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (rendererR_.renderCapsules)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = rendererR_.handCapsuleRenderers[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (rendererR_.renderMesh)
            {
                rendererR_.handRenderer.Render(out.Surfaces);
            }
        }

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
