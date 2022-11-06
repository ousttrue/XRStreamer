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
#include "XrHands.h"

#include "Input/AxisRenderer.h"
#include "Input/ControllerRenderer.h"
#include "Input/HandRenderer.h"
#include "Input/SkeletonRenderer.h"
#include "Input/TinyUI.h"
#include "Render/GeometryRenderer.h"
#include "Render/SimpleBeamRenderer.h"

#define FORCE_ONLY_SIMPLE_CONTROLLER_PROFILE

class XrHandsApp : public OVRFW::XrApp
{
    XrHands *_hands = nullptr;

    /// Hands - FB mesh rendering extensions
    PFN_xrGetHandMeshFB xrGetHandMeshFB_ = nullptr;

    OVRFW::ControllerRenderer controllerRenderL_;
    OVRFW::ControllerRenderer controllerRenderR_;
    OVRFW::HandRenderer handRendererL_;
    OVRFW::HandRenderer handRendererR_;
    OVRFW::TinyUI ui_;
    OVRFW::SimpleBeamRenderer beamRenderer_;
    std::vector<OVRFW::ovrBeamRenderer::handle_t> beams_;
    OVRFW::ovrAxisRenderer axisRendererL_;
    OVRFW::ovrAxisRenderer axisRendererR_;
    bool handTrackedL_ = false;
    bool handTrackedR_ = false;

    bool lastFrameClickedL_ = false;
    bool lastFrameClickedR_ = false;

    OVR::Vector4f jointColor_{0.4, 0.5, 0.2, 0.5};
    OVR::Vector4f capsuleColor_{0.4, 0.2, 0.2, 0.5};

    bool renderMeshL_ = true;
    bool renderMeshR_ = true;
    bool renderJointsL_ = true;
    bool renderJointsR_ = true;
    bool renderCapsulesL_ = true;
    bool renderCapsulesR_ = true;
    std::vector<OVRFW::GeometryRenderer> handJointRenderersL_;
    std::vector<OVRFW::GeometryRenderer> handJointRenderersR_;
    std::vector<OVRFW::GeometryRenderer> handCapsuleRenderersL_;
    std::vector<OVRFW::GeometryRenderer> handCapsuleRenderersR_;

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
                      [=]()
                      { renderMeshL_ = !renderMeshL_; });
        ui_.AddButton("Joints L", {-1.0f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [=]()
                      { renderJointsL_ = !renderJointsL_; });
        ui_.AddButton("Capsules L", {-1.0f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [=]()
                      { renderCapsulesL_ = !renderCapsulesL_; });

        ui_.AddButton("Mesh R", {1.20f, 1.75f, -2.0f}, {200.0f, 100.0f},
                      [=]()
                      { renderMeshR_ = !renderMeshR_; });
        ui_.AddButton("Joints R", {1.20f, 2.0f, -2.0f}, {200.0f, 100.0f},
                      [=]()
                      { renderJointsR_ = !renderJointsR_; });
        ui_.AddButton("Capsules R", {1.20f, 2.25f, -2.0f}, {200.0f, 100.0f},
                      [=]()
                      { renderCapsulesR_ = !renderCapsulesR_; });

        _hands = XrHands::Create(GetInstance(), GetSystemId());
        if(!_hands)
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
        delete _hands;
        _hands = nullptr;
        xrGetHandMeshFB_ = nullptr;

        OVRFW::XrApp::AppShutdown(context);
        ui_.Shutdown();
    }

    virtual bool SessionInit() override
    {
        /// Disable scene navitgation
        GetScene().SetFootPos({0.0f, 0.0f, 0.0f});
        this->FreeMove = false;
        /// Init session bound objects
        if (false == controllerRenderL_.Init(true))
        {
            ALOG("AppInit::Init L controller renderer FAILED.");
            return false;
        }
        if (false == controllerRenderR_.Init(false))
        {
            ALOG("AppInit::Init R controller renderer FAILED.");
            return false;
        }
        beamRenderer_.Init(GetFileSys(), nullptr, OVR::Vector4f(1.0f), 1.0f);

        /// Hand rendering
        axisRendererL_.Init();
        axisRendererR_.Init();

        /// Hand Trackers
        _hands->OnSessionInit(GetInstance(), GetSession());
        // if (xrCreateHandTrackerEXT_)
        {
            /// Setup skinning meshes for both hands
            if (xrGetHandMeshFB_)
            {
                for (int handIndex = 0; handIndex < 2; ++handIndex)
                {
                    /// Alias everything for initialization
                    const bool isLeft = (handIndex == 0);
                    auto &handTracker = isLeft ? _hands->handTrackerL_ : _hands->handTrackerR_;
                    auto &handRenderer = isLeft ? handRendererL_ : handRendererR_;
                    auto &handJointRenderers =
                        isLeft ? handJointRenderersL_ : handJointRenderersR_;
                    auto *jointLocations = isLeft ? _hands->jointLocationsL_ : _hands->jointLocationsR_;
                    auto &handCapsuleRenderers =
                        isLeft ? handCapsuleRenderersL_ : handCapsuleRenderersR_;

                    /// two-call pattern for mesh data
                    /// call 1 - figure out sizes

                    /// mesh
                    XrHandTrackingMeshFB mesh{XR_TYPE_HAND_TRACKING_MESH_FB};
                    mesh.next = nullptr;
                    /// mesh - skeleton
                    mesh.jointCapacityInput = 0;
                    mesh.jointCountOutput = 0;
                    mesh.jointBindPoses = nullptr;
                    mesh.jointRadii = nullptr;
                    mesh.jointParents = nullptr;
                    /// mesh - vertex
                    mesh.vertexCapacityInput = 0;
                    mesh.vertexCountOutput = 0;
                    mesh.vertexPositions = nullptr;
                    mesh.vertexNormals = nullptr;
                    mesh.vertexUVs = nullptr;
                    mesh.vertexBlendIndices = nullptr;
                    mesh.vertexBlendWeights = nullptr;
                    /// mesh - index
                    mesh.indexCapacityInput = 0;
                    mesh.indexCountOutput = 0;
                    mesh.indices = nullptr;
                    /// get mesh sizes
                    OXR(xrGetHandMeshFB_(handTracker, &mesh));

                    /// mesh storage - update sizes
                    mesh.jointCapacityInput = mesh.jointCountOutput;
                    mesh.vertexCapacityInput = mesh.vertexCountOutput;
                    mesh.indexCapacityInput = mesh.indexCountOutput;
                    /// skeleton
                    std::vector<XrPosef> jointBindLocations;
                    std::vector<XrHandJointEXT> parentData;
                    std::vector<float> jointRadii;
                    jointBindLocations.resize(mesh.jointCountOutput);
                    parentData.resize(mesh.jointCountOutput);
                    jointRadii.resize(mesh.jointCountOutput);
                    mesh.jointBindPoses = jointBindLocations.data();
                    mesh.jointParents = parentData.data();
                    mesh.jointRadii = jointRadii.data();
                    /// vertex
                    std::vector<XrVector3f> vertexPositions;
                    std::vector<XrVector3f> vertexNormals;
                    std::vector<XrVector2f> vertexUVs;
                    std::vector<XrVector4sFB> vertexBlendIndices;
                    std::vector<XrVector4f> vertexBlendWeights;
                    vertexPositions.resize(mesh.vertexCountOutput);
                    vertexNormals.resize(mesh.vertexCountOutput);
                    vertexUVs.resize(mesh.vertexCountOutput);
                    vertexBlendIndices.resize(mesh.vertexCountOutput);
                    vertexBlendWeights.resize(mesh.vertexCountOutput);
                    mesh.vertexPositions = vertexPositions.data();
                    mesh.vertexNormals = vertexNormals.data();
                    mesh.vertexUVs = vertexUVs.data();
                    mesh.vertexBlendIndices = vertexBlendIndices.data();
                    mesh.vertexBlendWeights = vertexBlendWeights.data();
                    /// index
                    std::vector<int16_t> indices;
                    indices.resize(mesh.indexCountOutput);
                    mesh.indices = indices.data();

                    /// call 2 - fill in the data
                    /// chain capsules
                    XrHandTrackingCapsulesStateFB capsuleState{
                        XR_TYPE_HAND_TRACKING_CAPSULES_STATE_FB};
                    capsuleState.next = nullptr;
                    mesh.next = &capsuleState;

                    /// get mesh data
                    OXR(xrGetHandMeshFB_(handTracker, &mesh));
                    /// init renderer
                    handRenderer.Init(&mesh, true);
                    /// Render jointRadius for all left hand joints
                    {
                        handJointRenderers.resize(XR_HAND_JOINT_COUNT_EXT);
                        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                        {
                            const OVR::Posef pose = FromXrPosef(jointLocations[i].pose);
                            OVRFW::GeometryRenderer &gr = handJointRenderers[i];
                            gr.Init(OVRFW::BuildTesselatedCapsuleDescriptor(
                                mesh.jointRadii[i], 0.0f, 7, 7));
                            gr.SetPose(pose);
                            gr.DiffuseColor = jointColor_;
                        }
                    }
                    /// One time init for capsules
                    {
                        handCapsuleRenderers.resize(XR_FB_HAND_TRACKING_CAPSULE_COUNT);
                        for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                        {
                            const OVR::Vector3f p0 =
                                FromXrVector3f(capsuleState.capsules[i].points[0]);
                            const OVR::Vector3f p1 =
                                FromXrVector3f(capsuleState.capsules[i].points[1]);
                            const OVR::Vector3f d = (p1 - p0);
                            const float h = d.Length();
                            const float r = capsuleState.capsules[i].radius;
                            const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
                            OVRFW::GeometryRenderer &gr = handCapsuleRenderers[i];
                            gr.Init(OVRFW::BuildTesselatedCapsuleDescriptor(r, h, 7, 7));
                            gr.SetPose(OVR::Posef(look, p0));
                            gr.DiffuseColor = capsuleColor_;
                        }
                    }
                    /// Print hierarchy
                    {
                        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                        {
                            const OVR::Posef pose = FromXrPosef(jointLocations[i].pose);
                            ALOG(" { {%.6f, %.6f, %.6f},  {%.6f, %.6f, %.6f, %.6f} } // "
                                 "joint = %d, parent = %d",
                                 pose.Translation.x, pose.Translation.y, pose.Translation.z,
                                 pose.Rotation.x, pose.Rotation.y, pose.Rotation.z,
                                 pose.Rotation.w, i, (int)parentData[i]);
                        }
                    }
                }
            }
        }

        return true;
    }

    virtual void SessionEnd() override
    {
        /// Hand Tracker
        _hands->Shutdown(GetInstance());
        controllerRenderL_.Shutdown();
        controllerRenderR_.Shutdown();
        beamRenderer_.Shutdown();
        axisRendererL_.Shutdown();
        axisRendererR_.Shutdown();
        handRendererL_.Shutdown();
        handRendererR_.Shutdown();
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
        _hands->Update(GetInstance(), GetStageSpace(), in.PredictedDisplayTime);
        {
            std::vector<OVR::Posef> handJointsL;
            std::vector<OVR::Posef> handJointsR;

            // Determine which joints are actually tracked
            // XrSpaceLocationFlags isTracked =
            // XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT
            //    | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;

            // Tracked joints and computed joints can all be valid
            XrSpaceLocationFlags isValid = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                           XR_SPACE_LOCATION_POSITION_VALID_BIT;

            handTrackedL_ = false;
            handTrackedR_ = false;

            if (_hands->locationsL.isActive)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    const OVR::Vector3f p0 =
                        FromXrVector3f(_hands->capsuleStateL.capsules[i].points[0]);
                    const OVR::Vector3f p1 =
                        FromXrVector3f(_hands->capsuleStateL.capsules[i].points[1]);
                    const OVR::Vector3f d = (p1 - p0);
                    const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
                    /// apply inverse scale here
                    const float h = d.Length() / _hands->scaleL.currentOutput;
                    const OVR::Vector3f start =
                        p0 + look.Rotate(OVR::Vector3f(0, 0, -h / 2));
                    OVRFW::GeometryRenderer &gr = handCapsuleRenderersL_[i];
                    gr.SetScale(OVR::Vector3f(_hands->scaleL.currentOutput));
                    gr.SetPose(OVR::Posef(look, start));
                    gr.Update();
                }
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    if ((_hands->jointLocationsL_[i].locationFlags & isValid) != 0)
                    {
                        const auto p = FromXrPosef(_hands->jointLocationsL_[i].pose);
                        handJointsL.push_back(p);
                        handTrackedL_ = true;
                        OVRFW::GeometryRenderer &gr = handJointRenderersL_[i];
                        gr.SetScale(OVR::Vector3f(_hands->scaleL.currentOutput));
                        gr.SetPose(p);
                        gr.Update();
                    }
                }
                handRendererL_.Update(&_hands->jointLocationsL_[0]);
                const bool didPinch = (_hands->aimStateL.status &
                                       XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) != 0;
                ui_.AddHitTestRay(FromXrPosef(_hands->aimStateL.aimPose),
                                  didPinch && !lastFrameClickedL_);
                lastFrameClickedL_ = didPinch;
            }
            if (_hands->locationsR.isActive)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    const OVR::Vector3f p0 =
                        FromXrVector3f(_hands->capsuleStateR.capsules[i].points[0]);
                    const OVR::Vector3f p1 =
                        FromXrVector3f(_hands->capsuleStateR.capsules[i].points[1]);
                    const OVR::Vector3f d = (p1 - p0);
                    const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
                    /// apply inverse scale here
                    const float h = d.Length() / _hands->scaleR.currentOutput;
                    const OVR::Vector3f start =
                        p0 + look.Rotate(OVR::Vector3f(0, 0, -h / 2));
                    OVRFW::GeometryRenderer &gr = handCapsuleRenderersR_[i];
                    gr.SetScale(OVR::Vector3f(_hands->scaleR.currentOutput));
                    gr.SetPose(OVR::Posef(look, start));
                    gr.Update();
                }
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    if ((_hands->jointLocationsR_[i].locationFlags & isValid) != 0)
                    {
                        const auto p = FromXrPosef(_hands->jointLocationsR_[i].pose);
                        handJointsR.push_back(p);
                        handTrackedR_ = true;
                        OVRFW::GeometryRenderer &gr = handJointRenderersR_[i];
                        gr.SetScale(OVR::Vector3f(_hands->scaleR.currentOutput));
                        gr.SetPose(p);
                        gr.Update();
                    }
                }
                handRendererR_.Update(&_hands->jointLocationsR_[0]);
                const bool didPinch = (_hands->aimStateR.status &
                                       XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) != 0;

                ui_.AddHitTestRay(FromXrPosef(_hands->aimStateR.aimPose),
                                  didPinch && !lastFrameClickedR_);
                lastFrameClickedR_ = didPinch;
            }
            axisRendererL_.Update(handJointsL);
            axisRendererR_.Update(handJointsR);
        }

        if (in.LeftRemoteTracked)
        {
            controllerRenderL_.Update(in.LeftRemotePose);
            const bool didPinch = in.LeftRemoteIndexTrigger > 0.5f;
            ui_.AddHitTestRay(in.LeftRemotePointPose, didPinch);
        }
        if (in.RightRemoteTracked)
        {
            controllerRenderR_.Update(in.RightRemotePose);
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
            controllerRenderL_.Render(out.Surfaces);
        }
        if (in.RightRemoteTracked)
        {
            controllerRenderR_.Render(out.Surfaces);
        }

        /// Render hand axes
        if (handTrackedL_)
        {
            axisRendererL_.Render(OVR::Matrix4f(), in, out);

            if (renderJointsL_)
            {
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = handJointRenderersL_[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (renderCapsulesL_)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = handCapsuleRenderersL_[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (renderMeshL_)
            {
                handRendererL_.Render(out.Surfaces);
            }
        }
        if (handTrackedR_)
        {
            axisRendererR_.Render(OVR::Matrix4f(), in, out);

            if (renderJointsR_)
            {
                for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = handJointRenderersR_[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (renderCapsulesR_)
            {
                for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
                {
                    OVRFW::GeometryRenderer &gr = handCapsuleRenderersR_[i];
                    gr.Render(out.Surfaces);
                }
            }

            if (renderMeshR_)
            {
                handRendererR_.Render(out.Surfaces);
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
