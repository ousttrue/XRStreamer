#include "Hand.h"
#include "XrApp.h"

bool Hand::Init(XrInstance &Instance, XrSession &Session,
                const OVR::Vector4f &jointColor_,
                const OVR::Vector4f &capsuleColor_,
                PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT_,
                PFN_xrGetHandMeshFB xrGetHandMeshFB_)
{
    /// Hand Trackers
    auto hand = isLeft_ ? XR_HAND_LEFT_EXT : XR_HAND_RIGHT_EXT;
    OXR(xrCreateHandTrackerEXT_(Session, tracking_.CreateInfo(hand), &tracking_.handTracker));
    ALOG("xrCreateHandTrackerEXT handTrackerL_=%llx",
         (long long)tracking_.handTracker);

    /// Setup skinning meshes for both hands
    if (xrGetHandMeshFB_)
    {
        /// Alias everything for initialization
        if (!renderer_.OnSessionInit(isLeft_))
        {
            ALOG("AppInit::Init controller(%d) renderer FAILED.", isLeft_);
            return false;
        }
        /// two-call pattern for mesh data
        /// call 1 - figure out sizes
        /// get mesh sizes
        OXR(xrGetHandMeshFB_(tracking_.handTracker, &renderer_.mesh));
        renderer_.OnMeshSize();
        /// call 2 - fill in the data
        /// get mesh data
        OXR(xrGetHandMeshFB_(tracking_.handTracker, &renderer_.mesh));
        renderer_.OnMeshData(&tracking_, jointColor_, capsuleColor_);
    }
    return true;
}

void Hand::Update(const OVRFW::ovrApplFrameIn &in, OVRFW::TinyUI &ui_)
{
    std::vector<OVR::Posef> handJointsL;

    // Determine which joints are actually tracked
    // XrSpaceLocationFlags isTracked =
    // XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT
    //    | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;

    // Tracked joints and computed joints can all be valid
    XrSpaceLocationFlags isValid = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
                                   XR_SPACE_LOCATION_POSITION_VALID_BIT;

    renderer_.handTracked = false;

    if (tracking_.locations.isActive)
    {
        for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
        {
            const OVR::Vector3f p0 =
                FromXrVector3f(tracking_.capsuleState.capsules[i].points[0]);
            const OVR::Vector3f p1 =
                FromXrVector3f(tracking_.capsuleState.capsules[i].points[1]);
            const OVR::Vector3f d = (p1 - p0);
            const OVR::Quatf look = OVR::Quatf::LookRotation(d, {0, 1, 0});
            /// apply inverse scale here
            const float h = d.Length() / tracking_.scale.currentOutput;
            const OVR::Vector3f start =
                p0 + look.Rotate(OVR::Vector3f(0, 0, -h / 2));
            OVRFW::GeometryRenderer &gr = renderer_.handCapsuleRenderers[i];
            gr.SetScale(OVR::Vector3f(tracking_.scale.currentOutput));
            gr.SetPose(OVR::Posef(look, start));
            gr.Update();
        }
        for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
        {
            if ((tracking_.jointLocations[i].locationFlags & isValid) != 0)
            {
                const auto p = FromXrPosef(tracking_.jointLocations[i].pose);
                handJointsL.push_back(p);
                renderer_.handTracked = true;
                OVRFW::GeometryRenderer &gr = renderer_.handJointRenderers[i];
                gr.SetScale(OVR::Vector3f(tracking_.scale.currentOutput));
                gr.SetPose(p);
                gr.Update();
            }
        }
        renderer_.handRenderer.Update(&tracking_.jointLocations[0]);
        const bool didPinch = (tracking_.aimState.status &
                               XR_HAND_TRACKING_AIM_INDEX_PINCHING_BIT_FB) != 0;
        ui_.AddHitTestRay(FromXrPosef(tracking_.aimState.aimPose),
                          didPinch && !renderer_.lastFrameClicked);
        renderer_.lastFrameClicked = didPinch;
    }
    renderer_.axisRenderer.Update(handJointsL);

    if (isLeft_)
    {
        if (in.LeftRemoteTracked)
        {
            renderer_.controllerRender.Update(in.LeftRemotePose);
            const bool didPinch = in.LeftRemoteIndexTrigger > 0.5f;
            ui_.AddHitTestRay(in.LeftRemotePointPose, didPinch);
        }
    }
    else
    {
        if (in.RightRemoteTracked)
        {
            renderer_.controllerRender.Update(in.RightRemotePose);
            const bool didPinch = in.RightRemoteIndexTrigger > 0.5f;
            ui_.AddHitTestRay(in.RightRemotePointPose, didPinch);
        }
    }
}

void Hand::Render(const OVRFW::ovrApplFrameIn &in, OVRFW::ovrRendererOutput &out)
{
    if (isLeft_)
    {
        if (in.LeftRemoteTracked)
        {
            renderer_.controllerRender.Render(out.Surfaces);
        }
    }
    else
    {
        if (in.RightRemoteTracked)
        {
            renderer_.controllerRender.Render(out.Surfaces);
        }
    }
    /// Render hand axes
    if (renderer_.handTracked)
    {
        renderer_.axisRenderer.Render(OVR::Matrix4f(), in, out);

        if (renderer_.renderJoints)
        {
            for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i)
            {
                OVRFW::GeometryRenderer &gr = renderer_.handJointRenderers[i];
                gr.Render(out.Surfaces);
            }
        }

        if (renderer_.renderCapsules)
        {
            for (int i = 0; i < XR_FB_HAND_TRACKING_CAPSULE_COUNT; ++i)
            {
                OVRFW::GeometryRenderer &gr = renderer_.handCapsuleRenderers[i];
                gr.Render(out.Surfaces);
            }
        }

        if (renderer_.renderMesh)
        {
            renderer_.handRenderer.Render(out.Surfaces);
        }
    }
}
