#pragma once
#include <openxr/openxr.h>

struct HandTracker
{
    XrHandTrackerCreateInfoEXT createInfo;
    XrHandTrackerEXT handTracker = XR_NULL_HANDLE;
    XrHandTrackingScaleFB scale;
    XrHandTrackingCapsulesStateFB capsuleState;
    XrHandTrackingAimStateFB aimState;
    XrHandJointLocationsEXT locations;
    XrHandJointLocationEXT jointLocations[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointVelocityEXT jointVelocities[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointVelocitiesEXT velocities;
    XrHandJointsLocateInfoEXT locateInfo;

    XrHandTrackerCreateInfoEXT *CreateInfo(XrHandEXT hand)
    {
        createInfo = {
            XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
        createInfo.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
        createInfo.hand = hand;
        return &createInfo;
    }

    void Update(XrSpace &space, XrTime time);
};

class XrHands
{
    /// Hands - extension functions
    PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT_ = nullptr;
    PFN_xrDestroyHandTrackerEXT xrDestroyHandTrackerEXT_ = nullptr;
    PFN_xrLocateHandJointsEXT xrLocateHandJointsEXT_ = nullptr;

    XrHands(XrInstance &Instance);

public:
    ~XrHands();
    HandTracker left_;
    HandTracker right_;

    static XrHands *Create(XrInstance &Instance, XrSystemId &systemId);
    void OnSessionInit(XrInstance &Instance, XrSession &Session);
    void Shutdown(XrInstance &Instance);
    void Update(XrInstance &Instance, XrSpace &space, XrTime time);
};
