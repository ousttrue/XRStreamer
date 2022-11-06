#pragma once
#include <openxr/openxr.h>

class XrHands
{
    /// Hands - extension functions
    PFN_xrCreateHandTrackerEXT xrCreateHandTrackerEXT_ = nullptr;
    PFN_xrDestroyHandTrackerEXT xrDestroyHandTrackerEXT_ = nullptr;
    PFN_xrLocateHandJointsEXT xrLocateHandJointsEXT_ = nullptr;

    XrHands(XrInstance &Instance);

public:
    ~XrHands();
    /// Hands - tracker handles
    XrHandTrackerEXT handTrackerL_ = XR_NULL_HANDLE;
    XrHandTrackerEXT handTrackerR_ = XR_NULL_HANDLE;

    XrHandTrackingScaleFB scaleL;
    XrHandTrackingCapsulesStateFB capsuleStateL;
    XrHandTrackingAimStateFB aimStateL;
    XrHandTrackingScaleFB scaleR;
    XrHandTrackingCapsulesStateFB capsuleStateR;
    XrHandTrackingAimStateFB aimStateR;
    XrHandJointLocationsEXT locationsL;
    XrHandJointLocationsEXT locationsR;

    /// Hands - data buffers
    XrHandJointLocationEXT jointLocationsL_[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointLocationEXT jointLocationsR_[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointVelocityEXT jointVelocitiesL_[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointVelocityEXT jointVelocitiesR_[XR_HAND_JOINT_COUNT_EXT];

    static XrHands *Create(XrInstance &Instance, XrSystemId &systemId);
    void OnSessionInit(XrInstance &Instance, XrSession &Session);
    void Shutdown(XrInstance &Instance);
    void Update(XrInstance &Instance, XrSpace &space, XrTime time);
};
