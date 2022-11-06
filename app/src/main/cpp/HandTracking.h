#pragma once
#include <openxr/openxr.h>

struct HandTracking
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
