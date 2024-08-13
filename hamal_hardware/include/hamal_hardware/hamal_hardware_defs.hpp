#ifndef HAMAL_HARDWARE_DEFS_HPP_
#define HAMAL_HARDWARE_DEFS_HPP_


#include <unordered_map>
#include <string>

enum ControlType
{
    Position,
    Velocity
};

enum class HomingStatus
{
    Unknown,
    HomingIsPerformed,
    HomingIsInterruptedOrNotStarted,
    HomingConfirmedTargetNotReached,
    HomingCompleted,
    ErrorDetectedMotorStillRunning,
    ErrorDuringHomingMotorAtStandstill
};

const std::unordered_map<HomingStatus, std::string> HomingStatusStrings = {
    {HomingStatus::Unknown, "Unknown"},
    {HomingStatus::HomingIsPerformed, "Homing is performed"},
    {HomingStatus::HomingIsInterruptedOrNotStarted, "Homing is interrupted or not yet started"},
    {HomingStatus::HomingConfirmedTargetNotReached, "Homing confirmed but target not yet reached"},
    {HomingStatus::HomingCompleted, "Homing completed"},
    {HomingStatus::ErrorDetectedMotorStillRunning, "Error detected, motor still running"},
    {HomingStatus::ErrorDuringHomingMotorAtStandstill, "Error during homing, motor at standstill"}
};

#endif // HAMAL_HARDWARE_DEFS_HPP