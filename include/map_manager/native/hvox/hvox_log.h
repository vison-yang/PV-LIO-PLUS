/* PV-LIO-PLUS adapter for the logging-only spdlog calls in Surfel-LIO. */
#pragma once

#include <ros/console.h>
#include <sstream>

namespace hvox_log
{
template <typename... Args>
std::string Message(const char *format, const Args &...args)
{
    std::ostringstream out;
    out << format;
    ((out << ' ' << args), ...);
    return out.str();
}
template <typename... Args> void warn(const char *format, const Args &...args)
{
    ROS_WARN_STREAM(Message(format, args...));
}
template <typename... Args> void error(const char *format, const Args &...args)
{
    ROS_ERROR_STREAM(Message(format, args...));
}
template <typename... Args> void info(const char *format, const Args &...args)
{
    ROS_INFO_STREAM(Message(format, args...));
}
}  // namespace hvox_log
