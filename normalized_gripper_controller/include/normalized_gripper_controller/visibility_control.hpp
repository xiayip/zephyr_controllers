#pragma once

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NORMALIZED_GRIPPER_CONTROLLER_EXPORT __attribute__((dllexport))
#define NORMALIZED_GRIPPER_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define NORMALIZED_GRIPPER_CONTROLLER_EXPORT __declspec(dllexport)
#define NORMALIZED_GRIPPER_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef NORMALIZED_GRIPPER_CONTROLLER_BUILDING_DLL
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC                                   \
  NORMALIZED_GRIPPER_CONTROLLER_EXPORT
#else
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC                                   \
  NORMALIZED_GRIPPER_CONTROLLER_IMPORT
#endif
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC_TYPE                              \
  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
#define NORMALIZED_GRIPPER_CONTROLLER_LOCAL
#else
#define NORMALIZED_GRIPPER_CONTROLLER_EXPORT                                   \
  __attribute__((visibility("default")))
#define NORMALIZED_GRIPPER_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC                                   \
  __attribute__((visibility("default")))
#define NORMALIZED_GRIPPER_CONTROLLER_LOCAL                                    \
  __attribute__((visibility("hidden")))
#else
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
#define NORMALIZED_GRIPPER_CONTROLLER_LOCAL
#endif
#define NORMALIZED_GRIPPER_CONTROLLER_PUBLIC_TYPE
#endif
