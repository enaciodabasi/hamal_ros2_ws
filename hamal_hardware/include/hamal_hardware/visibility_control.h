#ifndef HAMAL_HARDWARE__VISIBILITY_CONTROL_H_
#define HAMAL_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define HAMAL_HARDWARE_EXPORT __attribute__((dllexport))
#define HAMAL_HARDWARE_IMPORT __attribute__((dllimport))
#else
#define HAMAL_HARDWARE_EXPORT __declspec(dllexport)
#define HAMAL_HARDWARE_IMPORT __declspec(dllimport)
#endif
#ifdef HAMAL_HARDWARE_BUILDING_DLL
#define HAMAL_HARDWARE_PUBLIC HAMAL_HARDWARE_EXPORT
#else
#define HAMAL_HARDWARE_PUBLIC HAMAL_HARDWARE_IMPORT
#endif
#define HAMAL_HARDWARE_PUBLIC_TYPE HAMAL_HARDWARE_PUBLIC
#define HAMAL_HARDWARE_LOCAL
#else
#define HAMAL_HARDWARE_EXPORT __attribute__((visibility("default")))
#define HAMAL_HARDWARE_IMPORT
#if __GNUC__ >= 4
#define HAMAL_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define HAMAL_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define HAMAL_HARDWARE_PUBLIC
#define HAMAL_HARDWARE_LOCAL
#endif
#define HAMAL_HARDWARE_PUBLIC_TYPE
#endif

#endif  // HAMAL_HARDWARE__VISIBILITY_CONTROL_H_