#ifndef ros2_roamadome__VISIBILITY_CONTROL_H_
#define ros2_roamadome__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ros2_roamadome_EXPORT __attribute__ ((dllexport))
    #define ros2_roamadome_IMPORT __attribute__ ((dllimport))
  #else
    #define ros2_roamadome_EXPORT __declspec(dllexport)
    #define ros2_roamadome_IMPORT __declspec(dllimport)
  #endif
  #ifdef ros2_roamadome_BUILDING_LIBRARY
    #define ros2_roamadome_PUBLIC ros2_roamadome_EXPORT
  #else
    #define ros2_roamadome_PUBLIC ros2_roamadome_IMPORT
  #endif
  #define ros2_roamadome_PUBLIC_TYPE ros2_roamadome_PUBLIC
  #define ros2_roamadome_LOCAL
#else
  #define ros2_roamadome_EXPORT __attribute__ ((visibility("default")))
  #define ros2_roamadome_IMPORT
  #if __GNUC__ >= 4
    #define ros2_roamadome_PUBLIC __attribute__ ((visibility("default")))
    #define ros2_roamadome_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ros2_roamadome_PUBLIC
    #define ros2_roamadome_LOCAL
  #endif
  #define ros2_roamadome_PUBLIC_TYPE
#endif

#endif  // ros2_roamadome__VISIBILITY_CONTROL_H_
