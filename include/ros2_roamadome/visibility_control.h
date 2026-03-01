#ifndef roamadome_control__VISIBILITY_CONTROL_H_
#define roamadome_control__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define roamadome_control_EXPORT __attribute__ ((dllexport))
    #define roamadome_control_IMPORT __attribute__ ((dllimport))
  #else
    #define roamadome_control_EXPORT __declspec(dllexport)
    #define roamadome_control_IMPORT __declspec(dllimport)
  #endif
  #ifdef roamadome_control_BUILDING_LIBRARY
    #define roamadome_control_PUBLIC roamadome_control_EXPORT
  #else
    #define roamadome_control_PUBLIC roamadome_control_IMPORT
  #endif
  #define roamadome_control_PUBLIC_TYPE roamadome_control_PUBLIC
  #define roamadome_control_LOCAL
#else
  #define roamadome_control_EXPORT __attribute__ ((visibility("default")))
  #define roamadome_control_IMPORT
  #if __GNUC__ >= 4
    #define roamadome_control_PUBLIC __attribute__ ((visibility("default")))
    #define roamadome_control_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define roamadome_control_PUBLIC
    #define roamadome_control_LOCAL
  #endif
  #define roamadome_control_PUBLIC_TYPE
#endif

#endif  // roamadome_control__VISIBILITY_CONTROL_H_
