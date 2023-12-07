#ifndef POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_
#define POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POINTCLOUD_TO_LASERSCAN_EXPORT __attribute__ ((dllexport))
    #define POINTCLOUD_TO_LASERSCAN_IMPORT __attribute__ ((dllimport))
  #else
    #define POINTCLOUD_TO_LASERSCAN_EXPORT __declspec(dllexport)
    #define POINTCLOUD_TO_LASERSCAN_IMPORT __declspec(dllimport)
  #endif
  #ifdef POINTCLOUD_TO_LASERSCAN_BUILDING_DLL
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC POINTCLOUD_TO_LASERSCAN_EXPORT
  #else
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC POINTCLOUD_TO_LASERSCAN_IMPORT
  #endif
  #define POINTCLOUD_TO_LASERSCAN_PUBLIC_TYPE POINTCLOUD_TO_LASERSCAN_PUBLIC
  #define POINTCLOUD_TO_LASERSCAN_LOCAL
#else
  #define POINTCLOUD_TO_LASERSCAN_EXPORT __attribute__ ((visibility("default")))
  #define POINTCLOUD_TO_LASERSCAN_IMPORT
  #if __GNUC__ >= 4
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC __attribute__ ((visibility("default")))
    #define POINTCLOUD_TO_LASERSCAN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POINTCLOUD_TO_LASERSCAN_PUBLIC
    #define POINTCLOUD_TO_LASERSCAN_LOCAL
  #endif
  #define POINTCLOUD_TO_LASERSCAN_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // POINTCLOUD_TO_LASERSCAN__VISIBILITY_CONTROL_H_