#ifndef IB2_DATA_REPLAY__VISIBILITY_CONTROL_HPP_
#define IB2_DATA_REPLAY__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IB2_DATA_REPLAY_EXPORT __attribute__((dllexport))
#define IB2_DATA_REPLAY_IMPORT __attribute__((dllimport))
#else
#define IB2_DATA_REPLAY_EXPORT __declspec(dllexport)
#define IB2_DATA_REPLAY_IMPORT __declspec(dllimport)
#endif
#ifdef IB2_DATA_REPLAY_BUILDING_LIBRARY
#define IB2_DATA_REPLAY_PUBLIC IB2_DATA_REPLAY_EXPORT
#else
#define IB2_DATA_REPLAY_PUBLIC IB2_DATA_REPLAY_IMPORT
#endif
#define IB2_DATA_REPLAY_PUBLIC_TYPE IB2_DATA_REPLAY_PUBLIC
#define IB2_DATA_REPLAY_LOCAL
#else
#define IB2_DATA_REPLAY_EXPORT __attribute__((visibility("default")))
#define IB2_DATA_REPLAY_IMPORT
#if __GNUC__ >= 4
#define IB2_DATA_REPLAY_PUBLIC __attribute__((visibility("default")))
#define IB2_DATA_REPLAY_LOCAL __attribute__((visibility("hidden")))
#else
#define IB2_DATA_REPLAY_PUBLIC
#define IB2_DATA_REPLAY_LOCAL
#endif
#define IB2_DATA_REPLAY_PUBLIC_TYPE
#endif

#endif  // IB2_DATA_REPLAY__VISIBILITY_CONTROL_HPP_
