// Copyright 2026 CDEI-UPC (Centre de Disseny d'Equips Industrials,
// Universitat Politecnica de Catalunya)
// SPDX-License-Identifier: GPL-3.0-only

// Symbol visibility macros for the shared plugin library (standard ROS 2
// boilerplate, needed so the plugin class is exported on every platform).

#ifndef IRD_ETHERCAT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define IRD_ETHERCAT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define IRD_ETHERCAT_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define IRD_ETHERCAT_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define IRD_ETHERCAT_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define IRD_ETHERCAT_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef IRD_ETHERCAT_HARDWARE_INTERFACE_BUILDING_DLL
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC IRD_ETHERCAT_HARDWARE_INTERFACE_EXPORT
#else
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC IRD_ETHERCAT_HARDWARE_INTERFACE_IMPORT
#endif
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC_TYPE IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
#define IRD_ETHERCAT_HARDWARE_INTERFACE_LOCAL
#else
#define IRD_ETHERCAT_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define IRD_ETHERCAT_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define IRD_ETHERCAT_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC
#define IRD_ETHERCAT_HARDWARE_INTERFACE_LOCAL
#endif
#define IRD_ETHERCAT_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // IRD_ETHERCAT_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
