// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Dennis Nienhüser <nienhues@fzi.de>
* \date    15.03.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_HARDWARE_G25_WHEEL_IMPORTEXPORT_H
#define ICL_HARDWARE_G25_WHEEL_IMPORTEXPORT_H

#if defined(_SYSTEM_WIN32_) && !defined(_IC_STATIC_)
#  pragma warning(disable : 4251)

# if defined ICL_HARDWARE_G25_WHEEL_EXPORT_SYMBOLS
#  define ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT __declspec(dllexport)
# else
#  define ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT __declspec(dllimport)
# endif
#elif defined(__GNUC__) && (__GNUC__ > 3) && !defined(_IC_STATIC_)
# define ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT __attribute__ ((visibility("default")))
#else

# define ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT

#endif

#endif
