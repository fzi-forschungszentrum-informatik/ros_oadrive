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

#ifndef ICL_HARDWARE_G25_WHEEL_LOGGING_H
#define ICL_HARDWARE_G25_WHEEL_LOGGING_H

#include "ImportExport.h"
#include <icl_core_logging/Logging.h>

namespace icl_hardware {
namespace g25 {

DECLARE_LOG_STREAM_IMPORT_EXPORT(G25WheelLogger, ICL_HARDWARE_G25_WHEEL_IMPORT_EXPORT)

}
}

#define G25_ERROR(message) LOGGING_ERROR(icl_hardware::g25::G25WheelLogger, message << icl_core::logging::endl);
#define G25_WARNING(message) LOGGING_WARNING(icl_hardware::g25::G25WheelLogger, message << icl_core::logging::endl);
#define G25_INFO(message) LOGGING_INFO(icl_hardware::g25::G25WheelLogger, message << icl_core::logging::endl);
#define G25_DEBUG(message) LOGGING_DEBUG(icl_hardware::g25::G25WheelLogger, message << icl_core::logging::endl);
#define G25_DEBUG_NOENDL(message) LOGGING_DEBUG(icl_hardware::g25::G25WheelLogger, message);
#define G25_TRACE(message) LOGGING_TRACE(icl_hardware::g25::G25WheelLogger, message << icl_core::logging::endl);

#endif // ICL_HARDWARE_G25_WHEEL_LOGGING_H
