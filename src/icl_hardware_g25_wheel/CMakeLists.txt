# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------

ICMAKER_SET("icl_hardware_g25_wheel" IDE_FOLDER ${ICL_HARDWARE_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  Logging.cpp
  G25RacingWheel.cpp
)

ICMAKER_ADD_HEADERS(
  ImportExport.h
  Logging.h
  G25RacingWheel.h
)

ICMAKER_LOCAL_CPPDEFINES(-DICL_HARDWARE_G25_WHEEL_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_HARDWARE_G25_WHEEL_)
ICMAKER_INCLUDE_DIRECTORIES(${ICL_HARDWARE_INCLUDE_DIRS})

ICMAKER_INTERNAL_DEPENDENCIES(EXPORT
  icl_core_config
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(${icmaker_target})
