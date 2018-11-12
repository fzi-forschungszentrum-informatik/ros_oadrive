#!/usr/bin/env python
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------
__author__ = "Simon Roesler <simon.roesler@student.kit.edu>"


import roslib, os

import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

import rviz



SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))

class TeleOpWindow( QWidget ):

    def __init__(self, app):
        QWidget.__init__(self)

        # self.showMaximized()
        self.setFixedSize(app.primaryScreen().size())
        # self.setFixedSize(1920, 1080)

        self.mapFrame = rviz.VisualizationFrame()
        self.mapFrame.setSplashPath("")
        self.mapFrame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, SCRIPT_PATH + "/teleop.rviz")
        self.mapFrame.load(config)

        self.setWindowTitle("AlpaKa Teleoperator")

        self.mapFrame.setMenuBar( None )
        self.mapFrame.setStatusBar( None )
        self.mapFrame.setHideButtonVisibility( False )

        
        
        layout = QHBoxLayout()
        layout.addWidget(self.mapFrame)

        
        self.setLayout( layout )


if __name__ == '__main__':
    app = QApplication( sys.argv )

    teleop = TeleOpWindow(app)
    teleop.show()

    app.exec_()
