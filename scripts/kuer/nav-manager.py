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


import sys, zmq, argparse, json, logging, subprocess, os, time
#from zyre_pyzmq import Zyre as Pyre
from pyre import Pyre, zhelper 

from utils import find_ip_addr

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))

class RerenderThread(QThread):
    render = pyqtSignal(object)

    def __init__(self, window):
        QThread.__init__(self)
        self.window = window
        self.old_hosts = []

    def run(self):
        while True:
            if self.window.hosts != self.old_hosts:
                self.old_hosts = list(self.window.hosts)
                self.render.emit("NOW")
            time.sleep(1.0)

class NavigationManager():

    def __init__(self, name):
        self.name = name
    
        self.window = NavigationManagerWindow()
        
        def update_gui(data):
            self.window.render()

        self.rerender = RerenderThread(self.window)
            
        self.rerender.render.connect(update_gui)

        self.rerender.start()

        self.window.show()

         # communication things
        self.stopped = False
        self.ctx = zmq.Context()
        self.outgoing = zhelper.zthread_fork(self.ctx, self.network_thread)

        self.window.outgoing = self.outgoing

        print("Connecting..")
        # wait for ready from the thread
        try:
            time.sleep(1.0)
            self.outgoing.recv()
        except zmq.error.Again:
            print("ERROR! Again #0")
        print("..CONNECTED!")
    
    def stop(self):
        self.stopped = True
        self.outgoing.send("$$STOP".encode('utf_8'))
    

    def network_thread(self, ctx, pipe):
        print("Network thread started..")
        n = Pyre(self.name)
        n.join("Car2X")
        n.start()

        poller = zmq.Poller()
        poller.register(pipe, zmq.POLLIN)
        poller.register(n.inbox, zmq.POLLIN)


        # wait for others
        while not n.peer_groups():
            time.sleep(0.0001)
            pass

        pipe.send('$$ready'.encode('utf-8'))

        while not self.stopped:
            items = dict(poller.poll())
            
            if pipe in items and items[pipe] == zmq.POLLIN:
                # handle outgoing messages
                try:
                    message = pipe.recv()
                    if message.decode('utf-8') == "$$STOP":
                        break
                    n.shouts("Car2X", message.decode('utf-8'))
                except zmq.error.Again:
                    print("ERROR! Again #1")
            if n.inbox in items and items[n.inbox] == zmq.POLLIN:
                try:
                    cmds = n.recv()

                    msg_type = cmds.pop(0)
                    msg_uuid = cmds.pop(0)
                    msg_name = cmds.pop(0)
                    if msg_type == "JOIN":
                        print("{} joined!".format(msg_name))
                        # ADD CAR IF NOT CONTROL
                        if msg_name[0:1] != "$":
                            self.window.hosts.append(msg_name)
                    elif msg_type == "EXIT":
                        print("{} left!".format(msg_name))
                        # REMOVE CAR
                        if msg_name in self.window.hosts:
                            self.window.hosts.remove(msg_name)
                    elif msg_type == "SHOUT":
                        # handling incoming information
                        msg_channel = cmds.pop(0)
                        msg_str = cmds.pop(0)
                        self.process_data(msg_str, msg_name)
                except zmq.error.Again:
                    print("ERROR! Again #2")
        n.stop()

    def process_data(self, msg_str, host):
        data = json.loads(msg_str)

        if not ("payload" in data and "topic" in data):
            print("Dropping message with unknown format", msg_str, data)
            return

class NavigationManagerWindow( QWidget ):

    update_gui = pyqtSignal(object)

    def __init__(self):
        QWidget.__init__(self)
        self.hosts = []

        self.setWindowTitle("AlpaKa Navigation Starter")
        self.center()

        self.layout = QVBoxLayout()

        label = QLabel(self)
        pixmap = QPixmap(SCRIPT_PATH + '/alpacarry_logo.png').scaledToHeight(200)
        label.setPixmap(pixmap)

        self.layout.addWidget(label)

        self.btn_layout = QVBoxLayout()
        self.layout.addLayout(self.btn_layout)

        # self.render()

        self.setLayout(self.layout)

    # https://stackoverflow.com/questions/4528347/clear-all-widgets-in-a-layout-in-pyqt
    def clearLayout(self, layout):
        if layout != None:
            while layout.count():
                child = layout.takeAt(0)
                if child.widget() is not None:
                    child.widget().deleteLater()
                elif child.layout() is not None:
                    self.clearLayout(child.layout())

    def render(self):
        self.clearLayout(self.btn_layout)

        for h in ["ALL"] + self.hosts:
            host = h
            host_layout = QHBoxLayout()
            btn_get_ready = QPushButton( "GetReady " + host )

            
            def get_get_ready(host):
                def get_ready():
                    print("GetReady " + host)

                    try:
                        self.outgoing.send(json.dumps({"topic": "GET_READY_REQ", "payload": host}))
                    except zmq.error.Again:
                        rospy.logerr("ERROR EAGAIN!")
                return get_ready

            btn_get_ready.clicked.connect(get_get_ready(host))
            host_layout.addWidget(btn_get_ready)

            btn_start = QPushButton( "Start " + host )

            def get_start(host):
                def start():
                    print("Started " + host)

                    try:
                        self.outgoing.send(json.dumps({"topic": "START_REQ", "payload": host}))
                    except zmq.error.Again:
                        rospy.logerr("ERROR EAGAIN!")
                return start

            btn_start.clicked.connect(get_start(host))
            host_layout.addWidget(btn_start)

            btn_stop = QPushButton( "Stop " + host )

            def get_stop(host):
                def stop():
                    print("Stopped " + host)

                    try:
                        self.outgoing.send(json.dumps({"topic": "STOP_REQ", "payload": host}))
                    except zmq.error.Again:
                        rospy.logerr("ERROR EAGAIN!")

                return stop
                
            btn_stop.clicked.connect(get_stop(host))
            host_layout.addWidget(btn_stop)

            if host != "ALL":
                btn_viz = QPushButton( "Viz " + host )

                def get_viz(host):
                    def viz():
                        print("Start Viz " + host)
                        self.run_viz(host)
                    return viz

                btn_viz.clicked.connect(get_viz(host))
                host_layout.addWidget(btn_viz)

            self.btn_layout.addLayout(host_layout)

    def center(self):
        geo = self.frameGeometry()
        center = QDesktopWidget().availableGeometry().center()
        geo.moveCenter(center)
        self.move(geo.topLeft())
            
    def run_viz(self, host):
        print("START VIZ {}".format(host))
        # first kill old session:
        subprocess.call(["tmux", "kill-session", "-t", "tele"])
        # launch ros nodes for remote control with the roscore of the tele op car
        custom_env = dict(os.environ)
        custom_env["ROS_MASTER_URI"] = "http://" + host + ":11311"

        if "ROS_IP" not in custom_env:
            ip = find_ip_addr()
            if ip:
                custom_env["ROS_IP"] = ip
        subprocess.Popen(["python2", SCRIPT_PATH + "/teleop.py"], env=custom_env)




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', dest='name', default="$NavigationManager")
    args = parser.parse_args()

    logger = logging.getLogger("pyre")
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler())
    logger.propagate = False
    
    app = QApplication( sys.argv )

    node = NavigationManager(args.name)

    app.exec_()

    # while True:
    #     try:
    #         time.sleep(1000)
    #     except (KeyboardInterrupt, SystemExit):
    #         break

    node.stop()