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


SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))

sys.path.append(SCRIPT_PATH)

from utils import find_ip_addr

class TeleOperationRemoteNode():
    def __init__(self, name):
        self.name = name

        self.tele_op_active = False
        self.tele_op_list = []
        
        # communication things
        self.stopped = False
        self.ctx = zmq.Context()
        self.outgoing = zhelper.zthread_fork(self.ctx, self.network_thread)
    
    def stop(self):
        self.stopped = True
    
    def network_thread(self, ctx, pipe):
        print("Network thread started..")
        n = Pyre(self.name)
        n.join("Car2X")
        n.start()

        node_socket = n.socket()

        poller = zmq.Poller()
        poller.register(pipe, zmq.POLLIN)
        poller.register(node_socket, zmq.POLLIN)

        while not self.stopped:
            items = dict(poller.poll(1000))
            
            if pipe in items and items[pipe] == zmq.POLLIN:
                # handle outgoing messages
                message = pipe.recv()
                n.shouts("Car2X", message.decode('utf-8'))
            elif node_socket in items:
                cmds = n.recv()

                msg_type = cmds.pop(0)
                msg_uuid = cmds.pop(0)
                msg_name = cmds.pop(0)
                if msg_type == "JOIN":
                    print("{} joined!".format(msg_name))
                elif msg_type == "EXIT":
                    print("{} left!".format(msg_name))
                elif msg_type == "SHOUT":
                    # handling incoming information
                    msg_channel = cmds.pop(0)
                    msg_str = cmds.pop(0)
                    self.process_data(msg_str)
        n.stop()
    

    def process_data(self, msg_str):
        data = json.loads(msg_str)

        if not ("payload" in data and "topic" in data):
            print("Dropping message with unknown format", msg_str, data)
            return

        if data["topic"] == "TELE_OP_REQ":
            self.process_tele_op_req(data["payload"])
            
        if data["topic"] == "END_TELE_OP_REQ":
            self.process_end_tele_op_req(data["payload"])

    def process_tele_op_req(self, host):
        if host not in self.tele_op_list:
            self.tele_op_list.append(host)
            print("QUEUED {}".format(host))
        self.run_tele_op_list()
        

    def process_end_tele_op_req(self, host):
        if len(self.tele_op_list) > 0:
            if host == self.tele_op_list[0]:
                print("END {}".format(host))
                self.tele_op_active = False
                self.tele_op_list.pop(0)
                self.run_tele_op_list()
            else:
                print("WARNING! Wrong host tried to end tele op!")

        else: 
            print("WARNING! tele op ended without teleops")
    
    def run_tele_op_list(self):
        if not self.tele_op_active and len(self.tele_op_list) > 0:
            self.tele_op_active = True
            host = self.tele_op_list[0]
            print("START {}".format(host))
            # first kill old session:
            subprocess.call(["tmux", "kill-session", "-t", "tele"])
            # launch ros nodes for remote control with the roscore of the tele op car
            custom_env = dict(os.environ)
            if "ROS_IP" not in custom_env:
                ip = find_ip_addr()
                if ip:
                    print("Using IP: ", ip)
                    custom_env["ROS_IP"] = ip
            custom_env["ROS_MASTER_URI"] = "http://" + host + ":11311"
            subprocess.Popen(["rosrun", "catmux", "create_session", "-n", "tele", SCRIPT_PATH + "/../../catmux/kuer_remote.yml"], env=custom_env)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', dest='name', default="$TeleOperator")
    args = parser.parse_args()

    logger = logging.getLogger("pyre")
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler())
    logger.propagate = False

    node = TeleOperationRemoteNode(args.name)

    while True:
        try:
            time.sleep(1000)
        except (KeyboardInterrupt, SystemExit):
            break

    node.stop()