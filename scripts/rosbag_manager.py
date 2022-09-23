#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2021-2022 Work Robotics Co., Ltd.

from subprocess import PIPE
from websocket_server import WebsocketServer
import glob
import json
import os
import datetime
import math
import yaml
import rospy
from rqt_bag.recorder import Recorder
import rosbag


class ROSBagManager(object):
    def __init__(self):
        rospy.on_shutdown(self.kill_process)

        self.systemInfo = {}
        self.systemInfo["isRecording"] = False
        self.systemInfo["recordingTopics"] = []
        self.systemInfo["recordingFileName"] = ""

        self.server = WebsocketServer(14567, host="0.0.0.0")
        self.server.set_fn_new_client(self.on_connect)
        self.server.set_fn_client_left(self.on_disconnect)
        self.server.set_fn_message_received(self.on_message)
        self.server.run_forever()

    def on_connect(self, client, server):
        rospy.loginfo("Connected")
        self.sendData("systemInfo", self.systemInfo)

    def on_disconnect(self, client, server):
        rospy.loginfo("Disconnect")

    def sendData(self, header, data):
        send_data = {"header": header, "data": data}
        self.server.send_message_to_all(json.dumps(send_data))

    def on_message(self, client, server, message):
        msg = json.loads(message)

        if msg["header"] == "requestFileList":
            self.sendData("requestFileListResponse", self.get_baglist())
        if msg["header"] == "requestTopicList":
            self.sendData("requestTopicListResponse", self.get_topic_list())
        if msg["header"] == "recordStart":
            self.recordStart(msg["data"]["selectTopic"], datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".bag")
        if msg["header"] == "recordStop":
            self.recordStop()
        if msg["header"] == "deleteFile":
            self.deleteFile(msg["data"])
        if msg["header"] == "getBagInfo":
            self.getBagInfo(msg["data"]["filename"])

    def getBagInfo(self, filename):
        response = {}
        topic_info = []
        filepath = "{0}/.ros/{1}".format(os.environ['HOME'], filename)
        bag_data = rosbag.Bag(filepath, 'r', skip_index=True)
        info_dict = yaml.load(bag_data._get_yaml_info(), Loader=yaml.SafeLoader)
        if "topics" in info_dict:
            for item in info_dict["topics"]:
                info = {}
                info["topic"] = item["topic"]
                info["type"] = item["type"]
                info["messages"] = item["messages"]
                topic_info.append(info)
        if len(topic_info):
            response["start"] = {"unix": info_dict["start"], "formated": datetime.datetime.fromtimestamp(info_dict["start"]).strftime("%Y/%m/%d %H:%M:%S")}
            response["end"] = {"unix": info_dict["end"], "formated": datetime.datetime.fromtimestamp(info_dict["end"]).strftime("%Y/%m/%d %H:%M:%S")}
            response["duration"] = info_dict["duration"]
            response["messages"] = info_dict["messages"]
            response["version"] = info_dict["version"]
        else:
            response["start"] = {"unix": "undefined", "formated": "undefined"}
            response["end"] = {"unix": "undefined", "formated": "undefined"}
            response["duration"] = "undefined"
            response["messages"] = 0
            response["version"] = info_dict["version"]
        response["size"] = {"byte": info_dict["size"], "formated": self.convert_size(info_dict["size"])}
        response["topic_info"] = topic_info
        response["filename"] = filename
        self.sendData("getBagInfoResponse", response)

    def deleteFile(self, filename):
        os.remove("{0}/.ros/{1}".format(os.environ['HOME'], filename))
        self.sendData("requestFileListResponse", self.get_baglist())

    def setSystemInfo(self, isRecording, recordingTopics, recordingFileName):
        self.systemInfo["isRecording"] = isRecording
        self.systemInfo["recordingTopics"] = recordingTopics
        self.systemInfo["recordingFileName"] = recordingFileName

    def recordStart(self, topics, filename):
        if len(topics) != 0:
            self.record = Recorder("{0}/.ros/{1}".format(os.environ['HOME'], filename), topics=topics, all=False)
            self.record.start()
            rospy.loginfo("Record Start")

            self.setSystemInfo(True, topics, filename)
            self.sendData("systemInfo", self.systemInfo)
        else:
            self.setSystemInfo(False, [], "")
            self.sendData("systemInfo", self.systemInfo)

    def recordStop(self):
        self.record.stop()
        self.setSystemInfo(False, [], "")
        self.sendData("systemInfo", self.systemInfo)
        rospy.loginfo("Record Stop")

    def get_baglist(self):
        files = sorted(glob.glob("{0}/.ros/*.bag".format(os.environ['HOME'])), reverse=True)
        response = []
        for file in files:
            info = {}
            info["name"] = os.path.basename(file)
            info["size"] = self.convert_size(os.path.getsize(file))
            info["update_time"] = datetime.datetime.fromtimestamp(os.path.getmtime(file)).strftime("%Y/%m/%d %H:%M")
            info["create_time"] = datetime.datetime.fromtimestamp(os.path.getctime(file)).strftime("%Y/%m/%d %H:%M")
            response.append(info)
        return response

    def convert_size(self, size):
        sizes = ["B", "KB", "MB", "GB", "TB"]
        if size == 0:
            return "0B"
        index = int(math.floor(math.log(size, 1024)))
        convert_size = round(size / math.pow(1024, index), 2)
        return "{0} {1}".format(str(convert_size), str(sizes[index]))

    def get_topic_list(self):
        topic_list = rospy.get_published_topics('/')
        response = []
        for topic in topic_list:
            info = {}
            info["name"] = topic[0]
            info["type"] = topic[1]
            response.append(info)
        return response

    def kill_process(self):
        os.system("kill -KILL " + str(os.getpid()))


if __name__ == "__main__":
    rospy.init_node('rosbag_manager_node')
    ROSBagManager()
