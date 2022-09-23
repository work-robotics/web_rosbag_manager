# web_rosbag_manager

[English](README_en.md) | [日本語](README.md)

![UI Image](./docs/images/app_ui.png "UI Image")

## Setup
```
sudo python3 -m pip install Flask websocket-server
cd ~/catkin_ws && catkin build && catkin source
```

## Usage
```
roslaunch web_rosbag_manager web_rosbag_manager.launch
```

Access `http://$IP_ADDRESS:8080/` from a web browser.
`$IP_ADDRESS` is the IP address of the PC that executed the launch file.

### Record
1. Select the topic you want to record from the "Record" panel.
2. Press the "Record" button to start recording.
3. Press the "Stop" button to stop recording.

### Check meta-information
1. Press the "Info" button for the file you want to check.
2. Detailed meta-information is displayed in the "Info" panel.

### Download
1. Press the "Download" button for the file you want to download.

### Delete
1. Press the "Delete" button for the file you want to delete.
2. A confirmation screen is displayed. Check if there are no problems, and press the "OK" button.


## License

```
Copyright 2021-2022 Work Robotics Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

See [THIRD-PARTY-NOTICES.md](./THIRD-PARTY-NOTICES.md) for other third party resources.
