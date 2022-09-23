#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
# SPDX-FileCopyrightText: 2021-2022 Work Robotics Co., Ltd.

from flask import Flask, render_template, request, redirect, url_for, send_from_directory
import os
app = Flask(__name__)


@app.route("/")
def hello():
    return render_template('index.html')


@app.route('/rosbags/<path:path>')
def send_js(path):
    return send_from_directory("{0}/.ros/".format(os.environ['HOME']), path)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)
