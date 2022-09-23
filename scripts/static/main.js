class ROSBagWebManager {
  constructor() {
    this.systemInfo = {};
    this.filenameList = [];
    this.topicList = [];
    this.selectedTopicList = [];

    this.ws = new WebSocket("ws://" + location.hostname + ":14567");

    this.ws.onmessage = (message) => {
      message = JSON.parse(message.data);
      switch (message.header) {
        case "systemInfo":
          this.updateSystemInfo(message.data);
          break;
        case "requestFileListResponse":
          this.updateROSBagList(message.data);
          break;
        case "requestTopicListResponse":
          this.updateTopicList(message.data);
          break;
        case "getBagInfoResponse":
          this.updateBagInfo(message.data);
          break;
      }
    };

    this.ws.onopen = () => {
      document.getElementById("status").innerHTML = "Status: Connected";
      this.requestFileList();
      this.requestTopicList();
    };

    this.ws.onclose = () => {
      document.getElementById("status").innerHTML = "Status: Not connected";
    };

    // DOMの取得
    this.recordButtonElement = document.getElementById("record-button");
    this.stopButtonElement = document.getElementById("stop-button");
    this.databaseElement = document.getElementById("database");
    this.infoElement = document.getElementById("info");
    this.topicListElement = document.getElementById("topiclist");
    this.recordButtonElement = document.getElementById("record-button");
    this.stopButtonElement = document.getElementById("stop-button");
  }

  sendData(header, data) {
    this.ws.send(JSON.stringify({ header: header, data: data }));
  }

  requestFileList() {
    this.sendData("requestFileList", "");
  }

  requestTopicList() {
    this.sendData("requestTopicList", "");
  }

  getROSBagInfo(filename) {
    this.sendData("getBagInfo", { filename: filename });
  }

  removeFileRequest(filename) {
    this.sendData("deleteFile", filename);
  }

  recordStartRequest(topics) {
    this.sendData("recordStart", { selectTopic: topics });
  }

  recordStopRequest(topics) {
    this.sendData("recordStop", "");
  }

  updateSystemInfo(data) {
    this.systemInfo = data;
    if (this.systemInfo.isRecording == false) {
      this.recordButtonElement.disabled = false;
      this.stopButtonElement.disabled = true;
    }
    if (this.systemInfo.isRecording == true) {
      this.recordButtonElement.disabled = true;
      this.stopButtonElement.disabled = false;
    }
    this.requestFileList();
    this.requestTopicList();
  }

  updateROSBagList(data) {
    this.filenameList = [];
    this.databaseElement.innerHTML = "";

    const tbody = document.createElement("tbody");

    const names = ["Filename", "Date", "Size", "", "", ""];
    const thead = document.createElement("thead");
    let tr = document.createElement("tr");
    for (let i = 0; i < names.length; i++) {
      const th = document.createElement("th");
      th.innerHTML = names[i];
      if (i == 3) th.style = "width: 50px";
      if (i == 4) th.style = "width: 100px";
      if (i == 5) th.style = "width: 80px";
      tr.appendChild(th);
    }
    thead.className = "table-light";
    thead.appendChild(tr);

    for (let i = 0; i < data.length; i++) {
      tr = document.createElement("tr");
      let td = document.createElement("td");
      td.className = "align-middle";
      td.innerHTML = data[i].name;
      tr.appendChild(td);

      if (this.systemInfo.recordingFileName == data[i].name) {
        tr.className = "table-danger";
        td = document.createElement("td");
        td.innerHTML = "Recording...";
        tr.appendChild(td);
        td = document.createElement("td");
        td.innerHTML = "---";
        tr.appendChild(td);
        td = document.createElement("td");
        td.innerHTML = "---";
        tr.appendChild(td);
        td = document.createElement("td");
        td.innerHTML = "---";
        tr.appendChild(td);
        td = document.createElement("td");
        td.innerHTML = "---";
        tr.appendChild(td);
      } else {
        td = document.createElement("td");
        td.className = "align-middle";
        td.innerHTML = data[i].create_time;
        tr.appendChild(td);

        td = document.createElement("td");
        td.className = "align-middle";
        td.innerHTML = data[i].size;
        tr.appendChild(td);

        td = document.createElement("td");
        let infobotton = document.createElement("button");
        infobotton.className = "btn btn-outline-secondary btn-sm";
        infobotton.innerHTML = "Info";
        infobotton.onclick = () => {
          this.getROSBagInfo(data[i].name);
          this.infoElement.innerHTML = "<h3>Please wait...<h3>";
        };
        td.appendChild(infobotton);
        tr.appendChild(td);

        td = document.createElement("td");
        let downloadbotton = document.createElement("button");
        downloadbotton.className = "btn btn-outline-primary btn-sm";
        downloadbotton.innerHTML = "Download";
        downloadbotton.onclick = () => {
          location.href = "rosbags/" + data[i].name;
        };
        td.appendChild(downloadbotton);
        tr.appendChild(td);

        td = document.createElement("td");
        let deletebotton = document.createElement("button");
        deletebotton.className = "btn btn-outline-danger btn-sm";
        deletebotton.id = "delete-button-" + i.toString();
        deletebotton.innerHTML = "Delete";
        deletebotton.onclick = (event) => {
          let id = event.target.id.replace("delete-button-", "");
          const response = window.confirm(this.filenameList[id].name + "を削除しますか？");
          if (response) {
            this.removeFileRequest(this.filenameList[id].name);
          }
        };
        td.appendChild(deletebotton);
        tr.appendChild(td);
      }

      tbody.appendChild(tr);
      this.databaseElement.appendChild(thead);
      this.databaseElement.appendChild(tbody);
      this.filenameList.push({ name: data[i].name, create_time: data[i].create_time });

      if (this.systemInfo.recordingFileName == data[i].name) {
        const databaseBodyElement = document.getElementById("database-body");
        databaseBodyElement.scrollTop = databaseBodyElement.scrollHeight + 200;
      }
    }
  }

  updateBagInfo(data) {
    this.infoElement.innerHTML = "";
    const topicHeader = document.createElement("h5");
    topicHeader.innerText = data.filename;
    this.infoElement.appendChild(topicHeader);
    let tableElement = document.createElement("table");
    let thead = document.createElement("thead");
    let tr = document.createElement("tr");
    let th = document.createElement("th");
    th.innerHTML = "Topic";
    tr.appendChild(th);

    th = document.createElement("th");
    th.innerHTML = "MessageType";
    tr.appendChild(th);

    th = document.createElement("th");
    th.innerHTML = "Message Count";
    tr.appendChild(th);

    thead.appendChild(tr);

    let tbody = document.createElement("tbody");

    tableElement.className = "table table-bordered table-sm table-primary";
    for (let i = 0; i < data.topic_info.length; i++) {
      tr = document.createElement("tr");
      let td = document.createElement("td");
      td.innerHTML = data.topic_info[i].topic;
      tr.appendChild(td);

      td = document.createElement("td");
      td.innerHTML = data.topic_info[i].type;
      tr.appendChild(td);

      td = document.createElement("td");
      td.innerHTML = data.topic_info[i].messages;
      tr.appendChild(td);

      tbody.appendChild(tr);
    }

    tableElement.appendChild(thead);
    tableElement.appendChild(tbody);
    this.infoElement.appendChild(tableElement);

    // ------------------------------------------------

    tableElement = document.createElement("table");
    thead = document.createElement("thead");
    tr = document.createElement("tr");
    th = document.createElement("th");
    th.innerHTML = "Name";
    tr.appendChild(th);
    th = document.createElement("th");
    th.innerHTML = "Value";
    tr.appendChild(th);
    thead.appendChild(tr);
    tbody = document.createElement("tbody");
    tableElement.className = "table table-bordered table-sm table-success";

    tr = document.createElement("tr");
    let td = document.createElement("td");
    td.innerHTML = "StartTime";
    tr.appendChild(td);
    td = document.createElement("td");
    td.innerHTML = data.start.formated;
    tr.appendChild(td);
    tbody.appendChild(tr);

    tr = document.createElement("tr");
    td = document.createElement("td");
    td.innerHTML = "EndTime";
    tr.appendChild(td);
    td = document.createElement("td");
    td.innerHTML = data.end.formated;
    tr.appendChild(td);
    tbody.appendChild(tr);

    tr = document.createElement("tr");
    td = document.createElement("td");
    td.innerHTML = "Messages";
    tr.appendChild(td);
    td = document.createElement("td");
    td.innerHTML = data.messages;
    tr.appendChild(td);
    tbody.appendChild(tr);

    tr = document.createElement("tr");
    td = document.createElement("td");
    td.innerHTML = "Size";
    tr.appendChild(td);
    td = document.createElement("td");
    td.innerHTML = data.size.formated;
    tr.appendChild(td);
    tbody.appendChild(tr);

    tableElement.appendChild(thead);
    tableElement.appendChild(tbody);
    this.infoElement.appendChild(tableElement);
  }

  updateTopicList(data) {
    this.topicList = [];
    this.topicListElement.innerHTML = "";

    const thead = document.createElement("thead");
    let tr = document.createElement("tr");

    const tbody = document.createElement("tbody");

    for (let i = 0; i < data.length; i++) {
      const tr = document.createElement("tr");
      tr.onclick = (event) => {
        event.currentTarget.querySelectorAll("td:first-of-type input").forEach((check, index) => {
          if (event.target.tagName != "INPUT") {
            check.checked = !check.checked;
          }
        });
      };

      let td = document.createElement("td");
      td.className = "align-middle";
      const check = document.createElement("input");
      check.setAttribute("type", "checkbox");
      check.setAttribute("name", "name");
      check.id = data[i].name;

      if (this.systemInfo.isRecording) {
        check.disabled = true;
      }

      if (this.systemInfo.recordingTopics.indexOf(check.id) > -1) {
        check.checked = true;
      }

      if (this.selectedTopicList.indexOf(check.id) > -1) {
        check.checked = true;
      }

      td.appendChild(check);
      tr.appendChild(td);

      td = document.createElement("td");
      td.innerHTML = data[i].name;
      tr.appendChild(td);

      td = document.createElement("td");
      td.innerHTML = data[i].type;
      tr.appendChild(td);

      tbody.appendChild(tr);

      this.topicList.push({ name: data[i].name, type: data[i].type });
    }

    let th = document.createElement("th");
    th.className = "align-middle";
    th.style = "width: 25px";
    const check = document.createElement("input");
    check.setAttribute("type", "checkbox");
    check.setAttribute("name", "name");
    check.id = "allcheck-input";
    check.onclick = (event) => {
      if (event.target.checked) {
        tbody.querySelectorAll("td:first-of-type input").forEach((check, index) => {
          check.checked = true;
        });
      } else {
        tbody.querySelectorAll("td:first-of-type input").forEach((check, index) => {
          check.checked = false;
        });
      }
    };

    let isAllChecked = true;
    tbody.querySelectorAll("td:first-of-type input").forEach((check, index) => {
      if (check.checked == false) {
        isAllChecked = false;
      }
    });
    if (isAllChecked) {
      check.checked = true;
    }

    th.appendChild(check);
    tr.appendChild(th);
    th = document.createElement("th");
    th.innerHTML = "Topic";
    tr.appendChild(th);
    th = document.createElement("th");
    th.innerHTML = "MessageType";
    tr.appendChild(th);
    thead.appendChild(tr);

    this.topicListElement.appendChild(thead);
    this.topicListElement.appendChild(tbody);
  }

  recordStartButton() {
    this.recordButtonElement.disabled = true;
    const recordTopic = [];
    this.topicListElement.querySelectorAll("td:first-of-type input").forEach((check, index) => {
      if (check.checked) {
        recordTopic.push(this.topicList[index].name);
      }
    });
    this.selectedTopicList = recordTopic;

    if (recordTopic.length == 0) {
      alert("トピックを選択してからボタンを押してください。");
      this.recordButtonElement.disabled = false;
    } else {
      this.recordStartRequest(recordTopic);
    }
  }

  recordStopButton() {
    this.stopButtonElement.disabled = true;
    this.recordStopRequest();
  }
}

rosBagWebManager = new ROSBagWebManager();
