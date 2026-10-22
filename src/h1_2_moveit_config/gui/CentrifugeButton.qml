import QtQuick 2.12
import QtQuick.Controls 2.5

Rectangle {
  width: 380
  height: 82
  color: "#eeffffff"
  radius: 6
  Label {
    anchors.centerIn: parent
    text: "蓝色：开 / 关盖    绿色：启动 / 停止\n" +
          CentrifugeButton.machineStatus +
          "\n转子未停稳时，请勿操作盖子"
    horizontalAlignment: Text.AlignHCenter
    color: "#30343b"
  }
}
