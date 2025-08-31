# GNSS-DualRTK-Heading

[English Verion](./README.md)  

本套件利用**雙 RTK-GPS 天線**計算航向角，解決自動駕駛車輛開機時無法自動判斷車頭朝向。  
可搭配 [GNSS-Localizer](../GNSS-Localizer) 建立精細平面座標系

---

## 功能特色
- 發佈 `/heading`、`gnss_pose` 等結果，供 GNSS-Localizer、NDT-Matching等套件使用。  
- 可搭配 `GNSS-Localizer`，形成「精細平面座標系 + 航向角」的完整定位系統。

---

## RQT 關係圖
![](./images/rqt_graph.jpg)

---

## 開發環境
- Ubuntu 18.04  
- ROS (Melodic)  
- C++17  
- CMake 3.10+  
- ROS packages: `roscpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf`  
- Runtime dependency: `nmea_navsat_driver`  

---

## 安裝 / 使用範例
```bash
cd ~/catkin_ws/src
git clone https://github.com/LeviChen1126/GNSS-DualRTK-Heading.git
cd ..
catkin_make
source devel/setup.bash
```

---

## 執行
### 1. 啟動 GPS driver
```bash
roslaunch gnss_dualrtk_heading dual_gps_serial.launch port1:=/dev/ttyACM0 port2:=/dev/ttyACM1 baud:=19200
```

或使用 rosbag / 測試：
```bash
roslaunch gnss_dualrtk_heading dual_gps_topic_test.launch
```

### 2. 啟動 Heading Node
```bash
roslaunch gnss_dualrtk_heading dualrtk_heading.launch
``` 

---

## Launch Files
- **dual_gps_serial.launch**：同時啟動兩顆序列埠 RTK-GPS → `/fix`、`/fix_2`  
- **dual_gps_topic_test.launch**：測試模式，不開 serial，只跑 topic driver  
- **dualrtk_heading.launch**：啟動 heading node，載入 `gnss_dualrtk.yaml`  

---

## 專案結構
```
GNSS-DualRTK-Heading/
 ├── launch/
 │    ├── dualrtk_heading.launch
 │    ├── dual_gps_serial.launch
 │    └── dual_gps_topic_test.launch
 ├── config/
 │    └── gnss_dualrtk.yaml
 ├── src/
 │    └── dualrtk_heading_node.cpp
 ├── images/
 │    └── rqt_graph.jpg
 ├── .gitattributes
 ├── .gitignore
 ├── CMakeLists.txt
 ├── package.xml
 ├── README.md
 └── LICENSE
```

---

## 授權與致謝
- [MIT License](./LICENSE)  
- [Autoware](https://www.autoware.org/)