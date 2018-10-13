# wrc2018_customer_service
## 概要
World Robot Summit 2018でカスタマーサービスを行うことになったため、そのプログラムを作成した。

(Alienware 15 Xubuntu16.04.2, Cuda9.2)
## 前提条件
### OpenCV3.4.0のインストール
- [Ubuntu16.04: OpenCV3.4.0のインストール](http://demura.net/misc/14118.html)

### ROS Kineticのインストール
- [ロボットプログラミングⅡ：ROS Kineticのインストール](http://demura.net/lecture/13657.html)

### YOLO V3のインストール
- [YOLO V3 インストールメモ](http://demura.net/athome/14694.html)

### 音声認識
- [WellCome to Speech Recog](https://github.com/OkanoShogo0903/speech_recog)

### mikataArm
- [ROS対応ロボット専用アクチュエーターDYNAMIXEL(ダイナミクセル) Xシリーズで構成するオープンソースのマニピュレーターです。 教育・研究用、ロボカップ＠ホーム エデュケーション用 ](https://github.com/ROBOTIS-JAPAN-GIT/dynamixel_mikata_arm)

```
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-dynamixel-sdk ros-kinetic-jsk-rviz-plugins python-catkin-tools libeigen3-dev
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/ROBOTIS-JAPAN-$GIT/dynamixel_mikata_arm.git
$ cd ..
$ catkin build```

エラーを履くため、/home/demulab/catkin_ws/src/dynamixel_mikata_arm/mikata_arm_toolbox/src/dxl_util.cppの
```
packetHandler.printTxRxResult(dxl_res);
packetHandler.printRxPacketError(error);
```
をコメントアウト

```
$ sudo cp src/dynamixel_mikata_arm/99-ftdi_sio.rules /etc/udev/rules.d/
```

ここで実機とつなぐ

```
$ ./src/dynamixel_mikata_arm/install.sh
```

アームの関節等の設定データの変更
/home/demulab/catkin_backup/dynamixel_mikata_arm/mikata_arm_description/urdfのmikata_arm_4.urdfを自作のMini_Arm_X3.urdfに変更

ファイルの中身の書き換えmikata_arm_4.urdfをMini_Arm_X3.urdfに、場所は以下に
```
/home/demulab/catkin_ws/src/dynamixel_mikata_arm/mikata_arm_bringup/launch/bringup.launch
/home/demulab/catkin_ws/src/dynamixel_mikata_arm/mikata_arm_description/launch/mikata_arm_display.launch
```

実行例

```
$ roslaunch mikata_arm_bringup bringup.launch gui:=true
$ roslaunch mikata_arm_bringup rviz.launch
```

### kobuki
- [kobuki install](http://wiki.ros.org/kobuki/Tutorials/Installation/kinetic)

### sound_play
```
$ cd catkin_ws/src
$ git clone https://github.com/ros-drivers/audio_common.git
```
エラーをはくためaudio_captureとaudio_playを消去
```
$ catkin_make
```

### rosでYOLOv3をうごかす
#### 鍵の作成

```
$ cd ~/.ssh
$  ssh-keygen -t rsa
```
[ここ](https://github.com/settings/ssh)に鍵をアップ
titleにid_rsa
Keyにid_rsa.pubの中身を入れる

```
$ ssh -T git@github.com
```
Hi (account名)! You've successfully authenticated, but GitHub does not provide shell access.が出てきたらOK

#### ROS YOLOv3
```
cd ~/catkin_ws/src
git clone –recursive git@github.com:leggedrobotics/darknet_ros.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## rosrun
- アームのデモンストレーション

```
$ roslaunch mikata_arm_bringup bringup.launch

#サーボのトルクをON
$ rosservice call /enable_all

$ rosrun wrc2018_customer_service arm
```

- 足回りのデモンストレーション

```
$ roslaunch kobuki_node minimal.launch --screen
$ rosrun wrc2018_customer_service underbody
```

- 音声認識のデモンストレーション

```
$ python speech_recog/scripts/speech_recog_normal.py

# セリフに応じて音を鳴らす "hello","goodbye","happy strike"
rosrun wrc2018_customer_service talk
```

-音声再生のデモンストレーション

```
$ roscore

#ノード起動
$ rosrun sound_play soundplay_node.py

#任意の音声を流す
$ rostopic pub -1 /robotsound sound_play/SoundRequest "{sound: -2, command: 1, volume: 0.5, arg: "/home/demulab/catkin_ws/src/wrc2018_customer_service/voice/hello.ogg"}"
```

- YOLOv3

```
$ roscore

#カメラ起動
$ rosparam set usb_cam/pixel_format yuyv
$ rosrun usb_cam usb_cam_node

#YOLOv3起動
$ roslaunch darknet_ros yolo_v3.launch

#境界ボックスの位置とサイズの情報をピクセル座標で与えるバウンディングボックスの配列をパブリッシュします
$ rostopic echo /darknet_ros/bounding_boxes

#データの数を知る
$ rostopic echo /darknet_ros/found_object
```

## sh
roslaunchを一括で起動する
```
$ sh catkin_ws/src/wrc2018_customer_service/sh/wrs.sh
```

## class
### odome.hpp
- Odome(ros::NodeHandle *n);
  - コンストラクタ
  - ノードハンドルを引数にいれる


- void cycle();
  - 処理層
  - while等で常に回す


- void robotPos(float x,float y,float angle);
  - ロボットを指定の座標と角度に移動させる
  - 目的座標と到着後の向きたい角度を引数に入れる


- void postureSet(float angle);
  - ロボットを指定の角度に移動させる
  - 向きたい角度を引数に入れる


- bool moveCheck();
  - ロボットが動いているか確認する
  - 動いているとtrueを返す

### arm.hpp
- Arm(ros::NodeHandle *n);
  - コンストラクタ
  - ノードハンドルを引数にいれる

- void cycle();
  - 処理層
  - while等で常に回す


- void armPos(std::vector<double> value);
  - アームのサーボを所定の角度にする
  - 入れる配列の数は5つ


- float *armPos();
  - 今のアームのサーボの角度を返す
  - 型はfloat pos[5]


- bool moveCheck();
  - アームが動いているか確認する
  - 動いているとtrueを返す

### sound.hpp
- Sound(ros::NodeHandle *n);
  - コンストラクタ
  - ノードハンドルを引数にいれる


- void cycle();
  - 処理層
  - while等で常に回す


- void playSound(int id);
  - 音声を再生する
  - 引数はコンストラクタに書いてある音声ファイルの配列の番号
  - 音声を追加したい場合はコンストラクタでsoundUrlにpush_backする
  - 再生する型はogg

### talk.hpp
- Talk(ros::NodeHandle *n);
  - コンストラクタ
  - ノードハンドルを引数にいれる


- void cycle();
  - 処理層
  - while等で常に回す


- int voiceCheck();
  - voiceSet配列に入っている文字列が聞こえた時、対応する配列番号を返す
  - 対応する文字列を追加したい場合はコンストラクタでvoiceSetにpush_backする

### faceCheck.hpp
- FaceCheck(ros::NodeHandle *n);
  - コンストラクタ
  - ノードハンドルを引数にいれる


- void cycle();
  - 処理層
  - while等で常に回す


- string checkName();
  - YOLOのクラスの中でclassList配列に書いてあるものと同じものがある場合、その文字列を返す
  - 対応する文字列を追加したい場合はコンストラクタでclassListtにpush_backする


- int checkNameNum();
  - YOLOのクラスの中でclassList配列に書いてあるものと同じものがある場合、その配列番号を返す
  - 対応する文字列を追加したい場合はコンストラクタでclassListtにpush_backする
