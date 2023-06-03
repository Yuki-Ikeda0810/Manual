# **ROSのインストール方法**

## **目次**
1. [**Ubuntuのインストール**](#1-ubuntuのインストール)
2. [**ROSのインストール**](#2-rosのインストール)
3. [**ワークスペースの作成**](#3-ワークスペースの作成)
4. [**パッケージの作成**](#4-パッケージの作成)

<br>

## 1. Ubuntuのインストール

ROSをインストールする前に[Ubuntuのインストール](/docs/install_ubuntu.md)が必要です．
まずはそこからはじめましょう．

<br>

## 2. ROSのインストール

以下のコマンドでROSをインストールします．

``` bash
# ROSの情報をsources.listに記述
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# curlをまだインストールしていない場合
$ sudo apt install curl

# キーを設定す
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# パッケージ一覧を更新
$ sudo apt update

# ROSのインストール
$ sudo apt install ros-melodic-desktop-full

# 端末を立ち上げればROSが自動でセットされるように設定
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc

# ROSパッケージの依存関係をインストール
$ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# rosdepの初期化
$ sudo rosdep init
$ rosdep update
```

ROSをインストールできたか確認します．
以下のコマンドを実行してください．

``` bash
$ roscore
```

端末の最後に以下の表示があればインストールは成功です．

``` bash
$ started core service [/rosout] 
```

<br>

## 3. ワークスペースの作成

ROSを使ったプログラムを作るためにはワークスペースと言われる作業用のディレクトリが必要になります．
以下のコマンドで作成します．(これは初回のみ行う作業です)

``` bash
# catkin_wsという名前のディレクトリを作成
$ mkdir -p ~/catkin_ws/src

#「catkin_ws」内の「src」ディレクトリに移動
$ cd ~/catkin_ws/src

# ワークスペースの作成
$ catkin_init_workspace
```

次に，ワークスペースでビルドを行います．

``` bash
#「catkin_ws」ディレクトリに移動
$ cd ~/catkin_ws

# ワークスペースでビルド
$ catkin_make
```

次に，bashrcファイルを開きます．

``` bash
# bashrcのファイルを開く
$ gedit ~/.bashrc
```

以下のように，該当する1行を削除して1行を書き加えてください．

``` bash
source /opt/ros/melodic/setup.bash   #この行を削除
source ~/catkin_ws/devel/setup.bash  #この行を追記
```

ワークスペースを作れたか確認します．
新しい端末を開いて，以下のコマンドで確認します．

``` bash
$ echo $ROS_PACKAGE_PATH
```

以下のように表示されたら成功です．

``` bash
/home/youruser/catkin_ws/src:/opt/ros/melodic/share:/opt/ros/indigo/stacks  #youruserはユーザ名によって異なります
```

<br>

## 4. パッケージの作成 

パッケージとは，物体認識のパッケージ，自律移動のパッケージというように，それぞれの機能をまとめたディレクトリのことを言います．
以下のコマンドでパッケージを作成します．

``` bash
# 「catkin_ws」内の「src」ディレクトリに移動
$ cd ~/catkin_ws/src

# 「ros_start」パッケージを作成
$ catkin_create_pkg ros_start rospy roscpp std_msgs
```

コマンドの「ros_start」はパッケージ名で，自分で自由に名前を付けられます．
そのあとに続く「rospy」「roscpp」「std_msgs」は，「ros_start」パッケージで使いたい外部パッケージになります．

<details><summary> 外部パッケージの説明はこちらをクリック </summary>

- rospy  
    PythonでROSを使うためのパッケージ

- roscpp  
    C++でROSを使うためのパッケージ

- std_msgs  
    ROSの基本的なメッセージを使うためのパッケージ(詳しくはチュートリアルを参照)  

</details>


次に，パッケージを使えるようにビルドします．

``` bash
#「catkin_ws」ディレクトリに移動
$ cd ~/catkin_ws

# ワークスペースでビルド
$ catkin_make
```

パッケージが作成できたか確認します．
以下のコマンドを実行してください．

``` bash
#「ros_start(自分が作ったパッケージ名)」ディレクトリに移動
$ roscd ros_start
```

以下のように表示され，作ったパッケージに移動できていれば成功です．

```bash
~/catkin_ws/src/ros_start$ 
```

---

[トップに戻る](#rosのインストール方法)
