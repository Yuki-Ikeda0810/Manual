# **ROSコマンドの説明**

本ページではROSコマンドについて説明します．
ROSコマンドは端末から実行することで，ROS通信の実行や，通信情報の取得が出来ます．
本ページで説明する際のプログラムは，[ROS Template Programs](https://github.com/TeamSOBITS/ros_template_programs)を使用します．

:arrow_forward: [ROS用語の説明](/docs/about_ros_term.md)

:arrow_forward: [ROSの使用方法](/docs/using_ros.md)

## **目次**

1. [実行コマンド](#1-実行コマンド)
    1. [roscore](#1-roscore)
    2. [rosrun](#2-rosrun)
    3. [roslaunch](#3-roslaunch)
2. [通信情報コマンド](#2-通信情報コマンド)
    1. [rosnode](#1-rosnode)
    2. [rostopic](#2-rostopic)
    3. [rosmsg](#3-rosmsg)
    4. [rosservice](#4-rosservice)
    5. [rossrv](#5-rossrv)
    6. [rosparam](#6-rosparam)
    7. [rosbag](#7-rosbag)
3. [その他コマンド](#3-その他コマンド)
    1. [roscd](#1-roscd)
    2. [catkin_make](#2-catkin_make)
    3. [catkin_create_pkg](#3-catkin_create_pkg)
    4. [chmod](#4-chmod)


## 1. 実行コマンド
### 1. roscore
ROSで通信するときに使用するコマンドです．
roscoreはROS Master，ROS Parameter Server，rosoutログ用のノードが立ち上がります．
```bash
$ roscore
```

### 2. rosrun
プログラムを実行するときに使用するコマンドです．
このコマンドを実行する際は，予めroscoreを起動しておく必要があります．
```bash
# rosrun <パッケージ名> <プログラム名>
$ rosrun ros_template_programs topic_publisher_template.py
```

### 3. roslaunch
roscoreと複数のrosrunを同時に実行するコマンドです．
```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_template_programs topic_template_py.launch
```

## 2. 通信情報コマンド
### 1. rosnode
**ノードに関するコマンドです．**
- 立ち上がっているノード名一覧を表示する
```bash
$ rosnode list
```

- ノードの情報を表示する
```bash
# rosnode info /<ノード名>
$ rosnode info /topic_templete/publisher
```

- ノードのpingを表示する
```bash
# rosnode ping /<ノード名>
$ rosnode ping /topic_templete/publisher
```

- ノードをキル(強制終了)する
```bash
# rosnode kill /<ノード名>
$ rosnode kill /topic_templete/publisher
```
引数とするノード名は複数選択することもできる．

### 2. rostopic
**トピックに関するコマンドです．**
- 動いているトピック名一覧を表示する
```bash
$ rostopic list
```

- トピックの情報を表示する
```bash
# rostopic info /<トピック名>
$ rostopic info /topic_templete/pub_msg
```

- トピックのメッセージ種類を表示する
```bash
# rostopic type /<トピック名>
$ rostopic type /topic_templete/pub_msg
```

- トピックから発信されている値を表示する
```bash
# rostopic echo /<トピック名>
$ rostopic echo /topic_templete/pub_msg
```

<details><summary>※各オプションの説明はこちらをクリック</summary>
オプションの使用例

```bash
# rostopic echo <オプションコマンド> /<トピック名>
$ rostopic echo --noarr /topic_templete/pub_msg
```

- ***--noarr***  
    配列の非表示

- ***--nostr***  
    文字列の非表示

- ***-n &lt;num&gt;***  
    表示回数の設定
    &lt;num&gt;には表示したい回数を入れること
</details>

- トピックが発信されている周波数を確認する
```bash
# rostopic hz /<トピック名>
$ rostopic hz /topic_templete/pub_msg
```

- トピックをパブリッシュする
```bash
# rostopic pub /<トピック名> <メッセージ名> <値>
$ rostopic pub /topic_templete/pub_msg std_msgs/String "Hello_world!"
```

<details><summary>※各オプションの説明はこちらをクリック</summary>
オプションの使用例

```bash
# rostopic pub <オプションコマンド> /<トピック名>
$ rostopic pub -r 10 /topic_templete/pub_msg
```

- ***-l***  
    ラッチモード
    ラッチモードは一度パブリッシュした値を保持するモードです．
    そのため，ラッチモードでパブリッシュ後にサブスクライブしても，その値を受け取ることが出来ます．
    終了するためにはctrl + Cを押します．
    rostopic pubのデフォルトモードはラッチモードとなっています．

- ***-r &lt;num&gt;***  
    レートモード
    設定したレートでパブリッシュするモードです．
    &lt;num&gt;にはパブリッシュしたい周波数を入れること

- ***-1***  
    ワンスモード
    一回だけパブリッシュするモードです．

</details>

### 3. rosmsg
**メッセージに関するコマンドです．**
- メッセージ一覧を表示する
```bash
$ rosmsg list
```

- メッセージの型を表示する
```bash
# rosmsg show <メッセージ名>
$ rosmsg show std_msgs/String
```

### 4. rosservice
**サービスに関するコマンドです．**
- 立ち上がっているサービス一覧を表示する
```bash
$ rosservice list
```

- トピックの情報を表示する
```bash
# rosservice info /<サービス名>
$ rosservice info /service_templete
```

- サービスを要求する
```bash
# rosservice call /<サービス名>
$ rosservice call /service_templete
```

### 5. rossrv
**srvに関するコマンドです．**
- srv一覧を表示する
```bash
$ rossrv list
```

- srvの型を表示する
```bash
# rossrv show <メッセージ名>
$ rossrv show std_srvs/SetBool
```

### 6. rosparam
**パラメータに関するコマンドです．**
- パラメータ一覧を表示する
```bash
$ rosparam list
```

- パラメータの値を表示する
```bash
# rosparam get <パラメータ名>
$ rosparam get /topic_template/publisher/message
```

- パラメータの値一覧を表示する
```bash
$ rosparam dump
```

- パラメータの値を設定する
```bash
# rosparam set <パラメータ名> <値>
$ rosparam set /topic_template/publisher/message "Hello, World!"
```

### 7. rosbag
**現在の状況を記録したり、再現したりするコマンドです．**
動作しているトピックを記録することが出来ます．
記録したものはバグファイルと呼ばれ，バグファイル名を指定しない場合は，その時の日付と時刻がファイル名になります．
記録するトピックは1つ1つ指定することが出来ますが，引数-aを取ればすべてのトピックを記録することが出来ます．

- 動作しているトピックをバグファイルに記録する
```bash
# rosbag record <トピック名> <トピック名> ... <トピック名>
$ rosbag record /topic_template/pub_msg
```
<details><summary>※各オプションの説明はこちらをクリック</summary>
オプションの使用例(「sample.bag」という名前のファイルに動いてるすべてのトピックを記録する)

```bash
# rosbag record <オプションコマンド> (/<トピック名>)
$ rosbag record -O sample.bag -a
```

- ***-a***  
    動いてるすべてのトピックを記録する

- ***-O &lt;BAG_FILE_NAME&gt;***
    記録するバグファイル名を設定する
    &lt;BAG_FILE_NAME&gt;には記録したいバグファイル名を入れること

</details>


## 3. その他コマンド
### 1. roscd
**階層を指定したパッケージまで移動するコマンドです．**
```bash
# roscd <パッケージ名>
$ roscd ros_template_programs
```

### 2. catkin_make
**catkin_ws内をビルドするコマンドです．**
C，C++言語のプログラムや各種設定ファイルを書き換えた場合は必ずビルドしましょう．
また，ビルドは必ずcatkin_ws内の階層で行いましょう．
```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

上記のコマンドを一行で行う場合
```bash
$ ~/catkin_ws && catkin_make
```


### 3. catkin_create_pkg
**catkin_ws内に新しいリポジトリを作成するコマンドです．**
リポジトリの作成は必ずcatkin_ws/src内の階層で行いましょう．
また，リポジトリで使用するライブラリがある場合は，リポジトリ名の後に記述します．
```bash
$ cd ~/catkin_ws/src        # catkin_ws/src内の階層へ移動

# catkin_create_pkg <リポジトリ名> [使用するライブラリ] [使用するライブラリ] ... [使用するライブラリ]
$ catkin_create_pkg ros_start std_msgs rospy roscpp
```
<details><summary>※使用するライブラリ例の説明はこちらをクリック</summary>
※ここに含まれないライブラリもあるのでご注意ください

- ***std_msgs***  
    ROSの標準メッセージが含まれたライブラリ

- ***rospy***
    PythonでROSを使用するためのライブラリ

- ***roscpp***
    C++でROSを使用するためのライブラリ

- ***sensor_msgs***  
    センサ関係のメッセージが含まれたライブラリ

- ***geometry_msgs***
    速度や座標，クォータニオン関係のメッセージが含まれたライブラリ

</details>

### 4. chmod
**ファイルに権限を付与するコマンドです．**
chmodはROSコマンドではないですが，PythonのプログラムをROSで扱うのに必要なコマンドです．
Pythonで新たにプログラムを作成した場合は，必ず実行しましょう．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 topic_publisher_template.py
```

モードの数字は付与する権限の度合いを示しています．
以下の表にある数値の合計値を「所有者」「所有グループ」「その他」の順で入力することで権限を付与することが出来ます．  
そのため，上記の「755」は  
「所有者」に対して「読み取り」「書き込み」「実行」の権限を  
「所有グループ」に対して「読み取り」「実行」の権限を  
「その他」に対して「読み取り」「実行」の権限を付与しています．

<table>
    <tr>
        <th>モード(数字)</th>
        <th>権限</th>
    </tr>
    <tr>
        <td>4</td>
        <td>読み取り</td>
    </tr>
    <tr>
        <td>2</td>
        <td>書き込み</td>
    </tr>
    <tr>
        <td>1</td>
        <td>実行</td>
    </tr>
</table>

---

[トップに戻る](#rosコマンドの説明)
