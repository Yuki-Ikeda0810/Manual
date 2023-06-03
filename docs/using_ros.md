# **ROSの使用方法**

## **目次**
0. [パッケージの作成](#0-パッケージの作成)
1. [Topic通信](#1-topic通信)
    1. [Publisher](#1-publisher)
    2. [Subscriber](#2-subscriber)
    3. [Launchで同時起動](#3-launchで同時起動)
2. [Service通信](#2-service通信)
    1. [Server](#1-server)
    2. [Client](#2-client)
    3. [Launchで同時起動](#3-launchで同時起動)
3. [Action通信](#3-action通信)
    1. [Server](#1-server)
    2. [Client](#2-client)
    3. [Launchで同時起動](#3-launchで同時起動)

<br>

## 0. パッケージの作成
まずはじめに「ros_start」という名前のパッケージをcatkin_ws/src内で作成します．
今回はPythonとC++でプログラムを作成し，ROSの標準メッセージを扱うので，「std_msgs」，「rospy」，「roscpp」のライブラリをパッケージの作成時に選択します．

```bash
$ cd ~/catkin_ws/src        # catkin_ws/src内の階層へ移動
$ catkin_create_pkg ros_start std_msgs rospy roscpp
```

catkin_ws/src内に以下の階層のパッケージが作成されていればOKです．

<pre>
ros_strat
├── CMakeLists.txt
├── include
├── package.xml
└── src
</pre>

<br>

## 1. Topic通信
Topic通信のプログラムを作成します．
本節では，std_msgs/String型のメッセージを送受信するTopic通信を行います．
一般的にPythonのプログラムは「scripts」というディレクトリに，CやC++のプログラムは「src」というディレクトリに保存します．

Pythonのプログラムを保存するために「scripts」というディレクトリを作成します．
ディレクトリの作成はmkdirというコマンドを使用します．

```bash
$ cd ~/catkin_ws/src/ros_start      # catkin_ws/src/ros_start内の階層へ移動

# mkdir <ディレクトリ名>
$ mkdir scripts                     # 「scripts」という名前のディレクトリを作成する
``` 

### 1. Publisher
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「topic_publisher.py」という名前のファイルを作成します．
ファイルの作成はtouchというコマンドを使用します．

```bash
$ cd scripts                        # scripts内の階層へ移動

# touch <ファイル名>
$ touch topic_publisher.py          # 「topic_publisher.py」という名前のファイルを作成する
``` 

「topic_publisher.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_publisher.py」を開いて下さい．

※1行目のコメントアウトはプログラムに書き込まないで下さい．
```python
#! /usr/bin/env python              # シバン(ファイルに書かれたプログラム文をどうやって実行するか)の定義
#coding:utf-8                       # 文字コードの宣言(Magic comment)　これを書くと日本語のコメントアウトができる
                                    # Python 2. 必須
                                    # Python 3. 不要(むしろ非推奨)

# ライブラリのインポート
import rospy                        # PythonでROSを使用するためのライブラリをインポート
from std_msgs.msg import String     # ROSの標準メッセージが含まれたライブラリから文字列を扱うメッセージ「String」をインポート

# main関数
def main():
    # ノードの初期宣言
    rospy.init_node('topic_publisher')                              # 「topic_publisher」という名前のノードを宣言

    # パブリッシャの定義
    pub_msg = rospy.Publisher('pub_msg', String, queue_size=1)      # 第1引数 'pub_msg'     ：トピック名
                                                                    # 第2引数 String        ：パブリッシュするメッセージの型
                                                                    # 第3引数 queue_size=1  :キューサイズ(バッファのサイズ)
                                                                    #                        古い値をどれだけ見逃さずに取って
                                                                    #                        おくかの設定
                                                                    #                        例えば，queue_size=1は常に送信さ
                                                                    #                        れていない古い値を最新の値で更新
                                                                    #                        します                                                                    
                                                                    
    # メッセージの定義と代入
    msg = String()                                                  # msgという変数をString型で扱う様に定義
    msg.data = 'Hello, World!'                                      # msgのdataという要素に「Hello, World!」の文字列を代入

    # ループ周波数を定義
    loop_rate = rospy.Rate(1)                                       # ループ周波数を1Hzに定義

    # パブリッシュのループ処理
    while not rospy.is_shutdown():                                                          # rospyがシャットダウンされるまで繰り返す
        pub_msg.publish(msg)                                                                # msgをパブリッシュする
        rospy.loginfo('\n [ ' + rospy.get_name()  + ' ] Message Published\n%s\n', msg )     # 端末に出力
                                                                                            # rospy.get_name()：ノード名を取得
        
        loop_rate.sleep()                                                                   # 設定した周波数でループするようにsleep(一時停止)する
    return

# プログラムの起動時にのみ実行される
if __name__ == '__main__':
    main()                      # main関数を呼び出す
```

プログラムが書けたらプログラムファイルに権限を付与します．
権限の付与はchmodというコマンドを使用します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 topic_publisher.py
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

これで実行する準備が完了しました．
早速実行していきます．
ROSのプログラムを実行するにはROS Masterが必要です．
ROS Masterの起動にはroscoreというコマンドを使用します．

```bash
$ roscore
```

続いて，topic_publisher.pyを実行します．
ROSのプログラムの実行にはrosrunというコマンドを使用します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_publisher.py
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1649940215.846345284]:
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1649940216.846493880]: 
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1649940217.846514961]: 
[/topic_publisher] Message Published
Hello, World!
```

次に，トピックがちゃんとパブリッシュされているか確認します．
トピックからパブリッシュされた値の確認にはrostopic echoコマンド使用します．
別の端末を開き，以下を実行します．

```bash
# rostopic echo /<トピック名>
$ rostopic echo /pub_msg
```

端末上に以下のように表示されたら成功です．

```bash
data: "Hello, World!"
---
data: "Hello, World!"
---
data: "Hello, World!"
---
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「topic_publisher.cpp」という名前のファイルを作成します．
ファイルの作成はtouchというコマンドを使用します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch topic_publisher.cpp          # 「topic_publisher.cpp」という名前のファイルを作成する
``` 

「topic_publisher.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_publisher.cpp」を開いて下さい．

```cpp
// ライブラリのインポート
#include <ros/ros.h>                                        // C++でROSを使用するためのライブラリをインポート
#include <std_msgs/String.h>                                // ROSの標準メッセージが含まれたライブラリから文字列を扱うメッセージ「String」をインポート

// main関数
int main(int argc, char *argv[])  {                         // argc：引数の個数，*argv[]：引数の配列のポインタ
    // ノードの初期宣言
    ros::init(argc, argv, "topic_publisher");               // 「topic_publisher」という名前のノードを宣言

    // インスタンス(クラスや構造体の実体)の宣言や変数の定義
    ros::NodeHandle nh;                                     // ノードハンドル(ノードに関する操作をするもの)を宣言
    ros::Publisher pub_msg;                                 // パブリッシャを宣言
    std_msgs::String msg;                                   // String型変数の宣言

    // パブリッシャの定義
    pub_msg = nh.advertise<std_msgs::String>("pub_msg", 1); // std_msgs::String　   ：パブリッシュするメッセージの型
                                                            // 第1引数 "pub_msg"    ：トピック名
                                                            // 第2引数 1            ：キューサイズ(バッファのサイズ)
                                                            //                        古い値をどれだけ見逃さずに取って
                                                            //                        おくかの設定
                                                            //                        例えば，キューサイズ=1は常に送信
                                                            //                        されていない古い値を最新の値で更
                                                            //                        新します          

    // メッセージへの代入
    msg.data = "Hello, World!";                             // msgのdataという要素に「Hello, World!」の文字列を代入

    // ループ周波数を定義
    ros::Rate loop_rate(1);                                 // ループ周波数を1Hzに定義

    // パブリッシュのループ処理
	while(ros::ok()) {                                        // rosが正常な間繰り返す
        pub_msg.publish(msg);                               // msgをパブリッシュする
        ROS_INFO("\n[%s] Message Published\n%s\n", ros::this_node::getName().c_str(), msg.data.c_str() );       // 端末に出力
                                                                                                                // ros::this_node::getName() ：ノード名を取得
		loop_rate.sleep();                                  // 設定した周波数でループするようにsleep(一時停止)する
	}

    return 0;
}
```

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 136行目以下に追記 ---
add_executable(topic_publisher src/topic_publisher.cpp)             # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(topic_publisher ${catkin_LIBRARIES})          # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
```

CMakeLists.txtを編集したらメイクをします．
catkin_wsのメイクはcatkin_ws内の階層で行う必要があります．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
ROSのプログラムを実行するにはROS Masterが必要です．
ROS Masterの起動にはroscoreというコマンドを使用します．

```bash
$ roscore
```

続いて，topic_publisherを実行します．
ROSのプログラムの実行にはrosrunというコマンドを使用します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_publisher
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1649940215.846345284]:
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1649940216.846493880]: 
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1649940217.846514961]: 
[/topic_publisher] Message Published
Hello, World!
```

次に，トピックがちゃんとパブリッシュされているか確認します．
トピックからパブリッシュされた値の確認にはrostopic echoコマンド使用します．
別の端末を開き，以下を実行します．

```bash
# rostopic echo /<トピック名>
$ rostopic echo /pub_msg
```

端末上に以下のように表示されたら成功です．

```bash
data: "Hello, World!"
---
data: "Hello, World!"
---
data: "Hello, World!"
---
```

</details>


### 2. Subscriber
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「topic_subscriber.py」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/scripts                         # catkin_ws/src/ros_start/scripts内の階層へ移動

# touch <ファイル名>
$ touch topic_subscriber.py          # 「topic_subscriber.py」という名前のファイルを作成する
``` 

「topic_subscriber.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_subscriber.py」を開いて下さい．

※1行目のコメントアウトはプログラムに書き込まないで下さい．
```python
#! /usr/bin/env python              # シバン(ファイルに書かれたプログラム文をどうやって実行するか)の定義
#coding:utf-8                       # 文字コードの宣言(Magic comment)　これを書くと日本語のコメントアウトができる
                                    # Python 2. 必須
                                    # Python 3. 不要(むしろ非推奨)

# ライブラリのインポート
import rospy                        # PythonでROSを使用するためのライブラリをインポート
from std_msgs.msg import String     # ROSの標準メッセージが含まれたライブラリから文字列を扱うメッセージ「String」をインポート

# callback関数
def callback_message(msg):          # 第1引数 msg  ：受け取ったメッセージ
    rospy.loginfo('\n [ ' + rospy.get_name() + ' ] Message Subscribed\n%s\n', msg )     # 端末に出力
    return


# main関数
def main():
    try:
        # ノードの初期宣言
        rospy.init_node('topic_subscriber')                              # 「topic_subscriber」という名前のノードを宣言

        # サブスクライバの定義
        sub_msg = rospy.Subscriber('pub_msg', String, callback_message) # 第1引数 'pub_msg'         ：トピック名
                                                                        # 第2引数 String            ：サブスクライブするメッセージの型
                                                                        # 第3引数 callback_message  ：メッセージをサブスクライブした時
                                                                        #                             に行う処理の関数
                                                                        
        rospy.spin()                                                    # callbackの処理を行うためにスピン(無限ループ)させる
    
    except rospy.ROSInterruptException: pass                            # rosが例外的に終了された時(Ctrl+C　など)に正しく終了させる

# プログラムの起動時にのみ実行される
if __name__ == '__main__':
    main()                      # main関数を呼び出す
```

プログラムが書けたらプログラムファイルに権限を付与します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 topic_subscriber.py
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，topic_publisher.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_publisher.py
```

続いて，topic_subscriber.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_subscriber.py
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650004257.484570]: 
 [ /topic_subscriber ] Message Subscribed
data: "Hello, World!"

[INFO] [1650004258.483511]: 
 [ /topic_subscriber ] Message Subscribed
data: "Hello, World!"

[INFO] [1650004259.484356]: 
 [ /topic_subscriber ] Message Subscribed
data: "Hello, World!"
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「topic_subscriber.cpp」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch topic_subscriber.cpp          # 「topic_subscriber.cpp」という名前のファイルを作成する
``` 

「topic_subscriber.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_subscriber.cpp」を開いて下さい．

```cpp
// ライブラリのインポート
#include <ros/ros.h>                                        // C++でROSを使用するためのライブラリをインポート
#include <std_msgs/String.h>                                // ROSの標準メッセージが含まれたライブラリから文字列を扱うメッセージ「String」をインポート

// callback関数
void callbackMessage( const std_msgs::String msg) {         // 第1引数 msg  ：受け取ったメッセージ
    ROS_INFO ( "\n[%s] Message Subscribed\n%s\n", ros::this_node::getName().c_str(), msg.data.c_str() );        // 端末上に出力
    return;
}

// main関数
int main(int argc, char *argv[])  {                         // argc：引数の個数，*argv[]：引数の配列のポインタ
    // ノードの初期宣言
    ros::init(argc, argv, "topic_subscriber");              // 「topic_subscriber」という名前のノードを宣言

    // インスタンス(クラスや構造体の実体)や変数の宣言
    ros::NodeHandle nh;                                     // ノードハンドル(ノードに関する操作をするもの)を宣言
    ros::Subscriber sub_msg;                                // サブスクライバを宣言
  
    //サブスクライバの定義
    sub_msg = nh.subscribe( "pub_msg", 1, callbackMessage); // 第1引数 'pub_msg'         ：トピック名
                                                            // 第2引数 1                 ：キューサイズ(バッファのサイズ)
                                                            //                             古い値をどれだけ見逃さずに取って
                                                            //                             おくかの設定
                                                            //                             例えば，キューサイズ=1は常に送信
                                                            //                             されていない古い値を最新の値で更
                                                            //                             新します    
                                                            // 第3引数 callbackMessage   ：メッセージをサブスクライブした時
                                                            //                             に行う処理の関数

    ros::spin();                                            // callbackの処理を行うためにスピン(無限ループ)させる

    return 0;
}
```

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 138行目以下に追記 ---
add_executable(topic_subscriber src/topic_subscriber.cpp)             # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(topic_subscriber ${catkin_LIBRARIES})           # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
```

CMakeLists.txtを編集したらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，topic_publisherを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_publisher
```

続いて，topic_subscriberを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start topic_subscriber
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1650007058.460541822]: 
[/topic_subscriber] Message Subscribed
Hello, World!

[ INFO] [1650007059.460465242]: 
[/topic_subscriber] Message Subscribed
Hello, World!

[ INFO] [1650007060.460684455]: 
[/topic_subscriber] Message Subscribed
Hello, World!
```

</details>


### 3. Launchで同時起動
launchを使えば，roscoreと複数のノードを同時に起動することが出来ます．
launchファイルを保存するために「launch」という名前のディレクトリを作成します．
```bash
$ cd ~/catkin_ws/src/ros_start      # catkin_ws/src/ros_start内の階層へ移動

# mkdir <ディレクトリ名>
$ mkdir launch                     # 「launch」という名前のディレクトリを作成する
``` 

<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「topic_py.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch  # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch topic_py.launch                # 「topic_py.launch」という名前のファイルを作成する
``` 

「topic_py.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_py.launch」を開いて下さい．
launchファイルはxml形式で記述します．

```xml
<launch>
    <!-- publisher -->
    <!-- <node pkg=<パッケージ名> type=<実行ファイル名> name=<ノード名> output=<出力形式> /> -->
    <node pkg="ros_start" type="topic_publisher.py" name="topic_publisher" output="screen" />
    <!-- subscriber -->
    <node pkg="ros_start" type="topic_subscriber.py" name="topic_subscriber"  output="screen" />
</launch>
```

output=<出力形式>は標準出力の設定項目であり"screen"か"log"を選択できます．
"screen"の場合は端末上に，"log"の場合は，$ROS_HOME/ log内のログファイルに送信されます．

これで実行する準備が完了しました．
早速実行していきます．
launchの起動にはroslaunchというコマンドを使用します．

```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start topic_py.launch
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650010233.526313]: 
 [ /publisher ] Message Published
data: "Hello, World!"

[INFO] [1650010233.528023]: 
 [ /subscriber ] Message Subscribed
data: "Hello, World!"

[INFO] [1650010234.526270]: 
 [ /publisher ] Message Published
data: "Hello, World!"

[INFO] [1650010234.527461]: 
 [ /subscriber ] Message Subscribed
data: "Hello, World!"
```

</details>

<details><summary>※cppでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「topic_cpp.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch   # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch topic_cpp.launch                # 「topic_cpp.launch」という名前のファイルを作成する
``` 

「topic_cpp.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「topic_cpp.launch」を開いて下さい．

```xml
<launch>
    <!-- publisher -->
    <!-- <node pkg=<パッケージ名> type=<実行ファイル名> name=<ノード名> output=<出力形式> /> -->
    <node pkg="ros_start" type="topic_publisher" name="topic_publisher" output="screen" />
    <!-- subscriber -->
    <node pkg="ros_start" type="topic_subscriber" name="topic_subscriber"  output="screen" />
</launch>
```

output=<出力形式>は標準出力の設定項目であり"screen"か"log"を選択できます．
"screen"の場合は端末上に，"log"の場合は，$ROS_HOME/ log内のログファイルに送信されます．

これで実行する準備が完了しました．
早速実行していきます．
launchの起動にはroslaunchというコマンドを使用します．

```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start topic_py.launch
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1650011379.473640830]: 
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1650011379.473971327]: 
[/topic_subscriber] Message Subscribed
Hello, World!

[ INFO] [1650011380.473645259]: 
[/topic_publisher] Message Published
Hello, World!

[ INFO] [1650011380.473935846]: 
[/topic_subscriber] Message Subscribed
Hello, World!
```

</details>

<br>

## 2. Service通信
Service通信のプログラムを作成します．
本節では，計算式を送り，計算結果を返すService通信を行います．
ここでは，std_msgsのようなROS標準のメッセージではなく，新しく独自のメッセージとsrvファイルを作成し，使用します．
メッセージファイルを保存するために「msg」という名前のディレクトリを作成します．
```bash
$ cd ~/catkin_ws/src/ros_start      # catkin_ws/src/ros_start内の階層へ移動

# mkdir <ディレクトリ名>
$ mkdir msg                         # 「msg」という名前のディレクトリを作成する
``` 

「msg」ディレクトリの中に「Expression.msg」という名前のファイルを作成します．

```bash
$ cd msg                            # msg内の階層へ移動

# touch <ファイル名>
$ touch Expression.msg              # 「Expression.msg」という名前のファイルを作成する
``` 

「Expression.msg」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「Expression.msg」を開いて下さい．

```bash
# <型または変数> <変数名>
float64 a
float64 b
string calculate
```

srvファイルを保存するために「srv」という名前のディレクトリを作成します．
```bash
$ cd ~/catkin_ws/src/ros_start      # catkin_ws/src/ros_start内の階層へ移動

# mkdir <ディレクトリ名>
$ mkdir srv                         # 「srv」という名前のディレクトリを作成する
``` 

「srv」ディレクトリの中に「Calculation.srv」という名前のファイルを作成します．

```bash
$ cd srv                            # srv内の階層へ移動

# touch <ファイル名>
$ touch Calculation.srv             # 「Calculation.srv」という名前のファイルを作成する
``` 

「Calculation.srv」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「Calculation.srv」を開いて下さい．

```bash
# <型または変数> <変数名>           # サービスの要求となる引数
# ---
# <型または変数> <変数名>           # サービスの応答となる返り値

ros_start/Expression expression
---
float64 result
```

これらを実行ファイルで扱えるようにメイク設定ファイルのCMakeLists.txtと，パッケージに関する情報ファイルのpackage.xmlというファイルを編集します．

「CMakeLists.txt」ファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)で「CMakeLists.txt」を開いて下さい．
```cmake
# --- 10行目付近 ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation            # 追記　メッセージの自作用パッケージ
)
```
```cmake
# --- 50行目付近 ---
## Generate messages in the 'msg' folder
add_message_files(              # コメントアウト(#)をはずす        メッセージを新規作成する際に必要な設定
  FILES                         # コメントアウト(#)をはずす
  Expression.msg                # 追記　「Expression.msg」をメッセージファイルとして追加
#   Message1.msg                # 削除
#   Message2.msg                # 削除
)                               # コメントアウト(#)をはずす

## Generate services in the 'srv' folder
add_service_files(              # コメントアウト(#)をはずす        srvを新規作成する際に必要な設定
  FILES                         # コメントアウト(#)をはずす
  Calculation.srv               # 追記　「Calculation.srv」をsrvファイルとして追加
#   Service1.srv                # 削除
#   Service2.srv                # 削除
)                               # コメントアウト(#)をはずす
```

```cmake
# --- 73行目付近 ---
## Generate added messages and services with any dependencies listed here
generate_messages(             # コメントアウト(#)をはずす        メッセージやsrvを新規作成する際に必要な設定
  DEPENDENCIES                 # コメントアウト(#)をはずす
  std_msgs                     # コメントアウト(#)をはずす
)                              # コメントアウト(#)をはずす
```

「package.xml」ファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)で「package.xml」を開いて下さい．

```xml
  <!-- --- 40行目以降 --- -->
    <build_depend>message_generation</build_depend>         <!-- コメントアウト(<!-- -- >) 外す        メッセージを新規作成する際に必要な設定-->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
    <exec_depend>message_runtime</exec_depend>         <!-- コメントアウト(<!-- -- >) 外す        メッセージを新規作成する際に必要な設定-->
```

これらの編集ができたらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

### 1. Server
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「service_server.py」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/scripts                         # catkin_ws/src/ros_start/scripts内の階層へ移動

# touch <ファイル名>
$ touch service_server.py           # 「service_server.py」という名前のファイルを作成する
``` 

「service_server.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_server.py」を開いて下さい．

```python
#! /usr/bin/env python
#coding:utf-8

import rospy
from ros_start.msg import Expression                            # ros_start/msg/からExpressionをインポート
from ros_start.srv import Calculation, CalculationResponse      # ros_start/srv/からCalculation, CalculationResponseをインポート
                                                                # Calculation           ：クライアントからの要求を受け取る際に使用
                                                                # CalculationResponse   ：クライアントへの応答を送る際に使用

# calculate関数
def calculate(req):                         # クライアントから要求されたときの処理(引数で与えられた数値の計算を応答として返す)
    res = CalculationResponse()             # resという変数をCalculationResponse型で扱う様に定義
    ex = req.expression                     # 要求の引数をex変数に代入
    if ex.calculate == '+':                 # 演算記号が+のとき
        res.result = ex.a + ex.b            # 加算処理
    elif ex.calculate == '-':               # 演算記号が-のとき
        res.result = ex.a - ex.b            # 減算処理
    elif ex.calculate == '*':               # 演算記号が*のとき
        res.result = ex.a * ex.b            # 乗算処理
    else:                                   # 演算記号が/のとき
        res.result = ex.a / ex.b            # 除算処理
    rospy.loginfo('\n [ ' + rospy.get_name() + ' ] Received a call from a client\nRequest: %.2f %s %.2f = %.2f\n', ex.a, ex.calculate, ex.b, res.result)    # 端末に出力
    return res

# main関数
def main():
    try:
        # ノードの初期宣言
        rospy.init_node('service_server')                                           # 「service_server」という名前のノードを宣言
        # サービスの定義
        service = rospy.Service('calculate_two_numbers', Calculation, calculate)    # 第1引数 'calculate_two_numbers'       ：サービス名
                                                                                    # 第2引数 Calculation                   ：クライアントから受け取るsrvの型
                                                                                    # 第3引数 calculate                     ：クライアントから要求を受け取ったときに行う処理の関数
        
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ] Ready to calculate_two_numbers.\n')

        rospy.spin()                                                                # calculateの処理を行うためにスピン(無限ループ)させる

    except rospy.ROSInterruptException: pass                                        # rosが例外的に終了された時(Ctrl+C　など)に正しく終了させる
    
    return

if __name__ == '__main__':
    main()
```

プログラムが書けたらプログラムファイルに権限を付与します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 service_server.py
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，service_server.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_server.py
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650270744.724073]: 
 [ /service_server ] Ready to calculate_two_numbers.
```

次に，サービスが上手く動いているか確認します．
サーバーへ要求を送るにはrosservice callコマンド使用します．
別の端末を開き，以下を実行します．

```bash
# rosservice call <サービス名> <要求の引数となる値>
$ rosservice call /calculate_two_numbers "expression:
  a: 1.0
  b: 2.0
  calculate: '+'"
```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[INFO] [1650270787.655993]: 
 [ /service_server ] Received a call from a client
Request: 1.00 + 2.00 = 3.00

# client側の端末
result: 3.0
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「service_server.cpp」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch service_server.cpp          # 「service_server.cpp」という名前のファイルを作成する
``` 

「service_server.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_server.cpp」を開いて下さい．

```cpp
// ライブラリのインポート
#include <ros/ros.h>
#include <ros_start/Expression.h>                   // ros_start/msg/からExpressionをインポート
#include <ros_start/Calculation.h>                  // ros_start/srv/からCalculationをインポート

// calculate関数
bool calculate(ros_start::Calculation::Request &req, ros_start::Calculation::Response &res) {           // クライアントから要求されたときの処理(引数で与えられた数値の計算を応答として返す)
                                                                                                        // 第1引数      req         ：要求で与えられた引数
                                                                                                        // 第2引数      res         ：応答で返す値
    ros_start::Expression ex = req.expression;      // 要求の引数をex変数に代入
    if ( ex.calculate == "+" ) {                    // 演算記号が+のとき
        res.result = ex.a + ex.b;                   // 加算処理
    } else if ( ex.calculate == "-" ) {             // 演算記号が-のとき
        res.result = ex.a - ex.b;                   // 減算処理
    } else if ( ex.calculate == "*" ) {             // 演算記号が*のとき
        res.result = ex.a * ex.b;                   // 乗算処理
    } else {                                        // 演算記号が/のとき
        res.result = ex.a / ex.b;                   // 除算処理
    }
    ROS_INFO("[%s] Received a call from a client\nRequest: %.2f %s %.2f", 
        ros::this_node::getName().c_str(), ex.a, ex.calculate.c_str(), ex.b);                           // 端末に出力
    ROS_INFO("sending back response: [%.2f]\n", res.result);                                             // 端末に出力
}

// main関数
int main(int argc, char *argv[]) {
    // ノードの初期宣言
    ros::init(argc, argv, "service_server");                            // 「service_server」という名前のノードを宣言

    // インスタンス(クラスや構造体の実体)の宣言や変数の定義
    ros::NodeHandle nh;                                                 // ノードハンドル(ノードに関する操作をするもの)を宣言
    ros::ServiceServer service;                                         // サーバを宣言

    // サーバの定義
    service = nh.advertiseService("calculate_two_numbers", calculate);  // 第1引数 'calculate_two_numbers'       ：サービス名
                                                                        // 第2引数 calculate                     ：クライアントから要求を受け取ったときに行う処理の関数

    ros::spin();                                                        // calculateの処理を行うためにスピン(無限ループ)させる

    return 0;
}
```

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 146行目以下に追記 ---
add_executable(service_server src/service_server.cpp)                   # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(service_server ${catkin_LIBRARIES})               # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
add_dependencies(service_server ${PROJECT_NAME}_generate_messages_cpp)  # 独自型のメッセージの依存関係を追加する
```

CMakeLists.txtを編集したらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，service_serverを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_server
```

次に，サービスが上手く動いているか確認します．
サーバーへ要求を送るにはrosservice callコマンド使用します．
別の端末を開き，以下を実行します．

```bash
# rosservice call <サービス名> <要求の引数となる値>
$ rosservice call /calculate_two_numbers "expression:
  a: 1.0
  b: 2.0
  calculate: '+'"
```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[ INFO] [1650278795.001783028]: [/service_server] Received a call from a client
Request: 1.00 + 2.00
[ INFO] [1650278795.002412093]: sending back response: [3.00]

# client側の端末
result: 3.0
```

</details>


### 2. Client
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「service_client.py」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/scripts                         # catkin_ws/src/ros_start/scripts内の階層へ移動

# touch <ファイル名>
$ touch service_client.py           # 「service_client.py」という名前のファイルを作成する
``` 

「service_client.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_client.py」を開いて下さい．

```python
#! /usr/bin/env python
#coding:utf-8

import rospy
from ros_start.msg import Expression        # ros_start/msg/からExpressionをインポート
from ros_start.srv import Calculation       # ros_start/srv/からCalculation, CalculationResponseをインポート
                                            

# main関数
def main():
    try:
        # ノードの初期宣言
        rospy.init_node('service_client')                                           # 「service_client」という名前のノードを宣言

        # サービスの起動確認
        rospy.wait_for_service('calculate_two_numbers')                             # 「calculate_two_numbers」のサーバが立ち上がるまで待つ
        rospy.loginfo('['+ rospy.get_name() + '] Connect to the service server')

        # クライアントの定義
        client = rospy.ServiceProxy('calculate_two_numbers', Calculation)           # 第1引数 'calculate_two_numbers'       ：サービス名
                                                                                    # 第2引数 Calculation                   ：サーバへ送る要求の引数となるsrvの型

        # 変数の定義
        loop_rate = rospy.Rate(1)                                                   # ループ周波数を1Hzに定義
        count = 0                                                                   # カウンタの初期宣言
        ex = Expression()                                                           # exという変数をExpression型で扱う様に定義
        ex.a = rospy.get_param( rospy.get_name() + '/a', 1.0)                       # 変数aにrosparamで設定された値を代入(初期値：1.0)
        ex.b = rospy.get_param( rospy.get_name() + '/b', 1.0)                       # 変数bにrosparamで設定された値を代入(初期値：1.0)

        # クライアントのループ処理
        while not rospy.is_shutdown():                                              # rospyがシャットダウンされるまで繰り返す
            if count%4 == 0:
                ex.calculate = '+'
            elif count%4 == 1:
                ex.calculate = '-'
            elif count%4 == 2:
                ex.calculate = '*'
            else:
                ex.calculate = '/'
            try:
                res = client(ex)                                                    # クライアントからサービスを要求する(res：応答後の返り値)
                rospy.loginfo('\n [ ' + rospy.get_name()  + ' ] Result: %.2f\n', res.result)

            except rospy.ServiceException as e:                                     # サービスとの通信に失敗した場合
                rospy.logerr("Failed to call service calculate_two_numbers : %s"%e)
            count += 1                                                              # カウントアップ
            loop_rate.sleep()                                                       # 設定した周波数でループするようにsleep(一時停止)する
        return

    except rospy.ROSInterruptException: pass                                        # rosが例外的に終了された時(Ctrl+C　など)に正しく終了させる
    
    return

if __name__ == '__main__':
    main()
```

プログラムが書けたらプログラムファイルに権限を付与します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 service_client.py
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，service_server.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_server.py
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650270744.724073]: 
 [ /service_server ] Ready to calculate_two_numbers.
```

次に，計算処理で扱う変数a，bの値をrosparamを使って設定をします．
パラメータの値設定にはrosparam setコマンドを使用します．

```bash
# rosparam set <パラメータ名> <値>
$ rosparam set /service_client/a 3.0        # 変数aの値を3.0に設定
$ rosparam set /service_client/b 2.0        # 変数bの値を2.0に設定
```

次に，service_client.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_client.py
```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[INFO] [1650282034.164869]: 
 [ /service_server ] Received a call from a client
Request: 3.00 + 2.00 = 5.00

[INFO] [1650282035.160500]: 
 [ /service_server ] Received a call from a client
Request: 3.00 - 2.00 = 1.00

[INFO] [1650282036.160668]: 
 [ /service_server ] Received a call from a client
Request: 3.00 * 2.00 = 6.00

[INFO] [1650282037.160680]: 
 [ /service_server ] Received a call from a client
Request: 3.00 / 2.00 = 1.50
```

```bash
# client側の端末
[INFO] [1650282034.165809]: 
 [ /service_client ] Result: 5.00

[INFO] [1650282035.164584]: 
 [ /service_client ] Result: 1.00

[INFO] [1650282036.164341]: 
 [ /service_client ] Result: 6.00

[INFO] [1650282037.164359]: 
 [ /service_client ] Result: 1.50
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「service_client.cpp」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch service_client.cpp          # 「service_client.cpp」という名前のファイルを作成する
``` 

「service_client.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_client.cpp」を開いて下さい．

```cpp
#include <ros/ros.h>
#include <ros_start/Expression.h>
#include <ros_start/Calculation.h>

// main関数
int main(int argc, char *argv[]) {
    // ノードの初期宣言
    ros::init(argc, argv, "service_client");                                    // 「service_client」という名前のノードを宣言

    // インスタンス(クラスや構造体の実体)や変数の宣言
    ros::NodeHandle nh;                                                         // ノードハンドル(ノードに関する操作をするもの)を宣言
    ros::ServiceClient client;                                                  // クライアントを宣言

    //  サービスの起動確認
    ros::service::waitForService("calculate_two_numbers", ros::Duration(5.0));  // 「calculate_two_numbers」のサーバが立ち上がるまで待つ
    ROS_INFO("[%s] Connect to the service server", ros::this_node::getName().c_str());


    // クライアントの定義
    client = nh.serviceClient<ros_start::Calculation>("calculate_two_numbers"); // ros_start::Calculation                ：サーバへ送る要求の引数となるsrvの型
                                                                                // 第1引数 'calculate_two_numbers'       ：サービス名
    // 変数の定義
    ros::Rate loop_rate(1);                                                     // ループ周波数を1Hzに定義
    int count = 0;                                                              // カウンタの初期宣言
    ros_start::Expression ex;                                                   // exという変数をExpression型で扱う様に宣言
    ex.a = nh.param<double>(ros::this_node::getName() + "/a", 1);               // 変数aにrosparamで設定された値を代入(初期値：1.0)
    ex.b = nh.param<double>(ros::this_node::getName() + "/b", 1);               // 変数bにrosparamで設定された値を代入(初期値：1.0)

    // クライアントのループ処理
    while(ros::ok()) {
        if ( count%4 == 0 ) {
            ex.calculate = "+";
        } else if ( count%4 == 1 ) {
            ex.calculate = "-";
        } else if ( count%4 == 2 ) {
            ex.calculate = "*";
        } else {
            ex.calculate = "/";
        }
        ros_start::Calculation srv;                                             // srvという変数をCalculation型で扱う様に宣言
        srv.request.expression = ex;
        ROS_INFO("[%s] Call the calculate_two_numbers server\nExpression :%.2f %s %.2f", 
            ros::this_node::getName().c_str(), ex.a, ex.calculate.c_str(), ex.b);
        if (client.call(srv)) {                                                // クライアントからサービスを要求する(res：応答後の返り値)
            ROS_INFO("Result: %.2f\n", srv.response.result);
        } else {
            ROS_ERROR("Failed to call service calculate_two_numbers\n");
        }
        count++;                                                                // カウントアップ
		loop_rate.sleep();                                                      // 設定した周波数でループするようにsleep(一時停止)する
	}
    return 0;
};
}
```

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 150行目以下に追記 ---
add_executable(service_client src/service_client.cpp)                   # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(service_client ${catkin_LIBRARIES})               # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
add_dependencies(service_client ${PROJECT_NAME}_generate_messages_cpp)  # 独自型のメッセージの依存関係を追加する
```

CMakeLists.txtを編集したらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，service_serverを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_server
```


次に，計算処理で扱う変数a，bの値をrosparamを使って設定をします．
パラメータの値設定にはrosparam setコマンドを使用します．

```bash
# rosparam set <パラメータ名> <値>
$ rosparam set /service_client/a 3.0        # 変数aの値を3.0に設定
$ rosparam set /service_client/b 2.0        # 変数bの値を2.0に設定
```


次に，service_clientを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start service_client
```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[ INFO] [1650347400.159555846]: [/service_server] Received a call from a client
Request: 3.00 + 2.00
[ INFO] [1650347400.160019215]: sending back response: [5.00]

[ INFO] [1650347401.160301439]: [/service_server] Received a call from a client
Request: 3.00 - 2.00
[ INFO] [1650347401.160349889]: sending back response: [1.00]

[ INFO] [1650347402.160927726]: [/service_server] Received a call from a client
Request: 3.00 * 2.00
[ INFO] [1650347402.161039147]: sending back response: [6.00]

[ INFO] [1650347403.161087768]: [/service_server] Received a call from a client
Request: 3.00 / 2.00
[ INFO] [1650347403.161169472]: sending back response: [1.50]
```

```bash
# client側の端末
[ INFO] [1650347007.340172027]: [/service_client] Call the calculate_two_numbers server
Expression :3.00 + 2.00
[ INFO] [1650347007.341412381]: Result: 5.00

[ INFO] [1650347008.339689355]: [/service_client] Call the calculate_two_numbers server
Expression :3.00 - 2.00
[ INFO] [1650347008.342196803]: Result: 1.00

[ INFO] [1650347009.339690971]: [/service_client] Call the calculate_two_numbers server
Expression :3.00 * 2.00
[ INFO] [1650347009.342253494]: Result: 6.00

[ INFO] [1650347010.339703606]: [/service_client] Call the calculate_two_numbers server
Expression :3.00 / 2.00
[ INFO] [1650347010.342330917]: Result: 1.50
```

</details>


### 3. Launchで同時起動
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「service_py.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch  # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch service_py.launch              # 「service_py.launch」という名前のファイルを作成する
``` 

「service_py.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_py.launch」を開いて下さい．
launchファイルはxml形式で記述します．

```xml
<launch>
    <!-- namespace -->
    <!-- <group ns=<空間名>> -->
    <group ns="service">
        <!-- serveer -->
        <!-- <node pkg=<パッケージ名> type=<実行ファイル名> name=<ノード名> output=<出力形式> /> -->
        <node pkg="ros_start" type="service_server.py" name="server" output="screen" />
        <!-- client -->
        <node pkg="ros_start" type="service_client.py" name="client"  output="screen" >
            <!-- rosparam -->
            <!-- <param name=<パラメータ名> type=<型> value=<値> /> -->
            <param name="a" type="double" value="5" />
            <param name="b" type="double" value="2" />
        </node>
    </group>
</launch>
```

ここでは，ノードの同時起動の他にrosparamの設定と名前空間(namespace)の設定を行っています．
rosparamはlaunchファイルの中で設定することが出来ます．
名前空間とは，名前を識別するための概念であり，同じ名前の識別子(ノード名や，パラメータ名)を区別するのに扱います．
これにより，同じ名前を定義することによる競合を防ぐことが出来ます．

これで実行する準備が完了しました．
「service_py.launch」を実行します．

```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start service_py.launch
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650349388.327613]: 
 [ /service/server ] Ready to calculate_two_numbers.

[INFO] [1650349388.373219]: 
 [ /service/server ] Received a call from a client
Request: 5.00 + 2.00 = 7.00

[INFO] [1650349388.374058]: 
 [ /service/client ] Result: 7.00

[INFO] [1650349389.376948]: 
 [ /service/server ] Received a call from a client
Request: 5.00 - 2.00 = 3.00

[INFO] [1650349389.380783]: 
 [ /service/client ] Result: 3.00

[INFO] [1650349390.373761]: 
 [ /service/server ] Received a call from a client
Request: 5.00 * 2.00 = 10.00

[INFO] [1650349390.375018]: 
 [ /service/client ] Result: 10.00

[INFO] [1650349391.376919]: 
 [ /service/server ] Received a call from a client
Request: 5.00 / 2.00 = 2.50

[INFO] [1650349391.380505]: 
 [ /service/client ] Result: 2.50
```

</details>


<details><summary>※cppでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「service_cpp.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch  # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch service_cpp.launch              # 「service_cpp.launch」という名前のファイルを作成する
``` 

「service_cpp.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「service_cpp.launch」を開いて下さい．
launchファイルはxml形式で記述します．

```xml
<launch>
    <!-- namespace -->
    <!-- <group ns=<空間名>> -->
    <group ns="service">
        <!-- serveer -->
        <!-- <node pkg=<パッケージ名> type=<実行ファイル名> name=<ノード名> output=<出力形式> /> -->
        <node pkg="ros_start" type="service_server" name="server" output="screen" />
        <!-- client -->
        <node pkg="ros_start" type="service_client" name="client"  output="screen" >
            <!-- rosparam -->
            <!-- <param name=<パラメータ名> type=<型> value=<値> /> -->
            <param name="a" type="double" value="5" />
            <param name="b" type="double" value="2" />
        </node>
    </group>
</launch>
```

ここでは，ノードの同時起動の他にrosparamの設定と名前空間(namespace)の設定を行っています．
rosparamはlaunchファイルの中で設定することが出来ます．
名前空間とは，名前を識別するための概念であり，同じ名前の識別子(ノード名や，パラメータ名)を区別するのに扱います．
これにより，同じ名前を定義することによる競合を防ぐことが出来ます．

これで実行する準備が完了しました．
「service_cpp.launch」を実行します．


```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start service_cpp.launch
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1650349453.523979711]: [/service/client] Connect to the service server
[ INFO] [1650349453.525223858]: [/service/client] Call the calculate_two_numbers server
Expression :5.00 + 2.00
[ INFO] [1650349453.525683274]: [/service/server] Received a call from a client
Request: 5.00 + 2.00
[ INFO] [1650349453.526274353]: sending back response: [7.00]

[ INFO] [1650349453.526371699]: Result: 7.00

[ INFO] [1650349454.524859978]: [/service/client] Call the calculate_two_numbers server
Expression :5.00 - 2.00
[ INFO] [1650349454.527091403]: [/service/server] Received a call from a client
Request: 5.00 - 2.00
[ INFO] [1650349454.527172586]: sending back response: [3.00]

[ INFO] [1650349454.527425718]: Result: 3.00

[ INFO] [1650349455.524849026]: [/service/client] Call the calculate_two_numbers server
Expression :5.00 * 2.00
[ INFO] [1650349455.527035034]: [/service/server] Received a call from a client
Request: 5.00 * 2.00
[ INFO] [1650349455.527138944]: sending back response: [10.00]

[ INFO] [1650349455.527426087]: Result: 10.00

[ INFO] [1650349456.524815863]: [/service/client] Call the calculate_two_numbers server
Expression :5.00 / 2.00
[ INFO] [1650349456.527122709]: [/service/server] Received a call from a client
Request: 5.00 / 2.00
[ INFO] [1650349456.527202928]: sending back response: [2.50]

[ INFO] [1650349456.527461147]: Result: 2.50
```

</details>

<br>

## 3. Action通信
Action通信のプログラムを作成します．
本節では，指定した時間だけ待機するAction通信を行います．
Action通信では，メッセージやsrvファイルの代わりにactionファイルというものを使用します．
actionファイルを保存するために「action」というディレクトリを作成します．
```bash
$ cd ~/catkin_ws/src/ros_start      # catkin_ws/src/ros_start内の階層へ移動

# mkdir <ディレクトリ名>
$ mkdir action                      # 「action」という名前のディレクトリを作成する
```

「action」ディレクトリの中に「Timer.action」という名前のファイルを作成します．
```bash
$ cd action                         # action内の階層へ移動

# touch <ファイル名>
$ touch Timer.action                # 「Timer.action」という名前のファイルを作成する
```

「Timer.action」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「Timer.action」を開いて下さい．

```bash
# <型または変数> <変数名>       # goal(命令)の定義
---
# <型または変数> <変数名>       # result(最終結果)の定義
# ---
# <型または変数> <変数名>       # feedback(途中結果)の定義

float64 target_time
---
float64 total_time
---
float64 elapsed_time
```

Timer.actionから，次のメッセージが生成されます．
```bash
TimerAction.msg             # actionlib用のTimerAction型のメッセージ
TimerActionGoal.msg         # actionlib用のTimerActionのResult要素の型のメッセージ
TimerActionResult.msg       # actionlib用のResult要素の型のメッセージ
TimerActionFeedback.msg     # actionlib用のFeedback要素の型のメッセージ
TimerGoal.msg               # TimerAction.actionのGoal要素の型のメッセージ
TimerResult.msg             # TimerAction.actionのResult要素の型のメッセージ
TimerFeedback.msg           # TimerAction.actionのFeedback要素の型のメッセージ
```
これらのメッセージは，ActionClient-ActionServer間の通信のために，actionlibによって内部的に利用されます．

これらを実行ファイルで扱えるようにメイク設定ファイルのCMakeLists.txtと，パッケージに関する情報ファイルのpackage.xmlを編集します．

「CMakeLists.txt」ファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)で「CMakeLists.txt」を開いて下さい．

```cmake
# --- 10行目付近 ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib                 # 追記 action通信用パッケージ
)
```

```cmake
# --- 67行目付近 ---
## Generate actions in the 'action' folder
add_action_files(           # コメントアウト(#)をはずす        actionファイルを新規作成する際に必要な設定
  FILES                     # コメントアウト(#)をはずす
  Timer.action              # 追記 「Timer.action」をactionファイルとして追加
#   Action1.action          # 削除
#   Action2.action          # 削除
)                           # コメントアウト(#)をはずす

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs            # 追記 actionファイルを新規作成する際に必要な設定
)
```

```cmake
# --- 110行目付近 ---
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_start
CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs     # 追記 actionファイルを新規作成する際に必要な設定
#  DEPENDS system_lib
)

```

「package.xml」ファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)で「package.xml」を開いて下さい．

```xml
<!-- 61行目以下に追記 -->
  <depend>actionlib</depend>                     <!-- action通信用パッケージを使用する際に必要な設定 -->
  <depend>actionlib_msgs</depend>                <!-- actionファイルを新規作成する際に必要な設定 -->

```

これらの編集ができたらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

### 1. Server
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「action_server.py」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/scripts                         # catkin_ws/src/ros_start/scripts内の階層へ移動

# touch <ファイル名>
$ touch action_server.py           # 「action_server.py」という名前のファイルを作成する
``` 

「action_server.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「action_server.py」を開いて下さい．

```python
#! /usr/bin/env python
#coding:utf-8

# ライブラリのインポート
import rospy
import actionlib                                    # action通信用ライブラリのインポート
from ros_start.msg import TimerAction               # ros_start/actionからTimerActionをインポート
from ros_start.msg import TimerGoal                 # ros_start/actionからTimerAction.actionのGoal要素をインポート
from ros_start.msg import TimerResult               # ros_start/actionからTimerAction.actionのResult要素をインポート
from ros_start.msg import TimerFeedback             # ros_start/actionからTimerAction.actionのFeedback要素をインポート

# ActionServerクラスの定義
class ActionServer():
    # 初期定義
    def __init__(self):
        self.act_srv = actionlib.SimpleActionServer( 'timer_action_server', TimerAction, execute_cb=None, auto_start=False )    # actionserverの定義
                                                                                                                                # 第1引数 'timer_action_server' ：actionserverの名前
                                                                                                                                # 第2引数 TimerAction           ：使用するactionの型
                                                                                                                                # 第3引数 execute_cb=None       ：新規Goalを受け取ったときに行う処理関数の設定
                                                                                                                                # 第4引数 auto_start=False      ：actionserverの起動時に自動でパブリッシュを始めるかの設定
                                                                                                                                
        self.act_srv.register_goal_callback( self.goal_cb )                                                                     # Goal用コールバック処理の定義
        self.act_srv.register_preempt_callback( self.preempt_cb )                                                               # Preepmt(中断)用コールバック処理の定義
        self.goal = TimerGoal()                                                                                                 # goalという変数をTimerGoal型で扱う様に宣言
        self.act_srv.start()                                                                                                    # actionserverの起動
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nStart the Server...\n' )

    # Goal用コールバック関数
    def goal_cb(self):
        self.goal = self.act_srv.accept_new_goal();                                                                             # 新規Goalの受け入れ
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\n Got a new target_time = %.2f\n', self.goal.target_time )
        self.control_cb()                                                                                                       # Feedback，Result用コールバック関数の呼び出し
        return  

    # Preepmt(中断)用コールバック関数
    def preempt_cb(self):
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nPreempted...\n' )
        result = TimerResult()                                                                          # resultという変数をTimerResult型で扱う様に宣言
        result.total_time = -1.0;
        self.act_srv.setPreempted( result, 'I got Preempted!' );                                        # actionserverの中断
        return
    
    # Feedback，Result用コールバック関数
    def control_cb(self):
        if (self.act_srv.is_active() == False or self.act_srv.is_preempt_requested() == True) : return  # アクションサーバが起動していない，または中断が要求された場合リターンする
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nStart Timer\n' )
        feedback = TimerFeedback()                                                                      # feedbackという変数をTimerFeedback型で扱う様に宣言
        result = TimerResult()                                                                          # resultという変数をTimerResult型で扱う様に宣言

        init_time = rospy.Time.now().to_sec()                                                           # 初期時刻(sec)の定義
        curt_time = init_time                                                                           # 現在時刻を初期時刻と同期
        target_time = self.goal.target_time                                                             # 目標時間(sec)の代入
        feedback.elapsed_time = curt_time - init_time                                                   # 経過時間(sec)の計算

        loop_rate = rospy.Rate(2)                                                                       # ループ周波数を2Hzに定義
        while ( feedback.elapsed_time < target_time ) :                                                 # 経過時間が目標時間より小さい間繰り返す
            curt_time = rospy.Time.now().to_sec()                                                       # 現在時刻(sec)の取得
            feedback.elapsed_time = curt_time - init_time;                                              # 経過時間(sec)の計算
            self.act_srv.publish_feedback( feedback )                                                   # Feedback(途中結果)のパブリッシュ
            rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\npublishFeedback\nelapsed_time = %.2f\n', feedback.elapsed_time )
            loop_rate.sleep()                                                                           # 設定した周波数でループするようにsleep(一時停止)する
        result.total_time = feedback.elapsed_time                                                       # 最終結果に経過時間(sec)を代入
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nIt is time to end\ntotal_time = %.2f\n', result.total_time )
        self.act_srv.set_succeeded( result )                                                            # result(最終結果)のパブリッシュ
        return

if __name__ == '__main__':
    try:
        # ノードの初期宣言
        rospy.init_node('action_server')                # 「action_server」という名前のノードを宣言
        node = ActionServer()                           # ActionServerクラスのインスタンス生成
        rospy.spin()                                    # ActionServerの処理を行うためにスピン(無限ループ)させる

    except rospy.ROSInterruptException: pass            # rosが例外的に終了された時(Ctrl+C　など)に正しく終了させる
```

このコードではクラスというデータ構造を造る仕組みを用いています．
クラスの詳細な説明は割愛します．
クラスの概念はPythonやC++といったオブジェクト指向言語では基本となる概念であるため，各言語の参考書やインターネットなどで調べて，しっかりと理解しましょう．

プログラムが書けたらプログラムファイルに権限を付与します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 action_server.py
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，action_server.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_server.py
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650370790.795330]: 
 [ /action_server ]
Start the Server...
```

次に，アクションが上手く動いているか確認します．
途中結果のトピックを見るために，別の端末を開き，以下を実行します．

```bash
# rostopic echo <トピック名>
$ rostopic echo /timer_action_server/feedback

```

最終結果のトピックを見るために，別の端末を開き，以下を実行します．

```bash
# rostopic echo <トピック名>
$ rostopic echo /timer_action_server/result

```

アクションを起動するために，Goalのトピックをパブリッシュします．
トピックのパブリッシュにはrostopic pubコマンドを使用します．
別の端末を開き，以下を実行します．

```bash
# rostopic pub <トピック名> <型名> <値>
$ rostopic pub /timer_action_server/goal ros_start/TimerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_time: 10.0"
```
今回はtarget_timeを10.0secに設定し，パブリッシュしました．

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[INFO] [1650371592.295012]: 
 [ /action_server ]
 Got a new target_time = 10.00

[INFO] [1650371592.298262]: 
 [ /action_server ]
Start Timer

[INFO] [1650371592.301740]: 
 [ /action_server ]
publishFeedback
elapsed_time = 0.00

                    #
                    #
                    #
                    # 省略
                    #
                    #
[INFO] [1650371602.302747]: 
 [ /action_server ]
publishFeedback
elapsed_time = 10.00

[INFO] [1650371602.802312]: 
 [ /action_server ]
It is time to end
total_time = 10.00
```

```bash
# 途中結果トピックの出力端末
header: 
  seq: 1
  stamp: 
    secs: 1650371592
    nsecs: 301321029
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650371592
      nsecs: 294363021
    id: "/action_server-1-1650371592.294363021"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  elapsed_time: 7.39097595215e-05
---
                    #
                    #
                    #
                    # 省略
                    #
                    #
---
header: 
  seq: 21
  stamp: 
    secs: 1650371602
    nsecs: 302162885
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650371592
      nsecs: 294363021
    id: "/action_server-1-1650371592.294363021"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  elapsed_time: 10.0008370876
---


```

```bash
# 最終結果トピックの出力端末
header: 
  seq: 1
  stamp: 
    secs: 1650371602
    nsecs: 805594921
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650371592
      nsecs: 294363021
    id: "/action_server-1-1650371592.294363021"
  status: 3
  text: ''
result: 
  total_time: 10.0008370876
---
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「service_server.cpp」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch service_server.cpp          # 「service_server.cpp」という名前のファイルを作成する
``` 

「service_server.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「client_server.cpp」を開いて下さい．

```cpp
// ライブラリのインポート
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>  // action通信用ライブラリのインポート
#include <ros_start/TimerAction.h>                  // ros_start/actionからTimerActionをインポート

namespace ros_start {                                                   // ros_start名前空間の定義
    class ActionServer {                                                    // ActionServerクラスの宣言
        private:                                                                // メンバ変数の宣言
            ros::NodeHandle nh_;                                                    // ノードハンドル(pub，subの定義等)を宣言
            ros::NodeHandle pnh_;                                                   // プライベート用ノードハンドル(パラメータの設定等)を宣言
            actionlib::SimpleActionServer<ros_start::TimerAction> act_srv_;         // actionserverの宣言
                                                                                    // ros_start::TimerAction               ：使用するactionの型

            ros_start::TimerGoal goal_;                                             // goalという変数をTimerGoal型で扱う様に宣言
            double init_time;                                                       // 初期時刻(sec)変数の宣言

            void goalCb();                                                          // Goal用コールバック関数の宣言
            void preemptCb();                                                       // Preepmt(中断)用コールバック関数の宣言
            void controlCb();                                                       // Feedback，Result用コールバック関数の宣言

        public:
            ActionServer( );                                                        // コンストラクタ関数の宣言
    };
}

// Goal用コールバック関数
void ros_start::ActionServer::goalCb() {
    goal_ = *act_srv_.acceptNewGoal();                                              // 新規Goalの受け入れ
    ROS_INFO("[ %s ] Got a new target_time = %.2f", ros::this_node::getName().c_str(), goal_.target_time);
    controlCb();                                                                    // Feedback，Result用コールバック関数の呼び出し
    return;
}


// Preepmt(中断)用コールバック関数
void ros_start::ActionServer::preemptCb() {
    ROS_INFO("[ %s ] Preempted\n", ros::this_node::getName().c_str() );
    ros_start::TimerResult result;                                                  // resultという変数をTimerResult型で扱う様に宣言
    result.total_time = -1.0;
    act_srv_.setPreempted( result, "I got Preempted!" );                            // actionserverの中断
    return;
}


// Feedback，Result用コールバック関数
void ros_start::ActionServer::controlCb() {
    if ( !act_srv_.isActive() || act_srv_.isPreemptRequested() ) return;            // アクションサーバが起動していない，または中断が要求された場合リターンする
    ROS_INFO( "[ %s ]\nStart Timer", ros::this_node::getName().c_str() );

    ros_start::TimerFeedback feedback;                                              // feedbackという変数をTimerFeedback型で扱う様に宣言
    ros_start::TimerResult result;                                                  // resultという変数をTimerResult型で扱う様に宣言

    double init_time = ros::Time::now().toSec();                                    // 初期時刻(sec)の定義
    double curt_time = ros::Time::now().toSec();                                    // 現在時刻(sec)の定義
    double target_time = goal_.target_time;                                         // 目標時間(sec)の代入
    feedback.elapsed_time = curt_time - init_time;                                  // 経過時間(sec)の計算

    ros::Rate loop_rate(2);                                                         // ループ周波数を2Hzに定義
    while ( feedback.elapsed_time < target_time ) {                                 // 経過時間が目標時間より小さい間繰り返す
        curt_time = ros::Time::now().toSec();                                       // 現在時刻(sec)の取得
        feedback.elapsed_time = curt_time - init_time;                              // 経過時間(sec)の計算
        act_srv_.publishFeedback( feedback );                                       // Feedback(途中結果)のパブリッシュ
        ROS_INFO( "[ %s ]\npublishFeedback\nelapsed_time = %.2f\n", ros::this_node::getName().c_str(), feedback.elapsed_time );
        ros::spinOnce();                                                            // コールバック関数にアクセスする
        loop_rate.sleep();                                                          // 設定した周波数でループするようにsleep(一時停止)する
    }
    result.total_time = feedback.elapsed_time;                                      // 最終結果に経過時間(sec)を代入
    ROS_INFO("[ %s ]\nIt's time to end\ntotal_time = %.2f\n", ros::this_node::getName().c_str(), result.total_time );
    act_srv_.setSucceeded( result );                                                // result(最終結果)のパブリッシュ
    
    return;
}

// コンストラクタ関数の宣言
ros_start::ActionServer::ActionServer( ) : nh_(), pnh_("~"), act_srv_( nh_, "timer_action_server", false ) {    // actionserverの定義
    act_srv_.registerGoalCallback( boost::bind( &ros_start::ActionServer::goalCb, this ) );                     // Goal用コールバック処理の定義
    act_srv_.registerPreemptCallback ( boost::bind( &ros_start::ActionServer::preemptCb, this ) );              // Preepmt(中断)用コールバック処理の定義
    act_srv_.start();                                                                                           // actionserverの起動
    ROS_INFO( "[ %s ] Start the Server...\n", ros::this_node::getName().c_str() );
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "action_server");                     // 「action_server」という名前のノードを宣言
    ros_start::ActionServer act_srv;                            // ActionServerクラスのインスタンス生成
    ros::spin();                                                // ActionServerの処理を行うためにスピン(無限ループ)させる
    return 0;
}
```

このコードではクラスというデータ構造を造る仕組みを用いています．
クラスの詳細な説明は割愛します．
クラスの概念はPythonやC++といったオブジェクト指向言語では基本となる概念であるため，各言語の参考書やインターネットなどで調べて，しっかりと理解しましょう．

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 146行目以下に追記 ---
add_executable(action_server src/action_server.cpp)                   # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(action_server ${catkin_LIBRARIES})               # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
add_dependencies(action_server ${PROJECT_NAME}_generate_messages_cpp)  # 独自型のメッセージの依存関係を追加する
```

CMakeLists.txtを編集したらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，action_serverを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_server
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1650375336.617621663]: [ /action_server ] Start the Server...
```

次に，アクションが上手く動いているか確認します．
途中結果のトピックを見るために，別の端末を開き，以下を実行します．

```bash
# rostopic echo <トピック名>
$ rostopic echo /timer_action_server/feedback

```

最終結果のトピックを見るために，別の端末を開き，以下を実行します．

```bash
# rostopic echo <トピック名>
$ rostopic echo /timer_action_server/result

```

アクションを起動するために，Goalのトピックをパブリッシュします．
トピックのパブリッシュにはrostopic pubコマンドを使用します．
別の端末を開き，以下を実行します．

```bash
# rostopic pub <トピック名> <型名> <値>
$ rostopic pub /timer_action_server/goal ros_start/TimerActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_time: 10.0"
```
今回はtarget_timeを10.0secに設定し，パブリッシュしました．

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[ INFO] [1650375347.471596115]: [ /action_server ] Got a new target_time = 10.00
[ INFO] [1650375347.471635845]: [ /action_server ]
Start Timer
[ INFO] [1650375347.471657402]: [ /action_server ]
publishFeedback
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[ INFO] [1650375357.471845647]: [ /action_server ]
publishFeedback
elapsed_time = 10.00

[ INFO] [1650375357.971829065]: [ /action_server ]
It's time to end
total_time = 10.00
```

```bash
# 途中結果トピックの出力端末
header: 
  seq: 0
  stamp: 
    secs: 1650375347
    nsecs: 471645978
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650375347
      nsecs: 471546483
    id: "/action_server-1-1650375347.471546483"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  elapsed_time: 2.62260437012e-06
---
                    #
                    #
                    #
                    # 省略
                    #
                    #
---
header: 
  seq: 20
  stamp: 
    secs: 1650375357
    nsecs: 471760397
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650375347
      nsecs: 471546483
    id: "/action_server-1-1650375347.471546483"
  status: 1
  text: "This goal has been accepted by the simple action server"
feedback: 
  elapsed_time: 10.0001094341
---
```

```bash
# 最終結果トピックの出力端末
header: 
  seq: 0
  stamp: 
    secs: 1650375357
    nsecs: 971924141
  frame_id: ''
status: 
  goal_id: 
    stamp: 
      secs: 1650375347
      nsecs: 471546483
    id: "/action_server-1-1650375347.471546483"
  status: 3
  text: ''
result: 
  total_time: 10.0001094341
---
```

</details>

### 2. Client
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「scripts」ディレクトリの中に「action_client.py」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/scripts                         # catkin_ws/src/ros_start/scripts内の階層へ移動

# touch <ファイル名>
$ touch action_client.py           # 「action_client.py」という名前のファイルを作成する
``` 

「action_client.py」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「action_client.py」を開いて下さい．

```python
#! /usr/bin/env python
#coding:utf-8

import rospy
import actionlib                                    # action通信用ライブラリのインポート
from ros_start.msg import TimerAction               # ros_start/actionからTimerActionをインポート
from ros_start.msg import TimerGoal                 # ros_start/actionからTimerAction.actionのGoal要素をインポート

# ActionClientクラスの定義
class ActionClient():
    # 初期定義
    def __init__(self):
        self.client = actionlib.SimpleActionClient( 'timer_action_server', TimerAction)     # actionclientの定義
                                                                                            # 第1引数 'timer_action_server' ：actionserverの名前
                                                                                            # 第2引数 TimerAction           ：使用するactionの型

        self.goal = TimerGoal()                                                             # goalという変数をTimerGoal型で扱う様に定義
        self.action_executing = False
        
    # Goalを送る関数
    def send_goal(self):
        self.client.send_goal(                                                              # Goalを送る
            self.goal,                                                                      # 第1引数 self.goal         ：サーバへ送るGoal値
            self.done_cb,                                                                   # 第2引数 self.done_cb      ：Result(最終結果)用コールバック関数
            self.active_cb,                                                                 # 第3引数 self.active_cb    ：active(起動)確認用コールバック関数
            self.feedback_cb                                                                # 第4引数 self.feedback_cb  ：Feedback(途中結果)用コールバック関数
        )
        return

    # Result(最終結果)用コールバック関数
    def done_cb(self, state, result):
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nFinished!!\n' )
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nTotal_time = %.2f\n', result.total_time )
        action_executing = False
        return
    
    # active(起動)確認用コールバック関数
    def active_cb(self):
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nGoal just went active...\n' )
        return
    
    # Feedback(途中結果)用コールバック関数
    def feedback_cb(self, feedback):
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nGot Feedback of Progress to Goal\nelapsed_time = %.2f\n', feedback.elapsed_time )
        return

    # サーバを呼び出す関数
    def loop_call_server(self):
        loop_rate = rospy.Rate(1)                                                             # ループ周波数を1Hzに定義
        rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nWaiting For Server...\n' )
        self.client.wait_for_server()                                                         # 「timer_action_server」のサーバが立ち上がるまで待つ
        target_time = rospy.get_param( rospy.get_name() + '/target_time', 10.0)               # 変数target_taimeにrosparamで設定された値を代入(初期値：10.0)
        while not rospy.is_shutdown():                                                        # rospyがシャットダウンされるまで繰り返す
            if self.action_executing == False:                                                # アクションの起動時(初回)のみ実行する
                self.goal.target_time = target_time;                                          # 目標時間(sec)の代入
                self.send_goal()                                                              # send_goal関数を呼び出す
                rospy.loginfo('\n [ ' + rospy.get_name() + ' ]\nSet a new target_time = %.2f\n', self.goal.target_time )
                self.action_executing = True
            loop_rate.sleep()                                                                 # 設定した周波数でループするようにsleep(一時停止)する
        return

if __name__ == '__main__':
    try:
        # ノードの初期宣言
        rospy.init_node('action_client')                # 「action_client」という名前のノードを宣言
        
        node = ActionClient()                           # ActionClientクラスのインスタンス生成
        node.loop_call_server()                         # loop_call_server関数を呼び出す
        rospy.spin()                                    # ActionServerの処理を行うためにスピン(無限ループ)させる

    except rospy.ROSInterruptException: pass            # rosが例外的に終了された時(Ctrl+C　など)に正しく終了させる
```

プログラムが書けたらプログラムファイルに権限を付与します．

```bash
# chmod <モード> <ファイル名>
$ chmod 755 action_client.py
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，action_server.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_server.py
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650370790.795330]: 
 [ /action_server ]
Start the Server...
```

次に，action_client.pyを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_client.py

```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[INFO] [1650378633.509225]: 
 [ /action_server ]
Start the Server...

[INFO] [1650378648.279919]: 
 [ /action_server ]
 Got a new target_time = 10.00

[INFO] [1650378648.280642]: 
 [ /action_server ]
Start Timer

[INFO] [1650378648.281346]: 
 [ /action_server ]
publishFeedback
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[INFO] [1650378658.281944]: 
 [ /action_server ]
publishFeedback
elapsed_time = 10.00

[INFO] [1650378658.782526]: 
 [ /action_server ]
It is time to end
total_time = 10.00
```


```bash
# client側の端末
[INFO] [1650378648.182953]: 
 [ /action_client ]
Waiting For Server...

[INFO] [1650378648.279591]: 
 [ /action_client ]
Set a new target_time = 10.00

[INFO] [1650378648.280164]: 
 [ /action_client ]
Goal just went active...

[INFO] [1650378648.281545]: 
 [ /action_client ]
Got Feedback of Progress to Goal
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[INFO] [1650378658.282266]: 
 [ /action_client ]
Got Feedback of Progress to Goal
elapsed_time = 10.00

[INFO] [1650378658.787912]: 
 [ /action_client ]
Finished!!

[INFO] [1650378658.789808]: 
 [ /action_client ]
Total_time = 10.00
```

</details>


<details><summary>※C++でプログラムを作る場合はこちらをクリック</summary>
「src」ディレクトリの中に「service_server.cpp」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/src  # catkin_ws/src/ros_start/src内の階層へ移動

# touch <ファイル名>
$ touch service_server.cpp          # 「service_server.cpp」という名前のファイルを作成する
``` 

「service_server.cpp」ファイルに以下のプログラムを記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「client_server.cpp」を開いて下さい．

```cpp
// ライブラリのインポート
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>  // action通信用のクライアント用ライブラリのインポート
#include <actionlib/client/terminal_state.h>        // action通信用のゴール状態を定義するライブラリのインポート
#include <ros_start/TimerAction.h>                  // ros_start/actionからTimerActionをインポート

namespace ros_start {                                                   // ros_start名前空間の定義
    class ActionClient {                                                    // ActionClientクラスの定義
        private:                                                                // メンバ変数の定義
            ros::NodeHandle nh_;                                                    // ノードハンドル(pub，subの定義等)を宣言
            ros::NodeHandle pnh_;                                                   // プライベート用ノードハンドル(パラメータの設定等)を宣言
            actionlib::SimpleActionClient<ros_start::TimerAction> act_clt_;         // actionclientの定義
                                                                                    // ros_start::TimerAction               ：使用するactionの型

            ros_start::TimerGoal goal_;                                             // goalという変数をTimerGoal型で扱う様に定義
            bool action_executing_;

            void sendGoal();                                                        //  Goalを送る関数の定義
            void doneCb( const actionlib::SimpleClientGoalState& state, const ros_start::TimerResultConstPtr& result );   // Result(最終結果)用コールバック関数の定義
                                                                                                                          // 第1引数 state      ：Goal時の状態用変数
                                                                                                                          // 第2引数 result     ：最終結果用変数

            void activeCb( );                                                       // active(起動)確認用コールバック関数の定義
            void feedbackCb( const ros_start::TimerFeedbackConstPtr& feedback );    // Feedback(途中結果)用コールバック関数の定義
        public:
            ActionClient( );                                                        // コンストラクタ関数の宣言
            void loopCallServer();                                                  // サーバを呼び出す関数の定義
    };
}

//  Goalを送る関数
void ros_start::ActionClient::sendGoal() {
    act_clt_.sendGoal(                                                              // Goalを送る処理
        goal_,                                                                      // 第1引数 goal_       ：サーバへ送るGoal値
        boost::bind( &ros_start::ActionClient::doneCb, this, _1, _2 ),              // 第2引数 doneCb      ：Result(最終結果)用コールバック関数
        boost::bind( &ros_start::ActionClient::activeCb, this ),                    // 第3引数 activeCb    ：active(起動)確認用コールバック関数
        boost::bind( &ros_start::ActionClient::feedbackCb, this, _1 ) );            // 第4引数 feedbackCb  ：Feedback(途中結果)用コールバック関数
}

// Result(最終結果)用コールバック関数
void ros_start::ActionClient::doneCb( 
    const actionlib::SimpleClientGoalState& state, 
    const ros_start::TimerResultConstPtr& result ) {
    ROS_INFO( "[ %s ]\nFinished!!", ros::this_node::getName().c_str());
    ROS_INFO( "[ %s ]\nTotal_time = %.2f", ros::this_node::getName().c_str(), result->total_time );
    action_executing_ = false;
    ros::Duration(1.0).sleep();                                                     // 1.0secスリープする
}

// active(起動)確認用コールバック関数
void ros_start::ActionClient::activeCb( ) {
    ROS_INFO( "[ %s ]\nGoal just went active...\n", ros::this_node::getName().c_str());
}
// Feedback(途中結果)用コールバック関数
void ros_start::ActionClient::feedbackCb( 
    const ros_start::TimerFeedbackConstPtr& feedback ) {
    ROS_INFO( "[ %s ]\nGot Feedback of Progress to Goal\nelapsed_time = %.2f\n", ros::this_node::getName().c_str(), feedback->elapsed_time );
}

// コンストラクタ関数
ros_start::ActionClient::ActionClient( ) : nh_(), pnh_("~"),  act_clt_( "timer_action_server", true ) {
    ROS_INFO( "[ %s ] Waiting For Server...", ros::this_node::getName().c_str() );
    act_clt_.waitForServer();                                                                   // 「timer_action_server」のサーバが立ち上がるまで待つ
    ROS_INFO("[ %s ] Connect to the action server", ros::this_node::getName().c_str());
    action_executing_ = false;
}

// サーバを呼び出す関数
void ros_start::ActionClient::loopCallServer( ) {
    ros::Rate loop_rate(1);                                                                     // ループ周波数を1Hzに定義
    double target_time = pnh_.param<double>(ros::this_node::getName() + "/target_time", 10.0);  // 変数target_taimeにrosparamで設定された値を代入(初期値：10.0)
	while(ros::ok()) {                                                                            // rosが正常な間繰り返す
        if ( !action_executing_ ) {                                                             // アクションの起動時(初回)のみ実行する
            goal_.target_time = target_time;                                                    // 目標時間(sec)の代入
            sendGoal();                                                                         // sendGoal関数を呼び出す
            ROS_INFO( "[ %s ] Set a new target_time = %.2f\n", ros::this_node::getName().c_str(), target_time );
            action_executing_ = true;
        } 
		ros::spinOnce();                                                                            // コールバック関数にアクセスする
		loop_rate.sleep();                                                                          // 設定した周波数でループするようにsleep(一時停止)する
	}
    return;
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "action_client");                     // 「action_client」という名前のノードを宣言
    ros_start::ActionClient act_clt;                            // ActionClientクラスのインスタンス生成
    ros::spin();                                                // ActionClientの処理を行うためにスピン(無限ループ)させる
    return 0;
}
```

プログラムが書けたら実行ファイルにメイクします．
メイクをするために，メイクの設定が書かれたCMakeLists.txtというファイルを編集します．
お好きなエディタ(VScode，Atom，gedit　など)でcatkin_ws/src/ros_start/の階層にある「CMakeLists.txt」を開いて下さい．
以下を該当する箇所に追記して下さい．

```cmake
# --- 161行目以下に追記 ---
add_executable(action_client src/action_client.cpp)                   # ビルドする実行ファイルを定義 add_executable(<実行ファイル名> src/<ビルドするプログラムファイル名>)
target_link_libraries(action_client ${catkin_LIBRARIES})               # 実行ファイルで使用するライブラリをリンクする target_link_libraries(<実行ファイル名> ${catkin_LIBRARIES})
add_dependencies(action_client ${PROJECT_NAME}_generate_messages_cpp)  # 独自型のメッセージの依存関係を追加する
```

CMakeLists.txtを編集したらメイクをします．

```bash
$ cd ~/catkin_ws        # catkin_ws内の階層へ移動
$ catkin_make           # ビルド
```

これで実行する準備が完了しました．
早速実行していきます．
まず，roscoreを起動します．

```bash
$ roscore
```

続いて，action_serverを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_server
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650370790.795330]: 
 [ /action_server ]
Start the Server...
```

次に，action_clientを実行します．
別の端末を開き，以下を実行します．

```bash
# rosrun <パッケージ名> <実行ファイル名>
$ rosrun ros_start action_client

```

端末上に以下のように表示されたら成功です．

```bash
# server側の端末
[ INFO] [1650383062.899749141]: [ /action_server ] Start the Server...

[ INFO] [1650383063.116571234]: [ /action_server ] Got a new target_time = 10.00
[ INFO] [1650383063.116683892]: [ /action_server ]
Start Timer
[ INFO] [1650383063.116839185]: [ /action_server ]
publishFeedback
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[ INFO] [1650383073.116990128]: [ /action_server ]
publishFeedback
elapsed_time = 10.00

[ INFO] [1650383073.616898707]: [ /action_server ]
It's time to end
total_time = 10.00
```


```bash
# client側の端末
[ INFO] [1650383060.650283522]: [ /action_client ] Waiting For Server...
[ INFO] [1650383063.114779263]: [ /action_client ] Connect to the action server
[ INFO] [1650383063.116185362]: [ /action_client ] Set a new target_time = 10.00

[ INFO] [1650383063.116867754]: [ /action_client ]
Goal just went active...

[ INFO] [1650383063.117088285]: [ /action_client ]
Got Feedback of Progress to Goal
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[ INFO] [1650383073.117342448]: [ /action_client ]
Got Feedback of Progress to Goal
elapsed_time = 10.00

[ INFO] [1650383073.617493605]: [ /action_client ]
Finished!!
[ INFO] [1650383073.617585324]: [ /action_client ]
Total_time = 10.00
```

</details>

### 3. Launchで同時起動
<details><summary>※Pythonでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「action_py.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch  # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch action_py.launch               # 「action_py.launch」という名前のファイルを作成する
``` 

「action_py.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「action_py.launch」を開いて下さい．
launchファイルはxml形式で記述します．

```xml
<launch>
    <!-- namespace -->
    <!-- <group ns=<空間名>> -->
    <group ns="action">
        <!-- server -->
        <node pkg="ros_start" type="action_server.py" name="server" output="screen" />
        <!-- client -->
        <node pkg="ros_start" type="action_client.py" name="client" output="screen" >
            <!-- rosparam -->
            <rosparam>
                <!-- <パラメータ名> : <値> -->
                target_time : 5.0
            </rosparam>
        </node>
    </group>
</launch>

```

これで実行する準備が完了しました．
「action_py.launch」を実行します．

```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start action_py.launch
```

端末上に以下のように表示されたら成功です．

```bash
[INFO] [1650383612.406566]: 
 [ /action/server ]
Start the Server...

[INFO] [1650383613.489775]: 
 [ /action/client ]
Waiting For Server...

[INFO] [1650383613.543453]: 
 [ /action/client ]
Set a new target_time = 5.00

[INFO] [1650383613.543821]: 
 [ /action/server ]
 Got a new target_time = 5.00

[INFO] [1650383613.543854]: 
 [ /action/client ]
Goal just went active...

[INFO] [1650383613.544470]: 
 [ /action/server ]
Start Timer

[INFO] [1650383613.545096]: 
 [ /action/server ]
publishFeedback
elapsed_time = 0.00

[INFO] [1650383613.545186]: 
 [ /action/client ]
Got Feedback of Progress to Goal
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[INFO] [1650383618.546577]: 
 [ /action/server ]
publishFeedback
elapsed_time = 5.00

[INFO] [1650383618.547057]: 
 [ /action/client ]
Got Feedback of Progress to Goal
elapsed_time = 5.00

[INFO] [1650383619.046020]: 
 [ /action/server ]
It is time to end
total_time = 5.00

[INFO] [1650383619.051576]: 
 [ /action/client ]
Finished!!

[INFO] [1650383619.053382]: 
 [ /action/client ]
Total_time = 5.00
```

</details>


<details><summary>※cppでプログラムを作る場合はこちらをクリック</summary>
「launch」ディレクトリの中に「action_cpp.launch」という名前のファイルを作成します．

```bash
$ cd ~/catkin_ws/src/ros_start/launch  # catkin_ws/src/ros_start/launch内の階層へ移動

# touch <ファイル名>
$ touch action_cpp.launch              # 「action_cpp.launch」という名前のファイルを作成する
``` 

「action_cpp.launch」ファイルに以下を記述します．
お好きなエディタ(VScode，Atom，gedit　など)で「action_cpp.launch」を開いて下さい．
launchファイルはxml形式で記述します．

```xml
<launch>
    <!-- namespace -->
    <!-- <group ns=<空間名>> -->
    <group ns="action">
        <!-- serveer -->
        <!-- <node pkg=<パッケージ名> type=<実行ファイル名> name=<ノード名> output=<出力形式> /> -->
        <node pkg="ros_start" type="action_server" name="server" output="screen" />
        <!-- client -->
        <node pkg="ros_start" type="action_client" name="client"  output="screen" >
            <!-- rosparam -->
            <rosparam>
                <!-- <パラメータ名> : <値> -->
                target_time : 5.0
            </rosparam>
        </node>
    </group>
</launch>
```

これで実行する準備が完了しました．
「action_cpp.launch」を実行します．


```bash
# roslaunch <パッケージ名> <launchファイル名>
$ roslaunch ros_start action_cpp.launch
```

端末上に以下のように表示されたら成功です．

```bash
[ INFO] [1650383944.211233924]: [ /action/server ] Start the Server...

process[action/client-3]: started with pid [27330]
[ INFO] [1650383944.288483532]: [ /action/client ] Waiting For Server...
[ INFO] [1650383944.609955759]: [ /action/client ] Connect to the action server
[ INFO] [1650383944.612366072]: [ /action/client ] Set a new target_time = 5.00

[ INFO] [1650383944.612720696]: [ /action/server ] Got a new target_time = 5.00
[ INFO] [1650383944.612820786]: [ /action/server ]
Start Timer
[ INFO] [1650383944.613017667]: [ /action/server ]
publishFeedback
elapsed_time = 0.00
[ INFO] [1650383944.613010298]: [ /action/client ]
Goal just went active...


[ INFO] [1650383944.613235164]: [ /action/client ]
Got Feedback of Progress to Goal
elapsed_time = 0.00
                    #
                    #
                    #
                    # 省略
                    #
                    #
[ INFO] [1650383949.613138682]: [ /action/server ]
publishFeedback
elapsed_time = 5.00

[ INFO] [1650383949.613495062]: [ /action/client ]
Got Feedback of Progress to Goal
elapsed_time = 5.00

[ INFO] [1650383950.113101469]: [ /action/server ]
It's time to end
total_time = 5.00

[ INFO] [1650383950.113759970]: [ /action/client ]
Finished!!
[ INFO] [1650383950.113858277]: [ /action/client ]
Total_time = 5.00
```

</details>

ちなみに，汎用的なROSプログラムのテンプレートは下記のリンクにあります．
- [ROS Template](https://github.com/Yuki-Ikeda0810/ROS_template)

---

[トップに戻る](#rosの使用方法)
