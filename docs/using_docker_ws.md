# **Docker Workspaceの使用方法**

[Docker](https://www.docker.com/why-docker)は，コンテナという単位でOSレベルの環境を複数構築・管理することができるミドルウェアになります．
Dockerについて知らないという方は，以下のリンクからDockerについて確認してください．

:arrow_forward: [Dockerの使用方法](/docs/using_docker.md)

※ まだ，Ubuntu環境での開発をあまり経験したことがない場合，Dockerという仕組みを理解するのがとても難しいと思います．
たとえ，Dockerを使えこなせたとしても，誤った認識をしてしまったり，開発するにあたって大切な環境構築の方法がわからないままになってしまいます．
必ず，Ubuntu環境の構築や開発を行ってから，Dockerを使ってください．

ここでは，SOBITSがDockerを知らない人でも簡単にDockerを使いこなせるように開発したDocker Workspaceの使い方について説明していきます．
Docker Workspaceは，SOBITSで開発したこともあり，独自の仕様となっています．
Dockerの基本的な使い方とは大きくことなるので，Docker WorkspaceでないDockerを使用する際は，注意してください．

## **目次**

1. [**Docker Workspaceの使用方法**](#docker-workspaceの使用方法)
    1. [Docker Workspaceのコマンド操作](#1-docker-workspaceのコマンド操作)
    2. [Docker Workspaceの作成](#2-docker-workspaceの作成)
    3. [Docker Workspaceの操作](#3-docker-workspaceの操作)

## **Docker Workspaceの使用方法**

### 1. Docker Workspaceのコマンド操作

SOBITSで開発したDocker Workspaceを使用する場合は，コマンド操作を使用することはほとんどありません．
[Dockerの使用方法](/docs/using_docker.md)でも紹介していますが，ここでは，Docker Workspaceを操作する際に用いるコマンドについて紹介します．

以下のコマンドで，Dockerイメージを管理することができます．

```bash
# イメージの一覧を確認
$ docker images

# イメージを削除
$ docker rmi イメージ名

# イメージを削除
$ docker rmi イメージID
```

また，以下のコマンドで，Dockerコンテナを管理することができます．

```bash
# 実行中のコンテナの一覧を確認
$ docker ps

# コンテナの一覧を確認
$ docker ps -a

# コンテナを削除
$ docker rm <コンテナ名>

# コンテナを削除
$ docker rm <CONTAINER ID>
```

以上が，Docker Workspaceを使う上で，必要となるコマンドになります．
[Dockerの使用方法](/docs/using_docker.md)で紹介した，Dockerイメージやコンテナを作成・起動・停止する等のコマンドは，全てDocker Workspaceの中で行っています．
なので，Docker Workspaceを使用するユーザは，Dockerイメージとコンテナの管理を行うコマンドだけを知っていれば，十分にDockerを使いこなせるようになっています．

### 2. Docker Workspaceの作成

ここでは，Docker Workspaceを作成する方法について説明していきます．
その前に，[Dockerの使用方法](/docs/using_docker.md)，[Gitの使用方法](/docs/using_git.md)，[Dockerのインストール方法](/docs/install_docker.md)をそれぞれ理解し，実行したか確認してください．
それぞれの項目で，Docker Workspaceを使うためのセットアップも行っているため，実行していない場合は必ず実行してください．

以上を実行できたら，以下のコマンドをホーム直下で入力して[Docker Workspace](https://github.com/TeamSOBITS/docker_ws)をダウンロードします．
図は，ホーム直下に[Docker Workspace](https://github.com/TeamSOBITS/docker_ws)をダウンロードした様子になります．

```bash
# リモートからローカルにDocker Workspaceをダウンロード
$ git clone https://github.com/TeamSOBITS/docker_ws.git
```

<div align="center"><img src="/img/using_docker_ws01.png" width="60%"></div>

次に，「docker_ws」の中にある「container」ディレクトリまで移動します．
この「container」ディレクトリの中に，「ros_melodic_sobit_ws」ディレクトリがあるので，それをコピー(Ctrl + C)します．
基本的には，この「ros_melodic_sobit_ws」ディレクトリが，Docker Workspaceで用いる1つのコンテナになります．
図は，「docker_ws/container」ディレクトリの中の様子を示しています．

<div align="center"><img src="/img/using_docker_ws02.png" width="60%"></div>

そしたら，ホームに戻り，コピーした「ros_melodic_sobit_ws」ディレクトリをペースト(Ctrl + V)で貼り付けます．
このとき，「ros_melodic_sobit_ws」ディレクトリの名前を任意のコンテナ名に変更します．
Docker Workspaceでは，この任意で決めたディレクトリの名前を受け取り，自動的にその名前のDockerイメージとコンテナを作成するように設定しています．
このDockerイメージとコンテナの作成については後述しますが，とりあえず，このディレクトリ名がDockerのイメージ名とコンテナ名になると認識しておいてください．
SOBITS Manualでは，「ros_melodic_sobit_ws」の名前のまま，進めていきます．
図は，ホームディレクトリの中の様子を示しています．

<div align="center"><img src="/img/using_docker_ws03.png" width="60%"></div>

ここからは，Dockerイメージとコンテナの作成について説明していきます．
以下のコマンドで，先程ペーストした「ros_melodic_sobit_ws」の中にある「Dockerfiles」の「cpu」ディレクトリまで移動します．

※ この「cpu」というのは，開発で用いるプログラム等を動作させる計算機のことを指しています．
使用しているPCにGPUが搭載されていて，GPUで動作させたい場合は，「gpu」の方に移動します．
「gpu」ディレクトリの下に，GPUで動作させるためのCUDAやcuDNNのバージョンが書いてあるディレクトリがあるので，自身の環境に適したディレクトリに移動してください．
このとき，適した環境がない場合，自身で新しく作成する必要があります．
以下のリンクから，自身で新しく作成することができます．

:arrow_forward: [任意のCUDA・cuDNNバージョンでのDockerfileの作り方](aaa) (未完成)

```bash
# 「ros_melodic_sobit_ws/Dockerfiles/cpu」ディレクトリまで移動
$ cd ros_melodic_sobit_ws/Dockerfiles/cpu
```

「ros_melodic_sobit_ws/Dockerfiles/cpu」ディレクトリまで移動したら，そこで，以下のコマンドを実行します．
ここでは，「build.sh」というシェルスクリプトファイルを「bash」というコマンドを用いて実行しています．
シェルスクリプトファイルは，端末などに入力するコマンドを記述しておくことができるファイルです．
複数のコマンドを記述しておくことで，それらを自動的に実行することができます．
「build.sh」は，[Dockerの使用方法](/docs/using_docker.md)にて説明していたコマンドを省略することで，簡単にDockerイメージを作成することができるシェルスクリプトファイルになります．
このとき，「build.sh」にて作成されるDockerイメージは，その大本のディレクトリ名と同じ名前で自動的に作成されます．
SOBITS Manualでは，「ros_melodic_sobit_ws」というディレクトリ名のため，「ros_melodic_sobit_ws」というDockerイメージが作成されます．
この操作では，OSや必要なライブラリなどをインストール&構築するため，少し時間が掛ります．
気長に待ちましょう．

```bash
# Dockerイメージの作成
$ bash build.sh
```

Dockerイメージを作成することができたら，以下のコマンドを実行します．
ここでも先程と同様に，「run.sh」というシェルスクリプトファイルを「bash」というコマンドを用いて実行しています．
「run.sh」は，[Dockerの使用方法](/docs/using_docker.md)にて説明していたコマンドを省略することで，簡単にDockerコンテナを作成することができるシェルスクリプトファイルになります．
このとき，「run.sh」にて作成されるDockerコンテナは，その大本のディレクトリ名と同じ名前で自動的に作成されます．
SOBITS Manualでは，「ros_melodic_sobit_ws」というディレクトリ名のため，「ros_melodic_sobit_ws」というDockerコンテナが作成されます．
この操作は，作成したDockerイメージを起動するだけなので，一瞬で処理が終わります．

```bash
# Dockerコンテナの作成
$ bash run.sh
```

図に示すように，黄色い文字で「Dockerコンテナ名 sobits@~$」と表示されていれば，成功です．
SOBITS Manualでは，「ros_melodic_sobit_ws sobits@~$」と表示されます．
これでDockerコンテナを使うことができるようになりました．
以上の操作は，Dockerコンテナを作るときのみに行います．

<div align="center"><img src="/img/using_docker_ws07.png" width="60%"></div>

早速ですが，以下のコマンドを実行して，Dockerコンテナから出ましょう．
一度作成したコンテナは，何度でもその環境に入ることができます．
これらの操作については次章にて説明します．

```bash
# Dockerコンテナから出る
$ exit
```

### 3. Docker Workspaceの操作

ここでは，Docker Workspaceを操作する方法について説明していきます．
初回のみ，Dockerコンテナを簡単に起動・停止するための設定を行います．
ホームディレクトリで「Ctrl + H」キーを押して，「.」から始まる隠しファイルを表示します．
そうすると，「.bashrc」というファイルが表示されるので，そのファイルを開きます．
「.bashrc」は，端末を起動した時に実行される設定ファイルです．
図は，ホームディレクトリの隠しファイルを表示した様子を示しています．

「.bashrc」ファイルを開いたら，そこの一番下の行に以下の行を追記します．
「alias」というものを「.bashrc」に設定することで，よく使うコマンドを省略することができます．
ここでは，「ce」というコマンドを端末に入力すると，「python ~/docker_ws/container_executer.py」を実行するという省略コマンドを設定しています．

```bash
# Pythonファイルの実行コマンドを「ce」で省略する
$ alias ce="python ~/docker_ws/container_executer.py"
```

<div align="center"><img src="/img/using_docker_ws04.png" width="60%"></div>

「.bashrc」にこれらの設定を記述したら，一旦端末を閉じて，再度端末を開いてください．
「.bashrc」は，端末を起動した時に実行されるため，このようにすることで，書き換えた「.bashrc」の設定が端末に反映されるようになります．
早速，設定した「ce」コマンドを実行してみましょう．

```bash
# Dockerコンテナを管理するGUIを表示
$ ce
```

そうすると，以下の図のようなGUIが画面に表示されると思います．
このGUIを用いることで，[Dockerの使用方法](/docs/using_docker.md)にて説明していたコマンドを入力することなく，Dockerコンテナを起動・停止することができます．
このGUIには，作成したDockerコンテナの一覧を表示しています．
任意のコンテナの「start」ボタンをクリックすることで，Dockerコンテナを起動します．

<div align="center"><img src="/img/using_docker_ws05.png" width="60%"></div>

Dockerコンテナが起動すると，以下の図のように，他のボタンが表示されるようになります．
それでは，各ボタンについて簡単に説明していきます．
まず，「restart」ボタンは，クリックすることで，Dockerコンテナの再起動をすることができます．
次に，「stop」ボタンは，クリックすることで，Dockerコンテナを停止することができます．
作業が終わった場合は，必ずDockerコンテナを停止しましょう．
最後に，「exec」ボタンは，クリックすることで，Dockerコンテナの中に入った端末を追加で開くことができます．
Dockerコンテナ内で複数の端末を使いたい場合は，この「exec」をクリックしてください．
このようにGUIを駆使することで，Dockerのコマンドを入力することなく，簡単にDockerコンテナを起動・停止することができます．

<div align="center"><img src="/img/using_docker_ws06.png" width="60%"></div>

GUIを用いてDockerコンテナを起動し，Dockerコンテナを作成した際と同じように，黄色い文字で「Dockerコンテナ名 sobits@~$」と表示されていれば，成功です．
SOBITS Manualでは，「ros_melodic_sobit_ws sobits@~$」と表示されます．
このようにDocker Workspaceを使うことで，簡単にDockerコンテナを作成し，それらを管理・起動・停止することができます．

<div align="center"><img src="/img/using_docker_ws07.png" width="60%"></div>

しかし，Dockerは基本的にCUIでの操作しか行うことができません．
これでは，Dockerコンテナ内のファイルを開いてエディタで編集するなどのことが，できません．
そこで，Docker Workspaceでは，一部のディレクトリをホストOSとDockerコンテナで共有するように設定しています．
具体的には，ホストOS側の「コンテナ名/src」とDockerコンテナ側の「catkin_ws/src」を共有しているため，この場所を使って開発を進めていきます．
これにより，ホストOS側でエディタを使って編集したファイルを，共有された場所に保存することで，そのファイルをDockerコンテナ側でも使うことができます．
図では，ホストOS側の「ros_melodic_sobit_ws/src」ディレクトリとDockerコンテナ側の「catkin_ws/src」が共有されており，同じファイル構成であることが確認できます．
このようにすることで，CUIでの操作しかできないDockerコンテナ上で，疑似的にGUIでの操作を行っての開発を進めることができます．
以上のDocker Workspaceの機能を駆使することで，Dockerを知らない人でも簡単にDockerを使いこなせるようになります．

<div align="center"><img src="/img/using_docker_ws08.png" width="60%"></div>

今回紹介しきれなかったのですが，実は他にもホストOSとDockerコンテナ間で共有しているディレクトリがあります．
実機のロボットを接続し，ホストOS側で読み込んだものをDockerコンテナ側に共有しているというものです．
ほとんどの人が知らなくても問題ない設定ですが，気になる方は以下のリンクから確認することができます．

:arrow_forward: [docker runの設定](aaa) (未完成)

---

[トップに戻る](#docker-workspaceの使用方法)
