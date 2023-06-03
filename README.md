# **SOBITS Manual**

[SOBITS](https://home.soka.ac.jp/~choi/)は，創価大学 理工学研究科の崔研究室のメンバーによって構成された，ロボットの研究開発チームです．
ロボットや機械が「人と共創」することによって，より快適な社会を実現できることを目指し，日々研究を行っております．

:arrow_forward: [研究について](/docs/what_is_research.md)

:arrow_forward: [SOBITSミーティングの進め方](/docs/activity_laboratory.md)

:arrow_forward: [SOBITSの基本情報](/docs/basic_information.md)

このような研究の一環として，人工知能やロボット工学の研究推進を目的とした，自律移動型ロボットによる世界規模の競技会であるRoboCup@Homeに出場しています．RoboCup@Homeは，日常生活でロボットの利用を想定した競技会であり，リビングルームやキッチンなどの家庭環境において，いかに人の役に立つ作業を遂行できるかを競い合います．

:arrow_forward: [RoboCupでの活動について](/docs/activity_robocup.md)

:arrow_forward: [RoboCup SOBITS Openについて](/docs/robocup_sobits_open.md)

本リポジトリでは，チームSOBITSとして共通する開発ルールや開発環境の構築方法，開発の進め方などを記載しています．
チームで開発を進めていく上で，とても大切な情報になりますので，ご一読ください．

<div align="center"><img src="/logos/sobits_logo03.png" width="100%"></div>

## **目次**

1. [**開発環境について**](#開発環境について)
    1. [Ubuntu](#1-ubuntu)
    2. [ROS](#2-ros)
    3. [Docker](#3-docker)

---

2. [**開発の進め方**](#開発の進め方)
    1. [コーディングスタイル](#1-コーディングスタイル)
    2. [Gitの使用方法](#2-gitの使用方法)
    3. [Gitの命名規則](#3-gitの命名規則)
    4. [Dockerの使用方法](#4-dockerの使用方法)
    5. [Docker Workspaceの使用方法](#5-docker-workspaceの使用方法)
    6. [ROSの使用方法](#6-rosの使用方法)

---

3. [**SOBITSのロボットについて**](#sobitsのロボットについて)
    1. [SOBIT EDUを動かす](https://github.com/TeamSOBITS/sobit_education)
    2. [SOBIT MINIを動かす](https://github.com/TeamSOBITS/sobit_mini)
    3. [SOBIT PROを動かす](https://github.com/TeamSOBITS/sobit_pro)

---

4. [**主要なパッケージ一覧**](#主要なパッケージ一覧)

## **開発環境について**

### 1. Ubuntu

SOBITSでは，コンピュータの基本的な制御を行うOS(Operating System)として，UbuntuというLinuxを母体としたOSを用いて開発しています．
Ubuntuは，無料で公開されているOSであり，誰でも自由にプログラムを書き換えることができるため，様々な開発に適しています．
以下の「Ubuntuをインストールする」をクリックすると，Ubuntuをインストールする方法について詳細に書かれているページまで移動します．

:arrow_forward: [Ubuntuをインストールする](/docs/install_ubuntu.md)

### 2. ROS

SOBITSでは，Ubuntuの環境上で，ROS(Robot Operating System)というミドルウェアを用いて，ロボットを動作させています．
ミドルウェアとは，コンピュータの基本的な制御を行うOSと，各要素技術を実行するプログラムとの中間に入るソフトウェアのことです．
SOBITSで開発を進める場合，UbuntuというOSと，ロボットの各要素技術を実行するプログラムとの中間を担うミドルウェアとしてROSを用いています．
ROSは，ロボット開発のために必要な一連のライブラリとツール群，さらにはユーザと開発者を繋ぐオープンなコミュニティを含む統合的ソフトウェアプラットフォームです．
具体的には，ハードウェアの抽象化，デバイス等のドライバ，各種開発言語用のライブラリ，視覚化ツールをはじめとするデバッガー，メッセージ通信やパッケージ管理，Wikiサイトによる情報提供やQ&Aサイトなどが提供されています．
ROSの大きな特徴は，ROSコミュニティと呼ばれる大規模なエコシステムが出来上がっていることです．
世界中の膨大な人数の利用者がインターネットを介して，いつでもどこでも問題が生じれば対応してくれるシステムが出来上がっています．
以下の「ROSをインストールする」をクリックすると，ROSをインストールする方法について詳細に書かれているページまで移動します．

:arrow_forward: [ROSをインストールする](/docs/install_ros.md)

### 3. Docker

以上のUbuntuやROSを用いた環境下で，ある程度の開発ができるようになった場合，Dockerを用いると，とても便利になります．
Dockerは，コンテナという単位でOSレベルの環境を複数構築・管理することができます．
これにより，ベースとなるOS上で，「Windows OS」や「Ubuntu OS」など，別のOSと切り替えを行いながら，開発をすることができるようになります．
もちろん，同じOSの環境を複数構築することもできるため，複数の開発プロジェクトで競合が生じてしまうなどの問題を解決することができます．
また，上記で行ってきた環境構築を一瞬で行うこともできるため，ある程度の開発ができるようになるととても便利になります．
以下の「Dockerをインストールする」をクリックすると，Dockerをインストールする方法について詳細に書かれているページまで移動します．
    
※ まだ，Ubuntu環境での開発をあまり経験したことがない場合，Dockerという仕組みを理解するのがとても難しいと思います．
たとえ，Dockerを使えこなせたとしても，誤った認識をしてしまったり，開発するにあたって大切な環境構築の方法がわからないままになってしまいます．
必ず，Ubuntu環境の構築や開発を行ってから，Dockerを使ってください．

:arrow_forward: [Dockerをインストールする](/docs/install_docker.md)

## **開発の進め方**

### 1. コーディングスタイル

SOBITSの一員として，開発を進めていくためには，いくつかのルールを守る必要があります．
個人で開発を進める場合，プログラムの書き方や開発の仕方などは自由に進めてもらっても問題がないのですが，チームで開発を進める場合，そうはいきません．
チームで開発を進めるのに，個人が自由に好きなようにプログラムを書いてしまうと，本人以外の人がそのプログラムを理解することが難しくなってしまい，開発が滞ってしまいます．
また，プログラムが理解できないことで，そのプログラムの使い方がわからなくなってしまったり，プログラム同士がうまく連携できなくなってしまいます．
このようなことを防ぐために，SOBITSでは，プログラムの変数を命名する規則や書き方を統一するルールを作っています．
このルールのことを一般的にコーディングスタイルと言います．
SOBITSのコーディングスタイルをみなさんが守ることにより，誰が見ても読みやすく分かりやすいプログラムを書くことができます．
そうすることで，他の人が書いたプログラムを使うことや再利用することが簡単になり，チームとしての開発効率も向上させることができます．
ぜひ，SOBITSのコーディングスタイルを守って，開発を進めるよう心掛けてください．
以下の「コーディングスタイル」をクリックすると，SOBITSのコーディングスタイルについて詳細に書かれているページまで移動します．

:arrow_forward: [コーディングスタイル](/docs/coding_style.md)

### 2. Gitの使用方法

SOBITSでは，チームで開発を進めていくにあたって，プログラム等のリソースを管理するGitというシステムを用いています．
Gitは，プログラムやプロジェクト等の管理をするための，ソフトウェア開発者用のプラットフォームです．
「Git」という単語は，分散型バージョン管理を実現するシステムの総称のことを指しており，有名なサービスとしてGitHubやGitLabがあります．
どちらも使い方に大きな違いはありません．
SOBITSでは，主に，GitLabを使用しています．
このようなサービスは，Cloud上でプログラムを管理することができるため，組織での大規模な開発等ではとても大切になってきます．
以下の「Gitの使用方法」をクリックすると，SOBITSでのGitの使い方について詳細に書かれているページまで移動します．

:arrow_forward: [Gitの使用方法](/docs/using_git.md)

### 3. Gitの命名規則

SOBITSの一員として，Gitを用いて開発を進めていくためには，プログラムのコーディングスタイルと同様に，いくつかのルールを守る必要があります．
Gitを使う場合でも，BranchやCommitの命名する規則や作り方等を統一することで，チームとしての開発効率も向上させることができます．
以下の「Gitの命名規則」をクリックすると，SOBITSでのGitの命名規則について詳細に書かれているページまで移動します．

:arrow_forward: [Gitの命名規則](/docs/git_style.md)

### 4. Dockerの使用方法

UbuntuやROSを用いた環境下で，ある程度の開発ができるようになった場合，Dockerを用いると，とても便利になります．
SOBITSでは，独自のDockerコンテナ環境[Docker Workspace]((https://github.com/TeamSOBITS/docker_ws))を作成し，提供しています．
Dockerを用いて開発を進める場合，Docker Workspace内にある「SOBITS Workspace」を用います．
これらの正しい使い方を理解し，それぞれの開発を進めていってください．
以下の「Dockerの使用方法」をクリックすると，Dockerの使用方法について詳細に書かれているページまで移動します．

※ まだ，Ubuntu環境での開発をあまり経験したことがない場合，Dockerという仕組みを理解するのがとても難しいと思います．
たとえ，Dockerを使えこなせたとしても，誤った認識をしてしまったり，開発するにあたって大切な環境構築の方法がわからないままになってしまいます．
必ず，Ubuntu環境の構築や開発を行ってから，Dockerを使ってください．

:arrow_forward: [Dockerの使用方法](/docs/using_docker.md)

### 5. Docker Workspaceの使用方法

ここでは，SOBITSがDockerを知らない人でも簡単にDockerを使いこなせるように開発したDocker Workspaceの使い方について説明していきます．
Docker Workspaceは，SOBITSで開発したこともあり，独自の仕様となっています．
Dockerの基本的な使い方とは大きくことなるので，Docker WorkspaceでないDockerを使用する際は，注意してください．
以下の「Docker Workspaceの使用方法」をクリックすると，SOBITSで作成したDocker Workspaceの使用方法について詳細に書かれているページまで移動します．

※ まだ，Ubuntu環境での開発をあまり経験したことがない場合，Dockerという仕組みを理解するのがとても難しいと思います．
たとえ，Dockerを使えこなせたとしても，誤った認識をしてしまったり，開発するにあたって大切な環境構築の方法がわからないままになってしまいます．
必ず，Ubuntu環境の構築や開発を行ってから，Dockerを使ってください．

:arrow_forward: [Docker Workspaceの使用方法](/docs/using_docker_ws.md)

### 6. ROSの使用方法

SOBITSでロボットの開発を行うためには，ROSの知識は必要不可欠になります．
ここでは，SOBITSとしてではなく，一般的なROSについての知識と使用方法について説明していきます．
以下の「ROSの使用方法」をクリックすると，一般的に知られているROSの使用方法について詳細に書かれているページまで移動します．

:arrow_forward: [ROSの使用方法](/docs/using_ros.md)

## **SOBITSのロボットについて**

SOBITSでは，SOBIT EDU，SOBIT MINI，SOBIT PROというロボットの開発を進めています．
「SOBIT 〇〇を動かす」をクリックすると，それぞれのロボットに必要なリポジトリまで移動します．

※ 初めてロボットを動かす場合は，必ずロボットを動かしたことのある先輩方に付き添ってもらいながらロボットを動かしましょう．

|<div align="center"><img src="/img/sobit_edu.png" width="90%"></div>|<div align="center"><img src="/img/sobit_mini.png" width="90%"></div>|<div align="center"><img src="/img/sobit_pro.png" width="90%"></div>|
|:---:|:---:|:---:|
|:arrow_forward: [SOBIT EDUを動かす](https://github.com/TeamSOBITS/sobit_edu)|:arrow_forward: [SOBIT MINIを動かす](https://github.com/TeamSOBITS/sobit_mini)|:arrow_forward: [SOBIT PROを動かす](https://github.com/TeamSOBITS/sobit_pro)|

## **主要なパッケージ一覧**

SOBITSでは、ロボットを動かすためのプログラムをパッケージとしてまとめています．
自律移動系、画像認識系、音声系など必要最低限のパッケージを紹介しています．

以下の主要なパッケージ一覧をクリックすると具体的なパッケージ一覧を見ることができます．動かしてみましょう．

:arrow_forward: [主要なパッケージ一覧](/docs/using_package.md) (要修正)

---

[トップに戻る](#sobits-manual)
