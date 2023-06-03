# **Dockerの使用方法**

## **目次**

1. [**Dockerとは**](#1-dockerとは)

2. [**Dockerの使用方法**](#2-dockerの使用方法)
    1. [Dockerコンテナを作成する](#1-dockerコンテナを作成する)
    2. [複数の異なるDockerコンテナを作成する](#2-複数の異なるdockerコンテナを作成する)
    3. [複数の同じDockerコンテナを作成する](#3-複数の同じdockerコンテナを作成する)
    4. [同じDockerコンテナを複数のPCで作成する](#4-同じdockerコンテナを複数のpcで作成する)

3. [**Dockerの基本操作**](#3-dockerの基本操作)
    1. [Dockerイメージに関するコマンド](#1-dockerイメージに関するコマンド)
    2. [Dockerコンテナに関するコマンド](#2-dockerコンテナに関するコマンド)

4. [**Dockerfileの作成**](#4-dockerfileの作成)

<br>

## **1. Dockerとは**

Dockerは，Linuxのコンテナ技術を使ったもので，よく仮想マシンと比較されます．  
VirtualBoxなどの仮想マシンでは，ホストマシン上で仮想ソフトを利用して，仮想マシンとゲストOSを作成し，仮想カーネル動かすことでROSのようなミドルウェアを動かします．  
それに対し，コンテナはホストマシンのカーネルを利用し，プロセスやユーザなどを隔離することで，あたかも別のマシンが動いているかのように動かすことができます．  
そのため，仮想マシンの様に各仮想OSごとに仮想マシンとゲストOSを作成する必要がなく，ホストカーネルと物理マシンをそのまま使うことが出来ます．
要するに，仮想マシンと比べて，軽量で高速に起動・停止，複数の開発環境の管理などが可能な，開発環境構築ツールということです．

<details><summary> 各種専門用語の説明はこちらをクリック </summary>

- カーネル  
    OSの中核であり，CPUやメモリなどのハードウェアに指示を出すソフトウェアのこと

- ミドルウェア  
    コンピュータの基本的な制御を行うOSと，各要素技術を実行するプログラムとの中間に入るソフトウェアのこと  

</details>

Dockerではこの1つの開発環境のことを**Dockerコンテナ**と言います．Dockerを使うととても便利だという点には，以下のことが挙げられます．

- 1つのホストOSの中で，異なる複数のOS環境を切り替えて開発することができる．
- コード化されたファイルを共有することで，どこでも誰でも同じ環境が作れる．
- スクラップ & ビルドが容易にできる．

<div align="center"><img src="/img/using_docker01.png" width="70%"></div>

Dockerを使う上での注意点として，ホストOS上のGUI(Graphical User Interface)環境は変わらないという点があります．
これは，Ubuntu環境でWindowsのDockerコンテナを作成しても，ホストOSであるUbuntuの見た目は変わらないということになります．
では，WindowsのDockerコンテナを作成した場合，何が変化するのかというと，ホストOSであるUbuntu環境のCUI(Character User Interface)がWindowsに変化します．
どういうことを言っているのかわからないという方も多いと思うので，とりあえず「そういうものなんだ」という認識で読み進めてみてください．

また，SOBITSがDockerを知らない人でも簡単にDockerを使いこなせるように開発したDocker Workspaceについて知りたいという方は，以下のリンクからDocker Workspaceについて確認して下さい．

- [Docker Workspaceの使用方法](/docs/using_docker_ws.md)

<br>

## **2. Dockerの使用方法**

### 1. Dockerコンテナを作成する

ここからは具体的にどのようにすることで，便利なDockerというツールを使うことができるのか，図を用いて説明していきます．

以下の図は，Dockerfileからコンテナを作成するイメージです．
先程も説明しましたが，Dockerは，コンテナという単位でOSレベルの環境を複数構築・管理することができます．
このコンテナを作成するということが，Dockerを使う上での肝になってきます．

このDockerコンテナを作成するための手順は以下の通りです．

#### ① Dockerfileの作成

まず，作りたい開発環境の構成が記してあるDockerfileを作成します．
例えば，Ubuntu18.04のOSを使うことや，開発環境として必要なミドルウェアやライブラリ，パッケージ等を明記します．
(後で追加できるが，Dockerfileに記述しておくと環境の再構築が容易になる)

#### ② Dockerイメージの作成

次に，このDockerfileをBuildし，Dockerコンテナの土台となるDockerイメージを作成します．
これは，Dockerfileに記述した開発環境の構成を実際に作っていく過程になります．

#### ③ Dockerコンテナの作成

最後に，作成したDockerイメージをRunすることで，Dockerfileに記述した開発環境のコンテナを作成・起動することが出来ます．
コンテナに入っても，ホストOSのGUI環境(見た目)は変わることがありません．
基本的には，端末内などのCUI環境でのみ，コンテナに入ることが出来ます．

<div align="center"><img src="/img/using_docker02.png" width="60%"></div>

### 2. 複数の異なるDockerコンテナを作成する

Dockerの強みの1つとして，複数のOS環境を管理し，別のOSと切り替えを行いながら，開発することができる点があります．
以下の図は，複数のDockerfileから複数のコンテナを作成するイメージです．
このように，別のDockerfileを用意し，先ほどの手順でDockerコンテナを作成すると，同じホストOS内に複数のDockerコンテナを構築・管理することができます．

<div align="center"><img src="/img/using_docker03.png" width="60%"></div>

### 3. 複数の同じDockerコンテナを作成する

また，1つのDockerfileから複数の別のコンテナを作成することが出来ます．
以下の図は，同じDockerfileから複数のコンテナを作成するイメージです．
このように，同じDockerコンテナ環境で，開発作業ごとにコンテナを分けるということができます．

<div align="center"><img src="/img/using_docker04.png" width="60%"></div>

### 4. 同じDockerコンテナを複数のPCで作成する

DockerのコンテナはDockerfileやDockerイメージがあれば，同じDockerコンテナの開発環境を作成することができます．
そのため，DockerfileやDockerイメージを，別のホストOSであるPCに共有すれば，別のPCでも全く同じDockerコンテナの環境を共有することができます．
以下の図は，同じDockerイメージを違うホストOSに共有し，同じコンテナを作成するイメージです．
このように，同じDockerコンテナを複数の人と共有することが出来ます．

<div align="center"><img src="/img/using_docker05.png" width="80%"></div>

<br>

## **3. Dockerの基本操作**

冒頭でも説明しましたが，Dockerは基本的にホストOSのCUI(Character User Interface)環境に対して，任意のOS環境(Dockerコンテナ)を作成する形です．
そのため，Dockerを操作する方法は，全てCUIでのコマンド操作です．
ここでは，Dockerを使うためのコマンドについて紹介していきます．

### 1. Dockerイメージに関するコマンド

Dockerイメージを管理・作成するコマンドです．

```bash
# イメージの一覧を確認
$ docker images

# イメージを取得のみ
$ docker pull イメージ名

# イメージを削除
$ docker rmi イメージ名

# イメージを削除
$ docker rmi イメージID

# コンテナからイメージを作成
$ docker commit コンテナ名 作成するイメージ名

# Dockerfileを使ってイメージを作成
$ docker build -t 作成するイメージ名 Dockerfileが存在するディレクトリパス
```

### 2. Dockerコンテナに関するコマンド

Dockerイメージを用いてDockerコンテナを作成・起動するコマンドです．

```bash
# イメージを取得し，コンテナを作成・起動
$ docker run -it イメージ名
```

以下は，「docker run」コマンドの引数です．

<table>
    <tr>
        <th>引数</th>
        <th>説明</th>
    </tr>
    <tr>
        <td>-d</td>
        <td>コンテナをバックグラウンド実行する．
        この引数がない場合，コンテナ起動時に実行されるコマンドを実行した状態になる．
        例えばそのコマンドのコンソール出力が表示された状態．</td>
    </tr>
    <tr>
        <td>--gpus</td>
        <td>GPUを使用するときの設定． 「--gpu all」とすることで， 全てのGPUを使うことができる．</td>
    </tr>
    <tr>
        <td>--name</td>
        <td>コンテナ名を指定する．指定しなかった場合，自動で名前が付けられる．「--name コンテナ名」のようにして書く．</td>
    </tr>
    <tr>
        <td>--net</td>
        <td>Dockerネットワークを使用してコンテナを起動する．基本的には，「--net ネットワーク名」で書く．</td>
    </tr>
    <tr>
        <td>-p</td>
        <td>ホストとコンテナ間のポートフォワード設定．
        基本的には，「-p ホスト側のポート:コンテナ側のポート」で書く．ホスト側を省略すると自動で設定される．
        コンテナは，Dockerにより作成されるネットワークに属するため，このオプションを使わないと，ホストのIPアドレスを用いて，
        コンテナで使用しているポートにはアクセスができない．</td>
    </tr>
    <tr>
        <td nowrap>--privileged</td>
        <td>このコンテナに対して拡張権限を与える．
        パソコンに接続しているデバイスをコンテナで使用できる．
        コンテナに特権を付与することで，ホスト上のすべてのデバイスへのアクセスする権限が与えることができる．
        この場合，docker runのオプションとして「--privileged」と入力すると特権が付与される．</td>
    </tr>
    <tr>
        <td>--device</td>
        <td>docker runのオプションとして「--device=ホスト上のデバイスファイル名:コンテナ上のファイル名」と入力すると，コンテナ上にデバイスファイルが作成される．
        そして，そのファイルにアクセスすると，ホスト上のデバイスファイルにアクセスしたのと同じ結果が得られる．</td>
    </tr>
    <tr>
        <td>-v</td>
        <td>ホスト側とコンテナ側のディレクトリを共有させる設定．基本的には，「-v ホスト側のポート:コンテナ側のポート」で書く．</td>
    </tr>
    <tr>
        <td>-w</td>
        <td>コンテナ内の作業用ディレクトリを指定する．</td>
    </tr>
</table>

Dockerコンテナを管理・起動・停止するコマンドです．

```bash
# 実行中のコンテナの一覧を確認
$ docker ps

# コンテナの一覧を確認
$ docker ps -a

# コンテナを起動
$ docker start コンテナ名

# コンテナにログイン
$ docker exec -it コンテナ名 bash

# コンテナを停止
$ docker stop コンテナ名

# コンテナを削除
$ docker rm コンテナ名

# コンテナを削除
$ docker rm コンテナID
```

以下は，「docker rm」コマンドの引数です．

<table>
    <tr>
        <th>引数</th>
        <th>説明</th>
    </tr>
    <tr>
        <td>-f</td>
        <td>コンテナ起動中に強制的に削除する．</td>
    </tr>
</table>

DockerコンテナやホストOSのファイルをコピーするコマンドです．

```bash
# ホスト側からコンテナ内にファイルをコピー
$ docker cp ホスト側のファイルパス コンテナ名:コンテナ内のコピー先ディレクトリパス

# コンテナ側からホスト内にファイルをコピー
$ docker cp コンテナ名:コンテナ内のコピー元ファイルパス ホスト側のコピー先ディレクトリパス
```

Dockerコンテナ同士を繋ぐネットワークに関するコマンドです．

```bash
# Dockerネットワークを作成
$ docker network create 作成するネットワーク名

# Dockerネットワークの一覧を確認
$ docker network ls

# Dockerネットワークの詳細を確認
$ docker network inspect wordpress-network
```

Dockerコンテナを一括で削除するコマンドです．

```bash
# 停止コンテナ， タグなしイメージ， 未使用ボリューム， 未使用ネットワーク一括削除
$ docker system prune

# 停止コンテナの一括削除
$ docker rm `docker ps -f "status=exited" -q`

# 停止コンテナの一括削除
$ docker container prune

# 全コンテナ一括削除
$ docker rm -f `docker ps -a -q`

# 未使用イメージ一括削除
$ docker image prune

# 未使用イメージ一括削除
$ docker rmi `docker images -q`

# <none>イメージ一括削除
$ docker rmi $(docker images -f "dangling=true" -q)
```

<br>

## **4. Dockerfileの作成**

Dockerfileの作り方について説明します．
まず，Dockerfileのファイル名は必ず「Dockerfile」にして下さい．
他の名前だと上手くBuildすることが出来ません．  
以下は，Dockerfileの記述例と各行の記述内容についての説明です．

```bash
FROM centos:7                                               # ＜1＞
RUN yum install -y java                                     # ＜2＞
ADD files/apache-tomcat-9.0.6.tar.gz /opt/                  # ＜3＞
CMD [ "/opt/apache-tomcat-9.0.6/bin/catalina.sh", "run" ]   # ＜4＞
```

<table>
    <tr>
        <th nowrap>行番号</th>
        <th>説明</th>
    </tr>
    <tr>
        <td>＜1＞</td>
        <td>FROMは，ベースとするDockerイメージを指定する．
        今回は，Dockerイメージをもとに2以降を行う．
        また，事前にのDockerイメージを取得(docker pull)していなくても，ローカルにない場合は，Dockerイメージ作成のコマンド実行時に自動で取得する．</td>
    </tr>
    <tr>
        <td>＜2＞</td>
        <td>RUNは，OSのコマンドを実行する際に使用する．
        ここでは，Javaのインストールを実行している．
        「-y」オプションを付けて，インストールするかどうか聞かれないようにする．</td>
    </tr>
    <tr>
        <td>＜3＞</td>
        <td>ADDは，tar.gzファイルのコンテナへのコピーと，tarの展開を行う．
        「ADD <コピー元> <Dockerイメージ内のコピー・展開先>」の書き方．
        今回の例の場合は，Dockerfileと同階層にfilesディレクトリを作成し，その中にtomcatの媒体を配置しておく．
        また，ADDコマンドと似たコマンドとして，COPYコマンドがある．
        本連載では詳細は省略するが，COPYコマンドはtarの展開はおこなわず，コピー処理のみ行う．</td>
    </tr>
    <tr>
        <td>＜4＞</td>
        <td>CMDは，コンテナ起動時に実行するコマンドを記述する．
        CMDコマンドと似たコマンドとして，ENTRYPOINTコマンドがある．</td>
    </tr>
</table>

---

[トップに戻る](#dockerの使用方法)
