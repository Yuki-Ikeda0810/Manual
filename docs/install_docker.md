# **Dockerのインストール方法**

## **目次**
1. [**インストール方法**](#1-インストール方法)
    1. [事前準備](#1-事前準備)
    2. [依存関係のあるパッケージをインストール](#2-依存関係のあるパッケージをインストール)
    3. [公開鍵を入手する](#3-公開鍵を入手する)
    4. [外部リポジトリの登録](#4-外部リポジトリの登録)
    5. [docker-ceのインストール](#5-docker-ceのインストール)
    6. [実行権限の付与](#6-実行権限の付与)
    7. [サウンドの設定](#7-サウンドの設定)
    8. [サービスの有効化](#8-サービスの有効化)
    9. [再起動](#9-再起動)
2. [**クイックインストール**](#2-クイックインストール)

<br>

## **1. インストール方法**

### 1. 事前準備
本研究室では，DockerをUbuntuのOS上で使用します．  
WindowsやMacOSでも使用できますが，ここでは，UbuntuのOS上にインストールする手順を紹介していきます．  
もし，まだUbuntuをインストールしていない場合は，  
:arrow_forward: [Ubuntuをインストール](/docs/install_ubuntu.md)
からUbuntuをインストールして下さい．

<br>

### 2. 依存関係のあるパッケージをインストール
「Dockerをインストールする」上で必要となるパッケージを予めインストールします．

以下のコマンドを端末で実行します． 

``` bash
$ sudo apt install -y 
    apt-transport-https 
    ca-certificates 
    curl 
    software-properties-common
```

<details><summary>※各パッケージの説明はこちらをクリック</summary>

- ***apt-transport-https***  
    httpsに対応したaptを使用するためのパッケージ

- ***ca-certificates***  
    ubntuで扱う基本的なCA証明書を提供しているパッケージ  
    (CA：Certification Authority)とは認証局のことで，ネットワーク通信を行う際に公開鍵証明書を発行する役割を持つ  
    **主にセキュリティ面で使われるものだと認識しておいて下さい**

- ***curl***  
    ネットワーク上とデータの送受信を行うパッケージ  
    様々な通信手順を用いてURLで示されるネットワーク上の場所との間でデータの送受信を行う

- ***software-properties-common***  
    ソフトウェアの基本的な設定や情報を扱うパッケージ
    後に扱う*add-apt-repository*はこのパッケージに含まれている

</details>

<br>

### 3. 公開鍵を入手する
2.でインストールしたcurlを使ってネットワーク上から公開鍵を入手します．  

以下のコマンドを端末で実行します．  

``` bash
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

<br>

### 4. 外部リポジトリの登録
apt(Advanced Package Tool)はパッケージ管理システムであり，  
デフォルトだとUbuntuの公式リポジトリに登録されているものしかインストールできないため，    
現在の状態ではDockerをインストールすることができません．  
そこでDockerのパッケージが格納されている外部リポジトリを登録します．

以下のコマンドを端末で実行します．  

``` bash
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

<br>

### 5. docker-ceのインストール
aptを更新し，実際にDockerをインストールします．  
*docker-ce*のceとはCommunity Editionの略であり無償版を意味しています．  

以下のコマンドを端末で実行します．  

``` bash
$ sudo apt update  
$ sudo apt install -y docker-ce
```

<br>

### 6. 実行権限の付与
現在の状態ではDockerを使用するのに管理者権限sudoコマンドが必要になります．  
そこで，現在使用しているユーザにDockerを使用する権限を付与します．  
これにより，sudoコマンドなしでDockerが使用できます．

以下のコマンドを端末で実行します．  

``` bash
$ sudo groupadd docker  
$ sudo usermod -aG docker $USER  
$ sudo su - $USER
```  

<br>

### 7. サウンドの設定
Dockerのコンテナ内でサウンド関係を扱うために，PulseAudioというソフトの設定を行います．  

ここではまず，echoコマンドについて説明します．  
echoは，画面に文字列や数値，変数を表示するコマンドです．  
また，画面ではなく，ファイルに出力することも出来ます．  

```bash
$ echo "hogehoge" >> hoge.txt
```
上のようなコマンドを実行するとhoge.txtファイルの中に「hogehoge」が書き加えられます．  
但し，sudo(管理者)権限がないと編集の出来ないファイル(鍵のついたファイル)に書き加える場合は，  
以下のように「>>」の部分を「| sudo tee 」とする必要があります．

```bash
$ echo "hogehoge" | sudo tee hoge.txt
```
この機能を使って様々な設定をファイルに書き込んでいきます．  


以下のコマンドを端末で実行します．

```bash
$ echo "pacmd load-module module-native-protocol-unix socket=/tmp/pulseaudio.socket &> /dev/null" >> ~/.bashrc
$ echo '#!bin/bash
    touch /tmp/pulseaudio.client.conf
    echo "default-server = unix:/tmp/pulseaudio.socket 
        # Prevent a server running in the container 
        autospawn = no
        daemon-binary = /bin/true
        # Prevent the use of shared memory
        enable-shm = false" >> /tmp/pulseaudio.client.conf' | sudo tee /etc/profile.d/sound_setup.sh
$ sudo bash /etc/profile.d/sound_setup.sh
```

<br>

### 8. サービスの有効化
dockerをシステムとして有効にします．

以下のコマンドを端末で実行します．  

``` bash
$ sudo systemctl enable docker
```

<br>

### *9. 再起動
再起動をかけることで今までの設定を反映させます．

以下のコマンドを端末で実行します．  

``` bash
$ sudo reboot
```

<br>

## **2. クイックインストール**
なお，これらの手順は[setup_audio.sh](/install_sh/setup_audio.sh)と[install_docker.sh](/install_sh/install_docker.sh)をダウンロードし，以下のコマンドを打つことで自動でDockerのインストールまで行ってくれます．

``` bash
$ bash install_docker.sh
```

---

[トップに戻る](#dockerのインストール方法)
