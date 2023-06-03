# **Ubuntuのインストール方法**

SOBITSでは，コンピュータの基本的な制御を行うOS(Operating System)として，UbuntuというLinuxを母体としたOSを用いて開発しています．
Ubuntuは，無料で公開されているOSであり，誰でも自由にプログラムを書き換えることができるため，様々な開発に適しています．
ここでは，Ubuntuをインストールする方法について説明していきます．

## **目次**

1. [**Ubuntuのバージョンをアップデートする方法**](#Ubuntuのバージョンをアップデートする方法)
    1. [現在のUbuntuを削除](#1-現在のubuntuを削除)
    2. [新しいUbuntuのバージョンのインストール](#2-新しいubuntuのバージョンのインストール)
    3. [高速スタートアップの無効化](#3-高速スタートアップの無効化)
    4. [UEFI設定画面の表示](#4-uefi設定画面の表示)
    5. [BIOS画面を操作する方法](#5-bios画面を操作する方法)
    6. [Fast BootとSecure Bootの無効化](#6-fast-bootとsecure-bootの無効化)
    7. [優先順位の変更](#7-優先順位の変更)
    8. [Try Ubuntu without installingを選択](#8-try-ubuntu-without-installingを選択)
    9. [Ubuntuのインストール](#9-ubuntuのインストール)
    10. [Ubuntuの起動の確認](#10-ubuntuの起動の確認)
    11. [時刻同期の設定](#11-時刻同期の設定)

---

2. [**初めてWindowsとUbuntuをデュアルブートするためにUbuntuをインストールする方法**](#初めてWindowsとUbuntuをデュアルブートするためにUbuntuをインストールする方法)
    1. [高速スタートアップを無効化](#1-高速スタートアップを無効化)
    2. [UEFI設定画面の表示](#2-uefi設定画面の表示)
    3. [BIOS画面を操作する方法](#3-bios画面を操作する方法)
    4. [Fast BootとSecure Bootの無効化](#4-fast-bootとsecure-bootの無効化)
    5. [優先順位の変更](#5-優先順位の変更)
    6. [Try Ubuntu without installingを選択](#6-try-ubuntu-without-installingを選択)
    7. [Ubuntuのインストール](#7-ubuntuのインストール)
    8. [Ubuntuの起動の確認](#8-ubuntuの起動の確認)
    9. [時刻同期の設定](#9-時刻同期の設定)



## **Ubuntuのバージョンをアップデートする方法**

### 1. 現在のUbuntuを削除
        
Windowsからコマントプロンプトを管理者として起動して、ドライブディスクCのルートに移動して
コマンドプロンプトで `cd c:￥`を入力する。



<div align="center"><img src="/img/ubuntu_install01.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install02.png" width="60%"></div>




次に、コマンドプロンプトで `bcdedit /enum firmware`を実行して、ブート情報を調べる。  
この結果からUbuntuの識別子(identifier)を確認する。 

<div align="center"><img src="/img/ubuntu_install03.png" width="60%"></div>


続けて、コマンドプロンプトで `bcdedit /delete　{識別子}`コマンドを実行して、Ubuntuをブート メニューから削除する。



<div align="center"><img src="/img/ubuntu_install29.png" width="60%"></div>



再度、コマンドプロンプトで `bcdedit /enum firmware`を実行して、
ブートメニューからUbuntuが削除されたことを確認する。

続けて、コマンドプロンプトで `diskpart`コマンドを実行して、UEFIシステムパーティションを編集できるようにディスクドライブをマウントする。  
ここでは、Zドライブとしてマウントしている。  
コマンドプロンプトで `list disk`コマンドを実行  
ディスク0を選択するために、コマンドプロンプトで `sel disk 0`コマンドを実行

次に、コマンドプロンプトで `list vol`コマンドを実行  
Volumeのリストの中から **Fs** の欄が **FAT32** のVolumeを選択する。**←人によって違うから要チェック**   
コマンドプロンプトで`sel vol ?`コマンドを実行して、ボリュームを選択する。**←?には、各自の番号入れて下さい**

コマンドプロンプトで `assign letter=Z`コマンドを実行することで、
Zドライブとしてマウントする。  
コマンドプロンプトで `exit`コマンドを実行する。



<div align="center"><img src="/img/ubuntu_install05.png" width="60%"></div>



マウントしたのち、EFIのディレクトリまで進み、Ubuntuディレクトリを削除する。  
コマンドプロンプトで `cd /d Z:`コマンドを実行したら、次に `dir`コマンドを実行する。

<div align="center"><img src="/img/ubuntu_install37.png" width="60%"></div>

コマンドプロンプトで `cd EFI`コマンドを実行したら、次に `dir`コマンドした後に、コマンドプロンプトで `rmdir /S ubuntu`コマンドを実行して、Ubuntuを削除します。  
最後は、コマンドプロンプトで `exit`コマンドを実行したのち、再起動すると、LinuxのGRUBが表示されなくなり、Windowsが直接起動するようになれば成功です。
        

<div align="center"><img src="/img/ubuntu_install38.png" width="60%"></div>



### 2. 新しいUbuntuのバージョンのインストール

Windowsを起動して、スタートボタンを右クリックして、 **「ディスクの管理」** を選択する。



<div align="center"><img src="/img/ubuntu_install06.png" width="60%"></div>



ディスクの中で、いかにもUbuntuが入っていたのだろうなと伺える容量のパーティションを右クリックして、  **ボリュームの削除**  を行う。
削除ができれば、**未割り当て** と確認できれば、 **「ディスクの管理」** を閉じます。



<div align="center"><img src="/img/ubuntu_install07.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install08.png" width="60%"></div>



### 3. 高速スタートアップの無効化

スタートボタン → **「システム」** → **「電源とスリープ」** へ移動する。  
電源とスリープの画面の右側に、 **「電源の追加設定」** をクリックする。



<div align="center"><img src="/img/ubuntu_install09.png" width="60%"></div>



そして、左側にある **「電源ボタンの動作を選択する」** をクリック  
次に、**「現在利用可能ではない設定を変更します」** をクリック  
最後に、**「高速スタートアップを有効にする（推奨）」** のチェックを外し、
**「変更の保存」** を押して終了です。



<div align="center"><img src="/img/ubuntu_install10.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install11.png" width="60%"></div>



### 4.  UEFI設定画面の表示


このタイミングで、LiveUSB（新しいUbuntuのバージョンが入ったUSB）を挿入してください。  

＊＊＊注意＊＊＊   
Ubuntu20.04のLiveUSBを接続後に「フォーマットしますか？」と聞かれるので、「いいえ」を選択してください。   
＊＊＊＊＊＊＊＊

UEFI設定画面を表示させる方法は、WindowsでShiftキーを押しながら再起動する。

そうすると、次のような画面が表示される。
        
**「トラブルシューティング」** → **「詳細オプション」** → **「UEFIファームウェアの設定」** 
の順に選択する。すると、PCが再起動すると同時にUEFI設定画面（BIOS）の世界に移動する。



<div align="center"><img src="/img/ubuntu_install12.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install13.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install14.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install15.png" width="60%"></div>



### 5.  BIOS画面を操作する方法

BIOSとは、パソコンに接続されている周辺機器を制御するためのソフトウェアで、ノートパソコン、デスクとトップパソコンなどに必ず搭載されているシステム

BIOSの画面は基本的にキーボードだけで操作します。  
・項目の移動（選択）：カーソルキーの「↑」、「→」、「↓」、「←」、「Tab」  
・選択した項目を決定：「Enter」  
・数値の変更：「-」、「＋」  
・期値に戻す：「F9」  
・終了する：「Esc」  
・保存してから終了する:「F10」


### 6.  Fast BootとSecure Bootの無効化

・Secure Bootとは、コンピュータ起動時の安全性を確保するため、デジタル署名で起動するソフトウェアを検証する機能。  
・Fast Bootとは、PCの電源投入時やWindowsの再起動時に、BIOSレベルでのドライバーの初期化を最小限にすることで、PC起動時のPOSTに要する時間を短縮する機能。

これらの機能を無効（Disabled）にすることが目的です。PC or マザーボードメーカーによって、設定項目の場所が全く異なるので注意して下さい。
でも、やることは同じなので安心してください。

＊＊＊＊＊＊   
BIOSでの設定項目が見つからない場合、BIOSの種類によっては検索欄があるので、検索してみてください。   
＊＊＊＊＊＊


以下に一例を示す。

Secure Boot Enableのチェックを外して、Secure Bootを無効化する。



<div align="center"><img src="/img/ubuntu_install17.png" width="60%"></div>



Fast bootのThoroughにチェックを入れる。Thoroughの状態は、起動中にハードウェア及び設定の完全な初期化を行います。



<div align="center"><img src="/img/ubuntu_install18.png" width="60%"></div>




### 7.  優先順位の変更

続けて、BIBE画面上で、USBメモリから起動させるためにBoot優先度を変更します。  
Ubuntuを上に移動させ、Ubuntuの優先度を上げる。



<div align="center"><img src="/img/ubuntu_install16.png" width="60%"></div>



### 8.  Try Ubuntu without installingを選択


<div align="center"><img src="/img/ubuntu_install22.png" width="60%"></div>

＊＊＊＊＊＊   
Ubuntu20.04をインストールする場合に、「Try Ubuntu without installing」ではなく「Ubuntu」と表示される場合があるので注意してください。   
＊＊＊＊＊＊


**NvidiaのGPUを搭載していないPC**  であれば、「Enter」キーを押してください。

**NvidiaのGPUを搭載したPC**  であれば、「e」キーを押してください。  
「e」キーを押したら、次に、`quiet splash`と書かれた箇所を`nomodeset`に書き換えて、「F10」キーを押します。そしたら、起動が始まります。

```　
setparams ‘Try Ubuntu without installing’  
 set gfxpayload=keep  
 linux /casper/vmlinuz.efi file=/cdrom/pressed/ubuntu.seed boot=casper  **quiet splash**  — debian-installer/language=ja keyboard-configuration/layoutcode?=ja keyboard-configuration/modelcode?=jp106  
 initrd /casper/initrd.lz
```

```
setparams ‘Try Ubuntu without installing’  
 set gfxpayload=keep  
 linux /casper/vmlinuz.efi file=/cdrom/pressed/ubuntu.seed boot=casper  **nomodeset**  — debian-installer/language=ja keyboard-configuration/layoutcode?=ja keyboard-configuration/modelcode?=jp106  
 initrd /casper/initrd.lz
```



### 9.  Ubuntuのインストール

デスクトップの  **「Ubuntuをインストール」**  をクリックして、インストール作業を進めます。はじめに、使用言語やキーボードレイアウトを指定します。



<div align="center"><img src="/img/ubuntu_install23.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install24.png" width="60%"></div>




次に、アップデートと他のソフトウェアの画面では、  **「通常のインストール」**  を選択します。そして、  **「グラフィックスとWi-Fiハードウェアと追加のメディアフォーマットのサードパーティ製ソフトウェアをインストールする」**  を選択する。



<div align="center"><img src="/img/ubuntu_install25.png" width="60%"></div>




インスト－ルの種類の画面では、「それ以外」を選択します。  
ここで「Windows Boot Managerがインストールされています」と出ない場合は、SSDが認識できていない可能性があるので、AHCI設定を再確認して下さい。

<div align="center"><img src="/img/ubuntu_install34.png" width="60%"></div>


次に、Ubuntuの①インストール先のメイン領域を選択し、②変更を押します。  
「ext4ジャーナリングファイルシステム」を選択し、マウントポイントを「/」に設定します。  
次に、ブートローダをインストールするデバイスを選択します。  
選択するデバイスは、①で選択したパーテーションと同じ名前か、デバイス名となります。（人によって異なる）

設定が済んだら、「インストール」を選択します。


<div align="center"><img src="/img/ubuntu_install33.png" width="60%"></div>


<div align="center"><img src="/img/ubuntu_install40.png" width="60%"></div>



どこに住んでいますか？の画面では、   **「Tokyo」**  を選択します。



<div align="center"><img src="/img/ubuntu_install27.png" width="60%"></div>



あなたの情報を入力してくださいの画面では、研究室のパソコンを借りている場合は、割り振られているPC名と研究室でのパスワードを入力してください。  
私物のパソコンの場合は、自由に設定してください。



<div align="center"><img src="/img/ubuntu_install28.png" width="60%"></div>



ここまでの設定が終わると、インストールが始まるので、終わるまで待機してください。
インストールが終われば、再起動するのではなく、「試用を続ける」を選択しましょう。

次に、Ubuntuが起動するか確認を行うために、シャットダウンを行いましょう。
完全に電源が落ちたことが確認してから、LiveUSBを抜きます。


### 10.  Ubuntuの起動の確認

PCを起動させます。
起動すると、以下の選択画面が表示されます。

<div align="center"><img src="/img/ubuntu_install31.jpg" width="60%"></div>

「Ubuntu」選択して、UbuntuOSを起動させます。
上手く起動できれば、ログインし、インターネットに繋ぎます。

「Ctrl」 + 「Alt」 + 「T」キーを同時に押して、端末を起動させます。
端末上で、`sudo apt update`、`sudo apt upgrade`を実行してOSとソフトウェアを最新に更新します。

``` bash

$ sudo apt update
$ sudo apt upgrade

```

### 11.  時刻同期の設定
以下のコマンドを実行して、時刻同期を行います。

まず、UbuntuとWindowsの時刻の不一致を無くします。
``` bash
$ sudo timedatectl set-local-rtc 1​
```

通信プロトコルであるntpをインストールします。
```bash
$ sudo apt install -y ntp
```

``` bash
$ sudo gedit /etc/ntp.conf

```

18行目から以下の様に表示される。
```
# Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
# on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
# more information.
pool 0.ubuntu.pool.ntp.org iburst
pool 1.ubuntu.pool.ntp.org iburst
pool 2.ubuntu.pool.ntp.org iburst
pool 3.ubuntu.pool.ntp.org iburst

# Use Ubuntu's ntp server as a fallback.
pool ntp.ubuntu.com

```

21〜24行目と27行目の先頭に「＃」でコメントアウト  
29,30行目に「server time.soka.ac.jp」「server ntp.nict.jp」を挿入

```
# Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
# on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
# more information.
#pool 0.ubuntu.pool.ntp.org iburst　　# コメントアウト
#pool 1.ubuntu.pool.ntp.org iburst    # コメントアウト
#pool 2.ubuntu.pool.ntp.org iburst　　# コメントアウト
#pool 3.ubuntu.pool.ntp.org iburst　　# コメントアウト

# Use Ubuntu's ntp server as a fallback.
#pool ntp.ubuntu.com　　　　　　　　　# コメントアウト

server time.soka.ac.jp　　　　　　　　# 追記
server ntp.nict.jp                    # 追記
```


設定したntpを更新
```bash
$ sudo service ntp restart
```

ntpを確認
```bash
$ ntpq -p
```

以下のように、「time.soka.ac.jp」が表示されれば設定完了です。

<div align="center"><img src="/img/ubuntu_install36.png" width="60%"></div>

なお、これらの時間同期の手順は[setup_time.sh](/install_sh/setup_time.sh)をダウンロードし、以下のコマンドを打つことで自動で設定を行ってくれます。

``` bash
$ bash time_setup.sh
```


以上で、インストール作業を終了です。





## **初めてWindowsとUbuntuをデュアルブートするためにUbuntuをインストールする方法**


Windowsを起動して、スタートボタンを右クリックして、 **「ディスクの管理」** を選択する。



<div align="center"><img src="/img/ubuntu_install06.png" width="60%"></div>




Windowsがインストールされた（C:）ドライブを右クリックして、 **「ボリュームの縮小」** を選択する。  
縮小可能な領域が表示される。ここで、自分がUbuntuに割り当てたい容量を確保する。



<div align="center"><img src="/img/ubuntu_install19.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install20.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install21.png" width="60%"></div>



未割り当て領域が正しくできたことを確認して、 **「ディスクの管理」** を閉じる。



### 1.  高速スタートアップを無効化

スタートボタン → **「システム」** → **「電源とスリープ」** へ移動する。  
電源とスリープの画面の右側に、 **「電源の追加設定」** をクリックする。



<div align="center"><img src="/img/ubuntu_install09.png" width="60%"></div>



そして、左側にある **「電源ボタンの動作を選択する」** をクリック  
次に、**「現在利用可能ではない設定を変更します」** をクリック  
最後に、**「高速スタートアップを有効にする（推奨）」** のチェックを外し、
**「変更の保存」** を押して終了です。



<div align="center"><img src="/img/ubuntu_install10.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install11.png" width="60%"></div>




### 2. UEFI設定画面の表示

このタイミングで、LiveUSB（Ubuntuが入ったUSB）を挿入してください。

＊＊＊注意＊＊＊   
Ubuntu20.04のLiveUSBを接続後に「フォーマットしますか？」と聞かれるので、「いいえ」を選択してください。   
＊＊＊＊＊＊＊＊
        
UEFI設定画面を表示させる方法は、WindowsでShiftキーを押しながら再起動する。


そうすると、次のような画面が表示される。
        
**「トラブルシューティング」** → **「詳細オプション」** → **「UEFIファームウェアの設定」** 
の順に選択する。すると、PCが再起動すると同時にUEFI設定画面（BIOS）の世界に移動する。



<div align="center"><img src="/img/ubuntu_install12.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install13.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install14.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install15.png" width="60%"></div>




### 3.  BIOS画面を操作する方法

BIOSとは、パソコンに接続されている周辺機器を制御するためのソフトウェアで、ノートパソコン、デスクとトップパソコンなどに必ず搭載されているシステム

BIOSの画面は基本的にキーボードだけで操作します。  
・項目の移動（選択）：カーソルキーの「↑」、「→」、「↓」、「←」、「Tab」  
・選択した項目を決定：「Enter」  
・数値の変更：「-」、「＋」  
・期値に戻す：「F9」  
・終了する：「Esc」  
・保存してから終了する:「F10」


### 4.  Fast BootとSecure Bootの無効化

・Secure Bootとは、コンピュータ起動時の安全性を確保するため、デジタル署名で起動するソフトウェアを検証する機能。  
・Fast Bootとは、PCの電源投入時やWindowsの再起動時に、BIOSレベルでのドライバーの初期化を最小限にすることで、PC起動時のPOSTに要する時間を短縮する機能。

これらの機能を無効（Disabled）にすることが目的です。PC or マザーボードメーカーによって、設定項目の場所が全く異なるので注意して下さい。
でも、やることは同じなので安心してください。

＊＊＊＊＊＊   
BIOSでの設定項目が見つからない場合、BIOSの種類によっては検索欄があるので、検索してみてください。   
＊＊＊＊＊＊


以下に一例を示す。

Secure Boot Enableのチェックを外して、Secure Bootを無効化する。



<div align="center"><img src="/img/ubuntu_install17.png" width="60%"></div>



Fast bootのThoroughにチェックを入れる。Thoroughの状態は、起動中にハードウェア及び設定の完全な初期化を行います。



<div align="center"><img src="/img/ubuntu_install18.png" width="60%"></div>




### 5.  優先順位の変更

続けて、BIBE画面上で、USBメモリから起動させるためにBoot優先度を変更します。  
Ubuntuを上に移動させ、Ubuntuの優先度を上げる。



<div align="center"><img src="/img/ubuntu_install16.png" width="60%"></div>




### 6.  Try Ubuntu without installingを選択


<div align="center"><img src="/img/ubuntu_install22.png" width="60%"></div>

＊＊＊＊＊＊   
Ubuntu20.04をインストールする場合に、「Try Ubuntu without installing」ではなく「Ubuntu」と表示される場合があるので注意してください。   
＊＊＊＊＊＊


**NvidiaのGPUを搭載していないPC**  であれば、「Enter」キーを押してください。

**NvidiaのGPUを搭載したPC**  であれば、「e」キーを押してください。  
「e」キーを押したら、次に、`quiet splash`と書かれた箇所を`nomodeset`に書き換えて、「F10」キーを押します。そしたら、起動が始まります。

```
setparams ‘Try Ubuntu without installing’  
 set gfxpayload=keep  
 linux /casper/vmlinuz.efi file=/cdrom/pressed/ubuntu.seed boot=casper  **quiet splash**  — debian-installer/language=ja keyboard-configuration/layoutcode?=ja keyboard-configuration/modelcode?=jp106  
 initrd /casper/initrd.lz
```

```
setparams ‘Try Ubuntu without installing’  
 set gfxpayload=keep  
 linux /casper/vmlinuz.efi file=/cdrom/pressed/ubuntu.seed boot=casper  **nomodeset**  — debian-installer/language=ja keyboard-configuration/layoutcode?=ja keyboard-configuration/modelcode?=jp106  
 initrd /casper/initrd.lz
```




### 7.  Ubuntuのインストール

デスクトップの  **「Ubuntuをインストール」**  をクリックして、インストール作業を進めます。はじめに、使用言語やキーボードレイアウトを指定します。



<div align="center"><img src="/img/ubuntu_install23.png" width="60%"></div>

<div align="center"><img src="/img/ubuntu_install24.png" width="60%"></div>




次に、アップデートと他のソフトウェアの画面では、  **「通常のインストール」**  を選択します。そして、  **「グラフィックスとWi-Fiハードウェアと追加のメディアフォーマットのサードパーティ製ソフトウェアをインストールする」**  を選択する。



<div align="center"><img src="/img/ubuntu_install25.png" width="60%"></div>




インスト－ルの種類の画面では、「それ以外」を選択します。  
ここで「Windows Boot Managerがインストールされています」と出ない場合は、SSDが認識できていない可能性があるので、AHCI設定を再確認して下さい。


<div align="center"><img src="/img/ubuntu_install34.png" width="60%"></div>


次に、Ubuntuの①インストール先のメイン領域を選択し、②変更を押します。  
「ext4ジャーナリングファイルシステム」を選択し、マウントポイントを「/」に設定します。  
次に、ブートローダをインストールするデバイスを選択します。  
選択するデバイスは、①で選択したパーテーションと同じ名前か、デバイス名となります。（人によって異なる）

設定が済んだら、「インストール」を選択します。


<div align="center"><img src="/img/ubuntu_install33.png" width="60%"></div>


<div align="center"><img src="/img/ubuntu_install40.png" width="60%"></div>


どこに住んでいますか？の画面では、   **「Tokyo」**  を選択します。



<div align="center"><img src="/img/ubuntu_install27.png" width="60%"></div>



あなたの情報を入力してくださいの画面では、研究室のパソコンを借りている場合は、割り振られているPC名と研究室でのパスワードを入力してください。  
私物のパソコンの場合は、自由に設定してください。



<div align="center"><img src="/img/ubuntu_install28.png" width="60%"></div>



ここまでの設定が終わると、インストールが始まるので、終わるまで待機してください。
インストールが終われば、再起動するのではなく、「試用を続ける」を選択しましょう。

次に、Ubuntuが起動するか確認を行うために、シャットダウンを行いましょう。
完全に電源が落ちたことが確認してから、LiveUSBを抜きます。


### 8.  Ubuntuの起動の確認

PCを起動させます。
起動すると、以下の選択画面が表示されます。

<div align="center"><img src="/img/ubuntu_install31.jpg" width="60%"></div>

「Ubuntu」選択して、UbuntuOSを起動させます。
上手く起動できれば、ログインし、インターネットに繋ぎます。

「Ctrl」 + 「Alt」 + 「T」キーを同時に押して、端末を起動させます。
端末上で、`sudo apt update`、`sudo apt upgrade`を実行してOSとソフトウェアを最新に更新します。

``` bash
$ sudo apt update
$ sudo apt upgrade
```

### 9.  時刻同期の設定
以下のコマンドを実行して、時刻同期を行います。

まず、UbuntuとWindowsの時刻の不一致を無くします。
``` bash
$ sudo timedatectl set-local-rtc 1​
```

通信プロトコルであるntpをインストールします。
```bash
$ sudo apt install -y ntp
```

``` bash
$ sudo gedit /etc/ntp.conf

```

18行目から以下の様に表示される。
```
# Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
# on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
# more information.
pool 0.ubuntu.pool.ntp.org iburst
pool 1.ubuntu.pool.ntp.org iburst
pool 2.ubuntu.pool.ntp.org iburst
pool 3.ubuntu.pool.ntp.org iburst

# Use Ubuntu's ntp server as a fallback.
pool ntp.ubuntu.com

```

21〜24行目と27行目の先頭に「＃」でコメントアウト  
29,30行目に「server time.soka.ac.jp」「server ntp.nict.jp」を挿入

```
# Use servers from the NTP Pool Project. Approved by Ubuntu Technical Board
# on 2011-02-08 (LP: #104525). See http://www.pool.ntp.org/join.html for
# more information.
#pool 0.ubuntu.pool.ntp.org iburst　　# コメントアウト
#pool 1.ubuntu.pool.ntp.org iburst    # コメントアウト
#pool 2.ubuntu.pool.ntp.org iburst　　# コメントアウト
#pool 3.ubuntu.pool.ntp.org iburst　　# コメントアウト

# Use Ubuntu's ntp server as a fallback.
#pool ntp.ubuntu.com　　　　　　　　　# コメントアウト

server time.soka.ac.jp　　　　　　　　# 追記
server ntp.nict.jp                    # 追記
```


設定したntpを更新
```bash
$ sudo service ntp restart
```

ntpを確認
```bash
$ ntpq -p
```

以下のように、「time.soka.ac.jp」が表示されれば設定完了です。

<div align="center"><img src="/img/ubuntu_install36.png" width="60%"></div>

なお、これらの時間同期の手順は[time_setup.sh](/install_sh/time_setup.sh)をダウンロードし、以下のコマンドを打つことで自動で設定を行ってくれます。

``` bash
$ bash time_setup.sh
```

以上で、インストール作業を終了です。

---

[トップに戻る](#ubuntuのインストール方法)
