# **シリアルポート(デバイス)登録の設定**

rootアカウントやsudoを使用せずシリアルポートを使用する方法になります．

## **目次**
1. [設定方法](#1-設定方法)
    1. [事前準備](#1-事前準備)
    2. [設定ファイルの準備](#2-設定ファイルの準備)
    3. [設定の反映](#3-設定の反映)

<br>

## **1. 設定方法**

### 1. 事前準備

まず，必要最低限のシリアルポート(デバイス)のみを接続していきます (デバイス情報見る際、端末への出力が多くなるのを避けるため)．
その後，端末で下記のコマンドを入力し，デバイスの識別子(ID)確認を行います．

```bash
$ dmesg
```

この際，端末では下記のようなものが出力されます．
現在のシリアルポート名は「ttyUSB0」として表示され，ここにデバイスが接続されていることが確認できます．

```bash
[ 4179.797128] [Firmware Bug]: battery: (dis)charge rate invalid.
[ 4180.053980] ahci 0000:00:1f.2: port does not support device sleep
[ 6029.436494] usb 2-8: new full-speed USB device number 13 using xhci_hcd
[ 6029.459976] usb 2-8: New USB device found, idVendor=165c, idProduct=0008
[ 6029.459986] usb 2-8: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[ 6029.459991] usb 2-8: Product: DUAL USB ADAPTER HS
[ 6029.459995] usb 2-8: Manufacturer: Kondo Kagaku
[ 6029.459999] usb 2-8: SerialNumber: KOUSBCNV
[ 6029.463499] ftdi_sio 2-8:1.0: FTDI USB Serial Device converter detected
[ 6029.463592] usb 2-8: Detected FT232RL
[ 6029.463597] usb 2-8: Number of endpoints 2
[ 6029.463601] usb 2-8: Endpoint 1 MaxPacketSize 64
[ 6029.463605] usb 2-8: Endpoint 2 MaxPacketSize 64
[ 6029.463608] usb 2-8: Setting MaxPacketSize 64
[ 6029.463887] usb 2-8: FTDI USB Serial Device converter now attached to ttyUSB0（←現在のシリアルポート名）
```

そして，udev に現在のシリアルポート名でデバイスの情報を問い合わせていきます．

```bash
$ udevadm info -a -n /dev/ttyUSB0
```

この際，端末では下記のようなものが出力されます．
この情報を次の[2. 設定ファイルの準備](#2-設定ファイルの準備)にて使っていきます．

```bash
　　　　　　〜（省略
   ATTRS{idVendor}=="165c"
　　　　　　〜（省略）
   ATTRS{serial}=="KOUSBCNV"
　　　　　　〜（省略）
   ATTRS{idProduct}=="0008"
　　　　　　〜（省略）
```

### 2. 設定ファイルの準備

端末で下記のコマンドを入力し，設定ファイルを作成します．

```bash
$ sudo gedit /etc/udev/rules.d/ファイル名.rules
```

そして，作成した設定ファイルの中で，下記のの6項目を記述していきます．
この際，デバイスによっては「ATTRS{serial}」がない場合があるので注意が必要です．

```bash
・SUBSYSTEM=="tty"
  - このまま記述します．

・ATTRS{idVendor}
  - 事前準備で行った「$ udevadm info -a -n /dev/ttyUSB0」で問い合わせた値を「==」の後に記述します．

・ATTRS{idProduct}
  - 事前準備で行った「$ udevadm info -a -n /dev/ttyUSB0」で問い合わせた値を「==」の後に記述します．

・ATTRS{serial}
  - 事前準備で行った「$ udevadm info -a -n /dev/ttyUSB0」で問い合わせた値を「==」の後に記述します．

・SYMLINK
  - 自分が設定したいシリアルポート名を「+=」の後に記述します．

・MODE="0666"
  - このまま記述します．
```

<details><summary>※ 設定ファイルの記述例はこちらをクリック</summary>
<br>
  
```bash
＜記述例1＞
SUBSYSTEM=="tty", ATTRS{idVendor}=="165c", ATTRS{idProduct}=="0008", ATTRS{serial}=="KOUSBCNV", SYMLINK+="kondo", MODE="0666"
```

```bash
＜記述例2＞
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", SYMLINK+="dynamixel", MODE="0666"
```

```bash
＜記述例3＞
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="kobuki_AI02MP57", SYMLINK+="kobuki", MODE="0666"
```

```bash
＜記述例4＞
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="kobuki_AI02MW2H", SYMLINK+="sobit", MODE="0666"
```

</details>

### 3. 設定の反映

設定ファイルを作成したら，早速この設定を反映させていきます．
端末で下記のコマンドを入力し，設定を反映させます．

```bash
$ sudo /etc/init.d/udev reload
```

その後，デバイスを抜いて新しい端末を開き，デバイスを挿し直します．
ここで，自分が設定したシリアルポートの設定が反映されているか確認します．
端末で下記のコマンドを入力し，確認します．

```bash
$ ls /dev/シリアルポート名
```

しっかりとシリアルポート(デバイス)の登録ができていれば，青文字で「/dev/シリアルポート名」の表示が出てきます．

※ このような表示が出ていない場合は，「idVendor」や「idProduct」が正しくない可能性あります．
再度[1. 事前準備](#1-事前準備)の手順でIDを確認してみてください．
その後「$ udevadm info -a -n /dev/ttyUSB0」で表示された他のIDに書き換えてみてください．

---

[トップに戻る](#シリアルポート(デバイス)登録の設定)
