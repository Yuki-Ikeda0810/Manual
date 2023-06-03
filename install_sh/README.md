# SHファイルについて

環境構築を簡易化するために、SOBITSとしてこのレポジトリに存在するスクリプトファイル用意しました。
開発や研究のスピードアップさせるためのツールものの、エンジニアとして避けてならないプロセスだと思われるため、自分の手で一度もセットアップしたことなければ、その使用をできるだけ回避することをおすすめです。

それではそれぞれのファイルについて説明する。

## スクリプト(sh)ファイルとは

このようなファイルは基本的に順番の変わらない作業の自動化を対照としたコマンド群であり、そのファイルを呼び出した時、書かれているコマンドの順番通り1個ずつ実行できます。
`shell script`言語もしくは、最近の`bash script`言語に従い、プログラミングの基本的な変数、関数、if文なども定義できます。
その組み合わせによって、セットアップの自動化のためのファイルが造れます。是非、興味がある方も自分で作成してみてください！


<details><summary>shのついてのサイト</summary>
<p>

- [Bash Scriptの作法](https://qiita.com/autotaker1984/items/bc758fcf368c1a167353)

</p>
</details>

## インストール可能な環境について

install_shのフォルダの中から、SOBITSとしてよく使われるスクリプトファイルを用意しています。
ここになければ、自分で用意していただくと、次の使う人に役に立つでしょう！

まず、レポジトリをcloneしましょう。
```bash
$ git clone https://github.com/TeamSOBITS/sobits_manual
```

> **Note**
> install_sh以外の内容（マニュアル等）と一緒にダウンロードされますので、少々時間かかります。

次に、`install_sh`へ移動します。
```bash
$ cd sobits_manual/install_sh
```

それから、自分の求める環境に応じて、ファイルと選択します。

### CUDA
機械学習に必要なCUDAをインストールするのに、NVIDIAのGPU付きのデバイスのみ対応しています。

<details><summary>対応しているCUDAのバージョン</summary>
<p>

- 11.8.0
- 11.7.1
- 11.6.2
- 11.6.1
- 11.6.0
- 11.3.0

</p>
</details>


#### インストール方法
CUDAを選択する場合、スクリプトファイルを実行する前に、いくつのステップを進めなければなりませんので、ご注意ください。

1. NVIDIAの最新ドライバを選択してください。

    - Ubuntuの「`ソフトウェアとアップデート`」というプログラムを起動してください。
    - その中の「`追加のドライバー`」へ移動します。
    - NVIDIA Corporationのドライバーが複数並んでおり、「`NVIDIA driver metapackageをnvidia-driver-XXXから使用します`」を選択して、「`変更の適用`」ボタンを押してください。
    - 適応が終わったら、パソコンを再起動します。

> **Note**
> 「`nvidia-driver-XXX`」は最新のドライバを表していますので、番号の高い値を選択してください。

2. スクリプトを実行します（1回目）

- CUDAに必要なCuDNNを確認するため、試行として1回`install_cuda.sh`を実行します。

```bash
$ bash install_cuda.sh --{cuda_version}
```
- それから、必要なCuDNNのバーションが表示され、ダウンロードするためのサイトリンクを[Ctrl]+[Click]で移動できます。

> **Note**
> CuDNNをダウンロードするのに、NVIDIAのアカウントが必要です。なければ、個人メールアドレスで作成してください。

> **Note**
> CuDNNの最新版出ない場合、アーカイブのサイトから選択してください。
> [cuDNN Archive](https://developer.nvidia.com/rdp/cudnn-archive)

- そこで、自分のパソコンの環境に合わせて、インストラを選択します。今回は`Ubuntu20.04`を例として、「`Local Installer for Ubuntu20.04 x86_64 (Deb)`」を選択します。ただし、aarch64というSBCアーキテクチャの場合、「`Local Installer for Ubuntu20.04 aarch64sbsa (Deb)`」を選択します。

- ダウンロードされた、`.debファイル`を`install_sh`の中に作成された`cudnn`フォルダへ移動します。

3. スクリプトを実行します（2回目）
```bash
$ bash install_cuda.sh --{cuda_version}
```
> **Warning**
> 1回目に記入した`cuda_version`を使用してください！

4. セットアップが完了されたら、パソコンを再起動してください。


### OpenCV
画像処理によく使われるライブラリとしてOpenCVが取り上げられます。
このshファイルはCPUのみやGPU付きに対応しています。

<details><summary>対応しているOpenCVのバージョン</summary>
<p>

- 4.6.0
- 4.5.5

> **Note**
> その他のバージョンもインストールできるが、動作確認が済ましていないため、自己責任で実行してください。

</p>
</details>

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash install_opencv.sh --{opencv_version} --{arch}
```
> **Note**
> `{arch}`では`cpu`もしくは`gpu`というフラグを記入します


### PyTorch
機械学習によく使われるネットワーク構築のためのライブラリとしてPyTorchが取り上げられます。
このshファイルはGPU付きのみに対応しています。

<details><summary>対応しているPyTorchのバージョン</summary>
<p>

- 1.13.1
- 1.13.0
- 1.12.0
- 1.11.0
- 1.10.0
- 1.9.1
- 1.8.2
- 1.7.1

> **Note**
> その他のバージョンもインストールできるが、動作確認が済ましていないため、自己責任で実行してください。

</p>
</details>

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash install_pytorch.sh --{pytorch_version}
```


### ROS1
ロボットを動かすためのmiddlewareとして、ROS1がよく使われています。

<details><summary>対応しているROS1のディストリビューション</summary>
<p>

- noetic
- melodic
- indigo
- kinetic

</p>
</details>

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash install_ros1_.sh --{ros1_distro}
```

2. ワークスペースを作成します。
```bash
$ bash install_ros1_catkin_ws.sh --{ros1_distro}
```

3. SOBITSとしてよく使われるROS1のパッケージをインストールします。
```bash
$ bash install_ros1_sobits.sh --{ros1_distro}
```

4. 基本的なセットアップ（ネットワーク等）を行います。
```bash
$ bash setup_ros1.sh --{ros1_distro}
```

> **Warning**
> 1回目に記入した`ros1_distro`を使用してください！


### ROS2
ロボットを動かすためのmiddlewareとして、ROS2の使用が普及しています。

<details><summary>対応しているROS2のディストリビューション</summary>
<p>

- humble
- galactic
- foxy


</p>
</details>

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash install_ros2_.sh --{ros2_distro}
```

2. ワークスペースを作成します。
```bash
$ bash install_ros2_colcon_ws.sh --{ros2_distro}
```

3. 基本的なセットアップ（ネットワーク等）を行います。
```bash
$ bash setup_ros2.sh --{ros2_distro}
```

> **Warning**
> 1回目に記入した`ros2_distro`を使用してください！


### Alias
入力するコマンドを単純化するため、アリアス（alias）を提議します。

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash setup_alias.sh
```


### 時刻設定
WindowsとUbuntuのOSを切り替えるたびに、時間のズレが生じます。そこで、創価大学のIPに合わせ、同期をさせることで解決できます。

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash setup_time.sh
```


### Google Chrome（Docker用）
Google Chromeが自動的にインストールするために、Dockerfileに使っています。

#### インストール方法
1. スクリプトを実行します。
```bash
$ bash install_google_chrome.sh
```