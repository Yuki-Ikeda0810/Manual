# **Gitの使用方法**

## **目次**

1. [**Gitのインストールと設定**](#1-gitのインストールと設定)
    1. [Gitアカウントを作成](#1-gitアカウントを作成)
    2. [Gitのインストール](#2-gitのインストール)
    3. [GitアカウントをホストPCに登録](#3-gitアカウントをホストpcに登録)
    4. [GitHubトークンの作成](#4-githubトークンの作成)

2. [**Gitの基本操作**](#2-gitの基本操作)
    1. [環境セットアップ](#1-環境セットアップ)
    2. [リモートからローカルにリポジトリをダウンロードする](#2-リモートからローカルにリポジトリをダウンロードする)
    3. [リモートの変更をローカルのリポジトリに反映させる](#3-リモートの変更をローカルのリポジトリに反映させる)
    4. [ローカルの変更をリモートのリポジトリに反映させる](#4-ローカルの変更をリモートのリポジトリに反映させる)
    5. [Branchの作成](#5-branchの作成)
    6. [Branchを分岐元のBranchにMerge](#6-branchを分岐元のbranchにmerge)
    7. [他人のリポジトリを個人のリモート環境にforkする](#7-他人のリポジトリを個人のリモート環境にforkする)
    8. [localレポジトリをremoteレポジトリにlocalからあげる方法](#8-localレポジトリをremoteレポジトリにlocalからあげる方法)
    9. [パスワードを毎回打ちたくないそんなあなたへ](#9-パスワードを毎回打ちたくないそんなあなたへ)
    10. [GithubAction](#10-GithubAction)

<br>

## **1. Gitのインストールと設定**

### 1. Gitアカウントを作成

初回のみ，Gitのインストールと設定を行います．
まず，Gitのアカウントを作成します．

:arrow_forward: [GitHubのアカウントを作る](https://github.com/)

### 2. Gitのインストール

アカウントを作成したら，以下のコマンドで，Gitに必要なツール群をインストールします．

``` bash
# Gitのインストール
$ sudo apt install git-all
```

### 3. GitアカウントをホストPCに登録

次に，以下のコマンドで，ホストPCに自身のアカウントを登録します．
予め，Gitに登録したアカウント情報を入力していきます．

``` bash
# Gitアカウントのユーザー名を設定
$ git config --global user.name "ユーザー名"

# Gitアカウントのメールアドレスを設定
$ git config --global user.email "登録したメールアドレス"

# パスワードを一度入力すると1時間(3600秒)パスワードの入力が不要になる設定
$ git config --global credential.helper "cache --timeout=3600"
```

これで，Gitlabのインストールと設定は完了です．
Githubを利用する方は4の作業も行ってください．
    
### 4. GitHubトークンの作成

GitHubの個人用アクセストークン(PAT)は，GitHubAPIまたはコマンドラインを使用する場合に，Githubに対する認証でパスワードの代わりとして使用します．<br> 以下に，トークン作成方法について述べます．<br>
<details><summary>※Githubはこちらをクリック</summary><div>
a. 任意のページで右上隅にあるプロファイルの画像をクリックし，次に[Settings]をクリック<br>
    
<div align="center"><img src="/img/using_git01.png" width="20%"></div>
    
b. 左側のサイドバーで，[Developer Settings]をクリック<br>
    
<div align="center"><img src="/img/using_git02.png" width="30%"></div>
    
c. 左側のサイドバーで，[Personal access tokens]を選択する<br>
    
<div align="center"><img src="/img/using_git03.png" width="30%"></div>
    
d. [Generate new token]をクリック<br>
    
<div align="center"><img src="/img/using_git04.png" width="50%"></div>
    
e. [Note]にトークンの名前をつける<br>
    
<div align="center"><img src="/img/using_git05.png" width="50%"></div>
    
f. トークンに有効期限を設定するには，[Expiration]を選択し，設定を行います<br>
    
<div align="center"><img src="/img/using_git06.png" width="50%"></div>
    
g. このトークンに付与する権限を選択します．ここでは自分の使用用途に合わせて、権限を選択することができます（スコープ詳細）。<br>今回は，[Select scopes]の[repo]関連をチェックをつけます．<br> <br>

![token_g](https://user-images.githubusercontent.com/113363473/192263898-4d689483-f49f-4e9f-9a46-f42403127a90.gif)<br>
h. [Generate token]をクリックします<br>
    
<div align="center"><img src="/img/using_git08.png" width="50%"></div>
    
i. 作成されたトークンを忘れないようにする<br>
<div align="center"><img src="/img/using_git09.png" width="50%"></div>

</div>
</details>

### 参考URL
- <a href="https://qiita.com/KEINOS/items/216d138b0fdf994b9582#github-api-%E3%81%A7%E5%88%A9%E7%94%A8%E5%8F%AF%E8%83%BD%E3%81%AAscope" target="_blank">スコープ詳細</a> <br>
- <a href="https://docs.github.com/ja/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token" target="_blank">個人用アクセス トークンの作成</a>

以上で，Githubのインストールと設定は完了です．これらを行うことで，Gitの様々なツールやサービスが使えるようになります．

※1~3は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．
※4は有効期限やセキュリティの関係上，定期的に行う必要があります．

<br>

## **2. Gitの基本操作**

### 1. 環境セットアップ
現在作業をしているディレクトリをgitで管理できるようにするためのセットアップを行います．このコマンドは，管理したいディレクトリごとに最初に一回のみ行ってください．

```bash
# ディレクトリをgitで管理するためのセットアップ
$ git init
```

### 2. リモートからローカルにリポジトリをダウンロードする
以下のコマンドで，Cloud上にアップされているリモートリポジトリを，ローカルに複製する形でダウンロードします．
引数には，ダウンロードしたいリポジトリの「https」から始まるURLを入力してください．
図は，リモートからローカルにリポジトリをダウンロードするイメージになります

```bash
# 任意のディレクトリまで移動
$ cd ディレクトリ
# リモートからローカルにリポジトリをダウンロード
$ git clone https://○○○○.git

Username: ユーザー名
Password: トークン
```

<div align="center"><img src="/img/using_git10.png" width="45%"></div>

### 3. リモートの変更をローカルのリポジトリに反映させる

以下のコマンドで，Cloud上のリモートのリポジトリに変更(更新)があった場合に，その変更(更新)をローカルのリポジトリに反映させることができます．
図は，リモートの変更をローカルのリポジトリに反映させるイメージになります．

```bash
# リモートの変更をローカルのリポジトリに反映
$ git pull
```

<div align="center"><img src="/img/using_git11.png" width="45%"></div>

※ 2回以上の変更があった場合，Pullすることができなくなります．
一度，Cloneしたリポジトリを削除し，再度Cloneし直す必要があります．
チームで開発をする場合，このようなことが頻繁に起きてしまうため，後述するBranchという機能を駆使することでこれらを防ぎます．
以下のリンクから確認してください．

:arrow_forward: [4. Branchの作成](#4-branchの作成)

### 4. ローカルの変更をリモートのリポジトリに反映させる

以下のコマンドで，ローカルのリポジトリで変更(更新)があった場合に，その変更(更新)をCloud上のリモートのリポジトリに反映させることができます．
Pushコマンドの時に登場する「Branch」については，「[4. Branchの作成](#4-branchの作成)」にて説明します．
図は，ローカルの変更をリモートのリポジトリに反映させるイメージになります．

```bash
# リモートの変更をローカルのリポジトリに反映(必ず最新のリポジトリにしておく)
$ git pull

# リモートのリポジトリに反映させるファイルを指定する
## (ファイルを１つずつあげる場合)
$ git add ファイル名

## (ファイルを一括であげる場合 → 参考サイト：ファイルを一括であげる)
$ git add 一番深い階層のファイル
$ git add -A

# 変更(更新)がどのようなものなのか誰でもわかるように記述する
$ git commit -m "コメント内容"

# 初めてリモートのリポジトリにアップロードを行う場合はこのコマンドを打つ
$ git remote add origin https://○○○○.git

# 変更(更新)した点をリモートリポジトリ(指定のBranch)に反映させる
## (pushが初めての場合)
$ git push -u origin ブランチ名

## (pushが2回目以降の場合)
$ git push

## もしくは
$ git push origin Branch名
```

<div align="center"><img src="/img/using_git12.png" width="45%"></div>

コメントを記述する際の命名規則などは，以下のリンクから確認してください．

:arrow_forward: <a href="https://github.com/TeamSOBITS/sobits_manual/blob/main/docs/git_style.md" target="_blank">Git命名規則</a>

### 参考URL
- <a href="https://qiita.com/A__Matsuda/items/f71a935612a55d6e674e" target="_blank">Git Push</a> <br>
- <a href="https://qiita.com/fuwakun/items/d2ea19bf43eda3df0094" target="_blank">ファイルを一括であげる</a> <br>

### 5. Branchの作成

チームで開発を進めていく場合，Branchという機能を駆使する必要があります．
Branchとは，リポジトリの変更(更新)の履歴の流れを分岐して記録していくためのものです．
Gitを使い，チームで開発する上でとても大切な機能になるので，しっかりと理解して使いこなしてください．
分岐したBranchは，他のBranchの影響を受けないため，同じリポジトリ中で複数の変更を同時に進めていくことができます．
また，分岐したBranchは，他のBranchと合流(Merge)することで，1つのBranchにまとめ直すことが出来ます．

#### 5-1. ローカルでのBranchの基本操作

以下のコマンドで，ローカルのリポジトリが現在どのBranchにいるのか確認ができます．

```bash
# 現在のBranchを確認
$ git branch
```

また，以下のコマンドで，ローカルにあるBranchを削除することができます．

```bash
# Branchをローカルから削除
$ git branch -d Branch名

# Branchをローカルから削除
$ git branch -delete Branch名
```

#### 5-2. リモートからBranchを作成

リモートからBranchを作成する方法について説明します．
図は，リモートでBranchを作成し，それをローカルに反映させるイメージになります．

<div align="center"><img src="/img/using_git13.png" width="35%"></div>

<details><summary>※Githubはこちらをクリック</summary><div>
リモートからBranchを作成する場合は，Web上で作業を行います．
以下の図のように，「View all branches」➔ 「New branch」の順でクリックしてください．

<div align="center"><img src="/img/using_git14.png" width="60%"></div>

<div align="center"><img src="/img/using_git15.png" width="60%"></div>

その後，作成するBranch名と作成するBranchの分岐元となるBranchを指定し，「Create branch」をクリックしてください．
これで，リモートからBranchを作成することができます．

<div align="center"><img src="/img/using_git16.png" width="60%"></div>

※ リポジトリのトップ画面で，「Branch名」を選択すると，現在作成されているBranch名が表示されます．
ここで，確認したいBranch名をクリックすると，リポジトリの表示がクリックしたBranchに切り替わります．

ここまででの操作で，リモートのリポジトリにBranchが作成することができました．
リモートで作成したBranchをローカルに反映させるためには，以下のコマンドを入力します．

```bash
# リモートの変更をローカルのリポジトリに反映
$ git pull

# Branchをリモートからローカルにダウンロード
$ git fetch origin Branch名

# 指定したBranchに切り替え
$ git checkout Branch名
```
</div>
</details>


<details><summary>※Gitlabはこちらをクリック</summary><div>

リモートからBranchを作成する場合は，Web上で作業を行います．
以下の図のように，「+」を選択し，「New branch」をクリックしてください．

<div align="center"><img src="/img/using_git17.png" width="80%"></div>

その後，作成するBranch名と作成するBranchの分岐元となるBranchを指定し，「Create branch」をクリックしてください．
これで，リモートからBranchを作成することができます．

<div align="center"><img src="/img/using_git18.png" width="80%"></div>

※ リポジトリのトップ画面で，「Branch名」を選択すると，現在作成されているBranch名が表示されます．
ここで，確認したいBranch名をクリックすると，リポジトリの表示がクリックしたBranchに切り替わります．

ここまででの操作で，リモートのリポジトリにBranchが作成することができました．
リモートで作成したBranchをローカルに反映させるためには，以下のコマンドを入力します．

```bash
# リモートの変更をローカルのリポジトリに反映
$ git pull

# Branchをリモートからローカルにダウンロード
$ git fetch origin Branch名

# 指定したBranchに切り替え
$ git checkout Branch名
```
</div>
</details>
    
以上で，リモートでBranchを作成し，それをローカルに反映させることができます．

Branch上で，リモートの変更をローカルのリポジトリに反映させる方法は，前述した方法と同じです．
以下のリンクから確認してください．

:arrow_forward: [3. リモートの変更をローカルのリポジトリに反映させる](#3-リモートの変更をローカルのリポジトリに反映させる)

Branch上で，ローカルの変更をリモートのリポジトリに反映させる方法は，前述した方法と同じです．
以下のリンクから確認してください．

:arrow_forward: [4. ローカルの変更をリモートのリポジトリに反映させる](#4-ローカルの変更をリモートのリポジトリに反映させる)

Branchを作成する際の命名規則などは，以下のリンクから確認してください．

:arrow_forward: <a href="https://github.com/TeamSOBITS/sobits_manual/blob/git_devel/docs/git_style.md" target="_blank">Git命名規則</a>

#### 5-3. ローカルからBranchを作成

ローカルからBranchを作成する方法について説明します．
図は，ローカルでBranchを作成し，それをリモートに反映させるイメージになります．

<div align="center"><img src="/img/using_git19.png" width="45%"></div>

以下のコマンドで，ローカルからBranchを作成することができます．

```bash
# Branchをローカルに作成
$ git branch Branch名

# 指定したBranchに切り替え
$ git checkout Branch名

# 新しいBranchを作成して切り替え
$ git checkout -b Branch名
```

ここまででの操作で，ローカルのリポジトリにBranchが作成することができました．
ローカルで作成したBranchをリモートに反映させるためには，以下のコマンドを入力します．

```bash
# リモートのリポジトリに反映させるファイルを指定する
$ git add ファイル名

# 変更(更新)がどのようなものなのか誰でもわかるように記述する
$ git commit -m "コメント内容"

# 変更(更新)した点をリモートリポジトリ(指定のBranch)に反映させる
$ git push origin Branch名
```

以上で，ローカルでBranchを作成し，それをリモートに反映させることができます．

Branch上で，リモートの変更をローカルのリポジトリに反映させる方法は，前述した方法と同じです．
以下のリンクから確認してください．

:arrow_forward: [2. リモートの変更をローカルのリポジトリに反映させる](#2-リモートの変更をローカルのリポジトリに反映させる)

Branch上で，ローカルの変更をリモートのリポジトリに反映させる方法は，前述した方法と同じです．
以下のリンクから確認してください．

:arrow_forward: [3. ローカルの変更をリモートのリポジトリに反映させる](#3-ローカルの変更をリモートのリポジトリに反映させる)

Branchを作成する際の命名規則などは，以下のリンクから確認してください．

:arrow_forward: <a href="https://github.com/TeamSOBITS/sobits_manual/-/blob/main/docs/git_style.md" target="_blank">Git命名規則</a>


#### 参考5-1URL

- <a href="https://qiita.com/chihiro/items/e178e45a7fd5a2fb4599" target="_blank">git branch コマンド</a> <br>

### 6. Branchを分岐元のBranchにMerge

Branchを分岐元のBranchにMergeする方法について説明します．
図は，ローカルでBranchを変更(更新)し，それをリモートに反映させた後，作成したBranchをメインのBranchにMergeしたイメージになります．

<div align="center"><img src="/img/using_git20.png" width="50%"></div>

Branchを使いこなすためには，Mergeという機能を駆使する必要があります．
Mergeとは，Gitにおいて分岐した履歴を戻して統合する手段のことを言います．

BranchをMerge(統合)することは，今まで開発してきたプログラムを大きく変化させることになります．
そのため，Mergeする際は，必ずOwnerの権限が必要になります．
ここでは，OwnerにMergeのリクエストを送信するまでの手順を記載しています．


<details><summary>※Githubはこちらをクリック</summary><div>

基本的には，リモートのリポジトリに対してMergeを行うため，これらの作業はWeb上で行います．
以下の図のように，「Compare & pull request」をクリックしてください．

<div align="center"><img src="/img/using_git21.png" width="55%"></div>

次に，以下の図のように，Mergeの材料となるBranchとMerge後にメインとなるBranchを指定し，Mergeリクエストのタイトル，Mergeの概要を入力後に「Create pull request」をクリックしてください．

<div align="center"><img src="/img/using_git22.png" width="60%"></div>

これで，Mergeを行うためのリクエストを作成することができます．
その後，Ownerによる評価が完了し，許可が出されると，BranchがMergeされます．

Ownerは「pull request」からMergeしたい「pull request」の「Merge pull request」➔「Confirm merge」の順でクリックします．
これでMergeは完了です．
※この処理はOwnerのみ実行可能です．

<div align="center"><img src="/img/using_git23.png" width="60%"></div>

<div align="center"><img src="/img/using_git24.png" width="60%"></div>

※ このとき，特にMainとなるBranchをMergeした場合は，slackの「git hub」チャンネルに必ずアナウンスしてください
</div>
</details>

<details><summary>※Gitlabはこちらをクリック</summary>
<div>

基本的には，リモートのリポジトリに対してMergeを行うため，これらの作業はWeb上で行います．
以下の図のように，「Merge requests」を選択し，「New merge request」をクリックしてください．

<div align="center"><img src="/img/using_git25.png" width="80%"></div>

次に，以下の図のように，Mergeの材料となるBranchとMerge後にメインとなるBranchを指定し，「Compare branches and continue」をクリックしてください．
これで，Mergeを行うためのリクエストを作成する画面へ移ることができます．

<div align="center"><img src="/img/using_git26.png" width="80%"></div>

そして，以下の図のように，Mergeリクエストのタイトル，Mergeの概要，Mergeの許可をリクエストするOwnerを選択，Merge後に材料となるBranchを削除するかどうか(基本は削除)の入力を行い，「Create merge request」をクリックする．
これで，Mergeを行うためのリクエストを作成することができます．
その後，Ownerによる評価が完了し，許可が出されると，BranchがMergeされます．

※ このとき，特にMainとなるBranchをMergeした場合は，slackの「git hub」チャンネルに必ずアナウンスしてください

<div align="center"><img src="/img/using_git27.png" width="80%"></div>
    
</div>
</details>
    
BranchのMergeは，複数人が同じファイルを編集しない限り，複数人で別々のMergeを行っても，メインとなるBranchを保つことができます．
そのため，1つのリポジトリに対しての様々な要素技術の開発を，複数人で手分けして行うことができます．
以下の図は，複数人で1つのリポジトリの開発を行っているイメージになります．
このように，BranchやMergeを駆使することで，以下の図のような複数のローカル環境での開発が可能になります．

<div align="center"><img src="/img/using_git28.png" width="55%"></div>

#### 参考6-1URL

- <a href="https://docs.github.com/ja/pull-requests/collaborating-with-pull-requests/incorporating-changes-from-a-pull-request/merging-a-pull-request" target="_blank">git pull要求のマージ</a> <br>

### 7. 他人のリポジトリを個人のリモート環境にforkする
Forkを用いることで他人やチームが管理しているリポジトリを自身のリモート環境にコピーすることができます．自身のリモート環境にコピーすることで，他人やチームの環境に影響を与えることなく開発を行うことができます．
また，管理者が自身となるため，自身の環境においてMergeなどの作業が容易に行えるようになります．元のリポジトリに投稿するには，Pull Requestを送信することで元のリポジトリの管理者に投稿要求を送信することができます．
Githubのみ記載していますが，Gitlabでも同様にして利用可能です．

<div align="center"><img src="/img/using_git29.png" width="40%"></div>

<details><summary>※Githubはこちらをクリック</summary><div>
    
Fork元となるリポジトリの「Fork」をクリックすることで自身の環境にFork作成画面に移動します．

<div align="center"><img src="/img/using_git30.png" width="60%"></div>

作成先のOwner，リポジトリ名を選択し，「Copy the main branch only」のチェックを外したのちに「Create fork」をクリックします．

<div align="center"><img src="/img/using_git31.png" width="60%"></div>

これでForkは完了です．

Forkは大元のリポジトリに還元することを前提としている機能であり，開発した内容を開発元に提供するのが一般的です．
Fork先の変更をFork元にMergeする手順を以下に示します．
「Pull request」を選択し「New pull request」をクリックします．

<div align="center"><img src="/img/using_git32.png" width="60%"></div>
<div align="center"><img src="/img/using_git33.png" width="60%"></div>

開発した自身のBranchとMerge先のBranchを確認したのちに「View pull request」をクリックします．

<div align="center"><img src="/img/using_git34.png" width="50%"></div>
</div>
</details>

これでFork元のOwnerにPull Requestが送信されます．

#### 参考7-1URL

- <a href="https://docs.github.com/ja/get-started/quickstart/fork-a-repo" target="_blank">git リポジトリをフォークする</a> <br>

### 8. localレポジトリをremoteレポジトリにlocalからあげる方法
また、gitに上げていないlocalレポジトリをgithubにlocalで上げる方法をご紹介します。基本的には、github内で作ってcloneしてそこにコピペしていただくほうが早いと思いますので、あくまで参考として見てください。

```bash
# 手順１github似あげたいレポジトリに入って./git設定
$ cd ~/自身のレポジトリ
$ git init

手順２コミット
$git add -A
$git commit -m "first commit"
#Githubでレポジトリを作成しURLをコピーします。
$git remote add origin 作成したgithubレポジトリのURL
#リモートレポジトリにアップロード
$git push origin master

```

#### 参考8-1URL

- <a href="https://atmarkit.itmedia.co.jp/ait/articles/1701/24/news141.html" target="_blank">Git／GitHubにおけるリモートリポジトリの作成、確認、変更、更新時の基本5コマンド</a> <br>

### 9. パスワードを毎回打ちたくないそんなあなたへ
``` bash
"""ファイルに永久保存します。ただし、パスワードが平文が保存されます。
“store” モードにすると、認証情報がテキストファイルでディスクに保存されます。有効期限はありません。 パスワードを変更するまで、認証情報を記憶します。
ただし、パスワードが暗号化なしのテキストファイルでホームディレクトリ（指定すれば変更可能）に保存される。"""

$ git config --global credential.helper store --保存先<file path>

# 常駐プロセス（メモリ）に記憶させます。（デフォルト15分）
$ git credential-cache

# 指定した時間パスワードを記憶させます。再起動をすると消えます。
$ git config --global credential.helper "cache --timeout=秒数"

# Mac OS X のパスワード管理を使います。
$ git-credential-osxkeychain

# Windows のパスワード管理を使います。
$ git-Credential-Manager-for-Windows

```

#### 9-1.参考URL

- <a href="https://hawksnowlog.blogspot.com/2018/10/try-git-credential.html" target="_blank">git credential を使おう</a> <br>
- <a href="https://git-scm.com/book/ja/v2" target="_blank">Git Book</a> <br>

### 10. GithubAction
現在設定中のため、随時更新していきます。
概要
GitHub Actionsは、ほかのCI/CDツールと同様、リポジトリに対するプッシュやプルリクエストといった操作、もしくは指定した時刻になるといったイベントをトリガーとして、あらかじめ定義しておいた処理を実行する機能を持つ。
たとえばリポジトリにコミットが行われた際に特定の処理を実行したり、毎日決まった時刻に特定の処理を実行したりする、といったことができる。
これらの処理はGitHubが提供するサーバー上に用意された仮想マシン内で実行できるため、ユーザーが独自にサーバーなどを準備する必要はない。

利用できる仮想マシン
Linux（Ubuntu）およびWindows、macOSに対応
利用できる言語
さまざまな言語のコンパイラや各種ランタイム、主要ライブラリといったソフトウェア開発環境も標準でインストールされている。

#### 10-1.参考URL

- <a href="https://knowledge.sakura.ad.jp/23478/#GitHubCICDGitHub_Actions" target="_blank">GitHubの新機能「GitHub Actions」で試すCI/CD</a> <br>

---

[トップに戻る](#gitの使用方法)
