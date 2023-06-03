# **Gitの命名規則**

SOBITSのGitの命名規則では，BranchやCommitの命名する規則や作り方等を以下に示すように統一しています．
チームとしての開発効率も向上させるために，SOBITSとしてのGitの命名規則を守り，誰が見ても分かりやすいように作業するよう心掛けてください．

## **目次**

1. [**Branchの命名規則**](#branchの命名規則)
    1. [ROSパッケージではないリポジトリ](#rosパッケージではないリポジトリ)
    2. [ROSパッケージのリポジトリ](#rosパッケージのリポジトリ)
    3. [参考URL(Branch)](#参考urlbranch)

---

2. [**Commitの命名規則**](#commitの命名規則)
    1. [Commit参考URL](#参考urlcommit)

## **Branchの命名規則**

### ROSパッケージではないリポジトリ

| 命名種類 | 命名規則 |
----|----
| 安定版(旧Git Ver.) | main |
| 安定版(新Git Ver.) | master |
| 安定版の開発 | develop |
| 新規機能の開発 | feature/作業内容 |
| バグ等の修正 | hotfix/作業内容 |

### ROSパッケージのリポジトリ

| 命名種類 | 命名規則 |
----|----
| 安定版 | ROSのバージョン(kinetic, melodicなど)-devel |
| 安定版の開発 | develop |
| 特定のロボットでの試作 | trial/ロボット名 |
| 特定のRoboCup競技での試作 | trial/RoboCup競技名 |
| 新規機能の開発 | feature/作業内容 |
| バグ等の修正 | hotfix/作業内容 |

### 参考URL(Branch)
- [Git/GitHub branching standards & conventions](https://gist.github.com/digitaljhelms/4287848)
- [ブランチ運用ルール](https://gist.github.com/minop1205/b87a1f5ffab6d8af28a66aca64378171)
- [git flowの実際的運用方法](https://qiita.com/kanatatsu64/items/8feb5bf0352d39cfa3c3)
- [Gitのブランチモデルについて](https://www.tam-tam.co.jp/tipsnote/program/post16686.html)

## **Commit時の命名規則**

| 命名種類 | 命名規則 |
----|----
| バグ修正 | fix |
| 新規(ファイル)機能追加 | add |
| 機能修正(バグでない) | update |
| 削除(ファイル) | remove |

### 参考URL(Commit)
- [Gitのコミットメッセージの書き方](https://qiita.com/itosho/items/9565c6ad2ffc24c09364)

---

[トップに戻る](#gitの命名規則)
