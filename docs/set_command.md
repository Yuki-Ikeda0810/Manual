# **便利なショートカットコマンドの設定**

開発でよく使うコマンドのショートカットを設定しておくとを便利です．
.bashrcに，下記を記述してみてください．

``` bash
# どこの階層からでも「$ catkin_make」を行えるショートカットコマンド
alias cm='cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 && cd ~  '
```

``` bash
# 「$ rosclean purge」を行うショートカットコマンド
alias rc='rosclean purge '
```

``` bash
# ○○
alias mc='echo 3 > sudo /proc/sys/vm/drop_caches'
```

``` bash
# ○○
alias mc2='sudo sysctl -w vm.drop_caches=3'
```

``` bash
# ○○
alias cmpkg='f(){ cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -j4 --pkg  "$@" && cd ~ ;  unset -f f; }; f'
```

``` bash
# 「$ chmod 755」をすべてのpythonプログラムに対して行うショートカットコマンド
# ただし，これには専用のプログラムが必要
alias ch='cd ~/sobits-setup && ./chmod_all.py && cd ~  '
```

``` bash
# 更新のある全てのリポジトリに対して「$ git pull」を行うショートカットコマンド
# ただし，これには専用のプログラムが必要
alias gpa='cd && python git_pull_all/git_pull_all.py'
```
