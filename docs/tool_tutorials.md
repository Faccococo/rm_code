- [Linux命令](#linux命令)
  - [1.文件查看](#1文件查看)
  - [2.文件内容筛选](#2文件内容筛选)
    - [(1)cut 用指定规制切分文本](#1cut-用指定规制切分文本)
    - [(2) sort 对文件内容进行排序](#2-sort-对文件内容进行排序)
    - [（3）wc 统计单词的数量](#3wc-统计单词的数量)
  - [3.文件内容增删改查](#3文件内容增删改查)
    - [(1) grep](#1-grep)
    - [(2) sed](#2-sed)
    - [(3) awk](#3-awk)
- [Vim 命令](#vim-命令)
  - [1.普通模式](#1普通模式)
  - [2. 进入编辑模式](#2-进入编辑模式)
  - [3. 编辑模式](#3-编辑模式)
- [git命令](#git命令)
  - [1. 基本概念](#1-基本概念)
    - [（1）四个区](#1四个区)
  - [2. 初始化配置](#2-初始化配置)
  - [3. git 分支操作](#3-git-分支操作)
  - [4. git 删除撤销操作](#4-git-删除撤销操作)
- [docker](#docker)
  - [1. docker 安装](#1-docker-安装)
  - [2. 配置阿里云镜像加速](#2-配置阿里云镜像加速)
  - [3. docker常用命令](#3-docker常用命令)
  - [docker启动](#docker启动)
    - [镜像命令](#镜像命令)
    - [容器命令](#容器命令)
    - [容器数据卷](#容器数据卷)
    - [dockerfile](#dockerfile)
- [remote debug setting](#remote-debug-setting)
  - [1. Install ssh server](#1-install-ssh-server)
  - [2. vscode](#2-vscode)
  - [4.build and run](#4build-and-run)
  - [3. httpServer visualization](#3-httpserver-visualization)

# Linux命令

## 1.文件查看

|         常用命令          |             作用             |
| :-----------------------: | :--------------------------: |
|        stat file1         |       查看文件详细属性       |
|         cat file1         |         查看⽂件内容          |
|       cat -n file1        |      查看内容并标示⾏数       |
|         tac file1         |   从最后⼀⾏开始反看⽂件内容    |
|        more file1         |    一页一页的显示文件内容    |
|        less file1         | 类似more命令，但允许反向操作 |
|       head -2 file1       |         查看⽂件前两⾏         |
|       tail -2 file1       |         查看⽂件后两⾏         |
| head -20 file1 \| tail -5 |     查看文件第15行到20行     |

## 2.文件内容筛选

###  (1)cut 用指定规制切分文本

+ `cut -d ':' -f1,2 <filename> ` 将文件以冒号分隔符进行切分，并且显示每行的第一个和第二个字段

### (2) sort 对文件内容进行排序

+ `sort <filename>` 将文件每行按照字典序进行排序
+ `sort -t ':' -k2 <filename> `将文件内容按照':'进行分割，并且按照每行分隔出来的的第二元素字典序进行排序
+ `sort -t ':' -k3 -n <filename>`将文件内容按照':'进行分割，并且按照每行分隔出来的的第二元素数值大小进行排序，如果有字母，字母在前
+ `sort -t ':' -k2 -r <filename>`加了-r表示按照逆序排序

### （3）wc 统计单词的数量

+ `wc <flienme>`统计文件中行数、字数、字符数
+ `-l`表示行数 `-w`表示字数 `-c`表示字符数

## 3.文件内容增删改查

### (1) grep 

+ `grep <msg> <filename>`在文件中查找`msg`的内容
+ `grep -n <msg> <filename>` 显示行号
+ `grep -nvi <msg> <filename> `忽略大小写进行查询
+ `grep -E "[1-9]" <filename>`使用正则表达式进行匹配

### (2) sed 

**Reference:**[Linux实战教学笔记12:linux三剑客之sed命令精讲 - 陈思齐 - 博客园 (cnblogs.com)](https://www.cnblogs.com/chensiqiqi/p/6382080.html)

### (3) awk

**Reference:**[Linux实战教学笔记18:linux三剑客之awk精讲 - 陈思齐 - 博客园 (cnblogs.com)](https://www.cnblogs.com/chensiqiqi/p/6481647.html)

# Vim 命令

## 1.普通模式

|     命令     |                作用                 |
| :----------: | :---------------------------------: |
|      dd      |             删除当前行              |
|     1yy      |             复制当前行              |
|      p       |                粘贴                 |
|      u       |                撤销                 |
|   ctrl + r   |                还原                 |
| / <查询内容> | 查找相关内容 （再按n 为查找下一个） |
|      gg      |            跳至文档开口             |
|      G       |            跳至文档结尾             |
|      x       |            删除一个字符             |
|   ctrl + v   |      进入可视模式，支持块操作       |

## 2. 进入编辑模式

| 命令  |          作用          |
| :---: | :--------------------: |
|   i   |   在光标当前位置插入   |
|   I   | 在光标所在位置行首插入 |
|   a   |     在光标后面插入     |
|   A   |     在光标行尾插入     |

## 3. 编辑模式

|   命令   |   作用   |
| :------: | :------: |
| ctrl + a | 跳至行首 |
| ctrl + e | 跳至行尾 |

# git命令

## 1. 基本概念

### （1）四个区

+ 工作区
+ 暂存区
+ 本地仓库
+ 远程仓库

## 2. 初始化配置

+ 用户名和用户信息配置

```shell
git config --global user.name "user_name"
git config --global user.email "user_email"
#如果使用了 –-global 选项，那么该命令只需要运行一次，因为之后无论你在该系统上做任何事情，Git 都会使用那些信息。当你想针对特定项目使用不同的用户名称与邮件地址时，可以在那个项目目录下运行没有 –-global 选项的命令来配置。
git config --list
git config username
```

+ 密钥配置

```shell
ssh -keygen -t rsa -C "user_email"
```

+ 仓库初始化

```shell
# Create a new repository on the command line
echo "# test" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin git@github.com:lzh123315/test.git
git push -u origin main

#or push an existing repository from the command line
git remote add origin git@github.com:lzh123315/test.git
git branch -M main
git push -u origin main
```

## 3. git 分支操作

+ 查看分支

    ```shell
    git branch #查看本地分支
    git branch -a #查看所有分支
    git branch -r #查看远程分支
    ```

+ 建立分支

    ```shell
    git branch newBranch
    #或者
    git checkout -b newBranch #创建新分支，并切换到该分支
    ```

+ 本地分支推送并创建远程分支

    ```shell
    git push <远程主机名> <本地分支名>:<远程分支名>  #ex:git push origin newBranch:newBranch
    ```

+ 本地分支新建分支推送到远程同名分支

    ```shell
    git checkout newBranch
    git pull
    git branch --set-upsteam-to=origin/newBranch
    ```

+ 分支切换

    ```shell
    # (1)当前branch1支没有写完，且不想提交但是有紧急需求需要切换分支branch2
    git stash save "<message>"
    git checkout branch2
    #	处理完紧急需求后
    git checkout branch1
    git stash pop
    
    # (1)本来想在branch1分支上开发，但是开发过程中发现处在branch2分支上，想强制将工作区间代码迁到A分支
    git stash save "<message>"
    git checkout branch1
    git stash pop
    //解决所有冲突后
    got add -A
    ```

+ 合并分支

    ```shell
    git checkout develop
    git merge newBranch
    git push
    ```

    **冲突解决**:用VS code 或者 `git checkout --ours`或者`git checkout --theirs`以当前分支为准或者以合并分支为准。

+ 删除分支

    ```shell
    # 删除本地分支
    git branch -D xxxxx
    # 删除远程分支
    git push origin --delete newBranch
    ```

## 4. git 删除撤销操作

+ 已修改，未暂存：在工作区保存了文件，未执行`git add .`之前

    ```shell
    git diff #查看修改了那些文件
    git checkout . 
    #或者
    git reset --hard
    ```

+ 已暂存，未提交

    ```shell
    git diff --cached
    git reset gut checkout .
    #或者
    git reset --hard
    ```

+ 已提交,未推送

    ```shell
    git diff branch1 branch2 #ex:git diff master origin/main master:本地主分支，origin/master 远程主分支
    git reset --hard origin/branch1
    ```

+ 已推送

    ```C
    git reset --hard HEAD^ git push -f
    ```

    

+ git 删除文件夹

    ```shell
    # 删除暂存区或者本地仓库的文件 
    git rm --cached <filename> #递归删除添加-r

    ## 删除工作空间和本地仓库的文件
    git rm <filename> 

    ## 批量删除
    #修改.gitignore
    git rm -r --cached .

    git commit -m "message"
    # 如果不想再写commit message
    git commit --amend --allow-empty
    ```


# docker

## 1. docker 安装
**reference**: https://docs.docker.com/engine/install/ubuntu/

（1）赋予docker管理员权限
```shell
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```
## 2. 配置阿里云镜像加速
**reference**: https://www.cnblogs.com/qican/p/15507934.html
## 3. docker常用命令
## docker启动
```shell
systemctl start docker   #启动docker
systemctl enable docker  #设置开机自启动
```

### 镜像命令
```shell
docker images    #查看本机所有镜像 
docker search    #搜索镜像
docker pull      #拉取镜像
docker rmi       #删除镜像

docker commit -m "描述信息" -a "作者" 容器名 目标镜像名:[tag]  #编辑容器后提交容器成为一个新镜像

docker system prune #清理docker缓存
```
### 容器命令
```shell
docker run [可选参数] [镜像名]          #新建容器并启动        
docker exec [可选参数] [容器名]         #在容器中运行命令
#可选参数说明
--name="Name"   #容器运行时的名字
-d              #后台方式运行
-it             #使用交互方式运行，进入容器查看内容 
-p              #指定主机端口映射到容器端口 （-p ip:主机端口：容器端口 -p 主机端口:容器端口）
-P              #随机制定端口

docker run -it ubuntu /bin/bash    #启动并进入容器，并且运行bash
docker exec -it ubuntu /bin/bash

docker ps [可选参数]   #列出正在运行的容器

#退出容器
exit
#后台运行容器
Ctrl + P + Q
```
### 容器数据卷
为了实现数据持久化，使容器之间可以共享数据。可以将容器内的目录，挂载到宿主机上或其他容器内，实现同步和共享的操作。即使将容器删除，挂载到本地的数据卷也不会丢失。
```shell
#直接使用命令
docker run -it -v 主机内目录：容器内目录 镜像名
#具名挂载卷
docker run -d -v 卷名:容器内目录 镜像名

docker run -d -v volume01:/home ubuntu /bin/bash
#通过命令 docker volume inspect 卷名  可以找到主机内目录/
```
### dockerfile
```shell
docker build -f <dockerfile path> -t <image name> . #构建镜像
docker push         #发布镜像
```
### docker清理
```shell
# 删除所有已经停止的容器
docker rm $(docker ps -a|grep Exited |awk '{print $1}')docker rm $(docker ps -qf status=exited)

#删除所有未打标签的镜像
docker rmi $(docker images -q -f dangling=true)

#删除所有无用的volune
docker volume rm $(docker volume ls -qf dangling=true)

#清理磁盘、删除关闭的容器、无用的数据卷和网络
docker system prune
```

# remote debug setting

## 1. Install ssh server

```shell
sudo apt update
sudo apt install openssh-server -y
sudo systemctl status ssh #查看状态
# 如果你的防火墙开启了，使用下面语句
sudo ufw allow ssh
```

## 2. vscode 

(1)vscode 安装一下插件:

+ Remote-SSH
+ Remote Development

(2) 打开Remote-ssh

```shell
ssh username@xxx.xxx.xxx.xxx
```

## 4.build and run

(1) vscode远程ssh连接后，可以直接在vscode里编辑远程主机代码

(2) 在vscode终端`artinx_hub`目录下运行以下命令，即可编译并运行程序

```shell
sudo bash ./scripts/auto-make.bash    #需要修改一些文件内容
```

## 3. httpServer visualization

打开浏览器输入`xxx.xxx.xxx.xxx:5630/pages/index.html`,注意远程调试主机和机器人NUC连同一个wifi, `xxx.xxx.xxx.xxx`为机器人NUC的IP地址。