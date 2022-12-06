# ARTINX-HUB

Artinx视觉组 集成框架

<!-- vim-markdown-toc GFM -->

- [入门](#入门)
    - [计算机基础](#计算机基础)
    - [C++](#c)
    - [Linux/Git/Shell](#linuxgitshell)
- [快速跳转](#快速跳转)
- [开发规范](#开发规范)
    - [代码规范](#代码规范)
    - [Commit规范](#commit规范)
    - [GitLab工作流](#gitlab工作流)
- [仓库目录结构](#仓库目录结构)
- [本地构建指南](#本地构建指南)
    - [Windows](#windows)
    - [Linux](#linux)
    - [Genetic](#genetic)
- [机器人部署指南](#机器人部署指南)
    - [本地环境配置](#本地环境配置)
    - [CI配置](#ci配置)
- [CI持续部署指南](#ci持续部署指南)
    - [自启动流程](#自启动流程)
    - [自动部署](#自动部署)
- [故障排除](#故障排除)
- [框架指南](#框架指南)
    - [程序工作流](#程序工作流)
    - [框架工具类使用指南](#框架工具类使用指南)
    - [Group Mask使用方法](#group-mask使用方法)
    - [代码详解](#代码详解)
- [踩过的坑](#踩过的坑)

<!-- vim-markdown-toc -->
## 入门

没有速成，速成的都是垃圾

[CheckList](docs/checklist.md)

### 计算机基础

推荐书籍（按照难度排序）：

- Computer Systems: A Programmer's Perspective (CSAPP)
- 计算机程序的构造和解释（SICP）
- 计算机组成与设计：硬件/软件接口
- 操作系统概念（恐龙书）
- 程序员修炼之道2：通向务实的最高境界
- 编译原理（龙书）

### C++

基础

- C++ Primer(Plus)
- [于仕琪老师的Bilibili网课](https://www.bilibili.com/video/BV1Vf4y1P7pq)
- C++ Programming Language

进阶

- Effective C++
- More Effective C++
- Effective Modern C++
- Modern C++ Tutorial: C++11/14/17/20 On the Fly
- <https://github.com/AnthonyCalandra/modern-cpp-features>
- <http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines>
- [小彭老师的Bilibili公开课](https://space.bilibili.com/263032155/channel/collectiondetail?sid=53025)

骨灰

- <https://www.youtube.com/user/CppCon>
- <https://www.open-std.org/jtc1/sc22/wg21/docs/papers/>
- <http://purecpp.org/>

### Linux/Git/Shell

- <https://git-scm.com/docs/user-manual>
- <https://linuxtools-rst.readthedocs.io/zh_CN/latest/>

## 快速跳转

- [cppreference](https://en.cppreference.com/w/)
- [glm manual](https://github.com/g-truc/glm/blob/master/manual.md) or [Opengl-glm](https://nas.artinx.club:5001/sharing/q01EttQss)
- [OpenCV doc](https://docs.opencv.org/4.x/)
- [规则手册](https://www.robomaster.com/zh-CN/resource/pages/announcement/1370)

## 开发规范

### 代码规范

- 所有类型名使用大驼峰ThisIsType，宏使用大写+下划线THIS_IS_MACRO，其余使用小驼峰thisIsVariable
- 使用根目录下的.clang-format文件格式化代码
- 未经允许禁止引入新的第三方库
- 使用C++17标准，不在使用宏支持跨平台的情况下使用编译器/操作系统相关的代码
- 未经允许禁止引入新的单例
- 在vector大小已知时使用resize/reserve预分配空间
- 使用智能指针，一般情况下不允许出现任何形式的new/delete
- 使用函数/模板重用代码
- 仅允许Resharper C++和clang的linter标记
- 按值传递所有权，其余情况一遍按const引用传递参数。拥有SSO优化的string二者均可。
- 未经允许禁止更改公共API
- 尽量使用Transform.hpp提供的编译期量纲分析和参考系检查的Point/Vector/Normal/Transform，不直接使用glm库
- 使用Identifier和BlackBoard系统传递大型结构体
- 禁止使用C动态内存管理和字符串API
- 使用SynchronizedClock作为同步系统时钟
- 使用Clock获取系统时钟的相关类型信息
- 所有Atom必须使用**ACTOR_PROTOCOL_CHECK**和**ACTOR_PROTOCOL_DEFINE**检查参数类型

### Commit规范

- commit信息应遵循Angular规范，建议使用VSCode的插件commitizen
- 每个commit的更改内容应该尽可能保持小范围且集中
- 尽可能确保commit时源码能够正常编译，正常运行，通过测试
- 未经允许禁止提交二进制文件，禁止提交个人配置文件
- 一切新功能代码修改均在feature-（小写和-组成）分支下进行，一切bug修复均在hotfix-分支下进行，在review通过后，需向develop分支发起**merge request**，由管理员merge至develop分支
- develop分支的功能稳定后，将由管理员merge至main分支，非管理员无法直接对develop分支和main分支做修改

- 工作流样例：

  ```bash
  git branch <branch name>
  git checkout branch
  # do some modifications
  git add .
  git commit -m "<message>"
  git push -u origin <branch name>
  ```

+ git命令总结可参见[tool_tutorials.md](docs/tool_tutorials.md)

### GitLab工作流

1. 某人发起新的issue对应新的功能/bug修复，此时新的补充或纠正等讨论内容发在issue上
2. 管理员指定给某人在该issue上工作，创建新的merge request以跟踪进度
3. assignee完成工作后push代码，由reviewer审核代码，完成审核后由管理员merge入develop分支，并在log.md写明本次merge添加了什么功能，修改了那些程序逻辑
4. merge完成后issue被自动关闭

## 仓库目录结构

```markdown
.
|-- bringup_templates 自启动/自动部署脚本
|-- config_templates actor使用示例
|-- data 数据
    |-- camera_calibration 相机标定结果
    |-- weights 神经网络/SVM 权重文件
|-- deploy_config 机器人部署配置
|-- docs 次级文档
|-- include 头文件
|-- pages 可视化网页文件
|-- scripts 脚本
|-- src 源代码
|-- tests 单元测试文件
|-- .clang-format C/C++格式化配置文件
|-- .gitattributes
|-- .gitignore
|-- .gitlab-ci.yml GitLab CI配置文件
|-- CMakeLists.txt CMake根目录配置文件
|-- CMakeSettings.json Visual Studio 2019 CMake配置文件
|-- Folder.DotSettings Resharper++ Lint配置文件
|-- README.md 自述文件
```

## 本地构建指南

下面仅介绍VS工作流和Clion工作流，可以自行探索VS Code/Vim工作流

### Windows

- 安装IDE Visual Studio 2019（桌面C++ + 英文语言包）
- 按照Genetic步骤安装依赖
- 添加环境变量 `ONEAPI_ROOT=<path>\intel`
 `DAHENG_SDK=<path>\GalaxySDK\Samples\VC SDK`
- clone仓库
- 用VS打开文件夹,填写cmake参数`-DARTINX_HUB_CAMERA=USB3 -DCMAKE_TOOLCHAIN_FILE=<path to vcpkg>/scripts/buildsystems/vcpkg.cmake`
- 点击项目-配置ArtinxHub 生成构建文件
- 点击生成-全部生成 编译程序
- 点击调试-调试和启动ArtinxHub的设置，配置args字段（config文件路径）
- 按F5或调试-开始调试 开始调试

### Linux

+ 安装Clion
+ 按照Genetic步骤安装依赖
+ 添加环境变量
```shell
sudo vim /etc/profile                           #打开/etc/profile文件

#在文件末尾加入以后命令
export DAHENG_SDK=<PATH>/Galaxy_camera     #PATH为相机SDK所在目录
export ONEAPI_ROOT=/opt/intel                  #/opt/intel 为ONEAPI默认安装目录，若不在,请修改
source /opt/intel/openvino_2021/bin/setupvars.sh
```
+ clone仓库
+ 用clion打开文件夹
+ 打开CMake设置，填入参数`-DARTINX_HUB_CAMERA=USB3 -DCMAKE_TOOLCHAIN_FILE=<path to vcpkg>/scripts/buildsystems/vcpkg.cmake` **2.0相机写`USB2`**
+ 在CMake选项卡生成构建文件
+ 在Build选项卡编译程序
+ 添加运行配置，填入参数（config文件路径）
+ 运行/调试

### Genetic

- 克隆[vcpkg](https://github.com/microsoft/vcpkg)，并执行bootstrap脚本
- 用vcpkg安装以下软件包(windows下, 软件包名称后加`:x64-windows`)：
  - nlohmann-json
  - caf
  - glm
  - cpp-httplib
  - fmt
  - bullet3 (未来可能被移除)
  - magic-enum
  - opencv4[contrib,ffmpeg]
  - glew
  - glfw3
  - opengl
  - eigen3
  - boost
  如遇任何问题，请按照错误提示用apt补足缺少的软件包或更换网络重试一次
+ 集成vcpkg安装包，运行
`./vcpkg integrate install`

- 安装OneAPI [Download the Intel® oneAPI Base Toolkit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html)
  - 仅勾选TBB即可，其它没用
- 安装2021 离线版OpenVINO [Download Intel® Distribution of OpenVINO™ Toolkit](https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit-download.html)
  - 也要安装在OneAPI文件夹下

- 根据需求(USB2/USB3)安装大恒相机驱动[Daheng Imaging](https://daheng-imaging.com/list-58-1.html), 对应CMake参数的ARTINX_HUB_CAMERA=USB2/USB3

## 机器人部署指南

### 本地环境配置

- 按照Linux环境配置即可（统一使用Ubuntu 20.04LTS，如果相机驱动不工作考虑降Linux内核版本，统一文件夹路径）
- 在BIOS中配置通电/恢复供电自启动
- 配置开机自动登录
- 配置串口通讯免Root

```shell
- ~~use `groups ${USER}` to check groups~~
- sudo gpasswd --add ${USER} dialout
```

在机器人上用clion调试前记得临时关闭service：

```shell
sudo systemctl stop ArtinxHub.service
sudo kill $(pidof ArtinxHub)
```

- 安装gitlab-runner
- 注册runner(记得把名字设对)

```shell
sudo gitlab-runner register --url https://mirrors.sustech.edu.cn/git/ --registration-token GR1348941e3w2cGho8r6x-dJatgRC
sudo gitlab-runner start
```

- 给runner root权限

```shell
sudo usermod -a -G sudo gitlab-runner
sudo visudo
加入这一行
gitlab-runner ALL=(ALL) NOPASSWD: ALL
```

- 把vcpkg目录链接到/opt/vcpkg
- 如果使用USB2驱动，记得添加相关环境变量到/opt/env_setup.sh下

### CI配置

- 在settings-CI/CD-runner界面，打开对应runner的设置
- 设置tag为 linux, <机器人名>，如果tag是windows则为CI机
  - 机器人名可为infantry1,infantry2,sentry,uav,hero,engineer
- 设置不在 非develop和无tag commit后执行runner
- 在Deployments-Environments查看部署状态

## CI持续部署指南

### 自启动流程

- 系统启动后启动ArtinxHub.service
- service执行/opt/artinx-hub/bringup_templates/bringup.bash
  - 加载intel openvino的环境变量
  - 加载/opt/env_setup.sh的环境变量
  - 从/opt/deploy_target.conf中读取配置名
  - 启动artinx-hub
  - 若程序终止，则sleep 1s后尝试重新启动程序

### 自动部署

- 管理员手动下发部署信号
- 本地gitlab-runner pull最新develop分支，进行增量构建
- ci构建完毕后，执行./scripts/setup-service.bash
  - 关闭已有服务和进程
  - 拷贝构建结果到/opt/artinx-hub/文件夹下
  - 拷贝服务配置文件bringup_templates/ArtinxHub.service到/lib/systemd/system/ArtinxHub.service
  - 写入要启动的配置名到/opt/deploy_target.conf
  - 启动服务（要重启，因为没用start，可以试试直接改为start）

## 故障排除

按照概率排序：

- 程序无法启动（报BadConfig等）
  - 字段类型错误
  - 字段名错误
  - atom目标缺失
  - actor初始化抛异常
  - 缩进错误
- 程序卡死
  - 某个Actor挂了（使用Probe/std::cout定位bug）
  - 某个Actor死循环（监视系统未完成，谁来干一下）
- 串口无法收发数据
  - 重新插拔usb2ttl
  - 下位机/上位机包格式不一致（都是电控的锅）
  - buffer炸了（调整发包周期）
  - usb2ttl松了/坏了
  - 线扯断了
  - 用```ls /dev | grep ttyUSB`
  - ``来查看是否识别串口
- 相机无法启动
  - 查看错误码查文档
  - 打开Galaxy看看能不能检测到(仅限3.0相机）
  - 重装驱动，重新启动，重新插拔数据线
  - **哨兵靠相机的SN码区分上下云台，看看confg里面有没有写错**
- 机器人上自瞄不工作
  - 使用systemctl status ArtinxHub.service查看服务状态
  - 打开127.0.0.1:5430查看工作状态
  - 多半是相机或串口寄了

## 框架指南

CAF框架参见[actor_system.md](docs/actor_system.md)

### 程序工作流

- CAF框架读取caf-application.conf初始化并进入caf_main
- caf_main根据argv[1]读取conf文件并解析
- 根据配置文件通过工厂模式初始化各actor，并将actor的地址收集到map里用于通讯
- actor初始化完毕，main广播start_atom
- 主进程睡眠，等待terminateSystem被调用（仅测试结束时被调用）或因为异常退出

### 框架工具类使用指南

- HubLogger::watch: 所有键值对将被实时显示在页面中
- BlackBoard: 由于CAF框架不支持传递未被注册的类型（只支持基本类型，std::string和常见STL容器的序列化/反序列化/传参），故使用key-value分离的方式传参
  - sender先将key-value通过updateSync传到BlackBoard上，再拿到type-wrapped的key
  - sender将type-wrapped的key通过sendAll接口发送给receiver
  - receiver通过type-erased的key从BlackBoard拿到value的**拷贝**

- **ACTOR_PROTOCOL_CHECK**和**ACTOR_PROTOCOL_DEFINE**：指示atom对应的参数类型
  - ```ACTOR_PROTOCOL_DEFINE(set_target_atom, TypedIdentifier<SelectedTarget>);```
    表示atom对应的参数类型为一个type-wrapped的key，这个key指向被放在BlackBoard上的SelectedTarget类型的value
  - 在sendAll处会自动检查atom与参数类型的对应关系
  - 在actor的behavior接收参数时，需要手动check：```ACTOR_PROTOCOL_CHECK(set_target_atom, TypedIdentifier<SelectedTarget>);```, 与define对称，这里需要自觉和参数/实际使用方法匹配
  - 如遇编译错误则说明没用include对应define的头文件
- 如遇比较棘手的Bug，可以使用ACTOR_EXCEPTION_PROBE宏将BUG范围缩小至代码块级别：
  - 只要在任意代码块内使用```ACTOR_EXCEPTION_PROBE();```即可
  - 这个宏会建立一个变量，处理三类问题：
    - 浮点异常（除零，nan等）：生命周期开始时启动硬件浮点异常，结束时关闭硬件浮点异常，如果中间触发了浮点异常则会收到signal，可以定位到行
    - 异常超时（延迟过高）：生命周期开始和结束时会计时，由于程序每秒需要处理上百帧，过长的生命周期显然是个bug，可以定位到代码块
    - 未处理异常：正常情况下actor在遇到未处理异常后会析构，而程序本身不析构，所以这个probe会在遇到未处理异常导致的析构时直接崩溃，可以定位到代码块
  - 这个宏在release下检查超时，debug下检查浮点异常和未处理异常，开销可忽略不计

### Group Mask使用方法

由于哨兵的特殊用法（一个sentry strategy需要根据数据来自上下云台选择发给哪个angle solver），故引入了group mask机制：

- config里atom的定义改为所有**可能**接受消息的actor
- config新增两个内置属性group_id和group_mask，如果设置了group_id则mGroupMask为1<<group_id,如果设置了group_mask则mGroupMask为group_mask，否则默认为1
- HubHelper里的mGroupMask用于指示自己的身份和通讯组
- sendMasked比sendAll增加mask参数，当**mask和receiver的mGroupMask按位与不等于0**时才发送
- 如果发给不同mask的数据不一样，需要给每个mask分配一个独立的key

## 代码详解

+ 参见[code_details.md](docs/code_details.md)

## 踩过的坑

    记录神奇的错误

- 不要在debug模式下跑CTest
