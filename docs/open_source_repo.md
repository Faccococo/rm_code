# 2023赛季视觉开源内容整理



# 2022赛季

## 上海工程技术——步兵

https://github.com/Birdiebot/bubble/blob/main/.github/README_zhCN.md

文档：https://birdiebot.github.io/bubble_documentation/

检测算法均为传统视觉，但包含预测与弹道补偿部分

#### 实现内容

- 大小符
- 自瞄
- 旋转前哨站
- 弹道 / 运动补偿



## 华中科技大学——步兵

[XianMengxi/AutoAim_HUST: The robot team LANGYA of HUST,RoboMaster Open Source (github.com)](https://github.com/XianMengxi/AutoAim_HUST)

类似吉林大学的传统视觉识别灯条 + SVM分类数字

- 大小符识别预测
- 自瞄
- 反陀螺

## 华南师范大学——通用
[chenjunnn/rm_vision](https://github.com/chenjunnn/rm_vision)

实现内容
+ 基于ros2的视觉自瞄代码
+ 传统视觉 + 神经网络数字识别
+ 反陀螺算法（具反映小陀螺低速的时候效果较好，高转速全部预测到了车辆外面）

## 西安电子科技大学

[SanZoom/RM2022-Infantry-Vision: RMUC2022赛季 IRobot战队步兵视觉完整代码 (github.com)](https://github.com/SanZoom/RM2022-Infantry-Vision)

#### 实现内容

- 装甲板自瞄+预测（传统视觉 + 神经网络数字识别， 预测为匀速的卡尔曼滤波）
- 大小符

## 制导飞镖

[[飞镖视觉1.0开源\]基于FPGA和传统视觉的光点追踪【RoboMaster论坛-科技宅天堂】](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22053)

针对绿色的飞镖引导灯实现的光点追踪，可以提供一种思路

## 沈阳航空航天大学——通用

https://github.com/tup-robomaster/TUP-InfantryVision-2022

#### 亮点

神经网络实现

通过缩小送入网络的区域，强行提高识别极限距离（手动ROI模式）

#### 实现

装甲板检测

- 将网络头部的Focus层更换为6*6卷积，优化推理速度。
- 增加CoordConv，为网络输入中增加xy坐标特征图，提升装甲板角点的回归精度。
- 替换网络backbone为轻量级网络ShufflenetV2,加快推理速度。
- 将检测头原本的reg分支由回归bbox的(xywh)改为装甲板四点(x1y1x2y2x3y3x4y4),形式上类似关键点检测，并使用WingLoss训练。
- 为检测头增加color分支，将分类数由类别*颜色变为类别+颜色。有效降低分类维度，提高各类样本数，降低网络学习难度。

打符

- 转速求解
- 预测

## 南京航空航天——雷达站（前端项目）

https://github.com/bismarckkk/RadarDisplayer/



![image-20221104160257252](C:\Users\lenovo\AppData\Roaming\Typora\typora-user-images\image-20221104160257252.png)



## 川大&沈航视觉数据站

https://rmcv.52pika.cn/



# 2021赛季

## 华南理工广州学院——步兵

https://github.com/wildwolf-team/WolfVision.git

#### 实现内容

- 对敌方装甲板和己方能量机关进行有效识别。
- 自定义串口通讯协议与下位机进行通讯控制云台运动。
- 对哨兵的运动进行实时分析，实现预测击打。
- 根据目标深度和弹丸速度对Pitch轴进行补偿，实现准确命中。
- 通过计算能量机关当前的速度对击打位置进行预测，实现大小能量机关的击打。



## 青岛大学——步兵

https://github.com/qsheeeeen/qdu-rm-ai

#### 实现内容

- 自瞄+打符
- 通过DLA(深度学习加速器)加速妙算上模型的推断速度。利用行为树实现了可控的复杂行为
- 可能实现了yolov5的优化

最近更新时间在11个月前



## 西北工业——步兵

https://github.com/NZqian/WMJ2021

#### 实现内容

- 装甲识别
- 陀螺检测
- 目标解算
- 移动预测
- 能量机关的识别与击打



## 武汉科技——工程

https://github.com/chinaheyu/wust_engineer_robot_ws

#### 实现内容

- 矿石辅助对位

#### 实现方式

深度信息滤波 + 颜色信息滤波



## 上海交大——哨兵

https://github.com/Harry-hhj/CVRM2021-sjtu

#### 实现内容

- 反击：预测+反螺旋
- 反导
- 能量机关：暂时未部署



## 四川大学

https://scurm.coding.net/s/b2b521bf-b138-4054-bfd2-a62f487049a3/1

#### 实现内容

- 自瞄
- 大符
- 飞镖制导

Plus：川大的文档非常详细，可以参考

## 佛山科学技术学院——步兵

https://github.com/Ash1104/RoboMaster2021-FOSU-AWAKENLION-OpenSource

#### 实现内容

- 大小符
- 自瞄



# 2020赛季

## 华北理工——步兵

https://github.com/yunwaikongshan/RM2020-Horizon-InfantryVisionDetector

#### 实现内容

- 大符检测与击打

- 陀螺检测与击打

- 自瞄 + 预测

  



## 吉林大学——步兵

https://github.com/QunShanHe/JLURoboVision

#### 实现内容

- 传统视觉识别装甲板 + SVM数字识别确定装甲板
- 大小符识别——传统视觉



# 2019赛季



## 北京理工-珠海学院——步兵

https://github.com/Brauzz/YIHENG_ZHBIT_VISION

#### 实现内容：

- 颜色轮廓+灰度轮廓装甲板
- 大小符

#### 亮点

- 装甲板识别加入了多线程操作

## 华盛顿大学——步兵自瞄

https://github.com/uw-advanced-robotics/aruw-vision-platform-2019

#### 实现内容：

- 神经网络识别装甲板
- 深度相机实现三维空间定位
- 机器人里程表（作用未知，据描述称是为了对己方机器人的移动进行抵消）
- 较为完善的弹道修正（己方移动，对方移动，重力，弹速等）

#### 提到：

深度相机进行识别可能存在拖影问题，需要在数据集中加入包含拖影的图片



## 上海交通大学——步兵

https://github.com/xinyang-go/SJTU-RM-CV-2019

#### 注：2021赛季上交提供了一份新的装甲板识别算法，因此这里只列出打符部分

#### 实现内容：传统视觉识别能量机关



## 深圳大学——步兵

https://github.com/yarkable/RP_Infantry_Plus

#### 实现内容

传统视觉识别装甲板

深度学习+传统视觉识别能量机关
