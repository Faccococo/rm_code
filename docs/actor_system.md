

# 参考文档

[CAF User Manual — CAF 0.18.5 documentation (actor-framework.readthedocs.io)](https://actor-framework.readthedocs.io/en/stable/index.html)

# Actor信息发送

+ 通过sendAll函数发送信息
+ 第一个参数必须为atom，具体定义在DataDesc.hpp
+ 后续参数自选，但需要和前后端协调
+ 大结构体通过BlackBoard系统传递（使用get/updateSync），第二个参数设为Identifier，Identifier为自身的类型的hashcode
+ 所有输出atom的类型需在HubHelper的模板参数中注册，HubHelper\<actor类型，Settings类型（可为void），send atom类型...\>

# Actor初始化

+ 所有actor初始化后，会收到一条start_atom消息，此时可以正常send
+ 初始化后Settings会放在mConfig中，可以直接读取

# Actor事件处理

+ event_based_actor在make_behavior中处理消息

+ blocking_actor在act函数中使用receives函数处理消息
+ 使用基于动态模板匹配的方式来分发参数到对应函数，因此要确保调用方和被调用方的接口一致

# 运行配置

+ 一定要2空格缩进，LF行尾

+ 具体格式参考config_templates

+ 参数与settings一致，注意参数的有效性

+ 将配置文件放入config目录下，并将conf文件的路径作为命令行参数填入

+ 在VS中将启动目标设为ArtinxHub.exe，点击调试-调试和启动ArtinxHub的设置，在json中加入args参数

  ```json
  {
    "version": "0.2.1",
    "defaults": {},
    "configurations": [
      {
        "type": "default",
        "project": "CMakeLists.txt",
        "projectTarget": "ArtinxHub.exe (src\\ArtinxHub.exe)",
        "name": "ArtinxHub.exe (src\\ArtinxHub.exe)",
        "args": [
          <path to configuration file>
        ]
      }
    ]
  }
  ```

  

# 调试输出

+ 暂时使用logInfo函数
