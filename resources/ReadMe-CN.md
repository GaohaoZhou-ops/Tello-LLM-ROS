# Tello LLM ROS

这个仓库实现了在 ROS 框架下使用 LLM 对 Tello 无人机进行控制，并以自然语言指令作为输入。当前的版本仅支持通过本地 Ollama 模型的调用，我们正在测试使用在线模型和 Agent 系统的调用方式，一旦完成测试我们将在第一时间更新仓库。

# Benchmarks

## Local Model Test
当前我们仅在 `Nvidia Jetson Orin 64GB DK` 这个硬件上开展了实验，未来我们会尝试在更丰富的硬件设备上进行测试。实验环境的系统与库信息如下：

![jetson_release](./jetson_release.png)

在此基础上我们对多个不同的本地模型性能与表现进行了评估，测试样本可以查看脚本 `src/tello_llm_ros/scripts/test_llm_offline.py` 中 `_define_test_cases` 函数的内容：

|Model|大小|准确率|平均响应时长 s|平均生成速度 tokens/s|
|--|--|--|--|--|
| codellama:7b | 3.8 GB | 35.00% | 1.58 | 433.53 |
| codellama:13b | 4.7 GB | 55.00% | 3.44 | 191.98 |
| llama3.1:8b | 4.9 GB | 60.00% | 2.04 | 257.65 |
| llama3-groq-tool-use:8b | 4.7 GB | 50.00% | 2.03 | 261.59 |
| qwen3:4b | 2.5 GB | 50.00% | 80.61 | 32.65 | 
| qwen3:8b | 5.2 GB | | | | 
| qwen3:14b | 9.3 GB | | | |
| deepseek-coder-v2:16b | 8.9 GB | 60.00% | 1.56 | 376.31 | 
| gpt-oss:20b | 14 GB | | | |

我们初步的实验得到了以下几点结论：

1. 大多数本地模型测试样本失败是由于添加了额外的 `takeoff` 和 `land` 命令；
2. 对于本地小参数量的模型而言，使用纯文本系统提示词比使用 json 定义工具描述成功率更高；
3. 本地模型容易将一个动作拆成多个动作指令，这可能和系统提示词有关，例如 “旋转180度”，模型会输出2次 “旋转90度”的命令；

为了尽可能降低整体系统响应时长，我们对一些明确指令采取直接调用的形式，这些指令不会输入模型进行推理，例如 `takeoff`。你也可以添加更多直接运行的指令，修改 `config/llm_tools.json` 文件中 `direct_triggers` 字段如下所示，`takeoff`，`take off`，`launch` 这三条指令都是可以直接响应的：

```json
    {
      "name": "takeoff",
      "description": "Initiates the drone's automatic takeoff sequence...",
      "direct_triggers": [
        "takeoff",
        "take off",
        "launch"
      ],
      "parameters": [],
      "ros_service": "/takeoff",
      "service_type": "Trigger"
    },
```

## 在线模型测试

Comming Soon...

----
# Step1. 安装依赖库

## 1.1 安装基础依赖库
在运行之前你可能需要安装包括但不限于以下依赖库：

```bash
$ conda install libffi==3.3
```

## 1.2 创建conda环境

```bash
$ conda create -n tello python=3.8
$ conda activate tello
$ pip install -r requirements.txt
```

----
# Step2. 源码编译

进入到你的工程中拉取源码，这里假设为 `tello_ws`：

```bash
$ cd tello_ws/src
$ git clone https://github.com/GaohaoZhou-ops/Tello-LLM-ROS.git
```

拉取完成后即可执行编译：

```bash
$ cd tello_ws
$ catkin_make
```

----
# Step3. 拉取模型

你可以通过下面的命令拉取 Ollama 开源的模型，这里以 `Qwen3:8b` 为例：

```bash
$ ollama pull qwen3:8b
```

----
# 如何使用
我们提供了多种使用模式，包括真机与模型联合测试、模型独立测试、模型与mock联合测试。

如果你想要通过 XBox 手柄对无人机实现控制，可以使用我们另一个开源仓库：

* XBox Controller Reader: [https://github.com/GaohaoZhou-ops/XboxControllerReader](https://github.com/GaohaoZhou-ops/XboxControllerReader)

## 停止多余模型

在调用模型前，为了避免资源被未关闭的模型抢占，你可以通过下面的命令来关停正在运行的模型：

```bash
$ ollama ps
$ ollama stop codellama:7b
```

![ollama-stop](ollama_stop.png)

## 修改系统提示词

众所周知，系统提示词对模型的性能表现影响很大，尽管工程中的系统提示此内容已经经过了多次打磨，但并不一定适合你的任务。如果你发现模型在表现上无法令人满意，那么可以通过修改系统提示词的方式给模型指令约束。我们将系统提示词拆解成两部分 `通用系统提示词` 和 `工具描述`，这些文件保存在 `config` 目录下，最终的系统提示词是两部分拼接得到：

```bash
├── config
│   ├── common_system_prompt-CN.txt
│   ├── common_system_prompt-EN.txt
│   ├── llm_tools.json
│   ├── pure_text_tools_description-CN.txt
│   └── pure_text_tools_description-EN.txt
```

## 添加工具

如果你准备调用大模型，那么建议修改文件 `config/llm_tools.json` 来定义你的工具内容；

如果你准备调用小模型，那么建议修改纯文本文件 `config/pure_text_system_prompt-EN.txt` 来定义你的工具；

但无论你准备用哪种方法添加工具，都需要自己实现；

## 测试模型性能

在正式开始之前，我们强烈建议先用我们提供的测试节点测试一下模型在你当前设备上的性能与效率，如果成功率太低或者平均任务反应时长太长，那么建议更换模型。

```bash
$ unset all_proxy
$ unset ALL_PROXY
$ cd tello_ws
$ source devel/setup.bash
$ roslaunch tello_llm_ros test_llm.launch
```

![LLM-test](./LLM-test.png)

## 本地模型+仿真测试

在确认好使用的模型后，可以先以 mock 模式运行来测试当前模型的表现是否是你预期的效果，因为测试案例涵盖的样本太少。

* 修改 `launch/tello.launch` 文件中 `use_sim` 字段为 `true` 启用仿真：

```xml
<arg name="use_sim" default="true" doc="Set to true to run in simulation mode"/>
```

* 修改 `launch/llm_interface.launch` 文件中 `ollama_model` 字段为你想要运行的模型：

```xml
<arg name="ollama_model" default="qwen3:4b" doc="The Ollama model to use"/>
```

打开一个终端运行仿真器：
```bash
$ cd tello_ws
$ source devel/setup.bash
$ roslaunch tello_llm_ros tello.launch
```

![rviz](./rviz.png)

新开一个终端运行模型交互窗口，通过输入 `quit` 退出节点：
```bash
$ unset all_proxy
$ unset ALL_PROXY
$ cd tello_ws
$ source devel/setup.bash
$ roslaunch tello_llm_ros llm_interface.launch
```

![LLM-interface](./LLM-interface.png)

## 本地模型+真机测试

如果你的模型表现达到你的预期，接下来就可以使用真机进行测试：

* 修改 `launch/tello.launch` 文件中 `use_sim` 字段为 `false` 启用真机：

```xml
<arg name="use_sim" default="true" doc="Set to true to run in simulation mode"/>
```

* 修改 `launch/llm_interface.launch` 文件中 `ollama_model` 字段为你想要运行的模型：

```xml
<arg name="ollama_model" default="qwen3:4b" doc="The Ollama model to use"/>
```

打开一个终端运行真机：
```bash
$ cd tello_ws
$ source devel/setup.bash
$ roslaunch tello_llm_ros tello.launch
```

新开一个终端运行模型交互窗口：
```bash
$ unset all_proxy
$ unset ALL_PROXY
$ cd tello_ws
$ source devel/setup.bash
$ roslaunch tello_llm_ros llm_interface.launch
```

