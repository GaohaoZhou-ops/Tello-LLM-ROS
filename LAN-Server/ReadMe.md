# LAN Server

This submodule supports building and calling servers within the local area network. You need to follow the steps below to deploy it on the model inference server. The device startup method remains unchanged.

这个子模块用来支持在本地局域网内搭建服务器并调用的功能，你需要按照下面的步骤在模型推理服务器上进行部署，端侧设备启动方式保持不变。

## Ste1. 安装依赖 Install dependencies

```bash
$ conda activate your_env
$ pip install Flask
```

## Step2. 启动服务 Start the service

After completing the environment deployment on the server, start the service using the following command:

在服务器上完成环境部署后使用下面的命令启动服务：

```bash
$ python3 custom_model_server.py
```

