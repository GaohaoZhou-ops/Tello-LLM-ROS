#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, request, jsonify
import time
import random

def run_my_custom_model(model_name, system_prompt, user_prompt):
    """
    在这里调用您自己的模型。
    
    输入:
    - model_name (str): 您想使用的模型名称。
    - system_prompt (str): 系统提示词。
    - user_prompt (str): 用户输入。
    
    必须返回一个包含以下键的字典:
    - 'plan_text' (str): 模型生成的最终文本。
    - 'prompt_tokens' (int): prompt 的 token 数量。
    - 'completion_tokens' (int): 生成文本的 token 数量。
    """
    print(f"--- 正在使用模型 '{model_name}' 处理请求 ---")
    print(f"System Prompt: {system_prompt[:80]}...")
    print(f"User Prompt: {user_prompt}")
    
    # === 在这里替换为您模型的真实调用逻辑 ===
    time.sleep(random.uniform(1.5, 4.0)) 
    
    # 模拟模型输出
    mock_plan = (
        "[START_COMMANDS]\n"
        f"takeoff\n"
        f"move_forward {random.uniform(0.5, 2.0):.1f}m\n"
        f"rotate_clockwise {random.randint(45, 180)} degrees\n"
        "land\n"
        "[END_COMMANDS]"
    )
    
    # 模拟token计数
    mock_prompt_tokens = len(system_prompt.split()) + len(user_prompt.split())
    mock_completion_tokens = len(mock_plan.split())
    # ==========================================

    print(f"模型输出: {mock_plan.strip()}")
    print("-------------------------------------------\n")

    return {
        'plan_text': mock_plan,
        'prompt_tokens': mock_prompt_tokens,
        'completion_tokens': mock_completion_tokens,
    }
# --------------------------------------------------------------------------

# 创建 Flask 应用
app = Flask(__name__)

@app.route('/v1/chat/completions', methods=['POST'])
def handle_chat_completion():
    start_time = time.time()
    
    # 1. 解析客户端发来的JSON请求
    try:
        data = request.json
        model_name = data.get('model', 'default-model')
        system_prompt = data.get('system_prompt', '')
        user_prompt = data.get('user_prompt', '')

        if not user_prompt:
            return jsonify({"success": False, "error_message": "user_prompt is required."}), 400
    except Exception as e:
        return jsonify({"success": False, "error_message": f"Invalid request format: {e}"}), 400

    # 2. 调用模型
    try:
        model_output = run_my_custom_model(model_name, system_prompt, user_prompt)
        duration_s = time.time() - start_time
        
        # 3. 构造标准格式的成功响应
        response_data = {
            "success": True,
            "data": {
                "plan_text": model_output['plan_text'],
                "usage": {
                    "prompt_tokens": model_output['prompt_tokens'],
                    "completion_tokens": model_output['completion_tokens'],
                },
                "duration_s": duration_s
            },
            "error_message": ""
        }
        return jsonify(response_data), 200

    except Exception as e:
        # 4. 如果模型调用失败，返回错误响应
        return jsonify({"success": False, "error_message": f"Model inference failed: {e}"}), 500


if __name__ == '__main__':
    # 启动服务，监听在 0.0.0.0 的 5000 端口上
    # 这意味着服务器将接受来自局域网内任何IP的请求
    app.run(host='0.0.0.0', port=5000)