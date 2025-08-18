# -*- coding: utf-8 -*-

import rospy
import requests
import os
import json
from .base import LLMBase
import time

class DeepseekClient(LLMBase):
    """
    DeepSeek API implementation of the LLMBase.
    Connects to the DeepSeek online API to get model responses.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the DeepSeek client.
        - api_key is retrieved from environment variables or ROS params.
        """
        self.api_base = "https://api.deepseek.com/chat/completions"
        
        # 优先从环境变量获取API Key，这更安全。
        # 如果环境变量没有，则尝试从kwargs (来自ROS参数) 获取。
        self.api_key = os.getenv('DEEPSEEK_API_KEY')
        if not self.api_key:
            self.api_key = kwargs.get('api_key')
        if not self.api_key:
            rospy.logerr("DeepSeek API key not found. Please set the DEEPSEEK_API_KEY environment variable or provide it as a ROS param.")
            raise ValueError("DeepSeek API key is missing.")
            
        self.headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
        rospy.loginfo(f"DeepseekClient initialized for model: {self.model_name}")

    def query(self, system_prompt, user_prompt):
        """
        Queries the DeepSeek API.
        """
        payload = {
            "model": self.model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            "stream": False
        }

        try:
            start_time = time.time()
            response = requests.post(self.api_base, headers=self.headers, json=payload, timeout=30)
            end_time = time.time()
            duration_s = end_time - start_time
            response = requests.post(self.api_base, headers=self.headers, json=payload, timeout=30)
            
            # 检查API调用是否成功
            response.raise_for_status() 
            response_data = response.json()
            
            if 'choices' in response_data and len(response_data['choices']) > 0:
                content = response_data['choices'][0]['message']['content']
                # <--- 修改: 按约定返回0个token --->
                return True, content, "", duration_s, 0, 0
            else:
                error_msg = f"API response is malformed. Response: {response.text}"
                return False, "", error_msg, duration_s, 0, 0

        except Exception as e:
            return False, "", str(e), 0.0, 0, 0