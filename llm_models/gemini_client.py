# -*- coding: utf-8 -*-

import rospy
import os
import time
import requests
import json
from .base import LLMBase

class GeminiClient(LLMBase):
    """
    Google Gemini API implementation of the LLMBase, using direct REST API calls.
    This version avoids potential SDK version conflicts.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the Gemini client by storing the API key and base URL.
        """
        # 优先从环境变量 GOOGLE_API_KEY 获取
        self.api_key = os.getenv('GOOGLE_API_KEY') or kwargs.get('api_key')
        self.base_url = kwargs.get('base_url')

        if not self.api_key:
            rospy.logerr("Gemini API key not found. Please set the GOOGLE_API_KEY env var or 'api_key' ROS param.")
            raise ValueError("Gemini API key is missing.")
        
        if not self.base_url:
            rospy.logerr("Gemini base_url not found. Please set the 'base_url' ROS param.")
            raise ValueError("Gemini base_url is missing.")
        
        self.headers = {
            'Content-Type': 'application/json',
        }
        self.full_url = f"{self.base_url}?key={self.api_key}"
        rospy.loginfo(f"GeminiClient (REST API) initialized for model: {self.model_name}")

    def query(self, system_prompt, user_prompt):
        """
        Queries the Gemini REST API using the 'requests' library.
        """
        start_time = time.time()
        
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]
        
        payload = {
            "model": self.model_name,
            "contents": [{"parts": [{"text": f"{system_prompt}\n{user_prompt}"}]}]
        }

        self.full_url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}:generateContent?key={self.api_key}"
        
        try:
            response = requests.post(self.full_url, headers=self.headers, json=payload, timeout=60)
            response.raise_for_status()  # 如果状态码不是2xx，则抛出异常

            duration_s = time.time() - start_time
            response_data = response.json()
            
            # 从响应中解析出需要的数据
            # 根据 generateContent 的标准响应格式
            plan_text = response_data['candidates'][0]['content']['parts'][0]['text']
            
            # REST API响应中通常不直接提供token数，需要单独API计算
            # 为保持接口统一，我们暂时返回0
            prompt_tokens = 0
            completion_tokens = 0

            return True, plan_text.strip(), "", duration_s, prompt_tokens, completion_tokens

        except requests.exceptions.RequestException as e:
            duration_s = time.time() - start_time
            error_msg = f"An error occurred with Gemini REST API: {e}"
            rospy.logerr(error_msg)
            # 尝试打印API返回的详细错误信息
            if e.response:
                rospy.logerr(f"API Response: {e.response.text}")
            return False, "", error_msg, duration_s, 0, 0
        except (KeyError, IndexError) as e:
            duration_s = time.time() - start_time
            error_msg = f"Failed to parse Gemini API response. Structure might be unexpected. Error: {e}"
            rospy.logerr(error_msg)
            rospy.logerr(f"Full Response: {response_data}")
            return False, "", error_msg, duration_s, 0, 0