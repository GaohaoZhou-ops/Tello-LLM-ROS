# -*- coding: utf-8 -*-

import rospy
import os
import time
from baidu-aip import AipNlp
from .base import LLMBase

class ErnieClient(LLMBase):
    """
    Baidu ERNIE Bot API implementation of the LLMBase.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the AipNlp client using APP_ID, API_KEY, and SECRET_KEY.
        """
        # 优先从环境变量获取凭证，这更安全
        self.app_id = os.getenv('BAIDU_APP_ID') or kwargs.get('app_id')
        self.api_key = os.getenv('BAIDU_API_KEY') or kwargs.get('api_key')
        self.secret_key = os.getenv('BAIDU_SECRET_KEY') or kwargs.get('secret_key')

        if not all([self.app_id, self.api_key, self.secret_key]):
            msg = "Baidu API credentials (APP_ID, API_KEY, SECRET_KEY) not found."
            rospy.logerr(msg + " Please set environment variables or provide them as ROS params.")
            raise ValueError(msg)
        
        try:
            # 初始化AipNlp对象
            self.client = AipNlp(self.app_id, self.api_key, self.secret_key)
            rospy.loginfo(f"ErnieClient initialized for model: {self.model_name}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Baidu AipNlp client: {e}")
            raise

    def query(self, system_prompt, user_prompt):
        """
        Queries the ERNIE Bot API.
        """
        start_time = time.time()
        
        # 文心一言的 ernieBot 接口接收单个 'text' 输入，我们将 system 和 user prompt 组合起来
        # 使用分隔符让模型能更好地区分两部分
        full_prompt = f"{system_prompt}\n\n---\n\nUser Command: {user_prompt}"
        
        options = {
            'text': full_prompt,
        }

        try:
            # 发送请求并获取响应
            result = self.client.ernieBot(options)
            duration_s = time.time() - start_time

            # 检查API调用是否成功
            if result.get('error_code') != 0:
                error_msg = result.get('error_msg', 'Unknown error from ERNIE API.')
                rospy.logerr(error_msg)
                return False, "", error_msg, duration_s, 0, 0

            # 提取模型返回的内容和token使用情况
            plan_text = result.get('result', '')
            
            # Baidu API通常在结果中包含一个 'usage' 字段
            usage = result.get('usage', {})
            prompt_tokens = usage.get('prompt_tokens', 0)
            completion_tokens = usage.get('completion_tokens', 0)

            return True, plan_text, "", duration_s, prompt_tokens, completion_tokens

        except Exception as e:
            duration_s = time.time() - start_time
            error_msg = f"An unexpected error occurred with ERNIE API: {e}"
            rospy.logerr(error_msg)
            return False, "", error_msg, duration_s, 0, 0