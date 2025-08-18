# -*- coding: utf-8 -*-

import rospy
import os
import time
from openai import OpenAI
from .base import LLMBase

class ErnieClient(LLMBase):
    """
    Baidu ERNIE Bot (via OpenAI-compatible Qianfan endpoint) implementation of the LLMBase.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the OpenAI client to connect to Baidu's Qianfan endpoint.
        """
        # 优先从环境变量 BAIDU_QIANFAN_TOKEN 获取凭证
        api_key = os.getenv('BAIDU_QIANFAN_TOKEN') or kwargs.get('api_key')
        
        # 从ROS参数获取千帆的base_url
        base_url = kwargs.get('base_url')
        
        rospy.logwarn(f"\tbase_url: {base_url}")
        rospy.logwarn(f"\tAPI Key:  {api_key}")

        if not api_key:
            rospy.logerr("Baidu Qianfan API Key not found. Please set the BAIDU_QIANFAN_TOKEN env var or 'api_key' ROS param.")
            raise ValueError("Qianfan API Key is missing.")
        
        if not base_url:
            rospy.logerr("Baidu Qianfan base_url not found. Please set the 'base_url' ROS param.")
            raise ValueError("Qianfan base_url is missing.")

        try:
            # 使用获取到的凭证和URL初始化OpenAI客户端
            self.client = OpenAI(
                api_key=api_key,
                base_url=base_url
            )
            rospy.loginfo(f"ErnieClient initialized for model '{self.model_name}' via Qianfan endpoint.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize OpenAI client for Qianfan: {e}")
            raise

    def query(self, system_prompt, user_prompt):
        """
        Queries the Qianfan Chat Completions API using the OpenAI library.
        """
        start_time = time.time()
        
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages
            )
            duration_s = time.time() - start_time

            plan_text = response.choices[0].message.content
            usage = response.usage
            prompt_tokens = usage.prompt_tokens if usage else 0
            completion_tokens = usage.completion_tokens if usage else 0

            return True, plan_text.strip(), "", duration_s, prompt_tokens, completion_tokens
        except Exception as e:
            duration_s = time.time() - start_time
            error_msg = f"An unexpected error occurred with Qianfan API: {e}"
            rospy.logerr(error_msg)
            return False, "", error_msg, duration_s, 0, 0
        

    