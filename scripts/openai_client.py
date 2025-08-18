# -*- coding: utf-8 -*-

import rospy
import os
import time
from openai import OpenAI
from .base import LLMBase

class OpenAIClient(LLMBase):
    """
    OpenAI GPT API implementation of the LLMBase.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the OpenAI client.
        """
        # 优先从环境变量 OPENAI_API_KEY 获取，这是官方推荐的做法
        api_key = os.getenv('OPENAI_API_KEY')
        if not api_key:
            api_key = kwargs.get('api_key')

        if not api_key:
            rospy.logerr("OpenAI API key not found. Please set the OPENAI_API_KEY environment variable or provide it as a ROS param.")
            raise ValueError("OpenAI API key is missing.")
        
        try:
            self.client = OpenAI(api_key=api_key)
            rospy.loginfo(f"OpenAIClient initialized for model: {self.model_name}")
        except Exception as e:
            rospy.logerr(f"Failed to initialize OpenAI client: {e}")
            raise

    def query(self, system_prompt, user_prompt):
        """
        Queries the OpenAI Chat Completions API.
        """
        start_time = time.time()
        
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt}
        ]

        try:
            # 发送请求并获取响应
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages
            )
            duration_s = time.time() - start_time

            # 提取模型返回的内容和token使用情况
            plan_text = response.choices[0].message.content
            
            usage = response.usage
            prompt_tokens = usage.prompt_tokens if usage else 0
            completion_tokens = usage.completion_tokens if usage else 0

            return True, plan_text.strip(), "", duration_s, prompt_tokens, completion_tokens

        except Exception as e:
            duration_s = time.time() - start_time
            error_msg = f"An unexpected error occurred with OpenAI API: {e}"
            rospy.logerr(error_msg)
            return False, "", error_msg, duration_s, 0, 0