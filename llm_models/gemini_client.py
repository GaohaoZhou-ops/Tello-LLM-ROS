# -*- coding: utf-8 -*-

import rospy
import os
import time
import google.generativeai as genai
from .base import LLMBase
from utils.llm_utils import install_and_import

class GeminiClient(LLMBase):
    """
    Google Gemini API implementation of the LLMBase.
    """
    def _initialize(self, **kwargs):
        # 优先从环境变量 GOOGLE_API_KEY 获取，这是官方推荐的做法
        api_key = os.getenv('GOOGLE_API_KEY')
        if not api_key:
            api_key = kwargs.get('api_key')

        if not api_key:
            rospy.logerr("Gemini API key not found. Please set the GOOGLE_API_KEY environment variable or provide it as a ROS param.")
            raise ValueError("Gemini API key is missing.")
        
        try:
            genai.configure(api_key=api_key)
            rospy.loginfo(f"GeminiClient initialized for model: {self.model_name}")
        except Exception as e:
            rospy.logerr(f"Failed to configure Gemini API: {e}")
            raise

    def query(self, system_prompt, user_prompt):
        start_time = time.time()
        duration_s = 0
        prompt_tokens = 0
        completion_tokens = 0

        try:
            # 初始化模型，并将 system_prompt 作为系统指令
            model = genai.GenerativeModel(
                model_name=self.model_name,
                system_instruction=system_prompt
            )

            # 1. 生成内容
            response = model.generate_content(user_prompt)
            plan_text = response.text
            
            # 2. 计算Token数 (Gemini需要额外调用API)
            prompt_tokens_response = model.count_tokens(user_prompt)
            prompt_tokens = prompt_tokens_response.total_tokens

            completion_tokens_response = model.count_tokens(plan_text)
            completion_tokens = completion_tokens_response.total_tokens

            duration_s = time.time() - start_time
            return True, plan_text, "", duration_s, prompt_tokens, completion_tokens

        except Exception as e:
            duration_s = time.time() - start_time
            error_msg = f"An error occurred with Gemini API: {e}"
            rospy.logerr(error_msg)
            return False, "", error_msg, duration_s, prompt_tokens, completion_tokens