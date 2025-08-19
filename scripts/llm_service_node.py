#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tello_llm_ros.srv import LLMQuery, LLMQueryResponse
from llm_models.ollama_client import OllamaClient
from llm_models.gemini_client import GeminiClient
from llm_models.openai_protocol_client import GenericOpenAIClient
from utils.llm_utils import get_system_prompts
import os

class LLMServiceNode:
    def __init__(self):
        rospy.init_node('llm_service_node')

        self.model_name = rospy.get_param("~model_name", "llama3.1:8b")
        self.model_type = rospy.get_param("~model_type", "ollama")
        self.api_key = rospy.get_param("~api_key", None)
        self.base_url = rospy.get_param("~base_url", None)
        self.enable_deep_thinking = rospy.get_param("~enable_deep_thinking", False)
        self.timeout = rospy.get_param("~timeout", 150.0)
        
        common_system_prompt_file = rospy.get_param("~common_system_prompt_file")
        tools_description_file = rospy.get_param("~tools_description_file")
        
        self.system_prompt = get_system_prompts(common_system_prompt_file, tools_description_file)
        
        rospy.loginfo(f"Model type: {self.model_type}")
        rospy.loginfo(f"Model name: {self.model_name}")
        rospy.loginfo(f"API key   : {self.api_key}")
        rospy.loginfo(f"base url  : {self.base_url}")
        
        self.model = self._load_model()
        if not self.model:
            rospy.signal_shutdown("Failed to load a valid LLM model.")
            return

        rospy.Service('~query', LLMQuery, self.handle_query)
        rospy.loginfo(f"LLM Service Node is ready, using '{self.model_type}' model: '{self.model_name}'.")

    def _load_model(self):
        model_type_lower = self.model_type.lower()
        
        # 定义所有兼容OpenAI协议的模型类型
        openai_compatible_types = ['openai', 'deepseek', 'ernie', 'qwen', 'huggingface']
        base_url = rospy.get_param("~base_url", None)

        try:
            if model_type_lower == 'ollama':
                return OllamaClient(self.model_name, timeout=self.timeout)
            elif model_type_lower in openai_compatible_types:
                # 对于所有兼容OpenAI协议的模型，统一调用通用客户端
                api_key = rospy.get_param("~api_key", None)
                return GenericOpenAIClient(
                    self.model_name,
                    api_key=api_key,
                    base_url=base_url
                )
            elif model_type_lower == 'gemini':
                api_key = rospy.get_param("~api_key", None)
                return GeminiClient(self.model_name, api_key=api_key, base_url=base_url)
            elif model_type_lower == 'custom_api':
                server_url = rospy.get_param("~server_url", None)
                return CustomApiClient(self.model_name, server_url=server_url)
            else:
                rospy.logerr(f"Unsupported model type: {self.model_type}")
                return None
        except Exception as e:
            rospy.logerr(f"Error initializing model of type '{self.model_type}': {e}")
            return None


    def handle_query(self, req):
        rospy.loginfo(f"Received query for LLM [{self.model_name}]: '{req.user_prompt}'")

        current_system_prompt = self.system_prompt
        if self.enable_deep_thinking:
            # 在系统提示词前加入引导语，让模型进行更深入的思考
            deep_thinking_prefix = "You are an expert drone pilot. Think step-by-step. First, analyze the user's command carefully. Then, break it down into a sequence of executable drone actions. Finally, format the output strictly within [START_COMMANDS] and [END_COMMANDS] blocks.\n\n"
            current_system_prompt = deep_thinking_prefix + self.system_prompt
        success, plan_text, error_message, duration_s, prompt_tokens, completion_tokens = self.model.query(
            current_system_prompt, 
            req.user_prompt, 
            history=req.history # <--- 将 history 传递下去
        )

        # success, plan_text, error_message, duration_s, prompt_tokens, completion_tokens = self.model.query(self.system_prompt, req.user_prompt)
        
        if success:
            rospy.loginfo(f"Model '{self.model_name}' process done successfully in {duration_s:.3f} seconds.")
        else:
            rospy.logwarn(f"Model '{self.model_name}' process failed. Error: {error_message}")
            
        return LLMQueryResponse(
            success=success,
            plan_text=plan_text,
            error_message=error_message,
            duration_s=duration_s,
            prompt_tokens=prompt_tokens,
            completion_tokens=completion_tokens
        )

if __name__ == '__main__':
    try:
        LLMServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass