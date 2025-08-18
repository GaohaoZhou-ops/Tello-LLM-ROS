#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tello_llm_ros.srv import LLMQuery, LLMQueryResponse
from llm_models.ollama_client import OllamaClient
from llm_models.deepseek_client import DeepseekClient
from llm_models.gemini_client import GeminiClient
from utils.llm_utils import get_system_prompts
import os

class LLMServiceNode:
    def __init__(self):
        rospy.init_node('llm_service_node')

        self.model_name = rospy.get_param("~model_name", "llama3.1:8b")
        self.model_type = rospy.get_param("~model_type", "ollama")
        self.api_key = rospy.get_param("~api_key", None)
        self.timeout = rospy.get_param("~timeout", 150.0)
        
        common_system_prompt_file = rospy.get_param("~common_system_prompt_file")
        tools_description_file = rospy.get_param("~tools_description_file")
        
        self.system_prompt = get_system_prompts(common_system_prompt_file, tools_description_file)
        
        self.model = self._load_model()
        if not self.model:
            rospy.signal_shutdown("Failed to load a valid LLM model.")
            return

        rospy.Service('~query', LLMQuery, self.handle_query)
        rospy.loginfo(f"LLM Service Node is ready, using '{self.model_type}' model: '{self.model_name}'.")
        rospy.loginfo(f"API Key: {self.api_key}")

    def _load_model(self):
        try:
            if self.model_type.lower() == 'ollama':
                return OllamaClient(self.model_name, timeout=self.timeout)
            elif self.model_type.lower() == 'deepseek':
                return DeepseekClient(self.model_name, api_key=self.api_key)
            elif self.model_type.lower() == 'gemini':
                return GeminiClient(self.model_name, api_key=self.api_key)
            else:
                rospy.logerr(f"Unsupported model type: {self.model_type}")
                return None
        except Exception as e:
            rospy.logerr(f"Error initializing model of type '{self.model_type}': {e}")
            return None

    def handle_query(self, req):
        rospy.loginfo(f"Received query for LLM: '{req.user_prompt}', sending to model {self.model_name}'")
        success, plan_text, error_message, duration_s, prompt_tokens, completion_tokens = self.model.query(self.system_prompt, req.user_prompt)
        
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