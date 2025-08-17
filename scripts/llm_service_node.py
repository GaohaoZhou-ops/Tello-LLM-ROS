#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tello_llm_ros.srv import LLMQuery, LLMQueryResponse
from llm_models.ollama_client import OllamaClient
from utils.llm_utils import get_system_prompts
import os

class LLMServiceNode:
    def __init__(self):
        rospy.init_node('llm_service_node')

        # --- Parameters ---
        self.model_name = rospy.get_param("~model_name", "llama3")
        self.model_type = rospy.get_param("~model_type", "ollama") # 'ollama', 'huggingface', etc.
        self.timeout = rospy.get_param("~timeout", 150.0)
        
        # System prompt files
        common_system_prompt_file = rospy.get_param("~common_system_prompt_file")
        tools_description_file = rospy.get_param("~tools_description_file")
        
        # Generate and cache the system prompt
        self.system_prompt = get_system_prompts(common_system_prompt_file, tools_description_file)
        rospy.loginfo("System prompt:")
        print(self.system_prompt)
        
        self.model = self._load_model()
        if not self.model:
            rospy.signal_shutdown("Failed to load a valid LLM model.")
            return

        rospy.Service('~query', LLMQuery, self.handle_query)
        rospy.loginfo(f"LLM Service Node is ready, using '{self.model_type}' model: '{self.model_name}'.")

    def _load_model(self):
        try:
            if self.model_type.lower() == 'ollama':
                return OllamaClient(self.model_name, timeout=self.timeout)
            # elif self.model_type.lower() == 'huggingface':
            #     return HuggingFaceClient(self.model_name, api_key="...")
            else:
                rospy.logerr(f"Unsupported model type: {self.model_type}")
                return None
        except Exception as e:
            rospy.logerr(f"Error initializing model of type '{self.model_type}': {e}")
            return None

    def handle_query(self, req):
        rospy.loginfo(f"Received query for LLM: '{req.user_prompt}, sending to model {self.model_name}'")
        success, plan_text, error_message = self.model.query(self.system_prompt, req.user_prompt)
        
        rospy.loginfo(f"Model {self.model_name} process done, detail:")
        print(plan_text)
        return LLMQueryResponse(
            success=success,
            plan_text=plan_text,
            error_message=error_message
        )

if __name__ == '__main__':
    try:
        LLMServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass