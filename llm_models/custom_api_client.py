# -*- coding: utf-8 -*-

import rospy
import requests
import time
from .base import LLMBase

class CustomApiClient(LLMBase):
    """
    Client for a custom, self-hosted model API.
    """
    def _initialize(self, **kwargs):
        self.server_url = kwargs.get('server_url')
        if not self.server_url:
            rospy.logerr("Custom API server URL is not provided. Please set the 'server_url' parameter.")
            raise ValueError("server_url is missing.")
        
        self.api_endpoint = f"{self.server_url.rstrip('/')}/v1/chat/completions"
        self.headers = {'Content-Type': 'application/json'}
        rospy.loginfo(f"CustomApiClient initialized. Target server: {self.api_endpoint}")

    def query(self, system_prompt, user_prompt):
        payload = {
            "model": self.model_name,
            "system_prompt": system_prompt,
            "user_prompt": user_prompt
        }
        
        try:
            response = requests.post(self.api_endpoint, headers=self.headers, json=payload, timeout=60)
            response.raise_for_status()
            
            response_data = response.json()

            if response_data.get("success"):
                data = response_data.get("data", {})
                plan_text = data.get("plan_text", "")
                duration_s = data.get("duration_s", 0.0)
                usage = data.get("usage", {})
                prompt_tokens = usage.get("prompt_tokens", 0)
                completion_tokens = usage.get("completion_tokens", 0)
                return True, plan_text, "", duration_s, prompt_tokens, completion_tokens
            else:
                error_msg = response_data.get("error_message", "Unknown error from custom API.")
                rospy.logerr(error_msg)
                return False, "", error_msg, 0.0, 0, 0

        except requests.exceptions.RequestException as e:
            error_msg = f"Failed to connect to custom model server at {self.api_endpoint}: {e}"
            rospy.logerr(error_msg)
            return False, "", error_msg, 0.0, 0, 0