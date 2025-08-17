# -*- coding: utf-8 -*-

import rospy
import ollama
from .base import LLMBase

class OllamaClient(LLMBase):
    """
    Ollama implementation of the LLMBase.
    Connects to a local Ollama server to get model responses.
    """
    def _initialize(self, **kwargs):
        """
        Initializes the Ollama client.
        kwargs can include 'timeout'.
        """
        self.timeout = kwargs.get('timeout', 150.0)
        try:
            self.client = ollama.Client(timeout=self.timeout)
            self.client.list()
            rospy.loginfo(f"Successfully connected to Ollama client. Model: {self.model_name}")
        except Exception as e:
            rospy.logfatal(f"Failed to connect to Ollama. Is the server running? Error: {e}")
            raise

    def query(self, system_prompt, user_prompt):
        """
        Queries the Ollama model.
        """
        messages = [
            {'role': 'system', 'content': system_prompt},
            {'role': 'user', 'content': user_prompt}
        ]
        try:
            response = self.client.chat(
                model=self.model_name,
                messages=messages,
            )
            plan_text = response['message']['content']
            duration_ns = response.get('total_duration', 0)
            duration_s = duration_ns / 1_000_000_000.0
            prompt_tokens = response.get('prompt_eval_count', 0)
            completion_tokens = response.get('eval_count', 0)
            return True, plan_text, "", duration_s, prompt_tokens, completion_tokens

        except Exception as e:
            error_msg = f"Failed to query Ollama model '{self.model_name}': {e}"
            rospy.logerr(error_msg)
            return False, "", str(e), 0.0, 0, 0