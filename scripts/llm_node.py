#!/usr/bin/env python3

import rospy
import ollama
import yaml
import json
import re
import time
from math import pi
from std_srvs.srv import Trigger, TriggerRequest
from tello_llm_ros.srv import Move, MoveRequest
import os
import subprocess

from llm_utils import create_system_prompt, parse_llm_response, load_pure_system_prompt, get_file_type

subprocess.run(["export", "LC_ALL='en_US.UTF-8'"], shell=True)

class TelloLLMController:
    def __init__(self):
        rospy.init_node('tello_llm_controller')
        self.drone_name = rospy.get_param("~drone_name", "tello") 
        self.ollama_model = rospy.get_param("~ollama_model", "llama3")
        tools_config_path = rospy.get_param("~tools_config_path")

        self.ollama_client = ollama.Client()
        
        if get_file_type(tools_config_path) == 'txt':
            rospy.logwarn(f"System prompt is pure text file, loading...")
            self.system_prompt = load_pure_system_prompt(tools_config_path)
        else:
            rospy.logwarn(f"System prompt is json file, parasing and creating...")
            with open(tools_config_path, 'r') as f: self.tools_config = json.load(f)
            rospy.loginfo(f"Loaded {len(self.tools_config['tools'])} tools from config.")
            self.system_prompt = create_system_prompt(self.tools_config)
            
        self.messages = [{'role': 'system', 'content': self.system_prompt}]
        self.service_clients = {}
        self._create_service_clients()
        rospy.loginfo("Tello LLM Controller is ready (Hybrid Mode, Smart Defaults).")    
    
    def _create_service_clients(self):
        for tool in self.tools_config['tools']:
            tool_name = tool['name']
            service_name = f"/{self.drone_name}{tool['ros_service']}"
            service_type_str = tool['service_type']
            service_class = {'Trigger': Trigger, 'Move': Move}.get(service_type_str)
            if not service_class: continue
            try:
                rospy.wait_for_service(service_name, timeout=3.0)
                self.service_clients[tool_name] = rospy.ServiceProxy(service_name, service_class)
                rospy.loginfo(f"Successfully connected to service '{service_name}' for tool '{tool_name}'.")
            except rospy.ROSException:
                rospy.logerr(f"Service '{service_name}' not available. Tool '{tool_name}' will be disabled.")



    def parse_direct_command(self, user_input):
        """
        增强版的直接命令解析器。
        当匹配到简单关键词时，会自动应用默认参数。
        """
        clean_input = user_input.lower().strip()
        for tool in self.tools_config['tools']:
            if 'direct_triggers' not in tool:
                continue

            for trigger in tool['direct_triggers']:
                # 方案1: 简单的关键词匹配 (增强版)
                if isinstance(trigger, str):
                    if clean_input == trigger:
                        tool_name = tool['name']
                        params = {}
                        if 'parameters' in tool:
                            for param_def in tool['parameters']:
                                if 'default' in param_def:
                                    params[param_def['name']] = param_def['default']
                        return tool_name, params

                # 方案2: 正则表达式匹配 (保持不变)
                elif isinstance(trigger, dict) and 'pattern' in trigger:
                    match = re.match(trigger['pattern'], clean_input, re.IGNORECASE)
                    if match:
                        params = {}
                        for param_def in trigger.get('params', []):
                            param_name = param_def['name']
                            value = float(match.group(param_def['group']))
                            unit = match.group(param_def['unit_group']) if 'unit_group' in param_def else None
                            if unit in ['cm', 'centimeters']: value /= 100.0
                            elif unit in ['deg', 'degree', 'degrees']: value *= pi / 180.0
                            params[param_name] = value
                        return tool['name'], params
        
        return None, None
    
    def execute_tool(self, tool_name, parameters):
        if tool_name not in self.service_clients: return f"Error: Tool '{tool_name}' is not available or not connected."
        client = self.service_clients[tool_name]
        tool_info = next((t for t in self.tools_config['tools'] if t['name'] == tool_name), None)
        if not tool_info: return f"Error: Tool '{tool_name}' is defined but not found in config."
        try:
            if tool_info['service_type'] == 'Trigger': request = TriggerRequest()
            elif tool_info['service_type'] == 'Move':
                if not tool_info['parameters']: return f"Error: Tool '{tool_name}' is 'Move' type but has no parameters in config."
                param_info = tool_info['parameters'][0]
                param_name = param_info['name']
                value = parameters.get(param_name)
                if value is None:
                    if 'default' in param_info:
                        value = param_info['default']
                        print(f"System: Parameter '{param_name}' was not provided. Using default value: {value}")
                    else:
                        return f"Error: Missing parameter '{param_name}' for tool '{tool_name}' and no default is set."
                request = MoveRequest(value=float(value))
            else: return f"Error: Cannot execute unknown service type for tool '{tool_name}'."
            response = client(request)
            return f"OK! Executed '{tool_name}'. Msg: {response.message}" if response.success else f"Failed '{tool_name}'. Msg: {response.message}"
        except rospy.ServiceException as e: return f"Error calling service for '{tool_name}': {e}"
        except Exception as e: return f"An unexpected error occurred while executing '{tool_name}': {e}"


    def run(self):
        while not rospy.is_shutdown():
            try:
                user_input = input("You: ")
                if not user_input.strip():
                    continue
                if user_input.lower() in ['exit', 'quit']:
                    break

                tool_name, params = self.parse_direct_command(user_input)
                if tool_name:
                    print(f"System: Direct command recognized -> '{tool_name}' with params: {params}")
                    execution_result = self.execute_tool(tool_name, params)
                    print(f"System: {execution_result}")
                else:
                    print("System: Command not matched directly, consulting LLM...")
                    
                    messages_for_this_turn = [
                        {'role': 'system', 'content': self.system_prompt},
                        {'role': 'user', 'content': user_input}
                    ]

                    start_time = time.perf_counter()
                    response = self.ollama_client.chat(
                        model=self.ollama_model,
                        messages=messages_for_this_turn,
                    )
                    end_time = time.perf_counter()
                    
                    duration = end_time - start_time
                    prompt_tokens, completion_tokens = response.get('prompt_eval_count', 0), response.get('eval_count', 0)
                    print("├─ LLM Stats:")
                    print(f"        Time: {duration:.3f}s")
                    print(f"        Tokens: {completion_tokens + prompt_tokens} (p: {prompt_tokens}, c: {completion_tokens})")
                    print(f"        Speed: {(completion_tokens+prompt_tokens) / duration:.2f} tokens/s")

                    plan_text = response['message']['content'] 
                    valid_commands = parse_llm_response(plan_text, self.parse_direct_command)
                    if not valid_commands:
                        print("System: No valid commands found in the LLM response. Please try again.")
                        continue
                        
                    print(f"System: Parse complete. Found {len(valid_commands)} valid commands. Executing...")
                    for i, command_line in enumerate(valid_commands):
                        if rospy.is_shutdown():
                            print("Mission aborted due to ROS shutdown.")
                            break
                        
                        print(f"--- Step {i+1}/{len(valid_commands)}: '{command_line}' ---")
                        
                        step_tool, step_params = self.parse_direct_command(command_line)
                        
                        if step_tool:
                            print(f"Executing: '{step_tool}' with params: {step_params}")
                            execution_result = self.execute_tool(step_tool, step_params)
                            print(f"Result: {execution_result}")
                            
                            if "Failed" in execution_result:
                                print("Mission aborted due to a step failure.")
                                break
                            
                            rospy.sleep(3)
                        else:
                            print(f"Warning: Command '{command_line}' failed final parsing. Skipping.")
                    
                    print("--- Mission Complete ---")

            except KeyboardInterrupt:
                print("\nExiting.")
                break




            
if __name__ == '__main__':
    from tello_llm_ros.srv import Move, MoveRequest
    try:
        controller = TelloLLMController()
        controller.run()
    except rospy.ROSInterruptException: pass