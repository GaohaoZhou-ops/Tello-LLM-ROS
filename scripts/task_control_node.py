#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import json
import re
from math import pi

# ROS Imports
from std_srvs.srv import Trigger, TriggerRequest
from tello_llm_ros.srv import Move, MoveRequest, LLMQuery
from tello_llm_ros.msg import ExecuteTaskAction, ExecuteTaskFeedback, ExecuteTaskResult
from tello_llm_ros.srv import TakePicture, TakePictureRequest, RecordVideo, RecordVideoRequest
from tello_llm_ros.msg import ExecuteTaskAction, ExecuteTaskFeedback, ExecuteTaskResult

# Utils
from utils.llm_utils import parse_llm_response

class TaskControlNode:
    def __init__(self):
        rospy.init_node('task_control_node')

        # --- Parameters ---
        self.drone_name = rospy.get_param("~drone_name", "tello")
        tools_config_path = rospy.get_param("~tools_config_path")
        
        # --- Load Tool Config and Connect to Drone Services ---
        with open(tools_config_path, 'r') as f:
            self.tools_config = json.load(f)
        self.service_clients = {}
        self._create_drone_service_clients()

        # --- Connect to LLM Service ---
        llm_service_name = "/llm_service_node/query"
        rospy.loginfo(f"Waiting for LLM service at '{llm_service_name}'...")
        rospy.wait_for_service(llm_service_name)
        self.llm_service_client = rospy.ServiceProxy(llm_service_name, LLMQuery)
        rospy.loginfo("Connected to LLM service.")

        # --- Action Server ---
        self.action_server = actionlib.SimpleActionServer('~execute_task', ExecuteTaskAction, self.execute_cb, auto_start=False)
        self.action_server.start()
        rospy.loginfo("Task Control Action Server is ready.")
        
    def _create_drone_service_clients(self):
        for tool in self.tools_config['tools']:
            tool_name = tool['name']
            service_name = f"/{self.drone_name}{tool['ros_service']}"
            service_type_str = tool['service_type']
            
            service_class_map = {
                'Trigger': Trigger, 
                'Move': Move,
                'TakePicture': TakePicture,
                'RecordVideo': RecordVideo
            }
            service_class = service_class_map.get(service_type_str)

            if not service_class: 
                rospy.logwarn(f"Service type '{service_type_str}' for tool '{tool_name}' is not supported. Skipping.")
                continue
            try:
                rospy.wait_for_service(service_name, timeout=3.0)
                self.service_clients[tool_name] = rospy.ServiceProxy(service_name, service_class)
                rospy.loginfo(f"Successfully connected to service '{service_name}' for tool '{tool_name}'.")
            except rospy.ROSException:
                rospy.logerr(f"Service '{service_name}' not available. Tool '{tool_name}' will be disabled.")

    def parse_direct_command(self, user_input):
        """
        Parses user input to find a matching tool and its parameters.
        It now checks for a direct match with the tool's name first, 
        then checks for direct_triggers and regex patterns.
        """
        clean_input = user_input.lower().strip().replace('_', ' ')

        for tool in self.tools_config['tools']:
            # 将工具名也处理成与clean_input相同的格式
            tool_name_cleaned = tool['name'].replace('_', ' ')
            if clean_input == tool_name_cleaned:
                params = {}
                # 如果是直接匹配，自动填充默认参数
                if 'parameters' in tool:
                    for param_def in tool['parameters']:
                        if 'default' in param_def:
                            params[param_def['name']] = param_def['default']
                return tool['name'], params

            # 检查 direct_triggers
            if 'direct_triggers' in tool:
                for trigger in tool['direct_triggers']:
                    if isinstance(trigger, str):
                        if clean_input == trigger:
                            params = {}
                            if 'parameters' in tool:
                                for p_def in tool['parameters']:
                                    if 'default' in p_def: params[p_def['name']] = p_def['default']
                            return tool['name'], params
                    elif isinstance(trigger, dict) and 'pattern' in trigger:
                        match = re.match(trigger['pattern'], clean_input, re.IGNORECASE)
                        if match:
                            params = {}
                            for p_def in trigger.get('params', []):
                                p_name = p_def['name']
                                val = float(match.group(p_def['group']))
                                unit = match.group(p_def['unit_group']) if 'unit_group' in p_def else None
                                if unit in ['cm', 'centimeters']: val /= 100.0
                                elif unit in ['deg', 'degree', 'degrees']: val *= pi / 180.0
                                params[p_name] = val
                            return tool['name'], params
        return None, None
        

    def execute_tool(self, tool_name, parameters):
        if tool_name not in self.service_clients: return False, f"Error: Tool '{tool_name}' is not available."
        client = self.service_clients[tool_name]
        tool_info = next((t for t in self.tools_config['tools'] if t['name'] == tool_name), None)
        if not tool_info: return False, f"Error: Tool '{tool_name}' not found in config."
        
        try:
            service_type = tool_info['service_type']
            request = None
            
            if service_type == 'Trigger': 
                request = TriggerRequest()
            elif service_type == 'Move':
                param_info = tool_info['parameters'][0]
                param_name = param_info['name']
                value = parameters.get(param_name, param_info.get('default'))
                if value is None: return False, f"Error: Missing parameter '{param_name}' for tool '{tool_name}'."
                request = MoveRequest(value=float(value))
            elif service_type == 'TakePicture':
                request = TakePictureRequest()
            elif service_type == 'RecordVideo':
                param_info = tool_info['parameters'][0]
                param_name = param_info['name']
                value = parameters.get(param_name, param_info.get('default'))
                if value is None: return False, f"Error: Missing parameter '{param_name}' for tool '{tool_name}'."
                request = RecordVideoRequest(duration=float(value))
            else: 
                return False, f"Error: Unknown service type '{service_type}' for '{tool_name}'."
            
            res = client(request)
            if service_type in ['TakePicture', 'RecordVideo']:
                message = f"{res.message}" if res.success else f"Failed '{tool_name}'. Msg: {res.message}"
                return res.success, message
            else:
                return res.success, res.message

        except Exception as e: 
            return False, f"An unexpected error occurred executing '{tool_name}': {e}"


    def execute_cb(self, goal):
        feedback = ExecuteTaskFeedback()
        result = ExecuteTaskResult()
        
        rospy.loginfo(f"Received new task: '{goal.user_prompt}'")
        
        # 1. Check for Direct Command
        tool_name, params = self.parse_direct_command(goal.user_prompt)
        rospy.logwarn(f"Command parased: tool_name:{tool_name}, params:{params}")
        if tool_name:
            feedback.status = f"Direct command recognized. Executing '{tool_name}'..."
            self.action_server.publish_feedback(feedback)
            
            success, message = self.execute_tool(tool_name, params)
            if success:
                result.success = True
                result.final_message = f"Direct command '{tool_name}' executed successfully. {message}"
                self.action_server.set_succeeded(result)
            else:
                result.success = False
                result.final_message = f"Direct command '{tool_name}' failed. {message}"
                self.action_server.set_aborted(result)
            return

        # 2. Call LLM Service
        feedback.status = "Command not matched directly. Querying LLM..."
        self.action_server.publish_feedback(feedback)
        
        try:
            llm_res = self.llm_service_client(user_prompt=goal.user_prompt)
            if not llm_res.success:
                result.success = False
                result.final_message = f"LLM query failed: {llm_res.error_message}"
                self.action_server.set_aborted(result)
                return
        except rospy.ServiceException as e:
            result.success = False
            result.final_message = f"Failed to call LLM service: {e}"
            self.action_server.set_aborted(result)
            return
            
        # 3. Parse and Execute Plan
        commands = parse_llm_response(llm_res.plan_text, self.parse_direct_command)
        if not commands:
            result.success = False
            result.final_message = "LLM responded, but no valid commands were found in the plan."
            self.action_server.set_aborted(result)
            return

        num_commands = len(commands)
        for i, command_line in enumerate(commands):
            if self.action_server.is_preempt_requested():
                result.success = False
                result.final_message = "Task preempted by client."
                self.action_server.set_preempted(result)
                return

            feedback.status = f"Executing step {i+1}/{num_commands}: '{command_line}'"
            self.action_server.publish_feedback(feedback)
            
            tool_name, params = self.parse_direct_command(command_line)
            if not tool_name:
                rospy.logwarn(f"Could not parse command from LLM plan: '{command_line}'. Skipping.")
                continue

            success, message = self.execute_tool(tool_name, params)
            if not success:
                result.success = False
                result.final_message = f"Execution failed at step {i+1} ('{command_line}'). Reason: {message}"
                self.action_server.set_aborted(result)
                return
            rospy.sleep(1.0)

        result.success = True
        result.final_message = "Mission complete. All steps executed successfully."
        self.action_server.set_succeeded(result)


if __name__ == '__main__':
    try:
        TaskControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass