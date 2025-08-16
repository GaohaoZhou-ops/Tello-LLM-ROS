# -*- coding: utf-8 -*-

import json
import re
import rospy

# Ruels: 
# - First, think step-by-step about the user's request inside <think></think> tags.
# - After your thinking process, you MUST provide the final command sequence.

# ANSI color codes for terminal
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def create_system_prompt(tools_config):
    """
    Dynamically builds the system prompt using the loaded tools config.
    This function is now the single source of truth for the system prompt.
    
    :param tools_config: The loaded tools configuration dictionary.
    :return: The formatted system prompt string.
    """
    prompt = """You are a simple robot command translator. Your ONLY job is to break down a user's request into a list of simple, one-line text commands for a drone, based on the tools provided.

**RULES:**
- **CRITICAL**: The final command sequence MUST be enclosed between `[START_COMMANDS]` and `[END_COMMANDS]` tags.
- Inside the tags, output ONLY the command text, with each command on a new line.
- By default, the drone is already in the air and no additional takeoff call is required, unless there is a clear takeoff instruction.
- Unless there is a clear landing instruction, the land command cannot be called.

**Here are the available tools you can use to form your commands:**
"""
    simplified_tools = []
    for tool in tools_config['tools']:
        simplified_tool = {
            "name": tool.get("name"),
            "description": tool.get("description"),
            "parameters": []
        }
        if "parameters" in tool:
            for param in tool["parameters"]:
                simplified_param = {
                    "name": param.get("name"),
                    "type": param.get("type"),
                    "description": param.get("description")
                }
                if "default" in param:
                    simplified_param["default"] = param.get("default")
                simplified_tool["parameters"].append(simplified_param)
        
        simplified_tools.append(simplified_tool)

    prompt += json.dumps(simplified_tools, indent=2)
    
    prompt += """

**EXAMPLE 1:**
User: Fly a 2m square
Your output:
<think>The user wants a square with 2m sides. I should use the available tools. This requires a takeoff, four forward movements, four 90-degree right turns, and a landing to complete the mission.</think>
[START_COMMANDS]
takeoff
move_forward 2m
rotate_clockwise 90 degrees
move_forward 2m
rotate_clockwise 90 degrees
move_forward 2m
rotate_clockwise 90 degrees
move_forward 2m
rotate_clockwise 90 degrees
land
[END_COMMANDS]

**EXAMPLE 2:**
User: go up 1m then come down
Your output:
<think>The user wants to move up and then down. This is two simple commands based on the 'move_up' and 'move_down' tools.</think>
[START_COMMANDS]
move_up 1m
move_down 1m
[END_COMMANDS]
"""
    return prompt

def parse_llm_response(plan_text, direct_parser_func=None):
    """
    Parses the full text response from the LLM to extract a clean list of valid commands.
    
    :param plan_text: The raw string output from the LLM.
    :param direct_parser_func: A reference to a node's `parse_direct_command` function, used for fallback.
    :return: A list of strings, where each string is a valid command.
    """
    # Use re.findall to get all occurrences and take the last one.
    # 只提取正则表达式中最后一对
    matches = re.findall(r"\[START_COMMANDS\](.*?)\[END_COMMANDS\]", plan_text, re.DOTALL)
    
    if matches:
        command_block = matches[-1].strip()
        rospy.loginfo("System: Found [START_COMMANDS] block. Parsing commands...")
        valid_commands = [cmd.strip() for cmd in command_block.split('\n') if cmd.strip()]
        return valid_commands
    
    rospy.logwarn("System: Did not find [START_COMMANDS] block. Filtering all lines as fallback...")
    
    if not direct_parser_func:
        return []

    valid_commands = []
    all_lines = [line.strip() for line in plan_text.split('\n') if line.strip()]
    for line in all_lines:
        tool_name, _ = direct_parser_func(line)
        if tool_name:
            valid_commands.append(line)
            
    return valid_commands