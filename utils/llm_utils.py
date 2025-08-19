# -*- coding: utf-8 -*-

import json
import re
import rospy
import os

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
    

def install_and_import(package_name):
    """
    尝试导入一个包，如果失败则尝试使用 pip 安装它。
    """
    try:
        # 尝试导入
        importlib.import_module(package_name)
        print(f"'{package_name}' 已经安装。")
    except ImportError:
        print(f"'{package_name}' 未找到。正在尝试安装...")
        try:
            # 执行 pip install 命令
            subprocess.check_call([sys.executable, "-m", "pip", "install", package_name])
            print(f"'{package_name}' 安装成功。")
            # 再次尝试导入
            importlib.import_module(package_name)
        except subprocess.CalledProcessError:
            print(f"错误：安装 '{package_name}' 失败。请手动运行 'pip install {package_name}'。")
            sys.exit(1)
        except Exception as e:
            print(f"发生未知错误: {e}")
            sys.exit(1)
    
    
# Judge file type
def get_file_type(file_path: str) -> str:
    _, file_extension = os.path.splitext(file_path)
    file_extension = file_extension.lower()
    if file_extension == '.json':
        return 'json'
    elif file_extension == '.txt':
        return 'txt'
    else:
        return 'other'

# 加载txt文件
def load_txt_file(file_path:str):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()
        return content
    except Exception as e:
        print(f"发生未知错误: {e}")
        return None

# 获取系统提示词：
def get_system_prompts(prefix_file_path:str, tools_file:str):
    prefix_prompt = load_txt_file(prefix_file_path)

    # 如果文件加载失败，将prefix_prompt视为空字符串而不是None
    if prefix_prompt is None:
        rospy.logerr(f"Failed to load prefix prompt from {prefix_file_path}. Continuing without it.")
        prefix_prompt = ""
    
    if get_file_type(tools_file) == 'txt':
        subfix_prompt = load_txt_file(tools_file)
    else:   # json
        with open(tools_file, 'r') as f:
            json_contents = json.load(f)
        simplified_tools = []
        for tool in json_contents['tools']:
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
        subfix_prompt = json.dumps(simplified_tools, indent=2)
    return prefix_prompt + subfix_prompt


def parse_llm_response(plan_text, direct_parser_func=None):
    # 只匹配生成的最后一对
    regex = r".*\[START_COMMANDS\](.*?)\[END_COMMANDS\]"
    match = re.search(regex, plan_text, re.DOTALL)
    
    if match:
        command_block = match.group(1).strip()
        # rospy.loginfo("System: Found the last [START_COMMANDS] block. Parsing commands...")
        valid_commands = [cmd.strip() for cmd in command_block.split('\n') if cmd.strip()]
        rospy.loginfo("Valid commands:")
        rospy.loginfo(valid_commands)
        return valid_commands
    
    rospy.logwarn("System: Did not find a complete [START_COMMANDS]...[END_COMMANDS] block. Filtering all lines as fallback...")
    
    if not direct_parser_func:
        rospy.logerr("System: direct paraser function is None")
        return []

    valid_commands = []
    all_lines = [line.strip() for line in plan_text.split('\n') if line.strip()]
    for line in all_lines:
        tool_name, _ = direct_parser_func(line)
        if tool_name:
            valid_commands.append(line)
    return valid_commands