#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import ollama
import json
import re
import os
from llm_utils import create_system_prompt, parse_llm_response, bcolors

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

class LLMOfflineTester:
    def __init__(self):
        rospy.init_node('tello_llm_offline_tester')
        
        self.ollama_model = rospy.get_param("~ollama_model", "llama3.1:8b")
        tools_config_path = rospy.get_param("~tools_config_path")
        if not os.path.exists(tools_config_path):
            rospy.logerr(f"Tools config file not found at: {tools_config_path}")
            rospy.signal_shutdown("Tools config file not found.")
            return

        with open(tools_config_path, 'r') as f:
            self.tools_config = json.load(f)
        rospy.loginfo(f"Successfully loaded {len(self.tools_config['tools'])} tools from {tools_config_path}")

        try:
            self.ollama_client = ollama.Client()
            self.ollama_client.list() 
        except Exception as e:
            rospy.logerr(f"Failed to connect to Ollama client. Please ensure Ollama is running. Error: {e}")
            return
            
        self.system_prompt = create_system_prompt(self.tools_config)
        self.test_cases = self._define_test_cases()
        rospy.loginfo(f"LLM Offline Tester is ready. Model: {self.ollama_model}")

        # static
        self.g_total_duration = 0.0
        self.g_total_tokens = 0

    def _define_test_cases(self):
        """
        Defines a list of 20 test cases with prompts and their expected command outputs.
        """
        return [
            # 1. Basic Single Actions
            {"prompt": "Take off.", "expected_commands": ["takeoff"]},
            {"prompt": "Land the drone.", "expected_commands": ["land"]},
            {"prompt": "向前飞1米", "expected_commands": ["move_forward 1m"]},
            {"prompt": "向后飞50厘米", "expected_commands": ["move_backward 0.5m"]},
            
            # 2. Simple Sequences
            {"prompt": "Take off, go up 1 meter, then land.", "expected_commands": ["takeoff", "move_up 1m", "land"]},
            {"prompt": "先起飞, 然后向左平移半米, 最后降落", "expected_commands": ["takeoff", "move_left 0.5m", "land"]},
            
            # 3. Rotations
            {"prompt": "Turn right 90 degrees.", "expected_commands": ["rotate_clockwise 90 degrees"]},
            {"prompt": "向左旋转180度", "expected_commands": ["rotate_counter_clockwise 180 degrees"]},
            
            # 4. More Complex Sequences
            {"prompt": "Fly a triangle with 1.5m sides.", "expected_commands": ["move_forward 1.5m", "rotate_clockwise 120 degrees", "move_forward 1.5m", "rotate_clockwise 120 degrees", "move_forward 1.5m", "rotate_clockwise 120 degrees"]},
            {"prompt": "飞一个1米边长的等边三角形", "expected_commands": ["move_forward 1m", "rotate_clockwise 120 degrees", "move_forward 1m", "rotate_clockwise 120 degrees", "move_forward 1m", "rotate_clockwise 120 degrees"]},
            
            # 5. Up/Down Sequences
            {"prompt": "Fly up 2 meters, then come down 1 meter.", "expected_commands": ["move_up 2m", "move_down 1m"]},
            {"prompt": "做一个“上上下下”的动作，每次半米", "expected_commands": ["move_up 0.5m", "move_down 0.5m", "move_up 0.5m", "move_down 0.5m"]},
            
            # 6. Rectangular Path
            {"prompt": "Fly a rectangle 2m long and 1m wide.", "expected_commands": ["move_forward 2m", "rotate_clockwise 90 degrees", "move_forward 1m", "rotate_clockwise 90 degrees", "move_forward 2m", "rotate_clockwise 90 degrees", "move_forward 1m", "rotate_clockwise 90 degrees"]},
            {"prompt": "飞一个长2米，宽1米的长方形", "expected_commands": ["move_forward 2m", "rotate_clockwise 90 degrees", "move_forward 1m", "rotate_clockwise 90 degrees", "move_forward 2m", "rotate_clockwise 90 degrees", "move_forward 1m", "rotate_clockwise 90 degrees"]},
            
            # 7. Back and Forth
            {"prompt": "Go forward 3 meters, then return to start.", "expected_commands": ["move_forward 3m", "move_backward 3m"]},
            {"prompt": "向前飞3米，然后飞回来", "expected_commands": ["move_forward 3m", "move_backward 3m"]},
            
            # 8. Staircase Pattern
            {"prompt": "Fly a staircase pattern: go up 50cm, forward 50cm, three times.", "expected_commands": ["move_up 0.5m", "move_forward 0.5m", "move_up 0.5m", "move_forward 0.5m", "move_up 0.5m", "move_forward 0.5m"]},
            {"prompt": "飞一个三级的楼梯，每级高半米，长半米", "expected_commands": ["move_up 0.5m", "move_forward 0.5m", "move_up 0.5m", "move_forward 0.5m", "move_up 0.5m", "move_forward 0.5m"]},

            # 9. Rotational Sequence
            {"prompt": "Look left, then right, then center.", "expected_commands": ["rotate_counter_clockwise 90 degrees", "rotate_clockwise 180 degrees", "rotate_counter_clockwise 90 degrees"]},
            {"prompt": "先向左看，再向右看，最后回到前面", "expected_commands": ["rotate_counter_clockwise 90 degrees", "rotate_clockwise 180 degrees", "rotate_counter_clockwise 90 degrees"]}
        ]


    def run_tests(self):
        """
        Executes all defined test cases against the LLM and reports the results,
        including token speed statistics for each test.
        """
        if not hasattr(self, 'ollama_client'):
            rospy.logerr("Ollama client not initialized. Aborting tests.")
            return

        total_tests = len(self.test_cases)
        passed_tests = 0

        rospy.loginfo(f"{bcolors.HEADER}--- Starting LLM Offline Test Suite ---{bcolors.ENDC}")
        rospy.loginfo(f"Model: {bcolors.BOLD}{self.ollama_model}{bcolors.ENDC}")
        rospy.loginfo(f"Total test cases: {total_tests}\n")

        for i, case in enumerate(self.test_cases):
            rospy.loginfo(f"{bcolors.OKBLUE}--- Test {i+1}/{total_tests} ---{bcolors.ENDC}")
            rospy.loginfo(f"{bcolors.BOLD}Prompt:{bcolors.ENDC} {case['prompt']}")

            messages = [
                {'role': 'system', 'content': self.system_prompt},
                {'role': 'user', 'content': case['prompt']}
            ]

            try:
                response = self.ollama_client.chat(
                    model=self.ollama_model,
                    messages=messages
                )
                plan_text = response['message']['content']
                
                prompt_tokens = response.get('prompt_eval_count', 0)
                completion_tokens = response.get('eval_count', 0)
                total_tokens = prompt_tokens + completion_tokens
                self.g_total_tokens += total_tokens

                duration_ns = response.get('total_duration', 1)
                duration_s = duration_ns / 1_000_000_000
                self.g_total_duration += duration_s
                
                tokens_per_second = total_tokens / duration_s if duration_s > 0 else 0
                
                stats_str = (f"Time: {duration_s:.2f}s | "
                             f"Speed: {tokens_per_second:.2f} t/s | "
                             f"Tokens: {total_tokens} (p: {prompt_tokens}, c: {completion_tokens})")
                rospy.loginfo(f"{bcolors.HEADER}LLM Stats:{bcolors.ENDC} {stats_str}")

                actual_commands = parse_llm_response(plan_text)
                is_pass = (actual_commands == case['expected_commands'])

                if is_pass:
                    passed_tests += 1
                    rospy.loginfo(f"{bcolors.OKGREEN}{bcolors.BOLD}Result: PASS{bcolors.ENDC}")
                    rospy.loginfo(f"{bcolors.OKCYAN}Generated Commands:{bcolors.ENDC}\n" + "\n".join(actual_commands))
                else:
                    rospy.loginfo(f"{bcolors.FAIL}{bcolors.BOLD}Result: FAIL{bcolors.ENDC}")
                    rospy.loginfo(f"{bcolors.WARNING}Expected Commands:{bcolors.ENDC}\n" + "\n".join(case['expected_commands']))
                    rospy.loginfo(f"{bcolors.OKCYAN}Generated Commands:{bcolors.ENDC}\n" + "\n".join(actual_commands))
                    # rospy.loginfo(f"{bcolors.FAIL}Full LLM Output:\n{plan_text}{bcolors.ENDC}")

            except Exception as e:
                rospy.logerr(f"An error occurred during test case {i+1}: {e}")

            rospy.loginfo("\n") 

        pass_rate = (passed_tests / total_tests) * 100
        color = bcolors.OKGREEN if pass_rate > 80 else bcolors.WARNING if pass_rate > 50 else bcolors.FAIL
        
        rospy.loginfo(f"{bcolors.HEADER}--- Test Suite Complete ---{bcolors.ENDC}")
        rospy.loginfo(f"Summary: ")
        rospy.loginfo(f"        Pass Rate                   : {passed_tests}/{total_tests} tests passed ({color}{pass_rate:.2f}%{bcolors.ENDC})")
        rospy.loginfo(f"        Total Cost                  : {self.g_total_duration:.3} seconds.")
        rospy.loginfo(f"        Average Token Generate Speed: {self.g_total_tokens / self.g_total_duration:.5} tokens/s")
        rospy.loginfo(f"        Average Task Cost Time      : {self.g_total_duration / len(self.test_cases):.5} seconds.")

if __name__ == '__main__':
    try:
        tester = LLMOfflineTester()
        rospy.sleep(0.5) 
        if not rospy.is_shutdown():
            tester.run_tests()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        if "Tools config file not found" not in str(e):
             rospy.logerr(f"An unexpected error occurred during initialization: {e}")
