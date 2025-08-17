#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import ollama
import json
import re
import os
from utils.llm_utils import get_system_prompts, parse_llm_response, bcolors

class LLMOfflineTester:
    def __init__(self):
        rospy.init_node('tello_llm_offline_tester')
        
        self.ollama_model = rospy.get_param("~ollama_model", "llama3.1:8b")
        common_system_prompt_file = rospy.get_param("~common_system_prompt_file")
        tools_description_file = rospy.get_param("~tools_description_file")
        self.inference_timeout = rospy.get_param("~inference_timeout", 150.0)
        test_cases_file = rospy.get_param("~test_cases_file")

        # --- 检查配置文件是否存在 ---
        for f in [common_system_prompt_file, tools_description_file, test_cases_file]:
            if not os.path.exists(f):
                rospy.logerr(f"Configuration file not found at: {f}")
                rospy.signal_shutdown("Missing configuration file.")
                return
        
        # --- 加载和初始化 ---
        self.system_prompt = get_system_prompts(common_system_prompt_file, tools_description_file)

        try:
            self.ollama_client = ollama.Client(timeout=self.inference_timeout)
            self.ollama_client.list() 
        except Exception as e:
            rospy.logerr(f"Failed to connect to Ollama client. Please ensure Ollama is running. Error: {e}")
            rospy.signal_shutdown("Ollama connection failed.")
            return
            
        self.test_cases = self._load_test_cases_from_file(test_cases_file)
        if not self.test_cases:
            rospy.signal_shutdown("Failed to load test cases.")
            return

        rospy.loginfo(f"LLM Offline Tester is ready. Model: {self.ollama_model}")

        self.g_total_duration = 0.0
        self.g_total_tokens = 0

    def _load_test_cases_from_file(self, file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                # 假设JSON文件的顶层有一个名为 "test_cases" 的键
                rospy.loginfo(f"Successfully loaded {len(data['test_cases'])} test cases from {file_path}")
                return data['test_cases']
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error decoding JSON from {file_path}: {e}")
            return None
        except KeyError:
            rospy.logerr(f"JSON file {file_path} is missing the top-level 'test_cases' key.")
            return None
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred while reading {file_path}: {e}")
            return None

    def run_tests(self):
        """
        对LLM执行所有已定义的测试用例，并报告结果。
        第一个测试用例将作为“预热”运行，不计入最终的性能统计。
        """
        if not hasattr(self, 'ollama_client'):
            rospy.logerr("Ollama client not initialized. Aborting tests.")
            return

        total_tests = len(self.test_cases)
        if total_tests == 0:
            rospy.logwarn("No test cases loaded. Aborting tests.")
            return
            
        passed_tests = 0

        rospy.loginfo(f"{bcolors.HEADER}--- Starting LLM Offline Test Suite ---{bcolors.ENDC}")
        rospy.loginfo(f"Model: {bcolors.BOLD}{self.ollama_model}{bcolors.ENDC}")
        rospy.loginfo(f"Total test cases: {total_tests} (First test is for warm-up and excluded from stats)\n")

        for i, case in enumerate(self.test_cases):
            rospy.loginfo(f"{bcolors.OKBLUE}--- Testing {i+1}/{total_tests} ---{bcolors.ENDC}")
            rospy.loginfo(f"{bcolors.BOLD}Prompt:{bcolors.ENDC} {case['prompt']}")
            rospy.loginfo(f"Model: {self.ollama_model}")

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

                duration_ns = response.get('total_duration', 1)
                duration_s = duration_ns / 1_000_000_000
                
                tokens_per_second = total_tokens / duration_s if duration_s > 0 else 0
                
                # <--- 核心修改点 1: 增加条件判断 --->
                # 只有在不是第一次运行(i > 0)时，才将数据计入全局统计
                if i > 0:
                    self.g_total_tokens += total_tokens
                    self.g_total_duration += duration_s
                else:
                    # 对于第一次运行，打印一条提示信息
                    rospy.loginfo(f"{bcolors.WARNING}Note: This first run is a warm-up and is excluded from final statistics.{bcolors.ENDC}")

                stats_str = (f"Time: {duration_s:.2f} s | "
                             f"Speed: {tokens_per_second:.2f} tokens/s | "
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

            except Exception as e:
                if "timeout" in str(e).lower():
                    rospy.logerr(f"{bcolors.FAIL}Result: FAIL - Task timed out after {self.inference_timeout} seconds.{bcolors.ENDC}")
                else:
                    rospy.logerr(f"An error occurred during test case {i+1}: {e}")
                self.g_total_duration += self.inference_timeout
                
            rospy.loginfo('-' * 50) 

        pass_rate = (passed_tests / total_tests) * 100
        color = bcolors.OKGREEN if pass_rate > 80 else bcolors.WARNING if pass_rate > 50 else bcolors.FAIL
        
        # <--- 核心修改点 2: 调整最终统计的计算方式 --->
        # 用于统计的测试数量是总数减一（如果总数大于1）
        timed_tests_count = total_tests - 1 if total_tests > 1 else total_tests
        
        # 避免除以零的错误
        if timed_tests_count > 0:
            avg_duration = self.g_total_duration / timed_tests_count
            avg_speed = self.g_total_tokens / self.g_total_duration if self.g_total_duration > 0 else 0
        else:
            avg_duration = 0
            avg_speed = 0

        rospy.loginfo(f"{bcolors.HEADER}--- Test Suite Complete ---{bcolors.ENDC}")
        rospy.loginfo(f"Summary: ")
        rospy.loginfo(f"        Model                       : {self.ollama_model}")
        rospy.loginfo(f"        Pass Rate                   : {passed_tests}/{total_tests} tests passed ({color}{pass_rate:.2f}%{bcolors.ENDC})")
        
        # 使用新的计算结果
        if timed_tests_count > 0:
            rospy.loginfo(f"        Total Cost (timed runs)     : {self.g_total_duration:.3f} seconds for {timed_tests_count} tests.")
            rospy.loginfo(f"        Average Task Cost Time      : {avg_duration:.5f} seconds.")
            rospy.loginfo(f"        Average Token Generate Speed: {avg_speed:.5f} tokens/s")
        else:
            rospy.loginfo("        Not enough tests to calculate performance statistics.")



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