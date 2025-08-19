#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
from tello_llm_ros.srv import LLMQuery 
from utils.llm_utils import parse_llm_response, bcolors

class LLMServiceTester:
    def __init__(self):
        rospy.init_node('llm_service_tester')
        
        # --- 获取测试用例文件 ---
        test_cases_file = rospy.get_param("~test_cases_file")
        if not os.path.exists(test_cases_file):
            rospy.logerr(f"Test cases file not found at: {test_cases_file}")
            rospy.signal_shutdown("Missing test cases file.")
            return

        # --- 连接到LLM服务 ---
        service_name = "/llm_service_node/query"
        rospy.loginfo(f"Waiting for LLM service at '{service_name}'...")
        try:
            rospy.wait_for_service(service_name, timeout=30.0)
            self.llm_service_client = rospy.ServiceProxy(service_name, LLMQuery)
            rospy.loginfo("Successfully connected to LLM service.")
        except rospy.ROSException:
            rospy.logerr(f"Service '{service_name}' not available after waiting. Aborting test.")
            rospy.signal_shutdown("LLM service not found.")
            return
            
        # --- 加载测试用例 ---
        self.test_cases = self._load_test_cases_from_file(test_cases_file)
        if not self.test_cases:
            rospy.signal_shutdown("Failed to load test cases.")
            return

        rospy.loginfo(f"LLM Service Tester is ready.")

    def _load_test_cases_from_file(self, file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                rospy.loginfo(f"Successfully loaded {len(data['test_cases'])} test cases from {file_path}")
                return data['test_cases']
        except Exception as e:
            rospy.logerr(f"Error loading test cases from {file_path}: {e}")
            return None

    def run_tests(self):
        total_tests = len(self.test_cases)
        passed_tests = 0

        rospy.loginfo(f"{bcolors.HEADER}--- Starting LLM Service Test Suite ---{bcolors.ENDC}")
        rospy.loginfo(f"Total test cases: {total_tests}\n")

        self.g_total_duration = 0.0
        self.g_total_duration = 0.0
        self.g_total_completion_tokens = 0.0
        
        for i, case in enumerate(self.test_cases):
            rospy.loginfo(f"{bcolors.OKBLUE}--- Testing {i+1}/{total_tests} ---{bcolors.ENDC}")
            rospy.loginfo(f"{bcolors.BOLD}Prompt:{bcolors.ENDC} {case['prompt']}")

            try:
                # --- 调用ROS服务，而不是直接调用模型 ---
                response = self.llm_service_client(user_prompt=case['prompt'])
                
                if not response.success:
                    rospy.logerr(f"Service call failed: {response.error_message}")
                    continue
                duration_s = response.duration_s
                prompt_tokens = response.prompt_tokens
                completion_tokens = response.completion_tokens
                tokens_per_second = (completion_tokens / duration_s) if duration_s > 0 and completion_tokens > 0 else 0.0
                stats_str = (f"Time: {duration_s:.3f} s | "
                             f"Speed: {tokens_per_second:.2f} tokens/s | "
                             f"Tokens: {completion_tokens} (p: {prompt_tokens}, c: {completion_tokens})")
                rospy.loginfo(f"{bcolors.HEADER}LLM Stats:{bcolors.ENDC} {stats_str}")

                # <--- 将本次运行数据计入累加器 (预热除外) --->
                if i > 0:
                    self.g_total_duration += duration_s
                    self.g_total_completion_tokens += completion_tokens
                elif i == 0:
                    rospy.loginfo(f"{bcolors.WARNING}Note: This first run is a warm-up and is excluded from final statistics.{bcolors.ENDC}")

                plan_text = response.plan_text
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

            except rospy.ServiceException as e:
                rospy.logerr(f"An error occurred during test case {i+1}: {e}")
                
            rospy.loginfo('-' * 50) 

        pass_rate = (passed_tests / total_tests) * 100
        color = bcolors.OKGREEN if pass_rate > 99 else bcolors.WARNING if pass_rate > 70 else bcolors.FAIL
        
        timed_tests_count = total_tests - 1 if total_tests > 1 else total_tests
        
        if timed_tests_count > 0:
            avg_duration = self.g_total_duration / timed_tests_count
            avg_speed = (self.g_total_completion_tokens / self.g_total_duration) if self.g_total_duration > 0 and self.g_total_completion_tokens > 0 else 0.0
        else:
            avg_duration = 0
            avg_speed = 0

        rospy.loginfo(f"{bcolors.HEADER}--- Test Suite Complete ---{bcolors.ENDC}")
        rospy.loginfo(f"Summary: ")
        rospy.loginfo(f"        Pass Rate: {passed_tests}/{total_tests} tests passed ({color}{pass_rate:.2f}%{bcolors.ENDC})")
        
        if timed_tests_count > 0:
            rospy.loginfo(f"        Total Cost (timed runs)     : {self.g_total_duration:.3f} seconds for {timed_tests_count} tests.")
            rospy.loginfo(f"        Average Response Time       : {avg_duration:.5f} seconds/command.")
            rospy.loginfo(f"        Average Token Output Speed  : {avg_speed:.2f} tokens/s.")
        else:
            rospy.loginfo("        Not enough tests to calculate performance statistics.")

if __name__ == '__main__':
    try:
        rospy.sleep(1.0) 
        tester = LLMServiceTester()
        if not rospy.is_shutdown():
            tester.run_tests()
    except rospy.ROSInterruptException:
        pass