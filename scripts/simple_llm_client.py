#!/usr/bin/env python3
import rospy
import actionlib
from tello_llm_ros.msg import ExecuteTaskAction, ExecuteTaskGoal

def feedback_cb(feedback):
    rospy.loginfo(f"[STATUS] {feedback.status}")

def main():
    rospy.init_node('task_client_example')
    
    client = actionlib.SimpleActionClient('/task_control_node/execute_task', ExecuteTaskAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    
    rospy.loginfo("Action server found. You can now enter commands.")
    
    while not rospy.is_shutdown():
        try:
            user_input = input("You: ")
            if user_input.lower() in ['exit', 'quit']:
                break
            if not user_input.strip():
                continue

            goal = ExecuteTaskGoal(user_prompt=user_input)
            client.send_goal(goal, feedback_cb=feedback_cb)
            
            client.wait_for_result()
            
            result = client.get_result()
            state = client.get_state()
            
            rospy.loginfo(f"--- Task Complete ---")
            rospy.loginfo(f"Final State: {state}")
            rospy.loginfo(f"Final Message: {result.final_message}")

        except KeyboardInterrupt:
            rospy.loginfo("Exiting client.")
            break

if __name__ == '__main__':
    main()