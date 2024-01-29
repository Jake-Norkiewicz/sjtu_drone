import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Current pose subscriber
        self.gt_pose_sub = self.create_subscription(
            Pose,
            '/drone/gt_pose',
            self.pose_callback,
            1)

        self.gt_pose = None

        # Control command publisher
        self.command_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        
        # Callback for executing a control commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.square_size = 3.0
        self.accuracy = 0.01
        self.goals = [[self.square_size, 0.0], [self.square_size, self.square_size], [0.0, self.square_size], [0.0, 0.0]]
        #self.goals_numbers = [0,1,2,3]

        # Feel fre to fill with your code! Add some objects to represent a goal points to achieve

    
    def pose_callback(self, data): 
        self.gt_pose = data 
        print(f"{data}")


    def timer_callback(self):
        # HINT: Check a current pose. Use it to check if a drone achieved a desired pose.
        print(f"Current pose: {self.gt_pose}")
        
        msg = Twist()
        msg.linear.z = 3.0  #maintaining constant altitude

        x = float(self.gt_pose.position.x)
        y = float(self.gt_pose.position.y)

        distances_from_points = []
        distances_between_points = []
        current_goal_x = float()
        current_goal_y = float()

        for i in range(4):                      #measuring distance from each goal point
            x_goal, y_goal = self.goals[i]
            d = float(np.sqrt( pow((x-x_goal),2) + pow((y-y_goal),2) ))
            distances_from_points.append(d)
        
        for i in range(4):
            if(distances_from_points[i] <= self.accuracy):    #goals[i] has been achieved
                if(i == 3):
                    current_goal_x = (self.goals[0])[0] 
                    current_goal_y = (self.goals[0])[1]                        #back to the beginning
                    break                                                      #if the goal is known, there's no need to continue the loop
                else:
                    current_goal_x = (self.goals[i+1])[0] 
                    current_goal_y = (self.goals[i+1])[1]  
                    break
            else:                           #measuring the sum of distances BETWEEN goal points
                for i in range(4):
                    if(i == 3):
                        d = float(distances_from_points[0] + distances_from_points[3])
                        distances_between_points.append(d)
                    else:
                        d = float(distances_from_points[i] + distances_from_points[i+1])
                        distances_between_points.append(d)
                
                for i in range(4):
                    if(min(distances_between_points) == distances_between_points[i]):       #if the sum between points i and i+1 is the smallest, it means that the point i+1 is the next goal point
                        if(i==3):
                            current_goal_x = (self.goals[0])[0] 
                            current_goal_y = (self.goals[0])[1]
                            break
                        else:
                            current_goal_x = (self.goals[i+1])[0] 
                            current_goal_y = (self.goals[i+1])[1]
                            break
                        
                        
        msg.linear.x = float(current_goal_x)
        msg.linear.y = float(current_goal_y)
       
        self.command_pub.publish(msg)

        # HINT: Use a self.command_pub to publish a command
        # Fill with your code!
        print("Published!", "CURRENT GOAL:", current_goal_x, " ", current_goal_y)



def main(args=None):
    rclpy.init(args=args)

    node = DroneController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
