#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Twist
import tkinter as tk
import time
import os

class RobotController(): 
    # Initialization:
    def __init__(self): 
        # Define commanded velocities: 
        self.CMDlinvel = 0.5
        self.CMDangvel = 0.5

        # Define actuated velocities: 
        self.TRUElinvel = 0
        self.TRUEangvel = 0

        # Define pressing and releasing times: 
        self.presstime1 = 0
        self.releasetime1 = 0
        self.presstime2 = 0
        self.releasetime2 = 0

        # Initialize the node and publishers: 
        self.TwistPub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.init_node('robot_control_gui')
        rospy.loginfo("""

        Reading from the keyboard and Publishing to Twist!
        ---------------------------
        Moving around:
            ↑     
        ←       →
            ↓     
        ---------------------------
        q : in place rotation - counterclockwise
        w : in place rotation - clockwise
        a/s : decrease/increase linear velocity by 0.1
        z/x : decrease/increase angular velocity by 0.1
                      
        Initial velocity values: 
        linear velocity:            angular velocity: 
        0.5                         0.5

        """)

        # Create the GUI: 
        self.root = tk.Tk()
        self.root.title('Remote Robot Control')
        self.root.geometry("520x220")
        self.root.configure(background='grey')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(script_dir, "figures/joystick.png")
        icon_image = tk.PhotoImage(file=image_path)
        self.root.iconphoto(True,icon_image)

        # Instruction label
        instruction_label = tk.Label(
            self.root,
            text="Remote Robot Controller :)",
            font=('Helvetica', 16),
            background='grey',
            foreground='white')
        instruction_label.pack(pady=10)

        # Command recap label
        command_label = tk.Label(
            self.root,
            text="""
            Moving around:
                ↑     
            ← →
                ↓     
            ---------------------------
            q : in place rotation - counterclockwise
            w : in place rotation - clockwise
            a/s : decrease/increase linear velocity by 0.1
            z/x : decrease/increase angular velocity by 0.1
            """,
            font=('Helvetica', 12),
            background='grey',
            foreground='white',
            justify=tk.LEFT)
        command_label.pack()

        # Bind functions to events
        self.root.bind('<KeyPress-Up>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-Down>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-Left>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-Right>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-a>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-s>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-z>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-x>', lambda event: self.direction(event))
        self.root.bind('<KeyPress-q>', lambda event: self.rotationCCLK(event))
        self.root.bind('<KeyPress-w>', lambda event: self.rotationCLK(event))
        self.root.bind('<KeyRelease-Up>', lambda event: self.on_key_release_lin(event))
        self.root.bind('<KeyRelease-Down>', lambda event: self.on_key_release_lin(event))
        self.root.bind('<KeyRelease-Left>', lambda event: self.on_key_release_lin(event))
        self.root.bind('<KeyRelease-Right>', lambda event: self.on_key_release_lin(event))
        self.root.bind('<KeyRelease-q>', lambda event: self.on_key_release_ang(event))
        self.root.bind('<KeyRelease-w>', lambda event: self.on_key_release_ang(event))
        
        # Root mainloop: 
        self.root.focus_set()  # Ensure the window has focus to capture key events
        self.root.mainloop()

    # Linear velocity:
    def LinearVelocityFun(self): 
        if self.TRUElinvel < self.CMDlinvel:
            self.TRUElinvel += 0.1*self.CMDlinvel
        return self.TRUElinvel

    # Angular velocity: 
    def AngularVelocityFun(self):
        if self.TRUEangvel < self.CMDangvel:
            self.TRUEangvel += 0.1*self.CMDangvel
        return self.TRUEangvel

    def direction(self,event):
        # Create twist message: 
        twistmsg = Twist()
        twistmsg.linear.y = 0
        twistmsg.linear.z = 0
        twistmsg.angular.x = 0
        twistmsg.angular.y = 0

        # Compute interval
        self.presstime1 = time.time()
        interval= self.presstime1-self.releasetime1
        
        # Set velocities to 0 if needed:
        if interval>0.2:
            self.TRUElinvel = 0
        
        # Define event functions:
        if event.keysym == 'Up':
            self.TRUElinvel = self.LinearVelocityFun()
            twistmsg.linear.x = self.TRUElinvel 
            twistmsg.angular.z = 0.0
            self.TwistPub.publish(twistmsg)

        elif event.keysym == 'Down':
            self.TRUElinvel = self.LinearVelocityFun()
            twistmsg.linear.x = -self.TRUElinvel 
            twistmsg.angular.z = 0.0
            self.TwistPub.publish(twistmsg)

        elif event.keysym == 'Left':
            self.TRUElinvel = self.LinearVelocityFun()
            twistmsg.angular.z = self.CMDangvel
            twistmsg.linear.x = self.TRUElinvel
            self.TwistPub.publish(twistmsg)

        elif event.keysym == 'Right':
            self.TRUElinvel = self.LinearVelocityFun()
            twistmsg.angular.z = -self.CMDangvel
            twistmsg.linear.x = self.TRUElinvel
            self.TwistPub.publish(twistmsg)

        elif event.keysym == "s":
            self.CMDlinvel += 0.1
            rospy.loginfo("Linear velocity increased to %f", self.CMDlinvel)

        elif event.keysym == "a":
            self.CMDlinvel += -0.1
            rospy.loginfo("Linear velocity decreased to %f", self.CMDlinvel)

        elif event.keysym == "z":
            self.CMDangvel += -0.1
            rospy.loginfo("Angular velocity decreasd to %f", self.CMDangvel)

        elif event.keysym == "x":
            self.CMDangvel += 0.1
            rospy.loginfo("Angular velocity increased to %f", self.CMDangvel)


    def rotationCCLK(self,event):
        # Compute interval
        self.presstime2 = time.time()
        interval= self.presstime2-self.releasetime2
        
        # Set velocities to 0 if needed:
        if interval>0.2:
            self.TRUEangvel = 0

        # Create twist message: 
        twistmsg = Twist()
        twistmsg.linear.y = 0
        twistmsg.linear.z = 0
        twistmsg.angular.x = 0
        twistmsg.angular.y = 0

        # Publish:
        twistmsg.linear.x = 0.0
        self.TRUEangvel = self.AngularVelocityFun()
        twistmsg.angular.z = self.TRUEangvel
        self.TwistPub.publish(twistmsg)

    def rotationCLK(self,event): 
        # Compute interval
        self.presstime2 = time.time()
        interval= self.presstime2-self.releasetime2
        
        # Set velocities to 0 if needed:
        if interval>0.2:
            self.TRUEangvel = 0

        # Create twist message: 
        twistmsg = Twist()
        twistmsg.linear.y = 0
        twistmsg.linear.z = 0
        twistmsg.angular.x = 0
        twistmsg.angular.y = 0

        # Publish:           
        twistmsg.linear.x = 0.0
        self.TRUEangvel = self.AngularVelocityFun()
        twistmsg.angular.z = -self.TRUEangvel
        self.TwistPub.publish(twistmsg)

    # Key release functions:
    def on_key_release_lin(self,event):
            #Update release time:
            self.releasetime1 = time.time()

    def on_key_release_ang(self,event):
            #Update release time:
            self.releasetime2 = time.time()

if __name__ == '__main__': 
    # Create controller
    controller = RobotController()
    rospy.spin()