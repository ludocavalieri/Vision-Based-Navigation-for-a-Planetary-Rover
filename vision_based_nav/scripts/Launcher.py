#!/usr/bin/env python3

import subprocess
import tkinter as tk
import os
import time
import rospy
import threading
from tkinter import messagebox
import rospy

# Create launcher class:
class Launcher():
    def __init__(self):
        # Create GUI:
        self.root = tk.Tk()
        self.root.title('Launcher')
        self.root.geometry("700x420")
        self.root.configure(background='grey')
        script_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(script_dir, "figures/control panel.png")
        icon_image = tk.PhotoImage(file=image_path)
        self.root.iconphoto(True,icon_image)

        # Create open process list: 
        self.process_dict = {}

        # Command recap label:
        command_label = tk.Label(
            self.root,
            text="""
            Launch Simulation : opens Gazebo model
            Robot Controller : launches controller to pilot rover
            Visual Odometry : starts visual odometry node
            Visualize VO Output : records VO and ground truth messages, when it shuts down 
            it shows the trajectory plot
            Generate Costmap : starts dense point cloud and costmap publisher
            """,
            font=('Helvetica', 12),
            background='grey',
            foreground='white',
            justify=tk.LEFT)
        command_label.grid(row=0, column=0, columnspan=4, pady=10)

        # Create buttons:
        tk.Button(self.root, text="Launch Simulation", command=self.LaunchSim).grid(row=1, column=0, pady=10, padx=5)
        tk.Button(self.root, text="Stop Simulation", command=lambda: self.StopProcess("LaunchSim")).grid(row=1, column=1, pady=10, padx=5)

        tk.Button(self.root, text="Robot Controller", command=self.RunController).grid(row=2, column=0, pady=10, padx=5)
        tk.Button(self.root, text="Stop Controller", command=lambda: self.StopProcess("RunController")).grid(row=2, column=1, pady=10, padx=5)

        tk.Button(self.root, text="Visual Odometry", command=self.RunVO).grid(row=3, column=0, pady=10, padx=5)
        tk.Button(self.root, text="Stop VO", command=lambda: self.StopProcess("RunVO")).grid(row=3, column=1, pady=10, padx=5)

        tk.Button(self.root, text="Visualize VO output", command=self.RunLocTest).grid(row=4, column=0, pady=10, padx=5)
        tk.Button(self.root, text="Stop Visualizing VO output", command=lambda: self.StopProcess("RunLocTest")).grid(row=4, column=1, pady=10, padx=5)

        tk.Button(self.root, text="Generate Dense PC", command=self.RunDensePC).grid(row=5, column=0, pady=10, padx=5)
        tk.Button(self.root, text="Stop Generating Dense PC", command=lambda: self.StopProcess("RunDensePC")).grid(row=5, column=1, pady=10, padx=5)

    # Start master:
    def StartRosMaster(self): 
        # Roscore:
        master_process = subprocess.Popen(["roscore"])
        self.process_dict['roscore'] = master_process

        # Wait for master to start:
        time.sleep(2)      

    # Shutdown function:
    def Shutdown(self):  
        # Shutdown ROS nodes:
        def shutdown_ros_nodes():
            os.system("rosnode list | xargs -n1 rosnode kill")   
            subprocess.run(["sleep", "2"])
        
        # Terminate master processes:
        def terminate_processes(): 
            # Look for Gazebo master and shut it down:
            try:
                gazpid_list = subprocess.check_output(["pgrep", "-f", "gazebo"])  
                gazebo_pid = list(map(int, gazpid_list.split()))
                self.process_dict.extend(gazebo_pid)
            except subprocess.CalledProcessError:
                rospy.logwarn("WARNING: No sign of gazebo processes")
                pass

            # Terminate all processes:
            for process in self.process_dict.values():
                process.terminate()
                process.wait()

        # Quitting messag:
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            shutdown_ros_nodes()
            terminate_processes()
            rospy.loginfo("Shutting Interface down.")
            rospy.signal_shutdown("Shutting interface down") 
            self.root.destroy()

    # Buttons functions:
    def LaunchSim(self): 
        # Define command:
        sim_command = ["roslaunch", "vision_based_nav", "gazebo_model.launch"]

        # Launch in a new terminal:
        sim_process = subprocess.Popen(sim_command)
        self.process_dict['LaunchSim'] = sim_process

    def RunController(self): 
        # Run Controller:
        def run_controller():
            controller_process = subprocess.Popen(["rosrun", "vision_based_nav", "RobotController.py"])
            self.process_dict['RunController'] = controller_process

        # Start thread:
        controller_thread = threading.Thread(target=run_controller)
        controller_thread.start()

    def RunVO(self):
        # Run VO:
        def run_VO():
            vo_process = subprocess.Popen(["rosrun", "vision_based_nav", "VOnode.py"])
            self.process_dict['RunVO'] = vo_process

        # Start thread:
        VO_thread = threading.Thread(target=run_VO)
        VO_thread.start()
        
    def RunLocTest(self):
        # Run localization test:
        def run_loctest():
            loctest_process = subprocess.Popen(["rosrun", "vision_based_nav", "LocalizationTest.py"])
            self.process_dict['RunLocTest'] = loctest_process

        # Start thread:
        loctest_thread = threading.Thread(target=run_loctest)
        loctest_thread.start()
        
    def RunDensePC(self):
        # Run point cloud node:
        def run_PC():
            pc_process = subprocess.Popen(["rosrun", "vision_based_nav", "DensePointCloud.py"])
            self.process_dict['RunDensePC'] = pc_process

        # Start thread:
        PC_thread = threading.Thread(target=run_PC)
        PC_thread.start()

    # Stop individual process:
    def StopProcess(self, process_name):
        if process_name in self.process_dict:
            process = self.process_dict[process_name]
            if process.poll() is None:
                process.terminate()
                process.wait()
                del self.process_dict[process_name]
            else:
                messagebox.showwarning("Warning", f"{process_name} is not running")

# Main code:
if __name__ == '__main__':
    # Activate launcher:
    try: 
        ri = Launcher()
        ri.StartRosMaster()
        rospy.init_node('launcher')
        ri.root.protocol("WM_DELETE_WINDOW", ri.Shutdown)
        ri.root.mainloop()
    except rospy.ROSInterruptException:
        pass