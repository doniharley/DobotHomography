import tkinter as tk
from tkinter import Toplevel, Text, Scrollbar, Canvas, Frame
import subprocess  # Import the subprocess module
import re

class DobotControlUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Dobot Magician Tools UI")
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.master.geometry("480x800")  # Set initial window size

        self.roscore_process = None
        self.dobotserver_process = None

        self.create_widgets()

    def create_widgets(self):
        # Create a canvas
        self.canvas = Canvas(self.master)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Add a scrollbar linked to the canvas
        self.scrollbar = Scrollbar(self.master, orient="vertical", command=self.canvas.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        # Create a frame inside the canvas to hold all the widgets
        self.scrollable_frame = Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")

        # Bind the scrolling event to the canvas
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        # PTP Common Parameters Setup
        ptp_frame = tk.LabelFrame(self.scrollable_frame, text="PTP Common Parameters")
        ptp_frame.pack(pady=5, fill=tk.X, expand=True)

        tk.Label(ptp_frame, text="PTP Velocity Ratio (0 to 100):").grid(row=0, column=0, padx=10, pady=5, sticky="w")
        self.ptp_velocity_var = tk.DoubleVar(value=50)
        tk.Entry(ptp_frame, textvariable=self.ptp_velocity_var).grid(row=0, column=1, padx=10, pady=5)

        tk.Label(ptp_frame, text="PTP Acceleration Ratio (0 to 100):").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.ptp_acceleration_var = tk.DoubleVar(value=50)
        tk.Entry(ptp_frame, textvariable=self.ptp_acceleration_var).grid(row=1, column=1, padx=10, pady=5)

        set_ptp_button = tk.Button(ptp_frame, text="Set PTP Params", command=self.set_ptp_params)
        set_ptp_button.grid(row=2, column=1, pady=10)

        self.ptp_params_label = tk.Label(ptp_frame, text="Current PTP Params: Not Set")
        self.ptp_params_label.grid(row=3, column=0, columnspan=2, padx=10, pady=10)

        # Setup for PTP Jump Parameters
        jump_frame = tk.LabelFrame(self.scrollable_frame, text="PTP Jump Parameters")
        jump_frame.pack(pady=5, fill=tk.X, expand=True)

        tk.Label(jump_frame, text="Jump Height:").grid(row=0, column=0, padx=10, pady=5, sticky="w")
        self.jump_height_entry = tk.Entry(jump_frame)
        self.jump_height_entry.grid(row=0, column=1, padx=10, pady=5)

        tk.Label(jump_frame, text="Z Limit:").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.z_limit_entry = tk.Entry(jump_frame)
        self.z_limit_entry.grid(row=1, column=1, padx=10, pady=5)

        self.ptp_jump_btn = tk.Button(jump_frame, text="Set Jump Params", command=self.set_ptp_jump_params)
        self.ptp_jump_btn.grid(row=1, column=2, padx=10, pady=5, sticky="ew")

        # Dobot Server, Suction Cup, and Alarm Controls
        dobot_frame = tk.LabelFrame(self.scrollable_frame, text="Dobot Server Controls")
        dobot_frame.pack(pady=5, fill=tk.X, expand=True)

        self.tty_label = tk.Label(dobot_frame, text="Enter USB port (e.g., ttyCH343USB0):")
        self.tty_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")

        self.tty_entry = tk.Entry(dobot_frame)
        self.tty_entry.insert(0, "ttyCH343USB0")
        self.tty_entry.grid(row=0, column=1, padx=10, pady=5)

        self.dobotserver_btn = tk.Button(dobot_frame, text="Run DobotServer", command=self.run_dobotserver)
        self.dobotserver_btn.grid(row=1, column=0, padx=10, pady=5, sticky="ew")

        self.stopserver_btn = tk.Button(dobot_frame, text="Stop DobotServer", command=self.stop_dobotserver)
        self.stopserver_btn.grid(row=1, column=1, padx=10, pady=5, sticky="ew")

        self.activate_suctioncup_btn = tk.Button(dobot_frame, text="Activate Suction Cup", command=self.activate_suctioncup)
        self.activate_suctioncup_btn.grid(row=2, column=0, padx=10, pady=5, sticky="ew")

        self.deactivate_suctioncup_btn = tk.Button(dobot_frame, text="Deactivate Suction Cup", command=self.deactivate_suctioncup)
        self.deactivate_suctioncup_btn.grid(row=2, column=1, padx=10, pady=5, sticky="ew")

        self.get_alarm_status_btn = tk.Button(dobot_frame, text="Get Alarm Status", command=self.get_alarm_status)
        self.get_alarm_status_btn.grid(row=3, column=0, padx=10, pady=5, sticky="ew")

        self.clear_alarm_status_btn = tk.Button(dobot_frame, text="Clear Alarm Status", command=self.clear_alarm_status)
        self.clear_alarm_status_btn.grid(row=3, column=1, padx=10, pady=5, sticky="ew")

        self.emergency_stop_btn = tk.Button(dobot_frame, text="Emergency Stop", bg="red", fg="white", command=self.emergency_stop)
        self.emergency_stop_btn.grid(row=4, column=0, columnspan=2, padx=10, pady=5, sticky="ew")

        # Roscore Button
        self.roscore_btn = tk.Button(self.scrollable_frame, text="Start Roscore", bg='purple', fg='white', command=self.start_roscore)
        self.roscore_btn.pack(pady=5, fill=tk.X)

        # Queue Frame
        queue_frame = tk.LabelFrame(self.scrollable_frame, text="Queue Controls")
        queue_frame.pack(pady=10, fill=tk.X, expand=True)

        self.start_exec_btn = tk.Button(queue_frame, text="Start Execution", command=self.start_execution)
        self.start_exec_btn.grid(row=0, column=0, padx=10, pady=5, sticky="ew")

        self.stop_exec_btn = tk.Button(queue_frame, text="Stop Execution", command=self.stop_execution)
        self.stop_exec_btn.grid(row=0, column=1, padx=10, pady=5, sticky="ew")

        self.clear_queue_btn = tk.Button(queue_frame, text="Clear Queue", command=self.clear_queue)
        self.clear_queue_btn.grid(row=0, column=2, padx=10, pady=5, sticky="ew")

        # Home and Watch buttons stacked
        button_frame = tk.Frame(self.scrollable_frame)
        button_frame.pack(pady=5, fill=tk.X)

        self.home_position_btn = tk.Button(button_frame, text="Home Position", command=self.set_home_position)
        self.home_position_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.watch_pose_btn = tk.Button(button_frame, text="Watch Get Pose", command=self.toggle_watch_pose)
        self.watch_pose_btn.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        self.watch_pose_active = False  # Flag to track if automatic GetPose is on

        # Start watching pose if needed
        self.watch_pose()

        # Labels for Pose Data
        self.pose_label = tk.Label(self.scrollable_frame, text="Current Pose:", font=("Helvetica", 14))
        self.pose_label.pack(pady=5, fill=tk.X)

        self.pose_frame = tk.Frame(self.scrollable_frame)
        self.pose_frame.pack(pady=5, fill=tk.X)

        self.x_label = tk.Label(self.pose_frame, text="X: N/A", fg="red", font=("Helvetica", 12))
        self.x_label.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        self.y_label = tk.Label(self.pose_frame, text="Y: N/A", fg="green", font=("Helvetica", 12))
        self.y_label.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
        self.z_label = tk.Label(self.pose_frame, text="Z: N/A", fg="blue", font=("Helvetica", 12))
        self.z_label.grid(row=1, column=0, padx=10, pady=2, sticky="ew")
        self.r_label = tk.Label(self.pose_frame, text="R: N/A", fg="purple", font=("Helvetica", 12))
        self.r_label.grid(row=1, column=1, padx=10, pady=2, sticky="ew")
        self.joint_label = tk.Label(self.pose_frame, text="Joint Angles: N/A", fg="orange", font=("Helvetica", 12))
        self.joint_label.grid(row=2, column=0, columnspan=2, padx=10, pady=2, sticky="ew")

        # Start watching pose periodically
        self.watch_pose()

    def show_terminal_output(self):
        # Create a new top-level window
        terminal_window = Toplevel(self.master)
        terminal_window.title("Terminal Output")

        # Add a text box with a scrollbar
        text_frame = tk.Frame(terminal_window)
        text_frame.pack(fill=tk.BOTH, expand=True)

        scrollbar = Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.text_output = Text(text_frame, wrap=tk.WORD, yscrollcommand=scrollbar.set)
        self.text_output.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.text_output.yview)

        # Add a button to show the terminal output
        show_button = tk.Button(terminal_window, text="Show Terminal Output", command=self.display_terminal_output)
        show_button.pack(pady=5)

    def display_terminal_output(self):
        # Example of how to show output in the text box
        output = "Example output text...\n"  # Replace with actual output
        self.text_output.insert(tk.END, output)
        self.text_output.see(tk.END)  # Scroll to the end of the text

    def set_ptp_params(self):
        velocity_ratio = self.ptp_velocity_var.get()
        acceleration_ratio = self.ptp_acceleration_var.get()

        # Construct the command packet
        header = b'\xAA\xAA'
        ctrl = 83  # Control ID for SetPTPCommonParams
        params = struct.pack('<ff', velocity_ratio, acceleration_ratio)  # Assuming float for ratios
        isQueued = 0  # 0 for not queued, 1 for queued
        rw = 0  # 0 for write
        length = len(params) + 4  # Control, rw, isQueued, and checksum

        # Construct the packet
        packet = struct.pack('<2sBBffBB', header, ctrl, rw, isQueued, velocity_ratio, acceleration_ratio, 0, 0)
        checksum = sum(packet) & 0xFF
        packet += struct.pack('<B', checksum)

        # Example command, replace with actual ROS service call to send packet
        print(f"Sending packet: {packet.hex()}")

        # After sending, fetch the current parameters and update the label
        self.update_ptp_params()

    def update_ptp_params(self):
        # Example command, replace with actual ROS service call to get current params
        # Simulated response for demonstration
        response_packet = b'\xAA\xAA\x83\x00\x01\x32\x00\x00\x00\x00'  # Sample response packet
        result = struct.unpack('<2sBBfB', response_packet)

        velocity_ratio = result[3]  # Parsed velocity ratio from response
        acceleration_ratio = result[4]  # Parsed acceleration ratio from response

        self.ptp_params_label.config(text=f"Current PTP Params: Velocity Ratio={velocity_ratio}, Acceleration Ratio={acceleration_ratio}")

    def set_ptp_jump_params(self):
        jump_height = float(self.jump_height_entry.get())
        z_limit = float(self.z_limit_entry.get())
        # Assume isQueued is desired for this operation
        try:
            subprocess.run(['rosservice', 'call', '/DobotServer/SetPTPJumpParams', str(jump_height), str(z_limit), '1'], check=True)
            print("PTP Jump Parameters set successfully")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set PTP Jump Parameters: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def start_roscore(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            print("Roscore started")
        except Exception as e:
            print(f"Failed to start roscore: {e}")

    def run_dobotserver(self):
        tty_port = self.tty_entry.get()
        try:
            self.dobotserver_process = subprocess.Popen(['rosrun', 'dobot', 'DobotServer', tty_port])
            self.dobotserver_btn.config(bg="green")  # Change button color to green
            print(f"DobotServer started on {tty_port}")
        except Exception as e:
            self.dobotserver_btn.config(bg="red")  # Ensure button color is red on failure
            print(f"Failed to run DobotServer on {tty_port}: {e}")

    def stop_dobotserver(self):
        try:
            if self.dobotserver_process:
                self.dobotserver_process.terminate()
                self.dobotserver_process.wait()
                self.dobotserver_btn.config(bg="red")  # Change button color to red
                print("DobotServer stopped")
        except Exception as e:
            print(f"Failed to stop DobotServer: {e}")

    def activate_suctioncup(self):
        try:
            command = "rosservice call /DobotServer/SetEndEffectorSuctionCup 'enableCtrl: 1\nsuck: 1'"
            subprocess.run(command, shell=True, check=True)
            print("Suction cup activated")
        except subprocess.CalledProcessError as e:
            print(f"Failed to activate suction cup, subprocess error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def deactivate_suctioncup(self):
        try:
            command = "rosservice call /DobotServer/SetEndEffectorSuctionCup 'enableCtrl: 1\nsuck: 0'"
            subprocess.run(command, shell=True, check=True)
            print("Suction cup deactivated")
        except subprocess.CalledProcessError as e:
            print(f"Failed to deactivate suction cup, subprocess error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def get_alarm_status(self):
        try:
            result = subprocess.check_output(['rosservice', 'call', '/DobotServer/GetAlarmsState', '{}']).decode('utf-8')
            print("Alarm status: ", result)
        except Exception as e:
            print(f"Failed to get alarm status: {e}")

    def clear_alarm_status(self):
        try:
            result = subprocess.check_output(['rosservice', 'call', '/DobotServer/ClearAllAlarmsState', '{}']).decode('utf-8')
            print("Clear alarm status: ", result)
        except Exception as e:
            print(f"Failed to clear alarm status: {e}")

    def emergency_stop(self, event=None):  # Allow calling from button and keyboard
        try:
            subprocess.run(['rosservice', 'call', '/DobotServer/SetQueuedCmdForceStopExec'], check=True)
            print("Emergency stop activated")
        except subprocess.CalledProcessError as e:
            print(f"Failed to execute emergency stop, subprocess error: {e.stderr}")
        except Exception as e:
            print(f"Unexpected error during emergency stop: {e}")

    def start_execution(self):
        try:
            subprocess.run(['rosservice', 'call', '/DobotServer/SetQueuedCmdStartExec'], check=True)
            print("Started execution of queued commands")
        except subprocess.CalledProcessError as e:
            print(f"Failed to start execution, subprocess error: {e.stderr}")
        except Exception as e:
            print(f"Unexpected error during start execution: {e}")

    def stop_execution(self):
        try:
            subprocess.run(['rosservice', 'call', '/DobotServer/SetQueuedCmdStopExec'], check=True)
            print("Stopped execution of queued commands")
        except subprocess.CalledProcessError as e:
            print(f"Failed to stop execution, subprocess error: {e.stderr}")
        except Exception as e:
            print(f"Unexpected error during stop execution: {e}")

    def clear_queue(self):
        try:
            subprocess.run(['rosservice', 'call', '/DobotServer/SetQueuedCmdClear'], check=True)
            print("Cleared command queue")
        except subprocess.CalledProcessError as e:
            print(f"Failed to clear queue, subprocess error: {e.stderr}")
        except Exception as e:
            print(f"Unexpected error during queue clearance: {e}")

    def toggle_watch_pose(self):
        if self.watch_pose_active:
            self.watch_pose_active = False
            self.watch_pose_btn.config(text="Watch Get Pose")
            print("Automatic GetPose deactivated")
        else:
            self.watch_pose_active = True
            self.watch_pose_btn.config(text="Stop Watching Pose")
            print("Automatic GetPose activated")
            self.watch_pose()

    def watch_pose(self):
        if self.watch_pose_active:
            try:
                result = subprocess.check_output(['rosservice', 'call', '/DobotServer/GetPose', '{}']).decode('utf-8')
                if not result.strip():
                    raise ValueError("Empty response from rosservice")

                pose = self.parse_pose(result)

                self.x_label.config(text=f"X: {pose['x']}")
                self.y_label.config(text=f"Y: {pose['y']}")
                self.z_label.config(text=f"Z: {pose['z']}")
                self.r_label.config(text=f"R: {pose['r']}")
                joint_angles = ', '.join([str(angle) for angle in pose['jointAngle']])
                self.joint_label.config(text=f"Joint Angles: {joint_angles}")

                print("Pose: ", pose)
            except ValueError as ve:
                print(f"Failed to get pose: {ve}")
            except Exception as e:
                print(f"Failed to get pose: {e}")
            finally:
                # Schedule the next call to this method after 1.8 seconds (1800 milliseconds)
                self.master.after(1800, self.watch_pose)

    def parse_pose(self, result):
        pose = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'r': 0.0,
            'jointAngle': [0.0, 0.0, 0.0, 0.0]
        }
        # Use regular expressions to find the values
        pose['x'] = float(re.search(r"x: ([\d\.\-]+)", result).group(1))
        pose['y'] = float(re.search(r"y: ([\d\.\-]+)", result).group(1))
        pose['z'] = float(re.search(r"z: ([\d\.\-]+)", result).group(1))
        pose['r'] = float(re.search(r"r: ([\d\.\-]+)", result).group(1))
        joint_angle_match = re.search(r"jointAngle: \[([^\]]+)\]", result)
        if joint_angle_match:
            pose['jointAngle'] = [float(val) for val in joint_angle_match.group(1).split(',')]
        return pose

    def set_home_position(self):
        try:
            subprocess.Popen(['rosservice', 'call', '/DobotServer/SetHOMECmd'])
            print("Home position set")
        except Exception as e:
            print(f"Failed to set home position: {e}")

    def on_closing(self):
        self.stop_dobotserver()
        self.stop_roscore()
        self.master.destroy()

    def stop_roscore(self):
        try:
            if self.roscore_process:
                self.roscore_process.terminate()
                self.roscore_process.wait()
                print("Roscore stopped")
        except Exception as e:
            print(f"Failed to stop roscore: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = DobotControlUI(root)
    root.mainloop()
