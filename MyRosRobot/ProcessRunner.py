import subprocess

class ProcessRunner:
    def __init__(self) -> None:
        pass

    def open_new_terminal_and_run(self,cmd):
        # Example command to open a new terminal and execute the given command
        terminal_command = f"gnome-terminal --tab -- bash -c '{cmd}; exec bash'"
        try:
            subprocess.run(terminal_command, shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error in {cmd}: {e}")
        return


    def start_tpm_core_node(self):
        #note: use 'launch' instead of 'run node' to load parameter .yaml file.
        package_name = "tpm_core_node"
        launch_file_name = "tpm_core.launch.py"
        cmd = f"ros2 launch {package_name} {launch_file_name}"
        self.open_new_terminal_and_run(cmd)
        return

    def start_demo_full(self): # 3 in 1 (rviz + move_group + tpm_controller)
        package_name = "ar3_moveit_config"
        launch_file_name = "demo_full.launch.py"
        cmd = f"ros2 launch {package_name} {launch_file_name}"
        self.open_new_terminal_and_run(cmd)
        pass

    def start_demo_noRviz(self): # 2 in 1 (move_group + tpm_controller)
        package_name = "ar3_moveit_config"
        launch_file_name = "demo_noRviz.launch.py"
        cmd = f"ros2 launch {package_name} {launch_file_name}"
        self.open_new_terminal_and_run(cmd)
        pass

    def start_rviz_only(self): # rviz only
        package_name = "ar3_moveit_config"
        launch_file_name = "moveit_rviz.launch.py"
        cmd = f"ros2 launch {package_name} {launch_file_name}"
        self.open_new_terminal_and_run(cmd)
        pass

    def start_mylinkpy(self):
        directory = "~/Desktop/MyLink_verPython/"
        processName = "Program.py"
        cmd = f"python3 {directory}{processName}"
        self.open_new_terminal_and_run(cmd)
        pass

    def start_script_node(self):
        package_name = "robot_command"
        node_name = "simple_thread"
        cmd = f"ros2 run {package_name} {node_name}"
        self.open_new_terminal_and_run(cmd)

        pass
    

    
