# Autonomous PID Controller By Saifullah
This repository is a fork of the Autonomous Subteam project where I contributed by implementing and testing a PID controller for variable speed regulation with anti-windup mechanism. The project logs speed response data into CSV files and visualizes the results as plots saved in PNG format.

## Features  
- PID controller with anti-windup  
- Speed response logging to CSV  
- Automatic plotting to PNG for analysis
- Variable speeds for each period of time

## Setup  
1. Fork this repository to your GitHub account.  
2. Clone your fork: `git clone https://github.com/<your-username>/<repo-name>.git`  
3. Navigate to the workspace: `cd <repo-name>`  
4. Build the workspace: `colcon build`  
5. Source the environment: `source install/setup.bash`  

## Running the Code  
To test the PID controller and generate results:  
```bash
ros2 run autonomous_subteam pid_controller
