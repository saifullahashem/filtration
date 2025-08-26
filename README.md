# Autonomous Subteam Tasks

## Setup Instructions

### 1. Fork the Repository
* Click **“Fork”** to create your own copy under your GitHub account.

### 2. Clone Your Fork
* In your proper environment (i.e., **Linux**, or **docker container** for win) run:
    ```bash
    git clone https://github.com/<your-username>/<repo-name>.git
    ```
    > make sure to replace `<your-username>` with your username, and `<repo-name>` with the actual repo name.

### 3. Build your workspace
```bash
cd <repo-name>/task_ws/
colcon build --symlink-install
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
> replace `<repo-name>` with the actual repo name.

### 4. Implement Your Solution
* Change your directory 
    ```bash
    cd src/pre_interview/pre_interview/
    ```
* Modify only the `controller.py` file.
    ```bash
    nano controller.py
    ```
* Make sure your code runs without errors inside the container.

### 5. Test Your Code

* Run the simulation and verify that your PID controller makes the vehicle track the target speed.
* Save any plots as part of your submission.

## Reminder
> [!IMPORTANT]  
> If you are on Windows, all development and testing must be done inside the provided Docker container. This ensures consistency across all submissions.
