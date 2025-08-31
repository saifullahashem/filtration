# Autonomous Subteam Tasks – Submission by Saifullah Hashem

## My Work
I implemented a **PID controller** with the following features:

- **Variable target speeds**:  
  - 0–10s → target speed = 0 km/h  
  - 10–40s → target speed = 60 km/h  
  - 40–70s → target speed = 30 km/h  
  - 70s → target speed = 50 km/h  

- **PID control with anti-windup mechanism** to prevent integral oversaturation.  
- **Automatic logging and visualization**:  
  - Logs speed data (time, actual, target) into `speed_data.csv`.  
  - Saves a plot (`speed_plot.png`) showing *target speed vs actual speed vs time* using Matplotlib.  
- **requirements.txt** added with all Python dependencies.  

---

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
