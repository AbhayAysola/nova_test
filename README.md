Since you're daily driving Arch and using Neovim, you want a README that is technically dense but quick to scan. This is the **Nova** Project Manual, tailored for your specific UID-mapped Docker workflow.

---

# 🏁 Project Nova: F1TENTH / ICRA 2026 Devkit

**Nova** is a high-performance autonomous racing environment built for the 2026 ICRA competition. It uses a custom-layered Docker architecture to bridge an **Arch Linux host** (Neovim/Development) with a **ROS 2 Humble container** (Simulation/Execution).

## 🏗 Architecture
- **Host:** Arch Linux (UID 1000) + Neovim
- **Container:** Ubuntu 22.04 + ROS 2 Humble + CUDA 12.x
- **Sync:** Bidirectional volume mount on `./src` with UID/GID mapping to prevent `Permission denied` errors.



---

## 🛠 Prerequisites
1. **NVIDIA Container Toolkit:** Ensure `nvidia-smi` works on the host.
2. **Docker Compose:** Installed on Arch (`sudo pacman -S docker-compose`).
3. **X11 Permissions:** Run `xhost +local:docker` before launching to enable GUI/RViz.

---

## 🚀 Getting Started

### 1. Environment Setup
Create a `.env` file in the root directory to map your Arch user to the container:
```bash
# Get your IDs
echo "UID=$(id -u)" > .env
echo "GID=$(id -g)" >> .env
```

### 2. Launch the Devkit
Build and start the **Nova** environment:
```bash
docker compose build
docker compose up -d
```

### 3. Enter the Cockpit
Jump into the container to run ROS 2 commands:
```bash
docker exec -it nova_devkit bash
```

---

## 💻 Development Workflow

### Writing Code
Edit files inside `./src` using **Neovim** on your Arch host. Changes are instantly visible inside the container.

### Building & Running
Inside the `nova_devkit` container:
```bash
# 1. Build the Nova packages
cd /home/ros2_ws
colcon build --symlink-install

# 2. Source the environment
source install/setup.bash

# 3. Launch the Follow the Gap (FTG) Node
ros2 run nova_ftg ftg_node
```

### GUI / Simulation
To visualize the LiDAR scans and the car's trajectory in RViz:
```bash
ros2 launch autodrive_roboracer bringup_graphics.launch.py
```

---

## 📦 Submission Strategy
When **Nova** is ready for the competition, use the provided `Dockerfile` to "bake" your source code into a portable image:

1. **Build Final Image:**
   ```bash
   docker build -t nova_submission:latest .
   ```
2. **Export for Judges:**
   ```bash
   docker save nova_submission:latest | gzip > nova_icra_practice.tar.gz
   ```

---

## ⚠️ Troubleshooting
- **Permission Errors:** Run `sudo rm -rf build/ install/ log/` on the host and rebuild.
- **Display Issues:** Run `xhost +local:docker` on Arch.
- **Command Not Found:** Ensure you have `source /opt/ros/humble/setup.bash` in your container's `.bashrc`.

---

> **Note:** This setup is optimized for the **2026-icra-practice** image. Ensure the `image:` tag in `compose.yaml` matches the latest competition release.
