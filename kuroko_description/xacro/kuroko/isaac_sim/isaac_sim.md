# Setup isaac sim
- Reference: [Isaac Lab 2.1.0 local installation](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/index.html)

# Install dependency
```sudo apt install libgl1 libglu1-mesa-dev```

# Install anaconda
```bash
mkdir -p ~/lib/anaconda; cd ~/lib/anaconda
wget -O anaconda.sh https://repo.anaconda.com/archive/Anaconda3-2024.06-1-Linux-x86_64.sh
bash anaconda.sh
export PATH="/home/$USER/anaconda3/bin:$PATH"
if ! grep -Fxq "## Conda paths" ~/.bashrc
then
    echo -e "\n## Conda paths"  >> ~/.bashrc
    echo 'export PATH="/home/$USER/anaconda3/bin:$PATH"' >> ~/.bashrc
    source ~/.bashrc
fi
```
- Do you accept the license terms? [yes|no] >>> `yes`
- Press ENTER to confirm the location >>> ENTER
- You can undo this by running `conda init --reverse $SHELL`? [yes|no] >>> `yes`

# Disable Conda base startup
```conda config --set auto_activate_base false #Disable conda autostart (base)```

# Install Isaac Sim
- Reference: [Robot Simulation (2): Configuring IsaacSim and IsaacLab with RTX 5090 on Ubuntu 24.04](https://blog.csdn.net/qq_45709806/article/details/149648493)
- Reference: [Isaac Sim 4.5.0 Python Environment Installation](https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/pip_installation.html)
- Reference: [Installation using Isaac Sim Pip Package](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)

```bash
mkdir -p ~/lib/isaaclab; cd ~/lib/isaaclab
conda install -c conda-forge gcc=12.1.0 #Reference: https://stackoverflow.com/questions/72540359/glibcxx-3-4-30-not-found-for-librosa-in-conda-virtual-environment-after-tryin
conda create -n env_isaaclab python=3.10 -y;conda activate env_isaaclab
pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
# For RTX50 Series: pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
pip install pyyaml typeguard
pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
```

# Run Isaac Sim
```bash
conda activate env_isaaclab
isaacsim
```

# isaac lab
```bash
git clone git@github.com:isaac-sim/IsaacLab.git
```

# Remove env_isaaclab
```bash
conda activate base
conda remove --name env_isaaclab --all -y
```