<p align="center">
  <h1 align="center">Gaussian-LIC2: LiDAR-Inertial-Camera<br> Gaussian Splatting SLAM</h1>
  <p align="center">
    ICRA 2025 & IJRR 2026
  </p>
  <p align="center">
    <a href="https://arxiv.org/pdf/2404.06926">
      <img src='https://img.shields.io/badge/Gaussian--LIC-Paper-red?style=flat&logo=arXiv&logoColor=red' alt='Gaussian-LIC Paper PDF'>
    </a>
    <a href="https://arxiv.org/pdf/2507.04004" style='padding-left: 0.5rem;'>
      <img src='https://img.shields.io/badge/Gaussian--LIC2-Paper-red?style=flat&logo=arXiv&logoColor=red' alt='Gaussian-LIC2 Paper PDF'>
    </a>
    <a href='https://xingxingzuo.github.io/gaussian_lic2/' style='padding-left: 0.5rem;'>
      <img src='https://img.shields.io/badge/Project-Page-blue?style=flat&logo=Google%20chrome&logoColor=blue' alt='Project Page'>
    </a>
  </p>
</p>

## Quick Start: Docker + GUI

Runs Gaussian-LIC on recordings from the Ouster + camera payload, in a GUI in your browser. Everything runs in a Docker container. The computer needs Ubuntu (tested on 24.04), an NVIDIA GPU with 12 GB or more (tested with an RTX 5090) and about 60 GB of free disk space.

| Your situation | Steps |
| --- | --- |
| New computer | 1, 2, 3, 4 |
| Computer already set up (step 1 done once), new clone | 2, 3, 4 |
| Image already built on this computer | 4 (after `git pull` or editing a config: 3, then 4) |

1. **Set up the computer** (once per computer).

   - NVIDIA driver 570 or newer: `nvidia-smi` shows the version. If it is missing or older, run `sudo ubuntu-drivers install` and reboot.
   - Docker and the NVIDIA Container Toolkit (running this again is fine, and Docker is skipped if you have it):

     ```shell
     docker --version || sudo apt install docker.io docker-buildx
     sudo usermod -aG docker $USER
     curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
       sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
     curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
       sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
       sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
     sudo apt update && sudo apt install nvidia-container-toolkit
     sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
     ```

   - Log out and back in, so that `docker` works without `sudo`.
   - Check: `docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L` lists your GPU.

2. **Get the code** (once per clone).

   ```shell
   git clone https://github.com/PandaHood/Gaussian-LIC.git
   cd Gaussian-LIC
   ```

3. **Build the image** (in the clone). About 20 minutes the first time, a few minutes when you rebuild after `git pull` or editing a config (the configs are part of the image). It builds for the GPU of this computer and downloads the depth completion model by itself.

   ```shell
   docker/build.sh
   ```

4. **Run the GUI** (in the clone, whenever you use it).

   ```shell
   docker/gui.sh
   ```

   It opens http://localhost:8080 in your browser; the first start on a computer takes a few extra minutes to build the TensorRT engines for its GPU. Pick a recording and click **Start**: the coloured point cloud builds up while it runs, and the Gaussian splats appear at the end. Results are saved in `~/gaussian_lic_result`. Press Ctrl-C in the terminal to quit.

   The GUI lists the recordings in your home folder, newest first. For recordings elsewhere, e.g. on an external drive: `GAUSSIAN_LIC_DATA=/media/$USER/<drive> docker/gui.sh`.

   To map while the payload is running instead, pick **Live topics** as the source. The page shows which of the payload's sensors it can see; this computer has to be on the payload's network with the same `ROS_DOMAIN_ID` (as for `ros2 topic list`). Click **Finish** to end the run, and Gaussian-LIC saves the map.

Gaussian-LIC2 is a photo-realistic LiDAR-Inertial-Camera Gaussian Splatting SLAM system, which simultaneously performs robust, accurate pose estimation and constructs a photo-realistic 3D Gaussian map in real time.

<p align="center">
    <img src="figure/r1_compressed.gif" alt="Logo" width="32%">
    <img src="figure/r0_compressed.gif" alt="Logo" width="32%">
    <img src="figure/f2_compressed.gif" alt="Logo" width="32%">
</p>

### 📢 News

- [2026-07-25] Gaussian-LIC2 is accepted by the International Journal of Robotics Research (IJRR)! 🎉
- [2026-02-21] Gaussian-LIC2 is released! 🚀 (stay tuned for updates)
- [2025-07-08] Gaussian-LIC2 is unveiled! 🎉 [[`Paper`](https://arxiv.org/pdf/2507.04004)] [[`Page`](https://xingxingzuo.github.io/gaussian_lic2/)] [[`YouTube`](https://www.youtube.com/watch?v=SkPnpuCfh88)] [[`bilibili`](https://www.bilibili.com/video/BV1fJ3kzfEYv/?spm_id_from=333.337.search-card.all.click&vd_source=99ac6409fc9373f3960feff31c28a189)] 
- [2025-07-07] The enhanced version of the Gaussian-LIC code is released!
- [2025-01-28] Gaussian-LIC is accepted to ICRA 2025! 🎉
- [2024-09-26] The second version of the paper is available on arXiv.
- [2024-04-10] The first version of the paper is available on arXiv.

### 💌 Contact

Questions? Please don't hesitate to reach out to Xiaolei Lang (Jerry) at jerry_locker@zju.edu.cn.

## Install

The ROS 2 version is tested with ROS 2 Jazzy on Ubuntu 24.04 with an NVIDIA RTX 5090 (the original ROS 1 version was tested on Ubuntu 20.04 with an RTX 3090 / 4090). All third-party libraries go to `~/Software` without root, which is where the build looks for them by default.

> This repository holds two **ROS 2** (ament / colcon) packages: the Gaussian-LIC mapping back-end in [`gaussian_lic/`](gaussian_lic) and its odometry front-end Coco-LIC in [`cocolic/`](cocolic), see [ROS 2 Interface](#ros-2-interface).

1. Exit Conda environment.

2. Install [CUDA 12.8](https://developer.nvidia.com/cuda-12-8-1-download-archive) (toolkit only, the installed NVIDIA driver is left alone; RTX 50xx GPUs need CUDA >= 12.8).

   ```shell
   mkdir -p ~/Software && cd ~/Software
   wget https://developer.download.nvidia.com/compute/cuda/12.8.1/local_installers/cuda_12.8.1_570.124.06_linux.run
   sh cuda_12.8.1_570.124.06_linux.run --nox11 --silent --toolkit --toolkitpath=$HOME/Software/cuda-12.8 --no-man-page
   rm cuda_12.8.1_570.124.06_linux.run
   ```

3. Build [OpenCV 4.12.0](https://github.com/opencv/opencv/archive/refs/tags/4.12.0.tar.gz).（must be built with [opencv_contrib 4.12.0](https://github.com/opencv/opencv_contrib/archive/refs/tags/4.12.0.tar.gz) and CUDA, no installation required; `CUDA_ARCH_BIN` is the compute capability of your GPU, 12.0 for RTX 50xx）

   ```shell
   mkdir -p ~/Software/opencv && cd ~/Software/opencv
   wget -O opencv-4.12.0.tar.gz https://github.com/opencv/opencv/archive/refs/tags/4.12.0.tar.gz && tar -xzf opencv-4.12.0.tar.gz && rm opencv-4.12.0.tar.gz
   wget -O opencv_contrib-4.12.0.tar.gz https://github.com/opencv/opencv_contrib/archive/refs/tags/4.12.0.tar.gz && tar -xzf opencv_contrib-4.12.0.tar.gz && rm opencv_contrib-4.12.0.tar.gz

   mkdir -p opencv-4.12.0/build && cd opencv-4.12.0/build
   export PATH=$HOME/Software/cuda-12.8/bin:$PATH
   cmake -DCMAKE_BUILD_TYPE=Release \
         -DWITH_CUDA=ON -DCUDA_ARCH_BIN=12.0 -DCUDA_ARCH_PTX= -DCUDA_TOOLKIT_ROOT_DIR=$HOME/Software/cuda-12.8 \
         -DWITH_CUDNN=OFF -DOPENCV_DNN_CUDA=OFF -DWITH_NVCUVID=OFF -DWITH_NVCUVENC=OFF \
         -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.12.0/modules \
         -DBUILD_TIFF=ON -DBUILD_ZLIB=OFF -DBUILD_JPEG=ON \
         -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF ..
   make -j$(nproc)
   ```

4. Prepare [LibTorch](https://pytorch.org/get-started/locally/).（no compilation or installation required）

   ```shell
   cd ~/Software
   wget https://download.pytorch.org/libtorch/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu128.zip
   unzip libtorch-cxx11-abi-shared-with-deps-2.7.1+cu128.zip && rm libtorch-cxx11-abi-shared-with-deps-2.7.1+cu128.zip
   ```

5. Prepare [TensorRT](https://developer.nvidia.com/tensorrt/download).（no compilation or installation required; use TensorRT 10, not 11）

   ```shell
   cd ~/Software
   wget https://developer.download.nvidia.com/compute/machine-learning/tensorrt/10.16.1/tars/TensorRT-10.16.1.11.Linux.x86_64-gnu.cuda-12.9.tar.gz
   tar -xzf TensorRT-10.16.1.11.Linux.x86_64-gnu.cuda-12.9.tar.gz && rm TensorRT-10.16.1.11.Linux.x86_64-gnu.cuda-12.9.tar.gz
   ```

6. Download Gaussian-LIC. (Coco-LIC is included)

   ```shell
   mkdir -p ~/ros2_gaussian/src
   cd ~/ros2_gaussian/src
   git clone https://github.com/APRIL-ZJU/Gaussian-LIC.git
   ```

7. TensorRT Deployment.

   `setup_spnet.sh` creates a Python venv in `~/Software/spnet-venv` with PyTorch for CUDA 12.8 (it needs `python3-pip`); `export_onnx.sh` downloads the SPNet checkpoint [Large_300.pth](https://drive.google.com/file/d/11dujPviL4pKLEXytXK0mEmPBNQDqgEak/view?pli=1) into `gaussian_lic/ckpt` if it is not there yet, and exports it; `build_trt.sh` builds FP32 engines (FP16 engines return wrong depths with TensorRT 10 on RTX 50xx).

   ```shell
   cd ~/ros2_gaussian/src/Gaussian-LIC/gaussian_lic/ckpt
   ./setup_spnet.sh
   ./export_onnx.sh
   ./build_trt.sh
   ```

8. Build Gaussian-LIC and Coco-LIC. (the TensorRT engines from step 7 are installed with the package, so build them first, or re-run `colcon build` after rebuilding them)

   ```shell
   cd ~/ros2_gaussian
   source /opt/ros/$ROS_DISTRO/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build --symlink-install
   ```

   If your third-party libraries are not in the `~/Software` locations above, point Gaussian-LIC at them (Coco-LIC uses the system OpenCV, so build it separately):

   ```shell
   colcon build --symlink-install --packages-select cocolic
   colcon build --symlink-install --packages-select gaussian_lic --cmake-args \
     -DOpenCV_DIR=/path/to/opencv/build \
     -DTorch_DIR=/path/to/libtorch/share/cmake/Torch \
     -DTENSORRT_ROOT=/path/to/TensorRT \
     -DCUDA_TOOLKIT_ROOT_DIR=/path/to/cuda
   ```

### Docker

Instead of the steps above, everything can run in a container ([`docker/`](docker)): ROS 2 Jazzy with ouster-ros, CUDA 12.8, LibTorch 2.7.1, TensorRT 10.16, OpenCV 4.12 with CUDA and the splat viewer. The host only needs the NVIDIA driver, Docker and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), see [Quick Start](#quick-start-docker--gui) for the install commands.

Build the image with `docker/build.sh` (about 39 GB). It compiles the CUDA code for the compute capability `nvidia-smi` reports (`CUDA_ARCH=8.9 docker/build.sh` for another GPU) and passes other arguments on to `docker build`. The SPNet model comes from `gaussian_lic/ckpt/spnet_480_640.onnx` if it is there; otherwise a build stage exports it on the CPU from `Large_300.pth`, downloading that first if needed, and only the ONNX file goes into the image.

```shell
docker/build.sh
```

`docker/run.sh` runs a command in the container, or a shell without one. The first start builds the TensorRT engines for the image sizes in the configs (a minute or two, kept in `~/.cache/gaussian_lic`). Your home folder is mounted at the same path, so bag paths are the same as outside the container. Results go to `~/gaussian_lic_result`: `gaussian_lic/` holds the renders, metrics and `point_cloud.ply` of the latest run, `cocolic/` the trajectory of every bag (TUM format). RViz2 opens on your display and the splat viewer is at http://localhost:8080 as usual:

```shell
docker/gui.sh  # the GUI, opens in your browser, see Own Data
docker/run.sh ros2 launch gaussian_lic payload.launch.xml bag_path:=$HOME/<recording>
docker/run.sh ros2 launch gaussian_lic payload.launch.xml  # the live topics, Ctrl-C finishes and saves the map
docker/run.sh ros2 launch gaussian_lic view_map.launch.xml
```

The container uses the host network and the ROS discovery settings of the shell it is started from (`ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE`, `ROS_STATIC_PEERS`, a Fast DDS profile file), so it sees the same topics as a node started there. The image uses Fast DDS, the ROS 2 default; nodes on another DDS implementation (e.g. Cyclone DDS) usually work with it, but ROS does not guarantee that.

## Run

Quick start on the sequence CBD_Building_01 in the FAST-LIVO2 dataset.

- Download [FAST-LIVO Dataset](https://connecthkuhk-my.sharepoint.com/personal/zhengcr_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fzhengcr%5Fconnect%5Fhku%5Fhk%2FDocuments%2FFAST%2DLIVO%2DDatasets&ga=1) or [FAST-LIVO2 Dataset](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) or [R3LIVE Dataset](https://github.com/ziv-lin/r3live_dataset) or [MCD Dataset](https://mcdviral.github.io/) or [M2DGR Dataset](https://github.com/SJTU-ViSYS/M2DGR).

+ Convert the ROS 1 bag to a ROS 2 bag (see the [Coco-LIC README](cocolic/README.md)), e.g. `rosbags-convert --src CBD_Building_01.bag --dst CBD_Building_01`.

+ Modify `bag_path` in `cocolic/config/ct_odometry_fastlivo2.yaml`, or pass `bag_path:=...` when launching Coco-LIC.

+ Launch Gaussian-LIC.

  ```shell
  cd ~/ros2_gaussian
  source install/setup.bash
  ros2 launch gaussian_lic fastlivo2.launch.xml  // The terminal will print "😋 Gaussian-LIC Ready!".
  ```


+ Launch Coco-LIC.

  Note：For real-time use and runtime analysis, please turn off the rviz in Coco-LIC with `rviz:=false`.

  ```shell
  cd ~/ros2_gaussian
  source install/setup.bash
  ros2 launch cocolic odometry.launch.xml config_path:=config/ct_odometry_fastlivo2.yaml rviz:=false
  ```


+ Once Coco-LIC has finished the bag, Gaussian-LIC evaluates the map, prints PSNR / SSIM / LPIPS and exits. The mapping and rendering results will be saved in  `~/ros2_gaussian/install/gaussian_lic/share/gaussian_lic/result` (this directory is wiped at the start of each evaluation).

### Own Data: Ouster + Camera Rig

The easiest way in is the GUI in the browser, `docker/gui.sh` (or `ros2 launch gaussian_lic gui.launch.xml` without Docker), which opens http://localhost:8080 in your default browser. It lists the recordings in your home folder (`bag_folder:=<dir>` for another one, or change it on the page), newest first. Pick one and click **Start**, or pick **Live topics** as the source to map while the payload runs (see below) and click **Finish** to end. While the data is processed, the coloured point cloud of the map builds up live, the same as in RViz2 (`/gaussian_lic/map`, the Gaussian centres in their colour), with the camera trajectory. When Gaussian-LIC has saved its map, the page shows the Gaussian splats and the novel-view PSNR. **Show the last result** loads the map of the latest run again. The page runs the same `payload.launch.xml` as below, without RViz2.

`cocolic/config/ct_odometry_payload.yaml` (with `cocolic/config/payload/`) and `gaussian_lic/config/payload.yaml` are a working setup for an Ouster OS-DOME-128 with its built-in IMU and a 2880x1860 camera. Coco-LIC tracks the camera on a 640x480 crop of it (`image_crop` / `image_resize` in `payload/camera.yaml`), Gaussian-LIC maps at the native resolution, 2880x1856 (`gs_image_crop` / `gs_cam_*` in `payload/camera.yaml`, image sizes must be multiples of 32). That needs its own depth completion engine, `./build_trt.sh 1856x2880` in `gaussian_lic/ckpt`, and about 9 GB of RAM for a 30 s recording (17 GB for 60 s) and 10 GB of GPU memory.

Recordings with the raw Ouster packets (`/ouster/lidar_packets`, `/ouster/imu_packets`, `/ouster/metadata`) are used as they are: when a bag has no `/ouster/points`, Coco-LIC decodes the packets itself with the Ouster SDK of [ouster-ros](https://github.com/ouster-lidar/ouster-ros) (`cocolic/src/lidar/ouster_packet_decoder.cpp`), into the same clouds and IMU messages the driver publishes (checked point by point against it). Unlike a replay through the driver, nothing is dropped and it runs at full speed. IMU measurements from before the first LiDAR packet are skipped: the sensor sends several seconds of them in a burst when a recording starts. Bags with `/ouster/points` and `/ouster/imu`, e.g. recorded from the running driver, work too. `ros2 run gaussian_lic replay_ouster_bag.py <bag> <bag>_replayed` still converts a recording through the driver if you want such a bag.

`<bag>` is the bag directory, or a single `.mcap` file of it when its `metadata.yaml` lists files that are missing.

Coco-LIC initialises gravity and the IMU biases from the first second of IMU data. A still start is best, but walking with the rig from the start works too, see `imu_excite_threshold` in `payload/imu.yaml`.

Then run both Gaussian-LIC and Coco-LIC on it with one command:

```shell
ros2 launch gaussian_lic payload.launch.xml bag_path:=<bag>
```

This also opens

+ a live 3D Gaussian splat viewer in the browser at http://localhost:8080 ([viser](https://viser.studio)), fed by `/gaussian_lic/splats` (every Gaussian with its colour, opacity, scale and rotation), with the current camera and its trajectory. It starts at the first camera pose; drag to orbit, scroll to zoom. It keeps showing the final map after Gaussian-LIC exits, until you stop the launch with Ctrl-C. It needs viser once:

  ```shell
  python3 -m venv --without-pip --system-site-packages ~/Software/viser-venv
  ~/Software/viser-venv/bin/python -m pip install viser "numpy<2"
  ```

+ RViz2 with the Gaussian centres in their base colour (`/gaussian_lic/map`, Gaussians with opacity below 0.1 are hidden), the splat render from the latest keyframe (`/gaussian_lic/render`) next to the camera image, and Coco-LIC's trajectory.

Without `bag_path`, the same launch maps the **live topics** while the payload runs:

```shell
ros2 launch gaussian_lic payload.launch.xml
```

Coco-LIC waits until every sensor is published (it says which ones it is waiting for), then reads the same topics as from a recording: `/camera/left/image_raw`, and `/ouster/points` with `/ouster/imu` or the raw packets, which it decodes. To end the run, press **Ctrl-C**, or call `ros2 service call /cocolic/finish std_srvs/srv/Trigger`, or stop publishing: Coco-LIC finishes once nothing has arrived for `live_timeout:=5.0` s (`0`: never), so playing a recording with `ros2 bag play` works too. Gaussian-LIC then evaluates and saves its map as at the end of a recording, which takes a minute or two after Ctrl-C. On a recording, Ctrl-C stops without saving, and the same service call stops early and saves. Coco-LIC's trajectory is saved as `live_<date>_<time>_LICO.txt`. Coco-LIC needs about 25 s for a 30 s payload recording on a 32-thread CPU next to Gaussian-LIC; if it falls behind the live data, it drops camera images (never LiDAR or IMU data) and says so.

Gaussian-LIC exits after saving its results (renders, metrics and `point_cloud.ply`) to `install/gaussian_lic/share/gaussian_lic/result`, which every run replaces; add `result_path:=<dir>` to keep them elsewhere. To look at a saved map again in the splat viewer, `ros2 launch gaussian_lic view_map.launch.xml` (the latest run, or `ply:=<dir>/point_cloud.ply`). Add `viewer:=false` / `rviz:=false` to run without the viewer / RViz2.

### ROS 2 Interface

`gs_mapping` subscribes to the following topics. Messages of the four topics are paired by header stamp (±10 ms around the point cloud stamp). Mapping finishes and the results are saved once no point cloud has arrived for 5 s.

| Topic | Type | Content |
| --- | --- | --- |
| `/points_for_gs` | `sensor_msgs/msg/PointCloud2` | colored points (`x y z rgb`) in the world frame, in front of the camera |
| `/pose_for_gs` | `geometry_msgs/msg/PoseStamped` | camera-to-world pose `T_wc` |
| `/image_for_gs` | `sensor_msgs/msg/Image` | color image (convertible to `bgr8`), `width`/`height` as in the config |
| `/depth_for_gs` | `sensor_msgs/msg/Image` | sparse metric depth (`32FC1`) aligned with the image, 0 = no depth |

Coco-LIC publishes these topics when `if_3dgs: true` is set in its config. Any other ROS 2 odometry front-end can feed Gaussian-LIC by publishing them.

## Checklist

- [ ] Support fast post-optimization
- [ ] Release the optimized Coco-LIC
- [ ] Provide the dockerfile
- [ ] Release the meshing tools
- [ ] Release our Gaussian-LIC2 dataset

## Citation

If you find our work helpful, please consider citing 🌟:

```bibtex
@inproceedings{lang2025gaussian,
  title={Gaussian-LIC: Real-time photo-realistic SLAM with Gaussian splatting and LiDAR-inertial-camera fusion},
  author={Lang, Xiaolei and Li, Laijian and Wu, Chenming and Zhao, Chen and Liu, Lina and Liu, Yong and Lv, Jiajun and Zuo, Xingxing},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8500--8507},
  year={2025},
  organization={IEEE}
}
```

```bibtex
@article{lang2026gaussian2,
  title={Gaussian-LIC2: LiDAR-Inertial-Camera Gaussian Splatting SLAM}, 
  author={Lang, Xiaolei and Lv, Jiajun and Tang, Kai and Li, Laijian and Huang, Jianxin and Liu, Lina and Liu, Yong and Zuo, Xingxing},
  journal={The International Journal of Robotics Research},
  year={2026}
}
```

## Acknowledgement

Thanks for [3DGS](https://github.com/graphdeco-inria/gaussian-splatting), [Taming-3DGS](https://github.com/humansensinglab/taming-3dgs), [StopThePop](https://github.com/r4dl/StopThePop), [Photo-SLAM](https://github.com/HuajianUP/Photo-SLAM) and [SPNet](https://github.com/Wang-xjtu/SPNet).

## LICENSE

The code is released under the [GNU General Public License v3 (GPL-3)](https://www.gnu.org/licenses/gpl-3.0.txt).
