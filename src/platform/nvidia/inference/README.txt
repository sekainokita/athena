link : https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md

sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
git clone --recursive --depth=1 https://github.com/dusty-nv/jetson-inference
cd jetson-inference
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig

# RTSP Server Setting
$ video-viewer /dev/video0 rtsp://192.168.100.20:8554/my_stream

# Depthmap
$ depthnet v4l2:///dev/video0 --depth-scale=0.5 --input-width=1280 --input-height=720

# Pose
$ posenet v4l2:///dev/video0
