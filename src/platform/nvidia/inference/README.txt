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
bman@ubuntu:~$ video-viewer /dev/video0 rtsp://192.168.100.20:8554/my_stream
[gstreamer] initialized gstreamer, version 1.16.3.0
[gstreamer] gstCamera -- attempting to create device v4l2:///dev/video0

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.841: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed

(video-viewer:3862): GStreamer-CRITICAL **: 11:28:18.842: gst_element_message_full_with_details: assertion 'GST_IS_ELEMENT (element)' failed
[gstreamer] gstCamera -- found v4l2 device: vi-output, ar0233 30-0044
[gstreamer] v4l2-proplist, device.path=(string)/dev/video0, udev-probed=(boolean)false, device.api=(string)v4l2, v4l2.device.driver=(string)tegra-video, v4l2.device.card=(string)"vi-output\,\ ar0233\ 30-0044", v4l2.device.bus_info=(string)platform:tegra-capture-vi:1, v4l2.device.version=(uint)330344, v4l2.device.capabilities=(uint)2216689665, v4l2.device.device_caps=(uint)69206017;
[gstreamer] gstCamera -- found 8 caps for v4l2 device /dev/video0
[gstreamer] [0] video/x-raw, format=(string)UYVY, width=(int)1920, height=(int)1080, framerate=(fraction)30/1;
[gstreamer] [1] video/x-raw, format=(string)UYVY, width=(int)1280, height=(int)720, framerate=(fraction)45/1;
[gstreamer] [2] video/x-raw, format=(string)UYVY, width=(int)960, height=(int)540, framerate=(fraction){ 58/1, 30/1 };
[gstreamer] [3] video/x-raw, format=(string)UYVY, width=(int)640, height=(int)480, framerate=(fraction){ 60/1, 45/1 };
[gstreamer] [4] video/x-raw, format=(string)NV16, width=(int)1920, height=(int)1080, framerate=(fraction)30/1;
[gstreamer] [5] video/x-raw, format=(string)NV16, width=(int)1280, height=(int)720, framerate=(fraction)45/1;
[gstreamer] [6] video/x-raw, format=(string)NV16, width=(int)960, height=(int)540, framerate=(fraction){ 58/1, 30/1 };
[gstreamer] [7] video/x-raw, format=(string)NV16, width=(int)640, height=(int)480, framerate=(fraction){ 60/1, 45/1 };
[gstreamer] gstCamera -- selected device profile:  codec=raw format=uyvy width=1280 height=720
[gstreamer] gstCamera pipeline string:
[gstreamer] v4l2src device=/dev/video0 do-timestamp=true ! video/x-raw, format=(string)UYVY, width=(int)1280, height=(int)720 ! appsink name=mysink sync=false
[gstreamer] gstCamera successfully created device v4l2:///dev/video0
[video]  created gstCamera from v4l2:///dev/video0
------------------------------------------------
gstCamera video options:
------------------------------------------------
  -- URI: v4l2:///dev/video0
     - protocol:  v4l2
     - location:  /dev/video0
  -- deviceType: v4l2
  -- ioType:     input
  -- codec:      raw
  -- codecType:  cpu
  -- width:      1280
  -- height:     720
  -- frameRate:  45
  -- numBuffers: 4
  -- zeroCopy:   true
  -- flipMethod: none
------------------------------------------------
[gstreamer] gstEncoder -- codec not specified, defaulting to H.264
[gstreamer] gstEncoder -- detected board 'Jetson AGX Orin'
[gstreamer] gstEncoder -- pipeline launch string:
[gstreamer] appsrc name=mysource is-live=true do-timestamp=true format=3 ! nvvidconv name=vidconv ! video/x-raw(memory:NVMM) ! nvv4l2h264enc name=encoder bitrate=4000000 insert-sps-pps=1 insert-vui=1 idrinterval=30 maxperf-enable=1 ! video/x-h264 ! rtph264pay config-interval=1 name=pay0
[rtsp]   waiting for RTSP server to start...
[rtsp]   RTSP server started @ rtsp://ubuntu:8554
[rtsp]   RTSP route added /my_stream @ rtsp://ubuntu:8554
[video]  created gstEncoder from rtsp://192.168.100.20:8554/my_stream
------------------------------------------------
gstEncoder video options:
------------------------------------------------
  -- URI: rtsp://192.168.100.20:8554/my_stream
     - protocol:  rtsp
     - location:  192.168.100.20
     - port:      8554
  -- deviceType: ip
  -- ioType:     output
  -- codec:      H264
  -- codecType:  v4l2
  -- frameRate:  30
  -- bitRate:    4000000
  -- numBuffers: 4
  -- zeroCopy:   true
  -- latency     10
------------------------------------------------
[OpenGL] glDisplay -- X screen 0 resolution:  1920x1080
[OpenGL] glDisplay -- X window resolution:    1920x1080
[OpenGL] glDisplay -- display device initialized (1920x1080)
[video]  created glDisplay from display://0
------------------------------------------------
glDisplay video options:
------------------------------------------------
  -- URI: display://0
     - protocol:  display
     - location:  0
  -- deviceType: display
  -- ioType:     output
  -- width:      1920
  -- height:     1080
  -- frameRate:  0
  -- numBuffers: 4
  -- zeroCopy:   true
------------------------------------------------
[gstreamer] opening gstCamera for streaming, transitioning pipeline to GST_STATE_PLAYING
[gstreamer] gstreamer changed state from NULL to READY ==> mysink
[gstreamer] gstreamer changed state from NULL to READY ==> capsfilter0
[gstreamer] gstreamer changed state from NULL to READY ==> v4l2src0
[gstreamer] gstreamer changed state from NULL to READY ==> pipeline0
[gstreamer] gstreamer changed state from READY to PAUSED ==> capsfilter0
[gstreamer] gstreamer stream status CREATE ==> src
[gstreamer] gstreamer changed state from READY to PAUSED ==> v4l2src0
[gstreamer] gstreamer changed state from READY to PAUSED ==> pipeline0
[gstreamer] gstreamer message new-clock ==> pipeline0
[gstreamer] gstreamer stream status ENTER ==> src
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> capsfilter0
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> v4l2src0
[gstreamer] gstreamer message stream-start ==> pipeline0
[gstreamer] gstCamera -- onPreroll
[gstreamer] gstBufferManager recieve caps:  video/x-raw, format=(string)UYVY, width=(int)1280, height=(int)720, framerate=(fraction)45/1, colorimetry=(string)2:4:7:1, interlace-mode=(string)progressive
[gstreamer] gstBufferManager -- recieved first frame, codec=raw format=uyvy width=1280 height=720 size=1843200
[cuda]   allocated 4 ring buffers (1843200 bytes each, 7372800 bytes total)
[cuda]   allocated 4 ring buffers (8 bytes each, 32 bytes total)
[gstreamer] gstreamer changed state from READY to PAUSED ==> mysink
[gstreamer] gstreamer message async-done ==> pipeline0
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> mysink
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> pipeline0
[cuda]   allocated 4 ring buffers (2764800 bytes each, 11059200 bytes total)
video-viewer:  captured 0 frames (1280x720)
[OpenGL] glDisplay -- set the window size to 1280x720
[OpenGL] creating 1280x720 texture (GL_RGB8 format, 2764800 bytes)
[cuda]   registered openGL texture for interop access (1280x720, GL_RGB8, 2764800 bytes)
[cuda]   allocated 2 ring buffers (1382400 bytes each, 2764800 bytes total)
[gstreamer] gstEncoder -- starting pipeline, transitioning to GST_STATE_PLAYING
Opening in BLOCKING MODE 
[gstreamer] gstreamer changed state from NULL to READY ==> pay0
[gstreamer] gstreamer changed state from NULL to READY ==> capsfilter2
[gstreamer] gstreamer changed state from NULL to READY ==> encoder
[gstreamer] gstreamer changed state from NULL to READY ==> capsfilter1
[gstreamer] gstreamer changed state from NULL to READY ==> vidconv
[gstreamer] gstreamer changed state from NULL to READY ==> mysource
[gstreamer] gstreamer changed state from NULL to READY ==> pipeline1
[gstreamer] gstreamer changed state from READY to PAUSED ==> pay0
[gstreamer] gstreamer changed state from READY to PAUSED ==> capsfilter2
[gstreamer] gstreamer changed state from READY to PAUSED ==> encoder
[gstreamer] gstreamer changed state from READY to PAUSED ==> capsfilter1
[gstreamer] gstreamer changed state from READY to PAUSED ==> vidconv
[gstreamer] gstreamer stream status CREATE ==> src
[gstreamer] gstreamer changed state from READY to PAUSED ==> mysource
[gstreamer] gstreamer changed state from READY to PAUSED ==> pipeline1
[gstreamer] gstreamer message new-clock ==> pipeline1
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> pay0
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> capsfilter2
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> encoder
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> capsfilter1
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> vidconv
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> mysource
[gstreamer] gstreamer stream status ENTER ==> src
[gstreamer] gstreamer changed state from PAUSED to PLAYING ==> pipeline1
[gstreamer] gstEncoder -- new caps: video/x-raw, width=1280, height=720, format=(string)I420, framerate=30/1
video-viewer:  captured 1 frames (1280x720)
NvMMLiteOpen : Block : BlockType = 4 
===== NVMEDIA: NVENC =====
NvMMLiteBlockCreate : Block : BlockType = 4 
H264: Profile = 66, Level = 0 
NVMEDIA: Need to set EMC bandwidth : 376000 
NVMEDIA_ENC: bBlitMode is set to TRUE 
[gstreamer] gstreamer message latency ==> encoder
[gstreamer] gstreamer stream status CREATE ==> src
[gstreamer] gstreamer stream status ENTER ==> src
video-viewer:  captured 2 frames (1280x720)
[gstreamer] gstreamer message qos ==> encoder
[gstreamer] gstreamer mysource ERROR Internal data stream error.
[gstreamer] gstreamer Debugging info: gstbasesrc.c(3072): gst_base_src_loop (): /GstPipeline:pipeline1/GstAppSrc:mysource:
streaming stopped, reason not-linked (-1)
video-viewer:  captured 3 frames (1280x720)
video-viewer:  captured 4 frames (1280x720)
video-viewer:  captured 5 frames (1280x720)
video-viewer:  captured 6 frames (1280x720)
video-viewer:  captured 7 frames (1280x720)
video-viewer:  captured 8 frames (1280x720)
video-viewer:  captured 9 frames (1280x720)
video-viewer:  captured 10 frames (1280x720)

