bazel build --linkopt=-latomic --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/hand_tracking:hand_tracking_tflite

#GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/hand_tracking/hand_tracking_tflite \
  --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_2.pbtxt \
  --input_side_packets=input_video_path=samples/in/hand1.MP4,output_video_path=samples/out/hands.mp4