bazel build -c opt --linkopt=-latomic --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/object_detection:object_detection_tflite

GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/object_detection/object_detection_tflite \
  --calculator_graph_config_file=mediapipe/graphs/object_detection/object_detection_desktop_tflite_graph.pbtxt \
  --input_side_packets=input_video_path=samples/in/obj1.MP4,output_video_path=samples/out/objects2.mp4