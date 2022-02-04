#export GLOG_logtostderr=1

# if you are running on Linux desktop with CPU only
#bazel run --define MEDIAPIPE_DISABLE_GPU=1 \
#   mediapipe/examples/desktop/hello_world:hello_world

#return to main
#cd /mediapipe

#build hello-world example
export GLOG_logtostderr=1
bazel run --define MEDIAPIPE_DISABLE_GPU=1 --linkopt=-latomic \
mediapipe/examples/desktop/object_detection:object_detection_tflite


#GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/object_detection/object_detection_tflite \
 # --calculator_graph_config_file=mediapipe/graphs/object_detection/object_detection_desktop_tflite_graph.pbtxt \
  #--input_side_packets=input_video_path=mediapipe/examples/desktop/object_detection/test_video.mp4,output_video_path=mediapipe/samples/objects.mp4
