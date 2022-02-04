#GLOG_logtostderr=1 bazel-bin/mediapipe/mediapipe/examples/desktop/hand_tracking/hand_tracking_cpu \
#  --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
export GLOG_logtostderr=1

bazel run --define MEDIAPIPE_DISABLE_GPU=1 --linkopt=-latomic \
mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu