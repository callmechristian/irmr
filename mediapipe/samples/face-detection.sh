#mediapipe/examples/desktop/face_detection:face_detection_cpu

export GLOG_logtostderr=1

#build hello-world example
bazel build --linkopt=-latomic --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/face_detection:face_detection_cpu

bazel run --define MEDIAPIPE_DISABLE_GPU=1 --linkopt=-latomic \
mediapipe/examples/desktop/face_detection:face_detection_cpu