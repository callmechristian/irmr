export GLOG_logtostderr=1

# if you are running on Linux desktop with CPU only
#bazel run --define MEDIAPIPE_DISABLE_GPU=1 \
#   mediapipe/examples/desktop/hello_world:hello_world

#return to main
cd /mediapipe

#build hello-world example
bazel run --define MEDIAPIPE_DISABLE_GPU=1 --linkopt=-latomic \
mediapipe/examples/desktop/hello_world:hello_world