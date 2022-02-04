while getopts i:o: flag
do
    case "${flag}" in
        i) input=${OPTARG};;
        o) output=${OPTARG};;
    esac
done

bazel build -c opt --linkopt=-latomic --define MEDIAPIPE_DISABLE_GPU=1 mediapipe/examples/desktop/hand_tracking:hand_tracking_out_robot

tput bold
echo
echo "Run the last successful build? [Y/N]"
echo "Starting gpio daemon..."
sudo pigpiod
echo

read
if [ "${REPLY}" = "y" ]
then
    echo
    echo "Trying to run most recent successful build..."
    echo
    tput sgr0

    if [ -z "$input" ] && [ -z "$output" ]
    then 
        echo "Running without params.."
        GLOG_logtostderr=1 bazel-bin/mediapipe/examples/desktop/hand_tracking/hand_tracking_out_robot --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt
    else
        echo "Attempting to run with input/output path but that doesn't work yet."
    fi
else
    return $ec 2>/dev/null || exit $ec
fi
