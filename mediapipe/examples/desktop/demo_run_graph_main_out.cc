// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// An example of sending OpenCV webcam frames into a MediaPipe graph.

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/commandlineflags.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

//Take stream from /mediapipe/graphs/hand_tracking/hand_detection_desktop_live.pbtxt
// RendererSubgraph - LANDMARKS:hand_landmarks
#include "mediapipe/calculators/util/landmarks_to_render_data_calculator.pb.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"

// input and output streams to be used/retrieved by calculators
constexpr char kInputStream[] = "input_video";
constexpr char kOutputStream[] = "output_video";
constexpr char kLandmarksStream[] = "hand_landmarks";
constexpr char kWindowName[] = "MediaPipe";
constexpr char kLandmarksRectStream[] = "hand_rect";

// cli inputs
DEFINE_string(
    calculator_graph_config_file, "",
    "Name of file containing text format CalculatorGraphConfig proto.");
DEFINE_string(input_video_path, "",
              "Full path of video to load. "
              "If not provided, attempt to use a webcam.");
DEFINE_string(output_video_path, "",
              "Full path of where to save result (.mp4 only). "
              "If not provided, show result in a window.");
              
//helper funcs for recognition
float get_Euclidean_DistanceAB(float a_x, float a_y, float b_x, float b_y)
{
	float dist = std::pow(a_x - b_x, 2) + pow(a_y - b_y, 2);
	return std::sqrt(dist);
}

bool isThumbNearFirstFinger(::mediapipe::NormalizedLandmark point1, ::mediapipe::NormalizedLandmark point2)
{
	float distance = get_Euclidean_DistanceAB(point1.x(), point1.y(), point2.x(), point2.y());
	return distance < 0.1;
}

::mediapipe::Status RunMPPGraph() {
  std::string calculator_graph_config_contents;
  MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
      FLAGS_calculator_graph_config_file, &calculator_graph_config_contents));
  LOG(INFO) << "Get calculator graph config contents: "
            << calculator_graph_config_contents;
  mediapipe::CalculatorGraphConfig config =
      mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
          calculator_graph_config_contents);

  LOG(INFO) << "Initialize the calculator graph.";
  mediapipe::CalculatorGraph graph;
  MP_RETURN_IF_ERROR(graph.Initialize(config));

  LOG(INFO) << "Initialize the camera or load the video.";
  cv::VideoCapture capture;
  const bool load_video = !FLAGS_input_video_path.empty();
  //LOG(INFO) << FLAGS_input_video_path << ";" << FLAGS_output_video_path;
  if (load_video) {
	  LOG(INFO) << "Opening catpure...";
    capture.open(FLAGS_input_video_path);
  } else {
    capture.open(0);
    //set additional capture properties
    capture.set(cv::CAP_PROP_SETTINGS, 0); //toggle menu for cam settings
    capture.set(cv::CAP_PROP_FRAME_WIDTH,640);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT,480);
	capture.set(cv::CAP_PROP_FPS, 5);
	capture.set(cv::CAP_PROP_BRIGHTNESS,1);
	capture.set(cv::CAP_PROP_CONTRAST,15);
	capture.set(cv::CAP_PROP_SATURATION,90);
	capture.set(cv::CAP_PROP_HUE,1);
	//capture.set(cv::CAP_PROP_GAIN,1);
	//capture.set(cv::CAP_PROP_EXPOSURE,1);
	//capture.set(cv::CAP_PROP_CONVERT_RGB,true);
  }
  RET_CHECK(capture.isOpened());

  cv::VideoWriter writer;
  const bool save_video = !FLAGS_output_video_path.empty();
  if (save_video) {
    LOG(INFO) << "Prepare video writer.";
    cv::Mat test_frame;
    capture.read(test_frame);                    // Consume first frame.
    capture.set(cv::CAP_PROP_POS_AVI_RATIO, 0);  // Rewind to beginning.
    writer.open(FLAGS_output_video_path,
                mediapipe::fourcc('a', 'v', 'c', '1'),  // .mp4
                capture.get(cv::CAP_PROP_FPS), test_frame.size());
    RET_CHECK(writer.isOpened());
  } else {
    cv::namedWindow(kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
  }

  // pollers to retrieve streams from graph
  // output stream (i.e. rendered landmark frame)
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                   graph.AddOutputStreamPoller(kOutputStream));
  // hand landmarks stream
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmark,
            graph.AddOutputStreamPoller(kLandmarksStream));
  //hand landmarks_rect stream
  ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller_landmark_rect,
            graph.AddOutputStreamPoller(kLandmarksRectStream));

  LOG(INFO) << "Start running the calculator graph.";
  MP_RETURN_IF_ERROR(graph.StartRun({}));

  LOG(INFO) << "Start grabbing and processing frames.";
  size_t frame_timestamp = 0;
  bool grab_frames = true;
  while (grab_frames) {
    // Capture opencv camera or video frame.
    cv::Mat camera_frame_raw;
    capture >> camera_frame_raw;
    if (camera_frame_raw.empty()) break;  // End of video.
    cv::Mat camera_frame;
    cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
    if (!load_video) {
      cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
    }

    // Wrap Mat into an ImageFrame.
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
        mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
        mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);

    // Send image packet into the graph.
    MP_RETURN_IF_ERROR(graph.AddPacketToInputStream(
        kInputStream, mediapipe::Adopt(input_frame.release())
                          .At(mediapipe::Timestamp(frame_timestamp++))));

    // Get the graph result packet, or stop if that fails.
    mediapipe::Packet packet;
    mediapipe::Packet landmark_packet;

    //Polling the poller to get landmark packet
    if (!poller.Next(&packet)) break;
    if (!poller_landmark.Next(&landmark_packet)) break;

    // Use packet.Get to recover values from packet
    auto& output_frame = packet.Get<mediapipe::ImageFrame>();
    auto& output_landmarks = landmark_packet.Get<std::vector<::mediapipe::NormalizedLandmark>>();

	//added this for rects
	mediapipe::Packet landmark_rect_packet;
	if (!poller_landmark_rect.Next(&landmark_rect_packet)) break;
	auto& output_landmarks_rect = landmark_rect_packet.Get<::mediapipe::NormalizedRect>();
	
    // Convert back to opencv for display or saving.
    cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
    cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
    if (save_video) {
      writer.write(output_frame_mat);
    } else {
      cv::imshow(kWindowName, output_frame_mat);
      // Press any key to exit.
      // sets the delay to capture a frame default: 5
      const int pressed_key = cv::waitKey(25);
      if (pressed_key >= 0 && pressed_key != 255) grab_frames = false;
    }
    
    //calc finger states for recognition
    
    // finger states
    bool thumbIsOpen = false;
    bool firstFingerIsOpen = false;
    bool secondFingerIsOpen = false;
    bool thirdFingerIsOpen = false;
    bool fourthFingerIsOpen = false;
    
    //construct landmark list
    int i = 0;
    ::mediapipe::NormalizedLandmark landmarkList [21];
    
    for (const ::mediapipe::NormalizedLandmark& landmark : output_landmarks) {
		landmarkList[i] = landmark;
		i++;
	}
	i=0;
	
	//calc finger states from landmark list
	float pseudoFixKeyPoint = landmarkList[2].x();
	if (landmarkList[3].x() > pseudoFixKeyPoint && landmarkList[4].x() > pseudoFixKeyPoint)
	{
		thumbIsOpen = true;
	}

	pseudoFixKeyPoint = landmarkList[6].y();
	if (landmarkList[7].y() < pseudoFixKeyPoint && landmarkList[8].y() < pseudoFixKeyPoint)
	{
		firstFingerIsOpen = true;
	}

	pseudoFixKeyPoint = landmarkList[10].y();
	if (landmarkList[12].y() < pseudoFixKeyPoint && landmarkList[12].y() < pseudoFixKeyPoint)
	{
		secondFingerIsOpen = true;
	}

	pseudoFixKeyPoint = landmarkList[14].y();
	if (landmarkList[15].y() < pseudoFixKeyPoint && landmarkList[16].y() < pseudoFixKeyPoint)
	{
		thirdFingerIsOpen = true;
	}

	pseudoFixKeyPoint = landmarkList[18].y();
	if (landmarkList[19].y() < pseudoFixKeyPoint && landmarkList[20].y() < pseudoFixKeyPoint)
	{
		fourthFingerIsOpen = true;
	}
    
    // printout landmark values
    /*
    for (const ::mediapipe::NormalizedLandmark& landmark : output_landmarks) {
		
	    std::cout << landmark.DebugString();
	    
    }
    */
    
    std::cout<<"\n-----------------------------------\n";
    //printout the hand rectangle
	std::cout << "\nRectangle landmarks: " << "\n";
	std::cout << "x_center:" << output_landmarks_rect.x_center() << "\n";
	std::cout << "y_center:" << output_landmarks_rect.y_center() << "\n";
	std::cout << "height:" << output_landmarks_rect.height() << "\n";
	std::cout << "width:" << output_landmarks_rect.width() << "\n";
	std::cout << "rotation:" << output_landmarks_rect.rotation() << "\n";
	//printout the finger states
	std::cout<<"\nThumb open:"<<thumbIsOpen;
	std::cout<<"\nPointer finger open:"<<firstFingerIsOpen;
	std::cout<<"\nMiddle finger open:"<<secondFingerIsOpen;
	std::cout<<"\nRing finger open:"<<thirdFingerIsOpen;
	std::cout<<"\nPinky finger open:"<<fourthFingerIsOpen;
	//printout the recognized hand gesture
	std::cout<<"\n\nRecognized gesture: ";
	if (thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && thirdFingerIsOpen && fourthFingerIsOpen)
    {
        std::cout << "Five";
    }
    else if (!thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && thirdFingerIsOpen && fourthFingerIsOpen)
    {
        std::cout << "Four";
    }
    else if (thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Four";
    }
    else if (!thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Three";
	system("/home/pi/Desktop/ir_send/irrp.py -p -g18 -f/home/pi/Desktop/ir_send/codes 3");
    }
    else if (thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Three";
    }
    else if (thumbIsOpen && firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Two";
    }
    else if (!thumbIsOpen && firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "One";
	system("/home/pi/Desktop/ir_send/irrp.py -p -g18 -f/home/pi/Desktop/ir_send/codes 2");
    }
    else if (!thumbIsOpen && firstFingerIsOpen && secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Peace or Two";
    }
    else if (!thumbIsOpen && firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && fourthFingerIsOpen)
    {
        std::cout << "Rock on";
    }
    else if (thumbIsOpen && firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && fourthFingerIsOpen)
    {
        std::cout << "Spiderman";
	system("/home/pi/Desktop/ir_send/irrp.py -p -g18 -f/home/pi/Desktop/ir_send/codes 1");
    }
    else if (!thumbIsOpen && !firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen)
    {
        std::cout << "Fist";
    }
    else if (!firstFingerIsOpen && secondFingerIsOpen && thirdFingerIsOpen && fourthFingerIsOpen && isThumbNearFirstFinger(landmarkList[4], landmarkList[8]))
    {
        std::cout << "Ok";
    }
    else if (!firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && !fourthFingerIsOpen && thumbIsOpen)
    {
		std::cout << "Thumbs up";
	}
	else if (!firstFingerIsOpen && !secondFingerIsOpen && !thirdFingerIsOpen && fourthFingerIsOpen && thumbIsOpen)
    {
		std::cout << "Skater";
	}
	std::cout<<"\n-----------------------------------\n";
  }

  LOG(INFO) << "Shutting down.";
  if (writer.isOpened()) writer.release();
  MP_RETURN_IF_ERROR(graph.CloseInputStream(kInputStream));
  return graph.WaitUntilDone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ::mediapipe::Status run_status = RunMPPGraph();
  if (!run_status.ok()) {
    LOG(ERROR) << "Failed to run the graph: " << run_status.message();
  } else {
    LOG(INFO) << "Success!";
  }
  return 0;
}
