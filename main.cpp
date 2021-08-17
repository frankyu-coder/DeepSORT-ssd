// This code is written at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html

// Usage example:  ./object_detection_yolo.out --video=run.mp4
//                 ./object_detection_yolo.out --image=bird.jpg
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/types_c.h>
#include <opencv2/core/cuda.hpp>
#include <sys/timeb.h>
#include <opencv2/bgsegm.hpp>
#include "./DeepAppearanceDescriptor/FeatureTensor.h"
#include "KalmanFilter/tracker.h"
//#include "./concurrency_server_CountingBees/concurrency_server_CountingBees.h"
#include "CountingBees.h"

using namespace  std;
using namespace  cv;
const char *keys =
    "{help h usage ? | | Usage examples: \n\t\t./object_detection_yolo.out --image=dog.jpg \n\t\t./object_detection_yolo.out --video=run_sm.mp4 --stream=192.168.31.83}"
    "{image i        |<none>| input image   }"
    "{video v       |<none>| input video   }"
    "{stream s       |<none>| input stream   }";

// Initialize the parameters
const float confThreshold = 0.1; // Confidence threshold
const float nmsThreshold = 0.6;  // Non-maximum suppression threshold
cv::String detectionresult="";
cv::String traceresult="";
int jg=10;
//cv::String regioncount="";
std::vector<std::string> classes;
int tempregion=0;
// The index of video frame
int nFrame = 0;
//cap.open("http://169.254.92.99:8080/?action=stream?dummy=param.mjpg");
long long time_start_init = 0;
long long time_start = 0;
long long time_end = 0;

//Deep SORT parameter
const int nn_budget = 400;
const float max_cosine_distance = 0.2;

// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat &frame, const cv::Mat &out, DETECTIONS &d,int zh);

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat &frame);

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net &net);
void get_detections(DETECTBOX box, float confidence, DETECTIONS &d);

static long long gettimeU();
static std::string itos(int i); // convert int to string
static void timeusePrint(const std::string &mod, const long time_start, const long time_end);
static int cudaDeviceEnabled();
static void dnnConfig(cv::dnn::Net& Net);
static int paraParser(cv::CommandLineParser& parser, std::string& str, std::string& outputFile, std::string& total_result_file, cv::VideoCapture& cap);
static int frameProcess(tracker& mytracker, CountingBees& counter, int& cnInBees, int& cnOutBees, cv::dnn::Net& net, 
cv::CommandLineParser& parser, std::string& outputFile, cv::VideoCapture& cap, cv::Mat& frame, cv::Mat& blob);

std::string g_total_result_file;

int cnInBees_total = 0;
int cnOutBees_total = 0;
int cnInBees_bak = 0;
int cnOutBees_bak = 0;

int main(int argc, char **argv)
{

	if (-1 == cudaDeviceEnabled()) 
	{
		std::cout << "error : faild to find cuda device, program quit ..." << std::endl;
		return -1;
	}	

	//deep SORT
	tracker mytracker(max_cosine_distance, nn_budget);



	cv::dnn::Net net;
	dnnConfig(net);

	cv::CommandLineParser parser(argc, argv, keys);

	// Open a video file or an image file or a camera stream.
	std::string str, outputFile;
	cv::VideoCapture cap;
	
	if (-1 == paraParser(parser, str, outputFile, g_total_result_file, cap))
	{
	    return -1;
	}

	//cv::VideoWriter video;
	cv::Mat frame, blob;

	// Get the video writer initialized to save the output video
	//if (!parser.has("image"))
	//{
	//	video.open(outputFile, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 28.0,
	//			cv::Size(static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)), static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT))));
	//}

	// Create a window
	//static const std::string kWinName = "Multiple Object Tracking";

	// counter with the gate area of bee box
	CountingBees counter(60*60*24, 670, 190, 1665, 580, BEEBOX_GATE_270);
	int cnInBees = 0;
	int  cnOutBees = 0;

	// Process frames
	frameProcess(mytracker, counter,  cnInBees, cnOutBees, net, parser, outputFile, cap, frame, blob);

	cap.release();

	//if (!parser.has("image"))
	//	video.release();

	return 0;
}
const char* classNames[] = { "bee"};
// Remove the bounding boxes with low confidence using non-maxima suppression
void postprocess(cv::Mat &frame, const cv::Mat &output, DETECTIONS &d,int zh)
{
	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;
	cv::Mat detectionMat(output.size[2], output.size[3], CV_32F, (cv::Scalar*)output.ptr<float>());
	/*int*/ tempregion=0;
	for (int i = 0; i < detectionMat.rows; i++)
	{
		float confidence = detectionMat.at<float>(i, 2);
		if (confidence > confThreshold)
		{
			size_t objectClass = (size_t)(detectionMat.at<float>(i, 1));
			int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
			int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
			int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
			int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
			int width=xRightTop-xLeftBottom;
			int height=yLeftBottom-yRightTop;
			int left=xLeftBottom;
			int top=yRightTop;

			cv::Rect object((int)xLeftBottom, (int)yLeftBottom,(int)(xRightTop - xLeftBottom),(int)(yRightTop - yLeftBottom));
			rectangle(frame, object, Scalar(0, 255, 0), 2);
			// String label = String(classNames[objectClass]) + ": " + conf;
			String label = cv::format("%.2f", confidence);
			int baseLine = 0;
			Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
			cv::Rect Matafterzh=cv::Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),Size(labelSize.width, labelSize.height + baseLine));
			if(Matafterzh.x<0 || Matafterzh.y<0 || Matafterzh.x>frame.cols || Matafterzh.y>frame.rows || (Matafterzh.x+Matafterzh.width)>frame.cols || (Matafterzh.y+Matafterzh.height)>frame.rows)
				continue;
			rectangle(frame, Matafterzh,Scalar(0, 255, 0),  cv::FILLED);
			//putText(frame, label, Point(xLeftBottom, yLeftBottom), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));
			classIds.push_back(objectClass);
			confidences.push_back(confidence);
			boxes.push_back(Matafterzh);
			//cv::String temp=cv::format("%.5d %f %d %d %d %d;\r\n",zh, confidence, (int)xLeftBottom, (int)yLeftBottom,(int)(xRightTop - xLeftBottom),(int)(yRightTop - yLeftBottom));
			//detectionresult=detectionresult+temp;
			tempregion=tempregion+1;
		}
	}
	//cv::String temp1=cv::format("%.5d %d;\r\n",zh,tempregion);
	//regioncount=regioncount+temp1;
	std::vector<int> indices;
	std::cout<<"boxesize before nms:"<<boxes.size()<<std::endl;
	cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
	std::cout<<"boxesize after nms:"<<boxes.size()<<std::endl;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		size_t idx = static_cast<size_t>(indices[i]);
		cv::Rect box = boxes[idx];
		//目标检测 代码的可视化
		//drawPred(classIds[idx], confidences[idx], box.x, box.y,box.x + box.width, box.y + box.height, frame);
		get_detections(DETECTBOX(box.x, box.y, box.width, box.height), confidences[idx], d);
	}

	std::cout<<"havewriteten"<<std::endl;
	//cv::waitKey(1200);
	return;
}

// Draw the predicted bounding box
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat &frame)
{
	//Draw a rectangle displaying the bounding box
	cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
	//Get the label for the class name and its confidence
	std::string label = cv::format("%.2f", conf);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ":" + label;
	}
	//Display the label at the top of the bounding box
	int baseLine;
	cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = cv::max(top, labelSize.height);
	cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
	//cv::putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}

// Get the names of the output layers
std::vector<cv::String> getOutputsNames(const cv::dnn::Net &net)
{
	static std::vector<cv::String> names;
	if (names.empty())
	{
		//Get the indices of the output layers, i.e. the layers with unconnected outputs
		std::vector<int> outLayers = net.getUnconnectedOutLayers();
		//get the names of all the layers in the network
		std::vector<cv::String> layersNames = net.getLayerNames();
		// Get the names of the output layers in names
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}
void get_detections(DETECTBOX box, float confidence, DETECTIONS &d)
{
	DETECTION_ROW tmpRow;
	tmpRow.tlwh = box; //DETECTBOX(x, y, w, h);
	tmpRow.confidence = confidence;
	d.push_back(tmpRow);
}

long long gettimeU()
{
  struct timeval tv;
  //struct timezone tz;
  gettimeofday(&tv,NULL);
  //std::cout << "==" << tv.tv_sec << std::endl;
  return tv.tv_sec * 1000000 + tv.tv_usec;
}

void timeusePrint(const std::string &mod, const long time_start, const long time_end){std::cout << mod << " cost " << (time_end - time_start) / 1000 << " ms .." << std::endl;}

std::string itos(int i) // convert int to string
{
  std::stringstream s;
  s << i;
  return s.str();
}

int cudaDeviceEnabled()
{
	int num_devices = cv::cuda::getCudaEnabledDeviceCount();
	if (num_devices <= 0)
	{
		std::cout << "there is no device ." << std::endl;
		return -1;
	}

	 std::cout << "num_devices = " << num_devices << std::endl;
	int enable_device_id = -1;
	for (int i = 0; i < num_devices; i++) //cv::cuda::DeviceInfo::deviceID() ;
	{
		cv::cuda::DeviceInfo dev_info(i);
		if (dev_info.isCompatible())
		{
			enable_device_id = i;
			std::cout << "enable_device_id = " << enable_device_id << std::endl;
			cv::cuda::setDevice(enable_device_id);

			return 0;
		}
	}

	return -1;
}

void dnnConfig(cv::dnn::Net& net)
{
	// Give the configuration and weight files for the model
	cv::String modelConfiguration = "./model2/frozen_inference_graph.pb";
	//"/home/guoxm/models-1.13.01/research/object_detection/ssd_bee/model1/frozen_inference_graph.pb";

	cv::String modelWeights ="./model2/bee.pbtxt";
	//"/home/guoxm/models-1.13.01/research/object_detection/ssd_bee/model1/bee.pbtxt";

	/*cv::dnn::Net*/ net = cv::dnn::readNetFromTensorflow(modelConfiguration, modelWeights);
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	//net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	//net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

int paraParser(cv::CommandLineParser& parser, std::string& str, std::string& outputFile, std::string& total_result_file, cv::VideoCapture& cap)
{
	try
	{
		outputFile = "yolo_out_cpp.avi";
		if (parser.has("image"))
		{
			// Open the image file
			str = parser.get<cv::String>("image");
			std::ifstream ifile(str);
			if (!ifile)
				throw("error");
			cap.open(str);
			//cap.open("http://169.254.92.99:8080/?action=stream?dummy=param.mjpg");
			str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.jpg");
			outputFile = str;
			total_result_file = str + "image.total.result";
		}
		else if (parser.has("video"))
		{
			// Open the video file
			str = parser.get<cv::String>("video");
			std::ifstream ifile(str);
			if (!ifile)
				throw("error");
			cap.open(str);
			// cap.open("http://169.254.92.99:8080/?action=stream?dummy=param.mjpg");
			str.replace(str.end() - 4, str.end(), "_yolo_out_cpp.avi");
			outputFile = str;
			total_result_file = str + "video.total.result";
		}
		else if (parser.has("stream"))
		{
			str = parser.get<cv::String>("stream");
			string getstreamurl = string("http://") + str + string(":8080") + string("/?action=stream?dummy=param.mjpg");

			std::cout << "getstreamurl = " << getstreamurl << std::endl;
			cap.open(getstreamurl);
			total_result_file = str + "stream.total.result";
		}
		else
		{
			cap.open(0);
			total_result_file = "error.total.result";
		}

	}
	catch (...)
	{
		std::cout << "Could not open the input image/video stream" << std::endl;
		return -1;
	}

	return 0;
}

int frameProcess(tracker& mytracker, CountingBees& counter, int& cnInBees, int& cnOutBees,  cv::dnn::Net& net, 
cv::CommandLineParser& parser, std::string& outputFile, cv::VideoCapture& cap, cv::Mat& frame, cv::Mat& blob)
{
	time_start_init = gettimeU();

	while (cv::waitKey(1) < 0)
	{
		time_start = gettimeU();

		// get frame from the video
		cap >> frame;
		if (frame.empty())
		{
			counter.Count(cnInBees,cnOutBees);
			cnInBees_bak  = cnInBees;
			cnOutBees_bak = cnOutBees;

			std::cout << "Done processing !!!" << std::endl;

			time_t now;
			struct tm *fmt; 

			time(&now);  
			fmt = localtime(&now);

			ofstream file(g_total_result_file, ios::app);
			std::string content;
			content = std::to_string(fmt->tm_year+1900) + "/" + std::to_string(fmt->tm_mon+1) 
				+ "/" + std::to_string(fmt->tm_mday) 
				+ "-" + std::to_string(fmt->tm_hour) 
				+ ":" + std::to_string(fmt->tm_min) + ":" + std::to_string(fmt->tm_sec) 
				+ " " + "cnInBees = " + std::to_string(cnInBees_bak) + ", cnOutBees = " + std::to_string(cnOutBees_bak);
			file << content << std::endl;
			std::cout << "Total result  is stored as " << g_total_result_file << std::endl;
			cv::waitKey(3000);
			break;
		}

		// Create a 4D blob from a frame.
		//std::cout << "befoe  detection"<< std::endl;
		blob=cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300), cv::Scalar(), true, false);

		//Sets the input to the network
		net.setInput(blob);
		cv::Mat outs;
		outs= net.forward();//net.forward()

		// Remove the bounding boxes with low confidence
		DETECTIONS detections;
		postprocess(frame, outs, detections,nFrame);
		if (FeatureTensor::getInstance()->getRectsFeature(frame, detections))
		{
			std::cout << "Tensorflow get feature succeed!" << std::endl;
			mytracker.predict();
			mytracker.update(detections);
			std::vector<RESULT_DATA> result;
			for (Track &track : mytracker.tracks)
			{
				if (!track.is_confirmed() || track.time_since_update > 1)
				{
					continue;
				}
				result.push_back(std::make_pair(track.track_id, track.to_tlwh()));
				// for test 
				DETECTBOX bbox = track.to_tlwh();
				counter.Update(cnInBees_total, cnOutBees_total, nFrame, track.track_id, bbox[0], bbox[1], bbox[2], bbox[3]);
			}
			// cvScalar的储存顺序是B-G-R，CV_RGB的储存顺序是R-G-B


			for (unsigned int k = 0; k < result.size(); k++)
			{
				DETECTBOX tmp = result[k].second;
				cv::Rect rect = cv::Rect(tmp(0), tmp(1), tmp(2), tmp(3));
				rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
				std::string label = cv::format("%d", result[k].first);
				//cv::putText(frame, label, cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
				//cv::String cvstr=cv::format("%.5d %d %d %d %d %d;\r\n", nFrame,result[k].first,(int)tmp(0), (int)tmp(1), (int)tmp(2), (int)tmp(3));
				//traceresult=traceresult+cvstr;
			}

			if(int(nFrame/jg)>int((nFrame-1)/jg))
			{
				//std::cout<<"detectionresult:"<<detectionresult;
				//std::cout<<"traceresult:"<<traceresult;
				//std::cout<<"regioncount:"<<regioncount;
				//detectionresult="";
				//traceresult="";
				//regioncount="";
			}
			
			//cv::waitKey(500);
		}
		else
		{
			std::cout << "Tensorflow get feature failed!" << std::endl;
		}

		// Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
		std::vector<double> layersTimes;
		double freq = cv::getTickFrequency() / 1000;
		int64_t ticktime;
		try
		{
			ticktime=net.getPerfProfile(layersTimes);
		} catch (cv::Exception ex) {
			std::cout<<"biaoji;"<< ex.what()<<std::endl;
			return 1;
		}

		std::cout << ticktime << ":286:" << freq << std::endl;
		double t= ticktime/ freq;
		//std::string label = cv::format("Inference time for a frame : %.2f ms", t);
		//putText(frame, label, cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

		// Write the frame with the detection boxes
		//cv::Mat detectedFrame;
		//frame.convertTo(detectedFrame, CV_8U);
		//if (parser.has("image"))
		//	imwrite(outputFile, detectedFrame);
		//else
		//	video.write(detectedFrame);

		time_end = gettimeU();
		timeusePrint("total", time_start, time_end);
		
		int time_pass =(time_end-time_start_init)/1000;

		if (time_pass > (60 *1000))// == nFrame % 30)
		{
		counter.Count(cnInBees,cnOutBees);
		cnInBees_bak  = cnInBees;
		cnOutBees_bak = cnOutBees;
		//cnInBees_total += cnInBees;
		//cnOutBees_total += cnOutBees;
		//nFrame = 0;
		time_start_init = gettimeU();
		std::cout << "--------------cnInBees  =  " << cnInBees << " , " << "--------------cnOutBees = " << cnOutBees << " , "  << "--------------tempRegion = " << tempregion <<  std::endl;
		std::cout << "--------------cnInBees_total = " << cnInBees_total << "--------------cnOutBees_total = -" << cnOutBees_total << std::endl;
		}
		std::cout << "nFrame = " << nFrame << std::endl;
		nFrame++;
		usleep(1000);
	}

	return 0;
}
