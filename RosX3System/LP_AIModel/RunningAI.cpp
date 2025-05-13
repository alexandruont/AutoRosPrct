#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeCommon.h>
#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <fstream>

using namespace nvinfer1;

class Logger : public ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) std::cout << msg << std::endl;
    }
};

Logger gLogger;

void* loadEngine(const std::string& enginePath, ICudaEngine*& engine, IRuntime*& runtime) {
    std::ifstream file(enginePath, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open engine file." << std::endl;
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();
    runtime = createInferRuntime(gLogger);
    engine = runtime->deserializeCudaEngine(buffer.data(), size);
    return engine;
}

const int c_ImageWidth = 640;
const int c_ImageHeigth = 640;
const unsigned int c_NumberOfPixels = c_ImageHeigth * c_ImageWidth;

void preprocess_image(const std::string& image_path, float* input_data, int input_w, int input_h) {
	cv::Mat img = cv::imread(image_path);
	if (img.empty()) {
		std::cerr << "Image not found!" << std::endl;
		return;
	}
	cv::Mat resized;
	cv::resize(img, resized, cv::Size(input_w, input_h));
	cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
	resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);

	int c = 3;
	for (int ch = 0; ch < c; ++ch) {
		for (int y = 0; y < input_h; ++y) {
			for (int x = 0; x < input_w; ++x) {
				input_data[ch * input_h * input_w + y * input_w + x] = resized.at<cv::Vec3f>(y, x)[ch];
			}
		}
	}
}

void preprocess_image(cv::Mat& img, float* input_data, int input_w, int input_h) {
	if (img.empty()) {
		std::cerr << "Image not loaded!" << std::endl;
		return;
	}
	cv::Mat resized;
	cv::resize(img, resized, cv::Size(input_w, input_h));
	cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
	resized.convertTo(resized, CV_32FC3, 1.0 / 255.0);


	cv::Mat channels[3];
	cv::split(resized, channels);

	// Rearrange data in a vector in such a way each channel gets in separately(not combined)
	memcpy(input_data, channels[0].data, c_NumberOfPixels * sizeof(float));
	memcpy(input_data + c_NumberOfPixels, channels[1].data, c_NumberOfPixels * sizeof(float));
	memcpy(input_data + (c_NumberOfPixels * 2), channels[2].data, c_NumberOfPixels * sizeof(float));
}

void infer(IExecutionContext* context, void** buffers, cudaStream_t stream, int batchSize) {
    context->enqueue(batchSize, buffers, stream, nullptr);
    cudaStreamSynchronize(stream);
}

int main() {
    std::string engineFile = "res/Yolo.engine";

    IRuntime* runtime = nullptr;
    ICudaEngine* engine = nullptr;
    if (!loadEngine(engineFile, engine, runtime)) return -1;
    
    IExecutionContext* context = engine->createExecutionContext();
    
    cudaStream_t stream;
    cudaStreamCreate(&stream);

    cv::VideoCapture c(1);
    if (!c.isOpened()) {
	    std::cerr << "Unable to open camera";
	    return -1;
    }
    cv::Mat image;
    float* outData = new float[5 * 8400];
    float* inData = new float[c_NumberOfPixels * 3];
    
    float* IOTensors[2];
    cudaMalloc(reinterpret_cast<void**>(&IOTensors[0]), 3 * c_NumberOfPixels * sizeof(float));
    cudaMalloc(reinterpret_cast<void**>(&IOTensors[1]), 5 * 8400 * sizeof(float));

    while (true){
        c >> image;
        preprocess_image(image, (float*)inData, 640, 640);
        cudaMemcpy(IOTensors[0], inData, 3 * c_NumberOfPixels * sizeof(float), cudaMemcpyHostToDevice);
        infer(context, reinterpret_cast<void**>(IOTensors), stream,  1);
        cudaMemcpy(outData, IOTensors[1], 5 * 8400 * sizeof(float), cudaMemcpyDeviceToHost);
        float max = outData[8400 * 4];
        int index = 8400 * 4;
        for (size_t i = 8400 * 4; i < 8400 * 5; i++) {
	    if (max < outData[i]) {
		        max = outData[i];
		        index = i;
	        }
        }
        if(outData[index] < 0.5f){
            cv::imshow("YOLOv8 Detections", image);
            continue;
        }
        index = index % 8400;
        const int locX = outData[index];
        const int locY = outData[8400 + index];
        const int width = outData[8400 * 2 + index];
        const int heigth = outData[8400 * 3 + index];
        cv::resize(image, image, { c_ImageWidth ,c_ImageHeigth });

        cv::rectangle(image, cv::Point(locX - (width / 2), locY - (heigth / 2)), cv::Point(locX + (width / 2), locY + (heigth / 2)), cv::Scalar(0, 255, 0, 0));
        cv::imshow("YOLOv8 Detections", image);
        if (cv::waitKey(20) % 0xFF == 'd') {
	        return 0;
        }
    }

    cudaFree(IOTensors[0]);
    cudaFree(IOTensors[1]);
    cudaStreamDestroy(stream);
    context->destroy();
    engine->destroy();
    runtime->destroy();
    
    return 0;
}