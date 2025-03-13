#include "inference.cuh"
#include <ios>

using namespace nvinfer1;

#include <NvOnnxParser.h>

/**
 * cudaMemcpys CPU memory in inputTensor to GPU based on bindings
 * Queues that tensor to be passed through model
 * cudaMemcpys the result back to CPU memory
 * Requires bindings, inputTensor, stream
 * Modifies stream, outputTensor
 */

Inference::Inference(std::string modelName, std::string packagePathString) :  mModelName{std::move(modelName)}, mPackagePath{std::move(packagePathString)}{

	//Create ONNX and engine file paths
	mLogger.log(ILogger::Severity::kINFO, mModelName.c_str());
	mLogger.log(ILogger::Severity::kINFO, mPackagePath.c_str());
	mONNXModelPath = std::filesystem::path{mPackagePath} / "data" / std::string(mModelName + ".onnx");
	mEngineModelPath = std::filesystem::path{mPackagePath} / "data" / std::string("tensorrt-engine-" + mModelName + ".engine");;

	std::array<char, 150> message{};
	std::snprintf(message.data(), message.size(), "Reading from ONNX model at %s and creating TensorRT engine at %s", mONNXModelPath.c_str(), mEngineModelPath.c_str());
	mLogger.log(ILogger::Severity::kINFO, message.data());

	// Create the engine object from either the file or from onnx file
	mEngine = std::unique_ptr<ICudaEngine>{createCudaEngine()};
	if (!mEngine) throw std::runtime_error("Failed to create CUDA engine");

	mLogger.log(ILogger::Severity::kINFO, "Created CUDA Engine");

	// Check some assumptions about the model
	if (mEngine->getNbIOTensors() != 2) throw std::runtime_error("Invalid Binding Count");

	// Store the IO Tensor Names
	mInputTensorName = mEngine->getIOTensorName(0);
	mOutputTensorName = mEngine->getIOTensorName(1);

	if (mEngine->getTensorIOMode(mInputTensorName.c_str()) != TensorIOMode::kINPUT) throw std::runtime_error("Expected Input Binding 0 Is An Input");
	if (mEngine->getTensorIOMode(mOutputTensorName.c_str()) != TensorIOMode::kOUTPUT) throw std::runtime_error("Expected Output Binding Input To Be 1");

	// Be verbose about the input tensor size
	auto inputTensorSize = getInputTensorSize();
	std::snprintf(message.data(), message.size(), "%s Tensor's Dimensions:", mInputTensorName.c_str());
	mLogger.log(ILogger::Severity::kINFO, message.data());
	for(size_t i = 0; i < inputTensorSize.size(); ++i){
		std::snprintf(message.data(), message.size(), "Dimension: %zu Size: %zu", i, inputTensorSize[i]);
		mLogger.log(ILogger::Severity::kINFO, message.data());
	}

	// Be verbose about the input tensor size
	auto outputTensorSize = getOutputTensorSize();
	std::snprintf(message.data(), message.size(), "%s Tensor's Dimensions:", mOutputTensorName.c_str());
	mLogger.log(ILogger::Severity::kINFO, message.data());
	for(size_t i = 0; i < outputTensorSize.size(); ++i){
		std::snprintf(message.data(), message.size(), "Dimension: %zu Size: %zu", i, outputTensorSize[i]);
		mLogger.log(ILogger::Severity::kINFO, message.data());
	}

	createExecutionContext();

	prepTensors();
}

auto Inference::createCudaEngine() -> ICudaEngine* {
	mLogger.log(ILogger::Severity::kINFO, "Creating engine building tools...");
	constexpr auto explicitBatch = 1U << static_cast<std::uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);

	IBuilder* builder = createInferBuilder(mLogger);
	if (!builder) throw std::runtime_error("Failed to create Infer Builder");
	mLogger.log(ILogger::Severity::kINFO, "Created Infer Builder");

	std::unique_ptr<INetworkDefinition> network{builder->createNetworkV2(explicitBatch)};
	if (!network) throw std::runtime_error("Failed to create Network Definition");
	mLogger.log(ILogger::Severity::kINFO, "Created Network Definition");

	std::unique_ptr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, mLogger)};
	if (!parser) throw std::runtime_error("Failed to create ONNX Parser");
	mLogger.log(ILogger::Severity::kINFO, "Created ONNX Parser");

	std::unique_ptr<IBuilderConfig> config{builder->createBuilderConfig()};
	if (!config) throw std::runtime_error("Failed to create Builder Config");
	mLogger.log(ILogger::Severity::kINFO, "Created Builder Config");

	if (!parser->parseFromFile(mONNXModelPath.c_str(), static_cast<int>(ILogger::Severity::kINFO))) {
		throw std::runtime_error("Failed to parse ONNX file");
	}

	IRuntime* runtime = createInferRuntime(mLogger);

	// Define the engine file location relative to the mrover package
	// Check if engine file exists
	if (!exists(mEngineModelPath)) {
		std::cout << "Optimizing ONXX model for TensorRT. This make take a long time..." << std::endl;

		// Create the Engine from onnx file
		IHostMemory* serializedEngine = builder->buildSerializedNetwork(*network, *config);
		if (!serializedEngine) throw std::runtime_error("Failed to serialize engine");

		// Create temporary engine for serializing
		ICudaEngine* tempEng = runtime->deserializeCudaEngine(serializedEngine->data(), serializedEngine->size());
		if (!tempEng) throw std::runtime_error("Failed to create temporary engine");

		// Save Engine to File
		auto trtModelStream = tempEng->serialize();
		std::ofstream outputFileStream{mEngineModelPath, std::ios::binary};
		outputFileStream.write(static_cast<char const*>(trtModelStream->data()), static_cast<std::streamsize>(trtModelStream->size()));
		outputFileStream.close();

		return tempEng;
	}

	// Load engine from file
	std::ifstream inputFileStream{mEngineModelPath, std::ios::binary};
	std::stringstream engineBuffer;

	// Stream in the engine file to the buffer
	engineBuffer << inputFileStream.rdbuf();
	std::string enginePlan = engineBuffer.str();
	// Deserialize the Cuda Engine file from the buffer
	return runtime->deserializeCudaEngine(enginePlan.data(), enginePlan.size());
}

auto Inference::createExecutionContext() -> void {
	// Create Execution Context
	mContext.reset(mEngine->createExecutionContext());
	if (!mContext) throw std::runtime_error("Failed to create execution context");

	// Set up the input tensor sizing
	mContext->setInputShape(mInputTensorName.c_str(), mEngine->getTensorShape(mInputTensorName.c_str()));
}

auto Inference::doDetections(cv::Mat const& img) const -> void {
	// Do the forward pass on the network
	launchInference(img, mOutputTensor);
}

auto Inference::getOutputTensor() -> cv::Mat {
	return mOutputTensor;
}

auto Inference::launchInference(cv::Mat const& input, cv::Mat const& output) const -> void {
	//Assert these items have been initialized
	assert(!input.empty());
	assert(!output.empty());
	assert(input.isContinuous());
	assert(output.isContinuous());
	assert(mContext);

	// Get the binding id for the input tensor
	int inputId = getBindingInputIndex(mContext.get());

	// Memcpy the input tensor from the host to the gpu
	cudaMemcpy(mBindings[inputId], input.data, input.total() * input.elemSize(), cudaMemcpyHostToDevice);

	// Execute the model on the gpu
	mContext->executeV2(mBindings.data());

	// Memcpy the output tensor from the gpu to the host
	cudaMemcpy(output.data, mBindings[1 - inputId], output.total() * output.elemSize(), cudaMemcpyDeviceToHost);
}

auto Inference::prepTensors() -> void {
	// Assign the properties to the input and output tensors
	for (int i = 0; i < mEngine->getNbIOTensors(); i++) {
		char const* tensorName = mEngine->getIOTensorName(i);
		auto [rank, extents] = mEngine->getTensorShape(tensorName);

		// Multiply sizeof(float) by the product of the extents
		// This is essentially: element count * size of each element
		std::size_t size = 1;
		for(int32_t i = 0; i < rank; ++i){
			size *= extents[i];
		}

		// Create GPU memory for TensorRT to operate on
		if (cudaError_t result = cudaMalloc(mBindings.data() + i, size * sizeof(float)); result != cudaSuccess)
			throw std::runtime_error{"Failed to allocate GPU memory: " + std::string{cudaGetErrorString(result)}};
	}

	assert(mContext);
	// Create an appropriately sized output tensor
	auto const [nbDims, d] = mEngine->getTensorShape(mOutputTensorName.c_str());
	for (int i = 0; i < nbDims; i++) {
		std::array<char, 512> message{};
		std::snprintf(message.data(), message.size(), "Output Size %d %d", i, d[i]);
		mLogger.log(ILogger::Severity::kINFO, message.data());
	}

	// Create the mat wrapper around the output matrix for ease of use
	assert(nbDims == 3);
	assert(d[0] == 1); // TODO: This is cooked       vvvvvv
	mOutputTensor = cv::Mat::zeros(d[1], (!d[2]) ? 1 : d[2], CV_32FC1);
}

auto Inference::getBindingInputIndex(IExecutionContext const* context) -> int {
	// Returns the id for the input tensor
	return context->getEngine().getTensorIOMode(context->getEngine().getIOTensorName(0)) != TensorIOMode::kINPUT; // 0 (false) if bindingIsInput(0), 1 (true) otherwise
}


auto Inference::getInputTensorSize() -> std::vector<int64_t>{
	auto dims =  mEngine->getTensorShape(mInputTensorName.c_str());
	std::vector<int64_t> inputBlobSize;
	inputBlobSize.reserve(dims.nbDims);

	for(int32_t i = 0; i < dims.nbDims; ++i){
		inputBlobSize.push_back(dims.d[i]);
	}

	return inputBlobSize;
}

auto Inference::getOutputTensorSize() -> std::vector<int64_t>{
	auto dims =  mEngine->getTensorShape(mOutputTensorName.c_str());
	std::vector<int64_t> inputBlobSize;
	inputBlobSize.reserve(dims.nbDims);

	for(int32_t i = 0; i < dims.nbDims; ++i){
		inputBlobSize.push_back(dims.d[i]);
	}

	return inputBlobSize;
}
