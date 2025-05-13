import tensorrt as trt
import torch
import sys

# TensorRT Logger
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def build_engine(onnx_file_path, engine_file_path):
    """Convert ONNX model to TensorRT engine using TensorRT's PyTorch integration."""
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.max_workspace_size = 1 << 30  # 1GB workspace size for TensorRT

    # Create parser
    parser = trt.OnnxParser(network, TRT_LOGGER)

    # Load ONNX model
    with open(onnx_file_path, "rb") as model_file:
        if not parser.parse(model_file.read()):
            print("ERROR: Failed to parse ONNX model!")
            for error in range(parser.num_errors):
                print(parser.get_error(error))
            sys.exit(1)

    # Build TensorRT engine
    print("Building TensorRT engine...")
    engine = builder.build_engine(network, config)

    if engine is None:
        print("Failed to create TensorRT engine.")
        sys.exit(1)

    # Save serialized engine
    with open(engine_file_path, "wb") as f:
        f.write(engine.serialize())

    print(f"Successfully created TensorRT engine: {engine_file_path}")

# Define file paths
onnx_model_path = "last300E.onnx"
trt_engine_path = "last300E.engine"

# Convert ONNX to TensorRT engine
build_engine(onnx_model_path, trt_engine_path)
