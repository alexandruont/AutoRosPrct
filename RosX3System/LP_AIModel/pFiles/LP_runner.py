import tensorrt as trt
import numpy as np
import pycuda.driver as cuda
import cv2

# Load TensorRT engine
def load_engine(engine_path):
    """Loads a serialized TensorRT engine."""
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def print_engine_info(engine_path):
    """Loads a TensorRT engine and prints input/output details."""
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    # Load the serialized TensorRT engine
    with open(engine_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())

    print("\n===== Engine Information =====")
    for i in range(engine.num_bindings):
        name = engine.get_binding_name(i)
        dtype = engine.get_binding_dtype(i)
        shape = engine.get_binding_shape(i)
        is_input = engine.binding_is_input(i)

        print(f"{'Input' if is_input else 'Output'}: {name}")
        print(f"  - Shape: {shape}")
        print(f"  - Data type: {dtype}\n")
        
def get_tensor_size(shape, dtype):
    """Calculates the size of a tensor in bytes."""
    dtype_size = np.dtype(np.float32).itemsize  # Assume float32 (4 bytes per value)
    size = np.prod(shape) * dtype_size
    return size

def process_yolo_output(output_tensor, conf_threshold=0.5, iou_threshold=0.4):
    """Process YOLOv8 TensorRT output into bounding boxes, class IDs, and confidence scores."""
    
    batch_size, num_outputs, num_detections = output_tensor.shape
    assert batch_size == 1, "Batch size should be 1 during inference"
    
    output_tensor = output_tensor[0]  # Remove batch dimension

    boxes, confidences, class_ids = [], [], []

    for i in range(num_detections):
        row = output_tensor[:, i]  # Get detection `i`
        confidence = row[4]  # Object confidence score

        if confidence > conf_threshold:
            class_id = np.argmax(row[5:])  # Get class with highest probability
            class_prob = row[5:][class_id]  # Probability of detected class

            if class_prob > conf_threshold:
                # Convert YOLO format (center x, center y, width, height) to (x1, y1, x2, y2)
                x_c, y_c, w, h = row[:4]
                x1 = int((x_c - w / 2))
                y1 = int((y_c - h / 2))
                x2 = int((x_c + w / 2))
                y2 = int((y_c + h / 2))

                boxes.append([x1, y1, x2, y2])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply Non-Maximum Suppression (NMS)
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, iou_threshold)

    final_boxes = [boxes[i] for i in indices.flatten()]
    final_confidences = [confidences[i] for i in indices.flatten()]
    final_class_ids = [class_ids[i] for i in indices.flatten()]

    return final_boxes, final_confidences, final_class_ids

def draw_detections(image, boxes, confidences, class_ids, class_names):
    """Draws bounding boxes on an image."""
    for box, conf, cls in zip(boxes, confidences, class_ids):
        x1, y1, x2, y2 = box
        label = f"{class_names[cls]}: {conf:.2f}"
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 Detection", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def infer(engine, input_tensor):
    """Performs inference using the TensorRT engine."""
    context = engine.create_execution_context()

    # Allocate memory
    input_shape = input_tensor.shape
    input_size = input_tensor.nbytes

    d_input = context.mem_alloc(input_size)
    d_output = context.mem_alloc(input_size)
    
    bindings = [int(d_input), int(d_output)]

    # Copy data to GPU
    stream = context.Stream()
    context.memcpy_htod_async(d_input, input_tensor, stream)

    # Run inference
    context.execute_async_v2(bindings, stream.handle, None)

    # Copy results back
    output_tensor = np.empty(input_shape, dtype=np.float32)
    context.memcpy_dtoh_async(output_tensor, d_output, stream)
    stream.synchronize()

    return output_tensor

# Load engine
engine_path = "last300E.engine"
engine = load_engine(engine_path)
c = cv2.VideoCapture(0)
ret,frame= c.read()
output = infer(engine, frame)
draw_detections(process_yolo_output(output))