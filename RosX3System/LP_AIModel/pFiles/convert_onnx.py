import torch
import torchvision.models as models
import torch_tensorrt

# Load a pre-trained ResNet model (example)
model = models.resnet50(pretrained=True)
model.eval().cuda()
example_input = torch.randn(1, 3, 224, 224).cuda()
traced_model = torch.jit.trace(model, example_input)

trt_model = torch_tensorrt.compile(
    traced_model,
    inputs=[torch_tensorrt.Input(example_input.shape)],
    enabled_precisions={torch.float16},  # Use torch.float32 for full precision
    truncate_long_and_double=True
)
torch.jit.save(trt_model, "model_trt.pt")
loaded_trt_model = torch.jit.load("model_trt.pt").cuda()
with torch.no_grad():
    output = loaded_trt_model(example_input)
print(output)
torch.onnx.export(model, example_input, "model.onnx", opset_version=11)
