import torch
from SPNet.src.networks import V2Net

dims = [192, 384, 768, 1536]
depths = [3, 3, 27, 3]
dp_rate = 0.2
norm_type = 'CNX'
model_dir = 'Large_300.pth'

device = 'cuda' if torch.cuda.is_available() else 'cpu'  # the exported model is the same
net = V2Net(dims, depths, dp_rate, norm_type).to(device).eval()
net.load_state_dict(torch.load(model_dir, map_location=device)['network'])

rgb = torch.randn(1, 3, 480, 640, device=device)
depth = torch.randn(1, 1, 480, 640, device=device)
mask = torch.ones_like(depth)
mask[depth == 0] = 0

torch.onnx.export(
    net,
    (rgb, depth, mask),
    "spnet_480_640.onnx",
    input_names=["rgb", "depth", "mask"],
    output_names=["pred"],
    dynamic_axes={
        "rgb": {0: "batch", 2: "height", 3: "width"},
        "depth": {0: "batch", 2: "height", 3: "width"},
        "mask": {0: "batch", 2: "height", 3: "width"},
        "pred": {0: "batch", 2: "height", 3: "width"},
    },
    opset_version=17
)

print("ONNX Export ok.")