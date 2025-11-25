#!/usr/bin/env python3
import argparse
import os

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms


# -------------------------------
# Config defaults
# -------------------------------

FRAC_BITS = 7       # Q0.7 => scale = 2^7
INPUT_DIM = 28 * 28
DEFAULT_HIDDEN_DIM = 32     # ~25k params -> ~25 KB of weights
NUM_CLASSES = 10


# -------------------------------
# Straight-through fake quantizer
# -------------------------------

class STEQuantize(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x, scale, qmin, qmax):
        q = torch.round(x * scale)
        q = torch.clamp(q, qmin, qmax)
        return q / scale

    @staticmethod
    def backward(ctx, grad_output):
        # Straight-through estimator: pass gradient as-is
        return grad_output, None, None, None


def fake_quant_q07(x, frac_bits=FRAC_BITS):
    scale = float(1 << frac_bits)
    qmin, qmax = -128.0, 127.0
    return STEQuantize.apply(x, scale, qmin, qmax)


# -------------------------------
# QAT MLP model
# -------------------------------

class QATMLP(nn.Module):
    def __init__(self, hidden_dim=DEFAULT_HIDDEN_DIM):
        super().__init__()
        self.hidden_dim = hidden_dim

        self.fc1 = nn.Linear(INPUT_DIM, hidden_dim)
        self.act1 = nn.Hardtanh(min_val=-1.0, max_val=1.0)
        self.fc2 = nn.Linear(hidden_dim, NUM_CLASSES)

    def forward(self, x):
        # x: [B, 1, 28, 28], in [-1, 1]
        x = x.view(x.size(0), -1)

        # Quantize input to Q0.7
        x = fake_quant_q07(x)

        # Quantize weights & biases of first layer
        w1 = fake_quant_q07(self.fc1.weight)
        b1 = fake_quant_q07(self.fc1.bias)

        # Linear + hardtanh + quantized activations
        x = torch.nn.functional.linear(x, w1, b1)
        x = self.act1(x)
        x = fake_quant_q07(x)

        # Second layer (logits, no activation; we'll argmax logits later)
        w2 = fake_quant_q07(self.fc2.weight)
        b2 = fake_quant_q07(self.fc2.bias)
        x = torch.nn.functional.linear(x, w2, b2)

        # No softmax: CrossEntropyLoss expects logits
        return x


# -------------------------------
# Training / evaluation
# -------------------------------

def train_epoch(model, loader, optimizer, device):
    model.train()
    criterion = nn.CrossEntropyLoss()

    total_loss = 0.0
    correct = 0
    total = 0

    for data, target in loader:
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()

        output = model(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()

        total_loss += loss.item() * data.size(0)
        _, pred = output.max(1)
        correct += pred.eq(target).sum().item()
        total += data.size(0)

    return total_loss / total, correct / total


@torch.no_grad()
def eval_epoch(model, loader, device):
    model.eval()
    criterion = nn.CrossEntropyLoss()

    total_loss = 0.0
    correct = 0
    total = 0

    for data, target in loader:
        data, target = data.to(device), target.to(device)
        output = model(data)
        loss = criterion(output, target)

        total_loss += loss.item() * data.size(0)
        _, pred = output.max(1)
        correct += pred.eq(target).sum().item()
        total += data.size(0)

    return total_loss / total, correct / total


# -------------------------------
# Quantization helpers for export
# -------------------------------

def quantize_param_to_int8(t, frac_bits=FRAC_BITS):
    """Quantize a float tensor to int8 Q0.7."""
    scale = float(1 << frac_bits)
    q = torch.round(t * scale)
    q = torch.clamp(q, -128, 127)
    return q.to(torch.int8)


def float_image_to_q07(img_tensor, frac_bits=FRAC_BITS):
    """
    img_tensor: [1, 28, 28] in [-1, 1].
    Returns flattened int8 Q0.7.
    """
    flat = img_tensor.view(-1)
    return quantize_param_to_int8(flat, frac_bits=frac_bits)


# -------------------------------
# C file writer
# -------------------------------

def write_c_file(
    filename,
    model,
    test_dataset,
    hidden_dim=DEFAULT_HIDDEN_DIM,
    num_images=16,
    frac_bits=FRAC_BITS
):
    """
    Export weights/biases and a few MNIST test images as a C file compatible
    with your MLP_INT8 implementation.
    """
    model.eval()
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

    # Get float weights/biases
    fc1_w = model.fc1.weight.detach().cpu()
    fc1_b = model.fc1.bias.detach().cpu()
    fc2_w = model.fc2.weight.detach().cpu()
    fc2_b = model.fc2.bias.detach().cpu()

    # Quantize to int8 Q0.7
    W1_q = quantize_param_to_int8(fc1_w, frac_bits)
    b1_q = quantize_param_to_int8(fc1_b, frac_bits)
    W2_q = quantize_param_to_int8(fc2_w, frac_bits)
    b2_q = quantize_param_to_int8(fc2_b, frac_bits)

    # Collect a few test images
    images_int8 = []
    labels = []
    for i in range(num_images):
        img, label = test_dataset[i]
        # img already in [-1, 1] because of transform
        q_img = float_image_to_q07(img, frac_bits=frac_bits)
        images_int8.append(q_img)
        labels.append(int(label))

    with open(filename, "w") as f:
        f.write("/* Auto-generated MNIST int8 MLP parameters (Q0.7).\n")
        f.write(" * Generated by train_mnist_qat_to_c.py\n")
        f.write(" */\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write("#define MLP_INT8_INPUT_DIM   %d\n" % INPUT_DIM)
        f.write("#define MLP_INT8_HIDDEN_DIM  %d\n" % hidden_dim)
        f.write("#define MLP_INT8_OUTPUT_DIM  %d\n" % NUM_CLASSES)
        f.write("#define MLP_INT8_OUTPUT_HARD_TANH\n\n")
        f.write("#include \"pogo-utils/MLP_int8.h\"\n\n")

        # MLP parameters
        f.write("MLP_INT8 mnist_mlp = {\n")

        # W1: [hidden_dim][INPUT_DIM]
        f.write("  .W1 = {\n")
        for i in range(hidden_dim):
            row = ", ".join(str(int(v)) for v in W1_q[i].tolist())
            comma = "," if i != hidden_dim - 1 else ""
            f.write("    { %s }%s\n" % (row, comma))
        f.write("  },\n")

        # b1: [hidden_dim]
        f.write("  .b1 = { %s },\n" %
                ", ".join(str(int(v)) for v in b1_q.tolist()))

        # W2: [NUM_CLASSES][hidden_dim]
        f.write("  .W2 = {\n")
        for i in range(NUM_CLASSES):
            row = ", ".join(str(int(v)) for v in W2_q[i].tolist())
            comma = "," if i != NUM_CLASSES - 1 else ""
            f.write("    { %s }%s\n" % (row, comma))
        f.write("  },\n")

        # b2: [NUM_CLASSES]
        f.write("  .b2 = { %s }\n" %
                ", ".join(str(int(v)) for v in b2_q.tolist()))
        f.write("};\n\n")

        # Images and labels
        f.write("/* %d MNIST test images, quantized to int8 Q0.7 and flattened. */\n"
                % num_images)
        f.write("const int MNIST_NUM_TEST_IMAGES = %d;\n\n" % num_images)

        f.write("const int8_t mnist_images[%d][MLP_INT8_INPUT_DIM] = {\n" %
                num_images)
        for idx, img_q in enumerate(images_int8):
            vals = ", ".join(str(int(v)) for v in img_q.tolist())
            comma = "," if idx != num_images - 1 else ""
            f.write("  { %s }%s\n" % (vals, comma))
        f.write("};\n\n")

        f.write("const uint8_t mnist_labels[%d] = { %s };\n" %
                (num_images, ", ".join(str(l) for l in labels)))

    print(f"Wrote C file: {filename}")


# -------------------------------
# Main
# -------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Train MNIST MLP with QAT and export to C (int8 Q0.7)."
    )
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden-dim", type=int, default=DEFAULT_HIDDEN_DIM)
    parser.add_argument("--num-images", type=int, default=16,
                        help="Number of MNIST test images to export to C.")
    parser.add_argument("--output-c", type=str,
                        default="mnist_mlp_params.c")
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

    # MNIST transforms: map [0,1] -> [-1,1]
    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Lambda(lambda x: x * 2.0 - 1.0),
    ])

    train_dataset = datasets.MNIST(
        root="./data", train=True, download=True, transform=transform
    )
    test_dataset = datasets.MNIST(
        root="./data", train=False, download=True, transform=transform
    )

    train_loader = DataLoader(
        train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=2
    )
    test_loader = DataLoader(
        test_dataset, batch_size=args.batch_size, shuffle=False, num_workers=2
    )

    model = QATMLP(hidden_dim=args.hidden_dim).to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    # Training loop
    for epoch in range(1, args.epochs + 1):
        train_loss, train_acc = train_epoch(model, train_loader, optimizer, device)
        test_loss, test_acc = eval_epoch(model, test_loader, device)

        print(
            f"Epoch {epoch:02d} | "
            f"train loss {train_loss:.4f}, acc {train_acc*100:.2f}% | "
            f"test loss {test_loss:.4f}, acc {test_acc*100:.2f}%"
        )

    # Export parameters and some test images to C
    write_c_file(
        filename=args.output_c,
        model=model.cpu(),
        test_dataset=test_dataset,
        hidden_dim=args.hidden_dim,
        num_images=args.num_images,
        frac_bits=FRAC_BITS,
    )


if __name__ == "__main__":
    main()

