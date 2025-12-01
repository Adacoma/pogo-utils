#!/usr/bin/env python3
import argparse
import os
import math

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torchvision import datasets, transforms

# -------------------------------
# Config defaults
# -------------------------------

FRAC_BITS = 7       # Q0.7 => scale = 2^7
INPUT_DIM = 28 * 28
DEFAULT_HIDDEN_DIM = 32
NUM_CLASSES = 10

# -------------------------------
# PRANC routines
# -------------------------------

UINT32_MAX = 0xFFFFFFFF

def xorshift32_step(state: int) -> int:
    state ^= (state << 13) & UINT32_MAX
    state ^= (state >> 17) & UINT32_MAX
    state ^= (state << 5) & UINT32_MAX
    return state & UINT32_MAX

def pranc_make_basis(seed: int, num_basis: int, D: int, scale: float = 1.0):
    """
    Returns tensor of shape [num_basis, D] with random values in [-scale, scale].
    """
    basis = torch.empty(num_basis, D, dtype=torch.float32)
    state = seed & UINT32_MAX

    for k in range(num_basis):
        for j in range(D):
            state = xorshift32_step(state)
            # map uint32 -> (-1, 1)
            u = (state / 2**31) - 1.0  # in [-1, 1)
            basis[k, j] = u * scale
    return basis

# -------------------------------
# Straight-through fake quantizer
# -------------------------------

def fake_quant_q07(x, frac_bits=FRAC_BITS):
    scale = float(1 << frac_bits)
    q = torch.round(x * scale)
    q = torch.clamp(q, -128, 127)
    q = q / scale
    # STE: forward uses q, backward uses identity
    return x + (q - x).detach()

# -------------------------------
# QAT MLP model (PRANC on W1)
# -------------------------------

class PRANCMLP(nn.Module):
    """
    PRANC only on W1.

    - W1: reconstructed from alpha @ basis (PRANC)
    - b1, W2, b2: dense nn.Parameters, trained normally
    """
    def __init__(self, hidden_dim, num_basis, seed, basis_scale=0.5):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_basis = num_basis
        self.seed = int(seed)

        # Dimensions
        self.n_w1 = hidden_dim * INPUT_DIM   # ONLY W1 is PRANC'ed
        self.D = self.n_w1                   # basis spans R^{n_w1}

        # Fixed random basis for W1
        basis = pranc_make_basis(self.seed, num_basis, self.D,
                                 scale=basis_scale)
        self.register_buffer("basis", basis)  # [K, D_w1]

        # Trainable coefficients α_k
        self.alpha = nn.Parameter(0.01 * torch.randn(num_basis))

        # Dense parameters for b1, W2, b2
        self.b1 = nn.Parameter(torch.zeros(hidden_dim))
        self.w2 = nn.Parameter(torch.empty(NUM_CLASSES, hidden_dim))
        self.b2 = nn.Parameter(torch.zeros(NUM_CLASSES))

        # Init W2 like a Linear layer
        nn.init.kaiming_uniform_(self.w2, a=math.sqrt(5))

        self.act = nn.Hardtanh(min_val=-1.0, max_val=1.0)

    def _w1_float(self):
        # basis: [K, D], alpha: [K]  -> theta: [D]
        theta = torch.mv(self.basis.t(), self.alpha)  # (D,)
        w1 = theta.view(self.hidden_dim, INPUT_DIM)
        return w1

    def forward(self, x):
        # x: [B, 1, 28, 28] in [-1, 1]
        B = x.size(0)
        x = x.view(B, -1)
        x = fake_quant_q07(x)  # quantized input

        # Reconstruct W1 from PRANC, use dense b1/W2/b2
        w1 = self._w1_float()

        # Quantize everything to Q0.7 (STE keeps gradients)
        w1_q = fake_quant_q07(w1)
        b1_q = fake_quant_q07(self.b1)
        w2_q = fake_quant_q07(self.w2)
        b2_q = fake_quant_q07(self.b2)

        x = F.linear(x, w1_q, b1_q)
        x = self.act(x)
        x = fake_quant_q07(x)
        x = F.linear(x, w2_q, b2_q)
        return x  # logits

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
# C file writer for ENSEMBLE
# -------------------------------

def write_pranc_ensemble_c_file(
    filename,
    models,          # list[PRANCMLP]
    test_dataset,
    num_images,
):
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)

    ensemble_size = len(models)
    if ensemble_size == 0:
        raise ValueError("No models provided to export.")

    # assume all models share these
    hidden_dim = models[0].hidden_dim
    num_basis = models[0].num_basis

    # PRANC + dense params for each model
    seeds = []
    alphas = []
    b1_q_list = []
    w2_q_list = []
    b2_q_list = []

    for m in models:
        m.eval()
        seeds.append(int(m.seed))

        alpha = m.alpha.detach().cpu().numpy()
        if alpha.shape[0] != num_basis:
            raise ValueError("All models must use the same num_basis.")
        alphas.append(alpha)

        b1_q_list.append(quantize_param_to_int8(m.b1.detach().cpu()))
        w2_q_list.append(quantize_param_to_int8(m.w2.detach().cpu()))
        b2_q_list.append(quantize_param_to_int8(m.b2.detach().cpu()))

    # Collect test images (shared across models)
    images_int8 = []
    labels = []
    for i in range(num_images):
        img, label = test_dataset[i]
        q_img = float_image_to_q07(img, frac_bits=FRAC_BITS)
        images_int8.append(q_img)
        labels.append(int(label))

    with open(filename, "w") as f:
        f.write("/* Auto-generated MNIST PRANC-W1 ENSEMBLE + dense W2/biases\n")
        f.write(" * for int8 MLP (Q0.7).\n")
        f.write(" * Generated by train_mnist_qat_to_c.py\n")
        f.write(" */\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write("#define MLP_INT8_INPUT_DIM   %d\n" % INPUT_DIM)
        f.write("#define MLP_INT8_HIDDEN_DIM  %d\n" % hidden_dim)
        f.write("#define MLP_INT8_OUTPUT_DIM  %d\n" % NUM_CLASSES)
        f.write("#define MLP_INT8_OUTPUT_HARD_TANH\n\n")
        f.write("#include \"pogo-utils/MLP_int8.h\"\n")
        f.write("#define PRANC_NUM_BASIS %d\n" % num_basis)
        f.write("#include \"pogo-utils/MLP_int8_pranc.h\"\n\n")
        f.write("const int MNIST_ENSEMBLE_SIZE = %d;\n\n" % ensemble_size)

        # PRANC signatures for all models
        f.write("const MLP_PRANC_SIGNATURE mnist_mlp_pranc_sigs[%d] = {\n" %
                ensemble_size)
        for i in range(ensemble_size):
            f.write("  {\n")
            f.write("    .seed = %uu,\n" % seeds[i])
            f.write("    .num_basis = PRANC_NUM_BASIS,\n")
            f.write("    .alpha = {\n")
            alpha = alphas[i]
            for j, a in enumerate(alpha):
                comma = "," if j + 1 < num_basis else ""
                f.write("      %.8ff%s\n" % (float(a), comma))
            f.write("    }\n")
            comma = "," if i + 1 < ensemble_size else ""
            f.write("  }%s\n" % comma)
        f.write("};\n\n")

        # Dense b1
        f.write("const int8_t mnist_b1[%d][MLP_INT8_HIDDEN_DIM] = {\n" %
                ensemble_size)
        for i in range(ensemble_size):
            b1_q = b1_q_list[i].tolist()
            row = ", ".join(str(int(v)) for v in b1_q)
            comma = "," if i + 1 < ensemble_size else ""
            f.write("  { %s }%s\n" % (row, comma))
        f.write("};\n\n")

        # Dense W2: [ENSEMBLE][OUTPUT][HIDDEN]
        f.write("const int8_t mnist_W2[%d][MLP_INT8_OUTPUT_DIM]"
                "[MLP_INT8_HIDDEN_DIM] = {\n" % ensemble_size)
        for i in range(ensemble_size):
            w2_np = w2_q_list[i].numpy()
            f.write("  {\n")
            for o in range(NUM_CLASSES):
                row = ", ".join(str(int(v)) for v in w2_np[o])
                comma = "," if o + 1 < NUM_CLASSES else ""
                f.write("    { %s }%s\n" % (row, comma))
            comma_m = "," if i + 1 < ensemble_size else ""
            f.write("  }%s\n" % comma_m)
        f.write("};\n\n")

        # Dense b2
        f.write("const int8_t mnist_b2[%d][MLP_INT8_OUTPUT_DIM] = {\n" %
                ensemble_size)
        for i in range(ensemble_size):
            b2_q = b2_q_list[i].tolist()
            row = ", ".join(str(int(v)) for v in b2_q)
            comma = "," if i + 1 < ensemble_size else ""
            f.write("  { %s }%s\n" % (row, comma))
        f.write("};\n\n")

        # Images and labels (shared)
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

# -------------------------------
# Main
# -------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Train MNIST PRANC-MLP ensemble with QAT and export to C (int8 Q0.7)."
    )
    parser.add_argument("--epochs", type=int, default=5)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden-dim", type=int, default=DEFAULT_HIDDEN_DIM)
    parser.add_argument("--num-basis", type=int, default=64)
    parser.add_argument("--pranc-seed", type=int, default=12345,
                        help="Base seed; model i uses pranc-seed + i.")
    parser.add_argument("--num-models", type=int, default=5,
                        help="Number of PRANC MLPs in the ensemble.")
    parser.add_argument("--num-images", type=int, default=16,
                        help="Number of MNIST test images to export to C.")
    parser.add_argument("--output-c", type=str,
                        default="mnist_mlp_pranc_params.c")
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

    models = []
    for i in range(args.num_models):
        seed_i = args.pranc_seed + i
        print(f"\n=== Training model {i+1}/{args.num_models} with PRANC seed {seed_i} ===")
        model = PRANCMLP(
            hidden_dim=args.hidden_dim,
            num_basis=args.num_basis,
            seed=seed_i
        ).to(device)
        optimizer = optim.Adam(model.parameters(), lr=args.lr)

        for epoch in range(1, args.epochs + 1):
            train_loss, train_acc = train_epoch(model, train_loader, optimizer, device)
            test_loss, test_acc = eval_epoch(model, test_loader, device)
            print(
                f"Model {i} | Epoch {epoch:02d} | "
                f"train loss {train_loss:.4f}, acc {train_acc*100:.2f}% | "
                f"test loss {test_loss:.4f}, acc {test_acc*100:.2f}%"
            )

        models.append(model.cpu())

    # Export ensemble parameters and some test images to C
    write_pranc_ensemble_c_file(
        filename=args.output_c,
        models=models,
        test_dataset=test_dataset,
        num_images=args.num_images,
    )

if __name__ == "__main__":
    main()

