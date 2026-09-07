
import numpy as np
import pandas as pd
import tensorflow as tf

# --- Config ---
WINDOW_SIZE = 50        # 50 samples @ 100Hz = 0.5s sliding window
STEP = 5                # stride between windows
EPOCHS = 100
BATCH_SIZE = 32

# --- Load & prep data ---
df = pd.read_csv("data.csv")
roll = df["roll"].values.astype(np.float32)
pitch = df["pitch"].values.astype(np.float32)
label = df["label"].values.astype(np.float32)

# Stack roll and pitch as 2 features
features = np.stack([roll, pitch], axis=-1)  # (N, 2)

# Create sliding windows
X, y = [], []
for i in range(0, len(features) - WINDOW_SIZE, STEP):
    window = features[i : i + WINDOW_SIZE]
    # Label = majority vote in window
    window_label = 1.0 if label[i : i + WINDOW_SIZE].mean() > 0.5 else 0.0
    X.append(window)
    y.append(window_label)

X = np.array(X, dtype=np.float32)  # (samples, WINDOW_SIZE, 2)
y = np.array(y, dtype=np.float32)

print(f"Dataset: {len(X)} windows, {y.sum():.0f} positive, {len(y) - y.sum():.0f} negative")

# --- Auto class balancing ---
n_pos = y.sum()
n_neg = len(y) - n_pos
class_weight = {0: n_pos / len(y), 1: n_neg / len(y)}
print(f"Class weights: neg={class_weight[0]:.2f}, pos={class_weight[1]:.2f}")

# --- Normalize: compute per-feature min/max for quantization-friendly scaling ---
feat_min = X.reshape(-1, 2).min(axis=0)
feat_max = X.reshape(-1, 2).max(axis=0)
X = (X - feat_min) / (feat_max - feat_min + 1e-7)

# Save normalization params for inference
np.savez("norm_params.npz", feat_min=feat_min, feat_max=feat_max)
print(f"Normalization — roll: [{feat_min[0]:.2f}, {feat_max[0]:.2f}], pitch: [{feat_min[1]:.2f}, {feat_max[1]:.2f}]")

# --- Shuffle & split ---
idx = np.random.permutation(len(X))
split = int(0.8 * len(X))
X_train, X_val = X[idx[:split]], X[idx[split:]]
y_train, y_val = y[idx[:split]], y[idx[split:]]

# --- Model: tiny dense net (fits in <10KB flash) ---
model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(WINDOW_SIZE, 2)),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(16, activation="relu"),
    tf.keras.layers.Dense(8, activation="relu"),
    tf.keras.layers.Dense(1, activation="sigmoid"),
])

model.compile(optimizer="adam", loss="binary_crossentropy", metrics=["accuracy"])
model.summary()

model.fit(X_train, y_train, validation_data=(X_val, y_val),
          epochs=EPOCHS, batch_size=BATCH_SIZE, class_weight=class_weight)

# --- Extract weights and write C header ---
def write_float_array(f, name, arr):
    flat = arr.flatten()
    f.write(f"static const float {name}[] = {{\n")
    for i, v in enumerate(flat):
        if i % 8 == 0:
            f.write("  ")
        f.write(f"{v:.6f}f, ")
        if i % 8 == 7:
            f.write("\n")
    f.write("\n};\n\n")

weights = []
for layer in model.layers:
    w = layer.get_weights()
    if w:
        weights.append(w)

# weights[0] = Dense(16): kernel (100,16), bias (16)
# weights[1] = Dense(8):  kernel (16,8),  bias (8)
# weights[2] = Dense(1):  kernel (8,1),   bias (1)

with open("src/model.h", "w") as f:
    f.write("#ifndef MODEL_H\n#define MODEL_H\n\n")

    f.write(f"// Normalization params (min/max per feature: roll, pitch)\n")
    f.write(f"static constexpr float NORM_MIN[] = {{{feat_min[0]:.4f}f, {feat_min[1]:.4f}f}};\n")
    f.write(f"static constexpr float NORM_MAX[] = {{{feat_max[0]:.4f}f, {feat_max[1]:.4f}f}};\n")
    f.write(f"static constexpr int MODEL_WINDOW_SIZE = {WINDOW_SIZE};\n\n")

    write_float_array(f, "WEIGHT0", weights[0][0])  # (100, 16)
    write_float_array(f, "BIAS0",   weights[0][1])  # (16,)
    write_float_array(f, "WEIGHT1", weights[1][0])  # (16, 8)
    write_float_array(f, "BIAS1",   weights[1][1])  # (8,)
    write_float_array(f, "WEIGHT2", weights[2][0])  # (8, 1)
    write_float_array(f, "BIAS2",   weights[2][1])  # (1,)

    f.write("#endif // MODEL_H\n")

total_params = sum(w.size for ws in weights for w in ws)
print(f"Exported {total_params} params to src/model.h")
