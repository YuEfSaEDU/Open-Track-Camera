#!/usr/bin/env python3
"""Trains Open Track Camera's aircraft detector with MediaPipe Model Maker.

Takes the folders written by prepare_dataset.py and exports MediaPipe .tflite models with
metadata (labels inside), drop-in replacements for the app's EfficientDet-Lite0:
  aircraft_fp16.tflite  float16 weights (GPU delegate, or CPU)
  aircraft_int8.tflite  int8 (CPU, like the app's current model): quantization aware training
                        for MobileNetV2, post-training quantization otherwise

  python train.py --data /content/dataset --out /content/model --epochs 30
"""
import argparse
import json
import math
import os
import time

import tensorflow as tf
from mediapipe_model_maker import object_detector, quantization

MODELS = {
    "mobilenet_multi_avg_384": object_detector.SupportedModels.MOBILENET_MULTI_AVG_I384,
    "mobilenet_multi_avg_256": object_detector.SupportedModels.MOBILENET_MULTI_AVG,
    "mobilenet_v2_320": object_detector.SupportedModels.MOBILENET_V2_I320,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--model", default="mobilenet_multi_avg_384", choices=MODELS)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--lr", type=float, default=0.3, help="Model Maker's default for batch 32 is 0.3")
    ap.add_argument("--int8", default="auto", choices=["auto", "qat", "ptq", "none"],
                    help="int8 model: quantization aware training (MobileNetV2 only), post-training, or none")
    ap.add_argument("--qat-epochs", type=int, default=10)
    ap.add_argument("--calibration-steps", type=int, default=300)
    ap.add_argument("--int8-input", default="uint8", choices=["uint8", "float32"],
                    help="input type of the post-training int8 model")
    ap.add_argument("--cache", default=None, help="TFRecord cache (local disk; default OUT/_cache)")
    ap.add_argument("--resume", action="store_true",
                    help="continue from the checkpoint in OUT/checkpoint (saved after every epoch)")
    a = ap.parse_args()
    if a.int8 == "auto":
        # Post-training int8 of the MobileNet-MultiAVG models does not run on XNNPACK (the fast
        # CPU path) and is ~3x slower than float16; QAT exists only for MobileNetV2.
        a.int8 = "qat" if a.model.startswith("mobilenet_v2") else "none"

    os.makedirs(a.out, exist_ok=True)
    cache = a.cache or os.path.join(a.out, "_cache")
    t0 = time.time()
    train = object_detector.Dataset.from_coco_folder(os.path.join(a.data, "train"), cache_dir=os.path.join(cache, "train"))
    val = object_detector.Dataset.from_coco_folder(os.path.join(a.data, "val"), cache_dir=os.path.join(cache, "val"))
    print(f"train {train.size} images, val {val.size} images, labels {train.label_names}", flush=True)

    # Checkpoint after every epoch (Model Maker's object detector keeps none until the end), so a
    # disconnected Colab can resume: load it and train the remaining epochs, starting from where
    # the cosine learning rate schedule was.
    ckpt_dir = os.path.join(a.out, "checkpoint")
    progress_file = os.path.join(a.out, "progress.json")
    done = 0
    latest = tf.train.latest_checkpoint(ckpt_dir) if a.resume else None
    if latest and os.path.exists(progress_file):
        done = json.load(open(progress_file))["epochs_done"]
        print(f"resuming from {latest}: {done} of {a.epochs} epochs done", flush=True)
    else:
        latest = None

    class SaveProgress(tf.keras.callbacks.Callback):
        def on_epoch_end(self, epoch, logs=None):
            self.model.save_weights(os.path.join(ckpt_dir, "latest"))
            with open(progress_file, "w") as f:
                json.dump({"epochs_done": done + epoch + 1, "loss": float((logs or {}).get("total_loss", 0))}, f)

    original = object_detector.ObjectDetector._train_model

    def train_with_checkpoints(self, train_data, validation_data, preprocessor=None, checkpoint_path=None):
        self._callbacks = list(self._callbacks or []) + [SaveProgress()]
        return original(self, train_data, validation_data, preprocessor=preprocessor, checkpoint_path=ckpt_dir if latest else None)

    object_detector.ObjectDetector._train_model = train_with_checkpoints
    remaining = max(1, a.epochs - done)
    alpha = 0.05
    lr = a.lr * (alpha + (1 - alpha) * 0.5 * (1 + math.cos(math.pi * done / a.epochs)))
    hp = object_detector.HParams(
        export_dir=a.out,
        epochs=remaining,
        batch_size=a.batch,
        learning_rate=lr,
        cosine_decay_epochs=remaining,
        cosine_decay_alpha=alpha * a.lr / lr,
    )
    opts = object_detector.ObjectDetectorOptions(supported_model=MODELS[a.model], hparams=hp)
    model = object_detector.ObjectDetector.create(train_data=train, validation_data=val, options=opts)
    loss, coco = model.evaluate(val, batch_size=a.batch)
    print("float model:", json.dumps({k: round(float(v), 4) for k, v in coco.items()}), flush=True)
    results = {"float": {k: float(v) for k, v in coco.items()}, "labels": train.label_names, "model": a.model,
               "epochs": a.epochs, "train_images": train.size, "val_images": val.size}

    model.export_model("aircraft_fp16.tflite", quantization_config=quantization.QuantizationConfig.for_float16())

    if a.int8 == "qat":
        # Quantization aware training: best int8 accuracy, but Model Maker supports it only for
        # the MobileNetV2 models.
        qat = object_detector.QATHParams(learning_rate=a.lr / 10, batch_size=a.batch, epochs=a.qat_epochs, decay_steps=6, decay_rate=0.96)
        model.quantization_aware_training(train, val, qat_hparams=qat)
        loss, coco = model.evaluate(val, batch_size=a.batch)
        print("int8 (QAT) model:", json.dumps({k: round(float(v), 4) for k, v in coco.items()}), flush=True)
        results["int8"] = {k: float(v) for k, v in coco.items()}
        model.export_model("aircraft_int8.tflite")
    elif a.int8 == "ptq":
        # Post-training full integer quantization, calibrated on training images. Outputs stay
        # float (MediaPipe's detection post-processing reads float boxes and scores).
        cfg = quantization.QuantizationConfig.for_int8(
            train, quantization_steps=a.calibration_steps,
            inference_input_type=tf.uint8 if a.int8_input == "uint8" else tf.float32, inference_output_type=tf.float32)
        model.export_model("aircraft_int8.tflite", quantization_config=cfg)

    results["minutes"] = round((time.time() - t0) / 60, 1)
    with open(os.path.join(a.out, "results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("done in", results["minutes"], "min ->", a.out, flush=True)


if __name__ == "__main__":
    main()
