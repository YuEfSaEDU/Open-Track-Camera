#!/usr/bin/env python3
"""Writes train_colab.ipynb: a self-contained Colab notebook (the repository is private, so
the notebook carries prepare_dataset.py and train.py itself). Run after changing either:

  python3 tools/detector/make_notebook.py
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# Versions that work together (Model Maker 0.2.1.4 needs TensorFlow 2.15, i.e. Python <= 3.11;
# Colab's own Python is newer, so everything runs in a Python 3.11 virtual environment).
REQUIREMENTS = [
    "mediapipe-model-maker==0.2.1.4",
    "tensorflow[and-cuda]==2.15.1",
    "tf-keras==2.15.1",
    "keras==2.15.0",
    "tf-models-official==2.15.0",
    "tensorflow-text==2.15.0",
    "tensorflow-addons==0.23.0",
    "tensorflow-hub==0.16.1",
    "tensorflow-datasets==4.9.10",
    "tensorflow-metadata==1.15.0",
    "tensorflow-model-optimization==0.7.3",
    "mediapipe==0.10.14",
    "numpy==1.26.4",
    "opencv-python==4.10.0.84",
    "protobuf==4.25.9",
    "setuptools==80.10.2",
    "pycocotools==2.0.11",
    "remotezip==0.12.6",
]


def md(text):
    return {"cell_type": "markdown", "metadata": {}, "source": text.strip("\n").splitlines(keepends=True)}


def code(text):
    return {"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [], "source": text.strip("\n").splitlines(keepends=True)}


def script_cell(name):
    with open(os.path.join(HERE, name)) as f:
        body = f.read()
    return code(f"%%writefile /content/{name}\n" + body)


def main():
    cells = [
        md("""
# Open Track Camera – hava aracı dedektörü eğitimi

Uygulamanın kendi dedektörünü eğitir: **uçak** (jet, yolcu uçağı, pervaneli, nakliye…),
**helikopter**, **drone** (çok rotorlu ve TB2 / Akıncı / MQ-9 gibi İHA'lar) ve **kuş**
(gökyüzündeki en sık yanlış alarm; uygulama kuşa kilitlenmez).

**Kullanım**
1. *Çalışma zamanı → Çalışma zamanı türünü değiştir → **T4 GPU***.
2. Aşağıdaki ayarları gerekirse değiştir, sonra *Çalışma zamanı → **Tümünü çalıştır***.
3. Google Drive izni istenirse ver: kontroller ve sonuç oraya yazılır; bağlantı koparsa
   *Tümünü çalıştır* kaldığı yerden devam eder.
4. Bitince `aircraft_fp16.tflite` ve `results.json` iner (Drive'da `OpenTrackDetector/` içinde de
   durur). Bu iki dosyayı Claude'a gönder: uygulamanın test videolarında eski modelle karşılaştırıp
   uygulamaya ekleyecek.

**Süre** (T4): veri hazırlama ~20–40 dk, eğitim `small` ile ~1–2 saat. Ücretsiz Colab için `small`,
Colab Pro için `medium` önerilir.

Veri: COCO 2017, Open Images V7, Military Aircraft Detection Dataset (103 askeri tip + 11 yolcu
uçağı), Seraphim Drone Dataset; ayrıca küçük/uzak hedefler için bu nesnelerin küçültülüp gökyüzü ve
bulut arka planlarına yerleştirildiği sentetik görüntüler.
"""),
        code("""
# --- Ayarlar ---
PRESET = "small"   # "small" (~16 bin görüntü), "medium" (~37 bin), "large" (~77 bin)
EPOCHS = 25
BATCH = 32
MODEL = "mobilenet_multi_avg_384"  # 384 px giriş (uygulamanın yakınlaştırılmış tespitine uygun)
USE_DRIVE = True   # kontrol noktaları ve sonuç Google Drive'a (kopmada devam için)
"""),
        code("""
import os, glob, subprocess, sys
gpu = subprocess.run(["nvidia-smi", "-L"], capture_output=True, text=True).stdout
print(gpu or "GPU YOK!")
assert gpu, "Çalışma zamanı türünü T4 GPU yapın (Çalışma zamanı → Çalışma zamanı türünü değiştir)."
OUT = "/content/model"
if USE_DRIVE:
    from google.colab import drive
    drive.mount("/content/drive")
    OUT = "/content/drive/MyDrive/OpenTrackDetector"
os.makedirs(OUT, exist_ok=True)
print("sonuç klasörü:", OUT)
"""),
        md("## 1. Ortam (Python 3.11 + TensorFlow 2.15 + Model Maker)"),
        code("""
REQ = """ + json.dumps(REQUIREMENTS) + """
REQ_ARGS = " ".join('"' + r + '"' for r in REQ)
if not os.path.exists("/content/mm/bin/python"):
    !pip -q install uv
    !uv venv -q /content/mm --python 3.11
    !uv pip install -q --python /content/mm/bin/python {REQ_ARGS}
# TensorFlow 2.15'in GPU kütüphaneleri pip paketlerinden gelir: yollarını göster.
site = "/content/mm/lib/python3.11/site-packages/nvidia"
os.environ["LD_LIBRARY_PATH"] = ":".join(glob.glob(site + "/*/lib")) + ":" + os.environ.get("LD_LIBRARY_PATH", "")
os.environ["PATH"] = ":".join(glob.glob(site + "/cuda_nvcc/bin")) + ":" + os.environ["PATH"]
# Colab's notebook plotting backend does not exist in the Python 3.11 environment.
os.environ["MPLBACKEND"] = "Agg"
PY = "/content/mm/bin/python"
!{PY} -c "import tensorflow as tf; g = tf.config.list_physical_devices('GPU'); print('TensorFlow', tf.__version__, 'GPU:', g); assert g, 'TensorFlow GPU görmüyor'"
"""),
        md("## 2. Betikler"),
        script_cell("prepare_dataset.py"),
        script_cell("train.py"),
        md("## 3. Veri seti"),
        code("""
DATA = "/content/dataset"
if not os.path.exists(f"{DATA}/stats.json"):
    !{PY} /content/prepare_dataset.py --out {DATA} --cache /content/downloads --preset {PRESET}
!cat {DATA}/stats.json
"""),
        md("## 4. Eğitim"),
        code("""
if os.path.exists(f"{OUT}/aircraft_fp16.tflite") and os.path.exists(f"{OUT}/results.json"):
    print("Eğitim zaten bitmiş (yeniden eğitmek için", OUT, "klasörünü sil).")
else:
    resume = "--resume" if glob.glob(f"{OUT}/checkpoint/*.index") else ""
    !{PY} /content/train.py --data {DATA} --out {OUT} --cache /content/tfcache --model {MODEL} --epochs {EPOCHS} --batch {BATCH} {resume}
"""),
        md("## 5. Sonuç"),
        code("""
!cat {OUT}/results.json
!ls -la {OUT}/*.tflite
from google.colab import files
for f in ["aircraft_fp16.tflite", "results.json"]:
    files.download(f"{OUT}/{f}")
"""),
    ]
    nb = {
        "cells": cells,
        "metadata": {
            "accelerator": "GPU",
            "colab": {"provenance": [], "gpuType": "T4"},
            "kernelspec": {"display_name": "Python 3", "name": "python3"},
            "language_info": {"name": "python"},
        },
        "nbformat": 4,
        "nbformat_minor": 0,
    }
    out = os.path.join(HERE, "train_colab.ipynb")
    with open(out, "w") as f:
        json.dump(nb, f, indent=1, ensure_ascii=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
