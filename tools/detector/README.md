# Hava aracı dedektörü (eğitim)

Uygulamanın COCO EfficientDet-Lite0'ının ("airplane" tek sınıf, 320 px) yerine geçecek kendi
modeli. Sınıflar: `airplane`, `helicopter`, `drone`, `bird`. Kuş, gökyüzündeki en sık yanlış
alarmdır; model onu ayırt etmeyi öğrenir, uygulama kuşa kilitlenmez.

## Colab'da eğitim

1. `train_colab.ipynb` dosyasını Colab'da aç (*Dosya → Not defteri yükle*).
2. *Çalışma zamanı türünü değiştir → T4 GPU*, sonra *Tümünü çalıştır*.
3. Bitince inen `aircraft_fp16.tflite` + `results.json` dosyalarını geri gönder.

Not defteri kendi içinde `prepare_dataset.py` ve `train.py`'yi taşır (depo özel olduğu için
klonlanmaz). Bu iki dosyayı değiştirdikten sonra not defterini yeniden üret:

```bash
python3 tools/detector/make_notebook.py
```

## Veri (`prepare_dataset.py`)

| Kaynak | İçerik | Lisans |
|---|---|---|
| COCO 2017 | airplane, bird | CC BY 4.0 (etiketler), Flickr görüntüleri |
| Open Images V7 | Fixed-wing aircraft, Helicopter, Bird (alt türleriyle) | CC BY 4.0 / CC BY 2.0 |
| Military Aircraft Detection Dataset (Kaggle, HF yansısı) | 103 askeri tip + 11 yolcu uçağı; helikopterler `helicopter`, İHA'lar (TB2, Akıncı, Kızılelma, MQ-9…) `drone`, gerisi `airplane`; tiltrotorlar (V-22, V-280) çıkarılır | Kaggle sayfasındaki koşullar |
| Seraphim Drone Detection Dataset | çok rotorlu (ve bazı sabit kanatlı) dronlar, 640x640 | CC BY 4.0 |
| Sentetik küçük hedefler | yukarıdaki nesnelerin kırpıntıları 6–48 px'e (384 girişte) küçültülüp nesnesiz arka planlara (gök, bulut, arazi) renk eşlenerek ve kenarları yumuşatılarak yerleştirilir; %8'i boş (sadece arka plan) | — |

Fotoğraf veri setlerinde uçak çoğunlukla kareyi doldurur; uygulama ise uçağı 320–384 px'lik
tespit girişinde çoğu zaman birkaç on piksel olarak görür. Sentetik küçük hedefler bu açığı kapatır.
1–3 piksellik noktalar sinir ağına değil gökyüzü dedektörüne kalır (o boyut eğitilmez).

Ön ayarlar (görüntü sayısı): `tiny` (yerel deneme), `small` (~16 bin, ücretsiz Colab),
`medium` (~37 bin), `large` (~77 bin). Görüntüler en uzun kenar 1024 px'e küçültülür.
`stats.json` sınıf, kaynak ve 384 px girişteki boyut dağılımını verir.

## Eğitim (`train.py`)

MediaPipe Model Maker, MobileNet-MultiAVG 384 px (varsayılan), kosinüs öğrenme hızı.
Çıktı: `aircraft_fp16.tflite` (etiketler meta veride; MediaPipe ObjectDetector doğrudan açar).
`--resume` son kontrol noktasından (her epoch sonunda yazılır) devam eder.

int8: Model Maker QAT'ı yalnız MobileNetV2 için destekliyor; MultiAVG'nin eğitim sonrası int8'i
XNNPACK'te çalışmıyor (bu makinede 214 ms, float16 76 ms, EfficientDet-Lite0 48 ms). Bu yüzden
varsayılan çıktı float16; `--model mobilenet_v2_320` ile QAT int8 de üretilebilir.

Ortam: Python 3.11 + TensorFlow 2.15 (Model Maker'ın sınırı); sürümler `make_notebook.py`
içindeki `REQUIREMENTS` listesinde sabit. Bu ortamda `tiny` ön ayarla uçtan uca denendi
(veri → eğitim → dışa aktarma → MediaPipe ile çıkarım, sınıf adları doğru).

## Karşılaştırma

```bash
python3 tools/video_bench/run_bench.py VIDEO --live --model aircraft_fp16.tflite \
    --labels airplane,helicopter,drone --roi-input 384 --tag _new
```
