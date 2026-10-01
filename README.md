# Open Track Camera

Android için **stabilize takip (tracking) kamerası**. Sensörden 4K60 akış alır, hedefi
(ilk aşamada **uçak**) bulur ve kilitler, hedefi merkezde tutan ve el titremesini
jiroskopla iptal eden hareketli bir kırpma (crop) penceresiyle **1080p60** video kaydeder.

```
 3840x2160 @60 sensör akışı
 ┌──────────────────────────────────────────┐
 │                                          │
 │        ┌──────────────┐                  │
 │        │   ✈ hedef    │ ← 1920x1080      │
 │        │  (merkezde)  │   kırpma (1:1)   │
 │        └──────────────┘   hedefle kayar  │
 │                                          │
 └──────────────────────────────────────────┘
```

## Özellikler (v0.1)

- **4K60 → 1080p60 kırpma**: 2x kırpma yakınlaştırmasında piksel kaybı yok (1:1). Kırpma zumu
  1x–4x ayarlanabilir; kamera zumu (`CONTROL_ZOOM_RATIO`) ayrıca ayarlanabilir (bazı telefonlarda
  belli bir oranın üstünde telefoto kameraya geçer; günlükteki odak uzaklığı bunu gösterir).
- **Hibrit hedef tespiti**
  - Sinir ağı: MediaPipe EfficientDet-Lite0 (COCO `airplane` sınıfı), ~10 Hz, asenkron.
  - Gökyüzü dedektörü: düz gökyüzünde birkaç piksellik uzak uçakları bulan klasik yöntem
    (yerel arka plan farkı + sakin ve parlak halka testi). Sinir ağının göremediği küçük hedefler için.
  - **Yakınlaştırılmış tespit**: hedef varken sinir ağı çoğunlukla hedefin çevresine (4K'da ~1:1)
    bakar, hedef yokken kareyi karolarla tarar. Gökyüzü dedektörünün bulduğu her şey sinir ağıyla
    **doğrulanır**; tanınmayan (kalabalık, deniz, duman) takip bırakılır. Onaylanana kadar kadraj
    ve zum hedefe dönmez (bulut kenarı, yer lekesi görüntüyü oynatmaz).
  - **Yumuşak kilitlenme**: kadraj hedefe ~0.8 s'de süzülerek geçer, tek karede atlamaz.
- **60 fps takip**: NCC şablon eşleme (alt-piksel), sabit hız Kalman filtresi, kaybolunca tahminle
  devam (coasting) ve tespitlerle yeniden yakalama.
- **İleriye bakan stabilizasyon** (DaVinci Resolve'un yaklaşımının gerçek zamanlı hali):
  kamera hareketi jiroskoptan (jiroskop yoksa görüntüden, KLT + RANSAC) ölçülür; kayıt birkaç kare
  (0.2–0.5 s) geciktirilir ve kamera yolu geçmiş + gelecek kareler görülerek optimize edilir
  (hız/ivme/sarsıntı cezası, kırpma ve hedef kısıtları). Kayma ve **roll (ufuk)** birlikte düzeltilir.
  Geçmiş kareler GPU'da YUV halka tamponda tutulur. Hedef yokken normal EIS gibi çalışır.
  Ön ayarlar: uçağa kilit / hızlı / dengeli / sinematik.
- **Perspektif doğru kırpma**: hedef sensörün kenarına kaydığında görüntü düz kesilmez, o yöne
  dönmüş sanal bir kameranın görüntüsü olarak çizilir. Geniş açılı lensin kenarındaki uzama ve
  yamukluk çıkışa taşınmaz.
- **Rolling shutter düzeltmesi**: jiroskopla her satırın okunma anındaki kamera pozu hesaplanır;
  pan'daki eğilme ve titremedeki "jöle" etkisi kayıtta GPU'da düzeltilir.
- **Uçak fotoğrafçılığı ayarları**: sonsuza sabit odak (gökyüzünde AF aramasını önler), EV,
  enstantane önceliği (oto, 1/250–1/2000 s; varsayılan 1/500) — ISO, kameranın otomatik pozlamasının
  parlaklığını koruyacak şekilde sürekli ayarlanır; stabilize kırpmada kare içi hareket bulanıklığını önler.
  Seçilen enstantane tercih edilen değerdir: ISO en altta (ör. 100) olduğu halde görüntü hâlâ parlaksa
  enstantane kısalır (1/1000, 1/2000…); karanlıkta ISO 800'e varınca (gürültü) enstantane kare
  süresinin yarısına kadar uzar (60 fps'te 1/120), ancak ondan sonra ISO yükselmeye devam eder.
  Hedef parlaklık, kameranın otomatik pozlaması oturduktan sonra öğrenilir (oturmazsa orta gri) ve
  **EV kaydırıcısıyla −3…+3 (1/3 adım)** kaydırılır; enstantane "oto" iken EV kameranın pozlama
  telafisidir. OIS açık / kamera EIS kapalı.
  Parlaklık **kaydedilen kırpmada** ölçülür (kayda girmeyen yer/güneş etkilemez), kilitli hedef
  ağırlıklıdır (parlak gökte uçak siluet kalmaz); kırpmanın parlak ucu patlıyorsa pozlama en fazla
  1 durak düşürülür. **Enstantane: akıllı** seçeneğinde süre, hedefin (hedef yoksa sahnenin)
  sensörde kayma hızından seçilir: çıkışta ~1.5 pikselden fazla bulanıklık olmayacak kadar kısa,
  gerisinde olabildiğince uzun (1/4000 – 1/120 s), daha az ISO gürültüsü için.
- **Otomatik zoom**: kullanıcının belirlediği min/max sınırlar içinde hedefi çıkışta sabit boyutta
  tutar; hedef yalnızca tahminle izlenirken veya kadrajın kenarına kayarken genişler, hedef yokken
  en geniş sınıra döner (yeniden yakalamak için). Ölü bölge ve hız sınırlarıyla "nefes alma" yapmaz.
- **Galeriden işleme**: telefonun kendi kamerasıyla çekilmiş bir video (ör. üreticinin yalnızca kendi
  uygulamasına açtığı 4K60) seçilir; takip, ileriye bakan stabilizasyon ve otomatik zoom aynı şekilde
  uygulanır, 1080p sonuç ve orijinal ses `Movies/OpenTrack` klasörüne yazılır. Kamera hareketi
  görüntüden kestirilir (kayıtlı videoda jiroskop yok).
- **Kayıt**: donanım H.264 (isteğe bağlı HEVC) 40 Mbps + AAC ses, `Movies/OpenTrack` klasörüne.
- **Kayıt günlüğü (telemetri)**: her kaydın yanına `Download/OpenTrack/OTC_….jsonl` yazılır: kare
  kare kamera ayarları (enstantane, ISO, odak uzaklığı, lens), pozlama döngüsünün kararları, tespitler,
  takip durumu, canlı ve kaydedilen kırpma — her biri videodaki zamanıyla. "Günlük" düğmesiyle
  kapatılabilir; özet için `python3 tools/telemetry/summarize.py OTC_….jsonl --events`.
- **Lens seçimi**: "Lens" düğmesi telefonun arka lensleri arasında geçer (ana, geniş açı, tele).
  Telefonun diğer uygulamalara listelemediği lensler de "(gizli)" olarak denenir; açılmazsa ana lense
  dönülür. Kamera raporunda "Arka lensler" satırı hangi lenslerin görüldüğünü gösterir.
- **Arayüz**: Geniş görünüm (tüm sensör + kırpma çerçevesi) / Çıkış görünümü (kaydedilen görüntü),
  dokunarak hedef seçme, otomatik kilit.

## Derleme

Gereksinimler: JDK 17+, Android SDK (platform 35). Tespit modeli ilk derlemede otomatik indirilir.

```bash
./gradlew :tracking:test        # takip/filtre/kadraj birim testleri (cihaz gerekmez)
./gradlew :app:assembleDebug    # APK: app/build/outputs/apk/debug/app-debug.apk
```

Minimum Android 11 (API 30). 4K60 için cihazın arka kamerasının `SurfaceTexture` çıkışında
3840x2160@60 desteklemesi gerekir; yoksa sırayla 4K30, 1440p60, 1080p60, 1080p30 denenir.

## Gerçek videolarda test

```bash
pip install imageio-ffmpeg numpy opencv-python-headless mediapipe
./gradlew :bench:installDist
python3 tools/video_bench/run_bench.py video1.mp4 video2.webm --out bench_out --auto-zoom --zoom-min 1 --zoom-max 3
# uygulamadaki gibi yakınlaştırılmış (ROI) sinir ağı tespitiyle:
python3 tools/video_bench/run_bench.py video1.mp4 --out bench_out --auto-zoom --live
# bağımsız referans (yoğun, karo karo tam çözünürlük tespit) → gt_* metrikleri:
python3 tools/video_bench/gt_tiles.py video1.mp4 --out bench_out
# eski (nedensel) kadrajla karşılaştırma:
python3 tools/video_bench/run_bench.py video1.mp4 --out bench_out --auto-zoom --tag _old --bench-args "--stabilizer old --motion none"
# rolling shutter shader'ı (başsız Chromium WebGL2):
NODE_PATH=$(npm root -g) node tools/shader_check/rolling_shutter.mjs
```

Her video için `bench_out/` altında takip CSV'si, metrikler (`summary.json`: takip oranları, giriş/çıkış
sarsıntısı, roll, hedef titremesi), sol tarafı geniş görünüm (beyaz: kırpma, yeşil: hedef, mavi: sinir
ağı) ve sağ tarafı kaydedilecek çıkış olan bir `.mp4` ile kare kolajı üretilir. Sarsıntı, stabilizörden
bağımsız olarak OpenCV ile ölçülür. Kayıtlı videolarda jiroskop verisi olmadığından kamera hareketi
görüntüden tahmin edilir.

## Proje yapısı

| Modül | İçerik |
|---|---|
| `tracking/` | Saf Kotlin/JVM: Kalman, NCC takipçi, gökyüzü dedektörü, jiroskop hareket modeli, kadraj kontrolcüsü. Cihazsız test edilir. |
| `bench/` + `tools/video_bench/` | Gerçek videolarda test: aynı Kotlin takip kodu + MediaPipe dedektörü, metrikler ve yan yana karşılaştırma videosu. |
| `tools/shader_check/` | Uygulamanın GLSL kodunu tarayıcıda (WebGL2) sentetik verilerle doğrular. |
| `app/` | Android: Camera2, OpenGL ES render hattı, MediaPipe dedektörü, MediaCodec/MediaMuxer kayıt, arayüz. |

Ayrıntılı tasarım, koordinat sistemleri ve yol haritası: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Durum

Bu ilk sürüm derleniyor ve algoritma katmanı birim testlerinden geçiyor, ancak **henüz gerçek
cihazda denenmedi**. Cihazda doğrulanması gerekenler için
[docs/ARCHITECTURE.md → Cihazda doğrulama listesi](docs/ARCHITECTURE.md#cihazda-doğrulama-listesi).

## Lisans

MIT
