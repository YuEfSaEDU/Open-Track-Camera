# Mimari

## Genel akış

```
                   ┌──────────── camera thread ────────────┐
 Camera2 (4K60) ──►│ TEMPLATE_RECORD, fps [60,60], ∞ odak, │
                   │ OIS açık, kamera EIS kapalı           │
                   └───────────────┬───────────────────────┘
                                   │ SurfaceTexture (OES doku)
                   ┌───────────────▼──────── render thread (GL) ─────────────────┐
                   │ 1. luma 640x360 (3x3 kutu filtre, 4 piksel/RGBA paketli)     │──► analysis thread
                   │    + ~10 Hz RGBA 640x360 (sinir ağı için)                    │──► detector thread
                   │ 2. FramingController.update(ts, tracker.prediction)          │
                   │ 3. kırpma → MediaCodec giriş yüzeyi (1920x1080, pts = ts)    │──► VideoEncoder ─┐
                   │ 4. önizleme (geniş / çıkış)                                  │                  │
                   │ 5. overlay durumu → UI                                        │                  ▼
                   └──────────────────────────────────────────────────────────────┘             MuxerSink ──► MP4
 Jiroskop (~200-500 Hz) ──► GyroMotionModel ──► (render, analiz, kadraj)            AudioEncoder ─┘
```

| Thread | İş | Bütçe |
|---|---|---|
| `camera` | Camera2 istekleri, kare meta verisi (pozlama, rolling-shutter skew) | — |
| `render` | GL: doku güncelle, analiz geri okuma, kırpma çiz (enkoder + önizleme) | < 16 ms |
| `analysis` | `TargetTracker.process` her kare, gökyüzü dedektörü her 3. (takipteyken 12.) kare | < 16 ms, meşgulse kare atlanır |
| `detector` | MediaPipe EfficientDet, ~10 Hz, asenkron | serbest |
| `gyro` | Sensör örnekleri → açı entegrasyonu | — |
| `video-encoder`, `audio-encoder` | MediaCodec çıktıları → muxer | — |

Analiz iş parçacığı geride kalırsa kareler **kuyruğa alınmaz, atlanır**; gecikme hiçbir zaman birikmez.

## Koordinat sistemleri

- **Kaynak pikselleri (source px)**: sensör akışının ekranda dik (yatay) görünen hali,
  ör. 3840x2160; x sağa, y aşağı. Takipçi ve kadraj her şeyi bu birimde yayınlar.
- **Analiz pikselleri**: 640x360 luma; `scale = 3840/640 = 6` kaynak px / analiz px.
- **Dünya koordinatları**: `dünya = görüntü − CameraMotion.offsetAt(t)`. Kameranın kendi dönüşü
  çıkarılmış konum. Sabit bir yıldız dünya koordinatlarında durur; el titremesi burada görünmez.
- **GL**: `SurfaceTexture` dönüşüm matrisi tamponu cihazın *doğal* yönüne (telefonda dikey) göre
  dik hale getirir. `FrameRenderer` üzerine ekran dönüşünü (`Display.rotation`) uygular.

## Stabilize takip: temel fikir

El titremesi yüksek frekanslı ve jiroskopla gecikmesiz ölçülebilir; uçağın hareketi ise düşük
frekanslı ve fiziksel olarak düzgündür. Bu yüzden ikisi ayrılır:

1. **Jiroskop → görüntü kayması.** Küçük açı yaklaşımıyla θ radyanlık dönüş görüntüyü ≈ f·θ piksel
   kaydırır (f: kaynak piksel cinsinden odak uzaklığı; `LENS_INFO_AVAILABLE_FOCAL_LENGTHS`,
   `SENSOR_INFO_PHYSICAL_SIZE` ve zum oranından). Cihaz eksenlerinden görüntü eksenlerine eşleme
   (`GyroMotionModel.forDisplayRotation`) ekran dönüşüne göre türetilmiştir; yatay (ROTATION_90)
   için `Δx = f·θx`, `Δy = −f·θy`.
2. **Zaman hizalama.** Kamera zaman damgası ilk satırın pozlama başlangıcıdır; jiroskopla
   karşılaştırmak için `skew/2 + exposure/2` eklenir (orta satırın pozlama ortası). Zaman kaynağı
   `REALTIME` değilse `elapsedRealtimeNanos − nanoTime` farkı da eklenir.
3. **Takip dünya koordinatlarında.** Ölçüm `z = görüntüdeki konum − offset(t)`; sabit hız Kalman
   filtresi (`KalmanCv1D`, sürekli beyaz-gürültü ivme modeli, değişken dt) bunu süzer. Jiroskop
   sapması (bias) hem ölçümde hem tahminde aynı ofsetle yer aldığından kısa ufuklarda birbirini götürür.
4. **Kadraj.** Sanal kamera dünya koordinatlarında kritik sönümlü bir yayla hedefi izler
   (hız ileri beslemesiyle sabit hızda gecikme ≈ 0, `leadTimeS` kadar öne bakar). Kırpma merkezi
   `sanal kamera + offset(t)` — yani titreme **aynı karede** geri eklenip iptal edilir.
   Hedef, kırpmanın merkezî %45'lik bölgesinden çıkamaz; kırpma sensör sınırına dayanırsa
   yay "wind-up" yapmasın diye sanal kamera sınırdaki konuma çekilir.
5. **Hedef yokken** hedef = mevcut kare merkezinin dünya konumu; düşük sertlikteki yay bunu izler.
   Bu, klasik EIS'tir: yavaş pan'lar geçer, titreme kesilir.

Jiroskop yoksa `CameraMotion.NONE` kullanılır ve Kalman'ın süreç gürültüsü artırılır (titreme
hedef hareketi gibi görünür, yay ile yumuşatılır).

## Tespit ve takip

`TargetTracker` durum makinesi: `IDLE → TRACKING ⇄ COASTING → LOST → (yeniden yakalama | IDLE)`.

- **Her kare**: Kalman tahmini etrafında (yarıçap = 3σ, 6–48 analiz px) NCC şablon araması
  (2 px kaba ızgara + ince arama + parabolik alt-piksel). Skor ≥ 0.82 ise şablon yavaşça güncellenir.
- **Tespitler** (sinir ağı veya gökyüzü) geçmiş bir kareye aittir; kamera hareketi ve hedef hızıyla
  şimdiki kareye taşınır, kapı (gate) içinde en yakınıyla eşleştirilir. Boyut tahmini tespitlerden gelir.
  Şablon skoru düşükse (< 0.75) şablon tespitten yeniden tohumlanır (drift düzeltme).
- **COASTING**: ölçüm yokken filtre tahminiyle devam (1.2 s), sonra `LOST`; kapı zamanla büyür.
- **Otomatik kilit**: `IDLE` iken en iyi tespit (sinir ağı tespitleri öncelikli).
- **Dokunarak seçim**: dokunulan noktadaki tespit kutusu, yoksa 160 px'lik varsayılan kutu.

### Gökyüzü dedektörü (`SkyBlobDetector`)

Uzaktaki bir uçak 4K karede birkaç piksel olabilir; 640x360'a (ve modelin 320x320 girişine)
küçültülünce sinir ağı onu göremez. Oysa düz gökyüzünde çok belirgindir:

1. 25x25 kutu ortalamasıyla yerel arka plan, fark görüntüsü.
2. Eşik = max(14, 6 × 1.4826 × MAD) — gürültüye göre uyarlanır.
3. Aynı işaretli pikseller için 8-bağlantılı bölge büyütme; alan ve doluluk filtreleri.
4. Blob'un 2x–3x çevresindeki halkanın standart sapması ≤ 9 olmalı ("burası gökyüzü").
   Ağaç, bina, kablo gibi dokulu bölgeler böylece elenir.

GPU tarafındaki 3x3 kutu filtre, 6x küçültmede tek piksellik hedeflerin örnekleme aralığına düşüp
kaybolmasını engeller.

## Kayıt

- Enkoder yüzeyi aynı EGL bağlamına (`EGL_RECORDABLE_ANDROID`) bağlanır; her kare
  `eglPresentationTimeANDROID(kamera ts)` ile gönderilir, böylece kare zamanlaması sensörden gelir.
- `MuxerSink` tüm izlerin formatı gelene kadar örnekleri tamponlar (ilk anahtar kare kaybolmaz),
  zamanları `CLOCK_MONOTONIC`'e çevirip ilk video karesine göre sıfırlar.
- Dosya `MediaStore` üzerinden `Movies/OpenTrack/OTC_yyyyMMdd_HHmmss.mp4` olarak yazılır.

## Cihazda doğrulama listesi

Bu kod bulut ortamında yazıldı; derleniyor ve `tracking` testleri geçiyor ama kamera/GL/sensör
katmanı gerçek cihazda denenmedi. İlk cihaz testinde kontrol edilecekler:

- [ ] **Görüntü yönü**: önizleme dik mi? Değilse `FrameRenderer.imgToTex` dönüş eşlemesi
      (SurfaceTexture matrisinin sensör yönünü içerdiği varsayımı) düzeltilmeli.
- [ ] **Jiroskop işaretleri**: Hedef yokken telefonu sallayınca çıkış görüntüsü sabit kalmalı;
      titreme *iki katına* çıkıyorsa ilgili eksenin işareti ters. `GyroMotionModelTest`'teki
      pan/tilt testleri beklenen yönleri belgeliyor.
- [ ] **Odak uzaklığı ölçeği**: kalan titreme hep aynı yönde ama daha küçükse `focalPx` hatalı.
- [ ] **Zaman ofseti**: hızlı sarsıntıda kısa "geri tepme" görülürse `cameraToGyroOffsetNs` ayarı.
- [ ] Cihazın 4K60 profili seçiliyor mu (durum satırı `3840x2160@60 → 1920x1080@60`)?
- [ ] Render ≥ 59 fps, analiz < 10 ms, NN süresi (durum satırında).
- [ ] Kayıt: ses/görüntü senkronu, 60 fps sabit kare hızı, dosya galeride görünüyor mu.

## Yol haritası

Öncelik sırasına göre öneriler:

1. **Gecikmeli (look-ahead) kayıt**: enkodere kareleri ~10 kare (≈166 ms) geç göndermek; kadraj
   o sırada "gelecekteki" ölçümleri de bilir → nedensel olmayan, çok daha yumuşak kamera yolu
   (post-prodüksiyon stabilizasyonu kalitesi, gerçek zamanlı).
2. **Yüksek çözünürlüklü ROI**: hedef çevresinden tam çözünürlükte 256x256 geri okuma; küçük
   hedeflerde alt-piksel hassasiyet ve ROI üzerinde sinir ağı (tile) tespiti.
3. **Asenkron geri okuma (PBO)** ile render thread'deki `glReadPixels` beklemesini kaldırmak.
4. **Roll (optik eksen etrafında dönme) telafisi** ve rolling-shutter düzeltmesi (satır bazlı
   jiroskop warp'ı) — hızlı pan'larda eğilmeyi azaltır.
5. **Uçak odaklı model**: uzak/küçük uçak veri setiyle eğitilmiş dedektör (YOLO-nano vb.),
   sonra kuş, drone, helikopter, araç gibi sınıflar.
6. **Otomatik zum**: hedefin kadrajdaki boyutunu sabit tutmak (kırpma + lens zumu birlikte).
7. **ADS-B entegrasyonu**: yakındaki uçakların konumundan hedefin ekrandaki yerini tahmin edip
   ön-kilit, kaybolunca arama yönü.
8. Eşzamanlı ham 4K kayıt, 10-bit HEVC / log profil, tam manuel pozlama arayüzü.
