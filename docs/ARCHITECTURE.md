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

## Otomatik zoom (`AutoZoom`)

Kırpma zumu, kullanıcının belirlediği `[minZoom, maxZoom]` aralığında kendiliğinden değişir:

- **Hedef boyutu sabit**: istenen zoom = `targetFill × kaynakGenişliği / hedefBoyutu` (varsayılan
  hedefin uzun kenarı kırpma genişliğinin %25'i). Boyut tespitlerden (EMA) gelir.
- **Güvenlik için genişleme**: takipçi yalnızca tahmin ediyorsa (COASTING) saniyede ~%45, hedef kırpma
  merkezinden yarı-boyutun %30'undan fazla uzaklaşırsa ayrıca genişler. Hedef yoksa en geniş sınıra
  döner; böylece yeniden yakalama kolaylaşır.
- **Nefes almayı önleme**: log-uzayında çalışır, ±%12 ölü bölge ve histerezis vardır; zoom-in en fazla
  0.45 ln/s, zoom-out 1.2 ln/s (hedefi kaçırmak gevşek kadrajdan daha kötüdür).
- Lens zumu (telefoto geçişi) otomatik değildir: fiziksel kamera değişimi kayıtta sıçrama yapar.
- `nativeZoom` (4K→1080p için 2x) üstü dijital büyütmedir; arayüz bu değerleri ⚠ ile işaretler.

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

Yanlış kilitlenmeye karşı korumalar (gerçek videolarda bulunan hatalardan):

- **Çapa şablonu**: kendini güncelleyen şablonun yanında, son tespit onayında alınmış sabit bir
  "çapa" tutulur. Eşleşme çapaya da benzemeli (NCC ≥ 0.45); aksi halde şablon kayması sayılır.
- **Kontrast kontrolü**: NCC kontrastı yok saydığından silikleşen hedef veya düz, gürültülü gökyüzü
  yüksek skor alabilir. Eşleşen yamanın kontrastı çapanın ≥ %45'i ve ≥ 2.5 gri seviye olmalı.
- **Tutarlı onay**: tespit, çapayı yalnızca şablon eşleşmesiyle aynı yerdeyse yeniler; çelişen bir
  gökyüzü blob'u takibi değiştiremez (parlama/bulut kenarı), yalnızca sinir ağı düzeltebilir.
- **Boyut uyumu**: ilişkilendirilen tespit, hedef boyutunun 1/2.5–2.5 katı (sinir ağı 1/4–4) olmalı.
- **Çözünürlükten bağımsız parametreler** (`TrackerConfig.forSource`) ve kare genişliğinin %20'si ile
  sınırlı kapı: tahmin modunda kapı tüm kareye yayılıp rastgele blob'lara atlamaz.

### Gökyüzü dedektörü (`SkyBlobDetector`)

Uzaktaki bir uçak 4K karede birkaç piksel olabilir; 640x360'a (ve modelin 320x320 girişine)
küçültülünce sinir ağı onu göremez. Oysa düz gökyüzünde çok belirgindir:

1. 25x25 kutu ortalamasıyla yerel arka plan, fark görüntüsü.
2. Eşik = max(14, 6 × 1.4826 × MAD) — gürültüye göre uyarlanır.
3. Aynı işaretli pikseller için 8-bağlantılı bölge büyütme; alan ve doluluk filtreleri.
4. Blob'un 2x–3x çevresindeki (en az arka plan penceresi kadar geniş) halkanın standart sapması
   ≤ 9 olmalı ("burası gökyüzü"). Pencere düz değilse (parlama şeridi, bulut kenarı) kontrast
   kestirimi güvenilmezdir.
   Ağaç, bina, kablo gibi dokulu bölgeler böylece elenir.

GPU tarafındaki 3x3 kutu filtre, 6x küçültmede tek piksellik hedeflerin örnekleme aralığına düşüp
kaybolmasını engeller.

## Kayıt

- Enkoder yüzeyi aynı EGL bağlamına (`EGL_RECORDABLE_ANDROID`) bağlanır; her kare
  `eglPresentationTimeANDROID(kamera ts)` ile gönderilir, böylece kare zamanlaması sensörden gelir.
- `MuxerSink` tüm izlerin formatı gelene kadar örnekleri tamponlar (ilk anahtar kare kaybolmaz),
  zamanları `CLOCK_MONOTONIC`'e çevirip ilk video karesine göre sıfırlar.
- Dosya `MediaStore` üzerinden `Movies/OpenTrack/OTC_yyyyMMdd_HHmmss.mp4` olarak yazılır.

## Gerçek video testleri

`tools/video_bench/run_bench.py` ile açık lisanslı 6 hava gösterisi videosu (Wikimedia Commons ve
archive.org, CC lisanslı; depoya eklenmedi) uygulamadaki Kotlin takip kodu + aynı MediaPipe modeliyle
oynatıldı. Otomatik zoom açık (1x–3x). Videolarda jiroskop verisi yok; yani en zor durum ölçüldü.
"Sinir ağıyla uyum": sinir ağının uçak bulduğu karelerde takip merkezinin o kutunun içinde olma oranı
(zayıf bir referans; sinir ağı küçük uçakları göremez).

| Video | Çözünürlük | Süre (s) | Takipte | Tahmin | Kayıp | Sinir ağıyla uyum | Hedef kırpmada | Zoom | ms/kare |
|---|---|---|---|---|---|---|---|---|---|
| Bournemouth Air Festival 2012 (Red Arrows) | 1920x1080@25 | 60 | 76% | 23% | 1% | 64% | 87% | 1.0–3.0 | 1.3 |
| F-18C Hornet, Turku Airshow 2015 | 1920x1080@25 | 60 | 93% | 7% | 0% | 90% | 95% | 1.0–3.0 | 1.3 |
| F-22 Raptor, Jones Beach 2012 | 640x480@30 | 20 | 91% | 9% | 0% | – | 98% | 1.0–3.0 | 1.0 |
| Jim Parker Airshow 2011 | 1280x720@30 | 16 | 75% | 22% | 3% | 84% | 93% | 1.0–3.0 | 2.0 |
| Red Arrows in Action | 640x480@30 | 59 | 45% | 34% | 20% | 17% | 78% | 1.0–3.0 | 1.2 |
| Wings Over Georgia 2014 | 1920x1080@25 | 23 | 84% | 15% | 0% | 92% | 95% | 1.2–3.0 | 1.3 |

Bu testlerde bulunup düzeltilen hatalar: şablon kayması (silikleşen hedefte boş gökyüzüne kilit),
çözünürlüğe bağlı süreç gürültüsü (640 px videoda kapı tüm kareyi kaplıyordu), parlama şeridi yanında
sahte gökyüzü blob'ları, çapanın tespit kutusu merkezinden alınması (büyük uçaklarda doğru eşleşmeler
reddediliyordu), kayıptan yeniden yakalamanın 8 s beklemesi ve lens toz lekesine kalıcı kilit
(sinir ağı geçersiz kılma kuralı). Örnek: F-18 videosunda sinir ağıyla uyum %0'dan %90'a çıktı.

Bilinen sınırlamalar:
- Red Arrows (480p ev kamerası) videosunda uçaklar çoğu zaman görünmüyor; takipçi zaman zaman duman
  bulutlarına veya seyircilerin düz yüzeylerine (gökyüzü dedektörü) kilitleniyor.
- Kol uçuşunda (birden çok uçak) hedef uçaktan uçağa geçebiliyor; çoklu hedef takibi yok.
- Büyük hedeflerde şablon (en fazla 41x41 analiz pikseli) uçağın merkez kısmını izler; kutu uçağa göre
  biraz kayık görünebilir, kadraj yine de uçağı merkezde tutar.

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
