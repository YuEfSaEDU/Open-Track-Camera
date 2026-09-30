# Mimari

## Genel akış

```
                   ┌──────────── camera thread ────────────┐
 Camera2 (4K60) ──►│ TEMPLATE_RECORD, fps [60,60], ∞ odak, │
                   │ OIS açık, kamera EIS kapalı           │
                   └───────────────┬───────────────────────┘
                                   │ SurfaceTexture (OES doku)
            ┌──────────────────────▼──────────── render thread (GL) ──────────────────────────┐
            │ 1. luma 640x360 (+ ~10 Hz RGBA) geri okuma ─────────────────────────────────────┼──► analysis / detector
            │ 2. nedensel FramingController: zoom + canlı kırpma (önizleme, yakalama penceresi)│
            │ 3. yakalama penceresi (kırpma × 1.25) → YuvRing (Y R8 + UV RG8, D+3 yuva)         │
            │ 4. LookaheadFramer: D kare önceki karenin stabilize kırpması (kayma + roll)       │
            │    → ring'den çiz → MediaCodec (pts = o karenin kamera zamanı) ─────────────────┼──► VideoEncoder ─► Muxer ─► MP4
            │ 5. önizleme (geniş: canlı / çıkış: kaydedilen), overlay                           │
            └──────────────────────────────────────────────────────────────────────────────────┘
 Jiroskop (x, y, z; ~200-500 Hz) ──► GyroMotionModel (poz: kayma + roll)        AudioEncoder ─► Muxer
 Jiroskop yoksa: VisualMotionEstimator (analysis thread, KLT + RANSAC)
```

| Thread | İş | Bütçe |
|---|---|---|
| `camera` | Camera2 istekleri, kare meta verisi (pozlama, rolling-shutter skew) | — |
| `render` | GL: doku güncelle, analiz geri okuma, ring'e yazma, gecikmeli kırpmayı çiz (enkoder + önizleme) | < 16 ms |
| `analysis` | (jiroskop yoksa görsel hareket) + `TargetTracker.process` her kare, gökyüzü dedektörü her 3. (takipteyken 12.) kare | < 16 ms, meşgulse kare atlanır |
| `detector` | MediaPipe EfficientDet, ~10 Hz, asenkron | serbest |
| `gyro` | Sensör örnekleri → 3 eksen açı entegrasyonu | — |
| `video-encoder`, `audio-encoder` | MediaCodec çıktıları → muxer | — |

Analiz iş parçacığı geride kalırsa kareler **kuyruğa alınmaz, atlanır**; gecikme hiçbir zaman birikmez.

## Koordinat sistemleri

- **Kaynak pikselleri (source px)**: sensör akışının ekranda dik (yatay) görünen hali,
  ör. 3840x2160; x sağa, y aşağı. Takipçi ve kadraj her şeyi bu birimde yayınlar.
- **Analiz pikselleri**: 640x360 luma; `scale = 3840/640 = 6` kaynak px / analiz px.
- **Kamera pozu (`CameraPose`)**: kameranın dönüşünün uzaktaki nesnelere etkisi: görüntü merkezi
  etrafında `roll` kadar dönme + `offset` kadar kayma. `görüntü = c + R(roll)(dünya − c) + offset`.
- **Dünya koordinatları**: görüntüden poz çıkarılmış konum (`toWorld`). Sabit bir yıldız dünya
  koordinatlarında durur; el titremesi (kayma ve roll) burada görünmez.
- **Kırpma (`CropWindow`)**: merkez, boyut ve dönme; çıkış pikseli u, görüntüde `merkez + R(dönme)·u`.
- **GL**: `SurfaceTexture` dönüşüm matrisi tamponu cihazın *doğal* yönüne (telefonda dikey) göre
  dik hale getirir. `FrameRenderer` üzerine ekran dönüşünü (`Display.rotation`) uygular.

## Stabilizasyon

DaVinci Resolve gibi masaüstü stabilizörlerin temel fikri: (1) kamera hareketini ölç, (2) kamera
yolunu *geçmişi ve geleceği görerek* pürüzsüzleştir, (3) her kareyi, gerçek yol ile pürüzsüz yol
arasındaki farkı kapatacak şekilde kırp/döndür. Uygulama bunu gerçek zamanlı yapar; "gelecek" için
kaydı birkaç kare geciktirir.

1. **Kamera hareketi.**
   - *Jiroskop* (`GyroMotionModel`): küçük açı yaklaşımıyla θ radyanlık dönüş görüntüyü ≈ f·θ piksel
     kaydırır (f: kaynak piksel cinsinden odak uzaklığı). x/y eksenleri kaymaya, z ekseni görüntü
     roll'una dönüşür (eksen eşlemesi `forDisplayRotation`, işaretler birim testlerde belgeli).
     Kamera zaman damgasına `skew/2 + exposure/2` eklenerek orta satırın pozlama ortası kullanılır.
   - *Görüntüden* (`VisualMotionEstimator`, jiroskop yoksa ve kayıtlı videolarda): ızgaraya yayılmış
     Shi-Tomasi köşeleri, 3 seviyeli piramidal Lucas-Kanade, RANSAC ile rijit dönüşüm (kayma + roll).
     Hedef uçak maskelenir. Düz gökyüzünde (doku yok) güvenilmez işaretlenir ve son hareketi
     sönümleyerek sürdürür.
2. **Takip dünya koordinatlarında.** Kalman filtresi hedefin dünya konumunu izler; titreme ölçümden
   çıkarıldığı için uçağın gerçek, düzgün hareketi kalır.
3. **İleriye bakan kamera yolu (`LookaheadFramer` + `PathSmoother`).** Her kare D kare sonra
   (varsayılan 0.33 s) üretilir. O anda kare için geçmiş 2D ve gelecek D karenin hedefleri bilinir;
   bu simetrik pencerede şu problem çözülür (x, y ve roll için ayrı):
   `min Σ a·huber(x − hedef) + w1·ẋ² + w2·ẍ² + w3·x⃛²` — kısıtlar: kırpma sensörün ve yakalama
   penceresinin içinde, hedef kırpmanın ±%70'inde. Türev ağırlıkları kare süresine göre ölçeklenir
   (fps'ten bağımsız) ve "yumuşaklık" ayarından (kesim frekansı 2 → 0.2 Hz) türetilir. Bantlı
   LDLᵀ ile O(n) çözülür (~mikrosaniyeler).
   - Pencere *hedeflerden* kurulur, önceki çıktılardan değil: geri besleme yok, dolayısıyla salınım
     yok (ilk denenen "kayan ufuk" yaklaşımı gerçek videolarda salındı).
   - Hedef terimi Huber'dir: takipçinin yeniden tohumlama sıçramaları kamerayı daha az çeker.
   - Kısıt ihlali ılımlı bir cezayla bükülür, en son kesin sınırlama yapılır: sert kısıtlar kısa
     pencerede polinom salınımları üretiyordu.
   - Hedef yokken hedef = kare merkezinin dünya konumu: klasik EIS (yavaş pan geçer, titreme kesilir).
4. **Roll (ufuk).** Sanal kameranın roll'u gerçek roll'u yavaşça izler; kırpma aradaki fark kadar
   döndürülür (en fazla ~5°, ve yalnızca döndürülmüş kırpma sensöre sığıyorsa — 1x zoom'da hiç).
5. **Gecikme halkası (`YuvRing`).** Kaydedilecek görüntü D kare sonra belli olduğundan, her karenin
   *yakalama penceresi* (nedensel kırpmanın 1.25 katı) GPU'da YUV olarak saklanır (kare başına
   ~4.9 MB; 0.33 s @ 60 fps ≈ 110 MB). Çıkış, bu yuvadan afin (kayma + dönme + ölçek) örnekleme ile
   çizilir ve o karenin kamera zaman damgasıyla kodlanır; ses senkronu bozulmaz. Kayıt durunca
   bekleyen kareler kısalan bakış penceresiyle tamamlanır.

Ön ayarlar: *hızlı* (0.2 s, yumuşaklık 0.4), *dengeli* (0.33 s, 0.6), *sinematik* (0.5 s, 0.8).
Önizlemede "çıkış" görünümü kaydedilen (gecikmeli, stabilize) görüntüyü gösterir.

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

### Stabilizasyon ölçümleri

Sarsıntı, stabilizörden bağımsız bir ölçümle (OpenCV köşe takibi + RANSAC, `tools/video_bench/shake_metric.py`)
ölçülür: arka planın kare kare hareketinden kamera yolu çıkarılır, 0.5 s'lik hareketli ortalamadan
sapma "sarsıntı" sayılır (kasıtlı yumuşak pan cezalandırılmaz). Değerler 1920 px genişliğe
ölçeklenmiş piksel; çıkışta zoom sarsıntıyı büyütür. Otomatik zoom 1–3x, gecikme 0.5 s, jiroskop
yerine görsel hareket tahmini (kayıtlı videolar):

| Video | Çıkış sarsıntısı (eski → yeni) | Ani sıçrama / ivme (eski → yeni) | Roll ° (eski → yeni) |
|---|---|---|---|
| Bournemouth (Red Arrows, çok uçak) | 81.6 → 59.7 (−27%) | 44.6 → 42.9 | 0.24 → 0.21 |
| Jim Parker Airshow | 21.0 → 12.8 (−39%) | 10.3 → 7.3 (−30%) | 0.20 → 0.18 |
| Red Arrows in Action | 70.2 → 43.3 (−38%) | 52.8 → 31.3 (−41%) | 0.33 → 0.24 |
| Wings Over Georgia | 65.4 → 48.8 (−25%) | 28.8 → 17.6 (−39%) | 0.17 → 0.14 |

F-18 ve F-22 videolarında arka plan düz gökyüzü olduğundan sarsıntı ölçülemez. 2x sabit zoom'da
(Jim Parker) çıkış sarsıntısı 100 px → 17 px.

Yol boyunca bulunan ve düzeltilen hatalar: "kayan ufuk" optimizasyonunun salınması (geri beslemesiz
simetrik pencereye geçildi), sert kısıtların kısa pencerede polinom salınımı üretmesi (ılımlı ceza),
dar hedef sınırının kırpmayı gürültülü ölçüme yapıştırması (merkezleme yumuşak hedef oldu), bantlı
LDLᵀ'de sütun sırası hatası, görsel hareket tahmininde düşük dokulu gökyüzünde özellik bulunamaması
(karelerin %52'si → %97'si güvenilir) ve metrikte zoom'un ötelemeye sızması.

Kayıtlı videolarda jiroskop yok; gerçek cihazda jiroskop kamera hareketini gecikmesiz ve daha
doğru verdiği için sonuçların daha iyi olması beklenir (cihazda doğrulanmalı).

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

1. **Rolling-shutter düzeltmesi**: hızlı pan'larda satır bazlı jiroskop warp'ı (4K sensörde okuma
   ~15–30 ms; uçak takibinde eğilme/jöle etkisi belirgin).
2. **Jiroskop + görüntü füzyonu**: görsel tahminle jiroskobun zaman ofseti ve odak ölçeğini cihazda
   otomatik kalibre etmek; ikisini birleştirip kalan kaymayı gidermek.
3. **Takipçi ölçüm kalitesi**: yeniden tohumlama sıçramaları (şablon ↔ tespit kutusu merkezi) hâlâ
   kalan sarsıntının ana kaynağı; kutu merkezi yerine şablon kaydı ile hizalama.
4. **Yüksek çözünürlüklü ROI**: hedef çevresinden tam çözünürlükte geri okuma; küçük hedeflerde
   alt-piksel hassasiyet ve ROI üzerinde sinir ağı (tile) tespiti.
5. **Asenkron geri okuma (PBO)** ile render thread'deki `glReadPixels` beklemesini kaldırmak.
6. **Uçak odaklı model**: uzak/küçük uçak veri setiyle eğitilmiş dedektör, sonra helikopter, drone,
   kuş gibi sınıflar.
7. **Hibrit zoom**: otomatik zoom kırpma sınırına dayanınca lens zumunu kontrollü değiştirmek.
8. **ADS-B entegrasyonu**: yakındaki uçakların konumundan hedefin ekrandaki yerini tahmin edip
   ön-kilit, kaybolunca arama yönü.
9. Eşzamanlı ham 4K kayıt, 10-bit HEVC / log profil, tam manuel pozlama arayüzü.
