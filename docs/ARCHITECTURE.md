# Mimari

## Genel akış

```
                   ┌──────────── camera thread ────────────┐
 Camera2 (4K60) ──►│ TEMPLATE_RECORD, fps [60,60], ∞ odak, │
                   │ OIS açık, kamera EIS kapalı           │
                   └───────────────┬───────────────────────┘
                                   │ SurfaceTexture (OES doku)
            ┌──────────────────────▼──────────── render thread (GL) ──────────────────────────┐
            │ 1. luma 640x360 + ~10 Hz RGBA (tam kare veya hedef çevresi 320x320) geri okuma ──┼──► analysis / detector
            │ 2. nedensel FramingController: zoom + canlı kırpma (önizleme, yakalama penceresi)│
            │ 3. yakalama penceresi (kırpma × 1.25) → YuvRing (Y R8 + UV RG8, D+3 yuva)         │
            │ 4. LookaheadFramer: D kare önceki karenin stabilize kırpması (kayma + roll)       │
            │    → ring'den çiz (+ rolling shutter düzeltmesi) → MediaCodec (pts = kamera zamanı)─┼──► VideoEncoder ─► Muxer ─► MP4
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
| `detector` | MediaPipe EfficientDet, ~10 Hz, asenkron; tam kare veya yakınlaştırılmış bölge (`DetectionPlanner`) | serbest |
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

6. **Uçağa kilit (nokta takibi, varsayılan).** Editörlerdeki nokta izleyici gibi: kadraj, takipçinin
   *her karede ölçtüğü* ham takip noktasına (alt-piksel şablon eşleşmesi) kilitlenir, Kalman
   filtresinin gecikmeli tahminine değil. Uçak çıkışta yerinde durur; el titremesi, uçağın ölçülen
   konumunun içinde olduğu için kendiliğinden iptal olur ve ileriye bakan pencere yalnızca ölçüm
   gürültüsünü yumuşatır (hedef ağırlığı 20, hedef sınırı ±%30).
   - **Sabit takip noktası**: nokta, tespit kutusunun merkezine göre konumu öğrenilerek tutulur;
     sinir ağı hedefi yeniden yakaladığında şablon kutu merkezine değil aynı fiziksel noktaya
     oturur (önceden her yeniden yakalamada nokta ±50 px zıplıyordu).
   - **Gökyüzünde referans uçağın kendisi**: arka planda doku yoksa (görsel hareket güvenilmez)
     takipçi uçağı kendi görüntü içi hareketiyle arar ve beklenmeyen hareketi kameraya atfederek
     pozu düzeltir; uçağın dünya yolu düzgün kalır.

7. **Rolling shutter düzeltmesi (`RollingShutter`, jiroskop varsa).** Sensör satırları sırayla
   okunur (4K'da ~10–30 ms, `SENSOR_ROLLING_SHUTTER_SKEW`); kamera dönerken her satır farklı bir
   pozla çekilir: pan'da görüntü eğilir, titremede "jöle" gibi dalgalanır. Stabilize edilip
   büyütülünce bu belirginleşir. Karenin pozu orta satıra aittir; y satırındaki nokta
   `t + skew·(y/H − 0.5)` anında çekilmiştir.
   - *Çıkış*: `YuvRing` çıkış shader'ı, her çıkış pikseli için ideal (global shutter) konumdan
     ham karedeki konuma rijit düzeltmeyi (kayma + dönme) okuma yönündeki 9 örnekten doğrusal
     aradeğerle uygular. Örnekler jiroskop pozlarından kare başına bir kez hesaplanır.
   - *Takip*: `TargetTracker` görüntü ↔ dünya dönüşümünü noktanın kendi satırının zamanıyla yapar;
     hızlı pan'da uçağın dünya yolu satır konumuna göre bükülmez.
   - Okuma yönü, sensör yönü (`SENSOR_ORIENTATION`) ve ekran dönüşünden hesaplanır (yatay tutuşta
     yukarıdan aşağı). Jiroskop yoksa kapalıdır: görüntüden kestirilen hareketin satır zamanlaması yok.
   - Doğrulama: `RollingShutterTest` (matematik, 0.5 px altı hata) ve `tools/shader_check`
     (gerçek GLSL kodu, başsız Chromium WebGL2'de sentetik rolling-shutter karesinde çalıştırılır:
     ortalama luma hatası düzeltmesiz 0.053 → düzeltmeli 0.003, yeniden örnekleme tabanı).

Ön ayarlar: *uçağa kilit* (0.33 s, kilit), *hızlı* (0.2 s, yumuşaklık 0.4), *dengeli* (0.33 s, 0.6),
*sinematik* (0.5 s, 0.8).
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
  Boyut yalnızca gökyüzü blob'undan geliyorsa (uçağın koyu çekirdeği) sinir ağı kutusu 8 katına kadar
  büyük olabilir; ilk sinir ağı onayında boyut doğrudan kutudan alınır.
- **Çözünürlükten bağımsız parametreler** (`TrackerConfig.forSource`) ve kare genişliğinin %20'si ile
  sınırlı kapı: tahmin modunda kapı tüm kareye yayılıp rastgele blob'lara atlamaz.

### Yakınlaştırılmış tespit ve doğrulama (`DetectionPlanner`)

SOLOTÜRK videolarında (kullanıcının kendi 4K60 çekimleri) asıl sorun buydu: tam kare 320x320'ye
küçültülünce uzaktaki F-16 birkaç piksel kalıyor; sinir ağı 10 denemenin ancak 1–3'ünde uçağı
buluyordu. Takip gökyüzü blob'larına kalıyor, blob dedektörü de kalabalıkta, denizde, dumanda
yanlış hedeflere kilitlenebiliyordu ve bunu düzeltecek bir şey yoktu.

- **Hedef çevresinde tespit**: hedef varken her 3 sinir ağı çalışmasından 2'si, hedefin
  çevresindeki kare bir bölgeye bakar (kenar = max(5 × hedef boyutu, kare yüksekliğinin %15'i),
  görülmediği her saniye büyür) — 4K'da yaklaşık 1:1 çözünürlük. Üçüncüsü tam karedir (yeni
  uçaklar, geçersiz kılma). Bölge GPU'da 3x3 kutu filtreyle 320x320'ye çizilir.
- **Tarama**: hedef yokken tam kareler arasında kare, yüksekliğin yarısı boyutunda karolarla taranır.
- **Doğrulama**: sinir ağı olmayan kaynaktan (gökyüzü blob'u) başlayan takip, uçağı tanıyabilecek
  kadar yakından bakan (hedef girişte ≥ 14 px ve kaynakta ≥ 12 px) 5 çalışmada onaylanmazsa bırakılır
  ve o nokta 3 s yok sayılır. Onaylı bir takip de 12 çalışma boyunca onay almazsa bırakılır (dumana,
  buluta, kalabalığa kaymıştır). Kullanıcının seçtiği hedefe uygulanmaz. Takipçi her çalışmanın
  nereye baktığını (`NeuralRun`) bilir; "uçak değil" ile "bakılmadı" ayrılır.
- **Onaysız takip hemen yol verir**: blob takibi sürerken sinir ağı başka yerde uçak görürse hemen
  ona geçilir (onaylı takipte kural eskisi gibi: 4 çelişki ve 1.5 s).
- **Duman izi**: şablon uçağın hemen yanında (genelde kendi duman izinde) eşleşmeye devam ederken
  sinir ağı arka arkaya 2 kez çelişirse şablon, skoru yüksek olsa da yeniden tohumlanır.
- **Sinir ağı önce**: aynı karede blob ve sinir ağı tespiti varsa ilişkilendirmede sinir ağı önceliklidir.
- **Tahmin modunda hız sönümü**: ölçüm yokken hız 0.5 s zaman sabitiyle azalır; buluta giren uçak
  kadrajı düz bir çizgide kare dışına sürüklemez.

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
5. Halka ayrıca gökyüzü kadar parlak olmalı (karenin 95. yüzdelik parlaklığının ≥ %60'ı): düz ama
   koyu bir mont, deniz veya gölge gökyüzü değildir.

GPU tarafındaki 3x3 kutu filtre, 6x küçültmede tek piksellik hedeflerin örnekleme aralığına düşüp
kaybolmasını engeller.

## Galeriden işleme (`VideoProcessor`)

Bazı üreticiler (ör. Honor 200 Pro) diğer uygulamalara normal oturumda en fazla 30 fps veriyor;
4K60 yalnızca kendi kamera uygulamalarında var (kamera raporu: tüm akışlar @30, yalnızca 1080p
yüksek hız modu 120/240). Çözüm: kendi kamerayla 4K60 çek, sonra uygulamada işle.

- `MediaExtractor` + `MediaCodec` çözücü → `SurfaceTexture` (OES); canlı hattaki parçalar aynen:
  luma geri okuma, `VisualMotionEstimator`, `DetectionPlanner` ile yakınlaştırılmış sinir ağı tespiti,
  `TargetTracker`, `FramingController`, `YuvRing`, `LookaheadFramer` → enkoder.
- Her kare işlenir (canlıdaki gibi atlanmaz), tespitler aynı kareye uygulanır (gecikme yok).
- Video döndürme bilgisi (`rotation`) `FrameRenderer` dönüşüyle uygulanır. Ses izi kopyalanır.
- Jiroskop yok: rolling shutter düzeltmesi yapılmaz; telefonun kendi stabilizasyonu kapalı
  çekilmiş videolar en iyi sonucu verir. Gerçek video testlerindeki ölçümler bu durumu temsil eder.

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

**Uçağa kilit + yakın kadraj** (varsayılan). Önceki sürümde uçak kırpma merkezinden uzakta kalıyor,
otomatik zoom da bu yüzden geri açılıyordu ("görüntü uzak"). Sabit takip noktası, nokta kilidi,
görüntü içi arama ve daha sıkı zoom (doluluk 0.35, geri açılma eşiği 0.6) ile, zoom sınırı 3x:

| Video | Ortalama zoom (ilk → önceki → yeni) | Uçağın merkezden uzaklığı* | Çıkış sarsıntısı |
|---|---|---|---|
| Jim Parker | 2.06 → 1.78 → 2.08 | 0.34 → 0.58 → 0.36 | 21.3 → 25.7 → 18.6 |
| Wings Over Georgia | 2.74 → 2.40 → 2.81 | 0.33 → 0.70 → 0.36 | 62.0 → 49.9 → 65.5 |
| Red Arrows in Action | 1.71 → 2.41 → 2.67 | 1.14 → 0.69 → 0.53 | 75.3 → 75.0 → 57.1 |

\* yarım kırpma boyutuna oranla (0 = tam merkez, 1 = kenar). Daha yakın kadraj aynı açısal
titremeyi daha fazla piksele büyütür; bu yüzden en yakın kadrajda piksel sarsıntısı bazen artar.
Takip oranı Jim Parker'da düştü (%78 → %69), diğerlerinde arttı.

### SOLOTÜRK videoları (4K60, telefonla çekim) ve bağımsız referans

Kullanıcının Antalya'da çektiği 9 SOLOTÜRK F-16 videosu (3840x2160@60, toplam ~3.5 dk): gri kapalı
gökyüzü, çoğu zaman çok küçük hedef, alt yarıda kalabalık ve bayraklar, klip sonlarında kamera yere
dönüyor. Önceki metrikler takipçinin kendi tespitlerine dayandığından, **bağımsız bir referans**
eklendi: `tools/video_bench/gt_tiles.py` aynı modeli saniyede 5 kez, üst üste binen 640 px karolarda
(tam çözünürlüğe yakın) çalıştırır; telefonda çalışamayacak kadar yavaş ama takipçiden bağımsızdır.
Görsel kontrolde kalabalıkta yanlış pozitif yoktu. Metrikler, referansın uçak bulduğu kareler üzerinden:

- *uçakta*: takip noktası bir uçak kutusunun içinde (kenar payı %25);
- *yanlış hedefte*: takip var ama hiçbir uçağın üzerinde değil;
- *kadrajda*: uçak (kutu merkezi) kaydedilen kırpmanın içinde;
- *iyi kadraj*: uçak, merkeze yarım kırpma boyutunun yarısından yakın.

Otomatik zoom 1–3x, uçağa kilit ön ayarı, jiroskop yok (görsel hareket), `run_bench.py --live`
(uygulamadaki gibi yakınlaştırılmış tespit). Önceki sürüm → yeni sürüm:

| Video | Uçakta | Kadrajda | İyi kadraj |
|---|---|---|---|
| 170136 | 33% → 94% | 39% → 94% | 36% → 94% |
| 170426 | 24% → 80% | 39% → 93% | 26% → 78% |
| 170554 | 61% → 91% | 61% → 93% | 59% → 91% |
| 170741 | 53% → 84% | 74% → 85% | 50% → 76% |
| 171348 | 53% → 71% | 73% → 98% | 55% → 79% |
| 171638 | 63% → 92% | 64% → 94% | 64% → 94% |
| 171811 | 33% → 70% | 43% → 97% | 27% → 53% |
| 172002 | 48% → 86% | 54% → 100% | 44% → 80% |
| 172109 | 2% → 96% | 9% → 96% | 0% → 80% |
| **Ortalama** | **41% → 85%** | **51% → 94%** | **40% → 81%** |

Yanlış hedefte geçen süre %58'den %10'a indi; uçağın merkezden uzaklığı (medyan) 1.33 → 0.15 yarım
kırpma, hedef ivmesi (sarsıntı göstergesi) 76 → 24. Ortalama zoom 2.46x → 2.29x. Çıkışta uçağın
yüksek frekanslı titremesi (3 videoda ölçüldü, 1080p px) 346 → 115, 170 → 81, 191 → 136; arka planın
sarsıntısı değişmedi (+%1–8; önceki sürüm çoğu zaman kalabalığı kaydediyordu).

Eski açık lisanslı videolarda (referansı anlamlı olan 4 video: Bournemouth, F-18, Jim Parker, Wings;
Red Arrows'ta referans tüm kol uçuşunu tek kutu sayıyor, F-22'de uçak bulamadı): uçakta %69 → %79
(Bournemouth %28 → %57, Jim Parker %67 → %82), yanlış hedefte %39 → %15 (referansı olan 5 video). Kadraj biraz geriledi:
kadrajda %93 → %90, iyi kadraj %75 → %72 (en çok Wings: ilk kilit önce 9 px'lik bir blob'a oluyor,
sinir ağı iki kez görünce uçağa geçiyor).

SOLOTÜRK videolarında bulunan hatalar: yeniden tohumlamada görüntü içi arama çapasının
güncellenmemesi (büyük ve net uçakta her birkaç karede bir yeniden tohumlama döngüsü), blob'dan
başlayan takipte boyutun yalnızca uçağın koyu çekirdeği olması (doğru sinir ağı onayları boyut
uyumsuzluğundan reddediliyordu), aynı karede blob ve sinir ağı tespiti varken blob'un seçilmesi,
kadrajın uçağın merkezi yerine takip noktasına (ör. kokpit) ortalanması ve tahmin modunda hedefin
düz çizgide kare dışına kaçması.

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
- [ ] **Rolling shutter yönü**: hızlı yatay pan'da dikey çizgiler (direk, bina kenarı) çıkışta dik
      kalmalı. Eğiklik *arttıysa* okuma yönü ters (`RollingShutter.readoutDirection`); cihaz
      `SENSOR_ROLLING_SHUTTER_SKEW` bildirmiyorsa (0) düzeltme kapalı kalır.
- [ ] **Yakınlaştırılmış tespit**: durum satırındaki NN süresi 320x320 bölgede tam kareden farklı
      olmamalı; uzak uçakta takip kutusunun yeşil (onaylı) kalması.
- [ ] Cihazın 4K60 profili seçiliyor mu (durum satırı `3840x2160@60 → 1920x1080@60`)?
- [ ] Render ≥ 59 fps, analiz < 10 ms, NN süresi (durum satırında).
- [ ] Kayıt: ses/görüntü senkronu, 60 fps sabit kare hızı, dosya galeride görünüyor mu.

## Yol haritası

Öncelik sırasına göre öneriler:

1. **Jiroskop + görüntü füzyonu**: görsel tahminle jiroskobun zaman ofseti ve odak ölçeğini cihazda
   otomatik kalibre etmek; ikisini birleştirip kalan kaymayı gidermek.
2. **Takipçi ölçüm kalitesi**: yeniden tohumlama sıçramaları (şablon ↔ tespit kutusu merkezi) hâlâ
   kalan sarsıntının ana kaynağı; kutu merkezi yerine şablon kaydı ile hizalama.
3. **Yüksek çözünürlüklü şablon**: sinir ağı artık hedef çevresine yakınlaşıyor; şablon eşleme de
   hedef çevresinden tam çözünürlükte bir luma yaması üzerinde yapılırsa küçük hedeflerde alt-piksel
   hassasiyet ve daha az kayma (şu an 640x360 analiz görüntüsünde).
4. **Asenkron geri okuma (PBO)** ile render thread'deki `glReadPixels` beklemesini kaldırmak.
5. **Uçak odaklı model**: uzak/küçük uçak veri setiyle eğitilmiş dedektör, sonra helikopter, drone,
   kuş gibi sınıflar.
6. **Hibrit zoom**: otomatik zoom kırpma sınırına dayanınca lens zumunu kontrollü değiştirmek.
7. **ADS-B entegrasyonu**: yakındaki uçakların konumundan hedefin ekrandaki yerini tahmin edip
   ön-kilit, kaybolunca arama yönü.
8. Eşzamanlı ham 4K kayıt, 10-bit HEVC / log profil, tam manuel pozlama arayüzü.
