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
  1x–4x ayarlanabilir; lens zumu (`CONTROL_ZOOM_RATIO`) ile telefoto kameraya geçilebilir.
- **Hibrit hedef tespiti**
  - Sinir ağı: MediaPipe EfficientDet-Lite0 (COCO `airplane` sınıfı), ~10 Hz, asenkron.
  - Gökyüzü dedektörü: düz gökyüzünde birkaç piksellik uzak uçakları bulan klasik yöntem
    (yerel arka plan farkı + sakin halka testi). Sinir ağının göremediği küçük hedefler için.
- **60 fps takip**: NCC şablon eşleme (alt-piksel), sabit hız Kalman filtresi, kaybolunca tahminle
  devam (coasting) ve tespitlerle yeniden yakalama.
- **Jiroskop tabanlı stabilizasyon**: Takip ve kadraj "dünya" koordinatlarında (kamera dönüşü
  çıkarılmış) yapılır; el titremesi gecikmesiz iptal edilir, sanal kamera ise kritik sönümlü yayla
  yumuşak hareket eder. Hedef yokken aynı sistem normal elektronik stabilizasyon (EIS) gibi çalışır.
- **Uçak fotoğrafçılığı ayarları**: sonsuza sabit odak (gökyüzünde AF aramasını önler), EV,
  hızlı enstantane (≈1/1000 s, ISO otomatik türetilir), OIS açık / kamera EIS kapalı.
- **Kayıt**: donanım H.264 (isteğe bağlı HEVC) 40 Mbps + AAC ses, `Movies/OpenTrack` klasörüne.
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

## Proje yapısı

| Modül | İçerik |
|---|---|
| `tracking/` | Saf Kotlin/JVM: Kalman, NCC takipçi, gökyüzü dedektörü, jiroskop hareket modeli, kadraj kontrolcüsü. Cihazsız test edilir. |
| `app/` | Android: Camera2, OpenGL ES render hattı, MediaPipe dedektörü, MediaCodec/MediaMuxer kayıt, arayüz. |

Ayrıntılı tasarım, koordinat sistemleri ve yol haritası: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Durum

Bu ilk sürüm derleniyor ve algoritma katmanı birim testlerinden geçiyor, ancak **henüz gerçek
cihazda denenmedi**. Cihazda doğrulanması gerekenler için
[docs/ARCHITECTURE.md → Cihazda doğrulama listesi](docs/ARCHITECTURE.md#cihazda-doğrulama-listesi).

## Lisans

MIT
