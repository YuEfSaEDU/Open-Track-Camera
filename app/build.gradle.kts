import java.net.URI

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
}

android {
    namespace = "com.opentrack.camera"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.opentrack.camera"
        // Camera2 CONTROL_ZOOM_RATIO (API 30) is used to reach telephoto lenses.
        minSdk = 30
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    androidResources {
        noCompress += "tflite"
    }
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
    }
}

dependencies {
    implementation(project(":tracking"))
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.appcompat)
    implementation(libs.mediapipe.tasks.vision)
}

// The COCO EfficientDet-Lite model (contains the "airplane" class) is fetched at build
// time instead of being committed to git.
val modelDir = layout.projectDirectory.dir("src/main/assets/models")
val downloadModels by tasks.registering {
    val models = mapOf(
        "efficientdet_lite0.tflite" to
            "https://storage.googleapis.com/mediapipe-models/object_detector/efficientdet_lite0/int8/latest/efficientdet_lite0.tflite",
    )
    outputs.dir(modelDir)
    doLast {
        models.forEach { (name, url) ->
            val target = modelDir.file(name).asFile
            if (!target.exists()) {
                target.parentFile.mkdirs()
                logger.lifecycle("Downloading $name")
                URI(url).toURL().openStream().use { input -> target.outputStream().use { input.copyTo(it) } }
            }
        }
    }
}
tasks.named("preBuild") { dependsOn(downloadModels) }
