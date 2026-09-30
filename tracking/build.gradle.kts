plugins {
    alias(libs.plugins.kotlin.jvm)
}

// Pure Kotlin/JVM module: all tracking, filtering and framing math lives here so it
// can be unit tested on the host without a device.
java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
    }
}

dependencies {
    testImplementation(libs.junit)
}
