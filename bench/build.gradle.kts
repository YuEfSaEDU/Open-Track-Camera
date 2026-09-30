plugins {
    alias(libs.plugins.kotlin.jvm)
    application
}

// Offline harness: runs the exact :tracking code (tracker + framing + auto zoom) on frames
// decoded from real videos. Driven by tools/video_bench/run_bench.py.
java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
    }
}

application {
    mainClass.set("com.opentrack.bench.MainKt")
}

dependencies {
    implementation(project(":tracking"))
}
