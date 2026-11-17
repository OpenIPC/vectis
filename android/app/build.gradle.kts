//
// Copyright (c) OpenIPC  https://openipc.org  MIT License
//

plugins {
    id("com.android.application")
}

// The version lives in PROGRAM_VERSION in vectis.c only: versionName is that,
// versionCode MAJOR * 10000 + MINOR * 100 + PATCH. No version, no build.
val vectisVersion: String = run {
    val src = rootProject.file("../vectis.c")
    val m = Regex("""(?m)^#\s*define\s+PROGRAM_VERSION\s+"(\d+)\.(\d+)\.(\d+)"\s*$""")
        .find(src.readText())
        ?: throw GradleException("no PROGRAM_VERSION \"X.Y.Z\" in $src")
    m.groupValues.drop(1).joinToString(".")
}
val vectisVersionCode: Int = vectisVersion.split(".").map { it.toInt() }
    .let { (major, minor, patch) -> major * 10000 + minor * 100 + patch }

// Release key from the environment only: VECTIS_KEYSTORE_FILE (the keystore's
// path), VECTIS_KEYSTORE_PASSWORD, VECTIS_KEY_ALIAS and VECTIS_KEY_PASSWORD, set
// by CI from its secrets or exported by hand. Without VECTIS_KEYSTORE_FILE the
// release APK stays unsigned.
val releaseKeystore: String? =
    System.getenv("VECTIS_KEYSTORE_FILE")?.takeIf { it.isNotBlank() }

// 64-bit devices and 32-bit TV boxes; -PvectisAbis=armeabi-v7a for one ABI
val vectisAbis: List<String> = ((project.findProperty("vectisAbis") as String?)
    ?: "arm64-v8a,armeabi-v7a").split(",").map { it.trim() }.also { abis ->
    // an empty list would leave abiFilters unset and build-native.sh would build its own
    // default, so the APK would not match the list; an empty entry ("a,,b") is an error too
    // (#91; android/test-abis.sh)
    val known = setOf("arm64-v8a", "armeabi-v7a", "x86_64")   // those of build-native.sh
    require(abis.isNotEmpty() && abis.all { it in known }) {
        "-PvectisAbis=${abis.joinToString(",")}: one or more of ${known.joinToString(",")}"
    }
}

// vectis itself: the C sources of the repository root, built by the NDK as an
// executable named libvectis.so (android/build-native.sh), packaged as a native library
val nativeOut = layout.buildDirectory.dir("jniLibs")
val buildNative by tasks.registering(Exec::class) {
    inputs.files(rootProject.files("../vectis.c", "../usbserial.c", "../usbserial.h", "../ch34x.c",
                                  "../webterm_assets.h"))
    inputs.file(rootProject.file("build-native.sh"))
    inputs.property("abis", vectisAbis)   // another ABI list must rebuild
    outputs.dir(nativeOut)
    commandLine(listOf("sh", rootProject.file("build-native.sh").path,
                       nativeOut.get().asFile.path) + vectisAbis)
}
tasks.named("preBuild") { dependsOn(buildNative) }

android {
    namespace = "org.openipc.vectis"
    compileSdk = 34

    defaultConfig {
        applicationId = "org.openipc.vectis"
        minSdk = 21
        targetSdk = 34
        versionCode = vectisVersionCode
        versionName = vectisVersion
        ndk { abiFilters += vectisAbis }
        // The app's own strings are English only; keep the libraries' translations
        // just for the README languages (#93)
        resourceConfigurations += listOf("en", "ru", "zh-rCN")
    }
    signingConfigs {
        // A fixed debug key in the repository (not a secret, as every debug
        // key): debug APKs from any machine install over each other.
        getByName("debug") {
            storeFile = rootProject.file("debug.keystore")
            storePassword = "android"
            keyAlias = "androiddebugkey"
            keyPassword = "android"
        }
        create("release") {
            if (releaseKeystore != null) {
                storeFile = file(releaseKeystore)
                storePassword = System.getenv("VECTIS_KEYSTORE_PASSWORD")
                keyAlias = System.getenv("VECTIS_KEY_ALIAS")
                keyPassword = System.getenv("VECTIS_KEY_PASSWORD")
            }
        }
    }
    sourceSets {
        getByName("main") { jniLibs.srcDir(nativeOut) }
    }
    packaging {
        // The executable must be a real file in nativeLibraryDir to be run
        jniLibs { useLegacyPackaging = true }
    }
    buildTypes {
        release {
            if (releaseKeystore != null)
                signingConfig = signingConfigs.getByName("release")
            isMinifyEnabled = true
            isShrinkResources = true
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"))
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    testOptions {
        unitTests.isReturnDefaultValues = true
    }
    lint {
        warningsAsErrors = true
        abortOnError = true
        // GradleDependency etc.: the versions are pinned on purpose (as in compot/android);
        // ChromeOsAbiSupport: phones and TV boxes with ARM USB hosts
        disable += listOf("GradleDependency", "OldTargetApi", "AndroidGradlePluginVersion", "ChromeOsAbiSupport")
    }
}

dependencies {
    // Platform classes only: no AppCompat or AndroidX
    testImplementation("junit:junit:4.13.2")
}
