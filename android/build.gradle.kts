//
// Copyright (c) OpenIPC  https://openipc.org  MIT License
//
// Top-level build file of the vectis Android app
plugins {
    id("com.android.application") version "8.2.0" apply false
    id("com.android.library") version "8.2.0" apply false   // :camex-tunnel
}

// The ABIs of the app (-PvectisAbis, app/build.gradle.kts checks them) for the
// native code of :camex-tunnel too
extra["camexTunnelAbis"] = ((findProperty("vectisAbis") as String?) ?: "arm64-v8a,armeabi-v7a")
