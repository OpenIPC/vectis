//
// Copyright (c) OpenIPC  https://openipc.org  MIT License
//

pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "vectis"
include(":app")
// camex built in (remote access through a camex server, #97): the tunnel library of
// the camex sources, a submodule pinned to a camex release
include(":camex-tunnel")
project(":camex-tunnel").projectDir = file("../bundles/camex/android/tunnel")
