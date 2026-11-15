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
