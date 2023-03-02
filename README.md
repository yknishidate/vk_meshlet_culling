# vk_meshlet_culling

VulkanのMesh shading拡張機能を使ってMeshletカリングをしてみるプロジェクト。

https://user-images.githubusercontent.com/30839669/222442191-299e824f-e64f-4018-9162-80f3e2904abc.mp4

# Build

[vcpkg](https://github.com/microsoft/vcpkg)を使っています。`vcpkg.json`に依存ライブラリを記述しているため、cmakeを走らせるとvcpkg installも走ります。

```sh
# cmakeを走らせる
cmake . -B build -D CMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake

# ソリューションを開く
build\VkMeshletCulling.sln
```
