# vk_meshlet_culling

VulkanのMesh shading拡張機能を使ってMeshletカリングをしてみるプロジェクト。

https://user-images.githubusercontent.com/30839669/222445582-19eb0182-7f2f-44f6-a2c6-357720c7477b.mp4

# Build

[vcpkg](https://github.com/microsoft/vcpkg)を使っています。`vcpkg.json`に依存ライブラリを記述しているため、cmakeを走らせるとvcpkg installも走ります。

```sh
# cmakeを走らせる
# 注意：インストールしたパスに変える
cmake . -B build -D CMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake

# ソリューションを開く
build\VkMeshletCulling.sln
```
