# vk_meshlet_culling


# Build

[vcpkg](https://github.com/microsoft/vcpkg)を使っています。`vcpkg.json`に依存ライブラリを記述しているため、cmakeを走らせるとvcpkg installも走ります。

```sh
cmake . -B build -D CMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake

# build\VkMeshletCulling.sln を開く
```
