#include <meshoptimizer.h>
#include <numbers>
#include "Engine.hpp"

struct Constants {
    glm::mat4 model{1};
    glm::mat4 view{1};
    glm::mat4 proj{1};
    vk::DeviceAddress materialBuffer;
    vk::DeviceAddress materialIDBuffer;
    vk::DeviceAddress meshletBuffer;
    vk::DeviceAddress meshletVertexBuffer;
    vk::DeviceAddress meshletTriangleBuffer;
    vk::DeviceAddress meshletBoundBuffer;
    int primitiveOffset;
    int meshletCount;
};

struct Uniforms {
    glm::mat4 prevView{1};
    glm::mat4 prevProj{1};
    glm::vec4 topFace;
    glm::vec4 bottomFace;
    glm::vec4 rightFace;
    glm::vec4 leftFace;
    glm::vec4 farFace;
    glm::vec4 nearFace;
};

struct FrustumConstants {
    glm::mat4 view{1};
    glm::mat4 proj{1};
    glm::mat4 invView{1};
    glm::mat4 invProj{1};
};

template <typename T>
void writeBinary(const std::string& filepath, const std::vector<T>& vec) {
    std::ofstream ofs(filepath, std::ios::out | std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(vec.data()), vec.size() * sizeof(T));
    ofs.close();
}

template <typename T>
void readBinary(const std::string& filepath, std::vector<T>& vec) {
    std::uintmax_t size = std::filesystem::file_size(filepath);
    vec.resize(size / sizeof(T));
    std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
    ifs.read(reinterpret_cast<char*>(vec.data()), vec.size() * sizeof(T));
    ifs.close();
}

class Scene {
public:
    Scene() = default;
    Scene(const std::string& filepath,
          glm::vec3 position = glm::vec3{0.0f},
          glm::vec3 scale = glm::vec3{1.0f},
          glm::vec3 rotation = glm::vec3{0.0f}) {
        transform.position = position;
        transform.scale = scale;
        transform.rotation = rotation;

        CPUTimer cpuTimer;
        loadFromObj(filepath);
        Log::info("Loaded: {} ms", cpuTimer.elapsedInMilli());
        Log::info("  Vertices: {}", vertices.size());
        Log::info("  Indices: {}", indices.size());
        Log::info("  Materials: {}", materials.size());
        Log::info("  Textures: {}", textures.size());

        buildMeshlets();

        vertexBuffer = DeviceBuffer{BufferUsage::Vertex, vertices};
        indexBuffer = DeviceBuffer{BufferUsage::Index, indices};

        materialBuffer = DeviceBuffer{BufferUsage::Storage, materials};
        materialIDBuffer = DeviceBuffer{BufferUsage::Storage, materialIDs};

        // Build aabbs
        aabbs.resize(meshSize);
        for (int meshID = 0; meshID < meshSize; meshID++) {
            glm::vec3 minPoint{std::numeric_limits<float>::max()};
            glm::vec3 maxPoint{-std::numeric_limits<float>::max()};
            int indexIndex = indexOffsets[meshID];
            for (int i = 0; i < indexCounts[meshID]; i++) {
                uint32_t index = indices[indexIndex];
                Vertex vertex = vertices[index];
                minPoint = glm::min(minPoint, transform.scale * vertex.pos);
                maxPoint = glm::max(maxPoint, transform.scale * vertex.pos);
                indexIndex++;
            }
            aabbs[meshID] = {minPoint, maxPoint};
        }
    }

    void loadFromObj(const std::string& filepath) {
        spdlog::info("Load file: {}", filepath);
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> objMaterials;
        std::string warn, err;

        std::string dir = std::filesystem::path{filepath}.parent_path().string();
        if (!tinyobj::LoadObj(&attrib, &shapes, &objMaterials, &warn, &err, filepath.c_str(),
                              dir.c_str())) {
            spdlog::error("Failed to load: {}", warn + err);
        }
        spdlog::info("Shapes: {}", shapes.size());
        spdlog::info("Materials: {}", objMaterials.size());

        int texCount = 0;
        std::unordered_map<std::string, int> textureNames{};

        materials.resize(objMaterials.size());
        for (size_t i = 0; i < objMaterials.size(); i++) {
            spdlog::info("material: {}", objMaterials[i].name);
            auto& mat = objMaterials[i];
            materials[i].ambient = {mat.ambient[0], mat.ambient[1], mat.ambient[2]};
            materials[i].diffuse = {mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]};
            materials[i].specular = {mat.specular[0], mat.specular[1], mat.specular[2]};
            materials[i].emission = {mat.emission[0], mat.emission[1], mat.emission[2]};

            // diffuse
            if (!mat.diffuse_texname.empty()) {
                if (textureNames.contains(mat.diffuse_texname)) {
                    materials[i].diffuseTexture = textureNames[mat.diffuse_texname];
                } else {
                    materials[i].diffuseTexture = texCount;
                    textureNames[mat.diffuse_texname] = texCount;
                    texCount++;
                }
            }
            // specular
            if (!mat.specular_texname.empty()) {
                if (textureNames.contains(mat.specular_texname)) {
                    materials[i].specularTexture = textureNames[mat.specular_texname];
                } else {
                    materials[i].specularTexture = texCount;
                    textureNames[mat.specular_texname] = texCount;
                    texCount++;
                }
            }
            // alpha
            if (!mat.alpha_texname.empty()) {
                if (textureNames.contains(mat.alpha_texname)) {
                    materials[i].alphaTexture = textureNames[mat.alpha_texname];
                } else {
                    materials[i].alphaTexture = texCount;
                    textureNames[mat.alpha_texname] = texCount;
                    texCount++;
                }
            }
            // emission
            if (!mat.emissive_texname.empty()) {
                if (textureNames.contains(mat.emissive_texname)) {
                    materials[i].emissionTexture = textureNames[mat.emissive_texname];
                } else {
                    materials[i].emissionTexture = texCount;
                    textureNames[mat.emissive_texname] = texCount;
                    texCount++;
                }
            }
        }

        textures.resize(texCount);
        for (auto& [name, index] : textureNames) {
            std::string path = name;
            std::ranges::replace(path, '\\', '/');
            path = dir + "/" + path;
            spdlog::info("  Texture {}: {}", index, path);
            textures[index] = Image{path};
        }

        std::unordered_map<Vertex, uint32_t> uniqueVertices;
        indexOffsets.resize(shapes.size());
        indexCounts.resize(shapes.size());
        meshSize = shapes.size();
        for (int shapeID = 0; shapeID < shapes.size(); shapeID++) {
            auto& shape = shapes[shapeID];
            spdlog::info("  Shape {}", shape.name);

            for (auto& id : shape.mesh.material_ids) {
                materialIDs.push_back(id);
            }
            indexOffsets[shapeID] = indices.size();

            for (const auto& index : shape.mesh.indices) {
                Vertex vertex;
                vertex.pos = {attrib.vertices[3 * index.vertex_index + 0],
                              -attrib.vertices[3 * index.vertex_index + 1],
                              attrib.vertices[3 * index.vertex_index + 2]};
                if (index.normal_index != -1) {
                    vertex.normal = {attrib.normals[3 * index.normal_index + 0],
                                     -attrib.normals[3 * index.normal_index + 1],
                                     attrib.normals[3 * index.normal_index + 2]};
                }
                if (index.texcoord_index != -1) {
                    vertex.texCoord = {attrib.texcoords[2 * index.texcoord_index + 0],
                                       1.0f - attrib.texcoords[2 * index.texcoord_index + 1]};
                }
                if (!uniqueVertices.contains(vertex)) {
                    vertices.push_back(vertex);
                    uniqueVertices[vertex] = uniqueVertices.size();
                }
                indices.push_back(uniqueVertices[vertex]);
            }
            indexCounts[shapeID] = indices.size() - indexOffsets[shapeID];
        }
    }

    void buildMeshlets() {
        constexpr size_t maxVertices = 64;
        constexpr size_t maxTriangles = 124;
        constexpr float coneWeight = 0.0f;

        maxMeshlets = meshopt_buildMeshletsBound(indices.size(), maxVertices, maxTriangles);
        meshlets.resize(maxMeshlets);
        meshletVertices.resize(maxMeshlets * maxVertices);
        meshletTriangles.resize(maxMeshlets * maxTriangles * 3);

        meshletCount = meshopt_buildMeshlets(meshlets.data(), meshletVertices.data(),
                                             meshletTriangles.data(), indices.data(),
                                             indices.size(), &vertices[0].pos.x, vertices.size(),
                                             sizeof(Vertex), maxVertices, maxTriangles, coneWeight);
        bounds.resize(meshletCount);
        for (size_t i = 0; i < meshletCount; i++) {
            auto& m = meshlets[i];
            bounds[i] = meshopt_computeMeshletBounds(
                &meshletVertices[m.vertex_offset], &meshletTriangles[m.triangle_offset],
                m.triangle_count, &vertices[0].pos.x, vertices.size(), sizeof(Vertex));
        }

        meshletVertexBuffer = DeviceBuffer{BufferUsage::Storage, meshletVertices};
        meshletTriangleBuffer = DeviceBuffer{BufferUsage::Storage, meshletTriangles};
        meshletBuffer = DeviceBuffer{BufferUsage::Storage, meshlets};
        meshletBoundBuffer = DeviceBuffer{BufferUsage::Storage, bounds};
    }

    size_t maxMeshlets;
    std::vector<meshopt_Bounds> bounds;
    std::vector<meshopt_Meshlet> meshlets;
    std::vector<unsigned int> meshletVertices;
    std::vector<unsigned char> meshletTriangles;
    size_t meshletCount;
    DeviceBuffer meshletBuffer;
    DeviceBuffer meshletVertexBuffer;
    DeviceBuffer meshletTriangleBuffer;
    DeviceBuffer meshletBoundBuffer;

    int meshSize = 0;

    DeviceBuffer vertexBuffer;
    std::vector<Vertex> vertices;

    DeviceBuffer indexBuffer;
    std::vector<uint32_t> indices;

    std::vector<int> indexOffsets;
    std::vector<int> indexCounts;

    std::vector<int> materialIDs;  // per-face
    DeviceBuffer materialIDBuffer;

    std::vector<Image> textures;
    std::vector<uint8_t> allTextureData;
    std::vector<vk::Extent2D> textureSizes;

    std::vector<Material> materials;
    DeviceBuffer materialBuffer;

    TopAccel topAccel;
    Transform transform;

    std::vector<AABB> aabbs;
};

uint32_t divRoundUp(uint32_t num, uint32_t den) {
    return (num + den - 1) / den;
}

Shader compileOrReadShader(const std::string& shaderFileName) {
    namespace fs = std::filesystem;
    auto glslFile = fs::path{SHADER_DIR + shaderFileName};
    auto spvFile = fs::path{SHADER_DIR + shaderFileName + ".spv"};
    std::vector<uint32_t> spvCode;
    if (!fs::exists(spvFile) || fs::last_write_time(glslFile) > fs::last_write_time(spvFile)) {
        spvCode = Compiler::compileToSPV(glslFile.string());
        writeBinary(spvFile.string(), spvCode);
    } else {
        spdlog::info("Read shader: {}", spvFile.string());
        readBinary(spvFile.string(), spvCode);
    }
    return {spvCode, Compiler::getShaderStage(shaderFileName)};
}

int main() {
    try {
        Log::init();
        uint32_t width = 1920;
        uint32_t height = 1080;
        Window::init(width, height);
        Context::init(true);

        Swapchain swapchain{};
        GUI gui{swapchain};
        Scene scene{"../asset/bistro/Exterior/exterior.obj", {}, glm::vec3{0.01f}};

        // Create RenderPass
        std::vector<Image> colorImages(3);
        colorImages[0] = {width, height, vk::Format::eB8G8R8A8Unorm, ImageUsage::ColorAttachment};
        colorImages[1] = {width, height, vk::Format::eB8G8R8A8Unorm, ImageUsage::ColorAttachment};
        colorImages[2] = {width, height, vk::Format::eB8G8R8A8Unorm, ImageUsage::ColorAttachment};
        Image depthImage{width, height, vk::Format::eD32Sfloat, ImageUsage::DepthStencilAttachment};
        RenderPass renderPass{width, height, colorImages, depthImage};

        Camera camera{width, height};
        Camera cullingCamera{width, height};
        camera.setPosition(-7.85f, -3.45f, -0.05f);
        cullingCamera.setPosition(-7.85f, -3.45f, -0.05f);
        cullingCamera.setFovY(glm::radians(20.0f));

        // Create uniform buffer
        Uniforms uniforms;
        HostBuffer uniformBuffer{BufferUsage::Uniform, sizeof(Uniforms)};
        uniformBuffer.copy(&uniforms);

        constexpr int numPipelines = 2;
        std::vector<std::vector<std::string>> shaderFileNames(numPipelines);
        shaderFileNames[0] = {"default.vert", "default.frag"};
        shaderFileNames[1] = {"meshlet_cull.task", "meshlet_cull.mesh", "meshlet_cull.frag"};
        std::vector<std::vector<Shader>> shaders(numPipelines);
        for (size_t pipeline = 0; pipeline < numPipelines; pipeline++) {
            for (const auto& fileName : shaderFileNames[pipeline]) {
                shaders[pipeline].push_back(compileOrReadShader(fileName));
            }
        }

        DescriptorSet descSet;
        for (size_t pipeline = 0; pipeline < numPipelines; pipeline++) {
            for (const auto& shader : shaders[pipeline]) {
                descSet.addResources(shader);
            }
        }
        descSet.record("textures", scene.textures);
        descSet.record("Vertices", scene.vertexBuffer);
        descSet.record("Indices", scene.indexBuffer);
        descSet.record("Uniforms", uniformBuffer);
        descSet.allocate();

        std::vector<GraphicsPipeline> pipelines(numPipelines);
        for (size_t pipeline = 0; pipeline < numPipelines; pipeline++) {
            pipelines[pipeline].setDescriptorSet(descSet);
            pipelines[pipeline].setPushSize(sizeof(Constants));
            for (const auto& shader : shaders[pipeline]) {
                pipelines[pipeline].addShader(shader);
            }
            pipelines[pipeline].setup(renderPass);
        }

        // Frustum pipeline
        FrustumConstants frustumConstants;
        Mesh frustumMesh = Mesh::createFrustumLines();
        DescriptorSet frustumDescSet;
        frustumDescSet.allocate();
        Shader frustumVert = compileOrReadShader("frustum.vert");
        Shader frustumFrag = compileOrReadShader("white.frag");
        GraphicsPipeline frustumPipeline{};
        frustumPipeline.addShader(frustumVert);
        frustumPipeline.addShader(frustumFrag);
        frustumPipeline.setTopology(vk::PrimitiveTopology::eLineList);
        frustumPipeline.setPolygonMode(vk::PolygonMode::eLine);
        frustumPipeline.setPushSize(sizeof(FrustumConstants));
        frustumPipeline.setDescriptorSet(frustumDescSet);
        frustumPipeline.setup(swapchain.getRenderPass());

        int frame = 0;
        Constants constants;
        constants.materialBuffer = scene.materialBuffer.getAddress();
        constants.materialIDBuffer = scene.materialIDBuffer.getAddress();
        constants.meshletBuffer = scene.meshletBuffer.getAddress();
        constants.meshletVertexBuffer = scene.meshletVertexBuffer.getAddress();
        constants.meshletTriangleBuffer = scene.meshletTriangleBuffer.getAddress();
        constants.meshletBoundBuffer = scene.meshletBoundBuffer.getAddress();
        constants.meshletCount = scene.meshletCount;

        GPUTimer gpuTimer;
        float cpuTime = 0.0f;
        float gpuTime = 0.0f;
        while (!Window::shouldClose()) {
            Window::pollEvents();
            CPUTimer cpuTimer;
            gui.startFrame();

            static int pipeline = 1;
            static int gbuffer = 0;
            static bool frustumCulling = true;
            static bool meshSort = true;
            gui.combo("Pipeline", pipeline, {"Graphics", "MeshletCull"});
            gui.combo("G-buffer", gbuffer, {"Color", "Position", "Normal"});
            gui.checkbox("Frustum culling", frustumCulling);
            gui.checkbox("Mesh sort", meshSort);
            gui.text("CPU Time: %f", cpuTime);
            gui.text("GPU Time: %f", gpuTime);
            if (gui.button("Recompile shaders")) {
                Context::waitIdle();
                try {
                    for (size_t shaderIndex = 0; shaderIndex < shaders[pipeline].size();
                         shaderIndex++) {
                        shaders[pipeline][shaderIndex] =
                            Shader{SHADER_DIR + shaderFileNames[pipeline][shaderIndex]};
                    }
                    pipelines[pipeline] = GraphicsPipeline{};
                    pipelines[pipeline].setDescriptorSet(descSet);
                    pipelines[pipeline].setPushSize(sizeof(Constants));
                    for (size_t shaderIndex = 0; shaderIndex < shaders[pipeline].size();
                         shaderIndex++) {
                        pipelines[pipeline].addShader(shaders[pipeline][shaderIndex]);
                    }
                    pipelines[pipeline].setup(renderPass);
                } catch (const std::exception& e) {
                    Log::error(e.what());
                }
            }

            camera.processInput();

            // Update uniform buffer
            cullingCamera.setYaw(glm::sin(frame * 0.025) * 10.0f);
            cullingCamera.setPitch(glm::cos(frame * 0.025) * 5.0f);
            Frustum frustum{cullingCamera};
            uniforms.topFace.xyz = frustum.topFace.normal;
            uniforms.topFace.w = frustum.topFace.distance;
            uniforms.bottomFace.xyz = frustum.bottomFace.normal;
            uniforms.bottomFace.w = frustum.bottomFace.distance;
            uniforms.rightFace.xyz = frustum.rightFace.normal;
            uniforms.rightFace.w = frustum.rightFace.distance;
            uniforms.leftFace.xyz = frustum.leftFace.normal;
            uniforms.leftFace.w = frustum.leftFace.distance;
            uniforms.farFace.xyz = frustum.farFace.normal;
            uniforms.farFace.w = frustum.farFace.distance;
            uniforms.nearFace.xyz = frustum.nearFace.normal;
            uniforms.nearFace.w = frustum.nearFace.distance;

            uniforms.prevView = constants.view;
            uniforms.prevProj = constants.proj;
            uniformBuffer.copy(&uniforms);

            // Update push constants
            constants.model = scene.transform.getMatrix();
            constants.proj = camera.getProj();
            constants.view = camera.getView();

            Camera frustumCamera = cullingCamera;
            frustumCamera.setNear(1.0f);
            frustumCamera.setFar(5.0f);
            frustumConstants.proj = camera.getProj();
            frustumConstants.view = camera.getView();
            frustumConstants.invView = frustumCamera.getInvView();
            frustumConstants.invProj = frustumCamera.getInvProj();

            // Frustum culling
            std::vector<int> drawMeshIDs;
            drawMeshIDs.reserve(scene.meshSize);
            for (int meshID = 0; meshID < scene.meshSize; meshID++) {
                if (frustumCulling && !scene.aabbs[meshID].isOnFrustum(frustum)) {
                    continue;
                }
                drawMeshIDs.push_back(meshID);
            }

            // Sort by distance from the camera
            if (meshSort) {
                std::sort(
                    drawMeshIDs.begin(), drawMeshIDs.end(),
                    [&scene, &camera](int meshID0, int meshID1) {
                        return glm::distance(scene.aabbs[meshID0].center, camera.getPosition()) <
                               glm::distance(scene.aabbs[meshID1].center, camera.getPosition());
                    });
            }

            swapchain.waitNextFrame();
            CommandBuffer commandBuffer = swapchain.beginCommandBuffer();
            commandBuffer.beginTimestamp(gpuTimer);
            commandBuffer.bindPipeline(pipelines[pipeline]);
            commandBuffer.clearColorImage(colorImages[0], {0.0f, 0.0f, 0.3f, 1.0f});
            commandBuffer.clearColorImage(colorImages[1], {0.0f, 0.0f, 0.0f, 1.0f});
            commandBuffer.clearColorImage(colorImages[2], {0.0f, 0.0f, 0.0f, 1.0f});
            commandBuffer.beginRenderPass(renderPass);

            // Rendering
            if (pipeline == 0) {
                for (int meshID : drawMeshIDs) {
                    constants.primitiveOffset = scene.indexOffsets[meshID] / 3;
                    commandBuffer.pushConstants(pipelines[pipeline], &constants);
                    commandBuffer.drawIndexed(scene.vertexBuffer, scene.indexBuffer,
                                              scene.indexCounts[meshID],
                                              scene.indexOffsets[meshID]);
                }
            } else {
                commandBuffer.pushConstants(pipelines[pipeline], &constants);
                commandBuffer.drawMeshTasks(divRoundUp(scene.meshletCount, 32), 1, 1);
            }

            commandBuffer.endRenderPass(renderPass);
            commandBuffer.copyToBackImage(colorImages[gbuffer]);
            commandBuffer.endTimestamp(gpuTimer);

            commandBuffer.bindPipeline(frustumPipeline);
            commandBuffer.beginDefaultRenderPass();
            commandBuffer.pushConstants(frustumPipeline, &frustumConstants);
            commandBuffer.drawIndexed(frustumMesh);
            commandBuffer.drawGUI(gui);
            commandBuffer.endDefaultRenderPass();
            commandBuffer.submit();

            cpuTime = cpuTimer.elapsedInMilli();
            gpuTime = gpuTimer.elapsedInMilli();
            swapchain.present();
            frame++;
        }
        Context::waitIdle();
        Window::shutdown();
    } catch (const std::exception& e) {
        Log::error(e.what());
    }
}
