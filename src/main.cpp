#include <meshoptimizer.h>
#include <numbers>
#include "Engine.hpp"

struct Constants {
    glm::mat4 model{1};
    glm::mat4 view{1};
    glm::mat4 proj{1};
    vk::DeviceAddress meshletBuffer;
    vk::DeviceAddress meshletVertexBuffer;
    vk::DeviceAddress meshletTriangleBuffer;
    vk::DeviceAddress meshletBoundBuffer;
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

        buildMeshlets();

        vertexBuffer = DeviceBuffer{BufferUsage::Vertex, vertices};
        indexBuffer = DeviceBuffer{BufferUsage::Index, indices};
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

        std::unordered_map<Vertex, uint32_t> uniqueVertices;
        meshSize = shapes.size();
        for (int shapeID = 0; shapeID < shapes.size(); shapeID++) {
            auto& shape = shapes[shapeID];
            spdlog::info("  Shape {}", shape.name);
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

    Transform transform;
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
        File::writeBinary(spvFile.string(), spvCode);
    } else {
        spdlog::info("Read shader: {}", spvFile.string());
        File::readBinary(spvFile.string(), spvCode);
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

        std::vector<std::string> shaderFileNames{"meshlet_cull.task", "meshlet_cull.mesh",
                                                 "meshlet_cull.frag"};
        std::vector<Shader> shaders;
        for (const auto& fileName : shaderFileNames) {
            shaders.push_back(compileOrReadShader(fileName));
        }

        DescriptorSet descSet;
        for (const auto& shader : shaders) {
            descSet.addResources(shader);
        }
        descSet.record("Vertices", scene.vertexBuffer);
        descSet.record("Indices", scene.indexBuffer);
        descSet.record("Uniforms", uniformBuffer);
        descSet.allocate();

        GraphicsPipeline pipeline;
        pipeline.setDescriptorSet(descSet);
        pipeline.setPushSize(sizeof(Constants));
        for (const auto& shader : shaders) {
            pipeline.addShader(shader);
        }
        pipeline.setup(renderPass);

        // Frustum pipeline
        FrustumConstants frustumConstants;
        Mesh frustumMesh = Mesh::createCubeLines();
        DescriptorSet frustumDescSet;
        frustumDescSet.allocate();
        Shader frustumVert = compileOrReadShader("frustum.vert");
        Shader frustumFrag = compileOrReadShader("frustum.frag");
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

            // GUI
            gui.startFrame();
            static int gbuffer = 0;
            gui.combo("G-buffer", gbuffer, {"Color", "Position", "Normal"});
            gui.text("CPU Time: %f", cpuTime);
            gui.text("GPU Time: %f", gpuTime);
            if (gui.button("Recompile shaders")) {
                Context::waitIdle();
                try {
                    for (size_t shaderIndex = 0; shaderIndex < shaders.size(); shaderIndex++) {
                        shaders[shaderIndex] = Shader{SHADER_DIR + shaderFileNames[shaderIndex]};
                    }
                    pipeline = GraphicsPipeline{};
                    pipeline.setDescriptorSet(descSet);
                    pipeline.setPushSize(sizeof(Constants));
                    for (size_t shaderIndex = 0; shaderIndex < shaders.size(); shaderIndex++) {
                        pipeline.addShader(shaders[shaderIndex]);
                    }
                    pipeline.setup(renderPass);
                } catch (const std::exception& e) {
                    Log::error(e.what());
                }
            }

            // Camera & Frustum
            camera.processInput();
            cullingCamera.setYaw(glm::sin(frame * 0.025) * 10.0f);
            cullingCamera.setPitch(glm::cos(frame * 0.025) * 5.0f);
            Frustum frustum{cullingCamera};

            // Update uniform buffer
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

            swapchain.waitNextFrame();
            CommandBuffer commandBuffer = swapchain.beginCommandBuffer();
            {
                // Mesh shading pass
                commandBuffer.beginTimestamp(gpuTimer);
                commandBuffer.bindPipeline(pipeline);
                commandBuffer.clearColorImage(colorImages[0], {0.0f, 0.0f, 0.3f, 1.0f});
                commandBuffer.clearColorImage(colorImages[1], {0.0f, 0.0f, 0.0f, 1.0f});
                commandBuffer.clearColorImage(colorImages[2], {0.0f, 0.0f, 0.0f, 1.0f});
                commandBuffer.beginRenderPass(renderPass);
                commandBuffer.pushConstants(pipeline, &constants);
                commandBuffer.drawMeshTasks(divRoundUp(scene.meshletCount, 32), 1, 1);
                commandBuffer.endRenderPass(renderPass);
                commandBuffer.copyToBackImage(colorImages[gbuffer]);
                commandBuffer.endTimestamp(gpuTimer);
            }
            {
                // GUI pass
                commandBuffer.bindPipeline(frustumPipeline);
                commandBuffer.beginDefaultRenderPass();
                commandBuffer.pushConstants(frustumPipeline, &frustumConstants);
                commandBuffer.drawIndexed(frustumMesh);
                commandBuffer.drawGUI(gui);
                commandBuffer.endDefaultRenderPass();
            }
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
