// See: https://www.w3.org/TR/WGSL/
// All uniform structs MUST match their C++ counterparts

struct SceneUniforms {
    worldToCamera: mat4x4f,
    cameraToClip: mat4x4f,

    lightInWorld: vec4f,
    cameraInWorld: vec4f,
    lightColor: vec4f,
}
@group(1) @binding(0) var<uniform> su: SceneUniforms;

struct MeshUniforms {
    modelToWorld: mat4x4f,
    modelToWorldForNormals: mat4x4f,
}
@group(0) @binding(0) var<uniform> mu: MeshUniforms;
@group(0) @binding(1) var texture: texture_2d<f32>;
@group(0) @binding(2) var textureSampler: sampler;
@group(0) @binding(3) var normalTexture: texture_2d<f32>;

struct InVertex {
    @location(0) positionInModel: vec3f,
    @location(1) normalInModel: vec3f,
    @location(2) tangentInModel: vec3f,
    @location(3) bitangentInModel: vec3f,
    @location(4) uv: vec2f,
}
struct OutVertex {
   @builtin(position) positionInClip: vec4f,
   @location(0) positionInWorld: vec4f,
   @location(1) normalInWorld: vec4f,
   @location(2) tangentInWorld: vec4f,
   @location(3) bitangentInWorld: vec4f,
   @location(4) uv: vec2f,
}
@vertex fn vs_main(in: InVertex) -> OutVertex {
    let positionInWorld = mu.modelToWorld * vec4(in.positionInModel, 1);
    var out : OutVertex;
    out.positionInClip = su.cameraToClip * su.worldToCamera * positionInWorld;
    out.positionInWorld = positionInWorld;
    out.normalInWorld = mu.modelToWorldForNormals * vec4(in.normalInModel, 0);
    // TODO: should I just use the regular modelToWorld matrix here?
    out.tangentInWorld = mu.modelToWorldForNormals * vec4(in.tangentInModel, 0);
    out.bitangentInWorld = mu.modelToWorldForNormals * vec4(in.bitangentInModel, 0);
    out.uv = in.uv;
    return out;
}

struct OutFragment {
    @location(0) color: vec4f,
    @location(1) normalInCamera: vec4f,
}
@fragment fn fs_main(in: OutVertex) -> OutFragment {
    let baseColor = textureSample(texture, textureSampler, in.uv);

    let normalMapStrength = 1.0;
    // each component is in the range [0,1]
    let encodedNormal = textureSample(normalTexture, textureSampler, in.uv).rgb;
    // shift to get negative values, then normalize because we only care about direction
    // this normal is in a frame local to the surface of this face
    let normalInLocal = normalize(encodedNormal - 0.5);

    let localToWorld = mat3x3f(
        normalize(in.tangentInWorld).xyz,
        normalize(in.bitangentInWorld).xyz,
        normalize(in.normalInWorld).xyz,
    );

    let normalInWorld = normalize(localToWorld * normalInLocal);
    let mixedNormal = normalize(mix(in.normalInWorld, vec4(normalInWorld, 0), normalMapStrength));

    var out : OutFragment;
    out.normalInCamera = (su.worldToCamera * mixedNormal + vec4(1, 1, 1, 0)) / 2;
    out.normalInCamera.a = 1;
    // Ambient
    let ambientStrength = 0.3;
    let ambient = ambientStrength * su.lightColor;
    // Diffuse
    let lightDirInWorld = normalize(su.lightInWorld - in.positionInWorld);
    let diff = max(dot(mixedNormal, lightDirInWorld), 0.0);
    let diffuse = diff * su.lightColor;
    // Specular
    let specularStrength = 0.1;
    let viewDirInWolrd = normalize(su.cameraInWorld - in.positionInWorld);
    let halfwayDir = normalize(lightDirInWorld + viewDirInWolrd);
    let spec = pow(max(dot(mixedNormal, halfwayDir), 0.0), 32);
    let specular = specularStrength * spec * su.lightColor;
    // Combination
    out.color = vec4(((ambient + diffuse) * baseColor + specular).rgb, 1);
    return out;
}

struct ComputeUniforms {
    clipToCamera: mat4x4f,
    resolution: vec2u,
}
// An array of points is eventually copied to CPU memory
// As such it must follow the layout defined in "point.hpp"
struct Point {
    xyz: vec3f,
    rgb: f32, // 3 rgb bytes packed into a 32-bit float
    normalXyz: vec3f,
    curvature: f32,
}
@group(0) @binding(0) var<uniform> cu: ComputeUniforms;
@group(0) @binding(1) var colorImage: texture_2d<f32>;
@group(0) @binding(2) var normalsImage: texture_2d<f32>;
@group(0) @binding(3) var depthImage: texture_depth_2d;
@group(0) @binding(4) var<storage, read_write> points: array<Point>;

// Convert a BGRA color with float values in [0, 1] to a byte vector in [0, 255]
// Then pack the bytes into a 32-bit float in RGBA order
// This is what ROS expects for point cloud colors
fn pack(color: vec4f) -> f32 {
    let c = vec4u(color * 255);
    return bitcast<f32>(c.b | (c.g << 8) | (c.r << 16));
}

fn reproject(pixelInImage: vec2u, depth: f32) -> vec3f {
    // See: https://www.w3.org/TR/webgpu/#coordinate-systems
    // These coordinate are in NDC (normalized device coordinates)
    let pointInClip = vec4f(
        // Map pixel coordinates to [-1, 1]
        2 * (f32(pixelInImage.x) / f32(cu.resolution.x)) - 1,
        -2 * (f32(pixelInImage.y) / f32(cu.resolution.y)) + 1,
        // Depth is already in [0, 1]
        depth,
        // Homogenous points have w=1
        1
    );
    let pointInCamera = cu.clipToCamera * pointInClip;
    // Dehomogenize
    return pointInCamera.xyz / pointInCamera.w;
}

// Workgroup size ensures one invocation per pixel
@compute @workgroup_size(1, 1, 1) fn cs_main(@builtin(global_invocation_id) id: vec3u) {
    let pixelInImage = id.xy; // 2D pixel coordinate in the camera image

    let depth = textureLoad(depthImage, pixelInImage, 0);
    let color = textureLoad(colorImage, pixelInImage, 0);
    let normal = textureLoad(normalsImage, pixelInImage, 0) * 2 - vec4(1, 1, 1, 0);

    let pointInCamera = reproject(pixelInImage, depth);

    // "points" is really a 2D array, but WGSL does not support 2D arrays with dynamic sizing
    // This "flattens" (maps) a 2D index to a 1D index
    let flatIndex = pixelInImage.y * cu.resolution.x + pixelInImage.x;
    points[flatIndex].xyz = pointInCamera;
    points[flatIndex].rgb = pack(color);
    points[flatIndex].normalXyz = normal.xyz;
    points[flatIndex].curvature = 0;
}
