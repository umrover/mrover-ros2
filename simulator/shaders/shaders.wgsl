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
    roughness: f32,
    metallic: f32,
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
// PBR FS
fn fresnelSchlick(cosTheta: f32, F0: vec3f) -> vec3f {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

fn DistributionGGX(normal: vec3f, halfway: vec3f, roughness: f32) -> f32 {
    let a = roughness * roughness;
    let a2 = a * a;
    let NdotH = max(dot(normal, halfway), 0.0);
    let NdotH2 = NdotH * NdotH;

    let num = a2;
    let x = (NdotH2 * (a2 - 1.0) + 1.0);
    let denom = 3.14159 * x * x;

    return num / denom;
}

fn GeometrySchlickGGX(NdotV: f32, roughness: f32) -> f32 {
    let r = roughness + 1.0;
    let k = r * r / 8.0;

    let num = NdotV;
    let denom = NdotV * (1.0 - k) + k;

    return num / denom;
}

fn GeometrySmith(normal: vec3f, viewDir: vec3f, lightDir: vec3f, roughness: f32) -> f32 {
    let NdotV = max(dot(normal, viewDir), 0.0);
    let NdotL = max(dot(normal, lightDir), 0.0);
    let ggx1 = GeometrySchlickGGX(NdotL, roughness);
    let ggx2 = GeometrySchlickGGX(NdotV, roughness);

    return ggx1 * ggx2;
}

@fragment fn fs_main(in: OutVertex) -> OutFragment {
    // let metallic = 0.0;
    let metallic = mu.metallic;
    // let roughness = 0.93;
    let roughness = mu.roughness;
    let ao = 0.1;

    // get the normal vector
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
    let mixedNormal = normalize(mix(in.normalInWorld.xyz, normalInWorld, normalMapStrength));

    let viewDirInWorld = normalize((su.cameraInWorld - in.positionInWorld).xyz);
    let lightDirInWorld = normalize((su.lightInWorld - in.positionInWorld).xyz);
    let halfwayDir = normalize((lightDirInWorld + viewDirInWorld).xyz);

    // now PBR stuff
    let distanceToLight = length(su.lightInWorld - in.positionInWorld);
    // let attenuation = 1 / (distanceToLight * distanceToLight);
    let attenuation = 1.0;
    let radiance = su.lightColor.rgb * attenuation * 5;

    let albedo = textureSample(texture, textureSampler, in.uv).rgb;
    let F0 = mix(vec3(0.04), albedo, metallic);
    let F = fresnelSchlick(max(dot(halfwayDir, viewDirInWorld), 0), F0);

    let NDF = DistributionGGX(mixedNormal, halfwayDir, roughness);
    let G = GeometrySmith(mixedNormal, viewDirInWorld, lightDirInWorld, roughness);
    let numerator = NDF * G * F;
    let denominator = 4.0 * max(dot(mixedNormal, viewDirInWorld), 0.0) * max(dot(mixedNormal, lightDirInWorld), 0.0) + 0.0001;
    let specular = numerator / denominator;

    let kS = F;
    let kD = (vec3(1.0) - kS) * (1.0 - metallic);

    let PI = 3.14159265;
    let NdotL = max(dot(mixedNormal, lightDirInWorld), 0.0);
    let Lo = (kD * albedo / PI + specular) * radiance * NdotL;
    let ambient = 0.03 * albedo * ao;

    var color = ambient + Lo;
    // gamma correct
    // TODO: why does this make it look bad...
    // color = color / (color + vec3(1.0));
    // color = pow(color, vec3(1.0/2.2));

    var out : OutFragment;
    out.normalInCamera = (su.worldToCamera * vec4(mixedNormal, 0) + vec4(1, 1, 1, 0)) / 2;
    out.normalInCamera.a = 1;
    // TODO: think about alpha??
    out.color = vec4(color, 1.0);
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
