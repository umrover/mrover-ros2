// Offline model tools for the MuJoCo sim, driven by scripts/convert_models.py.
// One binary, two subcommands (never invoked by the ROS build):
//
//   model_tools urdf2mjcf <in.urdf> <out.xml>   compile a prepared URDF to MJCF
//   model_tools fbx2msh  <in.fbx>  <out_dir>    split an FBX into per-material .msh
//
// C++ because this env reaches MuJoCo/assimp only through their C/C++ libs.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace {

    // --- urdf2mjcf -----------------------------------------------------------------

    // Compile a (xacro-expanded, path-rewritten) URDF to MJCF. MuJoCo's compiler discards
    // FBX visuals, bakes inertias, and resolves meshes, so the committed model needs no
    // runtime URDF workarounds.
    auto urdf2mjcf(char const* inPath, char const* outPath) -> int {
        std::ifstream in{inPath};
        if (!in) {
            std::fprintf(stderr, "could not open %s\n", inPath);
            return 1;
        }
        std::stringstream ss;
        ss << in.rdbuf();
        std::string urdf = ss.str();

        char error[1024] = "";
        mjSpec* spec = mj_parseXMLString(urdf.c_str(), nullptr, error, sizeof(error));
        if (!spec) {
            std::fprintf(stderr, "parse failed: %s\n", error);
            return 1;
        }

        // Compile mutates the spec, so the saved MJCF reflects the compiled result.
        mjModel* model = mj_compile(spec, nullptr);
        if (!model) {
            std::fprintf(stderr, "compile failed: %s\n", mjs_getError(spec));
            return 1;
        }

        int capacity = 1 << 23;
        std::string xml(capacity, '\0');
        if (mj_saveXMLString(spec, xml.data(), capacity, error, sizeof(error)) < 0) {
            std::fprintf(stderr, "save failed: %s\n", error);
            return 1;
        }

        std::ofstream out{outPath};
        out << xml.c_str();
        std::printf("%s -> %s (nbody=%d ngeom=%d)\n", inPath, outPath,
                    static_cast<int>(model->nbody), static_cast<int>(model->ngeom));

        mj_deleteModel(model);
        mj_deleteSpec(spec);
        return 0;
    }

    // --- fbx2msh -------------------------------------------------------------------

    auto sanitize(std::string const& s) -> std::string {
        std::string out;
        bool prevUnderscore = false;
        for (char c: s) {
            if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                out += c;
                prevUnderscore = false;
            } else if (!prevUnderscore) {
                out += '_';
                prevUnderscore = true;
            }
        }
        while (!out.empty() && out.back() == '_') out.pop_back();
        std::size_t start = out.find_first_not_of('_');
        return start == std::string::npos ? out : out.substr(start);
    }

    auto jsonEscape(std::string const& s) -> std::string {
        std::string out;
        for (char c: s) {
            if (c == '"' || c == '\\') out += '\\';
            out += c;
        }
        return out;
    }

    // Read raw node-local vertices (no PreTransformVertices), like the old loader; the
    // renderer places them at the URDF link frame. (`assimp export` bakes the FBX node
    // transforms in, mislocating everything.) MuJoCo allows one material per mesh, so
    // vertices are grouped by material and each group written as a legacy binary .msh
    // (smaller than OBJ) plus a materials.json sidecar of color/texture per material:
    //   int32 nvertex, nnormal, ntexcoord, nface  (nnormal/ntexcoord 0 or nvertex; nvertex>=4)
    //   float vertex[3n], normal[3n], texcoord[2n]; int32 face[3*nface] (0-based)
    auto fbx2msh(std::string const& in, std::string const& outDir) -> int {
        Assimp::Importer importer;
        importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);
        // No FlipUVs: legacy .msh texcoords are used as-is (MuJoCo's OBJ loader flips V).
        aiScene const* scene = importer.ReadFile(in, aiProcessPreset_TargetRealtime_Fast);
        if (!scene) {
            std::fprintf(stderr, "import error: %s\n", importer.GetErrorString());
            return 1;
        }

        std::map<unsigned, std::vector<aiMesh const*>> byMaterial;
        for (unsigned i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh const* mesh = scene->mMeshes[i];
            if (mesh->mNumVertices == 0) continue;
            byMaterial[mesh->mMaterialIndex].push_back(mesh);
        }

        std::string json = "{\n";
        bool first = true;

        for (auto const& [matIndex, meshes]: byMaterial) {
            bool hasUV = true, hasNormal = true;
            for (aiMesh const* mesh: meshes) {
                if (!mesh->HasTextureCoords(0)) hasUV = false;
                if (!mesh->HasNormals()) hasNormal = false;
            }

            std::vector<float> pos, nrm, uv;
            std::vector<std::int32_t> faces;
            std::int32_t base = 0;
            for (aiMesh const* mesh: meshes) {
                for (unsigned v = 0; v < mesh->mNumVertices; ++v) {
                    aiVector3D const& p = mesh->mVertices[v];
                    pos.insert(pos.end(), {p.x, p.y, p.z});
                    if (hasNormal) {
                        aiVector3D const& n = mesh->mNormals[v];
                        nrm.insert(nrm.end(), {n.x, n.y, n.z});
                    }
                    if (hasUV) {
                        aiVector3D const& t = mesh->mTextureCoords[0][v];
                        uv.insert(uv.end(), {t.x, t.y});
                    }
                }
                for (unsigned fa = 0; fa < mesh->mNumFaces; ++fa) {
                    aiFace const& face = mesh->mFaces[fa];
                    if (face.mNumIndices != 3) continue;
                    faces.push_back(base + static_cast<std::int32_t>(face.mIndices[0]));
                    faces.push_back(base + static_cast<std::int32_t>(face.mIndices[1]));
                    faces.push_back(base + static_cast<std::int32_t>(face.mIndices[2]));
                }
                base += static_cast<std::int32_t>(mesh->mNumVertices);
            }

            std::int32_t nvertex = base;
            if (nvertex < 4 || faces.empty()) continue; // MSH requires nvertex >= 4

            aiMaterial const* material = scene->mMaterials[matIndex];
            aiString matName;
            material->Get(AI_MATKEY_NAME, matName);
            std::string matSan = sanitize(matName.C_Str());
            if (matSan.empty()) {
                matSan = "mat_";
                matSan += std::to_string(matIndex);
            }

            std::int32_t nnormal = hasNormal ? nvertex : 0;
            std::int32_t ntexcoord = hasUV ? nvertex : 0;
            std::int32_t nface = static_cast<std::int32_t>(faces.size() / 3);

            std::string mshPath = outDir;
            mshPath.append("/").append(matSan).append(".msh");
            FILE* f = std::fopen(mshPath.c_str(), "wb");
            if (!f) {
                std::fprintf(stderr, "cannot write %s\n", mshPath.c_str());
                return 1;
            }
            std::int32_t header[4] = {nvertex, nnormal, ntexcoord, nface};
            std::fwrite(header, sizeof(std::int32_t), 4, f);
            std::fwrite(pos.data(), sizeof(float), pos.size(), f);
            if (nnormal) std::fwrite(nrm.data(), sizeof(float), nrm.size(), f);
            if (ntexcoord) std::fwrite(uv.data(), sizeof(float), uv.size(), f);
            std::fwrite(faces.data(), sizeof(std::int32_t), faces.size(), f);
            std::fclose(f);

            aiColor3D diffuse{0.7F, 0.7F, 0.7F};
            material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
            std::string texture;
            aiString texPath;
            if (material->GetTexture(aiTextureType_DIFFUSE, 0, &texPath) == AI_SUCCESS) {
                std::string p = texPath.C_Str();
                std::size_t slash = p.find_last_of("/\\");
                texture = slash == std::string::npos ? p : p.substr(slash + 1);
            }

            if (!first) json.append(",\n");
            first = false;
            json.append("  \"").append(jsonEscape(matSan)).append("\": {");
            json.append("\"rgba\": [").append(std::to_string(diffuse.r)).append(", ")
                .append(std::to_string(diffuse.g)).append(", ")
                .append(std::to_string(diffuse.b)).append(", 1.0], ");
            json.append("\"texture\": ");
            if (texture.empty()) json.append("null");
            else json.append("\"").append(jsonEscape(texture)).append("\"");
            json.append(", \"file\": \"").append(jsonEscape(matSan)).append(".msh\"}");
        }
        json.append("\n}\n");

        std::string jsonPath = outDir;
        jsonPath.append("/materials.json");
        FILE* jf = std::fopen(jsonPath.c_str(), "wb");
        if (!jf) {
            std::fprintf(stderr, "cannot write %s\n", jsonPath.c_str());
            return 1;
        }
        std::fputs(json.c_str(), jf);
        std::fclose(jf);
        return 0;
    }

    auto usage() -> void {
        std::fprintf(stderr,
                     "usage:\n"
                     "  model_tools urdf2mjcf <in.urdf> <out.xml>\n"
                     "  model_tools fbx2msh  <in.fbx>  <out_dir>\n");
    }

} // namespace

auto main(int argc, char** argv) -> int {
    if (argc < 4) {
        usage();
        return 2;
    }
    std::string cmd = argv[1];
    if (cmd == "urdf2mjcf") return urdf2mjcf(argv[2], argv[3]);
    if (cmd == "fbx2msh") return fbx2msh(argv[2], argv[3]);
    usage();
    return 2;
}
