#pragma once

#include "Object.hpp"
#include <memory>
#include <cstring>

bool rayTriangleIntersect(const Vector3f &va, const Vector3f &vb, const Vector3f &vc, const Vector3f &eye,
                          const Vector3f &dir, float &tnear, float &u, float &v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.

    float a = va.x - vb.x;
    float b = va.y - vb.y;
    float c = va.z - vb.z;
    float d = va.x - vc.x;
    float e = va.y - vc.y;
    float f = va.z - vc.z;
    float g = dir.x;
    float h = dir.y;
    float i = dir.z;
    float j = va.x - eye.x;
    float k = va.y - eye.y;
    float l = va.z - eye.z;

    auto eihf = e * i - h * f;
    auto gfdi = g * f - d * i;
    auto dheg = d * h - e * g;
    auto akjb = a * k - j * b;
    auto jcal = j * c - a * l;
    auto blkc = b * l - k * c;

    auto M = a * eihf + b * gfdi + c * dheg;

    tnear = -(f * akjb + e * jcal + d * blkc) / M;

    if (tnear < 0)
    {
        return false;
    }

    u = (i * akjb + h * jcal + g * blkc) / M;
    if (u < 0 || u > 1)
    {
        return false;
    }

    v = (j * eihf + k * gfdi + l * dheg) / M;
    if (v < 0 || v > 1 - u)
    {
        return false;
    }
    return true;
}

class MeshTriangle : public Object
{
public:
    // Vector3f verts[4] = {{-5, -3, -6}, {5, -3, -6}, {5, -3, -16}, {-5, -3, -16}};
    // uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};
    // Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    MeshTriangle(const Vector3f *verts, const uint32_t *vertsIndex, const uint32_t &numTris, const Vector2f *st)
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;

        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);

        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;

        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool intersect(const Vector3f &orig, const Vector3f &dir, float &tnear, uint32_t &index,
                   Vector2f &uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)
        {
            const Vector3f &v0 = vertices[vertexIndex[k * 3]];
            const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
            {
                tnear = t;
                uv.x = u;
                uv.y = v;
                index = k;
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &index, const Vector2f &uv, Vector3f &N,
                              Vector2f &st) const override
    {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y; //* st为纹理坐标，这里用barycentre差值计算了中心坐标
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const override
    {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
//-fsanitize-undefined-trap-on-error