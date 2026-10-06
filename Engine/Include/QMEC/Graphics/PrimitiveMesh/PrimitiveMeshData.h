#pragma once
#include "QMEC/Graphics/IMeshGenerator.h"
#include <utility>
#include <cmath>
#include "QMEC/Math/Vec3.h"

namespace qmec::graphics
{
	inline void CalculateTangents(MeshData& mesh)
	{
		std::vector<Vec3> tangentSums(mesh.vertices.size());
		for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
		{
			const std::uint32_t i0 = mesh.indices[i];
			const std::uint32_t i1 = mesh.indices[i + 1];
			const std::uint32_t i2 = mesh.indices[i + 2];
			if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() || i2 >= mesh.vertices.size())
				continue;

			const Vertex& v0 = mesh.vertices[i0];
			const Vertex& v1 = mesh.vertices[i1];
			const Vertex& v2 = mesh.vertices[i2];
			const Vec3 edge1 = v1.position - v0.position;
			const Vec3 edge2 = v2.position - v0.position;
			const float du1 = v1.uv[0] - v0.uv[0];
			const float dv1 = v1.uv[1] - v0.uv[1];
			const float du2 = v2.uv[0] - v0.uv[0];
			const float dv2 = v2.uv[1] - v0.uv[1];
			const float determinant = du1 * dv2 - du2 * dv1;
			if (std::abs(determinant) <= 1.0e-8f)
				continue;

			const Vec3 tangent = (edge1 * dv2 - edge2 * dv1) * (1.0f / determinant);
			tangentSums[i0] += tangent;
			tangentSums[i1] += tangent;
			tangentSums[i2] += tangent;
		}

		for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
		{
			const Vec3 normal = mesh.vertices[i].normal.Normalized();
			Vec3 tangent = tangentSums[i] - normal * Dot(normal, tangentSums[i]);
			if (tangent.LengthSquared() <= 1.0e-8f)
			{
				const Vec3 reference = std::abs(normal.y) < 0.999f? Vec3{ 0.0f, 1.0f, 0.0f }: Vec3{ 1.0f, 0.0f, 0.0f };
				tangent = Cross(reference, normal);
			}
			mesh.vertices[i].tangent = tangent.Normalized();
		}
	}

	struct CubeMesh final : IMeshGenerator
	{
		[[nodiscard]] MeshData Generate() const override
		{
			std::vector<Vertex> cubeVertices = 
            {

                {
                    { -0.5f, -0.5f, -0.5f },
                    {  0.0f,  0.0f, -1.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {0.0f,1.0f}
                },
                {
                    { -0.5f,  0.5f, -0.5f },
                    {  0.0f,  0.0f, -1.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {0.0f,0.0f}
                },
                {
                    {  0.5f, -0.5f, -0.5f },
                    {  0.0f,  0.0f, -1.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {1.0f,1.0f}
                },
                {
                    {  0.5f,  0.5f, -0.5f },
                    {  0.0f,  0.0f, -1.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {1.0f,0.0f}
                },


                {
                    { -0.5f, -0.5f, 0.5f },
                    {  0.0f,  0.0f, 1.0f },
                    {  1.0f,  1.0f, 1.0f },
                    {  1.0f,  1.0f }
                },
                {
                    {  0.5f, -0.5f, 0.5f },
                    {  0.0f,  0.0f, 1.0f },
                    {  1.0f,  1.0f, 1.0f },
                    {  0.0f,  1.0f }
                },
                {
                    { -0.5f,  0.5f, 0.5f },
                    {  0.0f,  0.0f, 1.0f },
                    {  1.0f,  1.0f, 1.0f },
                    {  1.0f,  0.0f }
                },
                {
                    {  0.5f,  0.5f, 0.5f },
                    {  0.0f,  0.0f, 1.0f },
                    {  1.0f,  1.0f, 1.0f },
                    {  0.0f,  0.0f }
                },


                {
                    { -0.5f, -0.5f, -0.5f },
                    { -1.0f,  0.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  1.0f,  1.0f }
                },
                {
                    { -0.5f, -0.5f,  0.5f },
                    { -1.0f,  0.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  0.0f,  1.0f }
                },
                {
                    { -0.5f,  0.5f, -0.5f },
                    { -1.0f,  0.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  1.0f,  0.0f }
                },
                {
                    { -0.5f,  0.5f,  0.5f },
                    { -1.0f,  0.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  0.0f,  0.0f }
                },


                {
                    { 0.5f, -0.5f, -0.5f },
                    { 1.0f,  0.0f,  0.0f },
                    { 1.0f,  1.0f,  1.0f },
                    { 0.0f,  1.0f }
                },
                {
                    { 0.5f,  0.5f, -0.5f },
                    { 1.0f,  0.0f,  0.0f },
                    { 1.0f,  1.0f,  1.0f },
                    { 0.0f,  0.0f }
                },
                {
                    { 0.5f, -0.5f,  0.5f },
                    { 1.0f,  0.0f,  0.0f },
                    { 1.0f,  1.0f,  1.0f },
                    { 1.0f,  1.0f }
                },
                {
                    { 0.5f,  0.5f,  0.5f },
                    { 1.0f,  0.0f,  0.0f },
                    { 1.0f,  1.0f,  1.0f },
                    { 1.0f,  0.0f }
                },


                {
                    { -0.5f, 0.5f, -0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f, 1.0f,  1.0f },
                    {  0.0f, 1.0f }
                },
                {
                    { -0.5f, 0.5f,  0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f, 1.0f,  1.0f },
                    {  0.0f, 0.0f }
                },
                {
                    {  0.5f, 0.5f, -0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f, 1.0f,  1.0f },
                    {  1.0f, 1.0f }
                },
                {
                    {  0.5f, 0.5f,  0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f, 1.0f,  1.0f },
                    {  1.0f, 0.0f }
                },


                {
                    { -0.5f, -0.5f, -0.5f },
                    {  0.0f, -1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  0.0f,  0.0f }
                },
                {
                    {  0.5f, -0.5f, -0.5f },
                    {  0.0f, -1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  1.0f,  0.0f }
                },
                {
                    { -0.5f, -0.5f,  0.5f },
                    {  0.0f, -1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  0.0f,  1.0f }
                },
                {
                    {  0.5f, -0.5f,  0.5f },
                    {  0.0f, -1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {  1.0f,  1.0f }
                }
            };

            std::vector<std::uint32_t> cubeIndices =
            {

                0U, 1U, 2U,
                1U, 3U, 2U,


                4U, 5U, 6U,
                5U, 7U, 6U,


                8U, 9U, 10U,
                9U, 11U, 10U,


                12U, 13U, 14U,
                13U, 15U, 14U,


                16U, 17U, 18U,
                17U, 19U, 18U,


                20U, 21U, 22U,
                21U, 23U, 22U
            };

			MeshData mesh{ std::move(cubeVertices), std::move(cubeIndices) };
			CalculateTangents(mesh);
			return mesh;

		}
	};

    struct PlaneMesh final : IMeshGenerator
    {
        [[nodiscard]] MeshData Generate() const override
        {
            
            std::vector<Vertex> planeVertices =
            {
                {
                    { -0.5f, 0.0f, -0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {0.0f,1.0f}
                },
                {
                    { -0.5f, 0.0f,  0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {0.0f,0.0f}
                },
                {
                    {  0.5f, 0.0f, -0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {1.0f,1.0f}
                },
                {
                    {  0.5f, 0.0f,  0.5f },
                    {  0.0f, 1.0f,  0.0f },
                    {  1.0f,  1.0f,  1.0f },
                    {1.0f,0.0f}
                }


            };

            std::vector<std::uint32_t> planeIndices =
            {
               
                0U, 1U, 2U,
                1U, 3U, 2U
            };

			MeshData mesh{ std::move(planeVertices), std::move(planeIndices) };
			CalculateTangents(mesh);
			return mesh;

        }
    };

    struct SphereMesh final : IMeshGenerator
    {
        [[nodiscard]] MeshData Generate() const override
        {
            const int latitudeSegments = 16;
            const int longitudeSegments = 16;
            const int verticesPerRing = longitudeSegments + 1;
            const float radius = 1.0f;
            const float PI = 3.14159265359f;

            std::vector<Vertex> sphereVertices{};
            sphereVertices.reserve((latitudeSegments + 1) * verticesPerRing);

            for (int lat = 0; lat <= latitudeSegments; ++lat)
            {
                float phi = static_cast<float>(lat) /latitudeSegments * PI;

                float y = radius * cos(phi);
                float ringRadius = radius * sin(phi);

                for (int lon = 0; lon <= longitudeSegments; ++lon)
                {
                    float theta = lon == longitudeSegments ? 0.0f : static_cast<float>(lon) /longitudeSegments * 2.0f * PI;
                    float x = ringRadius * cos(theta);
                    float z = ringRadius * sin(theta);

                    Vec3 position = { x, y, z };

                    Vec3 normal = position.Normalized();

                    Vec3 color = { 1.0f, 1.0f, 1.0f };

                    float u = static_cast<float>(lon) / longitudeSegments;

                    float v = static_cast<float>(lat) / latitudeSegments;

                    Vertex vertex =
                    {
                        position,
                        normal,
                        color,
                        { u, v }
                    };

                    sphereVertices.push_back(vertex);
                }
            }


            std::vector<std::uint32_t> sphereIndices{};

            sphereIndices.reserve(latitudeSegments *longitudeSegments *6);

            for (int lat = 0; lat < latitudeSegments; ++lat)
            {
                for (int lon = 0; lon < longitudeSegments; ++lon)
                {
                    int current = lat * verticesPerRing + lon;

                    int nextRow = current + verticesPerRing;

                    int nextLongitude = lon + 1;

                    int currentNextLongitude = lat * verticesPerRing + nextLongitude;

                    int nextRowNextLongitude = (lat + 1) * verticesPerRing + nextLongitude;

                  
                    sphereIndices.push_back(current);
                    sphereIndices.push_back(currentNextLongitude);
                    sphereIndices.push_back(nextRow);

                    sphereIndices.push_back(currentNextLongitude);
                    sphereIndices.push_back(nextRowNextLongitude);
                    sphereIndices.push_back(nextRow);
                }
            }

			MeshData mesh{ std::move(sphereVertices), std::move(sphereIndices) };
			CalculateTangents(mesh);
			return mesh;
        }
    };

    struct CylinderMesh final : IMeshGenerator
    {
        [[nodiscard]] MeshData Generate() const override
        {
            constexpr float PI = 3.14159265359f;

            const float height = 2.0f;
            const float radius = 1.0f;

            const int radiusSegments = 20;
            const int heightSegments = 2;
            const int verticesPerRing = radiusSegments + 1;

            std::vector<Vertex> cylinderVertices{};

           

            const float angleStep = 2.0f * PI / radiusSegments;
            const float heightStep = height / heightSegments;

            for (int lat = 0; lat <= heightSegments; ++lat)
            {
                float y = -height * 0.5f + heightStep * lat;

                for (int lon = 0; lon <= radiusSegments; ++lon)
                {
                   
                    float theta = lon == radiusSegments ? 0.0f : angleStep * lon;

                    float x = std::cos(theta) * radius;
                    float z = std::sin(theta) * radius;

                    Vec3 position = { x, y, z };

                    Vec3 normal = Vec3{ x, 0.0f, z }.Normalized();

                    Vec3 color = { 1.0f, 1.0f, 1.0f };

                    float u = static_cast<float>(lon) / radiusSegments;
                    float v = static_cast<float>(lat) / heightSegments;

                    Vertex vertex =
                    {
                        position,
                        normal,
                        color,
                        { u, v }
                    };

                    cylinderVertices.push_back(vertex);
                }
            }

         

            const int bottomCenter = static_cast<int>(cylinderVertices.size());

            cylinderVertices.push_back(
                Vertex
                {
                    { 0.0f, height * -0.5f, 0.0f },
                    { 0.0f, -1.0f, 0.0f },
                    { 1.0f, 1.0f, 1.0f },
                    { 0.5f, 0.5f }
                }
            );

            const int topCenter = static_cast<int>(cylinderVertices.size());

            cylinderVertices.push_back(
                Vertex
                {
                    { 0.0f, height * 0.5f, 0.0f },
                    { 0.0f, 1.0f, 0.0f },
                    { 1.0f, 1.0f, 1.0f },
                    { 0.5f, 0.5f }
                }
            );

          

            // Separate cap rims keep their flat normals independent of the sides.
            const int bottomStart = static_cast<int>(cylinderVertices.size());
            for (int lon = 0; lon < radiusSegments; ++lon)
            {
                Vertex vertex = cylinderVertices[lon];
                vertex.normal = {0.0f, -1.0f, 0.0f};
                vertex.uv[0] = 0.5f + vertex.position.x / (2.0f * radius);
                vertex.uv[1] = 0.5f + vertex.position.z / (2.0f * radius);
                cylinderVertices.push_back(vertex);
            }

            const int topStart = static_cast<int>(cylinderVertices.size());
            for (int lon = 0; lon < radiusSegments; ++lon)
            {
                Vertex vertex = cylinderVertices[heightSegments * verticesPerRing + lon];
                vertex.normal = {0.0f, 1.0f, 0.0f};
                vertex.uv[0] = 0.5f + vertex.position.x / (2.0f * radius);
                vertex.uv[1] = 0.5f + vertex.position.z / (2.0f * radius);
                cylinderVertices.push_back(vertex);
            }

            std::vector<std::uint32_t> cylinderIndices{};

            cylinderIndices.reserve(
                radiusSegments * heightSegments * 6
                + radiusSegments * 3
                + radiusSegments * 3
            );


            for (int lat = 0; lat < heightSegments; ++lat)
            {
                for (int lon = 0; lon < radiusSegments; ++lon)
                {
                    int current = lat * verticesPerRing + lon;

                    int nextRow = current + verticesPerRing;

                    int nextLongitude = lon + 1;

                    int currentNextLongitude = lat * verticesPerRing + nextLongitude;

                    int nextRowNextLongitude = (lat + 1) * verticesPerRing + nextLongitude;

                    // Triangle 1
                    cylinderIndices.push_back(current);
                    cylinderIndices.push_back(nextRow);
                    cylinderIndices.push_back(currentNextLongitude);

                    // Triangle 2
                    cylinderIndices.push_back(currentNextLongitude);
                    cylinderIndices.push_back(nextRow);
                    cylinderIndices.push_back(nextRowNextLongitude);
                }
            }


            for (int lon = 0; lon < radiusSegments; ++lon)
            {
                int nextLongitude =(lon + 1) % radiusSegments;

                int current = bottomStart + lon;
                int next = bottomStart + nextLongitude;

                cylinderIndices.push_back(bottomCenter);
                cylinderIndices.push_back(current);
                cylinderIndices.push_back(next);
            }

        


            for (int lon = 0; lon < radiusSegments; ++lon)
            {
                int nextLongitude = (lon + 1) % radiusSegments;

                int current = topStart + lon;

                int next = topStart + nextLongitude;

                cylinderIndices.push_back(topCenter);
                cylinderIndices.push_back(next);
                cylinderIndices.push_back(current);
            }

			MeshData mesh{ std::move(cylinderVertices), std::move(cylinderIndices) };
			CalculateTangents(mesh);
			return mesh;
        }
    };


}

namespace qmec
{
    using graphics::CalculateTangents;
    using graphics::CubeMesh;
    using graphics::PlaneMesh;
    using graphics::SphereMesh;
    using graphics::CylinderMesh;
}
