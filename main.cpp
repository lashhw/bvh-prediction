#include <bvh/triangle.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include "bvh-analysis/utils/parse.hpp"
#include "bvh-analysis/utils/build_bvh.hpp"

using Triangle = bvh::Triangle<float>;
using Ray = bvh::Ray<float>;
using Bvh = bvh::Bvh<float>;

int main() {
    std::vector<Triangle> triangles = parse_ply("sponza.ply");
    Bvh bvh = build_bvh(triangles);
    std::cout << "BVH has " << bvh.node_count << " nodes" << std::endl;

    bvh::ClosestPrimitiveIntersector<Bvh, Triangle> primitive_intersector(bvh, triangles.data());
    bvh::SingleRayTraverser<Bvh> traverser(bvh);

    std::ifstream ray_queries_file("ray_queries.bin", std::ios::in | std::ios::binary);
    float r[7];
    while (ray_queries_file.read(reinterpret_cast<char*>(&r), 7 * sizeof(float))) {
        Ray ray(
                Vector3(r[0], r[1], r[2]),
                Vector3(r[3], r[4], r[5]),
                0.f,
                r[6]
        );

        bvh::SingleRayTraverser<Bvh>::Statistics statistics;
        std::bitset<64> closest_hit_path;

        traverser.traverse(ray, primitive_intersector, statistics, closest_hit_path);
        std::cout << "Before: " << statistics.traversal_steps << ", " << statistics.intersections << std::endl;
        std::cout << closest_hit_path.to_string() << std::endl;
        std::cout << std::endl;
    }
}
