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

        bvh::SingleRayTraverser<Bvh>::Statistics before_statistics;
        std::bitset<64> before_closest_hit_path;
        auto before_result = traverser.traverse(ray, primitive_intersector, before_statistics,
                                                false, std::bitset<64>(),
                                                before_closest_hit_path);
        std::cout << "Before: " << before_statistics.node_traversed << ", "
                  << before_statistics.node_intersections << ", "
                  << before_statistics.trig_intersections << std::endl;
        if (before_result) {
            std::cout << before_result->primitive_index << ", "
                      << before_result->intersection.t << ", "
                      << before_result->intersection.u << ", "
                      << before_result->intersection.v << std::endl;
        } else {
            std::cout << "MISSED" << std::endl;
        }

        // check whether before_closest_hit_path is correct
        auto first_path = before_closest_hit_path;
        auto* curr = &bvh.nodes[0];
        while (!curr->is_leaf()) {
            if (first_path.test(0)) curr = &bvh.nodes[curr->first_child_or_primitive + 1];
            else curr = &bvh.nodes[curr->first_child_or_primitive];
            first_path = first_path >> 1;
        }
        if (before_result) {
            bool found = false;
            for (size_t i = curr->first_child_or_primitive;
                 i < curr->first_child_or_primitive + curr->primitive_count; i++)
                found |= bvh.primitive_indices[i] == before_result->primitive_index;
            assert(found);
        }

        bvh::SingleRayTraverser<Bvh>::Statistics after_statistics;
        std::bitset<64> after_closest_hit_path;
        auto after_result = traverser.traverse(ray, primitive_intersector, after_statistics,
                                               before_result.has_value(), before_closest_hit_path,
                                               after_closest_hit_path);
        std::cout << "After: " << after_statistics.node_traversed << ", "
                  << after_statistics.node_intersections << ", "
                  << after_statistics.trig_intersections << std::endl;
        if (after_result) {
            std::cout << after_result->primitive_index << ", "
                      << after_result->intersection.t << ", "
                      << after_result->intersection.u << ", "
                      << after_result->intersection.v << std::endl;
        } else {
            std::cout << "MISSED" << std::endl;
        }

        assert(before_result->primitive_index == after_result->primitive_index);
        assert(before_result->intersection.t == after_result->intersection.t);
        assert(before_result->intersection.u == after_result->intersection.u);
        assert(before_result->intersection.v == after_result->intersection.v);

        std::cout << std::endl;
    }
}
