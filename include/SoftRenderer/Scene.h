#pragma once
#include <memory>

#include "SoftRenderer/Camera.h"
#include "SoftRenderer/Geometry.h"
#include "SoftRenderer/Math.h"


namespace SR
{
	// Stores all objects, lightings in the world.
	class Scene {
	public:

		Scene()
		{
			bvh_root = std::make_shared<BVH_Node>();
		}

		bool intersect(Ray& ray, double tmin) const
		{
			return bvh_root->intersect(ray, tmin);
			
			// Linear search
			//bool intersected = false;
			//for (const auto& object_ptr : this->object_ptrs) {
			//	if (object_ptr->intersect(ray, tmin)) {
			//		intersected = true;
			//	}
			//}
			//return intersected;
		}

		void add_object(std::shared_ptr<Object3D> object_ptr)
		{
			object_ptrs.push_back(object_ptr);
		}

		void build_bvh()
		{
			// Construct the bvh after all objects are being inserted.
			bvh_root = std::make_shared<BVH_Node>(object_ptrs, 0, object_ptrs.size(), 0);
		}

	private:
		std::shared_ptr<BVH_Node> bvh_root;
		
		std::vector<std::shared_ptr<Object3D>> object_ptrs;
	};
}
