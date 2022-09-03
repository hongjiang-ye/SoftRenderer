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

		}

		bool intersect(Ray& ray, double tmin) const
		{
			bool intersected = false;
			for (const auto& object_ptr : this->object_ptrs) {
				if (object_ptr->intersect(ray, tmin)) {
					intersected = true;
				}
			}
			return intersected;
		}

		void add_object(std::shared_ptr<Object3D> object_ptr)
		{
			object_ptrs.push_back(object_ptr);
		}

	private:
		std::vector<std::shared_ptr<Object3D>> object_ptrs;
	};
}