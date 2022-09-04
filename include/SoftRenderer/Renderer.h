#pragma once
#include <memory>

#include "SoftRenderer/Material.h"
#include "SoftRenderer/Image.h"
#include "SoftRenderer/Scene.h"
#include "SoftRenderer/Camera.h"


namespace SR
{
	// Abstract class for renderers.
	class Renderer {
	public:
		// Synthesize a photo with the given scene (world) that describes a 3D world and the camera.
		virtual std::shared_ptr<Image> render(const Scene& scene, const Camera& camera) const = 0;
	};


	class TestRenderer : public Renderer {
	public:
		std::shared_ptr<Image> render(const Scene& scene, const Camera& camera) const override
		{
			size_t width = camera.get_image_width();
			size_t height = camera.get_image_height();

			std::shared_ptr<Image> image_ptr = std::make_shared<Image>(width, height);

			for (size_t j = 0; j < height; j++) {
				for (size_t i = 0; i < width; i++) {

					double r = double(i) / (width - 1);
					double g = double(j) / (height - 1);
					double b = 0.25;

					SR::Color color = { r, g, b };
					image_ptr->set_pixel(i, j, color);
				}
			}
			
			return image_ptr;
		}
	};


	class PathTracer : public Renderer {
	public:

		PathTracer() : max_bounces(1), samples_per_pixel(1) {}
		PathTracer(size_t max_bounces, size_t samples_per_pixel)
			: max_bounces(max_bounces), samples_per_pixel(samples_per_pixel) {}

		// Given a ray, return the color (amount of light) from the direction of that ray.
		Color trace_ray(const Scene& scene, Ray& ray, double tmin, size_t cur_bounces) const
		{
			if (scene.intersect(ray, tmin)) {  // Intersect with an object

				Color emittance = ray.get_hit_material()->get_emittance(ray);

				auto scattered_ray_tuple = ray.get_hit_material()->generate_scatter_ray(ray);
				if (!std::get<0>(scattered_ray_tuple) || cur_bounces == max_bounces)
				{
					// Has no valid sample scattered ray or reach the trace limit, then stop tracing.
					return emittance;
				}
				else {
					return emittance + elementwise_multiply(std::get<2>(scattered_ray_tuple), trace_ray(scene, std::get<1>(scattered_ray_tuple),
						std::max(tmin, EPS), cur_bounces + 1));
				}

				//// Visualize the normal vector
				//return 0.5 * (hit.normal + Vector3d{1, 1, 1});
			}
			else {
				// Hits nothing, set background color
				//return Color{ 0.5, 0.7, 1.0 };

				// Hits nothing, return black.
				return Color();
			}
		}

		std::shared_ptr<Image> render(const Scene& scene, const Camera& camera) const override
		{
			size_t width = camera.get_image_width();
			size_t height = camera.get_image_height();

			std::shared_ptr<Image> image_ptr = std::make_shared<Image>(width, height);

			for (int j = 1; j <= height; j++) {

				// Print the progress
				std::cout << "\rScanlines remaining: " << std::setw(5) << height - j 
					<< " (Progress: " << std::fixed << std::setprecision(1) << static_cast<double>(j) / height * 100.0 << "%)" << std::flush;

                #pragma omp parallel for schedule(dynamic, 4)
				for (int i = 1; i <= width; i++) {

					Color color = { 0, 0, 0 };
					
					// Jittered sampling within the pixel
					for (size_t k = 0; k < samples_per_pixel; k++) {
						Ray ray = camera.generate_ray({ i - 0.5, j - 0.5 });
						color += this->trace_ray(scene, ray, camera.get_tmin(), 0);
					}

					color /= static_cast<double>(samples_per_pixel);
					image_ptr->set_pixel(i - 1, j - 1, color);
				}
			}
			std::cout << std::endl;

			return image_ptr;
		}

	private:
		size_t max_bounces;
		size_t samples_per_pixel;
	};


	class Rasterizer : public Renderer {
	public:
	};
}