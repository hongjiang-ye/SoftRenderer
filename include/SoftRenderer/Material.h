#pragma once

#include "SoftRenderer/Math.h"
#include "SoftRenderer/Geometry.h"


namespace SR
{

	class Material {
    public:
        // Try to sample a scattered ray (either reflected or refracted) with its attenuation.
        virtual std::tuple<bool, Ray, Vector3d> generate_scatter_ray(const Ray& ray_in) const = 0;
	};


    // Ideal lambertian (with ideal diffusion)
    class Lambertian : public Material {
    public:
        Lambertian(const Vector3d& albedo) : albedo(albedo) {}

        virtual std::tuple<bool, Ray, Vector3d> generate_scatter_ray(const Ray& ray_in) const override
        {
            // Sample a reflected ray
            Vector3d reflected_dir = ray_in.get_hit_normal() + rand_vec3_on_unit_sphere();
            Ray scattered_ray = Ray(ray_in.get_hit_point(), reflected_dir);

            return std::make_tuple(true, scattered_ray, albedo);
        }

        Vector3d albedo;
    };

    // Fuzzy metal (perfect reflection with a bit fuzziness)
    class Metal : public Material {
    public:
        Metal(const Vector3d& albedo, double fuzziness) : albedo(albedo), fuzziness(fuzziness) {}

        virtual std::tuple<bool, Ray, Vector3d> generate_scatter_ray(const Ray& ray_in) const override
        {
            // Sample a reflected ray
            Vector3d reflected_dir = reflect_dir(ray_in.direction, ray_in.get_hit_normal()) 
                + fuzziness * rand_vec3_on_unit_sphere();

            Ray scattered_ray = Ray(ray_in.get_hit_point(), reflected_dir);

            bool valid = true;
            if (dot(reflected_dir, ray_in.get_hit_normal()) < 0) {
                // If scattered inside the surface, then don't trace it.
                valid = false;
            }

            return std::make_tuple(valid, scattered_ray, albedo);
        }

        Vector3d albedo;
        double fuzziness;
    };

    // Dielectric material with reflection and refraction
    class Dielectric : public Material {
    public:
        Dielectric(double ior) : ior(ior) {}

        virtual std::tuple<bool, Ray, Vector3d> generate_scatter_ray(const Ray& ray_in) const override
        {
            Vector3d albedo = { 1, 1, 1 };  // Don't absorb any light
            Vector3d scattered_dir;

            double ior_in, ior_out;
            if (ray_in.is_hit_front_face()) {
                ior_in = 1.0;  // from air
                ior_out = ior;
            }
            else {
                ior_in = ior;
                ior_out = 1.0;
            }
            std::tuple<bool, Vector3d, double> refracted_ray_tuple = refracted_dir(ray_in.direction, ray_in.get_hit_normal(), 
                ior_in, ior_out);

            if (std::get<0>(refracted_ray_tuple) && Random::rand_uniform(0, 1) > std::get<2>(refracted_ray_tuple)) {
                // Has valid sampled refraction ray, and decide to use it
                scattered_dir = std::get<1>(refracted_ray_tuple);
            }
            else {
                // Total internal reflection
                scattered_dir = reflect_dir(ray_in.direction, ray_in.get_hit_normal());
            }
            
            return std::make_tuple(true, Ray(ray_in.get_hit_point(), scattered_dir), albedo);
        }

        double ior;  // index of refraction
    };

}