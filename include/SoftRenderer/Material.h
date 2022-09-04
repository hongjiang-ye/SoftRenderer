#pragma once
#include "SoftRenderer/Math.h"
#include "SoftRenderer/Geometry.h"
#include "SoftRenderer/Image.h"


namespace SR
{
    // Determine the ray scattering behaviour.
	struct Material {
        // Return information about a sampled scattered ray (either reflected or refracted).
        // tuple[0]: whether a scattered direction is sampled
        // tuple[1]: the scattered ray
        // tuple[2]: the color attenuation of the scattered ray
        virtual std::tuple<bool, Ray, Color> generate_scatter_ray(const Ray& ray_in) const = 0;
	};


    // Determine the color (light attenuation) at a specific point of the material.
    struct Texture {
        virtual Color sample_color(Point2 text_coord) const = 0;
    };

    struct SolidColor : Texture {
        SolidColor(Color albedo) : albedo(albedo) {}

        Color sample_color(Point2 text_coord) const override
        {
            return albedo;
        }

        Color albedo;
    };

    struct ImageTexture : Texture {
        ImageTexture(const std::string& filename)
        {
            image_ptr = Image::load_ppm(filename);
        }

        Color sample_color(Point2 text_coord) const override
        {
            size_t x = static_cast<size_t>(text_coord.x() * image_ptr->get_width());
            size_t y = static_cast<size_t>(text_coord.y() * image_ptr->get_height());
            return image_ptr->get_pixel(x, y);
        }

        std::shared_ptr<Image> image_ptr;
    };


    // Ideal lambertian (with ideal diffusion)
    struct Lambertian : Material {
        Lambertian(const Color& albedo) : albedo(std::make_shared<SolidColor>(albedo)) {}
        Lambertian(const std::shared_ptr<Texture>& albedo) : albedo(albedo) {}

        virtual std::tuple<bool, Ray, Color> generate_scatter_ray(const Ray& ray_in) const override
        {
            // Sample a reflected ray
            Vector3 reflected_dir = ray_in.get_hit_normal() + rand_vec3_on_unit_sphere();
            Ray scattered_ray = Ray(ray_in.get_hit_point(), reflected_dir);

            return std::make_tuple(true, scattered_ray, albedo->sample_color(ray_in.get_hit_textcoord()));
        }

        std::shared_ptr<Texture> albedo;
    };

    // Fuzzy metal (perfect reflection with a bit fuzziness)
    struct Metal : Material {
        Metal(const Color& albedo, double fuzziness) : albedo(albedo), fuzziness(fuzziness) {}

        virtual std::tuple<bool, Ray, Color> generate_scatter_ray(const Ray& ray_in) const override
        {
            // Sample a reflected ray
            Vector3 reflected_dir = reflect_dir(ray_in.direction, ray_in.get_hit_normal()) 
                + fuzziness * rand_vec3_on_unit_sphere();

            Ray scattered_ray = Ray(ray_in.get_hit_point(), reflected_dir);

            bool valid = true;
            if (dot(reflected_dir, ray_in.get_hit_normal()) < 0) {
                // If scattered inside the surface, then don't trace it.
                valid = false;
            }

            return std::make_tuple(valid, scattered_ray, albedo);
        }

        Color albedo;
        double fuzziness;
    };

    // Dielectric material with reflection and refraction
    struct Dielectric : Material {
 
        Dielectric(double ior) : ior(ior) {}

        virtual std::tuple<bool, Ray, Color> generate_scatter_ray(const Ray& ray_in) const override
        {
            Color albedo = { 1, 1, 1 };  // Don't absorb any light
            Vector3 scattered_dir;

            double ior_in, ior_out;
            if (ray_in.is_hit_front_face()) {
                ior_in = 1.0;  // from air
                ior_out = ior;
            }
            else {
                ior_in = ior;
                ior_out = 1.0;
            }
            auto refracted_ray_tuple = refracted_dir(ray_in.direction, ray_in.get_hit_normal(), ior_in, ior_out);

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
