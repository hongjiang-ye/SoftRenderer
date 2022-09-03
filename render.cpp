#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
using std::cout;
using std::endl;

#include "SoftRenderer/Math.h"
#include "SoftRenderer/Renderer.h"
#include "SoftRenderer/Image.h"
#include "SoftRenderer/Scene.h"
#include "SoftRenderer/Camera.h"
#include "SoftRenderer/Geometry.h"


std::pair<SR::Scene, SR::PinholeCamera> pinhole_setting1(size_t supersampling_factor)
{
    /* Settings for Pinhole Camera */
    size_t image_height;
    image_height = 720;
    //image_height = 60;
    double aspect_ratio = 16.0 / 9.0;
    size_t image_width = static_cast<size_t>(std::round(image_height * aspect_ratio));

    size_t render_height = image_height * supersampling_factor;
    size_t render_width = image_width * supersampling_factor;

    double fov_y = 45;
    SR::PinholeCamera camera(
        { -3, 1, 1.5 }, { 0, 0, -1 }, { 0, 1, 0 },
        render_width, render_height, SR::get_radian(fov_y)
    );
    camera.print();

    SR::Scene scene;

    auto material_ground = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.8, 0.8, 0.0 });
    auto material_center = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.1, 0.2, 0.5 });
    auto material_left = std::make_shared<SR::Dielectric>(1.5);
    auto material_right = std::make_shared<SR::Metal>(SR::Vector3d{ 0.8, 0.6, 0.2 }, 0.5);

    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0.0, -100.5, -1.0 }, 100.0, material_ground));  // ground
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0.0, 0.0, -1.0 }, 0.5, material_center));
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ -1.0, 0.0, -1.0 }, 0.5, material_left));
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 1.0, 0.0, -1.0 }, 0.5, material_right));

    return std::make_pair(scene, camera);
}


std::pair<SR::Scene, SR::ThinLensCamera> thinlens_setting1(size_t supersampling_factor)
{
    /* Settings for ThinLens Camera 1 */
    size_t image_height = 180;  // resolution (xxP)
    size_t render_height = image_height * supersampling_factor;

    double sensor_width = 36;  // in unit mm, for convenience
    double sensor_height = 24;
    double focal_length = 26;  // in unit mm
    double aperture_F_num = 1.6;  // F-Number

    SR::Vector3d eye = { -15, 6, 20 };
    SR::Vector3d lookat = { 0, 4, 0 };
    SR::Vector3d eye_lookat = lookat - eye;
    SR::Vector3d eye_left = SR::Vector3d{ -9.0, 4.0, 2.0 } - eye;
    SR::Vector3d eye_right = SR::Vector3d{ 14.0, 4.0, -2.0 } - eye;
    double focus_left_dist = dot(eye_lookat, eye_left) / eye_lookat.norm() * 10;
    double focus_right_dist = dot(eye_lookat, eye_right) / eye_lookat.norm() * 10;
    double focal_plane_dist = (eye - lookat).norm() * 10;  // in unit mm; the desired depth of object

    SR::ThinLensCamera camera(
        eye, lookat, { 0, 1, 0 },
        sensor_width, sensor_height, render_height, focal_length, aperture_F_num, focal_plane_dist
    );
    camera.print();

    SR::Scene scene;

    auto material_ground = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.8, 0.8, 0.0 });
    auto material_center = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.1, 0.2, 0.5 });
    auto material_left = std::make_shared<SR::Dielectric>(1.5);
    auto material_right = std::make_shared<SR::Metal>(SR::Vector3d{ 0.8, 0.6, 0.2 }, 0.5);

    // Assume the position is in unit cm
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0.0, -1000.0, 0.0 }, 1000.0, material_ground));  // ground
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0.0, 4.0, 0.0 }, 4, material_center));
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ -9.0, 4.0, 2.0 }, 4, material_left));
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 14.0, 4.0, -2.0 }, 4, material_right));

    return std::make_pair(scene, camera);
}


std::pair<SR::Scene, SR::ThinLensCamera> thinlens_setting2(size_t supersampling_factor)
{
    /* Settings for ThinLens Camera 2 */
    size_t image_height = 720;  // resolution (xxP)
    size_t render_height = image_height * supersampling_factor;

    double sensor_width = 36;  // in unit mm, for convenience
    double sensor_height = 24;
    double focal_length = 35;  // in unit mm
    double aperture_F_num = 16;  // F-Number

    SR::Vector3d eye = { 13, 2, 3 };
    SR::Vector3d lookat = { 0, 0, 0 };
    double focal_plane_dist = 100;  // in unit mm; the desired depth of object

    SR::ThinLensCamera camera(
        eye, lookat, { 0, 1, 0 },
        sensor_width, sensor_height, render_height, focal_length, aperture_F_num, focal_plane_dist
    );
    camera.print();

    SR::Scene scene;

    auto material_ground = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.5, 0.5, 0.5 });
    scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0.0, -1000.0, 0.0 }, 1000.0, material_ground));

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = SR::Random::rand_uniform(0, 1);
            SR::Vector3d center = { a + 0.9 * SR::Random::rand_uniform(0, 1), 0.2, b + 0.9 * SR::Random::rand_uniform(0, 1) };

            if ((center - SR::Vector3d{ 4, 0.2, 0 }).norm() > 0.9) {
                std::shared_ptr<SR::Material> sphere_material;

                if (choose_mat < 0.6) {
                    // diffuse
                    auto albedo = SR::elementwise_multiply(SR::rand_vec3_on_unit_sphere(), SR::rand_vec3_on_unit_sphere());
                    sphere_material = std::make_shared<SR::Lambertian>(albedo);
                    scene.add_object(std::make_shared<SR::Sphere>(center, 0.2, sphere_material));
                }
                else if (choose_mat < 0.85) {
                    // metal
                    SR::Vector3d albedo = { SR::Random::rand_uniform(0.5, 1) , SR::Random::rand_uniform(0.5, 1) , SR::Random::rand_uniform(0.5, 1) };
                    auto fuzz = SR::Random::rand_uniform(0, 0.5);
                    sphere_material = std::make_shared<SR::Metal>(albedo, fuzz);
                    scene.add_object(std::make_shared<SR::Sphere>(center, 0.2, sphere_material));

                }
                else {
                    // glass
                    sphere_material = std::make_shared<SR::Dielectric>(1.5);
                    scene.add_object(std::make_shared<SR::Sphere>(center, 0.2, sphere_material));
                }
            }

            auto material1 = std::make_shared<SR::Dielectric>(1.5);
            scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 0, 1, 0 }, 1.0, material1));

            auto material2 = std::make_shared<SR::Lambertian>(SR::Vector3d{ 0.4, 0.2, 0.1 });
            scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ -4, 1, 0 }, 1.0, material2));

            auto material3 = std::make_shared<SR::Metal>(SR::Vector3d{ 0.7, 0.6, 0.5 }, 0.0);
            scene.add_object(std::make_shared<SR::Sphere>(SR::Vector3d{ 4, 1, 0 }, 1.0, material3));
        }
    }

    return std::make_pair(scene, camera);
}


int main()
{
    double a = 0.0;
    size_t max_bounces = 10;
    size_t samples_per_pixel = 40;
    SR::PathTracer renderer(max_bounces, samples_per_pixel);

    size_t supersampling_factor = 2;  // factor x factor subpixels

    //auto scene_camera = pinhole_setting1(supersampling_factor);
    //auto scene_camera = thinlens_setting1(supersampling_factor);
    auto scene_camera = thinlens_setting2(supersampling_factor);

    scene_camera.first.build_bvh();

    /* Start rendering */
    std::cout << "Start rendering..." << std::endl;
    auto t_start = std::chrono::steady_clock::now();

    std::shared_ptr<SR::Image> supersampled_image_ptr = renderer.render(scene_camera.first, scene_camera.second);

    auto t_end = std::chrono::steady_clock::now();
    std::cout << "Rendering is done in "
        << std::chrono::duration<double>(t_end - t_start).count()
        << " seconds." << std::endl;

    // Downsampling
    // super_sampled_image_ptr ->gaussian_blur();  // smoothing
    std::shared_ptr<SR::Image> image_ptr = supersampled_image_ptr->downsample(supersampling_factor);

    image_ptr->save("./test.ppm");

    return 0;
}


