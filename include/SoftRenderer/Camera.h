#pragma once
#include <iostream>
#include <cmath>

#include "SoftRenderer/Geometry.h"

using std::cout;
using std::endl;


namespace SR
{
	// Abstract class for cameras.
	class Camera {
	public:

		Camera(const Vector3d& eye, const Vector3d& lookat, const Vector3d& up,
			size_t image_width, size_t image_height)
			: eye(eye), lookat(lookat), direction((lookat - eye).normalized()), up(up.normalized()),
			image_width(image_width), image_height(image_height)
		{
			// Construct camera coordinate
			cam_z = -direction.normalized();  // z; camear is looking at -z;
			cam_x = cross(up, cam_z).normalized();  // y x z = x
			cam_y = cross(cam_z, cam_x).normalized();  // z x x = y
			
			aspect_ratio = (1.0f * image_width) / image_height;
		}

		// Generate a ray (which may be sampled from a random distribution) that originates from the target pixel.
		virtual Ray generate_ray(const Vector2d& pixel_coord) const = 0;

		size_t get_image_width() const
		{
			return image_width;
		}

		size_t get_image_height() const
		{
			return image_height;
		}

		double get_tmin() const
		{
			return 0.0f;
		}
		
		virtual void print() const = 0;

		virtual ~Camera() = default;

	protected:
		// Camera position and orientation
		Vector3d eye;  // camera position
		Vector3d lookat;
		Vector3d direction;  // look at direction
		Vector3d up;

		// Camera coordinate
		Vector3d cam_x;
		Vector3d cam_y;
		Vector3d cam_z;

		// The image properties
		size_t image_width;
		size_t image_height;
		double aspect_ratio;
	};


	class PinholeCamera : public Camera {
	public:
		PinholeCamera(const Vector3d& eye, const Vector3d& lookat, const Vector3d& up,
			size_t image_width, size_t image_height, double fov_y, double sensor_plane_dist = 1.0f)
			: Camera(eye, lookat, up, image_width, image_height), fov_y(fov_y), sensor_plane_dist(sensor_plane_dist)
		{
			sensor_height = std::tan(fov_y / 2) * sensor_plane_dist * 2;
			sensor_width = aspect_ratio * sensor_height;
		}

		Ray generate_ray(const Vector2d& pixel_coord) const override
		{
			// Randomly sample a point inside the pixel
			double Px = pixel_coord.x() + 0.5 * Random::rand_uniform();
			double Py = pixel_coord.y() + 0.5 * Random::rand_uniform();

			// Transform the image origin to the image center to make computation easier.
			Px -= image_width / 2.0f;
			Py -= image_height / 2.0f;

			// The corresponding point on viewport (sensor plane)
			double Vx = Px / image_width * sensor_width;
			double Vy = Py / image_height * sensor_height;

			// Sample a offset point on lens
			Vector3d dir = (Vx * cam_x + Vy * cam_y + sensor_plane_dist * direction).normalized();
			return Ray(eye, dir);
		}

		void print() const override
		{
			cout << "======== Pinhole Camera ========" << endl;
			cout << "eye: " << eye << endl;
			cout << "lookat: " << lookat << endl;
			cout << "up: " << up << endl;
			cout << endl;
			cout << "cam_x: " << cam_x << endl;
			cout << "cam_y: " << cam_y << endl;
			cout << "cam_z: " << cam_z << endl;
			cout << endl;
			cout << "image size: " << image_width << " x " << image_height << endl;
			cout << "aspect_ratio: " << aspect_ratio << endl;
			cout << "fov_y: " << fov_y << " (" << get_angle(fov_y) << " degrees)" << endl;
			cout << "================================" << endl;
		}

	private:
		/* Sensor properties */
		double sensor_width;
		double sensor_height;
		double sensor_plane_dist;

		// The image's properties
		double fov_y;  // in radian
	};


	class ThinLensCamera : public Camera {
	public:
		ThinLensCamera(const Vector3d& eye, const Vector3d& lookat, const Vector3d& up,
			double sensor_width, double sensor_height, size_t image_height,
			double focal_length, double aperture_F_num, double focal_plane_dist)
			: Camera(eye, lookat, up, 0, image_height), sensor_width(sensor_width), sensor_height(sensor_height), 
			focal_length(focal_length), aperture_F_num(aperture_F_num), focal_plane_dist(focal_plane_dist)
		{
			pixel_size = sensor_height / image_height;
			image_width = static_cast<size_t>(std::floor(sensor_width / pixel_size));
			aspect_ratio = (1.0f * image_width) / image_height;
			valid_sensor_width = image_width * pixel_size;  // a bit smaller than sensor_width
			
			aperture_d = focal_length / aperture_F_num;

			sensor_plane_dist = 1 / (1 / focal_length - 1 / focal_plane_dist);

			fov_y = std::atan(sensor_height / sensor_plane_dist / 2) * 2;
			fov_x = std::atan(sensor_width / sensor_plane_dist / 2) * 2;
		}

		Ray generate_ray(const Vector2d& pixel_coord) const override
		{
			// Randomly sample a point inside the pixel
			double Px = pixel_coord.x() + 0.5 * Random::rand_uniform();
			double Py = pixel_coord.y() + 0.5 * Random::rand_uniform();

			// Transform the image origin to the image center (where camera looks at) to make computation easier.
			Px -= image_width / 2.0f;
			Py -= image_height / 2.0f;

			// The corresponding point on focal plane (the desired object plane)
			double Vx = Px / image_width * (sensor_width * focal_plane_dist / sensor_plane_dist);
			double Vy = Py / image_height * (sensor_height * focal_plane_dist / sensor_plane_dist);

			// Sample a offset point on lens
			Vector2d rand_p = aperture_d / 2 * rand_vec2_in_unit_circle();  // [-r, r]
			Vector3d Lp = rand_p.x() * cam_x + rand_p.y() * cam_y;

			// All unit in mm, and normalization removes the scale
			Vector3d dir = (Vx * cam_x + Vy * cam_y + focal_plane_dist * direction - Lp).normalized();
			
			// Now have to scale to unit in cm, to be consistent with the object position
			Vector3d Lp_cm = Lp / 10;
			return Ray(eye + Lp_cm, dir);
		}

		void print() const override
		{
			cout << "======== Thin Lens Camera ========" << endl;
			cout << "eye: " << eye << endl;
			cout << "lookat: " << lookat << endl;
			cout << "up: " << up << endl;
			cout << endl;
			cout << "cam_x: " << cam_x << endl;
			cout << "cam_y: " << cam_y << endl;
			cout << "cam_z: " << cam_z << endl;
			cout << endl;
			cout << "sensor size: " << valid_sensor_width << "mm x " << sensor_height << "mm" << endl;
			cout << "focal_length: " << focal_length << "mm" << endl;
			cout << "aperture: F" << aperture_F_num << endl;
			cout << "aperture diameter: " << aperture_d << "mm" << endl;
			cout << endl;
			cout << "fov_x: " << fov_x << " (" << get_angle(fov_x) << " degrees)" << endl;
			cout << "fov_y: " << fov_y << " (" << get_angle(fov_y) << " degrees)" << endl;
			cout << endl;
			cout << "pixel_size: " << pixel_size << "mm" << endl;
			cout << "focal_plane_dist: " << focal_plane_dist << "mm" << endl;
			cout << "sensor_plane_dist: " << sensor_plane_dist << "mm" << endl;
			cout << endl;
			cout << "image size: " << image_width << " x " << image_height << endl;
			cout << "aspect_ratio: " << aspect_ratio << endl;
			cout << "================================" << endl;
		}

	private:
		/* Lens properties */
		double focal_length;
		double aperture_F_num;  // the F-number
		double aperture_d;  // the diameter of lens
		double focal_plane_dist;  // in unit mm; the desired depth of object

		/* Sensor properties */
		double sensor_width;  // in mm
		double valid_sensor_width;  // a bit of sensor width will be cropped to align the integer resolution
		double sensor_height;  // in mm
		double pixel_size;
		double sensor_plane_dist;

		// The image's properties
		double fov_y;  // in radian
		double fov_x;  // in radian
	};

}
