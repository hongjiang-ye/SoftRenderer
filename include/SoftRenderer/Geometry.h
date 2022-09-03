#pragma once
#include <cmath>
#include <vector>
#include <cassert>
#include <iostream>
#include <limits>
#include <omp.h>

#include "SoftRenderer/Math.h"


namespace SR
{
    /* Basic geometry utility functions */
    const double PI = 3.141592654;

    inline double get_radian(double angle)
    {
        return angle / 180 * PI;
    }

    inline double get_angle(double radiant)
    {
        return radiant / PI * 180;
    }

    double fresnel_term(double cos_theta, double ior_in, double ior_out)
    {
        // Schlick's approximation
        double R0 = std::pow((ior_in - ior_out) / (ior_in + ior_out), 2);
        return R0 + (1 - R0) * std::pow(1 - cos_theta, 5);
    }

    Vector3d reflect_dir(const Vector3d& in, const Vector3d& normal)
    {
        return in - 2 * dot(in, normal) * normal;
    }

    std::tuple<bool, Vector3d, double> refracted_dir(const Vector3d& in, const Vector3d& normal, double ior_in, double ior_out)
    {
        double cos_theta = dot(-in, normal);
        double ior_ratio = ior_in / ior_out;

        double reflectance_p = fresnel_term(cos_theta, ior_in, ior_out);
        
        bool valid = true;
        if (ior_ratio * std::sqrt(1 - cos_theta * cos_theta) > 1) {  
            // Total internal reflection
            valid = false;
        }

        Vector3d t_perp = std::sqrt(1 - ior_ratio * ior_ratio * (1 - cos_theta * cos_theta)) * (-normal);
        Vector3d t_hori = ior_ratio * (in + cos_theta * normal);

        return std::make_tuple(valid, t_perp + t_hori, reflectance_p);
    }

    /* Geometry objects */

    // Forward declaration
    class Material;

    class Ray {
    public:

        // Record information about an intersection of a ray with a 3D primitive
        struct HitRecord {

            HitRecord() : t(DOUBLE_INFINITY), is_front_face(true), normal(), material_ptr(nullptr), texture_coord() {}

            double t;

            // Whether intersect with the front face of the object
            bool is_front_face;

            // The normal vector on the intersection point (always pointing against the ray direction).
            Vector3d normal;

            std::shared_ptr<Material> material_ptr;
            Vector2d texture_coord;
        };

        // Don't default construct a ray.
        Ray() = delete;

        Ray(const Vector3d& o, const Vector3d& d) 
            : origin(o), direction(d.normalized()), hit() {}

        Ray(const Ray& r) : origin(r.origin), direction(r.direction), hit(r.hit) {}

        Vector3d at(double t) const
        {
            return origin + t * direction;
        }

        friend std::ostream& operator<<(std::ostream& os, const Ray& r)
        {
            os << "Ray: Origin (" << r.origin.x() << ", " << r.origin.y() << ", " << r.origin.z() << ") "
                << "Direction (" << r.direction.x() << ", " << r.direction.y() << ", "
                << r.direction.z() << ") " << std::endl;
            return os;
        }

        /* Member functions to manage the hit record */

        Vector3d get_hit_point() const
        {
            // Float a little bit out of the surface.
            return this->at(hit.t);
        }

        Vector3d get_hit_normal() const
        {
            return hit.normal;
        }

        void set_hit_normal(Vector3d outward_normal)
        {
            outward_normal.normalize();

            // Set hit record's normal vector according to the ray direction and outward normal.
            hit.is_front_face = dot(direction, outward_normal) < 0;
            hit.normal = hit.is_front_face ? outward_normal : -outward_normal;
        }

        bool is_hit_front_face() const
        {
            return hit.is_front_face;
        }

        void set_hit_t(double t)
        {
            hit.t = t;
        }

        double get_hit_t() const
        {
            return hit.t;
        }

        void set_hit_material(std::shared_ptr<Material> material_ptr)
        {
            hit.material_ptr = material_ptr;
        }

        std::shared_ptr<Material> get_hit_material() const
        {
            return hit.material_ptr;
        }

        const Vector3d origin;
        const Vector3d direction;

    private:
        HitRecord hit;
    };


    // Base class for 3D primitives
    class Object3D {
    public:

        Object3D() = delete;
        Object3D(std::shared_ptr<Material> material_ptr) : material_ptr(material_ptr) {}

        // Whether this object will intersect with ray in range [tmin, ray.hit.t]; if so, store the 
        // intersection informatino in HitRecord.
        virtual bool intersect(Ray& ray, double tmin) = 0;

    protected:
        std::shared_ptr<Material> material_ptr;
    };


    class Sphere : public Object3D {
    public:

        Sphere() = delete;
        Sphere(const Vector3d& center, double radius, std::shared_ptr<Material> material_ptr)
            : Object3D(material_ptr), center(center), radius(radius) {}

        bool intersect(Ray& ray, double tmin) override
        {
            Vector3d oc = ray.origin - center;
            double a = dot(ray.direction, ray.direction);
            double b = dot(2.0 * oc, ray.direction);
            double c = dot(oc, oc) - radius * radius;

            double delta = b * b - 4 * a * c;

            if (delta < 0) {
                return false;
            }

            double sqrt_delta = std::sqrt(delta);

            double t1 = (-b - sqrt_delta) / (2 * a);
            double t2 = (-b + sqrt_delta) / (2 * a);

            // We must have t1 < t2
            double t_intersect = t1;
            if (!(t1 > tmin && t1 < ray.get_hit_t())) {
                t_intersect = t2;
                if (!(t2 > tmin && t2 < ray.get_hit_t())) {
                    // Both roots are not in the range [tmin, hit.t]
                    return false;
                }
            }

            // Has valid intersection
            ray.set_hit_t(t_intersect);
            ray.set_hit_normal((ray.get_hit_point() - center) / radius);
            //ray.set_hit_normal((ray.get_hit_point() - center).normalized());
            ray.set_hit_material(this->material_ptr);

            return true;
        }

        Vector3d center;
        double radius;
    };

}
