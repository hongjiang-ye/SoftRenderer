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
    /* Basic geometry utilities */
    using Point3 = Vector3d;
    using Point2 = Vector2d;
    using Vector3 = Vector3d;
    using Vector2 = Vector2d;

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

    // Compute the reflection direction
    Vector3 reflect_dir(const Vector3& in, const Vector3& normal)
    {
        return in - 2 * dot(in, normal) * normal;
    }

    // Return information about refraction
    // tuple[0]: whether a refracted direction is sampled
    // tuple[1]: the refracted direction
    // tuple[2]: the probability of reflection (1 - p_refraction)
    std::tuple<bool, Vector3, double> refracted_dir(const Vector3& in, const Vector3& normal, double ior_in, double ior_out)
    {
        double cos_theta = dot(-in, normal);
        double ior_ratio = ior_in / ior_out;

        double reflectance_p = fresnel_term(cos_theta, ior_in, ior_out);
        
        bool valid = true;
        if (ior_ratio * std::sqrt(1 - cos_theta * cos_theta) > 1) {  
            // Total internal reflection
            valid = false;
        }

        Vector3 t_perp = std::sqrt(1 - ior_ratio * ior_ratio * (1 - cos_theta * cos_theta)) * (-normal);
        Vector3 t_hori = ior_ratio * (in + cos_theta * normal);

        return std::make_tuple(valid, t_perp + t_hori, reflectance_p);
    }

    /* Geometry objects */

    // Forward declaration
    struct Material;

    class Ray {
    public:

        // Record information about an intersection of a ray with a 3D primitive
        struct HitRecord {

            HitRecord() : t(DOUBLE_POS_INFINITY), is_front_face(true), normal(), material_ptr(nullptr), texture_coord() {}

            double t;

            // Whether intersect with the front face of the object
            bool is_front_face;

            // The normal vector on the intersection point (always pointing against the ray direction).
            Vector3 normal;

            std::shared_ptr<Material> material_ptr;
            Point2 texture_coord;
        };

        // Don't default construct a ray.
        Ray() = delete;

        Ray(const Point3& o, const Vector3& d)
            : origin(o), direction(d.normalized()), hit() {}

        Ray(const Ray& r) : origin(r.origin), direction(r.direction), hit(r.hit) {}

        Point3 at(double t) const
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

        Point3 get_hit_point() const
        {
            // Float a little bit out of the surface.
            return this->at(hit.t);
        }

        Vector3 get_hit_normal() const
        {
            return hit.normal;
        }

        void set_hit_normal(Vector3 outward_normal)
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

        void set_hit_textcoord(Point2 uv)
        {
            hit.texture_coord = uv;
        }

        const Point2 get_hit_textcoord() const
        {
            return hit.texture_coord;
        }

        const Point3 origin;
        const Vector3 direction;

    private:
        HitRecord hit;
    };


    // TODO: Make an abstract BoundingVolume class;
    struct AABoundingBox {

        AABoundingBox()
        {
            // Initialized to be an invalid bbox;
            //min_point = { DOUBLE_POS_INFINITY, DOUBLE_POS_INFINITY, DOUBLE_POS_INFINITY };
            //max_point = { DOUBLE_NEG_INFINITY, DOUBLE_NEG_INFINITY, DOUBLE_NEG_INFINITY };

            max_point = { DOUBLE_POS_INFINITY, DOUBLE_POS_INFINITY, DOUBLE_POS_INFINITY };
            min_point = { DOUBLE_NEG_INFINITY, DOUBLE_NEG_INFINITY, DOUBLE_NEG_INFINITY };
        }

        AABoundingBox(const Point3& min_point, const Point3& max_point)
            : min_point(min_point), max_point(max_point)
        {}

        bool intersect(const Ray& ray) const
        {
            double t_min = DOUBLE_NEG_INFINITY;
            double t_max = DOUBLE_POS_INFINITY;

            for (size_t i = 0; i < 3; i++) {
                double t0 = (min_point.at(i) - ray.origin.at(i)) / ray.direction.at(i);
                double t1 = (max_point.at(i) - ray.origin.at(i)) / ray.direction.at(i);

                t_min = std::max(t_min, std::min(t0, t1));
                t_max = std::min(t_max, std::max(t0, t1));

                // If ray.direction.at(i) == 0:
                // case 1: t0 == t1 == +-inf, t_min >= t_max must be false;
                // case 2: t0 == -inf && t0 == +inf, then t_min and t_max remains the same, also must be true
                // so this procedure considers all cases.
                if (t_min >= t_max) {
                    return false;
                }
            }
            return true;
        }

        // Compute the union of two aabbs.
        static AABoundingBox merge(const AABoundingBox& lhs_aabb, const AABoundingBox& rhs_aabb)
        {
            AABoundingBox res(lhs_aabb);
            res.merge(rhs_aabb);
            return res;
        }

        void merge(const AABoundingBox& aabb)
        {
            min_point.x() = std::min(min_point.x(), aabb.min_point.x());
            min_point.y() = std::min(min_point.y(), aabb.min_point.y());
            min_point.z() = std::min(min_point.z(), aabb.min_point.z());

            max_point.x() = std::max(max_point.x(), aabb.max_point.x());
            max_point.y() = std::max(max_point.y(), aabb.max_point.y());
            max_point.z() = std::max(max_point.z(), aabb.max_point.z());
        }

        friend std::ostream& operator<<(std::ostream& os, const AABoundingBox& aabb)
        {
            os << "AABB: bottom_left " << aabb.min_point << " top_right " << aabb.max_point << endl;
            return os;
        }

        Point3 min_point;
        Point3 max_point;
    };


    // TODO: Make an abstract Renderable class that contains a object3d object;
    // Base class for 3D primitives
    struct Object3D {

        Object3D() = delete;
        Object3D(std::shared_ptr<Material> material_ptr) : material_ptr(material_ptr), aabb() {}

        // Whether this object will intersect with ray in range [tmin, ray.hit.t]; if so, store the 
        // intersection informatino in HitRecord.
        virtual bool intersect(Ray& ray, double tmin) = 0;

        std::shared_ptr<Material> material_ptr;
        AABoundingBox aabb;
    };


    class BVH_Node {
    public:

        BVH_Node() : left(nullptr), right(nullptr), object_ptr(nullptr), aabb() {}

        // Use the i-th axis (0 for x, 1 for y, 2 for z) to sort and then split the objects.
        BVH_Node(std::vector<std::shared_ptr<Object3D>>& object_ptrs, size_t start, size_t end, size_t axis_i) : BVH_Node()
        {
            if (end - start == 1) {  // base case
                object_ptr = object_ptrs[start];
                aabb = object_ptr->aabb;
                //cout << aabb << endl;
            }
            else {  // recursive case
                std::sort(object_ptrs.begin() + start, object_ptrs.begin() + end,
                    [&](const std::shared_ptr<Object3D>& lhs, const std::shared_ptr<Object3D>& rhs) {
                    return lhs->aabb.min_point.at(axis_i) < rhs->aabb.min_point.at(axis_i);
                });

                // Alternatively choose the next axis
                size_t next_axis_i = (axis_i + 1) % 3;
                size_t mid = (start + end) / 2;

                left = std::make_shared<BVH_Node>(object_ptrs, start, mid, next_axis_i);
                right = std::make_shared<BVH_Node>(object_ptrs, mid, end, next_axis_i);

                aabb = AABoundingBox::merge(left->aabb, right->aabb);
                //cout << aabb << endl;
            }
        }

        bool intersect(Ray& ray, double tmin) const
        {
            if (!aabb.intersect(ray)) return false;

            if (object_ptr) {  // leaf
                return object_ptr->intersect(ray, tmin);
            }
            else {  // non-leaf
                bool intersected = false;
                if (left->intersect(ray, tmin)) {
                    intersected = true;
                }
                if (right->intersect(ray, tmin)) {
                    intersected = true;
                }
                return intersected;
            }
        }

    private:
        AABoundingBox aabb;

        // For non-leaf node
        std::shared_ptr<BVH_Node> left;
        std::shared_ptr<BVH_Node> right;

        // For leaf node
        std::shared_ptr<Object3D> object_ptr;
    };


    class Sphere : public Object3D {
    public:

        Sphere() = delete;
        Sphere(const Point3& center, double radius, std::shared_ptr<Material> material_ptr)
            : Object3D(material_ptr), center(center), radius(radius) 
        {
            aabb.min_point = center - Vector3d{ radius, radius, radius };
            aabb.max_point = center + Vector3d{ radius, radius, radius };
        }

        // Compute the texture coordinate based on the hit point on the surface
        Point2 get_textcoord(Point3 point)
        {
            Vector3 unit_dir = (point - center).normalized();

            double theta = std::acos(-unit_dir.y());
            double phi = std::atan2(-unit_dir.z(), unit_dir.x()) + PI;

            return Point2{ phi / (2 * PI), theta / PI };  // map to [0, 1]
        }

        bool intersect(Ray& ray, double tmin) override
        {
            Vector3 oc = ray.origin - center;
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
            ray.set_hit_normal((ray.get_hit_point() - center).normalized());
            ray.set_hit_material(material_ptr);
            ray.set_hit_textcoord(this->get_textcoord(ray.get_hit_point()));


            return true;
        }

        Point3 center;
        double radius;
    };


    class Triangle : public Object3D {
    public:

        Triangle() = delete;

        // With counterclock-wise winding (a->b->c), which defines the front face (direction of normal).
        Triangle(const Point3& a, const Point3& b, const Point3& c, std::shared_ptr<Material> material_ptr) 
            : a(a), b(b), c(c), Object3D(material_ptr)
        {
            aabb.min_point.x() = std::min({ a.x(), b.x(), c.x() }) - 0.001;
            aabb.min_point.y() = std::min({ a.y(), b.y(), c.y() }) - 0.001;
            aabb.min_point.z() = std::min({ a.z(), b.z(), c.z() }) + 0.001;
            
            aabb.max_point.x() = std::max({ a.x(), b.x(), c.x() }) + 0.001;
            aabb.max_point.y() = std::max({ a.y(), b.y(), c.y() }) + 0.001;
            aabb.max_point.z() = std::max({ a.z(), b.z(), c.z() }) + 0.001;

            //cout << "Tri AABB: " << aabb << endl;
            normal = cross(b - a, c - a).normalized();
        }
            

        // Compute the texture coordinate based on the hit point on the surface
        Point2 get_textcoord(Point3 point)
        {
            return Point2();
        }

        bool intersect(Ray& ray, double tmin) override
        {
            double t = (b - ray.origin).dot(normal) / ray.direction.dot(normal);
            Point3 p = ray.at(t);

            // Determine if p is in the triangle
            if (cross(b - a, p - a).dot(cross(b - a, c - a)) < 0) return false;  // ab, ap, ac
            if (cross(c - a, p - a).dot(cross(c - a, b - a)) < 0) return false;  // ac, ap, ab
            if (cross(c - b, p - b).dot(cross(c - b, a - b)) < 0) return false;  // bc, bp, ba

            if (!(t > tmin && t < ray.get_hit_t())) {
                return false;
            }

            // Has valid intersection
            ray.set_hit_t(t);
            ray.set_hit_normal(normal);
            ray.set_hit_material(material_ptr);
            ray.set_hit_textcoord(this->get_textcoord(ray.get_hit_point()));

            return true;
        }

        Point3 a;
        Point3 b;
        Point3 c;

        Vector3 normal;
    };

    //// Consists of a list of triangles
    //class Mesh : public Object3D {
    //    Mesh() = delete;

    //    // With counterclock-wise winding (a->b->c), which defines the front face (direction of normal).
    //    Mesh(std::vector<Triangle> triangles, std::shared_ptr<Material> material_ptr)
    //        : triangles(triangles), Object3D(material_ptr)
    //    {
    //        for (size_t i = 0; i < triangles.size(); i++) {
    //            aabb.merge(triangles[i].aabb);
    //        }
    //    }

    //    Point2 get_textcoord(Point3 point)
    //    {
    //        return Point2();
    //    }

    //    bool intersect(Ray& ray, double tmin) override
    //    {
    //        bool intersected = false;

    //        // Linear search all triangles
    //        for (size_t i = 0; i < triangles.size(); i++) {
    //            if (triangles[i].intersect(ray, tmin)) {
    //                intersected = true;
    //            }
    //        }

    //        return intersect;
    //    }

    //    std::vector<Triangle> triangles;
    //};


    // Consists of a list of triangles
    struct Rectangle : public Object3D {
        Rectangle() = delete;

        // a->b->c->d in counter-clock wise
        Rectangle(const Point3& a, const Point3& b, const Point3& c, const Point3& d, std::shared_ptr<Material> material_ptr)
            : Object3D(material_ptr), a(a), b(b), c(c), d(d)
        {
            aabb.min_point.x() = std::min({ a.x(), b.x(), c.x(), d.x() }) - 0.001;
            aabb.min_point.y() = std::min({ a.y(), b.y(), c.y(), d.y() }) - 0.001;
            aabb.min_point.z() = std::min({ a.z(), b.z(), c.z(), d.z() }) - 0.001;

            aabb.max_point.x() = std::max({ a.x(), b.x(), c.x(), d.x() }) + 0.001;
            aabb.max_point.y() = std::max({ a.y(), b.y(), c.y(), d.y() }) + 0.001;
            aabb.max_point.z() = std::max({ a.z(), b.z(), c.z(), d.z() }) + 0.001;

            normal = cross(b - d, c - a).normalized();
        }

        Point2 get_textcoord(Point3 point)
        {
            return Point2();
        }

        bool intersect(Ray& ray, double tmin) override
        {
            double t = (b - ray.origin).dot(normal) / ray.direction.dot(normal);
            Point3 p = ray.at(t);

            // Determine if p is in the rectangle (whether is between AB & CD, and AD & BC
            if (cross(b - a, p - a).dot(cross(p - d, c - d)) < 0) return false;  // (AB X AP) * (DP X DC) >= 0
            if (cross(a - d, p - d).dot(cross(p - c, b - c)) < 0) return false;  // (DA X DP) * (CP X CB) >= 0

            if (!(t > tmin && t < ray.get_hit_t())) {
                return false;
            }

            // Has valid intersection
            ray.set_hit_t(t);
            ray.set_hit_normal(normal);
            ray.set_hit_material(material_ptr);
            ray.set_hit_textcoord(this->get_textcoord(ray.get_hit_point()));

            return true;
        }

        Point3 a;
        Point3 b;
        Point3 c;
        Point3 d;

        Vector3 normal;
    };
}
