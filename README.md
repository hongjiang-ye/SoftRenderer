# SoftRenderer

A path tracer written in C++ with no third-party library support.

## Results



## Features

**Geometry**

* Supporting primitives:
  * Sphere
  * Triangular Mesh
* Camera model: 
  * Pinhole camera
  * Thin-lens camera

**Rendering**

* Path tracing:
  * Unbiased Monte Carlo estimation.
  * Antialiasing via super-sampling and Gaussian blur.

* Material (Scattering + BRDF):
  * Lambertian (perfect diffuse)
  * Metal (reflection)
  * Dielectric (refraction and reflection)
* Acceleration:
  * Naive BVH using AABB.

**Utilities**

* Image IO:

  * Image read & write in .PPM format.

* Math:

  * A matrix arithmetic library built with template, similar to Eigen.

    ```C++
    using Matrix3f = MatrixXf<3, 3>;
    using Vector3f = MatrixXf<3, 1>;
    Matrix3f mat = Matrix3f::Identity();
    Vector3f vec = {1, 2, 3};
    cout << mat * vec << endl;
    ```