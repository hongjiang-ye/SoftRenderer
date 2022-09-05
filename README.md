# SoftRenderer

A path tracer written in C++ with no third-party library support.

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

## Results

### Path Tracing

The Cornell box:



### Thin-lens camera model

Shoot with 36mm x 24mm full frame, focal length 26mm, aperture F1.6; focusing on the sphere in the middle:

<img src="https://s3.bmp.ovh/imgs/2022/09/06/ff2d80358def9372.png" style="zoom: 50%;" />

Now change focus to the right sphere:

<img src="https://s3.bmp.ovh/imgs/2022/09/06/9ec67ec81d31c902.png" style="zoom: 50%;" />

Change focus to the left sphere:

<img src="https://s3.bmp.ovh/imgs/2022/09/06/4a745553fe195f0e.png" style="zoom: 50%;" />