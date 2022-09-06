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

![cornell_box](C:\Users\yhj\Desktop\SoftRenderer\results\cornell_box.png)

### Thin-lens camera model

Shoot with 36mm x 24mm full frame, focal length 26mm, aperture F1.6; focusing on the sphere in the middle:

<img src="C:\Users\yhj\Desktop\SoftRenderer\results\thinlen_focus_mid.png" alt="thinlen_focus_mid" style="zoom: 33%;" />

Now change focus to the right sphere:

<img src="C:\Users\yhj\Desktop\SoftRenderer\results\thinlen_focus_right.png" alt="thinlen_focus_right" style="zoom: 33%;" />

Change focus to the left sphere:

<img src="C:\Users\yhj\Desktop\SoftRenderer\results\thinlen_focus_left.png" alt="thinlen_focus_left" style="zoom:33%;" />