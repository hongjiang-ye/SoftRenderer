#pragma once
#include <cassert>
#include <chrono>
#include <fstream>
#include <vector>
#include <algorithm>
#include <memory>

#include "SoftRenderer/Math.h"


namespace SR
{
    using Color = Vector3d;

    // Canvas coordinate: origin is the bottom left corner; 
    // x is horizontal direction, and y is vertical direction 
    class Image {
    public:
        Image(size_t w, size_t h) : width(w), height(h), pixels(w * h) 
        {
            aspect_ratio = static_cast<double>(width) / height;
        }

        size_t get_width() const
        {
            return width;
        }

        size_t get_height() const
        {
            return height;
        }

        double get_aspect_ratio() const {
            return aspect_ratio;
        }

        const Color& get_pixel(size_t x, size_t y) const
        {   
            assert(x < width && y < height);
            return pixels[this->get_index_of(x, y)];
        }

        void set_all_pixels(const Color& color)
        {
            for (size_t i = 0; i < width * height; ++i) {
                pixels[i] = color;
            }
        }

        void set_pixel(size_t x, size_t y, const Color& color)
        {
            assert(x < width && y < height);
            pixels[this->get_index_of(x, y)] = color;
        }

        std::shared_ptr<Image> downsample(size_t factor) const
        {
            assert(width % factor == 0 || height % factor == 0);
            size_t downsample_width = width / factor;
            size_t downsample_height = height / factor;

            std::shared_ptr<Image> downsampled_image_ptr = std::make_shared<Image>(
                downsample_width, downsample_height);

            for (size_t i = 0; i < downsample_width; i++) {
                for (size_t j = 0; j < downsample_height; j++) {
                    
                    Color color = {0, 0, 0};

                    for (size_t m = i * factor; m < (i + 1) * factor; m++) {
                        for (size_t n = j * factor; n < (j + 1) * factor; n++) {
                            color += this->get_pixel(m, n);
                        }
                    }

                    downsampled_image_ptr->set_pixel(i, j, color / std::pow(static_cast<double>(factor), 2));
                }
            }

            return downsampled_image_ptr;
        }

        void gaussian_blur()
        {
            // 3x3 kernel
            const double K[5] = { 0.1201, 0.2339, 0.2931, 0.2339, 0.1201 };

            Image hori_blurred_image(*this);

            int i_width = static_cast<int>(width);
            int i_height = static_cast<int>(height);

            for (int i = 0; i < i_width; i++) {
                for (int j = 0; j < i_height; j++) {
                    Color color;
                    color += K[0] * this->get_pixel(i, std::max(0, j - 2));
                    color += K[1] * this->get_pixel(i, std::max(0, j - 1));
                    color += K[2] * this->get_pixel(i, j);
                    color += K[3] * this->get_pixel(i, std::min(i_height - 1, j + 1));
                    color += K[4] * this->get_pixel(i, std::min(i_height - 1, j + 2));
                    hori_blurred_image.set_pixel(i, j, color);
                }
            }

            Image vert_blurred_image(hori_blurred_image);

            for (int i = 0; i < i_width; i++) {
                for (int j = 0; j < i_height; j++) {
                    Color color;
                    color += K[0] * hori_blurred_image.get_pixel(std::max(0, i - 2), j);
                    color += K[1] * hori_blurred_image.get_pixel(std::max(0, i - 1), j);
                    color += K[2] * hori_blurred_image.get_pixel(i, j);
                    color += K[3] * hori_blurred_image.get_pixel(std::min(i_width - 1, i + 1), j);
                    color += K[4] * hori_blurred_image.get_pixel(std::min(i_width - 1, i + 2), j);
                    vert_blurred_image.set_pixel(i, j, color);
                }
            }

            *this = vert_blurred_image;
        }

        static std::shared_ptr<Image> load_ppm(std::string filename)
        {
            assert(filename.substr(filename.size() - 4, 4) == ".ppm");

            std::ifstream file1(filename, std::ios::binary);

            // Process ascii header
            size_t width = 0;
            size_t height = 0;

            std::string line;
            std::getline(file1, line);
            assert(line == "P6");

            std::getline(file1, line);

            std::istringstream iss(line);
            iss >> width >> height;

            std::getline(file1, line);
            assert(line == "255");
            
            // Read pixel colors
            std::shared_ptr<Image> image_ptr = std::make_shared<Image>(width, height);

            for (int y = static_cast<int>(height) - 1; y >= 0; y--) {
                for (int x = 0; x < static_cast<int>(width); x++) {
                    char r, g, b;
                    file1.get(r);
                    file1.get(g);
                    file1.get(b);
                    
                    Color color{ reinterpret_cast<unsigned char&>(r) / 255.0,
                        reinterpret_cast<unsigned char&>(g) / 255.0, 
                        reinterpret_cast<unsigned char&>(b) / 255.0};
                    image_ptr->set_pixel(x, y, color);
                }
            }

            file1.close();

            return image_ptr;
        }

        // Save the image in .PPM format.
        void save_ppm(std::string filename) const
        {   
            std::cout << "Writing image to file " << filename << "..." << std::endl;
            auto t_start = std::chrono::steady_clock::now();

            // Use P6 binary mode for faster output
            std::ofstream ofs(filename, std::ofstream::binary);
            ofs << "P6\n" << width << " " << height << "\n255\n";

            // The coordinate of image format has origin at top left corner. So filp the y axis.
            for (int j = static_cast<int>(height) - 1; j >= 0; j--) {
                for (size_t i = 0; i < width; i++) {
                    for (size_t k = 0; k < 3; k++) {
                        // with gamma correction
                        double val = std::clamp(std::sqrt(this->get_pixel(i, j).at(k)), 0.0, 1.0);
                        
                        ofs << static_cast<char>(255 * val);
                    }
                }
            }
            ofs.close();

            //// Use P3 ASCII mode to facilitate debugging
            //std::ofstream ofs(filename);
            //ofs << "P3\n" << width << " " << height << "\n255\n";

            //// The coordinate of image format has origin at top left corner. So filp the y axis.
            //for (int j = static_cast<int>(height) - 1; j >= 0; j--) {
            //    for (size_t i = 0; i < width; i++) {
            //        for (size_t k = 0; k < 3; k++) {
            //            double val = std::clamp(this->get_pixel(i, j)[k], 0.0f, 1.0f);
            //            ofs << static_cast<short>(std::round(255 * val)) << ' ';
            //        }
            //        ofs << '\n';
            //    }
            //}
            //ofs.close();

            auto t_end = std::chrono::steady_clock::now();
            std::cout << "Image writing is done in "
                << std::chrono::duration<double>(t_end - t_start).count()
                << " seconds." << std::endl;
        }

    private:

        // Get the index in data array of pixel (x, y)
        size_t get_index_of(size_t x, size_t y) const
        {
            // Data is stored in row major
            return y * width + x;
        }

        size_t width;
        size_t height;
        double aspect_ratio;
        std::vector<Color> pixels;
    };

}
