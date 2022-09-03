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

        //void gaussian_blur()
        //{

        //}

        // Save the image in .PPM format.
        void save(std::string filename) const
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
                        
                        //double val = std::clamp(this->get_pixel(i, j).at(k), 0.0, 1.0);
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