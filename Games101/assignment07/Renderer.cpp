//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
std::mutex mutex_ins;
// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    // change the spp value to change sample ammount
    int spp = 16;
    std::cout << "SPP: " << spp << "\n";

    int process = 0;
    // 创造匿名函数
    auto castRayMultiThreading = [&](uint32_t rowStart, uint32_t rowEnd, uint32_t colStart, uint32_t colEnd)
    {
        for (uint32_t j = rowStart; j < rowEnd; ++j)
        {
            int m = j * scene.width + colStart;
            for (uint32_t i = colStart; i < colEnd; ++i)
            {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++)
                {
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                m++;
                process++;
            }
            // 互斥锁，用于打印处理进程(也就是下面一行UpdateProgress)
            std::lock_guard<std::mutex> g1(mutex_ins);
            UpdateProgress(1.0 * process / scene.width / scene.height); // update the console progress
        }
    };

    //*分5*5的块计算光线追踪
    constexpr int bx = 5;
    constexpr int by = 5;
    std::thread th[bx * by];
    // each 块的大小
    int strideX = (scene.width + 1) / bx;
    int strideY = (scene.height + 1) / by;
    int index = 0;
    for (int i = 0; i < scene.height; i += strideX)
    {
        for (int j = 0; j < scene.width; j += strideY)
        {
            // 每个块建立一个thread
            th[index] = std::thread(castRayMultiThreading, i, std::min(i + strideX, scene.height), j, std::min(j + strideY, scene.width));
            index++;
        }
    }

    for (int i = 0; i < bx * by; i++)
        th[i].join();

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
