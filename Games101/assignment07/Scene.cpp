//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    // p点
    Intersection pInter = intersect(ray);

    if (pInter.happened)
    {

        //light点
        Intersection lightInter;
        float lightPdf = 0.0f;
        sampleLight(lightInter, lightPdf);

        //p到light
        Vector3f ws = lightInter.coords - pInter.coords;

        //从p到light发射一条射线，如果碰到物体的距离，和发射点到光源的距离相同，即可以判断射线可以直接连接到光源
        Ray ray1(pInter.coords, ws.normalized());
        Intersection intersection = Scene::intersect(ray1);
        Vector3f l_dir = {0., 0., 0.};

        if (intersection.happened && (intersection.coords - lightInter.coords).norm() < 0.0001)
        {
            //直接光照
            Vector3f f_r = pInter.m->eval(ray.direction, ws.normalized(), pInter.normal);
            l_dir = lightInter.emit * f_r * dotProduct(ray1.direction, pInter.normal) * dotProduct(-ray1.direction, lightInter.normal) / (intersection.distance * intersection.distance) / lightPdf;
        }
        // std::cout << l_dir;
        return l_dir;
        // Vector3f lindir = {0., 0., 0.};
    }
    return {0., 0., 0.};
}