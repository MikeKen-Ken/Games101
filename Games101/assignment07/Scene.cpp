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

    Vector3f l_dir = {0., 0., 0.};
    Vector3f l_indir = {0., 0., 0.};

    // p点
    Intersection pInter = intersect(ray);

    if (pInter.happened)
    {
        if (pInter.m->hasEmission())
        {
            // 如果间接光照照射到了光源，
            if (depth > 0)
            {
                return {0., 0., 0.};
            }
            return {1., 1., 1.};
        }
        //光线和light的交点，初始化lightInter，并且拿到pdf
        Intersection lightInter;
        float lightPdf = 0.0f;
        sampleLight(lightInter, lightPdf);

        //p到light方向
        Vector3f ws = lightInter.coords - pInter.coords;

        //从p到light发射一条射线，如果碰到物体的距离，和发射点到光源的距离相同，即可以判断射线可以直接连接到光源
        Ray ray1(pInter.coords, ws.normalized());
        Intersection intersection = Scene::intersect(ray1);

        if (intersection.happened && (intersection.coords - lightInter.coords).norm() < 0.01)
        {
            //* 直接光照
            Vector3f f_r = pInter.m->eval(ray.direction, ray1.direction, pInter.normal);
            // 防止产生过亮的异常点
            if (lightPdf <= EPSILON)
            {
                lightPdf = EPSILON;
            }
            l_dir = lightInter.emit * f_r * dotProduct(ray1.direction, pInter.normal) * dotProduct(-ray1.direction, lightInter.normal) / (intersection.distance * intersection.distance) / lightPdf;
        }

        //* 间接光照
        if (get_random_float() < RussianRoulette)
        {
            Vector3f wi = pInter.m->sample(ray.direction, pInter.normal).normalized();

            Ray ray2(pInter.coords, wi);
            Intersection intersection2 = Scene::intersect(ray2);

            if (intersection2.happened && !intersection2.m->hasEmission())
            {
                Vector3f f_r1 = pInter.m->eval(ray.direction, wi, pInter.normal);
                float pdf1 = pInter.m->pdf(ray.direction, wi, pInter.normal);
                if (pdf1 <= EPSILON)
                {
                    pdf1 = EPSILON;
                }
                l_indir = castRay(ray2, depth + 1) * f_r1 * dotProduct(wi, pInter.normal) / pdf1 / RussianRoulette;
            }
        }
    }
    return l_dir + l_indir;
}
