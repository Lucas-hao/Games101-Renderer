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

Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f L_dir = {0, 0, 0}, L_indir = {0, 0, 0};
    Intersection intersection = Scene::intersect(ray);
    if (!intersection.happened)
        return {};
    if (intersection.m->hasEmission())
        return intersection.m->getEmission();

    Intersection light_pos;
    float light_pdf = 0.0f;
    sampleLight(light_pos, light_pdf);
    Vector3f collision_light = light_pos.coords - intersection.coords;
    float dis = dotProduct(collision_light, collision_light);
    Vector3f collision_light_dir = collision_light.normalized();
    Ray light_to_object_ray(intersection.coords, collision_light_dir);
    Intersection light_ray_inter = Scene::intersect(light_to_object_ray);
    auto f_r = intersection.m -> eval(ray.direction, collision_light_dir, intersection.normal);
    if (light_ray_inter.distance - collision_light.norm() > -0.005){
        L_dir = light_pos.emit * f_r * dotProduct(collision_light_dir, intersection.normal)
                * dotProduct(-collision_light_dir, light_pos.normal) / dis / light_pdf;
    }

    if (get_random_float() > RussianRoulette)
        return L_dir;

    Vector3f w0 = intersection.m -> sample(ray.direction, intersection.normal).normalized();
    Ray object_to_object_ray(intersection.coords, w0);
    Intersection islight = Scene::intersect(object_to_object_ray);
    if (islight.happened && !islight.m->hasEmission())
    {
        float pdf = intersection.m->pdf(ray.direction, w0, intersection.normal);
        f_r = intersection.m->eval(ray.direction, w0, intersection.normal);
        L_indir = castRay(object_to_object_ray, depth + 1) * f_r * dotProduct(w0, intersection.normal) / pdf / RussianRoulette;
    }
    return L_dir + L_indir;
}