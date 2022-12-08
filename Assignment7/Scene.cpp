//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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
    // TO DO Implement Path Tracing Algorithm here
    Intersection ipoint = intersect(ray);
    if (!ipoint.happened)
        return Vector3f(0.0f, 0.0f, 0.0f);
    if (ipoint.m->hasEmission())
        return ipoint.m->getEmission();

    //PDF Sample
    Intersection ipointx;
    float pdf_light = 1.0f;
    sampleLight(ipointx, pdf_light);
    Vector3f& p = ipoint.coords;
    Vector3f& x = ipointx.coords;

    Ray p2x(p, normalize(x - p));
    Intersection ip2x = intersect(p2x);

    Vector3f LightDirect(0.0f, 0.0f, 0.0f);
    //bounce ray hit light, we assume ipointx equals ip2x
    if ( (p-x).norm() - ip2x.distance < 0.1f) 
    {
        LightDirect = ipointx.emit * ipoint.m->eval(ray.direction, p2x.direction, ipoint.normal) * 
        	dotProduct(ipoint.normal, p2x.direction) * dotProduct(-p2x.direction, ipointx.normal) / (p.x * x.x + p.y * x.y + p.z * x.z) / (pdf_light + 0.00001);
    }

    float num = get_random_float();
    if (num > RussianRoulette)
        return LightDirect;
        
    Vector3f LightIndirect = {};
    Vector3f other_dir = ipoint.m->sample(ray.direction, ipoint.normal).normalized();
    Ray other_ray(p, other_dir);
    Intersection io = intersect(other_ray);
    if (io.happened && !io.m->hasEmission())
    {
        LightIndirect = ipoint.m->eval(ray.direction, other_dir, ipoint.normal) * castRay(other_ray, depth + 1) *
            dotProduct(other_dir, ipoint.normal) / ipoint.m->pdf(ray.direction, other_dir, ipoint.normal) / RussianRoulette;
    }

    return LightDirect + LightIndirect;
}

