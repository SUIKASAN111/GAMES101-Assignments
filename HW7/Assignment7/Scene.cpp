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
    // 直接与间接光照
    Vector3f L_dir(0, 0, 0);
    Vector3f L_indir(0, 0, 0);

    // 与BVH求交
    Intersection interObj = intersect(ray);
    // 无交点
    if(!interObj.happened)
        return Vector3f();
    // 打到光源 若交点材质发光，则是光源，返回光源颜色
    if(interObj.m->hasEmission())
        return interObj.m->getEmission();

    // 直接光照
    Intersection interLight;
    float pdfLight; 
    sampleLight(interLight, pdfLight);

    Vector3f p2x = interLight.coords - interObj.coords;
    Vector3f p2xDir = p2x.normalized();
    float p2xPow = p2x.x * p2x.x + p2x.y * p2x.y + p2x.z * p2x.z;//|x-p|^2

    Ray p2xRay(interObj.coords, p2xDir);//ray from p to x
    Intersection inter = intersect(p2xRay);
    //float比较  当交点p与光源采样点x的求交距离和实际距离之差
    if(inter.distance - p2x.norm() > -EPSILON){
        L_dir =  interLight.emit * interObj.m->eval(ray.direction, p2xDir, interObj.normal)
                * dotProduct(p2xDir, interObj.normal) * dotProduct(-p2xDir, interLight.normal)
                / p2xPow / pdfLight;
    }
    // 间接光照
    if(get_random_float() < RussianRoulette){
        Vector3f p2nextpDir = interObj.m->sample(ray.direction, interObj.normal).normalized();
        Ray p2nextpRay(interObj.coords, p2nextpDir);
        Intersection interNextP = intersect(p2nextpRay);
        if(interNextP.happened && !interNextP.m->hasEmission()){
            float pdf = interNextP.m->pdf(ray.direction, p2nextpDir, interObj.normal);
            if(pdf > EPSILON){
                L_indir = castRay(p2nextpRay, depth + 1)
                    * interObj.m->eval(ray.direction, p2nextpDir, interObj.normal)
                    * dotProduct(p2nextpDir, interObj.normal) / pdf / RussianRoulette;
            }
        }
    }
    
    return L_dir + L_indir;
}