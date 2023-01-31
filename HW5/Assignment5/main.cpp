#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960);

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);//设定球1的球心和半径
    sph1->materialType = DIFFUSE_AND_GLOSSY;//设定材质为DIFFUSE_AND_GLOSSY
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);//设定球2的球心和半径
    sph2->ior = 1.5;//设定折射率
    sph2->materialType = REFLECTION_AND_REFRACTION;//设定材质为REFLECTION_AND_REFRACTION

    scene.Add(std::move(sph1));//在场景中加入两个球体
    scene.Add(std::move(sph2));

    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};//设定地面范围顶点
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};//三角形顶点索引
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);//初始化三角形
    mesh->materialType = DIFFUSE_AND_GLOSSY;

    scene.Add(std::move(mesh));//将网格加入场景
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));//设定光源
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    Renderer r;
    r.Render(scene);

    return 0;
}