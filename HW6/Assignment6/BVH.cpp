#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == BVHAccel::SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else
        root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds()); //读取每个三角形的包围盒并逐一合并成大包围盒
    // 只有一个物体，则该物体就是叶子节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    // 有两个物体，则这两个物体是根节点的左右叶子节点
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }

    else {
        Bounds3 centroidBounds;
        // 将每个物体的包围盒的中心作为顶点构建大包围盒
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();//判断哪个是最长轴(1-x,2-y,3-z)
        // 根据最长轴，将物体按照对应轴坐标从小到大排序
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        // 将排序好物体对半分开，分别作为左右子节点，并各自递归构建子节点
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects){
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds()); //读取每个三角形的包围盒并逐一合并成大包围盒
    // 只有一个物体，则该物体就是叶子节点
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    // 有两个物体，则这两个物体是根节点的左右叶子节点
    else if (objects.size() == 2) {
        node->left = recursiveBuildSAH(std::vector{objects[0]});
        node->right = recursiveBuildSAH(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else{
        auto beginning = objects.begin();
        auto ending = objects.end();
        if(objects.size() < 12){
            auto middling = objects.begin() +objects.size() / 2;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveBuildSAH(leftshapes);
            node->right = recursiveBuildSAH(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else{
            Bounds3 centroidBounds;
            // 将每个物体的包围盒的中心作为顶点构建大包围盒
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();//判断哪个是最长轴(1-x,2-y,3-z)
            // 根据最长轴，将物体按照对应轴坐标从小到大排序
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
            }

            auto size = objects.size();
            int splitPosNum = 10;
            int finalSplitPos = 0;
            double mincost = __DBL_MAX__;
            for(int i = 1; i < splitPosNum; ++i)
            {
                auto middling = objects.begin() + size * i / splitPosNum;
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);

                assert(objects.size() == leftshapes.size() + rightshapes.size());

                double leftSA = computeSurfaceArea(leftshapes);
                double rightSA = computeSurfaceArea(rightshapes);
                double S = leftSA + rightSA;
                auto cost = leftSA / S * leftshapes.size() + rightSA / S * rightshapes.size();
                if(cost < mincost){
                    mincost = cost;
                    finalSplitPos = i;
                }
            }

            auto middling = objects.begin() + size * finalSplitPos / splitPosNum;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == leftshapes.size() + rightshapes.size());

            node->left = recursiveBuildSAH(leftshapes);
            node->right = recursiveBuildSAH(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        return node;
    }
}

double BVHAccel::computeSurfaceArea(std::vector<Object*> objects){
    Bounds3 centroidBounds;
    for(int i = 0; i < objects.size(); ++i){
        centroidBounds =  Union(centroidBounds, objects[i]->getBounds().Centroid());
    }
    return centroidBounds.SurfaceArea();
}


Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter, interL, interR;
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = (int)(ray.direction.x < 0);
    dirIsNeg[1] = (int)(ray.direction.y < 0);
    dirIsNeg[2] = (int)(ray.direction.z < 0);

    // 光线与当前包围盒没有交点
    if(!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return inter;
    // 光线与当前包围盒有交点且包围盒是叶子节点
    else if(node->left == nullptr && node->right == nullptr){
        inter = node->object->getIntersection(ray);
        return inter;
    }
    // 光线与包围盒有交点且包围盒是中间节点
    else{
        interL = getIntersection(node->left, ray);
        interR = getIntersection(node->right, ray);
        return interL.distance < interR.distance ? interL : interR;
    }
}