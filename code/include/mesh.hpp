#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
#include "constant.hpp"

using namespace std;

// 每个图元的AABB包围盒
struct Pel {   
    Vector3f minxyz;   // 最小顶点
    Vector3f maxxyz;   // 最大顶点
    Vector3f centroid; // 中点,粗略的质心
    int index;
    Pel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int idx) {
        minxyz = Vector3f(minx, miny, minz);
        maxxyz = Vector3f(maxx, maxy, maxz);
        centroid = (minxyz + maxxyz) * 0.5;
        index = idx;
    }
};

float now_center;

// 一系列关于Pel的比较函数
bool Pel_min_x(Pel x, Pel y) {
    return x.minxyz.x() < y.minxyz.x();
}

bool Pel_min_y(Pel x, Pel y) {
    return x.minxyz.y() < y.minxyz.y();
}

bool Pel_min_z(Pel x, Pel y) {
    return x.minxyz.z() < y.minxyz.z();
}

bool Pel_max_x(Pel x, Pel y) {
    return x.maxxyz.x() < y.maxxyz.x();
}

bool Pel_max_y(Pel x, Pel y) {
    return x.maxxyz.y() < y.maxxyz.y();
}

bool Pel_max_z(Pel x, Pel y) {
    return x.maxxyz.z() < y.maxxyz.z();
}

bool Pel_cen_x(Pel x) {
    return x.centroid.x() < now_center;
}

bool Pel_cen_y(Pel x) {
    return x.centroid.y() < now_center;
}

bool Pel_cen_z(Pel x) {
    return x.centroid.z() < now_center;
}

// BVH树的节点, 储存父子关系以及这一节点的AABB包围盒数据
// 如果是叶子节点，储存包含的图元AABB包围盒（连续储存，因此储存首个Pel的迭代器就行）
struct BVH_TreeNode {
    BVH_TreeNode* parent;
    BVH_TreeNode* lc;
    BVH_TreeNode* rc;
    Vector3f minxyz;
    Vector3f maxxyz;
    int pel_num;
    vector<Pel>::iterator first_pel; //有这个的是叶子节点
    BVH_TreeNode(BVH_TreeNode* a, vector<Pel>::iterator it, int _num){
        // BVH_TreeNode这个结构体的构造函数
        parent = a;
        lc = nullptr;
        rc = nullptr;
        first_pel = it;
        pel_num = _num;

        // _num代表了这个BVH_TreeNode存储了多少个图元Pel
        if(_num) {
            float minx = (*min_element(it, it + _num, Pel_min_x)).centroid.x();
            float miny = (*min_element(it, it + _num, Pel_min_y)).centroid.y();
            float minz = (*min_element(it, it + _num, Pel_min_z)).centroid.z();
            float maxx = (*max_element(it, it + _num, Pel_max_x)).centroid.x();
            float maxy = (*max_element(it, it + _num, Pel_max_y)).centroid.y();
            float maxz = (*max_element(it, it + _num, Pel_max_z)).centroid.z();
            minxyz = Vector3f(minx, miny, minz);
            maxxyz = Vector3f(maxx, maxy, maxz);
        }
    };
};


bool Ray_hit_AABB(BVH_TreeNode* e, const Ray &r, float &the_t){
    // 首先判断BVH节点的pel_num属性是否为0，pel_num表示包围盒内物体的数量。
    // 如果为0，说明包围盒内没有物体，直接返回false，表示与包围盒不相交。
    if(e->pel_num == 0)
        return false;

    // 获取光线的起点和单位化的方向向量。
    Vector3f origin = r.getOrigin();
    Vector3f direction = r.getDirection().normalized();

    // 初始化t值的范围。
    // t_xmin和t_xmax表示光线与包围盒在X轴上的相交位置，初始值为负无穷和正无穷；
    float t_xmin = -1e38;
    float t_xmax = 1e38;
    float t_ymin = -1e38;
    float t_ymax = 1e38;
    float t_zmin = -1e38;
    float t_zmax = 1e38;

    // 根据光线的方向向量的正负情况，计算t值的范围。
    // 首先判断direction.x()的正负，如果大于0，则计算t_xmin和t_xmax的值，否则计算t_xmax和t_xmin的值。
    // 类似地，根据direction.y()和direction.z()的正负情况，计算t_ymin、t_ymax、t_zmin和t_zmax的值
    if(direction.x() > 1e-4) {
        t_xmin = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmax = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if(direction.x() < -1e-4) {
        t_xmax = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmin = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if(origin.x() > e->maxxyz.x() || origin.x() < e->minxyz.x())
        return false;

    // 对于每个轴上的t值范围，进行一些边界判断。
    // 如果t_xmax或t_ymax或t_zmax小于等于0，表示光线与包围盒在正方向上无相交，返回false
    if(t_xmax <= 0)
        return false;

    if(direction.y() > 1e-4) {
        t_ymin = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymax = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if(direction.y() < -1e-4) {
        t_ymax = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymin = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if(origin.y() > e->maxxyz.y() || origin.y() < e->minxyz.y())
        return false;

    if(t_ymax <= 0)
        return false;

    if(direction.z() > 1e-4) {
        t_zmin = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmax = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if(direction.z() < -1e-4) {
        t_zmax = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmin = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if(origin.z() > e->maxxyz.z() || origin.z() < e->minxyz.z())
        return false;

    if(t_zmax <= 0)
        return false;
    
    // 计算t值的范围的交集
    float t0, t1;
    t0 = max(t_zmin, max(t_xmin, t_ymin));
    t1 = min(t_zmax, min(t_xmax, t_ymax));
    // 判断t0和t1的关系。如果t0小于t1，说明光线与包围盒相交，选择t0和t1中较大的一个值作为相交点的参数t值，并返回true。
    // 如果t0大于等于t1，说明光线与包围盒没有相交，返回false
    if(t0 < t1) {
        the_t = (t0 > 0) ? t0 : t1;
        return true;
    } else {
        return false;
    }
};

class Mesh : public Object3D {

public:
    BVH_TreeNode* produce_child(int axis, BVH_TreeNode* now_root);//axis:0为x轴，1为y轴，2为z轴

    Mesh(const char *filename, Material *m, const Vector3f& v);
    ~Mesh();
    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    vector<TriangleIndex> vIndex, tIndex, nIndex;  // 对应一个平面的三个点的顶点下标、贴图下标、法线下标
    vector<Vector3f> v, vn; // 对应 obj 文件中的一系列点和法向
    vector<Vector2f> vt;  // 对应 obj 文件中贴图 uv 位置
    vector<Object3D *> triangles;  // 储存每一个三角形

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override;
    bool hit_intersect(const Ray &r, Hit &h, float tmin, BVH_TreeNode* e);
    BVH_TreeNode* root;
    vector<Pel> triangles_info;

    void delete_node(BVH_TreeNode* e);


private:

    // Normal can be used for light estimation
    void computeNormal();
    void computeNormal_and_texture();
    void computeAABB();

    void setup_bvh_tree();
    void produce_child(BVH_TreeNode* e);
};

bool Mesh::hit_intersect(const Ray &r, Hit &h, float tmin, BVH_TreeNode* e) {//已经通过AABB包围盒测试的
    // 首先判断BVH节点的左子节点（lc）和右子节点（rc）是否都为空。
    // 如果是叶子节点，表示该节点所代表的几何体是三角形。
    // 对于每个三角形，调用其intersect方法，判断光线是否与三角形相交，更新相交点的信息。
    // 如果有任意一个相交结果为true，则返回true，表示光线与几何体相交。否则，返回false。
    if (e->rc == nullptr && e->lc == nullptr) {//叶子节点
        bool result = false;
        for (auto it = e->first_pel; it != e->first_pel + e->pel_num; ++it) {
            result |= ((Triangle *)triangles[(*it).index])->intersect(r, h, tmin, 0);
        }
        return result;
    }

    // 如果左子节点和右子节点不都为空，说明该节点是内部节点。
    // 首先调用Ray_hit_AABB函数判断光线是否与AABB包围盒相交，同时获取相交的参数t值（lc_t, rc_t）。
    float lc_t;
    float rc_t;
    bool lc_hit = Ray_hit_AABB(e->lc, r, lc_t);
    bool rc_hit = Ray_hit_AABB(e->rc, r, rc_t);

    // 左子节点和右子节点的相交结果都为false，说明光线与当前节点的AABB包围盒没有相交，返回false，表示光线与几何体没有相交。
    if(!(lc_hit || rc_hit))
        return false;

    bool real_hit;

    // 如果左子节点和右子节点的相交结果都为true，表示光线与两个子节点的AABB包围盒都相交。
    // 在这种情况下，根据t值的大小决定递归遍历左子树还是右子树。
    // 如果lc_t小于rc_t，先递归遍历左子树，如果返回true，则直接返回true，表示光线与几何体相交；如果返回false，则递归遍历右子树，并返回其结果。
    // 反之亦然
    if(lc_hit && rc_hit) {
        if(lc_t < rc_t) {
            real_hit = hit_intersect(r, h, tmin, e->lc);
            if(real_hit)
                return true;
            else
                real_hit = hit_intersect(r, h, tmin, e->rc);
            return real_hit;
        } else {
            real_hit = hit_intersect(r, h, tmin, e->rc);
            if(real_hit)
                return true;
            else
                real_hit = hit_intersect(r, h, tmin, e->lc);
            return real_hit;
        }
    } else if(lc_hit)
        return hit_intersect(r, h, tmin, e->lc); // 如果只有左子节点相交，递归遍历左子树，并返回结果。
    else
        return hit_intersect(r, h, tmin, e->rc); // 如果只有右子节点相交，递归遍历右子树，并返回结果。

};


bool Mesh::intersect(const Ray &r, Hit &h, float tmin, float time_) {

    // Optional: Change this brute force method into a faster one.
    bool result = false;
    // 根节点光线都没有撞上，那么肯定就是没有撞上
    float t;
    if(!Ray_hit_AABB(root, r, t))
        return false;

    if(!is_moving)
        // 光线没有移动的情况
        result = hit_intersect(r, h, tmin, root);
    else
        // 光线进行移动的情况
        result = hit_intersect(Ray(r.origin - time_ * velocity, r.direction), h, tmin, root);
    return result;
}

// 获得传入的文件名等信息, ve指得是运动向量
Mesh::Mesh(const char *filename, Material *material, const Vector3f& ve) : Object3D(material, ve) {
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    std::string vnTok("vn");

    char bslash = '/', space = ' ';
    std::string tok;
    bool texture_and_normal = false;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;

        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        } else if (tok == fTok) {
            TriangleIndex vrig, trig, nrig;
            if (line.find(bslash) != std::string::npos) {  //限定两种输入方式f 2 3 4与f 2/3/4 3/4/5 4/5/6
                // 有顶点、法向、贴图的下标
                texture_and_normal = true;
                std::replace(line.begin(), line.end(), bslash, space);

                std::stringstream facess(line);
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> vrig[ii] >> trig[ii] >> nrig[ii];
                    trig[ii]--;
                    vrig[ii]--;
                    nrig[ii]--;
                }
            } else {
                for (int ii = 0; ii < 3; ii++) {  // 只有顶点下标
                    ss >> vrig[ii];
                    vrig[ii]--;
                }
            }
            vIndex.push_back(vrig);
            tIndex.push_back(trig);
            nIndex.push_back(nrig);
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
            vt.push_back(texcoord);
        } else if (tok == vnTok) {
            Vector3f vnvec;
            ss >> vnvec[0] >> vnvec[1] >> vnvec[2];
            vn.push_back(vnvec);
        }
    }
    f.close();
    if(!texture_and_normal)
        computeNormal();
    else
        computeNormal_and_texture();

    computeAABB();
    setup_bvh_tree();   
}

void Mesh::computeAABB() {
    if((int) triangles.size() <= 0) {
        printf("Obj File Errors.\n");
        exit(0);
    }

    for (int i = 0; i < (int) triangles.size(); ++i) {
        float now_minx, now_maxx, now_miny, now_maxy, now_minz, now_maxz;
        for (int j = 0; j < 3; j++) {
            if(j == 0) {
                now_minx = now_maxx = ((Triangle*) triangles[i])->vertices[j].x();
                now_miny = now_maxy = ((Triangle*) triangles[i])->vertices[j].y();
                now_minz = now_maxz = ((Triangle*) triangles[i])->vertices[j].z();
            } else {
                if(((Triangle*) triangles[i])->vertices[j].x() < now_minx)
                    now_minx = ((Triangle*) triangles[i])->vertices[j].x();
                else if(((Triangle*) triangles[i])->vertices[j].x() > now_maxx)
                    now_maxx = ((Triangle*) triangles[i])->vertices[j].x();

                if(((Triangle*) triangles[i])->vertices[j].y() < now_miny)
                    now_miny = ((Triangle*) triangles[i])->vertices[j].y();
                else if(((Triangle*) triangles[i])->vertices[j].y() > now_maxy)
                    now_maxy = ((Triangle*) triangles[i])->vertices[j].y();

                if(((Triangle*) triangles[i])->vertices[j].z() < now_minz)
                    now_minz = ((Triangle*) triangles[i])->vertices[j].z();
                else if(((Triangle*) triangles[i])->vertices[j].z() > now_maxz)
                    now_maxz = ((Triangle*) triangles[i])->vertices[j].z();
            }
        }
        Pel t = Pel(now_minx, now_miny, now_minz, now_maxx, now_maxy, now_maxz, i);
        this->triangles_info.push_back(t);
    }
}

void Mesh::computeNormal() {  // 没有贴图和顶点法线, 直接根据三个顶点计算面法线
    vn.resize(vIndex.size());
    for (int triId = 0; triId < (int) vIndex.size(); ++triId) {
        TriangleIndex& triIndex = vIndex[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        vn[triId] = b / b.length();

        Triangle* triangle = new Triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], material, Vector3f::ZERO);
        triangle->normal = vn[triId];
        triangles.push_back(triangle);
    }
}

void Mesh::delete_node(BVH_TreeNode* e) {
    if(!e)
        return;
    if (e->rc == nullptr && e->lc == nullptr) {
        delete e;
        return;
    }
    delete_node(e->lc);
    delete_node(e->rc);
    delete e;
    return;
}

Mesh::~Mesh() {//删树节点, 非常重要
    for (int i = 0; i < triangles.size(); i++)
        delete triangles[i];
    delete_node(root);
}

void Mesh::computeNormal_and_texture() { // 有贴图和顶点法线
    for (int triId = 0; triId < (int) vIndex.size(); ++triId) {

        TriangleIndex& triIndex = vIndex[triId];

        Triangle* triangle = new Triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material, Vector3f::ZERO);

        triangles.push_back(triangle);

        if(!vt.empty()) {
            TriangleIndex& tIdx = tIndex[triId];
            ((Triangle *)triangles.back())->set_vt(vt[tIdx[0]], vt[tIdx[1]], vt[tIdx[2]]);
        }
        
        if(!vn.empty()) {
            TriangleIndex& nIdx = nIndex[triId];
            ((Triangle *)triangles.back())->set_vn(vn[nIdx[0]], vn[nIdx[1]], vn[nIdx[2]]);
        }
    }
}

void Mesh::setup_bvh_tree() {
    root = new BVH_TreeNode(nullptr, triangles_info.begin(), (int) triangles_info.size());
    produce_child(root);
}

void Mesh::produce_child(BVH_TreeNode* e) {
    // 递归建树
    if(e->pel_num < leaf_max_pel)
        return;
    bool (*comp)(Pel x);
    float dx = e->maxxyz.x() - e->minxyz.x();
    float dy = e->maxxyz.y() - e->minxyz.y();
    float dz = e->maxxyz.z() - e->minxyz.z();
    if (dx > dy && dx > dz) {
        // dx是三者中最大的
        comp = Pel_cen_x;
        now_center = e->minxyz.x() + dx / 2;
    } else if (dy > dz && dy > dx){
        // dy是三者中最大的
        comp = Pel_cen_y;
        now_center = e->minxyz.y() + dy / 2;
    } else {
        // dz是三者中最大的
        comp = Pel_cen_z;
        now_center = e->minxyz.z() + dz / 2;
    }

    auto bound = partition(e->first_pel, e->first_pel + e->pel_num, comp);
    int distance1 = distance(e->first_pel, bound);
    int distance2 = e->pel_num - distance1;
    BVH_TreeNode* l_child = new BVH_TreeNode(e, e->first_pel, distance1);
    BVH_TreeNode* r_child = new BVH_TreeNode(e, bound, distance2);
    produce_child(l_child);
    produce_child(r_child);
}

#endif
