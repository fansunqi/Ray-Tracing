#ifndef PHOTON_H
#define PHOTON_H
#include "camera.hpp"
#include "constant.hpp"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "random_producer.hpp"
#include "photon.hpp"
#include <queue>
#include <algorithm>

using namespace std;

struct Photon {
    Vector3f position;
    Vector3f direction;
    Vector3f power;
    Photon(Vector3f _position, Vector3f _direction, Vector3f _power):position(_position), direction(_direction), power(_power) {}
};

// 类似于DSA作业中那样模拟 KD-Tree 节点
struct KDTreeNode
{
    int depth;
    Photon* photon;
    KDTreeNode* lc;
    KDTreeNode* rc;
    KDTreeNode* parent;
    KDTreeNode(KDTreeNode* e) :parent(e), lc(nullptr), rc(nullptr), photon(nullptr){}
};

Vector3f Pos;

struct cmp
{
    bool operator()(const Photon* a, const Photon*b)
    {
         return (a->position - Pos).squaredLength() < (b->position - Pos).squaredLength();
    }
};


bool find_median_x(Photon* pp, Photon* qq) {
    return pp->position.x() < qq->position.x();
}

bool find_median_y(Photon* pp, Photon* qq) {
    return pp->position.y() < qq->position.y();
}

bool find_median_z(Photon* pp, Photon* qq) {
    return pp->position.z() < qq->position.z();
}

bool cmp_x(Vector3f& pp, Vector3f& qq) {
    return pp.x() < qq.x();
}

bool cmp_y(Vector3f& pp, Vector3f& qq) {
    return pp.y() < qq.y();
}

bool cmp_z(Vector3f& pp, Vector3f& qq) {
    return pp.z() < qq.z();
}


class Photon_KDtree {
public:
    KDTreeNode* root;
    vector<Photon*>::iterator start;
    vector<Photon*>::iterator end;
    vector<Photon*> k_near;

    Photon_KDtree(vector<Photon*>::iterator _start, vector<Photon*>::iterator _end) : start(_start) , end(_end){
        buildtree(start, end, 0, root);
        KDTreeNode* p = root;
    };

    void buildtree(vector<Photon*>::iterator _start, vector<Photon*>::iterator _end, int depth, KDTreeNode* e){
        e->depth = depth;
        if(distance(_start, _end) == 0)
            return;
        int total = distance(_start, _end) / 2;

        // 选择按照哪一个方向排序
        bool (*cmp_func) (Photon* p1, Photon* p2);
        int remainder = depth % 3;
        if (remainder == 0){
            cmp_func = find_median_z;
        }
        else if(remainder == 1){
            cmp_func = find_median_y;
        }
        else{
            cmp_func = find_median_x;
        }
        sort(_start, _end, cmp_func);

        auto it = _start + total;
        e->photon = *it;
        KDTreeNode* e1 = new KDTreeNode(e);
        KDTreeNode* e2 = new KDTreeNode(e);
        e->lc = e1;
        e->rc = e2;
        buildtree(_start, it, depth + 1, e1);
        buildtree(it + 1, _end, depth + 1, e2);
    };
    
    void search(Vector3f pos, float& radius, int& M, Vector3f& phi_i){
        k_near.clear();
        Pos = pos;
        recurve(root, radius);
        M = (int)k_near.size();
        auto it = k_near.begin();
        while(it != k_near.end()){
            phi_i += (*it)->power;
            ++it;
        }
    };
    
    void recurve(KDTreeNode* e, float radius){
        if(!e->photon)
            return;

        float axis_dist;
        bool (*cmp_func) (Vector3f& pp, Vector3f& qq);
        int remainder = e->depth % 3;
        if (remainder == 0){
            cmp_func = cmp_z;
            axis_dist = Pos.z() - e->photon->position.z();
        }
        else if(remainder == 1){
            cmp_func = cmp_y;
            axis_dist = Pos.y() - e->photon->position.y();
        }
        else{
            cmp_func = cmp_x;
            axis_dist = Pos.x() - e->photon->position.x();
        }

        if(cmp_func(Pos, e->photon->position))
            recurve(e->lc, radius);
        else
            recurve(e->rc, radius);

        if((e->photon->position - Pos).squaredLength() < radius * radius)
            k_near.push_back(e->photon);
        if (radius > abs(axis_dist)) {
            if(axis_dist < 0)
                recurve(e->rc, radius);
            else
                recurve(e->lc, radius);
        }
    };

    void delete_node(KDTreeNode* e) {
        if(!e)
            return;
        if(e->photon)
            delete e->photon;
        if (e->rc == nullptr && e->lc == nullptr) {
            delete e;
            return;
        }
        delete_node(e->lc);
        delete_node(e->rc);
        delete e;
        return;
    };

};

#endif