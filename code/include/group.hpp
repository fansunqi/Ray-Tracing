#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>



// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects) {
        num = num_objects;
    }

    ~Group() override {
        for (auto it = object_list.begin(); it != object_list.end(); it++) {
            delete (*it);
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        bool t = false;
        for (auto it = object_list.begin(); it != object_list.end(); it++) {
            bool is_intersect = (*it)->intersect(r, h, tmin, time_);
            t |= is_intersect;
        }
        return t;
    }

    

    void addObject(int index, Object3D *obj) {
        object_list.push_back(obj);
        assert(index == object_list.size() - 1);
    }

    int getGroupSize() {
        return num;
    }
    std::vector<Object3D *> object_list;
    int num;
};

#endif
	
