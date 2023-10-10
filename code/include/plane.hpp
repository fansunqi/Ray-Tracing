#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Vector3f normal;
    Vector3f main_tangent;
    Vector3f bi_normal;
    float offset;
    Plane() {}

    Plane(const Vector3f &normal, float d, Material *m, const Vector3f &v) : Object3D(m, v) {
        float THRESHOLD = 1e-3;
        this->offset = d / normal.length();
        this->normal = normal.normalized();
        if(normal.x() < THRESHOLD && normal.z() < THRESHOLD) {
            this->main_tangent = Vector3f::RIGHT;
        } else {
            this->main_tangent = Vector3f::cross(Vector3f::UP, normal);
            this->main_tangent.normalize();
        }     
        this->bi_normal =  Vector3f::cross(main_tangent, normal);
        this->bi_normal.normalize();
    }

    ~Plane() override = default;

    // 用于纹理贴图
    //（100这个常数可以调节，取决于希望铺开的大小，bi_normal和main_tangent是平行与法线的一组基向量）
    void get_uv(const Vector3f& p, float& u, float& v) {
        v = Vector3f::dot(p - offset * normal, bi_normal) / 100;
        u = Vector3f::dot(p - offset * normal, main_tangent) / 100;     
    }

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        
        Vector3f origin = r.getOrigin();
        if(is_moving)
            origin -= time_ * this->velocity;   // 处理运动模糊
        Vector3f direction = r.getDirection();
        float dir_len = direction.length();
        direction.normalize();
        float a = Vector3f::dot(origin, normal);
        float b = offset - a;
        float c = Vector3f::dot(direction, normal);
        if((c == 0) || (b > -1e-3 && b < 1e-3))
            return false;
        float t = (b / c) / dir_len;
        if(t > 0 && t > tmin && t < h.getT()) {
            Vector3f next_origin = origin + r.getDirection() * t;
            float v = next_origin.y();
            float u = Vector3f::dot(next_origin - r.getDirection() * normal, main_tangent);
            Vector2f grad = Vector2f::ZERO;
            float f = material->bump.get_disturb(u, v, grad);

            Vector3f new_normal = normal;

            if (!(f < 1e-4 && f > -1e-4)) {
                new_normal += main_tangent * grad[0];
                new_normal += bi_normal * grad[1];
                new_normal.normalize();
            }

            // 求交时调用
            float uu = 0;
            float vv = 0;
			get_uv(next_origin, uu, vv);         
            h.set(t, this->material, new_normal, material->get_color(uu, vv), next_origin, material->texture.is_texture);
            return true;
        }else{
            return false;
        }
    }


    

};

#endif //PLANE_H
		

