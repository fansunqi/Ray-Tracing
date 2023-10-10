#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
    
public:
    float radius;
    Vector3f center;
    Sphere() : radius(1.0), center(Vector3f::ZERO){}
    Sphere(const Vector3f &center, float radius, Material *material, const Vector3f &v) : Object3D(material, v), radius(radius), center(center) {}
    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        Vector3f origin = r.getOrigin();
        if(is_moving)
            origin -= time_ * this->velocity;
        Vector3f direction = r.getDirection();
        float dir_len = direction.length();
        direction.normalize();
        Vector3f l = this->center - origin;
        float l_sqlength = l.squaredLength();
        float radius_sq = radius * radius;
        
        if(std::abs(l_sqlength - radius_sq) < 0.01) {
            float t = 2 * Vector3f::dot(direction, l) / dir_len;
            if (t <= 0)
                return false;
            Vector3f point2 = r.pointAtParameter(t);
            Vector3f p = point2 - center;
            
            if(t > tmin &&  t < h.getT()) {
                float u = 0.5 + atan2(p.x(), p.z()) / (2 * M_PI);
                float v = 0.5 - asin(p.y()) / M_PI;
                p.normalize();
                Vector3f new_color = material->get_color(u, v);

                h.set(t, this->material, p, new_color, point2, material->texture.is_texture);
                return true;
            }
        }

        float tp = Vector3f::dot(l, direction);
        if ((tp < 0 && l_sqlength > radius_sq) || (l_sqlength - tp * tp > radius_sq))
            return false;

        float t_sq =  radius_sq - l_sqlength - tp * tp;
        float final_t;
        if(l_sqlength > radius_sq) {
            final_t = tp - sqrt(t_sq);
        } else {
            final_t = tp + sqrt(t_sq);
        }
        Vector3f p = final_t * direction + origin - center;
        Vector3f old_normal = p.normalized();
        if(final_t / dir_len > tmin &&  final_t / dir_len < h.getT()) {
            // 极坐标确定u,v
            float u = 0.5 + atan2(old_normal.x(), old_normal.z()) / (2 * M_PI);
            float v = 0.5 - asin(old_normal.y()) / M_PI;

            Vector2f grad = Vector2f::ZERO;
            // 用于凹凸贴图
            float f = material->bump.get_disturb(u, v, grad); // bump是Texture类型, 得到法线扰动
            
            Vector3f new_normal = p;
            if ((f > 1e-4) || (f < -1e-4)) {   // 法线扰动
                float phi = u * 2 * M_PI;
                float theta = M_PI - v * M_PI;
                Vector3f pu(-p.z(), 0, p.x()), pv(p.y() * cos(phi), -radius * sin(theta), p.y() * sin(phi));
                // 两个方向的扰动叉乘得到新的法线
                new_normal = Vector3f::cross(pu + old_normal * grad[0] / (2 * M_PI), pv + old_normal * grad[1] / M_PI).normalized();
            }
            Vector3f new_color = material->get_color(u, v); // 查询贴图颜色
            h.set(final_t / dir_len, this->material, new_normal, new_color, p + this->center, material->texture.is_texture);
            return true;
        }
        return false;
    }    
};


#endif
