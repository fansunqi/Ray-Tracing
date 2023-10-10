#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include <tuple>
#include "constant.hpp"

class RevSurface : public Object3D {

    Curve *pCurve;

    float x_max, y_max, y_min;

public:
    RevSurface(Curve* pCurve, Material* material)
    : pCurve(pCurve),
      Object3D(material, Vector3f::ZERO),
      x_max(0),
      y_max(-1e38),
      y_min(1e38)
    {
        for (const auto& cp : pCurve->getControls()) {
            // Check flat.
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
            if (std::abs(cp.x()) > x_max)
                x_max = std::abs(cp.x());
            if (cp.y() > y_max)
                y_max = cp.y();
            if (cp.y() < y_min)
                y_min = cp.y();
        }

    }


    ~RevSurface() override {
        delete pCurve;
    }

    float F(const Vector3f& direction, const Vector3f& origin, float xt, float yt) {
        float result = direction.y() * direction.y() * xt * xt;
        result = pow((yt - origin.y()) * direction.x() + direction.y() * origin.x(), 2) + pow((yt - origin.y()) * direction.z() + direction.y() * origin.z(), 2) - result;
        return result;
    }


    float F_grad(const Vector3f& direction, const Vector3f& origin, float xt, float yt, float x_grad_t, float y_grad_t) {
        float result = 2 * direction.y() * direction.y() * xt * x_grad_t;
        result += 2 * direction.x() * y_grad_t * ((yt - origin.y()) * direction.x() + direction.y() * origin.x());
        result += 2 * direction.z() * y_grad_t * ((yt - origin.y()) * direction.z() + direction.y() * origin.z());
        return result;
    }


    bool inter_AABB(const Ray &r, Hit &h, float & x_, float & y_) {
        Vector3f direction = r.direction;
        Vector3f origin = r.origin;
        if (std::abs(r.origin.x()) < x_max &&
            std::abs(r.origin.z()) < x_max &&
            y_min < r.origin.y() &&
            r.origin.y() < y_max)
        {
            x_ = -std::sqrt(r.origin.x() * r.origin.x() + r.origin.z() * r.origin.z());
            y_ = r.origin.y();
            return true;
        }


        float mylimit = 1e38;
        float my_lower_bound = 1e-4;
        float t_xmin = -1 * mylimit; float t_ymin = -1 * mylimit; float t_zmin = -1 * mylimit;
        float t_xmax = mylimit; float t_ymax = mylimit; float t_zmax = mylimit; 


        if(direction.x() > my_lower_bound) {
            t_xmin = (-x_max - origin.x()) / direction.x();
            t_xmax = (x_max - origin.x()) / direction.x();
        } else if(direction.x() < -1 * my_lower_bound) {
            t_xmax = (-x_max - origin.x()) / direction.x();
            t_xmin = (x_max - origin.x()) / direction.x();
        } else if(origin.x() > x_max || origin.x() < -x_max)
            return false;

        if(t_xmax <= 0)
            return false;

        if(direction.y() > my_lower_bound) {
            t_ymin = (y_min - origin.y()) / direction.y();
            t_ymax = (y_max - origin.y()) / direction.y();
        } else if(direction.y() < -1 * my_lower_bound) {
            t_ymax = (y_min - origin.y()) / direction.y();
            t_ymin = (y_max - origin.y()) / direction.y();
        } else if(origin.y() > y_max || origin.y() < y_min)
            return false;

        if(t_ymax <= 0)
            return false;

        if(direction.z() > my_lower_bound) {
            t_zmin = (-x_max - origin.z()) / direction.z();
            t_zmax = (x_max - origin.z()) / direction.z();
        } else if(direction.z() < -1 * my_lower_bound) {
            t_zmax = (-x_max - origin.z()) / direction.z();
            t_zmin = (x_max - origin.z()) / direction.z();
        } else if(origin.z() > x_max || origin.z() < -x_max)
            return false;

        if(t_zmax <= 0)
            return false;
        
        float t0 = max(t_zmin, max(t_xmin, t_ymin));
        float t1 = min(t_zmax, min(t_xmax, t_ymax));
        
        if(t0 < t1) {
            float the_t = (t0 > 0) ? t0 : t1;
            Vector3f next_ori = origin + the_t * direction;
            x_ =  - sqrt(next_ori.x() * next_ori.x() + next_ori.z() * next_ori.z());
            y_ = next_ori.y();
            return true;
        } else {
            return false;
        }
            
    }

    float truncate(float step){
        if(step > 0.05)
            step = 0.05;
        else if(step < -0.05)
            step = -0.05;
        return step;
    }

    //对于curve上的点(x, y)，其旋转后为(xcos, y, xsin)
    bool intersect(const Ray &r, Hit &h, float tmin, float time_) override {
        // (PA2 optional TODO): implement this for the ray-tracing routine using G-N iteration.
        float AABB_x, AABB_y;
        bool is_interAABB = inter_AABB(r, h, AABB_x, AABB_y);
        if(!is_interAABB) {
            return false;
        }
        // 与包围盒相交，得到一个估计的平面点(x, y)，然后估计一下t。

        float estimate_t;
        pCurve->get_t(AABB_x, AABB_y, estimate_t);
        // 牛顿迭代
        for(int dep = 0; dep < newton_depth; dep++){

            // 参考 https://github.com/Guangxuan-Xiao/THU-Computer-Graphics-2020 的做法, 检查 t 是否在 0 和 1 之间

            if((estimate_t >= 0) && (estimate_t <= 1)){

                Vector3f now_point;
                Vector3f now_grad;
                pCurve->get_point(estimate_t, now_point, now_grad);
                float ft = F(r.direction, r.origin, now_point.x(), now_point.y());
                float ft_grad = F_grad(r.direction, r.origin, now_point.x(), now_point.y(), now_grad.x(), now_grad.y());

                if(abs(ft) < 1e-5) {
                    float tr;
                    if(abs(r.direction.y()) > 1e-3) {
                        tr = (now_point.y() - r.origin.y()) / (r.direction.y());
                    }
                    else {                
                        float a = r.direction.z() * r.direction.z() + r.direction.x() + r.direction.x();
                        float b = 2 * (r.direction.z() * r.origin.z() + r.direction.x() * r.origin.x());
                        float c = pow(r.origin.x(), 2) + pow(r.origin.z(), 2) - pow(now_point.x(), 2);
                        tr = (sqrt(pow(b, 2) - 4 * a * c) - b) / (2 * a);
                    }
                    Vector3f next_origin = tr * r.direction + r.origin;
                    
                    if(tr > tmin && pCurve->is_on_curve(next_origin)) {                                    
                        if(abs(now_point.x()) < 1e-4) {
                            if (now_point.y() > 0){
                                h.set(tr, material, Vector3f(0, 1, 0));
                            }else{
                                h.set(tr, material, Vector3f(0, -1, 0));
                            }
                            return true;
                        }else{
                            Vector2f plane_normal(now_grad.y(), -now_grad.x()); // 平面的法向量
                            plane_normal.normalize();
                            float costheta = (r.direction.x() * tr + r.origin.x()) / now_point.x();
                            float sintheta = (r.direction.z() * tr + r.origin.z()) / now_point.x();
                            Vector3f new_normal = Vector3f(costheta * plane_normal.x(), plane_normal.y(), sintheta * plane_normal.x());
                            h.set(tr, material, new_normal);
                            return true;
                        }       
                    } 
                }               
                estimate_t -= truncate(ft / ft_grad);
            }
            else{
                return false;
            }
        }
        return false;
    }
};

#endif //REVSURFACE_HPP
