#ifndef PT_H
#define PT_H

#include "camera.hpp"
#include "constant.hpp"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "random_producer.hpp"
using namespace std;

float select_max(float a, float b, float c){
    return a > b && a > c ? a : b > c ? b : c;
}


// radiance:每个sample采样得到颜色的函数,也就是smallPt中的radiance函数
Vector3f radiance(Ray ray, Group* group, int depth) {
    
    Hit hit;
    bool isIntersect = group->intersect(ray, hit, 0, RND2);

    // 判断光线与一组几何体(group)有没有相交, 没有相交就返回黑色
    if(!isIntersect) {
        return Vector3f::ZERO; //递归终止，返回背景色 黑色
    }
    else{

        Material* material = hit.getMaterial();
        
        Vector3f hit_emission = material->emission;
        Vector3f hit_normal = hit.getNormal().normalized();
        Vector3f next_origin = ray.getOrigin() + hit.getT() * ray.getDirection();
        Vector3f ray_direction = ray.getDirection().normalized();

        
        if(material->emission != Vector3f::ZERO){
            return material->emission;
        }

        bool is_texture;
        Vector3f hit_color = hit.get_color(is_texture);
        if(is_texture == false){
            hit_color = material->color;
        }   

        // 判断是否迭代结束
        float p = select_max(hit_color.x(), hit_color.y(), hit_color.z());
        p = p < 0.75 ? p : 0.75;
        if(depth > pt_max_depth) {
            if(RND2 > p) {
                return material->emission;
            } 
            hit_color = hit_color / p;
        }
        

        float type_decision = RND2;
        float b = Vector3f::dot(ray_direction, hit_normal);
        if(type_decision < material->type.x()) {//漫反射
            Vector3f z_ = Vector3f::cross(ray_direction, hit_normal); z_.normalize();
            Vector3f x_ = Vector3f::cross(z_, hit_normal); x_.normalize();
            Vector3f next_direction;     
            if(b > 0)
                next_direction = RND1 * z_ + RND1 * x_ - RND2 * hit_normal;
            else
                next_direction = RND1 * z_ + RND1 * x_ + RND2 * hit_normal;
            next_direction.normalize();
            Ray next_ray = Ray(next_origin, next_direction);
            Vector3f next_color = radiance(next_ray, group, depth + 1);

            return hit_emission + hit_color * next_color;

        } else if(type_decision < material->type.x() + material->type.y()) {//镜面反射
            Vector3f next_direction = ray_direction - hit_normal * (b * 2);next_direction.normalize();
            Ray next_ray = Ray(next_origin, next_direction);
            Vector3f next_color = radiance(next_ray, group, depth + 1);
            return hit_emission + hit_color * next_color;

        } else {//折射
            float n;
            float R0 = ((1.0 - material->refractive_index) * (1.0 - material->refractive_index)) / ((1.0 + material->refractive_index) * (1.0 + material->refractive_index));
            if(b > 0) {
                hit_normal.negate();
                n = material->refractive_index;     
            } else {
                n = 1.0 / material->refractive_index;
            }
            
            float cos1 = -Vector3f::dot(hit_normal, ray_direction);
            float cos2 = 1.0 - n * n * (1.0 - cos1 * cos1); //cos(theta2)的平方
            Vector3f reflect = (ray_direction + hit_normal * (cos1 * 2));
            Ray next_ray_reflect = Ray(next_origin, reflect);
           
            if(cos2 < 0) {
                return hit_emission + hit_color * radiance(next_ray_reflect, group, depth + 1);
            }
            //Schlick估计菲涅尔项
            float Rprob = R0 + (1.0 - R0) * pow(1.0 - cos1, 5.0);

            Vector3f refrac =  ((ray_direction * n) + (hit_normal * (n * cos1 - sqrt(cos2)))).normalized();
             Ray next_ray_refrac = Ray(next_origin, refrac);
            float P = 0.25 + 0.5 * Rprob;
            if (depth > 1)
                if (RND2 < P) {
                    return hit_emission + hit_color * ((Rprob / P) * radiance(next_ray_reflect, group, depth + 1));
                } else {
                    return hit_emission + hit_color * (((1 - Rprob) / (1 - P)) * radiance(next_ray_reflect, group, depth + 1));
                }
            else{
                return hit_emission + hit_color * (Rprob * radiance(next_ray_reflect, group, depth + 1) + (1 - Rprob) * radiance(next_ray_refrac, group, depth + 1));
            }
        }
    }
    
}

class Path_Tracer {
    const SceneParser& scene;
    const char* output_file;
    public:
        Path_Tracer(const SceneParser& scene, const char* output):scene(scene), output_file(output){};
        void trace();
};

void Path_Tracer::trace(){
    Camera *camera = scene.getCamera();
    int w = camera->getWidth();
    int h = camera->getHeight();
    Image myImage(w, h);
    Group* group = scene.getGroup();
    Vector3f background = scene.getBackgroundColor();  
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            // 遍历一张图的所有像素
            Vector3f finalColor = Vector3f::ZERO;
            int s = 0;
            while(s < pt_samples) {
                Vector2f ori = Vector2f(x + RND1 / 2, y + RND1 / 2);
                Ray camRay = camera->generateRay(ori);
                finalColor += radiance(camRay, group, 0);
                s++;
            }
            finalColor = finalColor / float(pt_samples);
            finalColor = Vector3f(toFloat(finalColor.x()), toFloat(finalColor.y()), toFloat(finalColor.z()));
            myImage.SetPixel(x, y, finalColor);
        }
    }
    myImage.SaveImage(output_file);

}


#endif