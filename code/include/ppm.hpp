#ifndef PPM_H
#define PPM_H

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
using namespace std;

// ViewRecord 结构体记录视点的信息
struct ViewRecord {
    Vector3f direction;
    Vector3f position;
    Vector3f power_rate;
    Vector3f accumulate_power;
    Vector3f accumulate_photon_power;
    int photon_num;
    float radius;
    int pixel_x;
    int pixel_y;
    ViewRecord(int x, int y, Vector3f pos, float r) {
        pixel_x = x;
        pixel_y = y;
        position = pos;
        radius = r;
        photon_num = 0;
        accumulate_photon_power = Vector3f::ZERO;
    }
};

class PPM {
    public:
        const SceneParser& scene;
        const char* output_file;
        Camera* camera;
        int w;
        int h;
        Vector3f background;
        Group* group;
        int caustic_photon_total;    //焦散光子数
        int photon_total;            //总共有效光子数
        std::vector<ViewRecord> view_pointmap; 

        PPM(const SceneParser& _scene, const char* output):scene(_scene), output_file(output) {
            camera = scene.getCamera();
            w = camera->getWidth();
            h = camera->getHeight();
            background = scene.getBackgroundColor();
            group = scene.getGroup();
            photon_total = 0;
            caustic_photon_total = 0;
        };

        void pt_and_record_view_point(){
            for (int x = 0; x < w; ++x) {
                for (int y = 0; y < h; ++y) {
                    for (int s = 0; s < ppm_samples; s++) {
                        Vector2f ori = Vector2f(x + RND1 / 2, y + RND1 / 2);
                        Ray camRay = camera->generateRay(ori);
                        
                        Vector3f new_power = Vector3f(1, 1, 1);
                        Vector3f accumulate_power = Vector3f::ZERO;
                        int dep = 0;
                        
                        while(true) {
                            if(dep > ppm_max_depth)
                                break;
                            else if (new_power.x() < 1e-3 && new_power.y() < 1e-3 && new_power.z() < 1e-3)
                                break;
                                
                            dep++;
                            
                            Hit hit;
                            bool isIntersect = group->intersect(camRay, hit, 0, RND2);
                            if (!isIntersect){   // 光线与物体没有交集就打破这个循环
                                break;
                            }
                            
                            Material* material = hit.getMaterial();
                            Vector3f hit_emission = material->emission;
                            Vector3f hit_normal = hit.getNormal().normalized();
                            Vector3f next_origin = camRay.getOrigin() + hit.getT() * camRay.getDirection();
                            Vector3f ray_direction = camRay.getDirection().normalized();
                            Vector3f next_direction;

                            bool is_texture;
                            Vector3f hit_color = hit.get_color(is_texture);
                            if(!is_texture)
                                hit_color = material->color;     

                            for (int li = 0; li < scene.getNumLights(); li++) {
                                Light* light = scene.getLight(li);
                                Vector3f r = next_origin -light->position;
                                if(r.squaredLength() < ((light->radius) * (light->radius))) {
                                    auto dot_ = Vector3f::dot(r, light->get_direction());
                                    if(dot_ < 1e-3 && dot_ > -1e-3) {
                                        accumulate_power += light->emission * new_power;
                                    }                                    
                                }
                            }

                            float type_decision = RND2;
                            float b = Vector3f::dot(ray_direction, hit_normal);

                            camRay.origin = next_origin;

                            accumulate_power += material->emission * new_power;
                            new_power = new_power * hit_color;
                            if(type_decision < material->type.x()) {//漫反射
                                ViewRecord vr(x, y, next_origin, ini_photon_radius);
                                vr.power_rate = new_power;
                                vr.accumulate_power = accumulate_power;
                                view_pointmap.push_back(vr);
                                break;
                            } else if(type_decision < material->type.x() + material->type.y()) {//镜面反射
                                Vector3f next_direction = ray_direction - hit_normal * (b * 2);
                                next_direction.normalize();
                                camRay.direction = next_direction;
                            } else  {
                                float n = material->refractive_index;
                                float R0 = ((1.0 -n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n));
                                if(b > 0) {
                                    hit_normal.negate();
                                } else {
                                    n = 1.0 / n;
                                }
                                // 参考：https://blog.csdn.net/qjh5606/article/details/89220500
                                float cos1 = -Vector3f::dot(hit_normal, ray_direction);
                                float cos2 = 1.0 - n * n * (1.0 - cos1 * cos1); 
                                Vector3f reflect = (ray_direction + hit_normal * (cos1 * 2));
                                float Rprob = R0 + (1.0 - R0) * pow(1.0 - cos1, 5.0);
                                Vector3f refrac =  ((ray_direction * n) + (hit_normal * (n * cos1 - sqrt(cos2)))).normalized();

                                if (RND2 < Rprob || cos2 <= 0) {
                                    camRay.direction = reflect;
                                } else {
                                    camRay.direction = refrac;                                
                                }
                            }
                        }
                    }
                }
            }
        };

        void traverse_viewpoint(Photon_KDtree& tree, bool first = false){
            // 这个就是把第一步pt_and_record_view_point中得到的视点遍历、处理一遍
            for (auto it = view_pointmap.begin(); it != view_pointmap.end(); ++it) {
                float next_radius;
                int M;
                Vector3f phi_i(0, 0, 0);
                float itradius = (*it).radius;
                if(first)
                    itradius = ini_photon_radius;
                tree.search((*it).position, itradius, M, phi_i);

                if(first) {
                    (*it).photon_num += M;
                    (*it).accumulate_photon_power += phi_i;
                }
                else {
                    int ini_num = (*it).photon_num;
                    (*it).photon_num += ppm_alpha * M;
                    float rate = float((*it).photon_num) / float(ini_num + M);
                    
                    (*it).radius = (*it).radius * sqrt(rate);
                    (*it).accumulate_photon_power += phi_i;
                    (*it).accumulate_photon_power *= rate;
                }             
            }
        };

        void emit_photon_one_iter(vector<Photon*> PhotonMap, int per_light_photon_count, bool is_caustic){
            for (int li = 0; li < scene.getNumLights(); li++) {//这里先只渲染圆盘光
                for (int i = 0; i < per_light_photon_count; i++) {
                    Light* light = scene.getLight(li);
                    Vector3f photon_power;
                    Ray light_ray = light->get_ray(photon_power);
                    int dep = 0;
                    bool HasSpecularOrGlossy = false;
                    while(true) {
                        Hit hit;
                        if(dep > ppm_max_depth)
                            break;
                        else if (photon_power.x() < 1e-3 && photon_power.y() < 1e-3 && photon_power.z() < 1e-3)
                            break;
                        bool is_intersect = group->intersect(light_ray, hit, 0, RND2);
                        dep++;
                        if(!is_intersect){
                            break;
                        }
                        
                        Material* material = hit.getMaterial();
                        Vector3f hit_emission = material->emission;
                        Vector3f hit_normal = hit.getNormal().normalized();
                        Vector3f next_origin = light_ray.getOrigin() + hit.getT() * light_ray.getDirection();
                        Vector3f ray_direction = light_ray.getDirection().normalized();
                        Vector3f next_direction;

                        bool is_texture;
                        Vector3f hit_color = hit.get_color(is_texture);
                        if(!is_texture)
                            hit_color = material->color;

                        float type_decision = RND2;
                        float b = Vector3f::dot(ray_direction, hit_normal);

                        light_ray.origin = next_origin;

                        if(type_decision < material->type.x()) {//漫反射
                            if (is_caustic == true){
                                if(!HasSpecularOrGlossy)
                                    break;
                            }
                            Vector3f z_ = Vector3f::cross(ray_direction, hit_normal);
                            Vector3f x_ = Vector3f::cross(z_, hit_normal);
                            z_.normalize();
                            x_.normalize();
                            Vector3f next_direction;
                            if(b < 0)
                                next_direction = RND1 * z_ + RND1 * x_ + RND2 * hit_normal;
                            else
                                next_direction = RND1 * z_ + RND1 * x_ - RND2 * hit_normal;
                            next_direction.normalize();
                            light_ray.direction = next_direction;
                            Photon* pho = new Photon(next_origin, ray_direction, photon_power);
                            PhotonMap.push_back(pho);
                            //只记录第一次的光子
                            if (is_caustic == true){
                                break;
                            }
                            else{
                                //轮盘赌判断是否被吸收
                                float p = hit_color.x() > hit_color.y() && hit_color.x() > hit_color.z() ? hit_color.x() : hit_color.y() > hit_color.z() ? hit_color.y() : hit_color.z();
                                p = p < 0.75 ? p : 0.75;
                                if(RND2 < p) {
                                    hit_color = hit_color / p;
                                } else {
                                    break;
                                }
                            }
                            
                        } else if(type_decision < material->type.x() + material->type.y()) {//镜面反射
                            Vector3f next_direction = ray_direction - hit_normal * (b * 2);
                            next_direction.normalize();
                            light_ray.direction = next_direction;
                            HasSpecularOrGlossy = true;
                        } else  {
                            float n = material->refractive_index;
                            float R0 = ((1.0 -n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n));
                            if(b > 0) {
                                hit_normal.negate();
                            } else {
                                n = 1.0 / n;
                            }
                            float cos1 = -Vector3f::dot(hit_normal, ray_direction);
                            float cos2 = 1.0 - n * n * (1.0 - cos1 * cos1); //cos(theta2)的平方
                            Vector3f reflect = (ray_direction + hit_normal * (cos1 * 2));
                            //Schlick估计菲涅尔项
                            float Rprob = R0 + (1.0 - R0) * pow(1.0 - cos1, 5.0);
                            Vector3f refrac =  ((ray_direction * n) + (hit_normal * (n * cos1 - sqrt(cos2)))).normalized();

                            if (RND2 < Rprob || cos2 <= 0) {
                                light_ray.direction = reflect;
                            } else {
                                light_ray.direction = refrac;
                            }
                            HasSpecularOrGlossy = true;
                        }
                        photon_power = hit_emission + hit_color * photon_power;
                    }
                }
            }
        };

        void postprocess(){
            int _pass_count = pass_count;
            int per_light_photon_count =  per_sample_photon_count / scene.getNumLights();
            while(_pass_count--) {
                vector<Photon*> PhotonMap;
                emit_photon_one_iter(PhotonMap, per_light_photon_count, false);
                photon_total += PhotonMap.size();                  
                Photon_KDtree tree(PhotonMap.begin(), PhotonMap.end());
                if(pass_count == _pass_count + 1)
                    traverse_viewpoint(tree, true);
                else
                    traverse_viewpoint(tree);
            }
        };

        void caustic(){
            int per_light_photon_count =  photon_total / (2 * scene.getNumLights());
            vector<Photon*> PhotonMap;
            emit_photon_one_iter(PhotonMap, per_light_photon_count, true);
            caustic_photon_total += PhotonMap.size();
            Photon_KDtree tree(PhotonMap.begin(), PhotonMap.end());
            traverse_viewpoint(tree, true);

        };
        
        void compute_pixel(){
            // 逐像素地计算RGB值
            Image myImage(w, h);
            Vector3f img[w * h];  // 初始化一张暂存的图片
            for(int i = 0; i < w * h; ++i)
                img[i] = Vector3f::ZERO;
            for(auto it = view_pointmap.begin(); it != view_pointmap.end(); ++it) {
                Vector3f total_photon_power = (*it).accumulate_photon_power / (M_PI * (*it).radius * (*it).radius * (caustic_photon_total + photon_total));
                Vector3f color = (*it).accumulate_power + (*it).power_rate * total_photon_power;
                img[(*it).pixel_y * w + (*it).pixel_x] += color;
            }
            for (int x = 0; x < w; ++x) {
                for (int y = 0; y < h; ++y) {
                    Vector3f finalColor = img[y * w + x] / ppm_samples;
                    finalColor = Vector3f(toFloat(finalColor.x()), toFloat(finalColor.y()), toFloat(finalColor.z()));
                    myImage.SetPixel(x, y, finalColor);
                }
            }
            myImage.SaveImage(this->output_file);
        };

        void trace(){
            pt_and_record_view_point();
            postprocess();
            caustic(); 
            compute_pixel();
        };
        
};


#endif