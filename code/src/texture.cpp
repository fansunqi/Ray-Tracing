#define STB_IMAGE_IMPLEMENTATION  // 使用stb_image解码库来读取纹理图片，更加快速
#include <vecmath.h>
#include "stb_image.hpp"
#include <cstring>
#include "texture.hpp"

// 灰度图的转换，用于计算凹凸贴图中的梯度
float Texture::get_gray(int index) { return (picture[index] / 255.0 - 0.5) * 2; }

float get_decimal(float a){ 
    float tmp =  a - int(a);
    if (tmp < 0){
        return tmp + 1;
    }else{
        return tmp;
    }
}

int adjust_boundary(int a, int boundary){
    if (a < 0){
        return 0;
    }
    else{
        if (a >= boundary){
            return (boundary - 1);
        }
        else{
            return a;
        }
    }
}

int Texture::get_index(float u, float v) {
    // UV位置是用于在三维模型表面上定义纹理映射的二维坐标系统
    // 根据计算得到的uv位置获得纹理图中的位置
    int x = get_decimal(u) * width;
    int y = get_decimal(v) * height;
    x = adjust_boundary(x, width);
    y = adjust_boundary(y, height);
    return (y * width + x) * channel;;
};

Texture::Texture(const char* filename) {
    if (std::strlen(filename) == 0) {
        is_texture = false;
        return;
    }
    // 读取图片, 并填充Texture类中的宽width, 高height, 通道数channel
    this->picture = stbi_load(filename, &width, &height, &channel, 0);
    // 首先把is_texture置为true
    is_texture = true;
}

Vector3f Texture::get_color(float u, float v) {
    // 通过uv浮点数得到颜色
    u = get_decimal(u) * width;
    v = height * (1 - get_decimal(v));
    int iu = (int)u, iv = (int)v;
    Vector3f ret_color = Vector3f::ZERO;
    float s = u - iu;
    float t = v - iv;
    Vector3f color1 = (1 - s) * get_pic_color(iu, iv + 1) + s * get_pic_color(iu + 1, iv + 1);
    Vector3f color2 = (1 - s) * get_pic_color(iu, iv) + s * get_pic_color(iu + 1, iv);
    ret_color += (1 - t) * color2;
    ret_color += t * color1;
    return ret_color;
}

Vector3f Texture::get_pic_color(int x, int y) {
    // 在 Texture::get_color(float u, float v) 中调用
    x = adjust_boundary(x, width);
    y = adjust_boundary(y, height);
    int index = (y * width + x) * channel;
    return Vector3f(picture[index + 0], picture[index + 1], picture[index + 2]) / 255.0;
}

// 凹凸贴图得到梯度
float Texture::get_disturb(float u, float v, Vector2f &grad) {
    float disturb;
    if(!is_texture) {
        disturb = 0.0;
    }
    else{
        int idx = get_index(u, v);
        disturb = get_gray(idx);
        float du = 1.0 / width, dv = 1.0 / height;
        //计算梯度
        grad[0] = width * (get_gray(get_index(u + du, v)) - get_gray(get_index(u - du, v))) * 0.5;
        grad[1] = height * (get_gray(get_index(u, v + dv)) - get_gray(get_index(u, v - dv))) * 0.5;
    }
    return disturb;
}

