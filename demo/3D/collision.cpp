//
//  collision.cpp
//  collision3D
//
//  Created by Ninter6 on 2024/4/30.
//

#include <iostream>

#include "raylib.h"
#include "raymath.h"

#include "pxpls/3D/PhysicsWorld.hpp"

class Entity {
public:
    Entity(pxpls::DynamicsWorld& world, std::unique_ptr<pxpls::Collider>&& collider, bool is_static = false)
    : World(world), Collider(std::move(collider)), Rigidbody(new pxpls::Rigidbody()) {
        Rigidbody->collider = Collider.get();
        Rigidbody->IsKinematic = true;
        
        Rigidbody->IsDynamic = !is_static;
        
        World.AddRigidbody(Rigidbody.get());
    }
    
    Entity(Entity&&) = default;
    
    virtual ~Entity() = default;
    
    pxpls::Transform& Trans() const {
        return Rigidbody->transform;
    }
    
    virtual void draw() const = 0;
    
    pxpls::DynamicsWorld& World;
    std::unique_ptr<pxpls::Collider> Collider;
    std::unique_ptr<pxpls::Rigidbody> Rigidbody;
};

class Sphere : virtual public Entity {
public:
    Sphere(pxpls::DynamicsWorld& world, float radius, bool is_static = false)
    : Entity(world, std::make_unique<pxpls::SphereCollider>(pxpls::Point{}, radius), is_static) {}
    
    virtual void draw() const override {
        auto p = dynamic_cast<pxpls::SphereCollider*>(Collider.get());
        const auto& pos = p->Center + Trans().Position;
        const auto& R = p->Radius * Trans().Scale.x;
        
        DrawSphere({pos.x, pos.y, pos.z}, R, Rigidbody->IsDynamic ? YELLOW : BLUE);
    }
    
};

class Plane : virtual public Entity {
public:
    Plane(pxpls::DynamicsWorld& world, const pxpls::Plane& p, bool is_static = false)
    : Entity(world, std::make_unique<pxpls::PlaneCollider>(p), is_static) {}
    
    virtual void draw() const override {
        // null ...
    }
};

pxpls::DynamicsWorld world{std::make_unique<pxpls::UniformGrid>(pxpls::Bounds{-100, 100}, mathpls::uivec3{10})};
pxpls::VerletSolver solver{};

auto sphere0 = std::make_shared<Sphere>(world, 10, true);
auto plane = std::make_shared<Plane>(world, mathpls::vec4{0, 1, 0, 0}, true);

std::vector<std::shared_ptr<Entity>> entities{sphere0};

constexpr int screenWidth = 640;
constexpr int screenHeight = 480;

int main() {
    world.AddSolver(&solver);
    world.Gravity = {0, -50, 0};
    
    auto sphere1 = std::make_shared<Sphere>(world, 5);
    sphere1->Rigidbody->SetPosition({0, 20, 0});
    auto sphere2 = std::make_shared<Sphere>(world, 5);
    sphere2->Rigidbody->SetPosition({0, 50, 7});
    
    entities.emplace_back(sphere1);
    entities.emplace_back(sphere2);
    
    //启用反锯齿
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    
    //初始化窗口
    InitWindow(screenWidth, screenHeight, "pxpls demo: collision 3D");
    
    // 初始化摄像机
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 40.0f, 20.0f, 0.0f }; //相机所在位置{x,y,z}
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f }; //相机朝向位置{x,y,z}
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f }; //相机正上方朝向矢量
    
    
    camera.fovy = 90.0f; //相机视野宽度
    camera.projection = CAMERA_PERSPECTIVE; //采用透视投影
    
    //设置动画帧率（刷新率，fps）为60帧/秒
    SetTargetFPS(60);
    //--------------------------------------------------------------------------------------
    
    // 主游戏循环
    while (!WindowShouldClose())    //关闭窗口或者按ESC键时返回true
    {
        float time = GetTime();
        // 每次循环更新一帧
        // 摄像机围绕y轴转动
        float cameraTime = time * 0.3f;
        camera.position.x = cos(cameraTime) * 50.f;
        camera.position.z = sin(cameraTime) * 50.f;
        
        BeginDrawing();
        
        ClearBackground(WHITE);
        DrawFPS(0, 0);
        
        //以摄像机视角绘制3d内容
        BeginMode3D(camera);
        //绘制水平面网格
        DrawGrid(100, 5);
        //绘制Y轴
        DrawLine3D({0,100,0}, {0,-100,0}, BLACK);
        
        for (auto& i : entities)
            i->draw();
        
        EndMode3D();
        EndDrawing();
        
        world.Step(.02f);
        
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            auto pos = mathpls::random::rand_vec3() * 10;
            pos.y += 25;
            
            auto s = std::make_shared<Sphere>(world, 5);
            s->Rigidbody->SetPosition(pos);
            
            entities.push_back(s);
        }
    }
    
    //关闭窗口
    CloseWindow();
    
    return 0;
}
