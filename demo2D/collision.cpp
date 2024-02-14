//
//  collision.cpp
//  demo2D
//
//  Created by Ninter6 on 2023/12/23.
//

#include <raylib.h>
#include <rlgl.h>

#include "pxpls/2D/PhysicsWorld2D.hpp"

#include <iostream>
#include <random>
#include "nova/src/nova/Tool/se_tools/setimer.h"

class Entity {
public:
    Entity(pxpls::DynamicsWorld& world, std::unique_ptr<pxpls::Collider>&& collider)
    : World(world), Collider(std::move(collider)), Rigidbody(new pxpls::Rigidbody) {
        Rigidbody->Collider = Collider.get();
        Rigidbody->IsKinematic = true;
        Rigidbody->Restitution = .5f;
        
        World.AddRigidbody(Rigidbody.get());
    }
    
    Entity(Entity&&) = default;
    
    virtual ~Entity() = default;
    
    pxpls::Transform2D& Trans() const {
        return Rigidbody->Transform;
    }
    
    virtual void draw() const = 0;
    
    pxpls::DynamicsWorld& World;
    std::unique_ptr<pxpls::Collider> Collider;
    std::unique_ptr<pxpls::Rigidbody> Rigidbody;
};

class Circle : virtual public Entity {
public:
    Circle(pxpls::DynamicsWorld& world, float radius)
    : Entity(world, std::make_unique<pxpls::CircleCollider>(pxpls::Point2D{}, radius)) {}
    
    virtual void draw() const override {
        auto p = dynamic_cast<pxpls::CircleCollider*>(Collider.get());
        const auto& pos = p->Center + Trans().Position;
        const auto& R = p->Radius * Trans().Scale.x;
        
        DrawCircleLines(pos.x, pos.y, R, Rigidbody->IsDynamic ? YELLOW : BLUE);
    }
    
};

class Line : virtual public Entity {
public:
    Line(pxpls::DynamicsWorld& world, pxpls::Point2D ori, mathpls::vec2 vec)
    : Entity(world, std::make_unique<pxpls::LineCollider>(ori, vec)) {}
    
    virtual void draw() const override {
        auto line = dynamic_cast<pxpls::LineCollider*>(Collider.get());
        auto start = Rigidbody->Position() + line->Origin;
        auto end = start + line->Vector;
        
        DrawLineV({start.x, start.y}, {end.x, end.y}, BLUE);
    }
    
};

class AabbBox : virtual public Entity {
public:
    AabbBox(pxpls::DynamicsWorld& world, pxpls::Point2D pos, mathpls::vec2 ext)
    : Entity(world, std::make_unique<pxpls::AabbColloder>(pos, ext)) {}
    
    virtual void draw() const override {
        auto p = dynamic_cast<pxpls::AabbColloder*>(Collider.get());
        const auto& pos = p->Position + Trans().Position,
                    ext = p->Extent * Trans().Scale;
        DrawRectangleLinesEx({pos.x, pos.y, ext.x, ext.y}, 1, BLUE);
    }
};

constexpr mathpls::uivec2 WinSize{800, 600};

pxpls::DynamicsWorld world{std::make_unique<pxpls::QuadTree>(pxpls::Bounds2D{{-1000, -1000}, {1000, 1000}})};
pxpls::ImpulseSolver solver1;
pxpls::SmoothPositionSolver solver2;

int main() {
    Circle circle{world, 200},
    circle1{world, 20};
    Circle circle2{world, 100};
//    Line line{world, {-100, 0}, {200, 10}};
    
    circle.Trans().Position.y -= 50;
    
//    circle1.Trans().Position.y -= 200;
    
//    line.Rigidbody->IsDynamic = false;
//    line.Trans().Position.y += 100;
    circle2.Rigidbody->IsDynamic = false;
    
    world.AddSolver(&solver1);
    world.AddSolver(&solver2);
    world.Gravity = {0, 100};
    
    //启用反锯齿
    SetConfigFlags(FLAG_MSAA_4X_HINT);
        
    //初始化窗口
    InitWindow(WinSize.x, WinSize.y, "pxpls demo 2D: collsion");
    
    Camera2D camera = { 0 };
    camera.offset = {400, 300};
    camera.zoom = 1.0f;
    
    SetTargetFPS(60);
    rlEnableSmoothLines();
    
    std::vector<Circle> cv;
    
    st::Countdown cd{1};
    cd.Start();
    
    // Main game loop
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(BLACK);
        
        BeginMode2D(camera);
        
        rlPushMatrix();{
            rlTranslatef(0, 25*50, 0);
            rlRotatef(90, 1, 0, 0);
            DrawGrid(100, 50);
        }rlPopMatrix();
        
        circle.draw();
        circle1.draw();
        circle2.draw();
        
        for (auto& c : cv) {
            c.draw();
        }
        
        EndMode2D();
        EndDrawing();
        
        world.Step(1 / 50.);
        
        if (cd.IsTimeOut()) {
            cv.emplace_back(world, 10);
            auto r = dynamic_cast<pxpls::CircleCollider*>(cv.back().Collider.get())->Radius *= mathpls::random::rand01<float>() + .5f;
            cv.back().Trans().Position += mathpls::random::rand_vec2() * 50;
            cv.back().Rigidbody->Mass = r*r / 400;
            cd.Start();
        }
    }
    
    //关闭窗口
    CloseWindow();
}
