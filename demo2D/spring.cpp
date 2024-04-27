//
//  spring.cpp
//  demo2D
//
//  Created by Ninter6 on 2024/2/13.
//

#include <raylib.h>
#include <rlgl.h>

#include "pxpls/2D/PhysicsWorld2D.hpp"

#include <iostream>
#include <ranges>

mathpls::uivec2 win_ext{800, 500};

constexpr float dt = 3e-3f;

class Entity {
public:
    Entity(pxpls::DynamicsWorld& world, std::unique_ptr<pxpls::Collider>&& collider)
    : World(world), Collider(std::move(collider)), Rigidbody(new pxpls::Rigidbody) {
        Rigidbody->Collider = Collider.get();
        Rigidbody->IsKinematic = true;
        Rigidbody->Restitution = .5f;
        
        World.AddRigidbody(Rigidbody.get());
    }
    
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
        
        DrawCircleV({pos.x, pos.y}, R, Rigidbody->IsDynamic ? YELLOW : BLUE);
    }
    
};

class Game {
public:
    Game()
    : m_World(std::make_unique<pxpls::UniformGird>(pxpls::Bounds2D{-100, 100}, 10)),
    rectCol({win_ext.x * -.05f, win_ext.y * -.05f}, {win_ext.x * .1f, win_ext.y * .1f}) {
        rectRB.Collider = &rectCol;
        rectRB.IsDynamic = false;
        
        m_World.AddRigidbody(&rectRB);
        m_World.Gravity.g *= -3;
        
        m_World.AddSolver(&solver1);
        m_World.AddSolver(&solver2);
    }
    
    void addMass(pxpls::Point2D pos, bool pinned = false) {
        m_Entities.emplace_back(std::make_unique<Circle>(m_World, 1.f));
        m_Entities.back()->Trans().Position = pos;
        
        testLink(*m_Entities.back());
    }
    
    void tick(float deltaTime) {
        logicTick(deltaTime);
        renderTick();
    }
    
protected:
    void logicTick(float deltaTime) {
        testInput();
        constexpr int n = 1.f / 60.f / dt + .5f;
        for (int i = n; i--;)
            m_World.Step(deltaTime);
    }
    
    void renderTick() {
        for (auto& [a, b] : m_Links) {
            const auto& pa = a->Position();
            const auto& pb = b->Position();
            DrawLineEx({pa.x, pa.y}, {pb.x, pb.y}, .7f, GRAY);
        }
        for (auto& i : m_Entities)
            i->draw();
        DrawCircleV({0, 0}, 1.f, BLUE); // rect node
    }
    
    void testInput() {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            auto mpos = GetMousePosition();
            
            mathpls::vec2 pos = {mpos.x - win_ext.x * .5f, mpos.y - win_ext.y * .5f};
            pos *= .1f;
            
            addMass(pos);
        }
        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
            auto mpos = GetMousePosition();
            
            mathpls::vec2 pos = {mpos.x - win_ext.x * .5f, mpos.y - win_ext.y * .5f};
            pos *= .1f;
            
            m_World.Gravity.func = [=](const pxpls::Rigidbody* rb, mathpls::vec2& acc) {
                auto d = (pos - rb->Position()).normalized();
                auto r2 = d.length_squared();
                acc += d / r2 * 514.f * (1 - std::exp(-r2 * 191.f));
            };
        } else if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON)) {
            m_World.Gravity.func = nullptr;
        }
    }
    
    void testLink(Entity& mass) {
        pxpls::QuadTree qt{{-100.f, 100.f}};
        
        const auto& map = m_World.GetCollisionBodies();
        qt.Update(map);
        auto pairs = qt.GetCollisionPairs();
        
        const auto id = mass.Rigidbody->id;
        
        for (auto& i : pairs | std::views::filter([&](auto&& p){
            return (p.first == id || p.second == id) &&
                    isMass(map.at(p.first)) && isMass(map.at(p.second));
        })) {
            const auto& a = map.at(i.first);
            const auto& b = map.at(i.second);
            if (mathpls::distance_quared(a->Position(), b->Position()) < search_radius2) {
                m_Links.emplace_back((pxpls::Rigidbody*)a, (pxpls::Rigidbody*)b);
                m_World.AddSpring({m_Links.back(), k, rest_len, dashpot});
            }
        }
    }
    
    static bool isMass(const pxpls::CollisionBody* e) {
        return e->Collider != nullptr; // link doesnt have colloder;
    }
    
private:
    float search_radius2 = 144; // search_radius^2
    float rest_len = 7;
    float k = 1000;
    float dashpot = 100;
    
    pxpls::ImpulseSolver solver1;
    pxpls::SmoothPositionSolver solver2;
    
    pxpls::DynamicsWorld m_World;
    std::vector<std::unique_ptr<Entity>> m_Entities;
    std::vector<pxpls::Link> m_Links;
    
    pxpls::Rigidbody rectRB;
    pxpls::AabbColloder rectCol;
    
};

int main(int argc, const char * argv[]) {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    
    InitWindow(win_ext.x, win_ext.y, "pxpls demo 2D: spring");
    
    Camera2D cam{};
    cam.offset = {win_ext.x * .5f, win_ext.y * .5f};
    cam.zoom = 10.f;
    
    SetTargetFPS(60);
    rlEnableSmoothLines();
    
    Game game;
    
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(WHITE);
        
        BeginMode2D(cam);
        
        rlPushMatrix();{
            rlTranslatef(0, 25*5, 0);
            rlRotatef(90, 1, 0, 0);
            DrawGrid(100, 5);
        }rlPopMatrix();
        
        game.tick(dt);
        
        EndMode2D();
        
        DrawFPS(0, 0);
        EndDrawing();
    }
    
    CloseWindow();
    
    return 0;
}
