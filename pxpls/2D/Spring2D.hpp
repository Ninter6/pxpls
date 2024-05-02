//
//  Spring2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/12.
//

#pragma once

#include <functional>

namespace pxpls {

class Rigidbody2D;

struct Link {
    Link() = default;
    Link(Rigidbody2D* a, Rigidbody2D* b) : A(a), B(b) {}
    
    bool operator==(const Link& o) const {
        return A == o.A && B == o.B;
    }
    
    Rigidbody2D* A;
    Rigidbody2D* B;
};

/**
 * \brief Spring struct
 * hold tow Rigidbody pointer
 * the spring itself will not collide
 */
struct Spring {
    Spring() = default;
    Spring(const Link& link, float k, float rest_len, float dashpot = 0)
    : Link(link), K(k), RestLength(rest_len), Dashpot(dashpot) {}
    Spring(Rigidbody2D* a, Rigidbody2D* b, float k, float rest_len, float dashpot = 0)
    : Link(a, b), K(k), RestLength(rest_len), Dashpot(dashpot) {}
    
    pxpls::Link Link;
    
    float K; // elastic coefficient
    float RestLength;
    
    float Dashpot; // Dashpot damping
    
    Rigidbody2D* A() const {return this->Link.A;}
    Rigidbody2D* B() const {return this->Link.B;}
    
    /**
     * \brief Use Hooke's law to calculate the force and apply to each rigidbody
     */
    void CalcuForce() const;
};

}

template <>
struct std::hash<pxpls::Link> {
    size_t operator()(const pxpls::Link& l) const {
        size_t a = std::hash<void*>{}(l.A);
        size_t b = std::hash<void*>{}(l.B);
        return a + b + 1145141919810u ^ a * b;
    }
};
