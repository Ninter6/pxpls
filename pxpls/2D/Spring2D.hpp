//
//  Spring2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/12.
//

#pragma once

#include <functional>

namespace pxpls {

class Rigidbody;

struct Link {
    Link() = default;
    Link(Rigidbody* a, Rigidbody* b) : A(a), B(b) {}
    
    bool operator==(const Link& o) const {
        return A == o.A && B == o.B;
    }
    
    Rigidbody* A;
    Rigidbody* B;
};

/**
 * \brief Spring struct
 * hold tow Rigidbody pointer
 * the spring itself will not collide
 */
struct Spring {
    Spring() = default;
    Spring(Rigidbody* a, Rigidbody* b, float k, float rest_len)
    : Link(a, b), K(k), RestLength(rest_len) {}
    
    pxpls::Link Link;
    
    float K; // elastic coefficient
    float RestLength;
    
    Rigidbody* A() const {return this->Link.A;}
    Rigidbody* B() const {return this->Link.B;}
    
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
