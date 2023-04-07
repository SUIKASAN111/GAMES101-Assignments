#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        Vector2D step = (end - start) / (num_nodes - 1);
        for(int i = 0; i < num_nodes; ++i){
            masses.push_back(new Mass(start + step * i, node_mass, false));
            if(i > 0){
                springs.push_back(new Spring(masses[i-1], masses[i], k));
            }
        }
        for(auto &i : pinned_nodes){
           masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D dis = s->m1->position - s->m2->position;// 两质量距离矢量，m2指向m1
            float curr_length = dis.norm();// 弹簧当前长度
            Vector2D force = s->k * (curr_length - s->rest_length) * dis / curr_length;// 对于m2的力矢量
            s->m1->forces -= force;
            s->m2->forces += force;

            // Internal Damping
            Vector2D D_V = s->m1->velocity - s->m2->velocity;
            Vector2D disNorm = dis / curr_length;
            float k_d = 0.05f;
            Vector2D force_a = -k_d * (disNorm.x * D_V.x + disNorm.y * D_V.y) * dis / curr_length;
            s->m1->forces += force_a;
            s->m2->forces -= force_a;

            // Air Damping
            s->m1->forces += -0.005f * s->m1->velocity;
            s->m2->forces += -0.005f * s->m2->velocity;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                m->forces += gravity * m->mass;// 加上重力
                Vector2D a = m->forces / m->mass;// 加速度a=F/m
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dis = s->m1->position - s->m2->position;// 两质量距离矢量，m2指向m1
            float curr_length = dis.norm();// 弹簧当前长度
            Vector2D force = s->k * (curr_length - s->rest_length) * dis / curr_length;// 对于m2的力矢量
            s->m1->forces -= force;
            s->m2->forces += force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                // TODO (Part 4): Add global Verlet damping
                float damping_factor = 0.00005;
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position)
                                + a * delta_t * delta_t;
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0,0);
        }
    }
}
