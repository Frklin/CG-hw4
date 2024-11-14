//
// Implementation for Yocto/Particle.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#include "yocto_particle.h"

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shape.h>

#include <unordered_set>

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

particle_scene make_ptscene(
    const scene_data& ioscene, const particle_params& params) {
  // scene
  auto ptscene = particle_scene{};

  // shapes
  static auto velocity = unordered_map<string, float>{
      {"floor", 0}, {"particles", 1}, {"cloth", 0}, {"collider", 0}};
  auto iid = 0;
  for (auto& ioinstance : ioscene.instances) {
    auto& ioshape         = ioscene.shapes[ioinstance.shape];
    auto& iomaterial_name = ioscene.material_names[ioinstance.material];
    if (iomaterial_name == "particles") {
      add_particles(ptscene, iid++, ioshape.points, ioshape.positions,
          ioshape.radius, 1, 1);
    } else if (iomaterial_name == "cloth") {
      auto nverts = (int)ioshape.positions.size();
      add_cloth(ptscene, iid++, ioshape.quads, ioshape.positions,
          ioshape.normals, ioshape.radius, 0.5, 1 / 8000.0,
          {nverts - 1, nverts - (int)sqrt((float)nverts)});
    } else if (iomaterial_name == "collider") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else if (iomaterial_name == "floor") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else {
      throw std::invalid_argument{"unknown material " + iomaterial_name};
    }
  }

  // done
  return ptscene;
}

void update_ioscene(scene_data& ioscene, const particle_scene& ptscene) {
  for (auto& ptshape : ptscene.shapes) {
    auto& ioshape = ioscene.shapes[ptshape.shape];
    get_positions(ptshape, ioshape.positions);
    get_normals(ptshape, ioshape.normals);
  }
}

void flatten_scene(scene_data& ioscene) {
  for (auto& ioinstance : ioscene.instances) {
    auto& shape = ioscene.shapes[ioinstance.shape];
    for (auto& position : shape.positions)
      position = transform_point(ioinstance.frame, position);
    for (auto& normal : shape.normals)
      normal = transform_normal(ioinstance.frame, normal);
    ioinstance.frame = identity3x4f;
  }
}

// Scene creation
int add_particles(particle_scene& scene, int shape_id,
    const vector<int>& points, const vector<vec3f>& positions,
    const vector<float>& radius, float mass, float random_velocity) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.points            = points;
  shape.initial_positions = positions;
  shape.initial_normals.assign(shape.positions.size(), {0, 0, 1});
  shape.initial_radius = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.emit_rngscale = random_velocity;
  return (int)scene.shapes.size() - 1;
}
int add_cloth(particle_scene& scene, int shape_id, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius, float mass, float coeff,
    const vector<int>& pinned) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.quads             = quads;
  shape.initial_positions = positions;
  shape.initial_normals   = normals;
  shape.initial_radius    = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.initial_pinned = pinned;
  shape.spring_coeff   = coeff;
  return (int)scene.shapes.size() - 1;
}
int add_collider(particle_scene& scene, int shape_id,
    const vector<vec3i>& triangles, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius) {
  auto& collider     = scene.colliders.emplace_back();
  collider.shape     = shape_id;
  collider.quads     = quads;
  collider.triangles = triangles;
  collider.positions = positions;
  collider.normals   = normals;
  collider.radius    = radius;
  return (int)scene.colliders.size() - 1;
}

// Set shapes
void set_velocities(
    particle_shape& shape, const vec3f& velocity, float random_scale) {
  shape.emit_velocity = velocity;
  shape.emit_rngscale = random_scale;
}

// Get shape properties
void get_positions(const particle_shape& shape, vector<vec3f>& positions) {
  positions = shape.positions;
}
void get_normals(const particle_shape& shape, vector<vec3f>& normals) {
  normals = shape.normals;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

// Init simulation
void init_simulation(particle_scene& scene, const particle_params& params) {
  auto sid = 0;
  for (auto& shape : scene.shapes) {
    shape.emit_rng = make_rng(params.seed, (sid++) * 2 + 1);
    // initialize state
    shape.invmass    = shape.initial_invmass;
    shape.normals    = shape.initial_normals;
    shape.positions  = shape.initial_positions;
    shape.radius     = shape.initial_radius;
    shape.velocities = shape.initial_velocities;

    // initialize forces
    shape.forces.clear();
    for (int i = 0; i < shape.positions.size(); i++)
      shape.forces.push_back(zero3f);

    // initialize pinned particles
    for (auto index : shape.initial_pinned) 
      shape.invmass[index] = 0;
    

    // initialize velocities
    for (auto& velocity : shape.velocities) 
      velocity += sample_sphere(rand2f(shape.emit_rng)) * shape.emit_rngscale *rand1f(shape.emit_rng);
    

    // clear springs array
    shape.springs.clear();

    // initialize springs
    if (shape.spring_coeff > 0) 
      // add springs for each edge
      for (auto& edge : get_edges(shape.quads)) 
        shape.springs.push_back({edge.x, edge.y,distance(shape.positions[edge.x], shape.positions[edge.y]),shape.spring_coeff});
      
      // add 2 diagonal springs foreach square
      for (auto& quad : shape.quads) {
        shape.springs.push_back({quad.x, quad.z,distance(shape.positions[quad.x], shape.positions[quad.z]),shape.spring_coeff});
        shape.springs.push_back({quad.y, quad.w,distance(shape.positions[quad.y], shape.positions[quad.w]),shape.spring_coeff});
      }
  }
  for (auto& collider : scene.colliders) {
    // intiialize bvh
    collider.bvh = {};
    // add quads or triangles to bvh
    if (collider.quads.size() > 0)
      collider.bvh = make_quads_bvh(collider.quads, collider.positions, collider.radius);
    else
      collider.bvh = make_triangles_bvh(collider.triangles, collider.positions, collider.radius);
  }
}


// check if a point is inside a collider
bool collide_collider(const particle_collider& collider, const vec3f& position,
    vec3f& hit_position, vec3f& hit_normal) {
  auto                           ray = ray3f{position, vec3f{0, 1, 0}};
  shape_intersection isec;

  // call the appropriate intersect method
  if (collider.quads.size() > 0)
    isec = intersect_quads_bvh(collider.bvh, collider.quads, collider.positions, ray);
  else
    isec = intersect_triangles_bvh(collider.bvh, collider.triangles, collider.positions, ray);
  if (!isec.hit) return false;

  // calculate hit position and normal
  if (collider.quads.size() > 0) {
    auto q       = collider.quads[isec.element];
    hit_position = interpolate_quad(collider.positions[q.x],collider.positions[q.y], collider.positions[q.z],collider.positions[q.w], isec.uv);
    hit_normal   = normalize( interpolate_quad(collider.normals[q.x], collider.normals[q.y], collider.normals[q.z], collider.normals[q.w], isec.uv));
  } else {
    auto q       = collider.triangles[isec.element];
    hit_position = interpolate_triangle(collider.positions[q.x],collider.positions[q.y], collider.positions[q.z], isec.uv);
    hit_normal   = normalize(interpolate_triangle(collider.normals[q.x],collider.normals[q.y], collider.normals[q.z], isec.uv));
  }
  // return the angle between hit hormal and ray direction
  return dot(hit_normal, ray.d) > 0;
}

// simulate mass-spring
void simulate_massspring(particle_scene& scene, const particle_params& params) {
  // SAVE OLD POSITIONS
  for (auto& shape : scene.shapes) 
    shape.old_positions = shape.positions;
  

  // COMPUTE DYNAMICS
  for (auto& shape : scene.shapes) 
    for (int i = 0; i < params.mssteps; i++) {
      auto ddt = params.deltat / params.mssteps;

      // Compute forces
      for (int k = 0; k < shape.positions.size(); k++) {
        if (!shape.invmass[k]) continue;
        shape.forces[k] = vec3f{0, -params.gravity, 0} / shape.invmass[k];
      }

      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];
        auto  invmass   = shape.invmass[spring.vert0] + shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto delta_pos = particle1 - particle0;
        auto delta_vel = shape.velocities[spring.vert1] - shape.velocities[spring.vert0];

        auto spring_dir = normalize(delta_pos);
        auto spring_len = length(delta_pos);

        auto force = spring_dir * (spring_len / spring.rest - 1.f) /(spring.coeff * invmass);
        force += dot(delta_vel / spring.rest, spring_dir) * spring_dir / (spring.coeff * 1000 * invmass);

        shape.forces[spring.vert0] += force;
        shape.forces[spring.vert1] -= force;
      }

      for (int k = 0; k < shape.positions.size(); k++) {
        if (!shape.invmass[k]) continue;
        shape.velocities[k] += ddt * shape.forces[k] * shape.invmass[k];
        shape.positions[k] += ddt * shape.velocities[k];
      }
    }

  // Collision detection
  for (auto& shape : scene.shapes) 
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (collide_collider(collider, shape.positions[k], hitpos, hit_normal)) {
          shape.positions[k] = hitpos + hit_normal * 0.005f;
          auto projection     = dot(shape.velocities[k], hit_normal);
          shape.velocities[k] =(shape.velocities[k] - projection * hit_normal) *(1.f - params.bounce.x) - projection * hit_normal * (1.f - params.bounce.y);
        }
      }
    }

  // ADJUST VELOCITY
  for (auto& shape : scene.shapes) 
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  

  // RECOMPUTE NORMALS
  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

// simulate pbd
void simulate_pbd(particle_scene& scene, const particle_params& params) {
  // SAVE OLD POSITOINS
  for (auto& shape : scene.shapes) 
    shape.old_positions = shape.positions;
 

    


  auto wind      = params.windPower * vec3f{1.5, 0, 1};


  // PREDICT POSITIONS
  
  for (auto& shape : scene.shapes) 
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      auto tornado = zero3f, vortix = zero3f;
      if (params.vortixPower) {

        auto vortixOrigin = vec3f{params.vortixDirection.x,
            shape.positions[k].y, params.vortixDirection.z};
        auto d            = distance(shape.positions[k], vortixOrigin);
        auto direction    = normalize(shape.positions[k] - vortixOrigin);
        auto vortixPower  = params.vortixPower / pow2(d);
        vortix = normalize(cross(vortixOrigin, direction)) * vortixPower;
      }
      if (params.TornadoPower) {
          auto h             = shape.positions[k].y;
          auto tornadoOrigin = vec3f {params.tornadoOrigin.x,h,params.tornadoOrigin.z};
          auto point = ray_point(ray3f{tornadoOrigin,normalize(shape.positions[k]-tornadoOrigin)},log(h+1));// vec3f{log(h)-1,h,};
          auto d = distance(shape.positions[k],point);
          vec3f xvec;
          vec3f yvec = vec3f{0, 1, 0} * params.TornadoPower;
          vec3f zvec = normalize(cross(tornadoOrigin, normalize(shape.positions[k] - tornadoOrigin)));
          if (distance(shape.positions[k],tornadoOrigin) < distance(point,tornadoOrigin)) {
             xvec = normalize(point - shape.positions[k]);
          } else {
            xvec = normalize(tornadoOrigin - point);

          }
           tornado = zvec + xvec + yvec;

      }
      

      shape.velocities[k] += (tornado + vortix + wind +vec3f{0, -params.gravity, 0}) * params.deltat;
      shape.positions[k] += shape.velocities[k] * params.deltat;     
    }
  

  // DETECT COLLISIONS
  for (auto& shape : scene.shapes) {
    // clear collisions array
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hit_position = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hit_position, hit_normal))
          continue;
        // if there is a collision, push it back
        shape.collisions.push_back({k, hit_position, hit_normal});
        
      }
    }
  }

  // SOLVE CONSTRAINTS
  for (auto& shape : scene.shapes) 
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];
        
        auto invmass = shape.invmass[spring.vert0] + shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        // calculate len and normalize dir
        auto len = length(dir);
        dir /= len;

        // calculate lambda
        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda * dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda * dir;
      }

      // handle collisions
      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  

  // COMPUTE VELOCITIES
  for (auto& shape : scene.shapes) 
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) / params.deltat;
    }
  

  // ADJUST VELOCITY
  for (auto& shape : scene.shapes) 
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      //dumping
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      //sleeping
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = zero3f;
    }
  

  // RECOMPUTE NORMALS
  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}



// Simulate one step
void simulate_frame(particle_scene& scene, const particle_params& params) {
  switch (params.solver) {
    case particle_solver_type::mass_spring:
      return simulate_massspring(scene, params);
    case particle_solver_type::position_based:
      return simulate_pbd(scene, params);
    default: throw std::invalid_argument("unknown solver");
  }
}

// Simulate the whole sequence
void simulate_frames(particle_scene& scene, const particle_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)params.frames};

  if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
  init_simulation(scene, params);

  for (auto idx = 0; idx < params.frames; idx++) {
    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
    simulate_frame(scene, params);
  }

  if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
}

float sdSphere(vec3f p, float s) { return length(p) - s; }
float sdBox(vec3f p, vec3f b) {
  vec3f q = abs(p) - b;
  return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0f);
}
float sdPlane(vec3f p, vec3f n, float h) {
  // n must be normalized
  return dot(p, n) + h;
}

/*
void hp_SDF(vector<vec3f>& positions, float threshold, int p_max, int l_max) { 
    hash_grid grid = make_hash_grid(positions,0.001); //cell size
    float mu_e,nu_e,epsilon = 0.0;
    int p,l,n = 10, passo = 0; //nx ny nz
    bool refine_p,refine_h;
    couple max;
    vector<couple> pending = {}; //pending
    vector<float>  polynomials; 

    for (int e: range(positions.size())) {
      fit_polynomial(polynomials,e,2);   // da fare 
      auto epsilon_e = estimate_error(e); //da fare
      epsilon += epsilon_e;
      pending.push_back({e, epsilon_e});
    }

    
    while (!pending.empty() && epsilon > threshold) {
      make_heap(pending.begin(), pending.end() - passo,newcompare);
      max   = pending.front();  
      pop_heap(pending.begin(), pending.end() - passo++);

      //int e = max.e;
      p = degree(max.e);  //da fare 
      l = level(max.e);   // da fare 
      
      mu_e = estimate_improvement_p(max.e, p);
      nu_e = estimate_improvement_h(max.e, p);

      refine_p = p < p_max && (1 == l_max || mu_e > nu_e);
      refine_h = l < l_max && !refine_p;

      if (refine_p) {
        fit_polynomial(max.e, p + 1);
        epsilon -= max.epsilon;
        auto epsilon_e = estimate_error(max.e);
        epsilon += epsilon_e;
        pending.push_back({max.e, epsilon_e});
      } 
      else if (refine_h) {
        auto children = subdivide(max.e);
        epsilon -= max.epsilon;
        for (int j : range(children)) {
          fit_polynomial(j, p);
          auto epsilon_j = estimate_error(j);
          epsilon += epsilon_j;
          pending.push_back({j, epsilon_j});
        }
      } 
    }

}

void fit_polynomial(vector<float>& p,int e, int grade) { 
    p.push_back(integral()}

float estimate_error(int e) { return 2.0; }

int degree(int e) { return 2; }

int level(int e) { return 1; }

float estimate_improvement_p(int e, int p) { 
    auto d = n_c(p + 1) - n_c(p);
    auto n = pow(epsilon_e, p) - 8 * pow(epsilon_e, p + 1);
    return 1./d*n; };
float estimate_improvement_h(int e, int p) { 
    auto d = 7 * n_c(p);
    auto n = pow(epsilon_e, p) - 8 * max(epsilon_c, p);
    return 1./d*n; };

float n_c(int p) { return std::pow(p+1,3)/*1./6*(6+11*p+6*p*p+p*p*p); }
*/

}  // namespace yocto
