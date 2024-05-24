#include "sphere.h"

#include <Eigen/Dense>

#include "cloth.h"
#include "configs.h"

namespace {
void generateVertices(std::vector<GLfloat>& vertices, std::vector<GLuint>& indices) {
  // See http://www.songho.ca/opengl/gl_sphere.html#sphere if you don't know how to create a sphere.
  vertices.reserve(8 * (sphereStack + 1) * (sphereSlice + 1));
  indices.reserve(6 * sphereStack * sphereSlice);

  float x, y, z, xy;  //  position

  float sectorStep = static_cast<float>(EIGEN_PI * 2 / sphereSlice);
  float stackStep = static_cast<float>(EIGEN_PI / sphereStack);
  float sectorAngle, stackAngle;

  for (int i = 0; i <= sphereStack; ++i) {
    stackAngle = static_cast<float>(EIGEN_PI / 2 - i * stackStep);  // [pi/2, -pi/2]
    xy = cosf(stackAngle);                                          // r * cos(u)
    z = sinf(stackAngle);                                           // r * sin(u)

    for (int j = 0; j <= sphereSlice; ++j) {
      sectorAngle = j * sectorStep;  // [0, 2pi]

      x = xy * cosf(sectorAngle);  // r * cos(u) * cos(v)
      y = xy * sinf(sectorAngle);  // r * cos(u) * sin(v)
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }

  unsigned int k1, k2;  // EBO index
  for (int i = 0; i < sphereStack; ++i) {
    k1 = i * (sphereSlice + 1);  // beginning of current sphereStack
    k2 = k1 + sphereSlice + 1;   // beginning of next sphereStack
    for (int j = 0; j < sphereSlice; ++j, ++k1, ++k2) {
      if (i != 0) {
        indices.insert(indices.end(), {k1, k2, k1 + 1});
      }
      // k1+1 => k2 => k2+1
      if (i != (sphereStack - 1)) {
        indices.insert(indices.end(), {k1 + 1, k2, k2 + 1});
      }
    }
  }
}
}  // namespace

Spheres& Spheres::initSpheres() {
  static Spheres spheres;
  return spheres;
}

void Spheres::addSphere(const Eigen::Ref<const Eigen::Vector4f>& position, float size) {
  if (sphereCount == _particles.getCapacity()) {
    _particles.resize(sphereCount * 2);
    _radius.resize(sphereCount * 2);
    offsets.allocate(8 * sphereCount * sizeof(float));
    sizes.allocate(2 * sphereCount * sizeof(float));
  }
  _radius[sphereCount] = size;
  _particles.position(sphereCount) = position;
  _particles.velocity(sphereCount).setZero();
  _particles.acceleration(sphereCount).setZero();
  _particles.mass(sphereCount) = sphereDensity * size * size * size;

  sizes.load(0, _radius.size() * sizeof(float), _radius.data());
  ++sphereCount;
}

Spheres::Spheres() : Shape(1, 1), sphereCount(0), _radius(1, 0.0f) {
  offsets.allocate(4 * sizeof(float));
  sizes.allocate(sizeof(float));

  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  generateVertices(vertices, indices);

  vbo.allocate_load(vertices.size() * sizeof(GLfloat), vertices.data());
  ebo.allocate_load(indices.size() * sizeof(GLuint), indices.data());

  vao.bind();
  vbo.bind();
  ebo.bind();

  vao.enable(0);
  vao.setAttributePointer(0, 3, 6, 0);
  glVertexAttribDivisor(0, 0);
  vao.enable(1);
  vao.setAttributePointer(1, 3, 6, 3);
  glVertexAttribDivisor(1, 0);
  offsets.bind();
  vao.enable(2);
  vao.setAttributePointer(2, 3, 4, 0);
  glVertexAttribDivisor(2, 1);
  sizes.bind();
  vao.enable(3);
  vao.setAttributePointer(3, 1, 1, 0);
  glVertexAttribDivisor(3, 1);

  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Spheres::draw() const {
  vao.bind();
  offsets.load(0, 4 * sphereCount * sizeof(GLfloat), _particles.getPositionData());
  GLsizei indexCount = static_cast<GLsizei>(ebo.size() / sizeof(GLuint));
  glDrawElementsInstanced(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr, sphereCount);
  glBindVertexArray(0);
}

void Spheres::collide(Shape* shape) { shape->collide(this); }
void Spheres::collide(Cloth* cloth) {
  constexpr float coefRestitution = 0.0f;
  // TODO: Collide with particle (Simple approach to handle softbody collision)
  //   1. Detect collision.
  //   2. If collided, update impulse directly to particles' velocity
  // Note:
  //   1. There are `sphereCount` spheres (sphereCount is 1 in the default scene).
  //   2. There are `particlesPerEdge * particlesPerEdge` particles.
  //   3. See TODOs in Cloth::computeSpringForce if you don't know how to access data.
  for (int k = 0; k < sphereCount; k++) {
    for (int i = 0; i < particlesPerEdge * particlesPerEdge; i++) {
      float dis = (_particles.position(k) - cloth->particles().position(i)).norm();
      Eigen::Vector4f normal = (_particles.position(k) - cloth->particles().position(i)).normalized();
      if (dis - radius(k) < 0.05f && ( _particles.velocity(k) - cloth->particles().velocity(i)).dot(normal) < 0 ){
        Eigen::Vector4f tangent_v_sphere = _particles.velocity(k) - (_particles.velocity(k).dot(normal) * normal);
        Eigen::Vector4f normal_v_sphere = (_particles.velocity(k).dot(normal) * normal);
        Eigen::Vector4f tangent_v_cloth =
            cloth->particles().velocity(i) - (cloth->particles().velocity(i).dot(normal) * normal);
        Eigen::Vector4f normal_v_cloth = (cloth->particles().velocity(i).dot(normal) * normal);
        Eigen::Vector4f va = (_particles.mass(k) * normal_v_sphere + cloth->particles().mass(i) * normal_v_cloth +
                              cloth->particles().mass(i) * coefRestitution * (normal_v_cloth - normal_v_sphere)) /
                             (_particles.mass(k) + cloth->particles().mass(i));
        Eigen::Vector4f vb = (_particles.mass(k) * normal_v_sphere + cloth->particles().mass(i) * normal_v_cloth +
                              _particles.mass(k) * coefRestitution * (normal_v_sphere - normal_v_cloth)) /
                             (_particles.mass(k) + cloth->particles().mass(i));

        _particles.velocity(k) = va + tangent_v_sphere;
        cloth->particles().velocity(i) = vb + tangent_v_cloth;
        if (dis <= radius(k)) {
          auto penetration = this->radius(k) - dis;
          Eigen::Vector4f correct = penetration * normal * 0.5;
          cloth->particles().position(i) -= correct;
          _particles.position(k) += correct;
        }
      }
    }
  } 
}

void Spheres::setVelocity(int i, const Eigen::Vector4f vel) { _particles.velocity(i) = vel; }
