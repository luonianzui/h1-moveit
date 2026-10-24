#include <fcl/fcl.h>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>

using Surface = fcl::BVHModel<fcl::OBBRSSd>;

extern "C" void* create_surface(const double* triangles, std::size_t count) {
  auto* surface = new Surface;
  surface->beginModel(count, count * 3);
  for (std::size_t index = 0; index < count; ++index) {
    const double* values = triangles + index * 9;
    surface->addTriangle(fcl::Vector3d(values[0], values[1], values[2]),
                         fcl::Vector3d(values[3], values[4], values[5]),
                         fcl::Vector3d(values[6], values[7], values[8]));
  }
  surface->endModel();
  return surface;
}

extern "C" int query_surface(void* pointer, const double* matrix,
                             const double* dimensions, double* records, int capacity) {
  auto* surface = static_cast<Surface*>(pointer);
  fcl::Transform3d pose = fcl::Transform3d::Identity();
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column)
      pose.matrix()(row, column) = matrix[row * 4 + column];
  fcl::Boxd box(dimensions[0], dimensions[1], dimensions[2]);
  fcl::CollisionRequestd request(capacity, true);
  fcl::CollisionResultd result;
  fcl::collide(surface, fcl::Transform3d::Identity(), &box, pose, request, result);
  std::vector<fcl::Contactd> contacts;
  result.getContacts(contacts);
  int count = std::min(capacity, static_cast<int>(contacts.size()));
  for (int index = 0; index < count; ++index) {
    auto& contact = contacts[index];
    records[index * 8] = contact.penetration_depth;
    for (int axis = 0; axis < 3; ++axis) {
      records[index * 8 + axis + 1] = contact.pos[axis];
      records[index * 8 + axis + 4] = contact.normal[axis];
    }
    records[index * 8 + 7] = contact.b1;
  }
  return count;
}

extern "C" void delete_surface(void* pointer) {
  delete static_cast<Surface*>(pointer);
}
