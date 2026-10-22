#include "h1_2_moveit_demo/door_handoff_path_certificate.hpp"

#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 4 || std::string(argv[1]) != "inspect-format") {
    std::cerr << "usage: door_handoff_path_certificate_tool inspect-format "
              << "MANIFEST PAYLOAD\n";
    return 2;
  }

  const auto result =
    h1_2_moveit_demo::door_handoff_path_certificate::inspect_format_files(
    argv[2], argv[3]);
  if (!result.passed) {
    std::cerr << "DOOR_HANDOFF_PATH_CERTIFICATE_FORMAT_ONLY=FAIL: "
              << result.message << '\n';
    return 1;
  }
  std::cout << "DOOR_HANDOFF_PATH_CERTIFICATE_FORMAT_ONLY=PASS: "
            << result.message << "; states="
            << result.certificate.ingress.size() << "/"
            << result.certificate.coarse.size() << "/"
            << result.certificate.exact.size() << ", anchor="
            << result.certificate.selected_anchor_index
            << "; live model/collision/STL/hinge/timing gates not evaluated\n";
  return 0;
}
