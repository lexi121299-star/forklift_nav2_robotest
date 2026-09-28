// Remove isolated occupied components from a ROS P5 occupancy map.
// Usage: cleanup_static_map INPUT.pgm OUTPUT.pgm MIN_COMPONENT_PIXELS

#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string readToken(std::istream & input)
{
  std::string token;
  char character = '\0';
  while (input.get(character)) {
    if (character == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    if (!std::isspace(static_cast<unsigned char>(character))) {
      token.push_back(character);
      break;
    }
  }
  while (input.get(character) && !std::isspace(static_cast<unsigned char>(character))) {
    token.push_back(character);
  }
  return token;
}

unsigned int parseUnsigned(const std::string & token, const char * name)
{
  try {
    const unsigned long value = std::stoul(token);
    if (value == 0U || value > 100000000U) {
      throw std::out_of_range("range");
    }
    return static_cast<unsigned int>(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("Invalid ") + name + ": " + token);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0]
              << " INPUT.pgm OUTPUT.pgm MIN_COMPONENT_PIXELS\n";
    return 2;
  }

  std::ifstream input(argv[1], std::ios::binary);
  if (!input) {
    std::cerr << "Cannot open input map: " << argv[1] << '\n';
    return 1;
  }
  if (readToken(input) != "P5") {
    std::cerr << "Only binary P5 PGM maps are supported\n";
    return 1;
  }

  const unsigned int width = parseUnsigned(readToken(input), "width");
  const unsigned int height = parseUnsigned(readToken(input), "height");
  const unsigned int max_value = parseUnsigned(readToken(input), "max value");
  const unsigned int min_component_pixels = parseUnsigned(argv[3], "minimum component size");
  if (max_value != 255U) {
    std::cerr << "Only 8-bit PGM maps are supported\n";
    return 1;
  }

  const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
  std::vector<std::uint8_t> pixels(pixel_count);
  input.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
  if (static_cast<std::size_t>(input.gcount()) != pixels.size()) {
    std::cerr << "PGM ended before all pixels were read\n";
    return 1;
  }

  // ROS trinary maps use occupied_thresh=0.65 by default: 0..165 is occupied.
  constexpr std::uint8_t kOccupiedThreshold = 165;
  constexpr std::uint8_t kFreeValue = 254;
  std::vector<bool> visited(pixel_count, false);
  std::vector<std::size_t> component;
  std::queue<std::size_t> pending;
  std::size_t removed_components = 0U;
  std::size_t removed_pixels = 0U;
  std::size_t retained_components = 0U;

  const std::array<int, 8> dx{{-1, 0, 1, -1, 1, -1, 0, 1}};
  const std::array<int, 8> dy{{-1, -1, -1, 0, 0, 1, 1, 1}};
  for (std::size_t start = 0U; start < pixel_count; ++start) {
    if (visited[start] || pixels[start] > kOccupiedThreshold) {
      continue;
    }
    component.clear();
    pending.push(start);
    visited[start] = true;
    while (!pending.empty()) {
      const std::size_t index = pending.front();
      pending.pop();
      component.push_back(index);
      const int x = static_cast<int>(index % width);
      const int y = static_cast<int>(index / width);
      for (std::size_t neighbor = 0U; neighbor < dx.size(); ++neighbor) {
        const int next_x = x + dx[neighbor];
        const int next_y = y + dy[neighbor];
        if (next_x < 0 || next_y < 0 || next_x >= static_cast<int>(width) ||
          next_y >= static_cast<int>(height))
        {
          continue;
        }
        const std::size_t next = static_cast<std::size_t>(next_y) * width +
          static_cast<std::size_t>(next_x);
        if (!visited[next] && pixels[next] <= kOccupiedThreshold) {
          visited[next] = true;
          pending.push(next);
        }
      }
    }

    if (component.size() < min_component_pixels) {
      for (const std::size_t index : component) {
        pixels[index] = kFreeValue;
      }
      ++removed_components;
      removed_pixels += component.size();
    } else {
      ++retained_components;
    }
  }

  std::ofstream output(argv[2], std::ios::binary);
  if (!output) {
    std::cerr << "Cannot create output map: " << argv[2] << '\n';
    return 1;
  }
  output << "P5\n" << width << ' ' << height << "\n255\n";
  output.write(reinterpret_cast<const char *>(pixels.data()),
    static_cast<std::streamsize>(pixels.size()));
  if (!output) {
    std::cerr << "Failed while writing output map\n";
    return 1;
  }

  std::cout << "Removed " << removed_components << " occupied components ("
            << removed_pixels << " pixels); retained " << retained_components
            << " components at threshold " << min_component_pixels << " pixels.\n";
  return 0;
}
