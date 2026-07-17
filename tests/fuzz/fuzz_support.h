#pragma once

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#ifndef BALANCER_REPO_ROOT
#error "BALANCER_REPO_ROOT must be defined for fuzz harnesses"
#endif

namespace fuzz {

inline std::string repo_path(const char* relative_path) {
  return std::string(BALANCER_REPO_ROOT) + "/" + relative_path;
}

inline bool read_binary_file(const char* path, std::vector<uint8_t>& out) {
  const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0 || st.st_size < 0) {
    ::close(fd);
    return false;
  }

  out.resize(static_cast<std::size_t>(st.st_size));
  std::size_t offset = 0;
  while (offset < out.size()) {
    const ssize_t n = ::read(fd, out.data() + offset, out.size() - offset);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      ::close(fd);
      return false;
    }
    if (n == 0) {
      break;
    }
    offset += static_cast<std::size_t>(n);
  }

  ::close(fd);
  if (offset != out.size()) {
    out.resize(offset);
  }
  return true;
}

template <typename T>
inline void copy_prefix(const std::vector<uint8_t>& bytes, T& out) {
  static_assert(std::is_trivially_copyable_v<T>);
  out = T{};
  if (!bytes.empty()) {
    std::memcpy(&out, bytes.data(), std::min(bytes.size(), sizeof(T)));
  }
}

}  // namespace fuzz
