#include "selfdrive/ui/replay/filereader.h"

#include <fstream>
#include <iostream>

#include "selfdrive/common/util.h"
#include "selfdrive/ui/replay/util.h"

std::string FileReader::read(const std::string &file, std::atomic<bool> *abort) {
  const bool is_remote = file.find("https://") == 0;
  const std::string local_file = file;
  std::string result;

  if ((!is_remote || cache_to_local_) && util::file_exists(local_file)) {
    result = util::read_file(local_file);
  } else if (is_remote) {
    result = download(file, abort);
    if (cache_to_local_ && !result.empty()) {
      std::ofstream fs(local_file, std::ios::binary | std::ios::out);
      fs.write(result.data(), result.size());
    }
  }
  return result;
}

std::string FileReader::download(const std::string &url, std::atomic<bool> *abort) {
  for (int i = 0; i <= max_retries_ && !(abort && *abort); ++i) {
    std::string result = httpGet(url, chunk_size_, abort);
    if (!result.empty()) {
      return result;
    }
    if (i != max_retries_) {
      std::cout << "download failed, retrying " << i + 1 << std::endl;
    }
  }
  return {};
}
