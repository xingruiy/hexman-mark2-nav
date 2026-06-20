#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include <orb_slam3/ORBVocabulary.h>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr,
                 "usage: %s <ORBvoc.txt> <ORBvoc.bin>\n",
                 argv[0] ? argv[0] : "vocab_text_to_bin");
    return 2;
  }

  const std::string in_path  = argv[1];
  const std::string out_path = argv[2];

  ORB_SLAM3::ORBVocabulary voc;

  std::cout << "Loading text vocabulary from " << in_path << " ..." << std::endl;
  const auto t0 = std::chrono::steady_clock::now();
  if (!voc.loadFromTextFile(in_path)) {
    std::fprintf(stderr, "failed to load text vocabulary from %s\n",
                 in_path.c_str());
    return 1;
  }
  const auto t1 = std::chrono::steady_clock::now();
  const double load_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  std::cout << "Loaded " << voc.size() << " words in "
            << load_ms << " ms." << std::endl;

  std::cout << "Writing binary vocabulary to " << out_path << " ..." << std::endl;
  try {
    voc.saveToBinaryFile(out_path);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "failed to write binary vocabulary: %s\n", e.what());
    return 1;
  } catch (const std::string &e) {
    std::fprintf(stderr, "failed to write binary vocabulary: %s\n", e.c_str());
    return 1;
  }
  const auto t2 = std::chrono::steady_clock::now();
  const double save_ms =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  std::cout << "Wrote binary vocabulary in " << save_ms << " ms." << std::endl;

  return 0;
}
