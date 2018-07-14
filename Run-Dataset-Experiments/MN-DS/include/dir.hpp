#ifndef __GUARD_DIR__
#define __GUARD_DIR__

#include <dirent.h>
#include <string>
#include <vector>

namespace utils
{
  // > Segmentation fault when dir_path doesn't exists
  template <typename FILE_T>
  std::vector<std::string> getFiles(std::string dir_path, FILE_T file_type)
  {
    DIR *dir = opendir(dir_path.c_str());
    struct dirent *dir_file = readdir(dir);
    std::vector< std::string > files;
    while (dir_file != NULL)
    {
      if (dir_file->d_type == file_type && dir_file->d_name[0] != '.')
      {
        files.push_back(dir_file->d_name);
      }
      dir_file = readdir(dir);
    }
    closedir(dir);
    return files;
  }
}

#endif
