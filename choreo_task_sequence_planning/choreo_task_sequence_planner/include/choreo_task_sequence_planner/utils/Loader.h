#pragma once

#ifndef SELF_ASSEMBLY_LOADER_H
#define SELF_ASSEMBLY_LOADER_H

#include <string>

class Loader{

 private:
  Loader(){ ; }
  ~Loader(){ ; }

 public:

  static bool uniqueFilename(const std::string& filePathName,
							 const std::string& fileExtension,
							 std::string& uniqueFullFileName);
};

#endif
