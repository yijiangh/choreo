#include <utils/Loader.h>
#include <istream>
#include <fstream>
#include <iostream>
#include <set>


#define MYOUT std::cout
#define MYERR std::cerr

	bool Loader::uniqueFilename(const std::string &filePathName,
		const std::string &fileExtension,
		std::string &uniqueFullFileName)
	{
		int filen = 0;
		bool fexists = true;
		do{ //skip existing sequence files
			uniqueFullFileName = filePathName + std::to_string(filen) + fileExtension;
			std::ifstream iff(uniqueFullFileName);
			if (!iff.good()){
				fexists = false;
				break;
			}
			++filen;
		} while (filen < 100000);

		return !fexists;
	}