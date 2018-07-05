#include "choreo_task_sequence_planner/utils/Statistics.h"

#define OUTPUTPRECISION 12

bool Statistics::UniqueFileName()
{
	int file_n = 0;
	std::string UniqueFullFileName;
	bool fexists = true;
	do
	{
		if (file_n == 0)
		{
			UniqueFullFileName = store_path_ + "/" + filename_;
		}
		else
		{
			UniqueFullFileName = store_path_ + "/" + std::to_string(file_n) + filename_;
		}
		//UniqueFullFileName.c_str();
		std::ifstream	iff(UniqueFullFileName);
		if (!iff.good())
		{
			fexists = false;
			break;
		}
		++file_n;
	} while (file_n < 100000);

	filename_ = UniqueFullFileName;
	return !fexists;
}

void Statistics::StreamVectorOutPut()
{
	if (iteration_ != -1)
	{
		std::cout << "---------------------------------------------" << std::endl;
		std::cout << "Iteration " << iteration_ << std::endl;
	}
	for (int i = 0; i < Vx_.size(); i++)
	{
		std::cout << name_ << "[" << i << "]" << " = " << std::setprecision(OUTPUTPRECISION) << Vx_[i] << std::endl;
	}
	std::cout << "------------------------------" << std::endl;
}

void Statistics::StreamSpMatrixOutput()
{
	assert(SpMat_.cols() != 0 && SpMat_.rows() != 0);
	int c = SpMat_.cols();
	int r = SpMat_.rows();
	std::cout << "------------------------------" << std::endl;
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			std::cout << std::setprecision(OUTPUTPRECISION) << SpMat_.coeff(i, j) << "  ";
			if (c - 1 == j)
			{
				std::cout << "\n";
			}
		}
	}
	std::cout << "------------------------------" << std::endl;
}

void Statistics::StreamDenMatrixOutput()
{
	std::cout << "------------------------------" << std::endl;
	assert(denMat_.cols() != 0 && denMat_.rows() != 0);
	int c = denMat_.cols();
	int r = denMat_.rows();
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			std::cout << std::setprecision(OUTPUTPRECISION) << denMat_(i, j) << "  ";
			if (c - 1 == j)
			{
				std::cout << "\n";
			}
		}
	}
	std::cout << "------------------------------" << std::endl;
}

void Statistics::GenerateVectorFile()
{

	if (iteration_ != -1)
	{
		std::string iter = std::to_string(iteration_);
		std::string postfix = ".txt";
		filename_ = name_ + iter + postfix;
	}
	else
	{
		std::string postfix = ".txt";
		filename_ = name_ + postfix;
	}

	UniqueFileName();

	std::ofstream file;
	file.open(filename_);

	file << std::setprecision(16);
	for (int i = 0; i < Vx_.size(); i++)
	{
		file << Vx_[i] << std::endl;
	}

	file.close();
}

void Statistics::GenerateStdVecFile()
{

	if (iteration_ != -1)
	{
		std::string iter = std::to_string(iteration_);
		std::string postfix = ".txt";
		filename_ = name_ + iter + postfix;
	}
	else
	{
		std::string postfix = ".txt";
		filename_ = name_ + postfix;
	}

	UniqueFileName();

	std::ofstream file;
	file.open(filename_);

	file << std::setprecision(16);
	for (int i = 0; i < stdVec_.size(); i++)
	{
		file << stdVec_[i] << std::endl;
	}

	file.close();
}

void Statistics::GenerateMatrixFile()
{
	if (denMat_.cols() == 0 && denMat_.rows() == 0)
	{
		return;
	}

	std::string postfix = ".txt";

	filename_ = name_ + postfix;

	int r = denMat_.rows();
	int c = denMat_.cols();

	UniqueFileName();

	std::ofstream file;
	file.open(filename_);

	
	file << std::setprecision(5);
	for (int i = 0; i < r; i++)
	{
		for (int j = 0; j < c; j++)
		{
			file << denMat_(i,j) << " ";
			if (j == c-1)
			{
				file << "\n";
			}
		}
	}
	file.close();
}

void Statistics::GenerateSpFile()
{
	if (SpMat_.cols() == 0 && SpMat_.rows() == 0)
	{
		return;
	}

	std::string postfix = ".txt";

	filename_ = name_ + postfix;

	UniqueFileName();

	std::ofstream file;
	file.open(filename_);

	file << std::setprecision(5);
	
	for (int k = 0; k < SpMat_.outerSize(); ++k)
	{
		for (SparseMatrix<double>::InnerIterator it(SpMat_, k); it; ++it)
		{
			int r = it.row();
			int c = it.col();
			double	v = it.value();
			file << "(" << r << "," << c << ") " << v << "\n";
		}
	}

	file.close();
}