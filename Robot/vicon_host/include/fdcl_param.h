#ifndef _FDCL_PARAM_H
#define _FDCL_PARAM_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>      // std::setprecision
#include "Eigen/Dense"

using namespace std;
typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;

class fdcl_param
{

public:
	string file_name;
	fstream file_stream;
	
	fdcl_param() {};
	fdcl_param(std::string file_name);
	void open(std::string fname);
	void close();

	void read(const string, bool&);	
	void read(const string, double&);
	void read(const string, int&);
	void read(const string, string&);
	void read(const string, Matrix3&);
	void read(const string, Vector4&);
	void read(const string, Vector3&);
		
	template<typename Derived>
	void read(const string, Eigen::MatrixBase<Derived>& );
	
	void save(const string, bool);
	void save(const string, double);
	void save(const string, int);
	void save(const string, const string);
	void save(const string, Matrix3&);

	template<typename Derived>
	void save(const string, Eigen::MatrixBase<Derived>& );
	
	~fdcl_param()
	{
		close();
	}

private:
	string find_line(const string);
	void replace_value(const string, string);
	
};

 		
#endif
