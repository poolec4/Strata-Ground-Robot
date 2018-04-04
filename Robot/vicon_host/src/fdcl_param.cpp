#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>      // std::setprecision
#include "Eigen/Dense"
#include "fdcl_param.h"


using namespace std; 

fdcl_param::fdcl_param(std::string file_name)
{
	file_stream.open(file_name.c_str(),ios::in);
}
void fdcl_param::open(std::string fname)
{
	file_name=fname;
	file_stream.open(file_name.c_str(),ios::in);
}
void fdcl_param::close()
{
	file_stream.close();
}

string fdcl_param::find_line(const string param_name)
{
	bool found_var = false;
	string line, group, var;
	string::size_type pos;

	file_stream.clear();
	file_stream.seekg(0, file_stream.beg);

	while(!found_var)
	{
		if(std::getline(file_stream,line) && line.length() > 0)
		{
			if(!isspace(line[0]))
			{
				group=line.erase(line.size()-1);// rmeove the last ":"
			}
			else
			{
				pos=line.find(':');
				var=line.substr(1,pos-1);
				if(group+"."+var==param_name)
				{
					found_var=true;
					line=line.substr(pos+1,line.length()-pos+1);
				}
				
			}
		}
		else
		{
			cout << "ERROR: read_param: there is no entry of "  << param_name << endl;
			break;
		}
	}

	return line;
	
}

void fdcl_param::replace_value(const string param_name, string new_value)
{
	string line, group, param, var;
	string::size_type pos;
	stringstream buf;
	ofstream new_file;

	file_stream.clear();
	file_stream.seekg(0,file_stream.beg);
	
	while(std::getline(file_stream,line) && line.length() > 0)
	{
		if(!isspace(line[0]))
		{
			// line of group
			group=line;
			group=group.erase(group.size()-1); // rmeove the last ":"			
			buf << line << endl;
		}
		else
		{
			// line of variables
			pos=line.find(':');
			var=line.substr(1,pos-1);

			if(group+"."+var==param_name)
			{
				// line of param to replace
				buf << "\t" << var << ": " << new_value << endl;
			}
			else
			{
				// line of other parms
				buf << line << endl;
				
			}
				
		}
	}
	
//	cout << buf.str();
	file_stream.close();
	
	// reopen the file in the replace mode and save the buffer to the file
	new_file.open(file_name.c_str(),ios::trunc);
	new_file << buf.str();
	new_file.close();
	
	// reopen the file in the read mode
	file_stream.open(file_name.c_str());	
	
}

void fdcl_param::read(const string param_name, double& value)
{
	value=std::stod(find_line(param_name));	
}

void fdcl_param::read(const string param_name, int& value)
{
	value=std::stoi(find_line(param_name));	
}

void fdcl_param::read(const string param_name, bool& value)
{
	int i;
	i=std::stoi(find_line(param_name));	
	if (i==0)
		value=false;
	else if (i==1)
		value=true;
	else
		cout << "ERROR: fdcl_param::read: bool should be either 0 or 1" << endl;
	
}


void fdcl_param::read(const string param_name, string& value)
{
	string line;
	string::size_type pos;
	
	line=find_line(param_name);
	
	pos=line.find('"');
	line.erase(0,pos+1);// erase the first "
	pos=line.find('"');
	line.erase(pos);	// erase the last ""
	value=line;	
}


void fdcl_param::read(const string param_name, Matrix3& M)
{
	int i,j;
	string line;
	string::size_type pos_delimiter;
	
	line=find_line(param_name);
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			if((pos_delimiter=line.find(", ")) != std::string::npos)
			{
				M(i,j)=stod(line.substr(0,pos_delimiter));
				line.erase(0,pos_delimiter+1);								
			}
			else
			{
				M(i,j)=stod(line);
			}
		}
	}	
}

void fdcl_param::read(const string param_name, Vector4& M)
{
	int i,j;
	string line;
	string::size_type pos_delimiter;
	
	line=find_line(param_name);
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			if((pos_delimiter=line.find(", ")) != std::string::npos)
			{
				M(i,j)=stod(line.substr(0,pos_delimiter));
				line.erase(0,pos_delimiter+1);								
			}
			else
			{
				M(i,j)=stod(line);
			}
		}
	}	
}

void fdcl_param::read(const string param_name, Vector3& M)
{
	int i,j;
	string line;
	string::size_type pos_delimiter;
	
	line=find_line(param_name);
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			if((pos_delimiter=line.find(", ")) != std::string::npos)
			{
				M(i,j)=stod(line.substr(0,pos_delimiter));
				line.erase(0,pos_delimiter+1);								
			}
			else
			{
				M(i,j)=stod(line);
			}
		}
	}	
}

template<typename Derived>
void fdcl_param::read(const string param_name, Eigen::MatrixBase<Derived>& M)
{
	int i,j;
	string line;
	string::size_type pos_delimiter;
	
	line=find_line(param_name);
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			if((pos_delimiter=line.find(", ")) != std::string::npos)
			{
				M(i,j)=stod(line.substr(0,pos_delimiter));
				line.erase(0,pos_delimiter+1);								
			}
			else
			{
				M(i,j)=stod(line);
			}
		}
	}	
}

void fdcl_param::save(const string param_name, double value)
{
	replace_value(param_name,std::to_string(value));
}
void fdcl_param::save(const string param_name, int value)
{
	replace_value(param_name,std::to_string(value));
}
void fdcl_param::save(const string param_name, const string value)
{
	stringstream new_value;
	
	new_value << "\"" << value << "\"";
	replace_value(param_name,new_value.str());
}
void fdcl_param::save(const string param_name, bool value)
{
	int i;
	
	if (value==false)
		i=0;
	else 
		i=1;

	replace_value(param_name,std::to_string(i));
	
}

void fdcl_param::save(const string param_name, Matrix3& M)
{
	int i,j;
	stringstream new_value;
	
	new_value << std::scientific;
	new_value << std::setprecision(10);
	
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			new_value << M(i,j) << ", ";
		}
	}
	
	replace_value(param_name,new_value.str());
}


template<typename Derived>
void fdcl_param::save(const string param_name, Eigen::MatrixBase<Derived>& M)
{
	int i,j;
	stringstream new_value;
	
	new_value << std::scientific;
	new_value << std::setprecision(10);
	
	for (i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			new_value << M(i,j) << ", ";
		}
	}
	
	replace_value(param_name,new_value.str());
}

